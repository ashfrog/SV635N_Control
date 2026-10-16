"""Automatic reference acquisition with synthetic mechanics only."""
import os
from pathlib import Path
import struct
import tempfile
import time
import unittest
from unittest.mock import patch

from auto_center import CenterPlan, validate_auto_center
from auto_center_session import AutoCenterSession
from control_common import ControlError, adapter_lock
from motor_service import MotorService
from nic_backend import NICController
from platform_motion import PlatformGeometry
from test_height_control import options as platform_options
from test_nic_backend import NICMaster, NICSlave, OPTIONS
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError


def options(**changes):
    supplied=dict(enabled=True, nominal_travel_mm=1,minimum_travel_mm=.2,backoff_mm=.1,repeat_tolerance_mm=.04)
    supplied.update(changes)
    return validate_auto_center(supplied)


def setUpModule():
    global lock_patches
    lock_patches = [patch(name, side_effect=lambda adapter: adapter_lock(f'{adapter}:center-test:{os.getpid()}'))
                    for name in ('core.adapter_lock', 'nic_backend.adapter_lock')]
    for item in lock_patches:
        item.start()


def tearDownModule():
    for item in reversed(lock_patches):
        item.stop()


class PlannerTests(unittest.TestCase):
    def test_independent_mixed_endpoints_release_only_the_active_axes(self):
        position=[-.5,-.5,.5]
        sensors=lambda: ([p <= -.5+1e-8 for p in position],[p >= .5-1e-8 for p in position])
        plan=CenterPlan(options(coupled_axes=False),position,*sensors())
        phases=[]
        for _ in range(600):
            previous=list(position)
            goals=plan.targets(position,*sensors())
            phases.append(plan.phase)
            if goals is None:
                break
            if plan.phase=='release_initial_lower':
                self.assertEqual(goals[2],previous[2])
            elif plan.phase=='release_initial_upper':
                self.assertEqual(goals[:2],previous[:2])
            plan.commanded=list(goals)
            position=[max(-.5,min(.5,g)) for g in goals]
        self.assertTrue(plan.done,(plan.phase,position))
        self.assertIn('release_initial_upper',phases)
        self.assertTrue(all(abs(p)<.003 for p in position))

    def test_coupled_mixed_endpoints_and_new_limit_on_held_axis_abort(self):
        with self.assertRaisesRegex(ControlError,'分别处于上下端'):
            CenterPlan(options(),[-.5,0,.5],[True,False,False],[False,False,True])
        for lower,upper in (([True,True,False],[False]*3),
                            ([True,False,False],[False,True,False])):
            plan=CenterPlan(options(coupled_axes=False),[-.5,0,0],[True,False,False],[False]*3)
            with self.assertRaises(ControlError):
                plan.observe([-.48,0,0],lower,upper)

    def test_full_two_pass_calibration_from_middle_or_either_endpoint(self):
        for start in (0, -.5, .5):
            position = [start]*3
            sensors = lambda: ([p <= -.5+1e-8 for p in position], [p >= .5-1e-8 for p in position])
            plan = CenterPlan(options(), position, *sensors())
            for _ in range(500):
                goals = plan.targets(position, *sensors())
                if goals is None:
                    break
                position = [max(-.5,min(.5,g)) for g in goals]
            self.assertTrue(plan.done, (start,plan.phase,position))
            self.assertTrue(all(abs(p) < .003 for p in position))
            self.assertEqual(plan.lower, [-.5]*3)
            self.assertEqual(plan.upper, [.5]*3)

    def test_missing_sensor_skew_and_opposite_input_abort(self):
        for position,lower,upper,expected in (([-1.3]*3,[False]*3,[False]*3,'搜索距离'),
                ([-.2,0,0],[False]*3,[False]*3,'偏差'),
                ([0]*3,[False]*3,[True,False,False],'方向')):
            plan = CenterPlan(options(max_skew_mm=.1),[0]*3,[False]*3,[False]*3)
            with self.assertRaisesRegex(ControlError,expected):
                plan.observe(position,lower,upper)
        with self.assertRaisesRegex(ControlError,'同时'):
            CenterPlan(options(),[0]*3,[True]*3,[True]*3)

    def test_narrow_cam_latched_edge_cannot_be_accepted_after_signal_disappears(self):
        plan = CenterPlan(options(),[0]*3,[False]*3,[False]*3)
        plan.observe([-.5]*3,[True]*3,[False]*3)
        with self.assertRaisesRegex(ControlError,'消失'):
            plan.targets([-.52]*3,[False]*3,[False]*3)

    def test_stuck_switch_and_wrong_measured_stroke_abort(self):
        plan = CenterPlan(options(release_max_mm=.3),[-.5]*3,[True]*3,[False]*3)
        with self.assertRaisesRegex(ControlError,'搜索距离'):
            plan.targets([-.18]*3,[True]*3,[False]*3)
        plan = CenterPlan(options(),[0]*3,[False]*3,[False]*3)
        plan.lower,plan.first_upper = [0]*3,[.1]*3
        plan._change('latch_upper',[.08]*3)
        with self.assertRaisesRegex(ControlError,'机械行程'):
            plan.targets([.1]*3,[False]*3,[True]*3)

    def test_repeat_edge_disagreement_aborts(self):
        plan = CenterPlan(options(),[0]*3,[False]*3,[False]*3)
        plan.first_lower=[-.5]*3
        plan._change('latch_lower',[-.4]*3)
        with self.assertRaisesRegex(ControlError,'复测位置不一致'):
            plan.targets([-.56]*3,[True]*3,[False]*3)

    def test_opposite_switch_during_initial_backoff_aborts_immediately(self):
        plan=CenterPlan(options(),[0]*3,[True,False,False],[False]*3)
        with self.assertRaisesRegex(ControlError,'另一端'):
            plan.observe([.01]*3,[True,False,False],[False,True,False])

    def test_switch_hysteresis_larger_than_extra_backoff_is_measured(self):
        position=[-.5]*3
        lower,upper=[True]*3,[False]*3
        plan=CenterPlan(options(release_max_mm=.5),position,lower,upper)
        for _ in range(600):
            goals=plan.targets(position,lower,upper)
            if goals is None:
                break
            position=[max(-.5,min(.5,g)) for g in goals]
            lower=[p < -.3 if old else p <= -.5+1e-8 for p,old in zip(position,lower)]
            upper=[p > .3 if old else p >= .5-1e-8 for p,old in zip(position,upper)]
        self.assertTrue(plan.done,(plan.phase,position,lower,upper))
        self.assertTrue(all(abs(p)<.003 for p in position))

    def test_small_per_axis_following_errors_do_not_accumulate_in_shared_trajectory(self):
        plan=CenterPlan(validate_auto_center(),[0]*3,[False]*3,[False]*3)
        plan._change('latch_lower',[0]*3)
        plan.latch_search_mm=100
        position=[0]*3
        for _ in range(3000):
            goals=plan.targets(position,[False]*3,[False]*3)
            plan.commanded=goals
            position=[g+e for g,e in zip(goals,(.001,-.001,.0002))]
        self.assertLess(max(position)-min(position),.003)
        self.assertAlmostEqual(sum(position)/3,-60,delta=.003)

    def test_continuous_search_has_a_fixed_distance_budget_and_stops_at_first_edge(self):
        plan=CenterPlan(options(),[0]*3,[False]*3,[False]*3)
        self.assertEqual(plan.targets([0]*3,[False]*3,[False]*3),[-1.2]*3)
        plan.observe([-.5,-.4,-.4],[True,False,False],[False]*3)
        self.assertEqual(plan.targets([-.5,-.4,-.4],[True,False,False],[False]*3),[-.5,-1.2,-1.2])
        with self.assertRaisesRegex(ControlError,'搜索距离'):
            plan.targets([-.5,-1.2,-.4],[True,False,False],[False]*3)

    def test_coarse_edge_is_not_a_slow_repeat_reference_and_final_bounds_are_inward(self):
        plan=CenterPlan(options(),[0]*3,[False]*3,[False]*3)
        plan.coarse_lower=[-.7]*3
        plan._change('latch_lower',[-.4]*3)
        plan.targets([-.5]*3,[True]*3,[False]*3)
        self.assertEqual(plan.first_lower,[-.5]*3)
        self.assertEqual(plan.phase,'backoff_lower')
        plan._change('latch_lower',[-.4]*3)
        plan.targets([-.48]*3,[True]*3,[False]*3)
        self.assertEqual(plan.lower,[-.48]*3)
        self.assertEqual(plan.phase,'seek_upper')

    def test_invalid_config_rejects_unbounded_calibration(self):
        for changes in (dict(seek_rpm=3000),dict(step_mm=1),dict(enabled=1),
                        dict(timeout_s=float('nan')),dict(max_skew_mm=3)):
            with self.assertRaises(ValueError):
                options(**changes)


class CenterSlave(NICSlave):
    """One mm mechanism, direction derived from configured native P/N-OT."""
    def __init__(self,order):
        super().__init__(order)
        self.initial = self.position
        self.no_sensor = False
        self.min_seen = self.max_seen = 0

    def exchange(self):
        super().exchange()
        counts = 2**23
        q = -((self.position-self.initial+2**31)%2**32-2**31)*5/counts
        if not self.no_sensor:
            q = max(-.5,min(.5,q))
            self.position = (self.initial-round(q*counts/5)+2**31)%2**32-2**31
        self.min_seen,self.max_seen = min(self.min_seen,q),max(self.max_seen,q)
        self.raw = 0 if self.no_sensor else ((2 | 1<<16) if q <= -.5+1e-6 else
                                            (1 | 1<<17) if q >= .5-1e-6 else 0)
        self.input = struct.pack('<HHi',self.error_code,self.sw,self.position)+bytes(16)+struct.pack('<I',self.raw)


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.master = NICMaster()
        self.master.slaves = [CenterSlave(n) for n in (1,2,3)]
        self.directory = tempfile.TemporaryDirectory()
        self.controller = NICController(log_dir=self.directory.name,
            options=dict(OPTIONS,position_margin_degrees=1), master_factory=lambda:self.master)
        self.reference = Path(self.directory.name)/'reference.json'
        self.service = MotorService(hardware_controller=self.controller,reference_path=self.reference,
                                    auto_center_options=options())
        self.server = UDPServer(self.service,port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()
        self.directory.cleanup()

    def wait(self,predicate,timeout=12):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            state=self.client.status()
            if predicate(state):
                return state
            if state['phase']=='fault':
                self.fail(str(state))
            time.sleep(.01)
        self.fail(str(self.client.status()))

    def test_center_persists_new_bounds_and_returns_to_original_profile_without_old_targets(self):
        events=[]
        run=self.controller.run_auto_center
        def record(devices,commands,settings,stop,callback):
            def publish(event):
                if event.get('auto_center_phase'):
                    events.append((event['auto_center_phase'],
                        self.master.slaves[0].params[0x6081,0]*60/2**23))
                callback(event)
            return run(devices,commands,settings,stop,publish)
        self.controller.run_auto_center=record
        self.client.enable([1,2,3],rpm=3000,acceleration_rpm_s=3000)
        run_id=self.client.run_id
        self.wait(lambda s:s['control_mode']=='homing' and s['phase']=='enabled')
        with self.assertRaises(UDPError):
            self.client.target([500]*3)
        state=self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        self.assertEqual(state['run_id'],run_id)
        self.assertEqual(state['profile']['rpm'],3000)
        self.assertEqual(state['targets_deg'],[0,0,0])
        self.assertTrue(self.reference.exists())
        phase_profiles=dict(events)
        self.assertEqual(phase_profiles['seek_lower'],60)
        self.assertEqual(phase_profiles['seek_upper'],60)
        self.assertAlmostEqual(phase_profiles['latch_lower'],6,places=5)
        self.assertAlmostEqual(phase_profiles['latch_upper'],6,places=5)
        self.assertEqual(phase_profiles['center'],600)
        self.assertTrue(all(s.min_seen <= -.5 and s.max_seen >= .5 for s in self.master.slaves))
        self.assertTrue(all(abs((s.position-s.initial+2**31)%2**32-2**31)<5000 for s in self.master.slaves))
        for d in self.service.devices:
            entry=self.controller.limit_memory.entry(self.controller.adapter,d)
            self.assertTrue(entry['automatic'])
            self.assertAlmostEqual(entry['manual']['lower_degrees'],-28.8,places=2)
            self.assertAlmostEqual(entry['manual']['upper_degrees'],28.8,places=2)
        self.client.disable()
        self.client.wait_for(('idle',))
        before=[len(s.targets) for s in self.master.slaves]
        self.client.enable([1,2,3],rpm=60)
        self.client.wait_for(('enabled',))
        self.assertEqual(before,[len(s.targets) for s in self.master.slaves])

    def test_cancel_during_homing_never_persists_or_reenables(self):
        self.client.enable([1,2,3],rpm=3000)
        self.wait(lambda s:s['control_mode']=='homing' and s['phase']=='enabled')
        self.client.disable()
        state=self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(state['auto_center']['ready'])
        self.assertFalse(self.reference.exists())
        last=[len(s.targets) for s in self.master.slaves]
        time.sleep(.2)
        self.assertEqual(last,[len(s.targets) for s in self.master.slaves])

    def test_independent_axes_can_start_on_opposite_endpoints(self):
        self.service.auto_center=options(coupled_axes=False)
        for slave,mm in zip(self.master.slaves,(-.5,-.5,.5)):
            slave.position=(slave.initial-round(mm*2**23/5)+2**31)%2**32-2**31
            slave.exchange()
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.client.enable([1,2,3],rpm=60)
        state=self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        self.assertTrue(state['enabled'])
        self.assertTrue(all(abs(s.position-s.initial)<5000 for s in self.master.slaves))

    def test_profile_change_replans_first_latch_step_after_halt_coast(self):
        settle=AutoCenterSession._settle
        drifted=[]
        def coast(session):
            settle(session)
            if session.plan.phase in ('seek_lower','latch_lower') and session.plan.phase not in drifted:
                for axis in session.selected():
                    slave=axis['slave']
                    slave.position=(slave.position-round(.01*2**23/5)+2**31)%2**32-2**31
                drifted.append(session.plan.phase)
                session.tick()
        with patch.object(AutoCenterSession,'_settle',coast):
            self.client.enable([1,2,3],rpm=60)
            state=self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        self.assertEqual(drifted,['seek_lower','latch_lower'])
        self.assertTrue(state['enabled'])

    def test_platform_first_enable_homes_without_an_old_reference_then_accepts_heights(self):
        geometry=platform_options()
        for leg,device in zip(geometry['legs'],self.service.devices):
            leg.update(min_mm=-.5,max_mm=.5,
                extension_sign=device.extension_sign*(1 if device.positive_direction else -1))
        self.service.platform=PlatformGeometry(geometry)
        self.assertFalse(self.reference.exists())
        reply=self.client.request('enable',mode='platform',orders=[1,2,3],
            calibration_id=geometry['calibration_id'])
        self.client.run_id=reply['run_id']
        state=self.wait(lambda s:s['phase']=='enabled' and s['control_mode']=='platform')
        self.assertTrue(state['auto_center']['ready'])
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            self.client.heights([.05]*3)
            state=self.client.status()
            actual=state['platform']['actual_lengths_mm']
            if all(v is not None and abs(v-.05)<.005 for v in actual):
                break
            time.sleep(.02)
        else:
            self.fail(str(state))
        self.assertTrue(state['enabled'])

    def test_missing_sensors_aborts_bounded_seek_without_recording_limits(self):
        for slave in self.master.slaves:
            slave.no_sensor=True
        self.client.enable([1,2,3],rpm=3000)
        state=self.client.wait_for(('fault',),timeout=12)
        self.assertIn('搜索距离',state['result']['error'])
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(self.reference.exists())
        self.assertTrue(all(s.min_seen > -1.23 for s in self.master.slaves))

    def test_real_drive_fault_stops_without_automatic_reset(self):
        self.client.enable([1,2,3],rpm=3000)
        self.wait(lambda s:s['control_mode']=='homing' and s['phase']=='enabled')
        slave=self.master.slaves[1]
        slave.error_code,slave.fault_latched=0x7121,True
        state=self.client.wait_for(('fault',))
        self.assertIn('0x7121',state['result']['error'])
        self.assertTrue(state['result']['all_disabled'])
        self.assertEqual(slave.reset_edges,0)
        self.assertFalse(self.reference.exists())

    def test_stalled_axis_cannot_complete_step_or_generate_bounds(self):
        self.master.slaves[1].freeze=True
        self.client.enable([1,2,3],rpm=3000)
        state=self.client.wait_for(('fault',))
        self.assertIn('单步',state['result']['error'])
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(self.reference.exists())

    def test_real_drift_after_disabling_rejects_save_and_automatic_reenable(self):
        run=self.controller.run_auto_center
        def drift(*args,**kwargs):
            result=run(*args,**kwargs)
            slave=self.master.slaves[0]
            slave.position=(slave.position+round(2**23*1.1/5)+2**31)%2**32-2**31
            return result
        self.controller.run_auto_center=drift
        self.client.enable([1,2,3],rpm=60)
        state=self.client.wait_for(('fault',),timeout=12)
        self.assertIn('关闭使能后中位位置改变',state['result']['error'])
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(state['auto_center']['ready'])
        self.assertFalse(self.reference.exists())

    def test_acceptable_centre_offset_reanchors_zero_without_expanding_native_limits(self):
        run=self.controller.run_auto_center
        original=[]
        def offset(*args,**kwargs):
            result=run(*args,**kwargs)
            for row in result['auto_center']['axes']:
                original.append(tuple((row['origin']+round(v*2**23/360)+2**31)%2**32-2**31
                                      for v in (row['lower_degrees'],row['upper_degrees'])))
            slave=self.master.slaves[0]
            slave.position=(slave.position+round(2**23*.1/5)+2**31)%2**32-2**31
            return result
        self.controller.run_auto_center=offset
        self.client.enable([1,2,3],rpm=60)
        state=self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        self.assertAlmostEqual(abs(state['auto_center']['result']['axes'][0]['center_error_mm']),.1,places=3)
        for device,before in zip(self.service.devices,original):
            row=self.controller.limit_memory.entry(self.service.adapter,device)['manual']
            after=tuple((row['origin']+round(v*2**23/360)+2**31)%2**32-2**31
                        for v in (row['lower_degrees'],row['upper_degrees']))
            self.assertEqual(before,after)

    def test_lost_heartbeat_stops_calibration_without_automatic_resume(self):
        self.client.enable([1,2,3],rpm=3000)
        self.wait(lambda s:s['control_mode']=='homing' and s['phase']=='enabled')
        self.client.run_id=None
        state=self.client.wait_for(('idle',))
        self.assertIn('心跳',state['stop_reason'])
        self.assertFalse(state['auto_center']['ready'])
        self.assertFalse(self.reference.exists())

    def test_new_process_remeasures_in_new_encoder_coordinates(self):
        self.client.enable([1,2,3],rpm=60)
        self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        self.client.disable()
        self.client.wait_for(('idle',))
        self.client.close()
        self.server.close()
        self.service.close()
        old_origins=[self.controller.limit_memory.entry(self.controller.adapter,d)['manual']['origin']
                     for d in self.service.devices]
        for slave in self.master.slaves:
            slave.initial=(slave.initial+10000000+2**31)%2**32-2**31
            slave.position=slave.initial
        self.controller=NICController(log_dir=self.directory.name,
            options=dict(OPTIONS,position_margin_degrees=1),master_factory=lambda:self.master)
        self.service=MotorService(hardware_controller=self.controller,reference_path=self.reference,
                                  auto_center_options=options())
        self.server=UDPServer(self.service,port=0).start()
        self.client=MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        state=self.client.wait_for(('idle',))
        self.assertFalse(state['auto_center']['ready'])
        self.client.enable([1,2,3],rpm=60)
        self.wait(lambda s:s['auto_center']['ready'] and s['phase']=='enabled' and s['control_mode']=='motor')
        for old,device in zip(old_origins,self.service.devices):
            origin=self.controller.limit_memory.entry(self.controller.adapter,device)['manual']['origin']
            self.assertAlmostEqual((origin-old+2**31)%2**32-2**31,10000000,delta=5000)

    def test_clear_button_only_invalidates_records_until_next_explicit_enable(self):
        self.service.centered_this_boot=True
        devices=self.service.devices
        self.controller.limit_memory.configure_manual(self.controller.adapter,
            [dict(order=d.order,enabled=True,origin=d.position,lower_degrees=-30,upper_degrees=30)
             for d in devices],devices,automatic=True)
        self.client.reset_limits([1,2,3])
        state=self.client.wait_for(('idle',))
        self.assertFalse(state['auto_center']['ready'])
        self.assertFalse(any(s.words or s.targets for s in self.master.slaves))
        self.assertFalse(self.reference.exists())

    def test_reset_removes_generated_bounds_only_and_performs_no_motion(self):
        devices=self.service.devices
        memory=self.controller.limit_memory
        settings=[dict(order=d.order,enabled=True,origin=d.position,lower_degrees=-30,upper_degrees=30)
                  for d in devices]
        memory.configure_manual(self.controller.adapter,settings,devices,automatic=True)
        memory.configure_manual(self.controller.adapter,[settings[1]],devices)
        before=[len(s.words) for s in self.master.slaves]
        memory.reset_records(self.controller.adapter,devices)
        self.assertEqual(before,[len(s.words) for s in self.master.slaves])
        self.assertNotIn('manual',memory.entry(self.controller.adapter,devices[0]))
        self.assertIn('manual',memory.entry(self.controller.adapter,devices[1]))


if __name__=='__main__':
    unittest.main()
