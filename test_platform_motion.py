"""Geometry, stream isolation, watchdog and all-axis stop tests; no real hardware."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from backend import load_config
from control_common import ControlError, adapter_lock
from motor_service import MotorService
from platform_motion import PlatformGeometry, PlatformMotionQueue, validate_platform
from test_aps_backend import FakeAPS
from UdpControl.client import MotorClient, UDPError
from udp_server import UDPServer


def setUpModule():
    global fake_lock_patch
    fake_lock_patch = patch('aps_backend.adapter_lock', side_effect=lambda adapter:
        adapter_lock(f'{adapter}:platform-test:{os.getpid()}'))
    fake_lock_patch.start()


def tearDownModule():
    fake_lock_patch.stop()


def configuration():
    return dict(enabled=True, calibration_id='test-neutral-v1', legs=[
        dict(order=order, x_mm=x, y_mm=y, mm_per_rev=5., extension_sign=sign, min_mm=-20., max_mm=20.)
        for order, x, y, sign in ((1, 200, 0, 1), (2, -100, 150, -1), (3, -100, -150, 1))])


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.geometry = PlatformGeometry(configuration())

    def test_heave_pitch_roll_and_combined_plane(self):
        for h, p, r in ((0,0,0), (5,0,0), (0,2,0), (0,0,2), (-2,1,-2)):
            _, lengths, degrees = self.geometry.pose_to_targets(dict(heave_mm=h,pitch_deg=p,roll_deg=r))
            for leg, length, angle in zip(self.geometry.legs,lengths,degrees):
                expected=h+leg['x_mm']*math.tan(math.radians(p))+leg['y_mm']*math.tan(math.radians(r))/math.cos(math.radians(p))
                self.assertAlmostEqual(length, expected)
                self.assertAlmostEqual(angle,expected*72*leg['extension_sign'])

    def test_invalid_geometry_and_nonfinite_values(self):
        for change in (dict(enabled=1), dict(calibration_id=''), dict(legs=[]),
                       dict(max_pitch_deg=30), dict(max_heave_mm=True), dict(pose_timeout_s=10)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_platform(dict(configuration(),**change))
        for field, value in (('mm_per_rev',0),('extension_sign',True),('min_mm',1),('x_mm',math.inf),('order',True)):
            config=configuration();config['legs'][0][field]=value
            with self.subTest(field=field), self.assertRaises(ValueError): validate_platform(config)
        for mutation in ('duplicate','collinear','extra'):
            config=configuration()
            if mutation=='duplicate':config['legs'][1]['order']=1
            elif mutation=='collinear':
                for leg in config['legs']:leg['y_mm']=0
            else:config['legs'][0]['typo']=1
            with self.assertRaises(ValueError):validate_platform(config)
        for pose in (dict(heave_mm=math.nan,pitch_deg=0,roll_deg=0),
                     dict(heave_mm=True,pitch_deg=0,roll_deg=0),dict(heave_mm=11,pitch_deg=0,roll_deg=0),
                     dict(heave_mm=10,pitch_deg=3,roll_deg=3),dict(heave_mm=0,pitch_deg=0)):
            with self.assertRaises(ControlError):self.geometry.pose_to_targets(pose)

    def test_rate_limit_common_progress_latest_frame_and_worker_hitch(self):
        now=[0.];q=PlatformMotionQueue(self.geometry,clock=lambda:now[0]);q.mark_ready()
        q.submit([720,-360,180]);self.assertIsNone(q.pop())
        now[0]=.021;frame=q.pop()
        ratios=[actual/goal for actual,goal in zip(frame.targets,q.planned)]
        self.assertAlmostEqual(ratios[0],ratios[1]);self.assertAlmostEqual(ratios[1],ratios[2])
        self.assertLessEqual(abs(frame.targets[0])/72,.105+1e-9)
        q.submit([-720,360,-180]);q.submit([0,0,0]);self.assertEqual(q.pending(),1)
        now[0]=100.;old=q.output;frame=q.pop()
        self.assertTrue(all(abs(a-b)/72<=.25+1e-9 for a,b in zip(frame.targets,old)))
        q.close();now[0]+=1;self.assertIsNone(q.pop())

    def test_backend_loads_and_validates_platform_without_mutating_input(self):
        config=configuration();original=deepcopy(config)
        validate_platform(config);self.assertEqual(config,original)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'backend.json';path.write_text(json.dumps(dict(platform=config)),encoding='utf-8')
            self.assertTrue(load_config(path)['platform']['enabled'])
            config['enabled']='yes';path.write_text(json.dumps(dict(platform=config)),encoding='utf-8')
            with self.assertRaises(ValueError):load_config(path)


class PlatformUDPTests(unittest.TestCase):
    def setUp(self):
        self.api=FakeAPS()
        self.service=MotorService(platform_options=configuration(),aps_options=dict(
            extension_limits={str(axis):'di2' for axis in self.api.io},
            retraction_limits={str(axis):'di1' for axis in self.api.io}))
        self.addCleanup(self.service.close)
        self.service.hardware.api_factory=lambda:self.api
        self.server=UDPServer(self.service,port=0).start()
        self.addCleanup(self.server.close)
        self.client=MotorClient(port=self.server.port);self.addCleanup(self.client.close);self.client.hello()
        self.client.request('scan');self.client.wait_for(('idle',))
        # Bind synthetic mechanical direction to the drive's measured configuration.
        for leg,device in zip(self.service.platform.legs,self.service.devices):
            leg['extension_sign']=device.extension_sign*(1 if device.positive_direction else -1)

    def enable(self):
        self.client.enable_platform([1,2,3],'test-neutral-v1',reference_confirmed=True)
        return self.client.wait_for(('enabled',))

    def wait(self,condition,timeout=2):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if condition():return
            time.sleep(.01)
        self.fail(str(self.service.snapshot()))

    def test_arm_requires_calibration_all_axes_neutral_and_no_bypass(self):
        for orders,calibration,confirm in (([1,2,3],'wrong',True),([1,2],'test-neutral-v1',True),
                                          ([1,2,3],'test-neutral-v1',False)):
            with self.assertRaises(UDPError):
                self.client.enable_platform(orders,calibration,reference_confirmed=confirm)
        with self.assertRaises(UDPError):self.client.enable([1,2,3])
        self.assertFalse(any(name=='APS_set_servo_on' for name,_ in self.api.calls))
        state=self.enable();self.assertEqual(state['control_mode'],'platform')
        with self.assertRaises(UDPError):self.client.target([1,2,3])
        with self.assertRaises(UDPError):self.client.set_profile(9999,9999)
        self.client.pose(.5,.1,-.1)
        self.wait(lambda:any(n=='APS_ptp_all' for n,_ in self.api.calls))
        self.assertEqual(len(self.api.threads),1)
        self.assertLessEqual(self.service.commands.rpm,60)

    def test_heartbeats_cannot_keep_stale_platform_pose_alive(self):
        self.enable();self.client.pose(0,0,0)
        self.wait(lambda:self.service.phase=='idle')
        self.assertIn('姿态流超时',self.service.stop_reason)
        self.assertTrue(self.service.snapshot()['result']['all_disabled'])
        with self.assertRaises(UDPError):self.client.pose(0,0,0)

    def test_pose_ordering_is_independent_of_heartbeat_and_duplicate_does_not_renew(self):
        self.enable();run=self.client.run_id
        self.assertTrue(self.service.command(run,1000))
        self.assertTrue(self.service.command(run,2,pose=dict(heave_mm=.1,pitch_deg=0,roll_deg=0)))
        previous=self.service.last_pose
        self.assertFalse(self.service.command(run,1,pose=dict(heave_mm=999,pitch_deg=0,roll_deg=0)))
        self.assertFalse(self.service.command(run,2,pose=dict(heave_mm=.2,pitch_deg=0,roll_deg=0)))
        self.assertEqual(self.service.last_pose,previous)
        self.assertEqual(self.service.platform_pose['heave_mm'],.1)

    def test_old_run_invalid_pose_cannot_stop_current_run(self):
        self.enable()
        with self.assertRaises(UDPError):self.client.request('pose',run_id='old',seq=99,pose=dict(heave_mm=999,pitch_deg=0,roll_deg=0))
        self.assertEqual(self.service.phase,'enabled')
        self.client.pose(0,0,0)
        with self.assertRaises(UDPError):self.client.pose(11,0,0)
        self.wait(lambda:self.service.phase=='idle')
        self.assertTrue(self.service.snapshot()['result']['all_disabled'])

    def test_any_endpoint_stops_all_supports_and_cannot_resume(self):
        self.enable();self.client.pose(1,0,0)
        self.wait(lambda:any(n=='APS_ptp_all' for n,_ in self.api.calls))
        self.api.digital_inputs[12]=2
        self.wait(lambda:self.service.phase=='fault')
        state=self.service.snapshot();self.assertTrue(state['result']['all_disabled'])
        self.assertTrue(all(not a['enabled'] for a in state['axes']))
        self.api.digital_inputs[12]=0
        with self.assertRaises(UDPError):self.client.pose(0,0,0)

    def test_endpoint_preflight_never_enables_any_support(self):
        self.api.digital_inputs[14]=2
        self.client.enable_platform([1,2,3],'test-neutral-v1',reference_confirmed=True)
        self.client.wait_for(('fault',))
        self.assertFalse(any(n=='APS_set_servo_on' and a[1] for n,a in self.api.calls))

    def test_late_pose_is_rejected_before_watchdog_next_tick(self):
        self.enable()
        with self.service.lock:
            self.service.last_pose=time.monotonic()-1
            self.service.last_heartbeat=time.monotonic()
            with self.assertRaises(ControlError):
                self.service.command(self.client.run_id,999,pose=dict(heave_mm=0,pitch_deg=0,roll_deg=0))
            self.assertEqual(self.service.phase,'stopping')

    def test_missing_sensors_or_wrong_direction_prevents_platform_arm(self):
        self.service.hardware.options['retraction_limits']={}
        with self.assertRaises(UDPError):self.enable()
        self.service.hardware.options['retraction_limits']={str(axis):'di1' for axis in self.api.io}
        self.service.platform.legs[0]['extension_sign']*=-1
        with self.assertRaises(UDPError):self.enable()
        self.assertFalse(any(n=='APS_set_servo_on' and a[1] for n,a in self.api.calls))

    def test_lost_enable_ack_retries_same_run_and_stale_stream_stops(self):
        original=self.server._send;dropped=[]
        def send(packet,peer):
            if packet.get('run_id') and packet.get('type')=='ack' and not dropped:
                dropped.append(True);return
            original(packet,peer)
        self.server._send=send
        self.enable();self.assertTrue(dropped)
        self.assertEqual(sum(n=='APS_set_servo_on' and a[1]==1 for n,a in self.api.calls),3)
        self.client.pose(0,0,0)
        self.wait(lambda:self.service.phase=='idle')

    def test_persistent_tracking_error_stops_all_supports(self):
        self.service.platform.config['max_tracking_error_mm']=.01
        self.service.platform.config['tracking_timeout_s']=.1
        self.enable()
        end=time.monotonic()+1
        while self.service.phase=='enabled' and time.monotonic()<end:
            self.client.pose(1,0,0);time.sleep(.025)
        self.wait(lambda:self.service.phase=='idle')
        self.assertIn('跟随误差',self.service.stop_reason)
        self.assertTrue(self.service.snapshot()['result']['all_disabled'])


if __name__=='__main__':unittest.main()
