"""Motherboard NIC backend regression; all masters and drives are emulated."""
import json
import logging
import os
from pathlib import Path
import struct
import threading
import tempfile
import time
import unittest
from unittest.mock import patch

import backend
from control_common import DEFAULT_ADAPTER, ControlError, MotionQueue, Move, adapter_lock
from motor_service import MotorService
from nic_backend import NICController, _NICSession, input_state, validate_options
from nic_safety import LimitMemory, envelope_bounds, manual_bounds, stopping_distance
from platform_motion import PlatformGeometry
from test_height_control import options as platform_options
from test_platform_control import FakeMaster, FakeSlave
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError


OPTIONS = dict(extension_limits={str(n): 'di2' for n in (1, 2, 3)},
               retraction_limits={str(n): 'di1' for n in (1, 2, 3)})


def calibrated_fake_limits(controller, devices):
    # The fake mechanism has a known +/-10 revolution travel. Production must
    # receive measured boundaries through the operator's soft-limit editor.
    controller.limit_memory.configure_manual(controller.adapter,
        [dict(order=d.order,enabled=True,origin=d.position,lower_degrees=-3600,upper_degrees=3600)
         for d in devices], devices)


def setUpModule():
    global lock_patches
    lock_patches = [patch(name, side_effect=lambda adapter: adapter_lock(f'{adapter}:nic-test:{os.getpid()}'))
                    for name in ('core.adapter_lock', 'nic_backend.adapter_lock')]
    for item in lock_patches:
        item.start()


def tearDownModule():
    for item in reversed(lock_patches):
        item.stop()


class NICSlave(FakeSlave):
    def __init__(self, order):
        super().__init__(order)
        self.raw = 0
        self.freeze = False
        self.error_code = 0
        self.fault_latched = False
        self.resettable = True
        self.reset_edges = 0
        self.internal_limit = False
        self.accepted_target = self.position
        self.params.update({(0x2003, 3): 14, (0x2003, 5): 15, (0x605D, 0): 1, (0x2002, 8): 1})
        self.original = dict(self.params)

    def sdo_read(self, index, sub):
        if index == 0x603F:
            return self.error_code.to_bytes(2, 'little')
        if index == 0x6041 and self.fault_latched:
            return (0x208).to_bytes(2, 'little')
        return self.raw.to_bytes(4, 'little') if index == 0x60FD else super().sdo_read(index, sub)

    def exchange(self):
        word, target = struct.unpack_from('<Hi', self.output)
        was_halted = bool(self.previous & 0x100)
        if word & 0x10 and not word & 0x100 and not self.previous & 0x10:
            self.accepted_target = target
        if word & 0x80 and not self.previous & 0x80:
            self.reset_edges += 1
            if self.resettable:
                self.fault_latched = False
                self.error_code = 0
        previous = self.position
        previous_targets = len(self.targets)
        super().exchange()
        if word & 0x100:
            # Real SV635N traces: Halt retains enable but does not acknowledge
            # a new PP setpoint (0x13F never raises statusword bit12).
            self.sw &= ~0x1000
            del self.targets[previous_targets:]
            self.position = previous
        elif was_halted and word & 0xF == 15:
            self.position = self.accepted_target
        if self.fault_latched:
            self.sw = 0x208
        elif self.error_code:
            self.sw |= 0x80
        if self.internal_limit:
            self.sw |= 0x800
        if self.freeze:
            self.position = previous
        self.input = (struct.pack('<HHi', self.error_code, self.sw, self.position) + bytes(16)
                      + struct.pack('<I', self.raw))


class NICMaster(FakeMaster):
    def __init__(self):
        super().__init__()
        self.slaves = [NICSlave(n) for n in (1, 2, 3)]
        self.expected_wkc = 9
        self.opened_adapters = []

    def open(self, adapter):
        self.closed = False
        self.opened_adapters.append(adapter)


class ConfigTests(unittest.TestCase):
    def test_default_nic_and_explicit_aps_configuration(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.json'
            config = backend.load_config(path)
            self.assertEqual((config['backend'], config['adapter']), ('ethercat', DEFAULT_ADAPTER))
            path.write_text(json.dumps(dict(backend='aps', aps=dict(board_id=2))), encoding='utf-8')
            self.assertEqual(backend.load_config(path)['adapter'], 'PCIe-8332:2')
            for supplied in (dict(backend='other'), dict(adapter='PCIe-8332:0'), dict(adapter=1),
                             dict(nic=dict(extension_limits={'0': 'di2'}))):
                path.write_text(json.dumps(supplied), encoding='utf-8')
                with self.assertRaises(ValueError):
                    backend.load_config(path)
        with self.assertRaises(ValueError):
            validate_options(dict(extension_limits={'1':'di1'}, retraction_limits={'1':'di1'}))

    def test_service_defaults_to_nic_without_loading_aps(self):
        with patch('aps_backend.APSLibrary', side_effect=AssertionError('APS must not load')):
            service = MotorService()
            try:
                self.assertIsInstance(service.hardware, NICController)
                self.assertEqual(service.adapter, DEFAULT_ADAPTER)
            finally:
                service.close()

    def test_backend_startup_scans_motherboard_without_enabling(self):
        master = NICMaster()
        config = backend.load_config(Path('missing-nic-startup-test.json'))
        config.update(port=0, nic=OPTIONS)
        observed = []
        def tray(service, server, *_):
            with MotorClient(port=server.port) as client:
                state = client.wait_for(('idle', 'fault'))
                observed.append(state)
            self.assertEqual(state['phase'], 'idle', state)
            self.assertEqual(state['hardware_backend'], 'ethercat')
            self.assertFalse(state['enabled'])
        with patch('backend.load_config', return_value=config), \
             patch('backend.sys.argv', ['backend.py']), patch('backend.signal.signal'), \
             patch('backend.RotatingFileHandler', return_value=logging.NullHandler()), \
             patch('backend.logging.basicConfig'), patch('backend.run_tray', side_effect=tray), \
             patch('ctypes.windll.user32.MessageBoxW'), \
             patch('core.pysoem.Master', return_value=master), \
             patch('aps_backend.APSLibrary', side_effect=AssertionError('APS must not load')):
            self.assertEqual(backend.main(), 0)
        self.assertEqual(len(observed[0]['devices']), 3)
        self.assertEqual(set(master.opened_adapters), {DEFAULT_ADAPTER})
        self.assertFalse(any(s.words or s.targets for s in master.slaves))


class NativeSafetyTests(unittest.TestCase):
    def setUp(self):
        self.master = NICMaster()
        self.controller = NICController(options=OPTIONS, master_factory=lambda: self.master)
        self.devices = self.controller.scan()
        calibrated_fake_limits(self.controller, self.devices)

    def test_network_counters_are_preserved_after_normal_stop_before_the_next_scan_resets_them(self):
        stopped=threading.Event()
        reads=[]
        syncs=[]
        for slave in self.master.slaves:
            original_sync=slave.dc_sync
            def dc_sync(*args,original_sync=original_sync):
                syncs.append(args)
                return original_sync(*args)
            slave.dc_sync=dc_sync
            original=slave._fprd
            count=[0]
            def fprd(address,size,original=original,count=count):
                if address==0x0300:
                    self.assertFalse(any(s.sw&4 for s in self.master.slaves))
                    reads.append(address)
                    count[0]+=1
                    if count[0]==1:
                        return bytes(20)
                    raw=bytearray(20);raw[2]=3;raw[3]=2;raw[8]=3
                    return bytes(raw)
                return original(address,size)
            slave._fprd=fprd
        def event(item):
            if item['kind']=='continuous_ready':
                stopped.set()
        result=self.controller.run_continuous(self.devices,MotionQueue([1,2,3],60,120),stopped,event)
        self.assertTrue(result['all_disabled'])
        self.assertEqual(len(reads),6)
        self.assertEqual(result['bus_cycle_ms'],4)
        self.assertEqual(syncs.count((True,4_000_000)),3)
        self.assertEqual(syncs.count((False,4_000_000)),3)
        self.assertEqual([a['id'] for a in result['network_diagnostics']], [d.alias for d in self.devices])
        ports=result['network_diagnostics'][0]['ports']
        self.assertEqual(ports[1]['physical_rx_errors'],2)
        self.assertEqual(ports[1]['invalid_frames'],3)
        self.assertEqual(ports[0]['forwarded_rx_errors'],3)
        self.assertEqual(ports[1]['added_physical_rx_errors'],2)
        self.assertEqual(ports[1]['added_invalid_frames'],3)
        self.assertEqual(ports[0]['added_forwarded_rx_errors'],3)

    def test_reset_or_saturated_counters_do_not_report_a_false_zero_increase(self):
        session=_NICSession(self.controller,self.devices,MotionQueue([1],60,120),threading.Event(),lambda e:None)
        device,slave=self.devices[0],self.master.slaves[0]
        session.axes=[dict(device=device,slave=slave,report={})]
        before=bytearray(20);before[3]=10;before[8]=255
        after=bytearray(20);after[3]=2;after[8]=255
        session.network_baseline[device.order]=bytes(before)
        original=slave._fprd
        slave._fprd=lambda address,size: bytes(after) if address==0x0300 else original(address,size)
        session.network_diagnostics()
        ports=session.report['network_diagnostics'][0]['ports']
        self.assertIsNone(ports[1]['added_physical_rx_errors'])
        self.assertTrue(ports[1]['counter_reset_detected'])
        self.assertIsNone(ports[0]['added_forwarded_rx_errors'])
        self.assertTrue(ports[0]['counter_saturated'])

    def test_narrow_cam_clear_on_outer_side_keeps_latch_and_remembers_after_restart(self):
        device = self.devices[0]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'limits.json'
            memory = LimitMemory(path)
            edge = 2**31-10
            memory.protect('nic', device, edge, input_state(device, 2))
            state = memory.protect('nic', device, -2**31+20, input_state(device, 0))
            self.assertTrue(state['positive_limit'])
            self.assertTrue(state['positive_latched'])
            memory.save()
            memory = LimitMemory(path)
            self.assertTrue(memory.protect('nic', device, -2**31+50,
                                           input_state(device, 0))['positive_limit'])
            inside = edge-round(device.units_per_rev/360)
            self.assertFalse(memory.protect('nic', device, inside, input_state(device, 0))['positive_limit'])
            self.assertEqual(memory.entry('nic', device)['positive'], edge)

    def test_braking_distance_includes_reaction_delay_and_margin(self):
        self.assertAlmostEqual(stopping_distance(250, 250, .05, .5), 138)
        self.assertEqual(stopping_distance(-250, 250, .05, .5), 138)
        self.assertGreater(stopping_distance(250, 120, .05, .5), 260)
        self.assertEqual(stopping_distance(100, 250, .05, .5,
                                           acceleration=1000, velocity_limit=125), 38)

    def test_manual_limits_persist_with_native_wrap_and_reversed_axis(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'limits.json'
            memory = LimitMemory(path)
            settings = [dict(order=d.order, enabled=True, origin=d.position,
                             lower_degrees=-180, upper_degrees=360) for d in self.devices]
            memory.configure_manual('nic', settings, self.devices)
            restored = LimitMemory(path)
            for d in self.devices:
                origin, low, high = manual_bounds(restored.entry('nic', d)['manual'], d)
                self.assertEqual(origin, d.position)
                expected = (-d.units_per_rev//2, d.units_per_rev) if d.positive_direction else (-d.units_per_rev, d.units_per_rev//2)
                self.assertEqual((low, high), expected)

    def test_reset_only_selected_learned_edges_and_preserves_manual_and_backup(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'limits.json'
            memory = LimitMemory(path)
            d, other = self.devices[:2]
            manual = dict(order=d.order, enabled=True, origin=d.position, lower_degrees=-90, upper_degrees=90)
            memory.configure_manual('nic', [manual], self.devices)
            memory.observe('nic', d, d.position, True, False)
            memory.observe('nic', other, other.position, False, True)
            memory.save()
            original = path.read_bytes()
            memory.reset_records('nic', [d])
            restored = LimitMemory(path)
            self.assertNotIn('positive', restored.entry('nic', d))
            self.assertEqual(restored.entry('nic', d)['manual']['upper_degrees'], 90)
            self.assertEqual(restored.entry('nic', other)['negative'], other.position)
            self.assertEqual(path.with_name(path.name+'.before-edit.bak').read_bytes(), original)
            # A reset cannot bypass an input that is physically still asserted.
            state = restored.protect('nic', d, d.position, input_state(d, 2))
            self.assertTrue(state['positive_limit'])
            self.assertEqual(restored.entry('nic', d)['positive'], d.position)

    def test_manual_limit_validation_is_atomic_and_replaces_old_sensor_records(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'limits.json'
            memory = LimitMemory(path)
            d = self.devices[0]
            memory.observe('nic', d, d.position, True, False)
            memory.save()
            before = path.read_bytes()
            valid = dict(order=1, enabled=True, origin=d.position, lower_degrees=-90, upper_degrees=90)
            for extra in (dict(order=2, enabled=True, origin=0, lower_degrees=0, upper_degrees=0),
                          dict(order=2, enabled=True, origin=0, lower_degrees=-1, upper_degrees=float('inf')),
                          dict(order=2, enabled=True, origin=0, lower_degrees=-1, upper_degrees=1e10),
                          dict(order=1, enabled=False)):
                with self.assertRaises(ControlError):
                    memory.configure_manual('nic', [valid, extra], self.devices)
                self.assertEqual(path.read_bytes(), before)
                self.assertIn('positive', memory.entry('nic', d))
            memory.configure_manual('nic', [valid], self.devices)
            self.assertNotIn('positive', memory.entry('nic', d))
            self.assertEqual(memory.entry('nic', d)['manual']['origin'], d.position)

    def test_early_brake_before_recorded_edge_with_clear_sensor(self):
        controller = self.controller
        controller.options['position_margin_degrees'] = 1
        device, slave = self.devices[0], self.master.slaves[0]
        counts = device.units_per_rev
        native = lambda degrees: (device.position+round(counts*degrees/360)+2**31)%2**32-2**31
        controller.limit_memory.observe(controller.adapter, device, native(100), True, False)
        commands = MotionQueue([1], 5, 120)
        commands.mark_ready()
        session = _NICSession(controller, self.devices, commands, threading.Event(), lambda e: None)
        session.armed = True
        axis = dict(device=device, slave=slave, counts=counts, request=Move(1), origin=device.position,
                    direction_factor=1,
                    unwrapped_counts=round(counts*96/360), previous_position=native(96),
                    last=(0, 0x27, native(97)), cw=0x2F, tolerance=round(counts*.2/360),
                    report=dict(profile_acceleration_counts_s2=counts*120/60,
                                profile_velocity_counts_s=counts*5/60),
                    command_goal=round(counts*98/360), velocity_samples=[(.98, round(counts*96/360))])
        from collections import deque
        axis['velocity_samples'] = deque(axis['velocity_samples'])
        slave.input = struct.pack('<HHi', 0, 0x27, native(97))+bytes(20)
        session.axes = [axis]
        with patch('nic_backend.time.monotonic', return_value=1.):
            session.update_feedback(axis)
        self.assertFalse(session.limits[1]['positive_limit'])
        self.assertEqual(session.limit_pending, {1})
        commands.submit([200])
        self.assertTrue(session.interrupt_motion())
        self.assertEqual(commands.planned, (200,))
        request = session.prepare_request(commands.submit([210]))
        self.assertTrue(session.interrupt_motion(request))
        self.assertTrue(session.limit_halted)
        self.assertEqual(axis['cw'], 0x10F)
        request = session.prepare_request(commands.submit([60]))
        self.assertFalse(session.interrupt_motion(request))

    def test_network_state_poll_does_not_block_cyclic_pdo_and_fault_diagnostics_survive(self):
        commands = MotionQueue([1], 5, 120)
        session = _NICSession(self.controller, self.devices, commands, threading.Event(), lambda e: None)
        session.axes = [dict(device=d, slave=s) for d, s in zip(self.devices, self.master.slaves)]
        with patch.object(self.master, 'read_state', side_effect=AssertionError('blocking poll')) as poll:
            session.poll_network_state(1.)
            poll.assert_not_called()
        original = self.master.slaves[0]._fprd
        self.master.slaves[0]._fprd = lambda address, size: bytes(range(20)) if address == 0x0300 else original(address, size)
        session.network_diagnostics()
        self.assertEqual(session.report['network_diagnostics'][0]['esc_error_registers_0300_0313'], bytes(range(20)).hex())

    def test_new_outward_target_cancels_reverse_queued_during_coasting(self):
        from collections import deque
        device = self.devices[0]
        commands = MotionQueue([1],60,120)
        commands.mark_ready()
        session = _NICSession(self.controller,self.devices,commands,threading.Event(),lambda event:None)
        axis = dict(device=device,request=Move(1),counts=device.units_per_rev,origin=device.position,
                    direction_factor=1,unwrapped_counts=0,last=(0,0x427,device.position),
                    hold_direction=1,halt_samples=deque([(0,1000),(.05,0)]))
        session.axes = [axis]
        session.limits[1] = input_state(device,2)
        session.limit_halted = True
        commands.submit([-10])
        self.assertIsNone(session.next_request())
        commands.submit([20])
        self.assertTrue(session.interrupt_motion(session.next_request()))
        self.assertIsNone(session.deferred_reverse)
        axis['halt_samples'] = deque([(0,0),(.05,0)])
        self.assertIsNone(session.next_request())
        commands.submit([-20])
        self.assertEqual(session.next_request().targets,(-20,))

    def test_calibrated_bounds_verify_identity_and_native_direction(self):
        reference = dict(axes=[dict(order=d.order, position=d.position, identity=list(d.identity))
                               for d in self.devices])
        legs = [dict(order=d.order, mm_per_rev=5, min_mm=-50, max_mm=50,
                     extension_sign=d.extension_sign*(1 if d.positive_direction else -1)) for d in self.devices]
        for d in self.devices:
            centre, low, high = envelope_bounds(dict(reference=reference, legs=legs), d)
            self.assertEqual((centre, low, high), (d.position, -10*d.units_per_rev, 10*d.units_per_rev))
        reference['axes'][0]['identity'] = []
        with self.assertRaises(ControlError):
            envelope_bounds(dict(reference=reference, legs=legs), self.devices[0])


class NICUDPTests(unittest.TestCase):
    def setUp(self):
        self.master = NICMaster()
        self.directory = tempfile.TemporaryDirectory()
        self.reference = Path(self.directory.name)/'reference.json'
        controller = NICController(options=OPTIONS, master_factory=lambda: self.master)
        self.service = MotorService(hardware_controller=controller, reference_path=self.reference)
        self.server = UDPServer(self.service, port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))
        calibrated_fake_limits(controller, self.service.devices)

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()
        self.directory.cleanup()

    def wait(self, predicate):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        self.fail(str(self.service.snapshot()))

    def enable(self, orders=None):
        self.client.enable(orders or [1, 2, 3], rpm=5)
        self.client.wait_for(('enabled',))

    def assert_blocked_target(self, targets):
        counts = [len(s.targets) for s in self.master.slaves]
        self.client.target(targets)
        time.sleep(.08)
        self.assertEqual([len(s.targets) for s in self.master.slaves], counts)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertEqual(self.service.snapshot()['targets_deg'], targets)

    def test_motor_speed_allows_3000_and_rejects_above_limit_without_disabling(self):
        with self.assertRaises(UDPError):
            self.client.enable([1], rpm=3000.1)
        self.assertFalse(any(s.words for s in self.master.slaves))
        self.client.enable([1], rpm=3000, acceleration_rpm_s=3000)
        self.client.wait_for(('enabled',))
        self.assertEqual(self.service.commands.profile()[1], 3000)
        self.client.set_profile(1500, 3000)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 1500)
        self.client.set_profile(3000, 3000)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 3000)
        with self.assertRaises(UDPError):
            self.client.set_profile(3000.1, 3000)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertEqual(self.service.commands.profile()[1], 3000)
        self.client.set_profile(5, 120)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 5)

    def test_reset_stale_coincident_limits_from_fault_without_motion(self):
        controller = self.service.hardware
        controller.limit_memory.path = Path(self.directory.name)/'limits.json'
        device = self.service.devices[1]
        controller.limit_memory.observe(controller.adapter, device, device.position+12, True, False)
        controller.limit_memory.observe(controller.adapter, device, device.position-12, False, True)
        controller.limit_memory.save()
        self.client.enable([2])
        self.client.wait_for(('fault',))
        self.assertIn('限位位置记录冲突', self.service.message)
        self.assertFalse(any(word & 0xF == 15 for s in self.master.slaves for word in s.words))
        self.client.reset_limits([2])
        self.client.wait_for(('idle',))
        self.assertNotIn('positive', controller.limit_memory.entry(controller.adapter, device))
        self.assertNotIn('negative', controller.limit_memory.entry(controller.adapter, device))
        self.assertFalse(any(s.targets for s in self.master.slaves))
        self.assertFalse(self.service.snapshot()['enabled'])
        self.enable([2])
        self.client.target([10])
        self.wait(lambda: self.service.snapshot()['axes'][1]['travel_degrees'] > 9.9)

    def test_manual_limits_bound_motor_and_keep_slider_and_origin_across_enable(self):
        controller = self.service.hardware
        device = self.service.devices[1]  # Opposite shaft direction.
        settings = [dict(order=2, enabled=True, origin=device.position, lower_degrees=-45, upper_degrees=90)]
        self.client.configure_soft_limits(settings)
        self.client.wait_for(('idle',))
        self.assertTrue(self.service.snapshot()['limit_settings']['axes'][1]['enabled'])
        self.assertFalse(any(s.words or s.targets for s in self.master.slaves))
        self.enable([2])
        for action in (lambda:self.client.reset_limits([2]), lambda:self.client.configure_soft_limits(settings)):
            with self.assertRaises(UDPError):
                action()
        self.assertTrue(self.service.snapshot()['enabled'])
        self.client.target([300])
        self.wait(lambda: self.service.snapshot()['axes'][1]['travel_degrees'] > 89.9)
        self.assertLess(self.service.snapshot()['axes'][1]['travel_degrees'], 90.1)
        self.assertEqual(self.service.snapshot()['targets_deg'], [300])
        self.client.disable()
        self.client.wait_for(('idle',))
        self.enable([2])
        self.client.target([-300])
        self.wait(lambda: self.service.snapshot()['axes'][1]['travel_degrees'] < -134.9)
        self.assertGreater(self.service.snapshot()['axes'][1]['travel_degrees'], -135.1)
        self.assertEqual(controller.limit_memory.entry(controller.adapter, device)['manual']['origin'], device.position)

    def test_runtime_new_coincident_records_hold_enable_without_cleanup(self):
        self.enable([1])
        slave = self.master.slaves[0]
        words = len(slave.words)
        slave.raw = 2
        self.wait(lambda: 'positive' in self.service.hardware.limit_memory.entry(self.service.adapter, self.service.devices[0]))
        slave.raw = 1
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        self.assertTrue(self.service.snapshot()['enabled'])
        slave.raw = 0
        self.assert_blocked_target([10])
        self.assert_blocked_target([-10])
        self.assertTrue(all(word & 0xF == 15 for word in slave.words[words:]))

    def test_missing_full_soft_travel_holds_enable_without_sending_motor_target(self):
        controller = self.service.hardware
        controller.limit_memory.configure_manual(controller.adapter, [dict(order=1,enabled=False)], self.service.devices)
        self.enable([1])
        run_id = self.service.run_id
        self.client.target([300])
        self.wait(lambda:self.service.snapshot()['target_reset_revision'] > 0)
        self.assertIn('完整软行程', self.service.message)
        self.assertFalse(self.master.slaves[0].targets)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertEqual(self.service.run_id, run_id)
        self.assertEqual(self.service.snapshot()['targets_deg'], [300])
        self.assert_blocked_target([-300])

    def test_limit_editor_uses_fresh_stopped_position_and_rejects_external_enabled_drive(self):
        device, slave = self.service.devices[1], self.master.slaves[1]
        self.service.hardware.limit_memory.configure_manual(self.service.adapter,
            [dict(order=2,enabled=False)], self.service.devices)
        slave.position = device.position+12345
        self.wait(lambda:self.service.snapshot()['limit_settings']['axes'][1]['position'] == slave.position)
        self.assertEqual(self.service.snapshot()['limit_settings']['axes'][1]['origin'], slave.position)
        memory = self.service.hardware.limit_memory
        memory.observe(self.service.adapter, device, slave.position, True, False)
        slave.sw = 0x27  # External drive state disagrees with service's idle phase.
        self.client.reset_limits([2])
        self.client.wait_for(('fault',))
        self.assertIn('使能状态，禁止接管', self.service.message)
        self.assertIn('positive', memory.entry(self.service.adapter, device))
        self.assertFalse(slave.words or slave.targets)

    def test_coasting_limit_freezes_stop_target_and_defers_reverse_until_stopped(self):
        slave = self.master.slaves[0]
        slave.freeze = True
        coasting = threading.Event()
        original_exchange = slave.exchange
        def exchange():
            original_exchange()
            if coasting.is_set() and slave.words[-1] & 0x100:
                slave.position = (slave.position+1000+2**31)%2**32-2**31
                slave.input = (struct.pack('<HHi',slave.error_code,slave.sw,slave.position)
                               +bytes(16)+struct.pack('<I',slave.raw))
        slave.exchange = exchange
        self.client.enable([1],rpm=60,acceleration_rpm_s=3000)
        self.client.wait_for(('enabled',))
        self.client.target([20])
        self.wait(lambda:bool(slave.targets))
        coasting.set()
        slave.raw = 2
        self.wait(lambda:self.service.snapshot()['target_reset_revision'] > 0)
        fixed_stop_target = struct.unpack_from('<i',slave.output,2)[0]
        revision = self.service.snapshot()['target_reset_revision']
        previous_deceleration = slave.params[0x6085,0]
        for value in (30,40,50):
            self.client.target([value])
        self.client.set_profile(60,1)
        self.wait(lambda:self.service.snapshot()['applied_profile']['acceleration_rpm_s'] == 1)
        self.assertGreaterEqual(slave.params[0x6085,0],previous_deceleration)
        self.client.target([-10])
        target_count = len(slave.targets)
        time.sleep(.08)
        self.assertEqual(self.service.phase,'enabled')
        self.assertEqual(len(slave.targets),target_count)
        self.assertEqual(struct.unpack_from('<i',slave.output,2)[0],fixed_stop_target)
        self.assertTrue(slave.words[-1] & 0x100)
        self.assertEqual(self.service.snapshot()['target_reset_revision'],revision)
        coasting.clear()
        slave.freeze = False
        self.wait(lambda:self.service.snapshot()['axes'][0]['travel_degrees'] < -9.9)
        self.assertEqual(self.service.phase,'enabled')

    def test_independent_braking_and_drive_local_limit_stop_survive_profile_edits(self):
        self.enable([1])
        slave = self.master.slaves[0]
        expected = round(self.service.devices[0].units_per_rev*120/60)
        self.assertEqual(slave.params[0x605D, 0], 2)
        self.assertEqual(slave.params[0x2002, 8], 1)
        self.assertEqual(slave.params[0x6085, 0], expected)
        self.client.set_profile(4, .1)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 4)
        self.assertEqual(slave.params[0x6085, 0], expected)
        self.assertLess(slave.params[0x6084, 0], expected)

    def test_calibrated_manual_envelope_requires_confirmation_and_blocks_excess_travel(self):
        config = platform_options()
        config['allow_motor_debug'] = True
        for leg, device in zip(config['legs'], self.service.devices):
            leg['extension_sign'] = device.extension_sign*(1 if device.positive_direction else -1)
        self.service.platform = PlatformGeometry(config)
        self.service.reference.save(self.service.devices)
        with self.assertRaises(UDPError):
            self.client.enable([1], rpm=60)  # A file alone does not confirm physical coordinates.
        reply = self.client.request('enable', mode='motor', orders=[1], rpm=60,
                                    acceleration_rpm_s=120, reference_confirmed=True)
        self.client.run_id = reply['run_id']
        self.client.wait_for(('enabled',))
        self.client.target([2000])  # Beyond the measured +/-20 mm range.
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] > 1403.9)
        self.assertLess(self.service.snapshot()['axes'][0]['travel_degrees'], 1404.1)
        self.assertEqual(self.service.snapshot()['targets_deg'], [2000])
        target_count = len(self.master.slaves[0].targets)
        for value in (2100, 2200, 2300):
            self.client.target([value])
        time.sleep(.1)
        self.assertEqual(len(self.master.slaves[0].targets), target_count)
        self.assertTrue(self.service.snapshot()['enabled'])
        self.client.set_profile(3000, 3000)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 3000)
        with self.assertRaises(UDPError):
            self.client.set_profile(3000.1, 3000)
        self.client.set_profile(60, 120)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 60)
        self.client.target([10])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] > 9.9)

    def test_passing_narrow_lower_cam_does_not_allow_same_direction_again(self):
        slave = self.master.slaves[0]
        slave.freeze = True
        self.enable([1])
        self.client.target([20])
        self.wait(lambda: bool(slave.targets))
        slave.raw = 2
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        # Simulate coasting beyond the cam, after which its electrical level is clear.
        slave.position = (slave.position+round(self.service.devices[0].units_per_rev/360)+2**31)%2**32-2**31
        slave.raw = 0
        self.wait(lambda: self.service.snapshot()['limits'][0].get('positive_latched') is True
                  and self.service.snapshot()['limits'][0]['digital_inputs'] == 0)
        self.assert_blocked_target([30])
        slave.freeze = False
        self.client.target([-10])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] < -9.9)
        self.assertEqual(self.service.phase, 'enabled')

    def test_targets_profiles_and_stop_restore_with_encoder_rollover(self):
        self.enable([1, 2])
        self.assertFalse(any(s.targets for s in self.master.slaves))
        self.client.target([30, -30])
        self.wait(lambda: all(s.targets for s in self.master.slaves[:2]))
        self.client.target([30, -30])
        self.client.set_profile(4, 40)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 4)
        self.assertEqual([len(s.targets) for s in self.master.slaves], [1, 1, 0])
        self.client.disable()
        state = self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])
        self.assertTrue(self.master.closed)
        self.assertTrue(all(s.params == s.original for s in self.master.slaves))

    def test_active_endpoint_blocks_forward_and_allows_reverse(self):
        self.master.slaves[0].raw = 1 | (1 << 17)  # DI2=N-OT, extension in native negative direction.
        self.wait(lambda: self.service.snapshot()['limits'][0]['triggered'] is True)
        self.enable([1])
        self.assert_blocked_target([-10])
        self.assertFalse(self.master.slaves[0].targets)
        self.client.target([10])
        self.wait(lambda: bool(self.master.slaves[0].targets))
        self.client.disable()
        self.assertTrue(self.client.wait_for(('idle',))['result']['all_disabled'])

    def test_limit_during_motion_holds_enable_and_accepts_fresh_reverse_in_same_run(self):
        self.master.slaves[0].freeze = True
        self.enable()
        run_id = self.client.run_id
        word_counts = [len(s.words) for s in self.master.slaves]
        self.client.target([-20, 20, 20])
        self.wait(lambda: bool(self.master.slaves[0].targets))
        self.master.slaves[0].raw = 1
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        state = self.client.status()
        self.assertEqual(state['phase'], 'enabled')
        self.assertTrue(state['enabled'])
        self.assertEqual(state['run_id'], run_id)
        self.assertIsNone(state['result'])
        self.assertTrue(all(s.sw & 4 for s in self.master.slaves))
        self.assertEqual(state['targets_deg'][0], -20)
        self.assert_blocked_target([-20, 20, 20])
        self.master.slaves[0].freeze = False
        self.client.target([10, 20, 20])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] > 9.9)
        time.sleep(1.1)  # Beyond the old handshake timeout; keep the same run enabled.
        self.assertEqual(self.service.phase, 'enabled')
        self.assertEqual(self.service.run_id, run_id)
        self.assertTrue(all(word & 0xF == 15 for s, start in zip(self.master.slaves, word_counts)
                            for word in s.words[start:]))
        self.client.disable()
        self.assertTrue(self.client.wait_for(('idle',))['result']['all_disabled'])
        self.assertTrue(all(s.params == s.original for s in self.master.slaves))

    def test_conflicting_limits_prevent_any_enable(self):
        self.master.slaves[0].raw = 3
        self.client.enable([1, 2, 3], rpm=5)
        self.client.wait_for(('fault',))
        self.assertFalse(any(word & 0xF == 15 for s in self.master.slaves for word in s.words))

    def test_limit_clear_and_profile_change_do_not_resume_discarded_target(self):
        slave = self.master.slaves[0]
        slave.freeze = True
        self.enable([1])
        self.client.target([-20])
        self.wait(lambda: bool(slave.targets))
        slave.raw = 1
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        count, position = len(slave.targets), slave.position
        slave.raw, slave.freeze = 0, False
        self.client.set_profile(4, 60)
        self.wait(lambda: self.service.snapshot()['applied_profile']['rpm'] == 4)
        time.sleep(.25)
        self.assertEqual(len(slave.targets), count)
        self.assertEqual(slave.position, position)
        self.assertTrue(slave.words[-1] & 0x100)
        self.assertTrue(slave.sw & 4)
        self.assertEqual(self.service.phase, 'enabled')
        self.client.target([10])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] > 9.9)

    def test_runtime_conflicting_limits_hold_enable_and_require_new_target_after_clear(self):
        slave = self.master.slaves[0]
        self.enable([1])
        slave.raw = 3
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        self.assertTrue(self.client.status()['enabled'])
        for target in ([10], [-10]):
            self.assert_blocked_target(target)
        slave.raw = 0
        time.sleep(.15)
        self.assertFalse(slave.targets)
        self.assertTrue(slave.sw & 4)
        self.client.target([10])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] > 9.9)

    def test_limit_arriving_during_reverse_ack_keeps_enable_and_holds_again(self):
        slave = self.master.slaves[0]
        slave.freeze = True
        self.enable([1])
        self.client.target([-20])
        self.wait(lambda: bool(slave.targets))
        slave.raw = 1
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > 0)
        initial_revision = self.service.snapshot()['target_reset_revision']
        original_send = self.master.send_processdata
        injected = []
        def send():
            if struct.unpack_from('<H', slave.output)[0] == 0x2F and not injected:
                slave.raw = 3  # A conflict appears in the final acknowledgement frame.
                injected.append(True)
            original_send()
        self.master.send_processdata = send
        self.client.target([10])
        self.wait(lambda: self.service.snapshot()['target_reset_revision'] > initial_revision)
        self.assertTrue(injected)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertTrue(slave.words[-1] & 0x100)
        self.assertTrue(slave.sw & 4)

    def test_e950_at_enable_with_encoder_jitter_allows_reverse_without_halt(self):
        slave = self.master.slaves[0]
        slave.raw, slave.error_code = 2 | (1 << 16), 0x5443
        original_exchange = slave.exchange
        origin, enabled_words = slave.position, []
        def exchange():
            original_exchange()
            if slave.words[-1] & 0xF == 15 and not slave.targets:
                enabled_words.append(slave.words[-1])
                slave.position = origin + (8 if len(enabled_words) % 2 else -8)
                slave.input = (struct.pack('<HHi', slave.error_code, slave.sw, slave.position)
                               + bytes(16) + struct.pack('<I', slave.raw))
        slave.exchange = exchange
        self.enable([1])
        self.assertEqual(self.service.snapshot()['target_reset_revision'], 0)
        self.assertTrue(enabled_words)
        self.assertFalse(any(word & 0x100 for word in enabled_words))
        self.assert_blocked_target([10])
        self.client.target([-10])
        self.wait(lambda: self.service.snapshot()['axes'][0]['travel_degrees'] < -9.9)
        self.assertEqual(self.service.phase, 'enabled')

    def test_udp_heartbeat_expiry_disables_and_rejects_old_run(self):
        self.enable()
        self.client.run_id = None
        state = self.client.wait_for(('idle',))
        self.assertEqual(state['stop_reason'], 'UDP 心跳超时')
        self.assertTrue(state['result']['all_disabled'])

    def test_fixed_reference_return_survives_signed_counter_rollover(self):
        origins = [s.position for s in self.master.slaves]
        self.client.save_reference(physical_center_confirmed=True)
        state = self.client.wait_for(('idle',))
        reference_id = state['fixed_reference']['id']
        self.enable()
        self.client.target([10, 10, 10])
        self.wait(lambda: all(s.targets for s in self.master.slaves))
        self.client.disable()
        self.client.wait_for(('idle',))
        self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('enabled',))
        self.wait(lambda: [s.position for s in self.master.slaves] == origins)
        self.client.disable()
        state = self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])
        self.assertEqual(state['fixed_reference']['heights_mm'], [0, 0, 0])

    def test_platform_height_stream_and_endpoint_stop(self):
        config = platform_options()
        # DI2=N-OT is the native extension direction on every emulated drive.
        for leg, slave in zip(config['legs'], self.master.slaves):
            leg['extension_sign'] = -1 if slave.params[0x2002, 3] else 1
        self.service.platform = PlatformGeometry(config)
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        self.client.enable_platform([1, 2, 3], config['calibration_id'], reference_confirmed=True)
        self.client.wait_for(('enabled',))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            self.client.heights([.1, .1, .1])
            state = self.client.status()
            actual = state['platform']['actual_lengths_mm']
            if all(x is not None and abs(x-.1)<.01 for x in actual):
                break
            time.sleep(.025)
        else:
            self.fail(str(state))
        self.master.slaves[0].raw = 1
        self.assertTrue(self.client.wait_for(('fault',))['result']['all_disabled'])

    def test_switch_adapter_rebinds_worker_and_invalidates_old_reference(self):
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        other = r'\Device\NPF_{00000000-0000-0000-0000-000000000000}'
        self.client.request('scan', adapter=other)
        state = self.client.wait_for(('idle',))
        self.assertEqual(self.service.hardware.adapter, other)
        self.assertEqual(self.master.opened_adapters[-1], other)
        self.assertFalse(state['fixed_reference']['valid'])

    def test_platform_neutral_tolerates_small_offset_and_keeps_saved_zero(self):
        config=platform_options()
        for leg,slave in zip(config['legs'],self.master.slaves):
            leg['extension_sign']=-1 if slave.params[0x2002,3] else 1
        self.service.platform=PlatformGeometry(config)
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        initial=self.master.slaves[0].position
        self.master.slaves[0].position=(initial+round(.5*2**23/5)+2**31)%2**32-2**31
        self.client.enable_platform([1,2,3],config['calibration_id'],reference_confirmed=True)
        state=self.client.wait_for(('enabled',))
        self.wait(lambda: abs(self.client.status()['platform']['actual_lengths_mm'][0]+.5)<.0001)
        state=self.client.status()
        self.assertAlmostEqual(state['platform']['actual_lengths_mm'][0],-.5,places=4)
        self.assertFalse(any(s.targets for s in self.master.slaves))
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            self.client.heights([0,0,0])
            state=self.client.status()
            if all(v is not None and abs(v)<.005 for v in state['platform']['actual_lengths_mm']):
                break
            time.sleep(.02)
        else:
            self.fail(str(state))
        self.assertEqual(self.service.reference.data['axes'][0]['position'],initial)
        self.assertAlmostEqual((self.master.slaves[0].position-initial+2**31)%2**32-2**31,0,delta=5000)

    def test_platform_neutral_rejects_over_one_mm_before_enabling(self):
        config=platform_options()
        for leg,slave in zip(config['legs'],self.master.slaves):
            leg['extension_sign']=-1 if slave.params[0x2002,3] else 1
        self.service.platform=PlatformGeometry(config)
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        slave=self.master.slaves[0]
        slave.position=(slave.position+round(1.1*2**23/5)+2**31)%2**32-2**31
        self.client.enable_platform([1,2,3],config['calibration_id'],reference_confirmed=True)
        state=self.client.wait_for(('fault',))
        self.assertIn('固定初始位',state['message'])
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(any(w&0xf==15 for s in self.master.slaves for w in s.words))

    def test_nonfault_warnings_and_internal_limiting_do_not_block_control(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.internal_limit = 0x6320, True
        self.enable([1])
        self.client.target([10])
        self.wait(lambda: bool(slave.targets))
        slave.error_code = 0x7305  # Also a warning when the fault bit is clear.
        self.wait(lambda: self.client.status()['axes'][0]['error_code'] == 0x7305)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertFalse(self.client.state['axes'][0]['fault'])
        self.assertTrue(self.client.state['axes'][0]['internal_limit'])
        self.assertEqual(slave.reset_edges, 0)

    def test_explicit_rearm_resets_only_latched_communication_fault_once(self):
        for slave in self.master.slaves:
            slave.error_code, slave.fault_latched = 0x0E08, True
        self.enable()
        self.assertEqual([s.reset_edges for s in self.master.slaves], [1, 1, 1])
        self.assertFalse(any(s.targets for s in self.master.slaves))
        self.assertEqual(self.client.state['targets_deg'], [0, 0, 0])
        self.client.disable()
        state = self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])

    def test_persistent_communication_fault_never_enables_or_repeats_reset(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.fault_latched, slave.resettable = 0x0E08, True, False
        self.client.enable([1, 2, 3], rpm=5)
        state = self.client.wait_for(('fault',))
        self.assertIn('复位后仍有效', state['result']['error'])
        self.assertEqual(slave.reset_edges, 1)
        self.assertFalse(any(word & 0xF == 15 for s in self.master.slaves for word in s.words))

    def test_communication_reset_waits_when_error_clears_before_fault_status(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.fault_latched = 0x0E08, True
        exchange = slave.exchange
        transient = []
        def delayed_status():
            exchange()
            if slave.reset_edges and len(transient) < 20:
                slave.fault_latched = True
                slave.error_code = 0
                slave.sw |= 8
                transient.append(True)
                slave.input = struct.pack('<HHi', 0, slave.sw, slave.position)+bytes(16)+struct.pack('<I', slave.raw)
            elif transient:
                slave.fault_latched = False
        slave.exchange = delayed_status
        self.enable([1])
        self.assertEqual(len(transient), 20)
        self.assertEqual(slave.reset_edges, 1)
        self.assertFalse(slave.targets)

    def test_zero_error_fault_outside_known_reset_is_not_ignored(self):
        self.enable([1])
        slave = self.master.slaves[0]
        slave.fault_latched = True
        self.client.wait_for(('fault',))
        self.assertEqual(slave.reset_edges, 0)
        self.assertIn('0x0000', self.service.message)

    def test_drive_fault_is_published_before_cleanup_and_not_overwritten_by_heartbeat(self):
        self.enable([1])
        release = threading.Event()
        cleanup = _NICSession.cleanup
        def delayed_cleanup(session):
            release.wait(2)
            cleanup(session)
        with patch.object(_NICSession, 'cleanup', delayed_cleanup):
            try:
                slave = self.master.slaves[0]
                slave.error_code, slave.fault_latched = 0x3210, True
                self.wait(lambda:self.service.phase == 'stopping')
                self.assertIn('0x3210', self.service.message)
                self.client.run_id = None
                time.sleep(.6)
                self.assertIn('0x3210', self.service.stop_reason)
                self.assertNotIn('心跳', self.service.stop_reason)
            finally:
                release.set()
            self.client.wait_for(('fault',))

    def test_real_drive_fault_is_not_ignored_or_reset(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.fault_latched = 0x3210, True
        self.client.enable([1], rpm=5)
        state = self.client.wait_for(('fault',))
        self.assertIn('0x3210', state['result']['error'])
        self.assertEqual(slave.reset_edges, 0)
        self.assertFalse(any(s.targets or any(word & 0xF == 15 for word in s.words)
                             for s in self.master.slaves))

    def test_communication_fault_in_motion_stops_without_reset_or_replay(self):
        self.enable([1])
        self.client.target([10])
        slave = self.master.slaves[0]
        self.wait(lambda: bool(slave.targets))
        slave.error_code, slave.fault_latched = 0x0E08, True
        self.client.wait_for(('fault',))
        self.assertEqual(slave.reset_edges, 0)
        old_targets = len(slave.targets)
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.enable([1])
        self.assertEqual(slave.reset_edges, 1)
        self.assertEqual(len(slave.targets), old_targets)
        self.assertEqual(self.client.state['targets_deg'], [0])

    def test_software_position_limit_remains_directional_with_clear_physical_inputs(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.internal_limit = 0x5443, True
        self.wait(lambda: self.service.snapshot()['limits'][0]['positive_limit'] is True)
        self.enable([1])
        self.assert_blocked_target([10])
        self.assertFalse(slave.targets)
        self.client.target([-10])
        self.wait(lambda: bool(slave.targets))
        self.assertEqual(self.service.phase, 'enabled')

    def test_lost_pdo_prevents_communication_fault_reset(self):
        slave = self.master.slaves[0]
        slave.error_code, slave.fault_latched = 0x0E08, True
        self.master.receive_processdata = lambda _: -1
        self.client.enable([1], rpm=5)
        self.client.wait_for(('fault',))
        self.assertEqual(slave.reset_edges, 0)
        self.assertFalse(any(word & 0xF == 15 for s in self.master.slaves for word in s.words))

    def test_nonfault_warning_does_not_block_saving_stationary_reference(self):
        self.master.slaves[0].error_code = 0x6320
        self.client.save_reference(physical_center_confirmed=True)
        state = self.client.wait_for(('idle',))
        self.assertTrue(state['fixed_reference']['valid'])
        self.assertFalse(any(s.reset_edges or s.targets for s in self.master.slaves))

    def test_emergency_brake_warning_still_stops_and_is_never_reset(self):
        self.enable([1])
        slave = self.master.slaves[0]
        slave.error_code = 0x0900  # No.3 in the manual, but this is an actual stop input.
        state = self.client.wait_for(('fault',))
        self.assertIn('紧急刹车', state['result']['error'])
        self.assertEqual(slave.reset_edges, 0)
        self.assertTrue(state['result']['all_disabled'])


if __name__ == '__main__':
    unittest.main()
