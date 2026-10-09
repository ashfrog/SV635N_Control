"""PCIe-8332 regression using a fake APS API; no real card is initialized."""
import ctypes as C
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from aps_backend import (APSLibrary, PCIe8332Controller, ModuleInfo, AsyncCall,
                         SIGNATURES, I32, SVON, ONLINE, MDN, ASTP, EMG, PEL, MEL,
                         LIMIT_MAP_EN, validate_options)
from backend import load_config
from control_common import ControlError, MotionQueue
from motor_service import MotorService
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError


def output(pointer, value):
    pointer._obj.value = value


class FakeAPS:
    def __init__(self):
        self.calls = []
        self.threads = set()
        self.board_bits = 1
        self.card_type = 25
        self.bus = 1
        self.io = {10: ONLINE, 12: ONLINE, 14: ONLINE}
        self.status = {n: MDN for n in self.io}
        self.positions = {n: 1000. for n in self.io}
        self.velocity = {n: 0. for n in self.io}
        self.board_params = {0: 0, 0x19: 1, 0x1A: 0, 0x109: 1, 0x28: 1}
        self.limit_mapping = {n: 0xE for n in self.io}
        self.emg_polarity_works = True
        self.axis_params = {n: 777. for n in self.io}
        self.initial_board_params = self.board_params.copy()
        self.fail_servo = None
        self.fail_target = None
        self.fail_stop = None
        self.fail_start = False
        self.gear = 1
        self.module_axes = [10, 12, 14]
        self.servo_delay = 0
        self.disable_reads = {}
        self.digital_inputs = {n: 0 for n in self.io}
        self.fail_inputs = False
        self.input_bits = 32
        self.di_functions = {3: 14, 5: 15}
        self.stop_codes = {n: 123 for n in self.io}
        self.retain_astp_on_stop = False

    def call(self, name, *args):
        self.calls.append((name, args))
        self.threads.add(threading.get_ident())
        if name == 'APS_initial':
            output(args[0], self.board_bits)
        elif name == 'APS_get_card_name':
            output(args[-1], self.card_type)
        elif name == 'APS_get_first_axisId':
            output(args[-2], 10); output(args[-1], 16)
        elif name == 'APS_start_field_bus':
            if self.fail_start:
                raise ControlError('APS_start_field_bus 返回 APS 错误 -4012')
            self.bus = 6
        elif name == 'APS_stop_field_bus':
            self.bus = 1
        elif name == 'APS_get_field_bus_master_status':
            output(args[-1], self.bus)
        elif name == 'APS_get_board_param':
            output(args[-1], self.board_params[args[1]])
        elif name == 'APS_set_board_param':
            if args[1] == 0 and self.board_params[0] != args[2] and self.emg_polarity_works:
                for axis in self.io:
                    self.io[axis] ^= EMG
            self.board_params[args[1]] = args[2]
        elif name == 'APS_get_axis_param':
            if args[1] != LIMIT_MAP_EN:
                raise AssertionError('Unexpected integer axis parameter')
            output(args[-1], self.limit_mapping[args[0]])
        elif name == 'APS_set_axis_param':
            self.limit_mapping[args[0]] = args[2]
        elif name == 'APS_get_axis_param_f':
            output(args[-1], self.axis_params[args[0]])
        elif name == 'APS_set_axis_param_f':
            self.axis_params[args[0]] = args[2]
        elif name == 'APS_get_field_bus_last_scan_info':
            args[2][0] = 3; output(args[-1], 1)
        elif name == 'APS_get_field_bus_module_info':
            m = args[-1]._obj
            m.VendorID, m.ProductCode, m.RevisionNo = 0x100000, 0xc010e, 0x10000
            m.TotalAxisNum, m.Axis_ID[0] = 1, self.module_axes[args[2]]
        elif name == 'APS_get_field_bus_sdo':
            slave, index, sub = args[2:5]
            value = {(0x2000, 1): 14101, (0x2002, 3): 0 if slave == 2 else 1,
                     (0x6091, 1): self.gear, (0x6091, 2): 1, (0x6041, 0): 0x40,
                     (0x603f, 0): 0, (0x200e, 0x16): slave + 100,
                     (0x2003, 3): self.di_functions[3], (0x2003, 5): self.di_functions[5]}[index, sub]
            for n, v in enumerate(value.to_bytes(4, 'little')):
                args[5][n] = v
            output(args[7], 4)
        elif name == 'APS_get_field_bus_pdo_ODIndex':
            if self.fail_inputs:
                raise ControlError('60FD is not mapped in TxPDO')
            assert args[3:5] == (0x60FD, 0) and args[6] == 32
            value = self.digital_inputs[self.module_axes[args[2]]]
            for n, v in enumerate(value.to_bytes(4, 'little')):
                args[5][n] = v
            output(args[7], self.input_bits)
        elif name == 'APS_motion_io_status':
            axis = args[0]
            if self.disable_reads.get(axis, 0):
                self.disable_reads[axis] -= 1
                if not self.disable_reads[axis]:
                    self.io[axis] &= ~SVON
            return self.io[axis]
        elif name == 'APS_motion_status':
            return self.status[args[0]]
        elif name in ('APS_get_position_f', 'APS_get_command_f'):
            output(args[-1], self.positions[args[0]])
        elif name in ('APS_get_command_velocity_f', 'APS_get_feedback_velocity_f'):
            output(args[-1], self.velocity[args[0]])
        elif name == 'APS_get_stop_code':
            output(args[-1], self.stop_codes[args[0]])
        elif name == 'APS_get_field_bus_alarm':
            output(args[-1], 123)
        elif name == 'APS_set_servo_on':
            axis, enabled = args
            if enabled:
                if self.io[axis] & EMG:
                    raise ControlError('card refuses servo-on while native EMG is active')
                self.io[axis] |= SVON
                if self.fail_servo == axis:
                    raise ControlError('servo enable failed after applied')
            elif self.servo_delay:
                self.disable_reads[axis] = self.servo_delay
            else:
                self.io[axis] &= ~SVON
        elif name == 'APS_ptp_all':
            if self.fail_target == args[0]:
                raise ControlError('target rejected')
            self.status[args[0]] = 0  # Never finishes; updates MUST work while moving.
            self.velocity[args[0]] = 100. if args[2] > self.positions[args[0]] else -100.
        elif name in ('APS_stop_move', 'APS_emg_stop'):
            if name == 'APS_stop_move' and self.fail_stop == args[0]:
                raise ControlError('stop failed')
            historical = self.status[args[0]] & ASTP if self.retain_astp_on_stop else 0
            self.status[args[0]], self.velocity[args[0]] = MDN | historical, 0.
            self.stop_codes[args[0]] = 9 if name == 'APS_stop_move' else 8
        return 0

    def value(self, name, *args, kind=C.c_double):
        value = kind()
        self.call(name, *args, C.byref(value))
        return value.value


class BindingTests(unittest.TestCase):
    def test_abi_and_negative_error_diagnostic(self):
        class Function:
            def __call__(self, *args):
                return -4012
        class DLL:
            def __init__(self):
                for name in SIGNATURES:
                    setattr(self, name, Function())
        dll = DLL()
        with patch('aps_backend.C.WinDLL', return_value=dll):
            api = APSLibrary('fake.dll')
        self.assertEqual(C.sizeof(ModuleInfo), 1312)
        self.assertEqual(C.sizeof(AsyncCall), 16 if C.sizeof(C.c_void_p) == 8 else 12)
        self.assertEqual(dll.APS_get_field_bus_sdo.argtypes[4], C.c_uint16)
        self.assertIs(dll.APS_ptp_all.restype, I32)
        with self.assertRaisesRegex(ControlError, '-4012.*ESI'):
            api.call('APS_start_field_bus', 0, 0, 0)

    def test_missing_dll_and_export(self):
        with patch('aps_backend.C.WinDLL', side_effect=OSError('missing')):
            with self.assertRaisesRegex(ControlError, 'SDK'):
                APSLibrary('missing.dll')
        with patch('aps_backend.C.WinDLL', return_value=object()):
            with self.assertRaisesRegex(ControlError, 'APS_initial'):
                APSLibrary('missing_exports.dll')

    def test_backend_config(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'config.json'
            self.assertEqual(load_config(path)['adapter'], 'PCIe-8332:0')
            path.write_text(json.dumps(dict(aps=dict(board_id=2))), encoding='utf-8')
            self.assertEqual(load_config(path)['adapter'], 'PCIe-8332:2')
            path.write_text(json.dumps(dict(adapter='NPF-old')), encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'NPF'):
                load_config(path)
        for options in [dict(bus_no=1), dict(board_id=True), dict(axis_units_per_rev={'0': 0}),
                        dict(axis_units_per_rev={'01': 100}), dict(poll_interval_s=float('nan')),
                        dict(limit_inputs_connected=0), dict(emg_input_connected='false')]:
            with self.assertRaises(ValueError):
                validate_options(options)
        for limits in [[], {'0': 'di3'}, {'01': 'di1'}, {'0': True}]:
            with self.assertRaises(ValueError):
                validate_options(dict(extension_limits=limits))
            with self.assertRaises(ValueError):
                validate_options(dict(retraction_limits=limits))
        with self.assertRaisesRegex(ValueError, '同一路'):
            validate_options(dict(extension_limits={'0': 'di1'}, retraction_limits={'0': 'di1'}))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.api = FakeAPS()
        self.c = PCIe8332Controller(api_factory=lambda: self.api)

    def tearDown(self):
        self.c.close()

    def run_motion(self, action, orders=(1, 3)):
        devices = self.c.scan()
        commands, stop = MotionQueue(orders, 60, 120), threading.Event()
        ready = False
        def event(e):
            nonlocal ready
            if e['kind'] == 'continuous_ready':
                ready = True
                action(commands, stop, e)
            elif ready:
                action(commands, stop, e)
        return self.c.run_continuous(devices, commands, stop, event)

    def test_mapping_gear_direction_and_moving_override(self):
        self.api.gear = 2
        count = 0
        def action(commands, stop, e):
            nonlocal count
            if e['kind'] == 'continuous_ready':
                commands.submit([90, 180])
            elif e['kind'] == 'command':
                count += 1
                if count == 1:
                    commands.submit([180, -90])
                else:
                    stop.set()
        report = self.run_motion(action)
        moves = [args for name, args in self.api.calls if name == 'APS_ptp_all']
        self.assertEqual([m[0] for m in moves], [10, 14, 10, 14])
        self.assertEqual([m[1] for m in moves], [0] * 4)
        units = 2**22
        self.assertEqual([m[2] for m in moves], [1000 + units/4, 1000 - units/2,
                                               1000 + units/2, 1000 + units/4])
        self.assertTrue(all(m[-1] is None and m[4] == units and m[6] == 2*units for m in moves))
        self.assertTrue(report['all_disabled'])
        self.assertTrue(report['stopped'])
        self.assertEqual(self.api.axis_params, {10: 777., 12: 777., 14: 777.})
        self.assertFalse(any(name == 'APS_set_servo_on' and args[0] == 12 for name, args in self.api.calls))

    def test_selected_axis_unit_override(self):
        self.c.options['axis_units_per_rev'] = {'14': 3600.}
        devices = self.c.scan()
        self.assertEqual(devices[2].units_per_rev, 3600.)
        self.assertEqual(devices[0].units_per_rev, 2**23)

    def test_sensor_bits_and_unknown_feedback(self):
        self.c.options['extension_limits'] = {'10': 'di1', '14': 'di2'}
        devices = self.c.scan()
        self.api.digital_inputs[10] = 2 | (1 << 16)
        self.api.digital_inputs[14] = 1 | (1 << 17)
        sensors = self.c.read_inputs(devices)
        self.assertTrue(sensors[1]['triggered'])
        self.assertTrue(sensors[3]['triggered'])
        self.assertEqual(sensors[1]['extension_sign'], 1)
        self.assertEqual(sensors[3]['extension_sign'], -1)
        self.assertEqual(sensors[2]['state'], 'unconfigured')
        self.assertIsNone(sensors[2]['triggered'])
        self.assertTrue(sensors[1]['di1'])
        self.assertFalse(sensors[1]['negative_limit'])
        self.assertTrue(all(s['retraction_state'] == 'no_sensor' for s in sensors.values()))
        self.api.input_bits = 4  # Byte counts must not be mistaken for bit counts.
        self.assertEqual(self.c.read_inputs(devices)[1]['state'], 'unavailable')
        self.api.input_bits = 32
        self.api.fail_inputs = True
        self.assertIsNone(self.c.read_inputs(devices)[1]['triggered'])

    def test_bad_sensor_function_or_axis_prevents_scan(self):
        self.c.options['extension_limits'] = {'10': 'di1'}
        self.api.di_functions[3] = 31
        with self.assertRaisesRegex(ControlError, 'P-OT'):
            self.c.scan()
        self.api.di_functions[3] = 14
        self.c.options['extension_limits'] = {'999': 'di1'}
        with self.assertRaisesRegex(ControlError, '不存在'):
            self.c.scan()

    def test_single_limit_blocks_outward_and_allows_inward_both_directions(self):
        for input_name, bit, native_bit, outward in [('di1', 2, PEL, 5), ('di2', 1, MEL, -5)]:
            with self.subTest(input=input_name):
                self.c.options['extension_limits'] = {'10': input_name}
                self.c.options['limit_inputs_connected'] = False
                self.api.digital_inputs[10] = bit
                self.api.io[10] |= native_bit
                def action(q, stop, e):
                    if e['kind'] == 'continuous_ready':
                        q.submit([outward])
                    elif e['kind'] == 'limit_blocked':
                        q.submit([-outward])
                    elif e['kind'] == 'command':
                        stop.set()
                report = self.run_motion(action, (1,))
                self.assertTrue(report['stopped'], report)
                self.assertTrue(report['all_disabled'])
                moves = [a for n, a in self.api.calls if n == 'APS_ptp_all']
                self.assertEqual(len(moves), 1)
                self.assertLess((moves[0][2] - 1000) * (1 if input_name == 'di1' else -1), 0)
                self.api.io[10] &= ~native_bit
                self.api.calls.clear()

    def test_dual_sensor_states_and_driver_configuration(self):
        self.c.options['extension_limits'] = {'10': 'di2'}
        self.c.options['retraction_limits'] = {'10': 'di1'}
        devices = self.c.scan()
        for raw, extension, retraction in [(0, False, False), (1, True, False),
                                            (2, False, True), (3, True, True)]:
            self.api.digital_inputs[10] = raw
            sensor = self.c.read_inputs(devices)[1]
            self.assertEqual(sensor['triggered'], extension)
            self.assertEqual(sensor['retraction_triggered'], retraction)
            self.assertEqual(sensor['extension_sign'], -1)
            self.assertEqual(sensor['retraction_sign'], 1)
            self.assertEqual(sensor['conflict'], extension and retraction)
        self.api.fail_inputs = True
        sensor = self.c.read_inputs(devices)[1]
        self.assertEqual(sensor['retraction_state'], 'unavailable')
        self.assertIsNone(sensor['retraction_triggered'])
        self.api.fail_inputs = False
        self.api.di_functions[3] = 15  # Both DI inputs assigned N-OT is not valid dual-end protection.
        with self.assertRaisesRegex(ControlError, '分别配置'):
            self.c.scan()
        self.api.di_functions[3] = 14
        self.c.options['retraction_limits'] = {'999': 'di1'}
        with self.assertRaisesRegex(ControlError, '不存在'):
            self.c.scan()

    def test_dual_limits_block_either_end_and_allow_escape_with_ui_direction_conversion(self):
        for order, axis, ui_sign in [(1, 10, 1), (3, 14, -1)]:
            for connected in (False, True):
                for raw, native, direction in [(1, MEL, -1), (2, PEL, 1)]:
                    with self.subTest(order=order, connected=connected, raw=raw):
                        self.c.options['extension_limits'] = {str(axis): 'di2'}
                        self.c.options['retraction_limits'] = {str(axis): 'di1'}
                        self.c.options['limit_inputs_connected'] = connected
                        self.api.digital_inputs[axis] = raw
                        self.api.io[axis] |= native
                        outward = 5 * direction * ui_sign
                        def action(q, stop, e):
                            if e['kind'] == 'continuous_ready':
                                self.assertEqual(self.api.limit_mapping[axis], 14)
                                q.submit([outward])
                            elif e['kind'] == 'limit_blocked':
                                q.submit([-outward])
                            elif e['kind'] == 'command':
                                stop.set()
                        report = self.run_motion(action, (order,))
                        self.assertTrue(report['stopped'], report)
                        self.assertTrue(report['all_disabled'])
                        moves = [a for n, a in self.api.calls if n == 'APS_ptp_all']
                        self.assertEqual(len(moves), 1)
                        self.assertLess((moves[0][2] - 1000) * direction, 0)
                        self.api.digital_inputs[axis] = 0
                        self.api.io[axis] &= ~native
                        self.api.calls.clear()

    def test_both_endpoints_triggered_prevent_enable_and_fault_during_run(self):
        self.c.options['extension_limits'] = {'10': 'di2'}
        self.c.options['retraction_limits'] = {'10': 'di1'}
        self.api.digital_inputs[10] = 3
        report = self.run_motion(lambda *_: self.fail('must not enable'), (1, 3))
        self.assertIn('同时触发', report['error'])
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))
        self.api.digital_inputs[10] = 0
        def action(q, stop, e):
            if e['kind'] == 'continuous_ready':
                self.api.digital_inputs[10] = 3
        report = self.run_motion(action, (1, 3))
        self.assertIn('同时触发', report['error'])
        self.assertTrue(report['all_disabled'])

    def test_retraction_trigger_stops_motion_and_clearing_does_not_resume_old_target(self):
        self.c.options['extension_limits'] = {'10': 'di2'}
        self.c.options['retraction_limits'] = {'10': 'di1'}
        self.api.retain_astp_on_stop = True
        blocked = False
        after_clear = 0
        def action(q, stop, e):
            nonlocal blocked, after_clear
            if e['kind'] == 'continuous_ready':
                q.submit([5])  # Native positive moves toward retraction DI1.
            elif e['kind'] == 'command':
                self.api.digital_inputs[10] = 2
                self.api.status[10] = MDN | ASTP
                self.api.stop_codes[10] = 4
            elif e['kind'] == 'limit_blocked':
                self.assertIn('防缩回', e['text'])
                blocked = True
                self.api.digital_inputs[10] = 0
            elif e['kind'] == 'status' and blocked:
                after_clear += 1
                self.assertTrue(e['axes'][0]['enabled'])
                if after_clear == 2:
                    self.assertEqual(sum(n == 'APS_ptp_all' for n, _ in self.api.calls), 1)
                    stop.set()
        report = self.run_motion(action, (1,))
        self.assertTrue(blocked)
        self.assertTrue(report['stopped'], report)

    def test_trigger_during_motion_stops_and_does_not_resume_on_clear(self):
        self.c.options['extension_limits'] = {'10': 'di1'}
        self.api.retain_astp_on_stop = True
        moved = blocked = False
        status_after_clear = 0
        def action(q, stop, e):
            nonlocal moved, blocked, status_after_clear
            if e['kind'] == 'continuous_ready':
                q.submit([5])
            elif e['kind'] == 'command':
                moved = True
                self.api.digital_inputs[10] = 2 | (1 << 16)
                self.api.io[10] |= PEL
                self.api.status[10] = MDN | ASTP
                self.api.stop_codes[10] = 4
            elif e['kind'] == 'limit_blocked':
                blocked = True
                self.api.digital_inputs[10] = 0
                self.api.io[10] &= ~PEL
            elif e['kind'] == 'status' and blocked:
                status_after_clear += 1
                self.assertTrue(e['axes'][0]['enabled'])
                if status_after_clear == 2:
                    self.assertEqual(sum(n == 'APS_ptp_all' for n, _ in self.api.calls), 1)
                    q.submit([-5])
                elif status_after_clear == 3:
                    stop.set()
        report = self.run_motion(action, (1,))
        self.assertTrue(moved and blocked)
        self.assertTrue(report['stopped'], report)
        self.assertEqual(report['started_commands'], 2)

    def test_configured_sensor_failure_prevents_enable_and_stops_run(self):
        self.c.options['extension_limits'] = {'10': 'di1'}
        self.api.fail_inputs = True
        report = self.run_motion(lambda *_: self.fail('must not enable'), (1,))
        self.assertIn('反馈失效', report['error'])
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))
        self.api.fail_inputs = False
        def action(q, stop, e):
            if e['kind'] == 'continuous_ready':
                self.api.fail_inputs = True
        report = self.run_motion(action, (1,))
        self.assertIn('反馈失效', report['error'])
        self.assertTrue(report['all_disabled'])

    def test_native_limit_stop_allows_escape_but_other_astp_is_fault(self):
        self.c.options['extension_limits'] = {'14': 'di1'}
        self.api.digital_inputs[14] = 2
        self.api.io[14] |= PEL
        def action(q, stop, e):
            if e['kind'] == 'continuous_ready':
                self.api.status[14] = MDN | ASTP
                self.api.stop_codes[14] = 4
                q.submit([-5])  # H02.02=0: negative UI angle extends in native positive direction.
            elif e['kind'] == 'limit_blocked':
                q.submit([5])
            elif e['kind'] == 'command':
                stop.set()
        report = self.run_motion(action, (3,))
        self.assertTrue(report['stopped'], report)
        moves = [a for n, a in self.api.calls if n == 'APS_ptp_all']
        self.assertEqual(len(moves), 1)
        self.assertLess(moves[0][2], 1000)
        self.api.io[14] &= ~PEL
        self.api.digital_inputs[14] = 0
        self.api.status[14] = MDN
        def fail_action(q, stop, e):
            if e['kind'] == 'continuous_ready':
                self.api.status[14] = MDN | ASTP
                self.api.stop_codes[14] = 8
        report = self.run_motion(fail_action, (3,))
        self.assertIn('stop_code=8', report['error'])

    def test_arrival_keeps_servo_enabled_until_explicit_stop(self):
        status_count = 0
        moved = False
        def action(q, stop, e):
            nonlocal status_count, moved
            if e['kind'] == 'continuous_ready':
                q.submit([90])
            elif e['kind'] == 'command':
                moved = True
                self.api.positions[10] = 1000 + 2**23 / 4
                self.api.status[10], self.api.velocity[10] = MDN, 0.
            elif e['kind'] == 'status' and moved:
                status_count += 1
                self.assertTrue(e['axes'][0]['enabled'])
                self.assertEqual(e['axes'][0]['travel_degrees'], 90)
                if status_count == 2:
                    stop.set()
        report = self.run_motion(action, (1,))
        self.assertEqual(status_count, 2)
        self.assertEqual(report['completed_commands'], 1)
        self.assertEqual(report['started_commands'], 1)
        self.assertTrue(report['all_disabled'])

    def test_partial_enable_failure_stops_and_disables_all_owned(self):
        self.api.fail_servo = 14
        report = self.run_motion(lambda *_: self.fail('must not be ready'))
        self.assertIn('servo enable failed', report['error'])
        self.assertTrue(report['all_disabled'])
        self.assertEqual([a[0] for n, a in self.api.calls if n == 'APS_stop_move'], [10, 14])
        self.assertFalse(any(io & SVON for io in self.api.io.values()))
        self.assertEqual(self.api.board_params, self.api.initial_board_params)

    def test_unwired_limits_disable_only_selected_mapping_and_restore(self):
        self.c.options['limit_inputs_connected'] = False
        self.api.io[10] |= PEL | MEL
        def action(q, stop, e):
            self.assertEqual(self.api.limit_mapping, {10: 8, 12: 14, 14: 8})
            stop.set()
        report = self.run_motion(action)
        self.assertTrue(report['stopped'])
        self.assertTrue(report['all_disabled'])
        self.assertEqual(self.api.limit_mapping, {10: 14, 12: 14, 14: 14})

    def test_connected_limit_is_reported_by_name_and_blocks_enable(self):
        self.api.io[10] |= PEL
        report = self.run_motion(lambda *_: self.fail('must not be ready'))
        self.assertIn('正限位 PEL', report['error'])
        self.assertNotIn('急停输入', report['error'])
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))

    def test_screenshot_emg_is_not_ignored_when_only_limits_unwired(self):
        self.c.options['limit_inputs_connected'] = False
        self.api.io[10] = 0x1000050
        report = self.run_motion(lambda *_: self.fail('must not be ready'))
        self.assertIn('急停输入 EMG', report['error'])
        self.assertNotIn('正限位', report['error'])
        self.assertNotIn('负限位', report['error'])
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))

    def test_unwired_emg_releases_native_input_and_restores_on_close(self):
        self.c.options['emg_input_connected'] = False
        for axis in self.api.io:
            self.api.io[axis] = 0x1000050
        report = self.run_motion(lambda q, stop, e: stop.set())
        self.assertTrue(report['stopped'])
        self.assertTrue(report['all_disabled'])
        self.assertEqual(self.api.board_params[0], 1)
        self.assertFalse(any(io & EMG for io in self.api.io.values()))
        self.c.close()
        self.assertEqual(self.api.board_params[0], 0)

    def test_unwired_emg_still_active_does_not_enable(self):
        self.c.options['emg_input_connected'] = False
        self.api.emg_polarity_works = False
        for axis in self.api.io:
            self.api.io[axis] |= EMG
        with self.assertRaisesRegex(ControlError, '仍有效'):
            self.c.scan()
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))
        self.assertEqual(self.api.board_params[0], 0)

    def test_released_emg_historical_stop_clears_on_explicit_target(self):
        self.c.options['emg_input_connected'] = False
        for axis in self.api.io:
            self.api.io[axis] |= EMG
            self.api.status[axis] = MDN | ASTP
        def action(q, stop, e):
            if e['kind'] == 'continuous_ready':
                q.submit([1, 2])
            elif e['kind'] == 'command':
                stop.set()
        report = self.run_motion(action)
        self.assertTrue(report['stopped'])
        self.assertTrue(report['all_disabled'])
        self.assertEqual(report['started_commands'], 1)

    def test_alarm_and_software_limit_remain_active_without_physical_limits(self):
        self.c.options['limit_inputs_connected'] = False
        for bit, name in ((1, '伺服报警 ALM'), (1 << 11, '软件正限位 SPEL')):
            self.api.io[10] = ONLINE | bit
            report = self.run_motion(lambda *_: self.fail('must not be ready'))
            self.assertIn(name, report['error'])
            self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))

    def test_disable_waits_for_feedback(self):
        self.api.servo_delay = 3
        report = self.run_motion(lambda q, stop, e: stop.set())
        self.assertTrue(report['all_disabled'])
        self.assertFalse(any(io & SVON for io in self.api.io.values()))

    def test_runtime_faults_and_stop_fallback(self):
        for failure in ('alarm', 'offline', 'bus', 'abnormal', 'target'):
            with self.subTest(failure=failure):
                self.api = FakeAPS()
                self.api.fail_stop = 10
                self.c.api_factory = lambda: self.api
                def action(q, stop, e):
                    if e['kind'] != 'continuous_ready':
                        return
                    if failure == 'alarm': self.api.io[10] |= 1
                    if failure == 'offline': self.api.io[10] &= ~ONLINE
                    if failure == 'bus': self.api.bus = 5
                    if failure == 'abnormal': self.api.status[10] = MDN | ASTP
                    if failure == 'target':
                        self.api.fail_target = 10
                        q.submit([1, 2])
                report = self.run_motion(action)
                self.assertTrue(report['error'])
                self.assertEqual(report['all_disabled'], failure not in ('offline', 'bus'))
                self.assertTrue(any(n == 'APS_emg_stop' and a[0] == 10 for n, a in self.api.calls))
                self.assertFalse(self.c.initialized)

    def test_changed_mapping_rejected_before_enable(self):
        devices = self.c.scan()
        self.api.module_axes = [12, 10, 14]
        report = self.c.run_continuous(devices, MotionQueue([1], 60, 120), threading.Event(), lambda _: None)
        self.assertIn('发生变化', report['error'])
        self.assertFalse(any(n == 'APS_set_servo_on' for n, _ in self.api.calls))

    def test_card_discovery_and_close_on_failed_start(self):
        for failure in ('wrong_card', 'missing_card', 'start'):
            with self.subTest(failure=failure):
                self.api = FakeAPS()
                if failure == 'wrong_card': self.api.card_type = 26
                if failure == 'missing_card': self.api.board_bits = 0
                if failure == 'start': self.api.fail_start = True
                self.c.api_factory = lambda: self.api
                with self.assertRaises(ControlError):
                    self.c.scan()
                self.assertFalse(self.c.initialized)
                self.assertTrue(any(n == 'APS_close' for n, _ in self.api.calls))
                self.assertEqual(self.api.board_params, self.api.initial_board_params)

    def test_boot_auto_connected_bus_and_explicit_eni_regeneration(self):
        self.api.bus = 6
        self.assertEqual(len(self.c.scan()), 3)
        self.assertFalse(any(n == 'APS_start_field_bus' for n, _ in self.api.calls))
        self.c.close()
        self.api.calls.clear()
        self.api.bus = 6
        self.c.options['regenerate_eni'] = True
        self.c.scan()
        names = [n for n, _ in self.api.calls]
        self.assertLess(names.index('APS_stop_field_bus'), names.index('APS_scan_field_bus'))
        self.assertLess(names.index('APS_scan_field_bus'), names.index('APS_start_field_bus'))

    def test_already_enabled_axis_prevents_bus_adoption_and_new_enable(self):
        self.api.bus = 6
        self.api.io[12] |= SVON
        with self.assertRaisesRegex(ControlError, '已使能'):
            self.c.scan()
        self.assertFalse(any(n in ('APS_set_servo_on', 'APS_stop_field_bus') for n, _ in self.api.calls))


class APSUDPTests(unittest.TestCase):
    def test_udp_dual_limits_escape_each_end_and_latch_conflict(self):
        api = FakeAPS()
        api.digital_inputs[14] = 2 | (1 << 16)  # Retraction DI1 / P-OT.
        service = MotorService(aps_options=dict(extension_limits={'14': 'di2'},
                                               retraction_limits={'14': 'di1'}))
        service.hardware.api_factory = lambda: api
        server = UDPServer(service, port=0).start()
        client = MotorClient(port=server.port)
        def wait_state(predicate):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                state = client.status()
                if predicate(state):
                    return state
                time.sleep(.01)
            self.fail(str(state))
        try:
            client.hello()
            client.request('scan')
            client.wait_for(('idle',))
            client.enable([3])
            state = wait_state(lambda s: s['phase'] == 'enabled' and s['axes'])
            sensor = next(s for s in state['limits'] if s['order'] == 3)
            self.assertTrue(sensor['retraction_triggered'])
            with self.assertRaisesRegex(UDPError, '只允许推出'):
                client.target([-5])  # Axis14 H02.02=0: negative UI means native positive.
            client.target([5])
            wait_state(lambda s: any(n == 'APS_ptp_all' for n, _ in api.calls))
            api.digital_inputs[14] = 1 | (1 << 17)  # Extension DI2 / N-OT.
            wait_state(lambda s: s['phase'] == 'enabled' and '防推出限位触发' in s['message'])
            with self.assertRaisesRegex(UDPError, '只允许缩回'):
                client.target([5])
            client.target([-5])
            wait_state(lambda s: sum(n == 'APS_ptp_all' for n, _ in api.calls) == 2)
            api.digital_inputs[14] = 3
            state = client.wait_for(('fault',))
            self.assertIn('同时触发', state['message'])
            self.assertTrue(state['result']['all_disabled'])
        finally:
            client.close()
            server.close()
            service.close()
        self.assertEqual(len(api.threads), 1)

    def test_udp_rejects_extension_at_limit_and_accepts_retraction(self):
        api = FakeAPS()
        api.digital_inputs[14] = 2 | (1 << 16)
        api.io[14] |= PEL
        service = MotorService(aps_options=dict(extension_limits={'14': 'di1'},
                                               limit_inputs_connected=False))
        service.hardware.api_factory = lambda: api
        server = UDPServer(service, port=0).start()
        client = MotorClient(port=server.port)
        try:
            client.hello()
            client.request('scan')
            client.wait_for(('idle',))
            client.enable([3])
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                state = client.status()
                if state['phase'] == 'enabled' and state['axes']:
                    break
                time.sleep(.02)
            self.assertEqual(state['phase'], 'enabled')
            sequence = service.target_sequence
            heartbeat = service.last_heartbeat
            with self.assertRaisesRegex(UDPError, '只允许缩回'):
                client.target([-5])
            self.assertEqual(service.target_sequence, sequence)
            self.assertFalse(any(n == 'APS_ptp_all' for n, _ in api.calls))
            self.assertGreaterEqual(service.last_heartbeat, heartbeat)  # Independent heartbeats remain allowed.
            client.target([5])
            deadline = time.monotonic() + 2
            while not any(n == 'APS_ptp_all' for n, _ in api.calls) and time.monotonic() < deadline:
                time.sleep(.01)
            moves = [a for n, a in api.calls if n == 'APS_ptp_all']
            self.assertEqual(len(moves), 1)
            self.assertLess(moves[0][2], 1000)
            self.assertEqual(service.phase, 'enabled')
        finally:
            client.close()
            server.close()
            service.close()
        self.assertEqual(len(api.threads), 1)

    def test_idle_inputs_update_over_udp_without_enabling_any_motor(self):
        api = FakeAPS()
        service = MotorService(aps_options=dict(extension_limits={'14': 'di2'}))
        service.hardware.api_factory = lambda: api
        server = UDPServer(service, port=0).start()
        client = MotorClient(port=server.port)
        def wait_sensor(predicate):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                state = client.status()
                sensors = {s['order']: s for s in state.get('limits', [])}
                if 3 in sensors and predicate(sensors[3]):
                    return state
                time.sleep(.02)
            self.fail(str(state))
        try:
            client.hello()
            client.request('scan')
            client.wait_for(('idle',))
            wait_sensor(lambda s: s['state'] == 'clear')
            api.digital_inputs[14] = 1 | (1 << 17)
            state = wait_sensor(lambda s: s['state'] == 'triggered')
            self.assertEqual(state['orders'], [])
            api.digital_inputs[14] = 0
            wait_sensor(lambda s: s['state'] == 'clear')
            api.fail_inputs = True
            wait_sensor(lambda s: s['state'] == 'unavailable' and s['triggered'] is None)
            self.assertFalse(any(n in ('APS_set_servo_on', 'APS_ptp_all') for n, _ in api.calls))
        finally:
            client.close()
            server.close()
            service.close()
        self.assertEqual(len(api.threads), 1)

    def test_real_udp_target_and_heartbeat_stop_with_single_hardware_thread(self):
        api = FakeAPS()
        service = MotorService(heartbeat_timeout=.2)
        service.hardware.api_factory = lambda: api
        server = UDPServer(service, port=0).start()
        client = MotorClient(port=server.port)
        try:
            self.assertEqual(service.adapters()[0]['name'], 'PCIe-8332:0')
            self.assertEqual(api.calls, [])
            client.hello()
            client.request('scan')
            state = client.wait_for(('idle',))
            self.assertEqual([d['axis_id'] for d in state['devices']], [10, 12, 14])
            client.enable([1, 3], rpm=60, acceleration_rpm_s=120)
            client.wait_for(('enabled',))
            client.target([5, -5])
            deadline = time.monotonic() + 2
            while not any(n == 'APS_ptp_all' for n, _ in api.calls) and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertEqual([a[0] for n, a in api.calls if n == 'APS_ptp_all'], [10, 14])
            client.run_id = None  # Client process disappears: stop heartbeats, send no disable.
            state = client.wait_for(('idle',), timeout=3)
            self.assertEqual(state['stop_reason'], 'UDP 心跳超时')
            self.assertTrue(state['result']['all_disabled'])
        finally:
            client.close()
            server.close()
            service.close()
        self.assertEqual(len(api.threads), 1)
        self.assertEqual(api.board_params, api.initial_board_params)
        names = [name for name, _ in api.calls]
        self.assertLess(names.index('APS_stop_field_bus'), names.index('APS_close'))


if __name__ == '__main__':
    unittest.main()
