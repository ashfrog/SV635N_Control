"""Local continuous-control tests; all EtherCAT devices are emulated."""
import struct
import threading
import time
import unittest

from core import ControlError, EtherCATController, Move, Stopped, _Session, displacement
from continuous_control import MotionQueue, run_continuous
from test_platform_control import FakeMaster


class QueueTests(unittest.TestCase):
    def test_profile_has_no_software_upper_or_lower_range(self):
        for rpm, acceleration in ((3000, 6000), (.05, .5), (1e308, 1e308), (5e-324, 5e-324)):
            commands = MotionQueue([1], rpm=rpm, acceleration=acceleration)
            self.assertEqual((commands.rpm, commands.acceleration), (rpm, acceleration))
        Move(1, .1, .05, acceleration_rpm_s=.5).validate()
        self.assertTrue(Move(1, 30, 1e308, acceleration_rpm_s=1e308).estimated_seconds > 0)
        for value in (0, -1, float('inf'), float('nan'), True, None, 10**3000):
            with self.assertRaises(ControlError):
                MotionQueue([1], rpm=value)
            with self.assertRaises(ControlError):
                MotionQueue([1], acceleration=value)

    def test_latest_relative_projection_and_atomic_rejection(self):
        commands = MotionQueue([1, 2, 3])
        with self.assertRaises(ControlError):
            commands.submit([1, 2, 3])
        commands.mark_ready()
        source = [1, 2, 3]
        first = commands.submit(source)
        source[0] = 9
        second = commands.submit([2, 2, 2], relative=True)
        self.assertEqual(first.targets, (1, 2, 3))
        self.assertEqual(second.targets, (3, 4, 5))
        with self.assertRaises(ControlError):
            commands.submit([1000000, 0, 0], relative=True)
        self.assertEqual(commands.planned, (3, 4, 5))
        self.assertEqual(commands.pending(), 1)
        self.assertEqual(commands.pop(), second)
        self.assertIsNone(commands.pop())

    def test_finiteness_latest_slot_and_closed_session(self):
        commands = MotionQueue([1])
        commands.mark_ready()
        for values in ([float('nan')], [float('inf')], [10**3000], [True], [], [1, 2]):
            with self.assertRaises(ControlError):
                commands.submit(values)
        for _ in range(256):
            commands.submit([1], relative=True)
        self.assertEqual(commands.pending(), 1)
        self.assertEqual(commands.pop().targets, (256,))
        commands.close()
        self.assertEqual(commands.pending(), 0)
        with self.assertRaises(ControlError):
            commands.submit([0])
        fresh = MotionQueue([1])
        fresh.mark_ready()
        self.assertEqual(fresh.submit([1], relative=True).targets, (1,))

    def test_command_duration_controls_timeout(self):
        commands = MotionQueue([1], rpm=.1)
        commands.mark_ready()
        self.assertEqual(commands.submit([0]).estimated_seconds, 0)
        commands.submit([1])
        with self.assertRaises(ControlError):
            commands.submit([3600])

    def test_cumulative_targets_have_no_degree_limit(self):
        commands = MotionQueue([1])
        commands.mark_ready()
        for _ in range(100):
            commands.submit([1000], relative=True)
            commands.pop()
        self.assertEqual(commands.planned, (100000,))
        self.assertEqual(commands.submit([100001]).targets, (100001,))


class CommunicationTests(unittest.TestCase):
    def session(self):
        master = FakeMaster()
        controller = EtherCATController('fake', master_factory=lambda: master)
        devices = controller._discover(master)
        session = _Session(controller, [Move(1)], devices, threading.Event(), lambda _: None, False)
        session.expected = master.expected_wkc
        session.axes = [dict(slave=slave, device=device, request=Move(device.order), cw=0x3F,
                             target=slave.position, last=(0, 0x40, slave.position))
                        for device, slave in zip(devices, master.slaves)]
        return session, master

    def test_transient_timeout_and_partial_wkc_retry_without_new_target_edge(self):
        for bad_wkc in (-1, 0, 9):
            with self.subTest(wkc=bad_wkc):
                session, master = self.session()
                calls, feedback = [], []
                def receive(_):
                    calls.append(1)
                    if len(calls) == 1:
                        # An incomplete frame must not update position or handshake state.
                        master.slaves[0].input = struct.pack('<HHi', 1, 8, -999) + bytes(20)
                        return bad_wkc
                    return master.expected_wkc
                master.receive_processdata = receive
                session.update_feedback = lambda axis: feedback.append(axis['last'])
                session.tick()
                self.assertEqual(len(calls), 2)
                self.assertEqual(len(feedback), 4)
                self.assertNotIn((1, 8, -999), feedback)
                self.assertEqual(session.report['pdo_failed_exchanges'], 1)
                self.assertEqual(session.report['pdo_recoveries'], 1)
                self.assertTrue(all(len(s.targets) == 1 for s in master.slaves))

    def test_persistent_wkc_failure_is_bounded(self):
        session, master = self.session()
        calls = []
        def receive(_):
            calls.append(1)
            return -1
        master.receive_processdata = receive
        with self.assertRaisesRegex(ControlError, 'WKC=-1'):
            session.tick()
        self.assertLessEqual(len(calls), 5)
        self.assertNotIn('pdo_recoveries', session.report)
        self.assertEqual(session.axes[0]['last'][1], 0x40)

    def test_stop_prevents_retry_and_slow_recovery_still_stops(self):
        session, master = self.session()
        calls = []
        def stopped_receive(_):
            calls.append(1)
            session.stop.set()
            return -1
        master.receive_processdata = stopped_receive
        with self.assertRaises(Stopped):
            session.tick()
        self.assertEqual(len(calls), 1)

        session, master = self.session()
        def slow_receive(_):
            time.sleep(.012)
            return -1
        master.receive_processdata = slow_receive
        with self.assertRaises(ControlError):
            session.tick()
        self.assertEqual(session.report['pdo_failed_exchanges'], 1)

    def test_alarm_after_recovery_is_not_ignored(self):
        session, master = self.session()
        calls = []
        def receive(_):
            calls.append(1)
            if len(calls) == 1:
                return -1
            slave = master.slaves[0]
            slave.input = struct.pack('<HHi', 1, slave.sw | 8, slave.position) + bytes(20)
            return master.expected_wkc
        master.receive_processdata = receive
        with self.assertRaisesRegex(ControlError, '报警'):
            session.tick()


class SessionTests(unittest.TestCase):
    def test_transient_wkc_during_target_ack_continues_and_restores(self):
        master, controller, devices = self.make_controller()
        commands = MotionQueue([1, 2, 3])
        stop = threading.Event()
        failed = False
        def receive(_):
            nonlocal failed
            if master.slaves[0].targets and not failed:
                failed = True
                return -1
            return master.expected_wkc
        master.receive_processdata = receive
        def callback(event):
            if event['kind'] == 'continuous_ready':
                commands.submit([1, 2, 3])
            elif event['kind'] == 'command' and event['stage'] == 'completed':
                stop.set()
        report = run_continuous(controller, devices, commands, stop, callback)
        self.assertTrue(report['stopped'], report)
        self.assertEqual(report['completed_commands'], 1)
        self.assertEqual(report['pdo_recoveries'], 1)
        self.assertTrue(all(len(s.targets) == 1 for s in master.slaves[:3]))
        self.assert_clean(master, report, (1, 2, 3))

    def make_controller(self):
        master = FakeMaster()
        controller = EtherCATController('fake', master_factory=lambda: master)
        return master, controller, controller._discover(master)

    def assert_clean(self, master, report, orders):
        self.assertTrue(report['all_disabled'], report)
        self.assertNotIn('cleanup_errors', report)
        self.assertTrue(master.closed)
        for axis, slave in zip(report['axes'], master.slaves):
            if axis['order'] in orders:
                self.assertTrue(axis['parameters_restored'], report)
                for index in (0x6081, 0x6083, 0x6084, 0x6085, 0x6060):
                    self.assertEqual(slave.params[(index, 0)], slave.original[(index, 0)])
                self.assertEqual(slave.params[(0x200E, 2)], 1)
            else:
                self.assertFalse(slave.targets)
                self.assertTrue(all(word == 0 for word in slave.words))

    def test_high_profile_and_tiny_positive_profile_use_native_parameters(self):
        for rpm, acceleration in ((3000, 6000), (5e-324, 5e-324)):
            with self.subTest(rpm=rpm):
                master, controller, devices = self.make_controller()
                commands = MotionQueue([1], rpm=rpm, acceleration=acceleration)
                stop = threading.Event()
                def callback(event):
                    if event['kind'] == 'continuous_ready':
                        stop.set()
                report = run_continuous(controller, devices, commands, stop, callback)
                self.assertTrue(report['stopped'], report)
                axis = report['axes'][0]
                self.assertEqual(axis['profile_velocity_counts_s'], max(1, round(2**23 * (rpm / 60))))
                self.assertEqual(axis['profile_acceleration_counts_s2'], max(1, round(2**23 * (acceleration / 60))))
                self.assert_clean(master, report, (1,))

    def test_native_overflow_rejected_before_parameter_writes_or_enable(self):
        native_overflow = (2**32 * 60) / 2**23
        for rpm, acceleration in ((native_overflow, 120), (60, native_overflow), (1e308, 120), (60, 1e308)):
            with self.subTest(rpm=rpm, acceleration=acceleration):
                master, controller, devices = self.make_controller()
                writes = []
                for slave in master.slaves:
                    original = slave.sdo_write
                    def record_write(index, sub, data, original=original):
                        writes.append(index)
                        original(index, sub, data)
                    slave.sdo_write = record_write
                report = run_continuous(controller, devices, MotionQueue([1], rpm=rpm, acceleration=acceleration))
                self.assertIn('32 位', report.get('error', ''), report)
                self.assertFalse(report['motion_triggered'])
                self.assertFalse(writes)
                self.assertTrue(master.closed)

    def test_enable_holds_three_axes_and_idle_stays_enabled(self):
        master, controller, devices = self.make_controller()
        origin = [s.position for s in master.slaves]
        commands = MotionQueue([1, 2, 3])
        stop = threading.Event()
        idle_since = None
        completed_positions = []
        def callback(event):
            nonlocal idle_since
            if event['kind'] == 'continuous_ready':
                self.assertEqual([s.position for s in master.slaves], origin)
                self.assertFalse(any(s.targets for s in master.slaves))
                commands.submit([1, 2, 3])
            elif event['kind'] == 'command' and event['stage'] == 'completed':
                completed_positions.append([s.position for s in master.slaves[:3]])
                if event['number'] == 3:
                    idle_since = time.monotonic()
                elif event['number'] == 1:
                    commands.submit([4, 5, 6])
                else:
                    commands.submit([-2, -2, -2], relative=True)
            elif (event['kind'] == 'status' and idle_since and not stop.is_set()
                  and time.monotonic() - idle_since >= .7):
                self.assertTrue(all(a['enabled'] for a in event['axes'][:3]))
                stop.set()
        # Core isolates callback exceptions; collect them explicitly for assertions.
        errors = []
        def checked_callback(event):
            try:
                callback(event)
            except Exception as exc:
                errors.append(exc)
                stop.set()
        report = run_continuous(controller, devices, commands, stop, checked_callback)
        self.assertFalse(errors, errors)
        self.assertTrue(report['stopped'], report)
        self.assertEqual(report['completed_commands'], 3, report)
        self.assertTrue(all(c['completed'] for c in report['commands']))
        self.assert_clean(master, report, (1, 2, 3))
        for index, slave in enumerate(master.slaves[:3]):
            factor = -1 if index == 1 else 1
            values = ([1, 2, 3], [4, 5, 6], [2, 3, 4])
            expected = [(origin[index] + round(2**23 * row[index] * factor / 360) + 2**31)
                        % 2**32 - 2**31 for row in values]
            self.assertEqual([positions[index] for positions in completed_positions], expected)
            self.assertEqual(slave.targets, expected)
            indices = [n for n, word in enumerate(slave.words) if word == 0x3F]
            self.assertTrue(all(word & 0xF == 15 for word in slave.words[indices[0]:indices[-1] + 1]))

    def test_stop_during_first_move_clears_later_commands(self):
        master, controller, devices = self.make_controller()
        commands = MotionQueue([1])
        stop = threading.Event()
        def callback(event):
            if event['kind'] == 'continuous_ready':
                commands.submit([1])
        exchange = master.send_processdata
        def stop_on_trigger():
            exchange()
            if master.slaves[0].targets and not stop.is_set():
                commands.submit([2])
                commands.submit([3])
                stop.set()
        master.send_processdata = stop_on_trigger
        report = run_continuous(controller, devices, commands, stop, callback)
        self.assertTrue(report['stopped'], report)
        self.assertEqual(len(master.slaves[0].targets), 1)
        self.assertEqual(report['completed_commands'], 0)
        self.assertEqual(commands.pending(), 0)
        self.assert_clean(master, report, (1,))

    def test_updates_while_moving_without_waiting_for_arrival(self):
        master, controller, devices = self.make_controller()
        commands = MotionQueue([1])
        self.assertEqual((commands.rpm, commands.acceleration), (60, 120))
        stop = threading.Event()
        origin = master.slaves[0].position
        counts = 2**23
        stride = round(counts / 360)
        slave = master.slaves[0]
        previous_exchange = slave.exchange
        goal = origin
        velocities = []
        trigger_positions = []
        def moving_exchange():
            nonlocal goal
            before, old_count = slave.position, len(slave.targets)
            previous_exchange()
            if len(slave.targets) != old_count:
                goal = slave.position
                trigger_positions.append(displacement(before, origin) * 360 / counts)
            if slave.words[-1] & 0xF == 15:
                delta = displacement(goal, before)
                step = max(-stride, min(stride, delta))
                slave.position = (before + step + 2**31) % 2**32 - 2**31
                if len(slave.targets):
                    velocities.append(step)
                if abs(delta) > stride:
                    slave.sw &= ~0x400
                slave.input = struct.pack('<HHi', 0, slave.sw, slave.position) + bytes(20)
        slave.exchange = moving_exchange
        errors, submitted = [], 1
        send = master.send_processdata
        def exchange_and_update():
            nonlocal submitted
            send()
            travel = displacement(slave.position, origin) * 360 / counts
            if len(slave.targets) == 1 and travel >= 3 and submitted == 1:
                commands.submit([20], relative=True)  # 40-degree target before reaching 20.
                submitted = 2
            elif len(slave.targets) == 2 and travel >= 6 and submitted == 2:
                commands.submit([-50], relative=True)  # Reverse toward -10 while moving.
                submitted = 3
        master.send_processdata = exchange_and_update
        def callback(event):
            try:
                if event['kind'] == 'continuous_ready':
                    commands.submit([20])
                elif event['kind'] == 'command' and event['stage'] == 'completed':
                    self.assertEqual(event['number'], 3)
                    stop.set()
            except Exception as exc:
                errors.append(exc)
                stop.set()
        report = run_continuous(controller, devices, commands, stop, callback)
        self.assertFalse(errors, errors)
        self.assertEqual(report['started_commands'], 3, report)
        self.assertEqual(report['completed_commands'], 1)
        self.assertTrue(all(c['superseded'] for c in report['commands'][:2]))
        self.assertLess(trigger_positions[1], 20)
        self.assertLess(trigger_positions[2], 40)
        self.assertTrue(all(v != 0 for v in velocities[:6]))
        self.assertAlmostEqual(report['axes'][0]['measured_degrees'], -10, places=3)
        indices = [n for n, word in enumerate(slave.words) if word == 0x3F]
        self.assertTrue(all(word & 0xF == 15 for word in slave.words[indices[0]:indices[-1] + 1]))
        self.assertEqual(report['axes'][0]['profile_velocity_counts_s'], counts)
        self.assertEqual(report['axes'][0]['profile_acceleration_counts_s2'], counts * 2)
        self.assert_clean(master, report, (1,))

    def test_fault_and_ack_timeout_disable_and_restore(self):
        for fault in ('wkc', 'limit', 'enable_lost', 'ack_timeout'):
            with self.subTest(fault=fault):
                master, controller, devices = self.make_controller()
                commands = MotionQueue([1])
                origin = master.slaves[0].position
                def callback(event):
                    if event['kind'] == 'continuous_ready':
                        commands.submit([1])
                exchange = master.send_processdata
                def inject_fault():
                    exchange()
                    slave = master.slaves[0]
                    if slave.targets and slave.words[-1] & 0xF == 15:
                        if fault == 'wkc':
                            master.expected_wkc = 0
                        elif fault == 'limit':
                            slave.position = (origin + 2 * 2**23 + 2**31) % 2**32 - 2**31
                        elif fault == 'enable_lost':
                            slave.sw = 0x40
                        else:
                            slave.sw &= ~0x1000
                        slave.input = struct.pack('<HHi', 0, slave.sw, slave.position) + bytes(20)
                    else:
                        master.expected_wkc = 12
                master.send_processdata = inject_fault
                report = run_continuous(controller, devices, commands, callback=callback)
                self.assertFalse(report['success'])
                self.assertIn('error', report)
                self.assertEqual(report['completed_commands'], 0)
                self.assert_clean(master, report, (1,))

    def test_pre_cancel_never_enables(self):
        master, controller, devices = self.make_controller()
        stop = threading.Event()
        stop.set()
        commands = MotionQueue([1, 2, 3])
        report = run_continuous(controller, devices, commands, stop)
        self.assertTrue(report['stopped'])
        self.assertFalse(report['motion_triggered'])
        self.assertFalse(any(s.targets for s in master.slaves))
        self.assertTrue(commands.closed)

    def test_unlimited_travel_and_feedback_across_multiple_encoder_rollovers(self):
        master, controller, devices = self.make_controller()
        commands = MotionQueue([1], rpm=60, acceleration=120)
        stop = threading.Event()
        steps = [3600] * 60 + [-3600] * 120 + [3600] * 60
        feedback = []
        errors = []
        def callback(event):
            try:
                if event['kind'] == 'continuous_ready':
                    commands.submit([steps[0]], relative=True)
                elif event['kind'] == 'command' and event['stage'] == 'completed':
                    count = event['number']
                    if count == len(steps):
                        stop.set()
                    else:
                        commands.submit([steps[count]], relative=True)
                elif event['kind'] == 'status' and event['axes'][0]['travel_degrees'] is not None:
                    feedback.append(event['axes'][0]['travel_degrees'])
            except Exception as exc:
                errors.append(exc)
                stop.set()
        report = run_continuous(controller, devices, commands, stop, callback)
        self.assertFalse(errors, errors)
        self.assertEqual(report['completed_commands'], len(steps), report)
        self.assertGreater(max(feedback), 100000)
        self.assertLess(min(feedback), -100000)
        self.assertAlmostEqual(report['axes'][0]['measured_degrees'], 0)
        self.assert_clean(master, report, (1,))


if __name__ == '__main__':
    unittest.main()
