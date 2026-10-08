"""Local continuous-control tests; all EtherCAT devices are emulated."""
import struct
import threading
import time
import unittest

from core import ControlError, EtherCATController
from continuous_control import MotionQueue, run_continuous
from test_platform_control import FakeMaster


class QueueTests(unittest.TestCase):
    def test_fifo_relative_projection_and_atomic_rejection(self):
        commands = MotionQueue([1, 2, 3], limit_degrees=10)
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
            commands.submit([10, 0, 0], relative=True)
        self.assertEqual(commands.planned, (3, 4, 5))
        self.assertEqual(commands.pending(), 2)
        self.assertEqual(commands.pop(), first)
        self.assertEqual(commands.pop(), second)

    def test_limits_finiteness_capacity_and_closed_session(self):
        commands = MotionQueue([1])
        commands.mark_ready()
        for values in ([float('nan')], [float('inf')], [10**3000], [True], [], [1, 2], [361]):
            with self.assertRaises(ControlError):
                commands.submit(values)
        for _ in range(32):
            commands.submit([0])
        with self.assertRaises(ControlError):
            commands.submit([0])
        commands.close()
        self.assertEqual(commands.pending(), 0)
        with self.assertRaises(ControlError):
            commands.submit([0])
        fresh = MotionQueue([1])
        fresh.mark_ready()
        self.assertEqual(fresh.submit([1], relative=True).targets, (1,))

    def test_command_duration_not_full_limit_controls_timeout(self):
        commands = MotionQueue([1], limit_degrees=3600, rpm=.1)
        commands.mark_ready()
        self.assertEqual(commands.submit([0]).estimated_seconds, 0)
        commands.submit([1])
        with self.assertRaises(ControlError):
            commands.submit([3600])


class SessionTests(unittest.TestCase):
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

    def test_enable_holds_fifo_three_axes_and_idle_stays_enabled(self):
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
                commands.submit([4, 5, 6])
                commands.submit([-2, -2, -2], relative=True)
            elif event['kind'] == 'command' and event['stage'] == 'completed':
                completed_positions.append([s.position for s in master.slaves[:3]])
                if event['number'] == 3:
                    idle_since = time.monotonic()
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
            wanted = [round(2**23 * row[index] * factor / 360) for row in values]
            self.assertEqual(slave.targets, [wanted[0], wanted[1] - wanted[0], wanted[2] - wanted[1]])
            indices = [n for n, word in enumerate(slave.words) if word == 0x5F]
            self.assertTrue(all(word & 0xF == 15 for word in slave.words[indices[0]:indices[-1] + 1]))

    def test_stop_during_first_move_clears_later_commands(self):
        master, controller, devices = self.make_controller()
        commands = MotionQueue([1])
        stop = threading.Event()
        def callback(event):
            if event['kind'] == 'continuous_ready':
                commands.submit([1])
                commands.submit([2])
                commands.submit([3])
        exchange = master.send_processdata
        def stop_on_trigger():
            exchange()
            if master.slaves[0].targets:
                stop.set()
        master.send_processdata = stop_on_trigger
        report = run_continuous(controller, devices, commands, stop, callback)
        self.assertTrue(report['stopped'], report)
        self.assertEqual(len(master.slaves[0].targets), 1)
        self.assertEqual(report['completed_commands'], 0)
        self.assertEqual(commands.pending(), 0)
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


if __name__ == '__main__':
    unittest.main()
