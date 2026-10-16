"""DC timing and GIL scheduling tests; no bus or motion is used."""
import sys
import ctypes
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from ethercat_timing import DCPhaseClock, cyclic_worker, windows_cyclic_policy, _PowerThrottling
from core import EtherCATController
from test_platform_control import FakeMaster


class DCTimingTests(unittest.TestCase):
    def test_idle_discovery_enables_native_gil_release_before_scan_and_sdo(self):
        master=FakeMaster()
        master.always_release_gil=False
        initialize=master.config_init
        def checked_initialize():
            self.assertTrue(master.always_release_gil)
            return initialize()
        master.config_init=checked_initialize
        for slave in master.slaves:
            original=slave.sdo_read
            def checked_read(index,sub,original=original):
                self.assertTrue(master.always_release_gil)
                return original(index,sub)
            slave.sdo_read=checked_read
        devices=EtherCATController()._discover(master)
        self.assertEqual(len(devices),4)

    def test_phase_correction_direction_and_wrapping(self):
        self.assertEqual(DCPhaseClock().correction(0), 0)
        self.assertEqual(DCPhaseClock().correction(50_000), 0)
        self.assertLess(DCPhaseClock().correction(150_000), 0)
        self.assertGreater(DCPhaseClock().correction(950_000), 0)

    def test_phase_loop_converges_and_correction_is_bounded(self):
        clock = DCPhaseClock()
        error = 300_000
        for _ in range(300):
            adjustment = clock.correction(50_000+error)
            self.assertLessEqual(abs(adjustment), 50_000)
            error += adjustment
        self.assertLess(abs(error), 30_000)

    def test_worker_releases_native_gil_and_restores_settings_after_failure(self):
        master = SimpleNamespace(always_release_gil=False)
        report = {}
        previous = sys.getswitchinterval()
        with self.assertRaisesRegex(RuntimeError, 'test failure'):
            with cyclic_worker(master, report):
                self.assertTrue(master.always_release_gil)
                self.assertLessEqual(sys.getswitchinterval(), .001)
                raise RuntimeError('test failure')
        self.assertFalse(master.always_release_gil)
        self.assertEqual(sys.getswitchinterval(), previous)

    def test_background_policy_disables_only_timer_and_ecoqos_then_restores_on_failure(self):
        kernel = Mock()
        kernel.GetThreadPriority.return_value = 0
        calls = []
        def query(process, info, pointer, size):
            state = ctypes.cast(pointer, ctypes.POINTER(_PowerThrottling)).contents
            state.ControlMask, state.StateMask = 0x9, 0x9
            return True
        def update(process, info, pointer, size):
            state = ctypes.cast(pointer, ctypes.POINTER(_PowerThrottling)).contents
            calls.append((state.Version,state.ControlMask,state.StateMask))
            return True
        kernel.GetProcessInformation.side_effect = query
        kernel.SetProcessInformation.side_effect = update
        report = {}
        with self.assertRaisesRegex(RuntimeError, 'failure'):
            with windows_cyclic_policy(report, kernel):
                self.assertTrue(report['background_timer_opt_out'])
                self.assertEqual(calls, [(1,0xD,0x8)])
                raise RuntimeError('failure')
        self.assertEqual(calls, [(1,0xD,0x8),(1,0x9,0x9)])
        self.assertEqual([c.args[1] for c in kernel.SetThreadPriority.call_args_list], [2,0])

    def test_unsupported_power_policy_is_reported_and_not_restored_as_success(self):
        kernel = Mock()
        kernel.GetProcessInformation.return_value = False
        kernel.GetThreadPriority.return_value = 2
        report = {}
        with windows_cyclic_policy(report, kernel):
            self.assertFalse(report['background_timer_opt_out'])
        kernel.SetProcessInformation.assert_not_called()
        kernel.SetThreadPriority.assert_not_called()


if __name__ == '__main__':
    unittest.main()
