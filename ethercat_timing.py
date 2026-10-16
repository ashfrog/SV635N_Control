"""Bounded SOEM DC phase correction and scoped Windows worker scheduling."""
from contextlib import contextmanager
import ctypes
import sys


class _PowerThrottling(ctypes.Structure):
    _fields_ = [('Version', ctypes.c_uint32), ('ControlMask', ctypes.c_uint32), ('StateMask', ctypes.c_uint32)]


@contextmanager
def windows_cyclic_policy(report, kernel=None):
    """Keep tray/background operation eligible for Windows high-resolution timers."""
    kernel = kernel or ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetCurrentProcess.restype = ctypes.c_void_p
    kernel.GetCurrentThread.restype = ctypes.c_void_p
    kernel.GetThreadPriority.argtypes = (ctypes.c_void_p,)
    kernel.SetThreadPriority.argtypes = (ctypes.c_void_p, ctypes.c_int)
    signature = (ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_uint32)
    kernel.GetProcessInformation.argtypes = kernel.SetProcessInformation.argtypes = signature
    process, thread = kernel.GetCurrentProcess(), kernel.GetCurrentThread()
    previous_power = _PowerThrottling(1, 0, 0)
    restore_power = restore_priority = False
    previous_priority = kernel.GetThreadPriority(thread)
    try:
        # SetProcessInformation / ProcessPowerThrottling: explicitly disable
        # EcoQoS and IGNORE_TIMER_RESOLUTION; retain all unrelated policies.
        if kernel.GetProcessInformation(process, 4, ctypes.byref(previous_power), ctypes.sizeof(previous_power)):
            requested = _PowerThrottling(1, previous_power.ControlMask | 0x5, previous_power.StateMask & ~0x5)
            restore_power = bool(kernel.SetProcessInformation(process, 4, ctypes.byref(requested), ctypes.sizeof(requested)))
        report['background_timer_opt_out'] = restore_power
        if previous_priority != 0x7FFFFFFF and previous_priority < 2:
            restore_priority = bool(kernel.SetThreadPriority(thread, 2))
        if restore_priority:
            report['thread_priority'] = 'highest'
        yield
    finally:
        if restore_priority:
            kernel.SetThreadPriority(thread, previous_priority)
        if restore_power:
            kernel.SetProcessInformation(process, 4, ctypes.byref(previous_power), ctypes.sizeof(previous_power))


class DCPhaseClock:
    def __init__(self, cycle_ns=1_000_000):
        self.cycle = cycle_ns
        self.integral = 0
        self.phase_error = 0

    def correction(self, dc_time):
        if not dc_time:
            return 0
        # SOEM red_test ec_sync: aim 50 us after SYNC0, with PI correction.
        error = (dc_time-50_000) % self.cycle
        if error > self.cycle//2:
            error -= self.cycle
        self.phase_error = error
        self.integral = max(-100_000, min(100_000, self.integral+(1 if error > 0 else -1 if error < 0 else 0)))
        return max(-50_000, min(50_000, -int(error/100)-int(self.integral/20)))


@contextmanager
def cyclic_worker(master, report):
    previous_interval = sys.getswitchinterval()
    release_gil = getattr(master, 'always_release_gil', None)
    try:
        # A blocking native receive/state call must not freeze UDP heartbeats.
        if release_gil is not None:
            master.always_release_gil = True
        sys.setswitchinterval(min(previous_interval, .0005))
        report['cyclic_scheduling'] = dict(release_gil=release_gil is not None,
                                           python_switch_interval_s=sys.getswitchinterval())
        if sys.platform == 'win32':
            with windows_cyclic_policy(report['cyclic_scheduling']):
                yield
        else:
            yield
    finally:
        sys.setswitchinterval(previous_interval)
        if release_gil is not None:
            master.always_release_gil = release_gil
