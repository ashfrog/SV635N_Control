"""Shared motor types and exclusivity; no hardware or GUI imports."""
from contextlib import contextmanager
import ctypes
from dataclasses import dataclass
import hashlib
import math
import threading
from collections import deque

DEFAULT_CONTINUOUS_RPM = 60
DEFAULT_CONTINUOUS_ACCELERATION_RPM_S = 120
MAX_MOVE_DEGREES = 3600
MAX_MOVE_SECONDS = 120
LOCAL_LOCK = threading.Lock()

class ControlError(RuntimeError):
    pass


class Stopped(ControlError):
    pass


@dataclass(frozen=True)
class Device:
    order: int
    alias: int
    name: str
    vendor: int
    product: int
    revision: int
    state: int
    statusword: int
    error_code: int
    position: int
    gear_numerator: int
    gear_denominator: int
    motor_code: int = 14101
    positive_direction: int = 1  # H02.02: 0=CCW, 1=CW, viewed from shaft

    @property
    def identity(self):
        return (self.order, self.alias, self.vendor, self.product, self.revision,
                self.motor_code, self.positive_direction)


@dataclass(frozen=True)
class Move:
    """order is physical chain position (1-based); alias 0 is NOT an array index."""
    order: int
    degrees: float = 30.0  # shaft view: positive=CW, negative=CCW
    rpm: float = 5.0
    encoder_bits: int = 23  # user must match the motor nameplate: A3=23, A6=26
    acceleration_rpm_s: float = 10.0  # symmetric acceleration/deceleration

    @property
    def estimated_seconds(self):
        """Rest-to-rest PP duration, including triangular short-distance profiles."""
        distance = abs(self.degrees) / 6
        triangular_peak = math.sqrt(distance) * math.sqrt(self.acceleration_rpm_s)
        if self.rpm >= triangular_peak:
            return 2 * math.sqrt(distance) / math.sqrt(self.acceleration_rpm_s)
        return distance / self.rpm + self.rpm / self.acceleration_rpm_s

    @property
    def peak_rpm(self):
        return min(self.rpm, math.sqrt(abs(self.degrees) / 6) * math.sqrt(self.acceleration_rpm_s))

    def validate(self):
        self.validate_profile()
        if not math.isfinite(self.degrees) or not .1 <= abs(self.degrees) <= MAX_MOVE_DEGREES:
            raise ControlError('角度范围：0.1～3600°，正负表示方向。')
        if self.estimated_seconds > MAX_MOVE_SECONDS:
            raise ControlError('单次预计运动超过 120 秒，请减少角度或提高速度。')

    def validate_profile(self):
        """Speed and acceleration have no software range beyond positive finite values."""
        if isinstance(self.order, bool) or not isinstance(self.order, int) or self.order < 1:
            raise ControlError('电机位置编号无效，请重新扫描。')
        for value, label, unit in ((self.rpm, '速度', 'rpm'),
                                   (self.acceleration_rpm_s, '加减速度', 'rpm/s')):
            try:
                valid = type(value) in (int, float) and math.isfinite(value) and value > 0
            except OverflowError:
                valid = False
            if not valid:
                raise ControlError(f'{label}必须为大于 0 的有限数值（{unit}）。')
        if self.encoder_bits != 23:
            raise ControlError('当前版本只接受已核对的 A3 / 23位电机，其他编码器须另行核对。')



@contextmanager
def adapter_lock(adapter):
    """Exclude simultaneous sessions in this program, including other EXE instances."""
    if not LOCAL_LOCK.acquire(False):
        raise ControlError('正在扫描或运行，请等待或按停止。')
    kernel = None
    handle = None
    try:
        import os
        if os.name == 'nt':
            from ctypes import wintypes
            kernel = ctypes.WinDLL('kernel32', use_last_error=True)
            kernel.CreateMutexW.argtypes = (ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR)
            kernel.CreateMutexW.restype = wintypes.HANDLE
            kernel.WaitForSingleObject.argtypes = (wintypes.HANDLE, wintypes.DWORD)
            kernel.ReleaseMutex.argtypes = (wintypes.HANDLE,)
            kernel.CloseHandle.argtypes = (wintypes.HANDLE,)
            name = 'Local\\SV635N_' + hashlib.sha256(adapter.encode()).hexdigest()[:24]
            handle = kernel.CreateMutexW(None, False, name)
            if not handle:
                raise ControlError('无法创建硬件访问锁。')
            if kernel.WaitForSingleObject(handle, 0) not in (0, 0x80):
                kernel.CloseHandle(handle)
                handle = None
                raise ControlError('另一个本程序窗口正在使用此控制硬件。')
        yield
    finally:
        if handle:
            kernel.ReleaseMutex(handle)
            kernel.CloseHandle(handle)
        LOCAL_LOCK.release()


@dataclass(frozen=True)
class MotionCommand:
    number: int
    targets: tuple[float, ...]
    estimated_seconds: float


class MotionQueue:
    def __init__(self, orders, rpm=DEFAULT_CONTINUOUS_RPM, acceleration=DEFAULT_CONTINUOUS_ACCELERATION_RPM_S):
        self.orders = tuple(orders)
        if (not self.orders or len(set(self.orders)) != len(self.orders)
                or self.orders != tuple(sorted(self.orders))):
            raise ControlError('请选择不同的电机，按链路位置升序排列。')
        for order in self.orders:
            Move(order, .1, rpm, acceleration_rpm_s=acceleration).validate_profile()
        self.rpm, self.acceleration = rpm, acceleration
        self.lock = threading.Lock()
        self.commands = deque()
        self.planned = (0.0,) * len(self.orders)
        self.number = 0
        self.ready = self.closed = False

    def mark_ready(self):
        with self.lock:
            if self.closed:
                raise Stopped('连续控制已关闭。')
            self.ready = True

    def close(self):
        with self.lock:
            self.ready = False
            self.closed = True
            self.commands.clear()

    def submit(self, values, relative=False):
        values = tuple(values)
        try:
            valid = len(values) == len(self.orders) and all(
                type(x) in (int, float) and math.isfinite(x) for x in values)
        except OverflowError:
            valid = False
        if not valid:
            raise ControlError('请输入每个选中轴的有限角度数值。')
        with self.lock:
            if not self.ready or self.closed:
                raise ControlError('尚未完成使能，或正在关闭使能。')
            targets = tuple(old + change for old, change in zip(self.planned, values)) if relative else values
            if any(not math.isfinite(x) for x in targets):
                raise ControlError('累计目标数值无法表示，未更新目标。')
            duration = max(Move(n, target - old, self.rpm,
                                acceleration_rpm_s=self.acceleration).estimated_seconds
                           for n, target, old in zip(self.orders, targets, self.planned))
            if duration > MAX_MOVE_SECONDS:
                raise ControlError('本条指令预计运动超过 120 秒，请减少位移。')
            self.number += 1
            command = MotionCommand(self.number, targets, duration)
            self.commands.clear()  # New targets replace pending frames; relative intent still accumulates.
            self.commands.append(command)
            self.planned = targets
            return command

    def pop(self):
        with self.lock:
            return self.commands.popleft() if self.commands else None

    def pending(self):
        with self.lock:
            return len(self.commands)

