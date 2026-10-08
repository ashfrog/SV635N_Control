"""UI-independent SV635N EtherCAT PP control. All hardware calls run on one caller thread.

scan() never enables. execute(..., dry_run=True) verifies OP without enabling.
execute(..., dry_run=False) performs one bounded relative move on selected axes.
The threading.Event stop token may be set from any thread; callbacks must not block.
"""
from __future__ import annotations

from contextlib import contextmanager
import ctypes
from dataclasses import asdict, dataclass
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import threading
import time
from typing import Callable
import pysoem

DEFAULT_ADAPTER = r'\Device\NPF_{89B39964-A086-4A5D-BC55-D5804AE703C6}'
RX_MAP = [0x60400010, 0x607A0020, 0x60B80010, 0x60FE0120]
TX_MAP = [0x603F0010, 0x60410010, 0x60640020, 0x60770010,
          0x60F40020, 0x60B90010, 0x60BA0020, 0x60BC0020, 0x60FD0020]
LOCAL_LOCK = threading.Lock()
DEFAULT_CONTINUOUS_RPM = 60
DEFAULT_CONTINUOUS_ACCELERATION_RPM_S = 120
MAX_MOVE_DEGREES = 3600
MAX_MOVE_SECONDS = 120
PDO_MAX_FAILED_EXCHANGES = 5
PDO_RECOVERY_SECONDS = .010


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


def read(slave, index, sub=0, signed=False):
    for attempt in range(3):
        try:
            data = slave.sdo_read(index, sub)
            if not data:
                raise ControlError(f'SDO {index:04X}:{sub:02X} 返回空数据。')
            return int.from_bytes(data, 'little', signed=signed)
        except pysoem.SdoError:
            raise
        except Exception:
            if attempt == 2:
                raise
            time.sleep(.02)


def mapping(slave, assignment):
    result = []
    for sub in range(1, read(slave, assignment) + 1):
        pdo = read(slave, assignment, sub)
        result.extend(read(slave, pdo, entry) for entry in range(1, read(slave, pdo) + 1))
    return result


def displacement(actual, origin):
    return (actual - origin + 2**31) % 2**32 - 2**31


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
                raise ControlError('无法创建网卡访问锁。')
            if kernel.WaitForSingleObject(handle, 0) not in (0, 0x80):
                kernel.CloseHandle(handle)
                handle = None
                raise ControlError('另一个本程序窗口正在使用此网卡。')
        yield
    finally:
        if handle:
            kernel.ReleaseMutex(handle)
            kernel.CloseHandle(handle)
        LOCAL_LOCK.release()


class EtherCATController:
    """Public API; no Tk/GUI dependency. Each operation opens then closes the master."""
    def __init__(self, adapter=DEFAULT_ADAPTER, log_dir=None, master_factory=None):
        self.adapter = adapter
        self.log_dir = Path(log_dir) if log_dir else None
        self.master_factory = master_factory or pysoem.Master

    @staticmethod
    def adapters():
        return [(a.name, a.desc.decode(errors='replace') if isinstance(a.desc, bytes) else a.desc)
                for a in pysoem.find_adapters()]

    def _discover(self, master):
        count = master.config_init()
        if not 1 <= count <= 16:
            raise ControlError(f'扫描到 {count} 台设备；支持 1～16 台 SV635N。请检查网卡和 IN/OUT 接线。')
        master.state_check(pysoem.PREOP_STATE, 2_000_000)
        master.read_state()
        devices = []
        for order, s in enumerate(master.slaves, 1):
            if (s.man, s.id, s.rev) != (0x100000, 0xC010E, 0x10000):
                raise ControlError(f'链路位置 {order} 的设备型号/版本尚未支持，禁止运行。')
            if s.state != pysoem.PREOP_STATE:
                raise ControlError(f'位置 {order} 未进入 PRE-OP，AL=0x{s.al_status:04X}。')
            sw = read(s, 0x6041)
            if sw & 4:
                raise ControlError(f'位置 {order} 已处于使能状态，禁止接管。')
            devices.append(Device(order, read(s, 0x200E, 0x16), s.name, s.man, s.id, s.rev,
                                  s.state, sw, read(s, 0x603F), read(s, 0x6064, signed=True),
                                  read(s, 0x6091, 1), read(s, 0x6091, 2),
                                  read(s, 0x2000, 1), read(s, 0x2002, 3)))
        return devices

    def scan(self):
        with adapter_lock(self.adapter):
            m = self.master_factory()
            opened = False
            try:
                m.open(self.adapter)
                opened = True
                return self._discover(m)
            finally:
                if opened:
                    m.close()

    def execute(self, moves: list[Move], devices: list[Device], stop=None,
                callback: Callable | None = None, dry_run=False):
        if not moves:
            raise ControlError('请至少勾选一台电机。')
        for move in moves:
            move.validate()
        if len({p.order for p in moves}) != len(moves):
            raise ControlError('同一电机不能有重复指令。')
        known = {d.order for d in devices}
        if not {p.order for p in moves} <= known:
            raise ControlError('所选电机不在扫描结果中，请重新扫描。')
        with adapter_lock(self.adapter):
            return _Session(self, moves, devices, stop or threading.Event(),
                            callback or (lambda event: None), dry_run).run()


class _Session:
    def __init__(self, controller, moves, snapshot, stop, callback, dry_run):
        self.controller = controller
        self.master = controller.master_factory()
        self.snapshot = snapshot
        self.requests = {p.order: p for p in moves}
        self.stop = stop
        self.callback = callback
        self.dry_run = dry_run
        self.axes = []
        self.opened = self.mapped = self.timer = self.armed = self.op_required = False
        self.expected = 0
        self.deadline = time.perf_counter_ns()
        self.started = time.perf_counter()
        self.last_state = self.last_publish = 0.0
        self.trace = []
        self.report = {'time': datetime.now().astimezone().isoformat(),
                       'adapter': controller.adapter, 'dry_run': dry_run,
                       'requests': [asdict(p) for p in moves], 'success': False,
                       'stopped': False, 'motion_triggered': False, 'axes': []}
        if controller.log_dir:
            controller.log_dir.mkdir(parents=True, exist_ok=True)
            self.path = controller.log_dir / ('run_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.json')
        else:
            self.path = None

    def emit(self, kind, **payload):
        try:
            self.callback({'kind': kind, **payload})
        except Exception:
            pass  # A display/log callback must never interrupt stop/disable.

    def journal(self):
        if self.path:
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(self.report, ensure_ascii=False, indent=2), encoding='utf-8')
            temporary.replace(self.path)

    def write(self, a, index, size, value, sub=0, signed=False):
        a['slave'].sdo_write(index, sub, value.to_bytes(size, 'little', signed=signed))
        if read(a['slave'], index, sub, signed) != value:
            raise ControlError(f"ID {a['device'].alias}：参数 {index:04X}:{sub:02X} 写入核对失败。")

    def check_stop(self):
        if self.stop.is_set():
            raise Stopped('已请求停止。')

    def setup(self):
        self.check_stop()
        self.emit('phase', text='核对设备及参数')
        self.master.open(self.controller.adapter)
        self.opened = True
        devices = self.controller._discover(self.master)
        if [d.identity for d in devices] != [d.identity for d in self.snapshot]:
            raise ControlError('链路设备或 ID 已改变，请重新扫描后选择电机。')
        aliases = [d.alias for d in devices]
        if len(set(aliases)) != len(aliases):
            raise ControlError('存在重复 H0E.21，请设置唯一 ID 后重新扫描。')
        self.axes = [{'slave': s, 'device': d, 'request': self.requests.get(d.order),
                      'cw': 0, 'target': d.position, 'last': None, 'modified': False,
                      'report': {'order': d.order, 'id': d.alias, 'selected': d.order in self.requests}}
                     for d, s in zip(devices, self.master.slaves)]
        self.report['axes'] = [a['report'] for a in self.axes]
        # Read and journal ALL original parameters before the first write.
        for a in self.axes:
            d, s = a['device'], a['slave']
            self.check_stop()
            if d.error_code or d.statusword & 8:
                raise ControlError(f'ID {d.alias} 有报警 0x{d.error_code:04X}；请先处理报警，本程序不自动复位。')
            if mapping(s, 0x1C12) != RX_MAP or mapping(s, 0x1C13) != TX_MAP:
                raise ControlError(f'ID {d.alias} 的 PDO 配置不受支持。')
            if read(s, 0x2002, 1) != 9 or read(s, 0x200E, 0x20) != 1:
                raise ControlError(f'ID {d.alias} 必须使用 EtherCAT 控制和 DC 同步。')
            if a['request']:
                p = a['request']
                if d.motor_code != 14101:
                    raise ControlError(f'ID {d.alias}：H00.00={d.motor_code}，与本版本验证的 A3 电机设置 14101 不符。'
                                       '请按电机铭牌核对并修正驱动器参数，重新上电后再扫描；禁止按共享分辨率运行。')
                if d.positive_direction not in (0, 1):
                    raise ControlError(f'ID {d.alias}：H02.02 方向参数无效。')
                a['direction_factor'] = 1 if d.positive_direction == 1 else -1
                # This firmware returned 6502=2 after reading 6091:02. Reading
                # 6041 first restores the supported-mode response (941), observed
                # on this hardware. Never bypass the actual PP capability check.
                read(s, 0x6041)
                modes, quick_stop = read(s, 0x6502), read(s, 0x605A)
                if not modes & 1 or quick_stop != 2:
                    raise ControlError(f'ID {d.alias} 必须支持 PP 模式且快速停止方式 605A=2（读取：6502={modes:#x}，605A={quick_stop}）。')
                if d.gear_numerator <= 0 or d.gear_denominator <= 0:
                    raise ControlError(f'ID {d.alias} 齿轮比无效。')
                counts = (2**p.encoder_bits) * d.gear_denominator / d.gear_numerator
                a.update(counts=counts, delta=round(counts * p.degrees * a['direction_factor'] / 360),
                         tolerance=max(2, round(counts * .2 / 360)))
                if not 1 <= abs(a['delta']) < 2**31 or counts <= 0:
                    raise ControlError(f'ID {d.alias} 位移超出可用指令范围。')
                raw_velocity = counts * (p.rpm / 60)
                raw_accel = counts * (p.acceleration_rpm_s / 60)
                if not math.isfinite(raw_velocity) or not math.isfinite(raw_accel):
                    raise ControlError(f'ID {d.alias} 速度/加速度超出驱动器 32 位参数范围。')
                velocity = max(1, round(raw_velocity))
                accel = max(1, round(raw_accel))
                if max(velocity, accel) > 0xFFFFFFFF:
                    raise ControlError(f'ID {d.alias} 速度/加速度超出驱动器 32 位参数范围。')
                a['changes'] = [(0x6081, 4, velocity, False), (0x6083, 4, accel, False),
                                (0x6084, 4, accel, False), (0x6085, 4, accel, False),
                                (0x6060, 1, 1, True)]
                a['saved'] = [(idx, size, read(s, idx, signed=signed), signed)
                              for idx, size, _, signed in a['changes']]
                a['policy'] = read(s, 0x200E, 2)
                a['report'].update(original_parameters=a['saved'], original_save_policy=a['policy'],
                                   counts_per_rev=counts, delta_counts=a['delta'],
                                   motor_code=d.motor_code, positive_direction=d.positive_direction,
                                   direction_factor=a['direction_factor'],
                                   estimated_seconds=p.estimated_seconds, peak_rpm=p.peak_rpm,
                                   profile_velocity_counts_s=velocity, profile_acceleration_counts_s2=accel)
        self.journal()
        for a in self.axes:
            if not a['request']:
                continue
            self.check_stop()
            a['modified'] = True  # mark BEFORE write; handles uncertain write outcomes
            self.write(a, 0x200E, 2, 0, sub=2)
            for idx, size, value, signed in a['changes']:
                self.check_stop()
                self.write(a, idx, size, value, signed=signed)
            end = time.perf_counter() + 1
            while read(a['slave'], 0x6061, signed=True) != 1:
                self.check_stop()
                if time.perf_counter() > end:
                    raise ControlError(f"ID {a['device'].alias} 未接受 PP 模式。")
                time.sleep(.02)
        self.check_stop()
        self.emit('phase', text='验证全部设备 DC / OP 通信，保持未使能')
        self.master.manual_state_change = True
        self.master.config_map()
        self.mapped = True
        for a in self.axes:
            s = a['slave']
            if len(s.output) != 12 or len(s.input) != 28:
                raise ControlError('PDO 字节长度不匹配。')
            s.output = struct.pack('<HiHI', 0, a['target'], 0, 0)
            control = s._fprd(0x0814, 1)[0]
            divider = int.from_bytes(s._fprd(0x0400, 2), 'little')
            count = int.from_bytes(s._fprd(0x0420, 2), 'little')
            ms = (divider + 2) * 40 * count / 1_000_000
            a['report']['watchdog_ms'] = ms
            if not control & 0x40 or not 0 < ms <= 200:
                raise ControlError(f"ID {a['device'].alias} 的输出看门狗未通过验证。")
        if not self.master.config_dc():
            raise ControlError('未检测到 DC 时钟。')
        for a in self.axes:
            a['slave'].dc_sync(True, 1_000_000)
            if a['slave']._fprd(0x0981, 1)[0] != 3:
                raise ControlError('DC 同步激活核对失败。')
        self.master.state = pysoem.SAFEOP_STATE
        self.master.write_state()
        self.master.state_check(pysoem.SAFEOP_STATE, 2_000_000)
        self.master.read_state()
        if any(a['slave'].state != 4 for a in self.axes):
            raise ControlError('部分设备未进入 SAFE-OP。')
        self.expected = self.master.expected_wkc
        try:
            self.timer = ctypes.windll.winmm.timeBeginPeriod(1) == 0
        except AttributeError:
            pass
        self.deadline = time.perf_counter_ns()
        self.hold(.5, checked=False, cancellable=True)
        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        end = time.perf_counter() + 3
        while time.perf_counter() < end:
            self.tick(checked=False, cancellable=True)
            self.master.read_state()
            if all(a['slave'].state == 8 for a in self.axes):
                break
        if any(a['slave'].state != 8 for a in self.axes):
            details = [(a['device'].alias, a['slave'].state, a['slave'].al_status) for a in self.axes]
            raise ControlError(f'OP 验证失败（ID、状态、AL）：{details}')
        self.op_required = True
        self.hold(.5)
        if any(a['last'][1] & 4 for a in self.axes):
            raise ControlError('准备期间检测到意外使能。')
        self.report['op_verified'] = True

    def update_feedback(self, a):
        """Hook for sessions that track encoder rollover across many moves."""

    def travel_degrees(self, a):
        return displacement(a['last'][2], a['origin']) * 360 / a['counts'] * a['direction_factor']

    def check_travel(self, a):
        travel = displacement(a['last'][2], a['origin'])
        bound = abs(a['delta']) + max(a['tolerance'] * 5, round(a['counts'] * 5 / 360))
        if abs(travel) > bound:
            raise ControlError(f"ID {a['device'].alias} 超出位移边界。")

    def tick(self, checked=True, cancellable=True):
        if cancellable:
            self.check_stop()  # checked immediately BEFORE staging enable/setpoint
        now = time.perf_counter_ns()
        late = max(0, now - self.deadline) / 1_000_000
        if checked and self.armed and late > 20:
            raise ControlError(f'主机周期延迟 {late:.1f} ms，停止全部电机。')
        for a in self.axes:
            a['slave'].output = struct.pack('<HiHI', a['cw'], a['target'], 0, 0)
        exchange_started = time.perf_counter()
        failures = 0
        while True:
            if cancellable:
                self.check_stop()
            # Resend the same controlword/target: no new setpoint edge, and no
            # handshake or target updates until a complete feedback frame arrives.
            self.master.send_processdata()
            if any(a['request'] and a['cw'] & 0x18 == 0x18 for a in self.axes):
                self.report['motion_triggered'] = True
            wkc = self.master.receive_processdata(2_000)
            t = time.perf_counter() - self.started
            row = [t, wkc, late]
            for a in self.axes:
                feedback = struct.unpack_from('<HHi', a['slave'].input)
                row.extend([a['cw'], a['target'], *feedback])
                if wkc == self.expected or not checked:
                    a['last'] = feedback
                if wkc == self.expected:
                    self.update_feedback(a)
            self.trace.append(row)
            self.last_wkc = wkc
            if not checked:
                break
            elapsed = time.perf_counter() - exchange_started
            if wkc != self.expected:
                failures += 1
                self.report['pdo_failed_exchanges'] = self.report.get('pdo_failed_exchanges', 0) + 1
                if failures >= PDO_MAX_FAILED_EXCHANGES or elapsed >= PDO_RECOVERY_SECONDS:
                    raise ControlError(f'通信 WKC={wkc}，期望 {self.expected}；'
                                       f'连续 {failures} 次异常，停止全部电机。')
                continue
            if failures:
                if elapsed >= PDO_RECOVERY_SECONDS:
                    raise ControlError('通信恢复超过 10 ms，停止全部电机。')
                self.report['pdo_recoveries'] = self.report.get('pdo_recoveries', 0) + 1
                self.report['maximum_pdo_recovery_ms'] = max(
                    self.report.get('maximum_pdo_recovery_ms', 0), elapsed * 1000)
                self.emit('phase', text=f'通信短时异常已恢复（重试 {failures} 次），继续运行')
            break
        if checked:
            for a in self.axes:
                error, sw, actual = a['last']
                label = f"ID {a['device'].alias}"
                if error or sw & 8:
                    raise ControlError(f'{label} 报警 0x{error:04X}。')
                if a['request'] and sw & 0x800:
                    raise ControlError(f'{label} 内部限制生效。')
                if not a['request'] and sw & 4:
                    raise ControlError(f'{label} 未选中却处于使能状态。')
                if self.armed and a['request']:
                    self.check_travel(a)
            if self.op_required and t - self.last_state >= .05:
                self.master.read_state()
                self.last_state = t
                if any(a['slave'].state != 8 for a in self.axes):
                    raise ControlError('设备退出 OP，停止全部电机。')
        if t - self.last_publish >= .1:
            self.last_publish = t
            self.emit('status', wkc=wkc, expected_wkc=self.expected,
                      axes=[{'order': a['device'].order, 'position': a['last'][2],
                             'enabled': bool(a['last'][1] & 4), 'error_code': a['last'][0],
                             'travel_degrees': self.travel_degrees(a)
                             if 'origin' in a else None} for a in self.axes])
        self.deadline += 1_000_000
        remaining = self.deadline - time.perf_counter_ns()
        if remaining > 250_000:
            time.sleep((remaining - 150_000) / 1_000_000_000)
        while time.perf_counter_ns() < self.deadline:
            pass
        if time.perf_counter_ns() - self.deadline > 1_000_000:
            self.deadline = time.perf_counter_ns()

    def selected(self):
        return [a for a in self.axes if a['request']]

    def wait_all(self, condition, timeout, label):
        end = time.perf_counter() + timeout
        while time.perf_counter() < end:
            self.tick()
            if all(condition(a) for a in self.selected()):
                return
        raise ControlError(f'等待超时：{label}')

    def hold(self, seconds, checked=True, cancellable=True):
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self.tick(checked, cancellable)

    def command(self, word):
        for a in self.selected():
            a['cw'] = word

    def motion(self):
        self.check_stop()
        for a in self.selected():
            a['origin'] = a['last'][2]
            a['target'] = a['origin']
        self.armed = True
        self.emit('phase', text='选中电机使能，未选中电机保持关闭')
        for cw, state in [(6, 0x21), (7, 0x23), (15, 0x27)]:
            self.command(cw)
            self.wait_all(lambda a: a['last'][1] & 0x6F == state, 1, f'使能状态 {state:#x}')
        self.wait_all(lambda a: not a['last'][1] & 0x1000, 1, '清除旧位置确认')
        self.hold(.1)
        for a in self.selected():
            a['motion_origin'] = a['last'][2]
            if abs(displacement(a['motion_origin'], a['origin'])) > a['tolerance']:
                raise ControlError('使能期间出现意外位移。')
            a['report']['motion_start_position'] = a['motion_origin']
            a['target'] = a['delta']
        self.command(0x5F)  # relative, single point, bit4 rising on common exchange
        self.check_stop()
        self.report['trigger_trace_row'] = len(self.trace)
        # No journal/SDO/file I/O while enabled; keep the PDO exchange uninterrupted.
        self.tick()
        self.report['motion_triggered'] = True
        self.emit('phase', text='运行中；完成后自动关闭使能')
        self.wait_all(lambda a: bool(a['last'][1] & 0x1000), 1, '新位置确认')
        self.command(0x4F)
        self.wait_all(lambda a: not a['last'][1] & 0x1000, 1, '清除位置确认')
        timeout = max(a['request'].estimated_seconds for a in self.selected()) + 5
        self.wait_all(lambda a: bool(a['last'][1] & 0x400) and
                      abs(displacement(a['last'][2], a['motion_origin']) - a['delta']) <= a['tolerance'],
                      timeout, '到达目标')
        self.hold(.1)
        for a in self.selected():
            a['report']['measured_degrees'] = displacement(a['last'][2], a['motion_origin']) * 360 / a['counts'] * a['direction_factor']
        self.command(0)
        self.wait_all(lambda a: not a['last'][1] & 4, 1, '关闭使能')
        self.armed = False
        self.report['success'] = True

    def cleanup(self):
        if not self.opened:
            return
        errors = []
        self.emit('phase', text='停止、关闭使能并恢复临时参数')
        try:
            if self.mapped:
                self.op_required = False
                if self.armed:
                    self.command(2)
                    self.hold(max((a['report']['profile_velocity_counts_s'] /
                                   a['report']['profile_acceleration_counts_s2'] for a in self.selected()), default=0) + .3,
                              checked=False, cancellable=False)
                for a in self.axes:
                    a['cw'] = 0
                self.hold(.2, checked=False, cancellable=False)
                self.report['all_disabled'] = self.last_wkc == self.expected and all(not a['last'][1] & 4 for a in self.axes)
                if not self.report['all_disabled']:
                    errors.append('未通过 PDO 确认全部关闭使能；请检查驱动器，必要时断电。')
            if self.mapped:
                self.master.state = pysoem.SAFEOP_STATE
                self.master.write_state()
                self.master.state_check(pysoem.SAFEOP_STATE, 500_000)
            self.master.read_state()
            for a in self.axes:
                if a['slave'].state & pysoem.STATE_ERROR:
                    a['slave'].state = (a['slave'].state & 0xF) | pysoem.STATE_ACK
                    a['slave'].write_state()
            self.master.state = pysoem.PREOP_STATE
            self.master.write_state()
            self.master.state_check(pysoem.PREOP_STATE, 2_000_000)
            self.master.read_state()
        except Exception as exc:
            errors.append(f'通信清理：{exc}')
        for a in self.axes:
            try:
                s = a['slave']
                sw = read(s, 0x6041)
                a['report'].update(final_statusword=sw, final_error_code=read(s, 0x603F), final_state=s.state)
                if sw & 4 or s.state != 2:
                    raise ControlError('未确认关闭使能并退回 PRE-OP，请检查驱动器。')
                if self.mapped:
                    s.dc_sync(False, 1_000_000)
                if a['modified']:
                    # Keep EEPROM disabled throughout restoration; policy is last.
                    for idx, size, value, signed in reversed(a['saved']):
                        self.write(a, idx, size, value, signed=signed)
                    self.write(a, 0x200E, 2, a['policy'], sub=2)
                    a['report']['parameters_restored'] = True
            except Exception as exc:
                a['report']['cleanup_error'] = str(exc)
                errors.append(f"ID {a['device'].alias}：{exc}")
        try:
            self.master.close()
        finally:
            if self.timer:
                ctypes.windll.winmm.timeEndPeriod(1)
        if errors:
            self.report['cleanup_errors'] = errors
            self.report['success'] = False

    def run(self):
        try:
            self.setup()
            if self.dry_run:
                self.report['success'] = True
            else:
                self.motion()
        except Stopped as exc:
            self.report.update(stopped=True, error=str(exc))
        except Exception as exc:
            self.report['error'] = f'{type(exc).__name__}: {exc}'
        finally:
            self.cleanup()
        if self.trace:
            self.report['maximum_host_lateness_ms'] = max(row[2] for row in self.trace)
        if self.path:
            try:
                import csv
                with self.path.with_suffix('.csv').open('w', newline='', encoding='utf-8') as f:
                    writer = csv.writer(f)
                    names = ['seconds', 'wkc', 'host_lateness_ms']
                    for a in self.axes:
                        names += [f"order{a['device'].order}_{field}" for field in
                                  ['controlword', 'target', 'error', 'statusword', 'position']]
                    writer.writerow(names)
                    writer.writerows(self.trace)
                self.report['log_path'] = str(self.path)
                self.journal()
            except Exception as exc:
                self.report['log_error'] = str(exc)
        return self.report
