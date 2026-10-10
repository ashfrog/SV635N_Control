"""ADLINK PCIe-8332 hardware owner. APS calls run only on the motor worker.

ABI/constants follow the installed APS SDK headers and APS FunctionLibrary V2.1.
Synchronous API mode waits for the card's acknowledgement, not motion completion.
The card generates EtherCAT trajectories; Python does not transmit PDO cycles.
"""
import ctypes as C
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from functools import cached_property
import json
import logging
import math
import os
from pathlib import Path
import threading
import time

from control_common import ControlError, Stopped, Device, adapter_lock

LOG = logging.getLogger(__name__)
I32, U32, U16, U8, F64 = C.c_int32, C.c_uint32, C.c_uint16, C.c_uint8, C.c_double
PI32, PU32, PF64 = C.POINTER(I32), C.POINTER(U32), C.POINTER(F64)
CARD_PCIE_8332, BUS_OP = 25, 6
SVON, ONLINE, MDN, ASTP = 1 << 7, 1 << 24, 1 << 5, 1 << 16
ALM, PEL, MEL, EMG = 1, 2, 4, 16
IO_FAULTS = ALM | PEL | MEL | EMG | (1 << 10) | (1 << 11) | (1 << 12)
LIMIT_MAP_EN = 0x5D
SD_DEC = 7
AUTO_CONNECT = 0x25


class BusNotReady(ControlError):
    """A start was acknowledged but the master is still transitioning."""


class AsyncCall(C.Structure):
    _fields_ = [('h_event', C.c_void_p), ('i32_ret', I32), ('u8_asyncMode', U8)]


class ModuleInfo(C.Structure):
    _fields_ = [('VendorID', I32), ('ProductCode', I32), ('RevisionNo', I32),
                ('TotalAxisNum', I32), ('Axis_ID', I32 * 64), ('Axis_ID_manual', I32 * 64),
                ('All_ModuleType', I32 * 32), ('DI_ModuleNum', I32), ('DI_ModuleType', I32 * 32),
                ('DO_ModuleNum', I32), ('DO_ModuleType', I32 * 32),
                ('AI_ModuleNum', I32), ('AI_ModuleType', I32 * 32),
                ('AO_ModuleNum', I32), ('AO_ModuleType', I32 * 32), ('Name', C.c_char * 128)]


SIGNATURES = {
    'APS_initial': [PI32, I32], 'APS_close': [], 'APS_get_card_name': [I32, PI32],
    'APS_get_first_axisId': [I32, PI32, PI32],
    'APS_get_board_param': [I32, I32, PI32], 'APS_set_board_param': [I32, I32, I32],
    'APS_get_axis_param_f': [I32, I32, PF64], 'APS_set_axis_param_f': [I32, I32, F64],
    'APS_get_axis_param': [I32, I32, PI32], 'APS_set_axis_param': [I32, I32, I32],
    'APS_scan_field_bus': [I32, I32], 'APS_start_field_bus': [I32, I32, I32],
    'APS_stop_field_bus': [I32, I32], 'APS_get_field_bus_master_status': [I32, I32, PU32],
    'APS_get_field_bus_last_scan_info': [I32, I32, PI32, I32, PI32],
    'APS_get_field_bus_module_info': [I32, I32, I32, C.POINTER(ModuleInfo)],
    'APS_get_field_bus_sdo': [I32, I32, I32, U16, U16, C.POINTER(U8), U32, PU32, U32, U32],
    'APS_get_field_bus_pdo_ODIndex': [I32, I32, I32, U16, U16, C.POINTER(U8), U32, PU32],
    'APS_get_field_bus_alarm': [I32, PU32], 'APS_set_servo_on': [I32, I32],
    'APS_motion_status': [I32], 'APS_motion_io_status': [I32],
    'APS_get_position_f': [I32, PF64], 'APS_get_command_f': [I32, PF64],
    'APS_get_command_velocity_f': [I32, PF64], 'APS_get_feedback_velocity_f': [I32, PF64],
    'APS_get_stop_code': [I32, PI32], 'APS_stop_move': [I32], 'APS_emg_stop': [I32],
    'APS_ptp_all': [I32, I32, F64, F64, F64, F64, F64, F64, F64, C.POINTER(AsyncCall)],
}


class APSError(ControlError):
    def __init__(self, function, arguments, code, hint=''):
        self.function, self.code = function, code
        slave_functions = ('APS_get_field_bus_module_info', 'APS_get_field_bus_sdo',
                           'APS_get_field_bus_pdo_ODIndex')
        details = arguments[:3] if function in slave_functions else arguments[:2]
        super().__init__(f'{function}{details} 返回 APS 错误 {code}{hint}')


class APSLibrary:
    def __init__(self, path=''):
        if os.name != 'nt':
            raise ControlError('APS SDK 只能在安装 ADLINK 驱动的 Windows 上运行。')
        filename = 'APS168x64.dll' if C.sizeof(C.c_void_p) == 8 else 'APS168.dll'
        directory = 'APS Library_x64' if C.sizeof(C.c_void_p) == 8 else 'APS Library'
        candidates = [str(Path(path).resolve())] if path else [
            str(Path(os.environ.get('ProgramFiles(x86)', r'C:\Program Files (x86)')) /
                'ADLINK' / directory / filename), filename]
        errors = []
        for candidate in candidates:
            try:
                # Absolute paths and SDK search directories avoid project DLL shadowing.
                self.dll = C.WinDLL(candidate)
                self.path = candidate
                break
            except OSError as exc:
                errors.append(str(exc))
        else:
            raise ControlError(f'无法加载 {filename}，请安装同位数 APS SDK/PCIe-8332 驱动。' + '; '.join(errors))
        for name, args in SIGNATURES.items():
            try:
                fn = getattr(self.dll, name)
            except AttributeError as exc:
                raise ControlError(f'APS DLL 缺少 {name}，请核对 SDK 安装。') from exc
            fn.argtypes, fn.restype = args, I32

    def call(self, name, *args):
        result = getattr(self.dll, name)(*args)
        if result < 0:
            hint = ('；EtherCAT 主站配置错误，请在 MotionCreatorPro2 核对 ESI、重新扫描生成 ENI，'
                    '关闭 MCPro2 后重试（ESI 不是 APS 参数文件）' if result == -4012 else '')
            raise APSError(name, args, result, hint)
        return result

    def value(self, name, *args, kind=F64):
        value = kind()
        self.call(name, *args, C.byref(value))
        if kind is F64 and not math.isfinite(value.value):
            raise ControlError(f'{name} 返回非有限反馈。')
        return value.value


@dataclass(frozen=True)
class APSDevice(Device):
    axis_id: int = 0
    slave_id: int = 0
    units_per_rev: float = 0
    extension_input: str = ''
    extension_sign: int = 0  # APS native position direction; not UI-normalized direction.
    retraction_input: str = ''
    retraction_sign: int = 0

    @cached_property
    def endpoints(self):
        return tuple((prefix, label, input_name, sign) for prefix, label, input_name, sign in
                [('extension', '防推出限位', self.extension_input, self.extension_sign),
                 ('retraction', '防缩回限位', self.retraction_input, self.retraction_sign)] if input_name)

    @cached_property
    def limit_mask(self):
        return sum(PEL if sign == 1 else MEL for _, _, _, sign in self.endpoints)

    @property
    def identity(self):
        return super().identity + (self.axis_id, self.slave_id, self.units_per_rev,
                                   self.gear_numerator, self.gear_denominator, self.extension_input,
                                   self.extension_sign, self.retraction_input, self.retraction_sign)


def validate_options(options):
    defaults = dict(board_id=0, bus_no=0, start_axis_id=0, dll_path='', regenerate_eni=False,
                    axis_units_per_rev={}, poll_interval_s=.005,
                    limit_inputs_connected=True, emg_input_connected=True,
                    extension_limits={}, retraction_limits={})
    if not isinstance(options, dict) or set(options) - set(defaults):
        raise ValueError('aps 配置包含未知字段，或不是 JSON 对象。')
    defaults.update(options)
    for key, upper in [('board_id', 31), ('bus_no', 0), ('start_axis_id', 65535)]:
        if type(defaults[key]) is not int or not 0 <= defaults[key] <= upper:
            raise ValueError(f'aps.{key} 配置无效。')
    if not isinstance(defaults['dll_path'], str) or type(defaults['regenerate_eni']) is not bool:
        raise ValueError('aps.dll_path/regenerate_eni 配置无效。')
    for key in ('limit_inputs_connected', 'emg_input_connected'):
        if type(defaults[key]) is not bool:
            raise ValueError(f'aps.{key} 必须为 true/false。')
    interval = defaults['poll_interval_s']
    if type(interval) not in (int, float) or not math.isfinite(interval) or not .001 <= interval <= .1:
        raise ValueError('aps.poll_interval_s 必须为 0.001～0.1 秒。')
    units = defaults['axis_units_per_rev']
    if not isinstance(units, dict):
        raise ValueError('aps.axis_units_per_rev 必须是以 APS 轴号为键的对象。')
    for axis, value in units.items():
        if (not isinstance(axis, str) or not axis.isdecimal() or str(int(axis)) != axis or int(axis) > 65535
                or type(value) not in (int, float) or not math.isfinite(value) or value <= 0):
            raise ValueError('aps.axis_units_per_rev 的轴号或每转位置单位无效。')
    for key in ('extension_limits', 'retraction_limits'):
        limits = defaults[key]
        if not isinstance(limits, dict):
            raise ValueError(f'aps.{key} 必须是以 APS 轴号为键的对象。')
        for axis, value in limits.items():
            if (not isinstance(axis, str) or not axis.isdecimal() or str(int(axis)) != axis
                    or int(axis) > 65535 or value not in ('di1', 'di2')):
                raise ValueError(f'aps.{key} 格式应为 {{"0":"di1"}} 或 {{"0":"di2"}}。')
    for axis in defaults['extension_limits'].keys() & defaults['retraction_limits'].keys():
        if defaults['extension_limits'][axis] == defaults['retraction_limits'][axis]:
            raise ValueError(f'Axis {axis} 推出和缩回限位不能使用同一路 DI。')
    return defaults


class PCIe8332Controller:
    def __init__(self, adapter='PCIe-8332:0', log_dir=None, options=None, api_factory=None):
        self.options = validate_options(options or {})
        self.board = self.options['board_id']
        self.adapter = adapter
        if adapter != f'PCIe-8332:{self.board}':
            raise ControlError(f'请选择 PCIe-8332:{self.board}；后台不再使用普通网卡。')
        self.log_dir = Path(log_dir) if log_dir else None
        self.api_factory = api_factory or (lambda: APSLibrary(self.options['dll_path']))
        self.api = None
        self.initialized = self.started = False
        self.lock_context = None
        self.saved_board = {}
        self.worker_id = None
        self.pdo_buffers = {}
        self.eni_rebuilt = False

    def adapters(self):
        # Configuration labels only. Enumeration here never calls hardware from UDP thread.
        return [(f'PCIe-8332:{self.board}', f'ADLINK PCIe-8332 · Card {self.board} · APS SDK')]

    def _thread(self):
        current = threading.get_ident()
        if self.worker_id is None:
            self.worker_id = current
        elif self.worker_id != current:
            raise ControlError('APS 硬件只能由同一工作线程访问。')

    def bus_state(self):
        return self.api.value('APS_get_field_bus_master_status', self.board, 0, kind=U32)

    def _open(self):
        self._thread()
        if self.initialized:
            if self.bus_state() == BUS_OP:
                return
            # Only scan/enable preparation enters here; a motion fault is still
            # latched by MotorService. Reopen a lost idle bus on an explicit scan.
            LOG.warning('Idle APS bus left OP; reopening SDK session for scan')
            errors = self.close()
            if errors:
                raise ControlError(f'重新初始化前 APS 清理失败：{errors}')
        lock_context = adapter_lock('APS168-library')
        lock_context.__enter__()
        self.lock_context = lock_context
        try:
            self.api = self.api_factory()
            bits = I32()
            # Even a failed initialization can allocate partial SDK resources.
            self.initialized = True
            self.api.call('APS_initial', C.byref(bits), 0)  # Preserve parameters; auto IDs; synchronous ACK.
            if not bits.value & (1 << self.board):
                raise ControlError(f'没有检测到 Card {self.board}；板卡位图 {bits.value & 0xffffffff:#x}。')
            card = self.api.value('APS_get_card_name', self.board, kind=I32)
            if card != CARD_PCIE_8332:
                raise ControlError(f'Card {self.board} 类型为 {card}，需要 PCIe-8332 (25)。')
            initial_state = self.bus_state()
            LOG.info('APS Card %s initialized: master_state=%s', self.board, initial_state)
            already_op = initial_state == BUS_OP
            if already_op:
                # Card boot-time auto-connect may already be OP. Adopt it only
                # after validating topology and verifying EVERY axis servo-off.
                try:
                    self._devices()
                except APSError as exc:
                    if exc.function != 'APS_get_field_bus_module_info' or exc.code != -41:
                        raise
                    self._attach_running_bus()
                    self._devices()
                self.started = True
            # Save before writing. Never reset alarms automatically or recover/re-enable a failed run.
            for parameter, value in [(0x19, 0), (0x1A, 1), (0x109, 0), (0x28, 0)]:
                self.saved_board[parameter] = self.api.value('APS_get_board_param', self.board, parameter, kind=I32)
                self.api.call('APS_set_board_param', self.board, parameter, value)
            if self.options['regenerate_eni'] and not self.eni_rebuilt:
                if already_op:
                    self.api.call('APS_stop_field_bus', self.board, 0)
                    already_op = False
                self.api.call('APS_scan_field_bus', self.board, 0)
                self.eni_rebuilt = True
            # A failed start can leave a partial bus; close must still attempt stop.
            self.started = True
            if not already_op:
                self._start_bus(rebuild_eni=not self.options['regenerate_eni'])
            self._wait_bus_op()
            if not self.options['emg_input_connected']:
                self._configure_unwired_emg()
        except BaseException as exc:
            errors = self.close()
            if errors:
                LOG.error('APS opening cleanup failed: %s', errors)
                raise ControlError(f'{exc}；APS 初始化清理失败：{errors}') from exc
            raise

    def _wait_bus_op(self):
        deadline = time.monotonic() + 2
        while True:
            state = self.bus_state()
            if state == BUS_OP:
                return
            if time.monotonic() >= deadline:
                raise BusNotReady(f'现场总线启动后未进入 OP（当前状态 {state}），请核对电机供电和网线。')
            time.sleep(.02)

    def _start_bus(self, *, rebuild_eni=False):
        # The firmware auto-connect shortcut can leave OP with no DLL axis map
        # after a cold boot. Use the complete SDK start path, without saving flash.
        if AUTO_CONNECT not in self.saved_board:
            self.saved_board[AUTO_CONNECT] = self.api.value(
                'APS_get_board_param', self.board, AUTO_CONNECT, kind=I32)
        self.api.call('APS_set_board_param', self.board, AUTO_CONNECT, 0)
        self.started = True  # Also clean up a partially acknowledged start.
        try:
            self.api.call('APS_start_field_bus', self.board, 0, self.options['start_axis_id'])
        except APSError as exc:
            # Recover absent/stale ENI once on a stopped bus; never rebuild a
            # pre-existing OP bus just because its process-local map is missing.
            if (not rebuild_eni or self.eni_rebuilt or exc.function != 'APS_start_field_bus'
                    or exc.code not in (-1011, -1012, -1014, -4012, -4013, -4014, -4062)):
                raise
            LOG.warning('Starting saved ENI failed (%s); scanning bus and rebuilding ENI once', exc)
            self.api.call('APS_stop_field_bus', self.board, 0)
            self.eni_rebuilt = True
            self.api.call('APS_scan_field_bus', self.board, 0)
            self.api.call('APS_start_field_bus', self.board, 0, self.options['start_axis_id'])
        self._wait_bus_op()

    def _attach_running_bus(self):
        """Attach a fresh APS session to an already-OP single-axis bus.

        OP is a card state; it does not prove this DLL session has populated
        its slave/axis map. Until start, all axis slots may report -1009 or
        ONLINE=0. Check every slot for servo-on, then initialize the map and
        validate every mapped slave in _devices before publishing scan results.
        """
        info, count = (I32 * 1)(), I32()
        self.api.call('APS_get_field_bus_last_scan_info', self.board, 0, info, 1, C.byref(count))
        first, capacity = I32(), I32()
        self.api.call('APS_get_first_axisId', self.board, C.byref(first), C.byref(capacity))
        if (count.value != 1 or not 1 <= info[0] <= 128 or first.value < 0
                or not 1 <= capacity.value <= 65536 or first.value + capacity.value > 65536):
            raise ControlError('无法核对已运行总线的从站数量和轴范围，未重新接入。')
        online = 0
        for axis in range(first.value, first.value + capacity.value):
            try:
                io = self.api.call('APS_motion_io_status', axis)
            except APSError as exc:
                # Reserved slots and a wholly uninitialized session both report
                # SlaveNotOPState. This is not a usable per-drive ONLINE sample.
                if exc.function == 'APS_motion_io_status' and exc.code == -1009:
                    continue
                raise
            if io & SVON:
                raise ControlError(f'Axis {axis} 已使能；先停止并关闭使能，未重新接入总线。')
            online += bool(io & ONLINE)
        if online not in (0, info[0]):
            raise ControlError(f'已运行总线有 {info[0]} 个从站，但只能核对 {online} 个在线未使能轴；未重新接入。')
        LOG.info('Initializing missing APS session map: slaves=%s online_axis_slots=%s', info[0], online)
        self._start_bus()

    def _configure_unwired_emg(self):
        # Explicit commissioning configuration only. Never auto-clear an EMG
        # belonging to a connected emergency-stop circuit.
        devices = self._devices()  # Verifies all servo axes off before board-wide change.
        states = [bool(self.api.call('APS_motion_io_status', d.axis_id) & EMG) for d in devices]
        if not any(states):
            return
        if not all(states):
            raise ControlError('各轴 EMG 状态不一致；请核对板卡急停输入与驱动状态。')
        old = self.api.value('APS_get_board_param', self.board, 0, kind=I32)
        if old not in (0, 1):
            raise ControlError(f'PRB_EMG_LOGIC={old} 无效，不能配置空置急停输入。')
        self.saved_board[0] = old
        self.api.call('APS_set_board_param', self.board, 0, old ^ 1)
        LOG.info('Unwired EMG input: Card %s PRB_EMG_LOGIC %s -> %s', self.board, old, old ^ 1)
        deadline = time.monotonic() + .5
        while any(self.api.call('APS_motion_io_status', d.axis_id) & EMG for d in devices):
            if time.monotonic() >= deadline:
                raise ControlError('空置 EMG 输入调整极性后仍有效；请核对 PCIe-8332 急停端子，未使能。')
            time.sleep(self.options['poll_interval_s'])

    def _sdo(self, slave, index, sub=0, signed=False):
        data, length = (U8 * 4)(), U32()
        self.api.call('APS_get_field_bus_sdo', self.board, 0, slave, index, sub,
                      data, 4, C.byref(length), 500, 0)
        if length.value not in (1, 2, 4):
            raise ControlError(f'Slave {slave} SDO {index:#x}:{sub} 长度无效：{length.value}。')
        return int.from_bytes(bytes(data[:length.value]), 'little', signed=signed)

    def _devices(self):
        info, count = (I32 * 1)(), I32()
        self.api.call('APS_get_field_bus_last_scan_info', self.board, 0, info, 1, C.byref(count))
        if count.value != 1 or not 1 <= info[0] <= 128:
            raise ControlError('APS 扫描未返回有效的从站数量。')
        first, capacity = I32(), I32()
        self.api.call('APS_get_first_axisId', self.board, C.byref(first), C.byref(capacity))
        devices, axis_ids = [], set()
        for slave in range(info[0]):
            module = ModuleInfo()
            self.api.call('APS_get_field_bus_module_info', self.board, 0, slave, C.byref(module))
            if module.TotalAxisNum == 0:
                continue  # Allow IO slaves without presenting them as motors.
            if (module.VendorID != 0x00100000 or module.ProductCode != 0x000C010E
                    or module.TotalAxisNum != 1):
                raise ControlError(f'Slave {slave} 不是已核对的 SV635N 单轴设备。')
            axis = module.Axis_ID[0]
            if axis in axis_ids or not first.value <= axis < first.value + capacity.value:
                raise ControlError(f'Slave {slave} 的 APS 轴映射无效：{axis}。')
            axis_ids.add(axis)
            io = self.api.call('APS_motion_io_status', axis)
            if io & SVON:
                raise ControlError(f'Axis {axis} 已使能；先在原控制程序中停止并关闭使能。')
            if not io & ONLINE:
                raise ControlError(f'Axis {axis} 从站不在线。')
            motor = self._sdo(slave, 0x2000, 1)
            direction = self._sdo(slave, 0x2002, 3)
            numerator, denominator = self._sdo(slave, 0x6091, 1), self._sdo(slave, 0x6091, 2)
            if motor != 14101 or direction not in (0, 1) or not numerator or not denominator:
                raise ControlError(f'Axis {axis} 的电机型号、方向或电子齿轮参数未通过核对。')
            units = self.options['axis_units_per_rev'].get(str(axis), (2**23) * denominator / numerator)
            endpoints = {}
            for prefix in ('extension', 'retraction'):
                input_name = self.options[prefix + '_limits'].get(str(axis), '')
                sign = 0
                if input_name:
                    function = self._sdo(slave, 0x2003, 3 if input_name == 'di1' else 5)
                    if function not in (14, 15):
                        raise ControlError(f'Axis {axis} {input_name.upper()} 必须配置为 P-OT(14) 或 N-OT(15)。')
                    sign = 1 if function == 14 else -1
                endpoints[prefix + '_input'] = input_name
                endpoints[prefix + '_sign'] = sign
            if endpoints['extension_sign'] and endpoints['retraction_sign']:
                if (endpoints['extension_input'] == endpoints['retraction_input'] or
                        endpoints['extension_sign'] == endpoints['retraction_sign']):
                    raise ControlError(f'Axis {axis} 两端限位必须使用不同 DI，且分别配置为 P-OT 和 N-OT。')
            devices.append(APSDevice(order=slave + 1, alias=self._sdo(slave, 0x200E, 0x16),
                name=f'SV635N · APS Axis {axis} / Slave {slave}', vendor=module.VendorID,
                product=module.ProductCode, revision=module.RevisionNo, state=BUS_OP,
                statusword=self._sdo(slave, 0x6041), error_code=self._sdo(slave, 0x603F),
                position=self.api.value('APS_get_position_f', axis), gear_numerator=numerator,
                gear_denominator=denominator, motor_code=motor, positive_direction=direction,
                axis_id=axis, slave_id=slave, units_per_rev=units,
                **endpoints))
        if not devices:
            raise ControlError('总线上没有 SV635N 电机。')
        missing = (set(self.options['extension_limits']) | set(self.options['retraction_limits'])) - {str(d.axis_id) for d in devices}
        if missing:
            raise ControlError(f'端点限位配置中的 APS 轴号不存在：{sorted(missing)}')
        return devices

    def scan(self):
        self.eni_rebuilt = False
        for attempt in range(1, 4):
            try:
                self._open()
                return self._devices()
            except Exception as exc:
                errors = self.close()
                if errors:
                    raise ControlError(f'{exc}；APS 扫描清理失败：{errors}') from exc
                transient = isinstance(exc, BusNotReady) or (
                    isinstance(exc, APSError) and (exc.code in
                        (-5, -9, -15, -1008, -1009, -4003, -4004, -4005, -4042, -4043, -4044)
                        or (exc.code == -41 and exc.function == 'APS_get_field_bus_module_info')))
                if not transient or attempt == 3:
                    raise
                LOG.warning('APS startup/scan attempt %s/3 failed: %s; retrying fresh session', attempt, exc)
                time.sleep(.25)

    def read_inputs(self, devices, *, bus_ok=None, io_status=None):
        """Read cyclic PDO memory, never a blocking SDO in the motion loop.

        SV635N manual pp.521-522: 60FD bit0=N-OT, bit1=P-OT,
        bit16..20=DI1..5. APS PDO lengths are in BITS (SDK p.721).
        A missing PDO is unknown feedback, never an inactive limit.
        """
        self._thread()
        result = {}
        if bus_ok is None:
            bus_ok = self.initialized and self.bus_state() == BUS_OP
        for d in devices:
            sensor = dict(input=d.extension_input or None, state='unconfigured', triggered=None,
                          extension_sign=d.extension_sign,
                          retraction_input=d.retraction_input or None, retraction_sign=d.retraction_sign,
                          retraction_state='unconfigured' if d.retraction_input else 'no_sensor',
                          retraction_triggered=None, conflict=False, valid=False, digital_inputs=None,
                          positive_limit=None, negative_limit=None, di1=None, di2=None)
            try:
                if not bus_ok:
                    raise ControlError('总线不在 OP')
                io = (io_status[d.axis_id] if io_status is not None else
                      self.api.call('APS_motion_io_status', d.axis_id))
                if not io & ONLINE:
                    raise ControlError('从站离线')
                if d.slave_id not in self.pdo_buffers:
                    self.pdo_buffers[d.slave_id] = (U8 * 4)(), U32()
                data, length = self.pdo_buffers[d.slave_id]
                length.value = 0
                self.api.call('APS_get_field_bus_pdo_ODIndex', self.board, 0, d.slave_id,
                              0x60FD, 0, data, 32, C.byref(length))
                if length.value != 32:
                    raise ControlError(f'60FD PDO 长度为 {length.value} bit，需要 32 bit')
                raw = int.from_bytes(bytes(data), 'little')
                sensor.update(valid=True, digital_inputs=raw, positive_limit=bool(raw & 2),
                              negative_limit=bool(raw & 1), di1=bool(raw & (1 << 16)),
                              di2=bool(raw & (1 << 17)))
                for prefix, _, _, sign in d.endpoints:
                    bit, native = (2, PEL) if sign == 1 else (1, MEL)
                    triggered = bool(raw & bit or io & native)
                    stem = '' if prefix == 'extension' else 'retraction_'
                    sensor[stem + 'triggered'] = triggered
                    sensor[stem + 'state'] = 'triggered' if triggered else 'clear'
                sensor['conflict'] = bool(sensor['triggered'] and sensor['retraction_triggered'])
            except Exception as exc:
                sensor.update(state='unavailable', error=str(exc))
                if d.retraction_input:
                    sensor['retraction_state'] = 'unavailable'
            result[d.order] = sensor
        return result

    @staticmethod
    def check_limit_feedback(d, sensor):
        if d.endpoints and not sensor['valid']:
            raise ControlError(f'Axis {d.axis_id} 端点限位反馈失效：{sensor.get("error")}')
        if sensor.get('conflict'):
            raise ControlError(f'Axis {d.axis_id} 推出与缩回限位同时触发；请核对接线、极性和 DI 功能。')

    def close(self):
        self._thread()
        errors = []
        def attempt(name, *args):
            try:
                self.api.call(name, *args)
            except Exception as exc:
                errors.append(str(exc))
        if self.initialized:
            if self.started:
                attempt('APS_stop_field_bus', self.board, 0)
            for parameter, value in self.saved_board.items():
                attempt('APS_set_board_param', self.board, parameter, value)
            attempt('APS_close')
        self.initialized = self.started = False
        self.saved_board.clear()
        self.pdo_buffers.clear()
        if self.lock_context is not None:
            self.lock_context.__exit__(None, None, None)
            self.lock_context = None
        return errors

    def run_continuous(self, devices, commands, stop, callback):
        self._thread()
        report = dict(error=None, stopped=False, all_disabled=False, cleanup_errors=[],
                      backend='aps', started_commands=0, completed_commands=0)
        owned, saved_deceleration, origins = [], {}, {}
        saved_limit_mapping = {}
        selected, last_targets = [], {}
        stale_abnormal_stops = set()
        limit_stops = set()
        blocked_message = False
        history, trace = deque(maxlen=1000), deque(maxlen=1000)
        active = None
        api = self.api
        def check_stop():
            if stop.is_set() or commands.closed:
                raise Stopped('连续控制已停止。')
        def io_checked(d, enabled=False, io=None):
            if io is None:
                io = api.call('APS_motion_io_status', d.axis_id)
            mask = IO_FAULTS if self.options['limit_inputs_connected'] else IO_FAULTS & ~(PEL | MEL)
            # Configured endpoint limits permit motion away from the active end.
            mask &= ~d.limit_mask
            reasons = []
            if not io & ONLINE:
                reasons.append('从站离线')
            for bit, label in [(ALM, '伺服报警 ALM'), (PEL, '正限位 PEL'), (MEL, '负限位 MEL'),
                               (EMG, '急停输入 EMG'), (1 << 10, '软件环形限位 SCL'),
                               (1 << 11, '软件正限位 SPEL'), (1 << 12, '软件负限位 SMEL')]:
                if io & mask & bit:
                    reasons.append(label)
            if reasons:
                alarm = api.value('APS_get_field_bus_alarm', d.axis_id, kind=U32)
                hint = (' 请核对急停接线/输入极性；确实未接急停时配置 aps.emg_input_connected=false。'
                        if io & EMG else '')
                raise ControlError(f'Axis {d.axis_id} {"、".join(reasons)}：IO={io:#x}, alarm={alarm:#x}。{hint}')
            if enabled and not io & SVON:
                raise ControlError(f'Axis {d.axis_id} 运行期间退出使能。')
            return io
        try:
            check_stop()
            current = self.scan()
            api = self.api
            refreshed = {d.order: d for d in current}
            if [d.identity for d in devices] != [d.identity for d in current]:
                raise ControlError('扫描后设备/轴映射/电子齿轮发生变化，请重新扫描。')
            selected = [refreshed[n] for n in commands.orders]
            preflight = self.read_inputs(selected)
            for d in selected:
                self.check_limit_feedback(d, preflight[d.order])
            profile_revision, rpm, acceleration_rpm_s = commands.profile()
            def native_profile(rpm, acceleration_rpm_s):
                values = {d.axis_id: (d.units_per_rev * (rpm / 60),
                                     d.units_per_rev * (acceleration_rpm_s / 60)) for d in selected}
                if not all(math.isfinite(v) and v > 0 for pair in values.values() for v in pair):
                    raise ControlError('转换后的 APS 速度/加速度无法表示。')
                return values
            profile_values = native_profile(rpm, acceleration_rpm_s)
            for d in selected:
                check_stop()
                if not self.options['limit_inputs_connected']:
                    original = api.value('APS_get_axis_param', d.axis_id, LIMIT_MAP_EN, kind=I32)
                    saved_limit_mapping[d.axis_id] = original
                    # Retain both configured ends, ORG and unrelated mapping bits.
                    unwired = (PEL | MEL) & ~d.limit_mask
                    api.call('APS_set_axis_param', d.axis_id, LIMIT_MAP_EN, original & ~unwired)
                if d.endpoints:
                    self.check_limit_feedback(d, self.read_inputs([d])[d.order])
                io_checked(d)
                if d.error_code or d.statusword & 8:
                    raise ControlError(f'Axis {d.axis_id} 存在驱动报警 {d.error_code:#x}。')
                velocity, acceleration = profile_values[d.axis_id]
                saved_deceleration[d.axis_id] = api.value('APS_get_axis_param_f', d.axis_id, SD_DEC)
                api.call('APS_set_axis_param_f', d.axis_id, SD_DEC, acceleration)
                before_enable = api.value('APS_get_position_f', d.axis_id)
                # Record ownership before the call: an error may occur after servo-on was applied.
                owned.append(d)
                api.call('APS_set_servo_on', d.axis_id, 1)
                deadline = time.monotonic() + 2
                while not io_checked(d) & SVON:
                    check_stop()
                    if time.monotonic() >= deadline:
                        raise ControlError(f'Axis {d.axis_id} 使能确认超时。')
                    stop.wait(self.options['poll_interval_s'])
                origins[d.axis_id] = api.value('APS_get_position_f', d.axis_id)
                command = api.value('APS_get_command_f', d.axis_id)
                tolerance = max(2, d.units_per_rev * .2 / 360)
                if abs(before_enable - origins[d.axis_id]) > tolerance:
                    raise ControlError(f'Axis {d.axis_id} 使能期间出现意外位移。')
                if abs(command - origins[d.axis_id]) > tolerance:
                    raise ControlError(f'Axis {d.axis_id} 使能后指令位置与反馈不一致。')
                status = api.call('APS_motion_status', d.axis_id)
                if status & ASTP and status & MDN:
                    # APS retains ASTP until the next motion command (manual p.274/1222).
                    # Current ALM/EMG/limits remain checked; no motion is issued to clear it.
                    stale_abnormal_stops.add(d.axis_id)
                    LOG.info('Axis %s has historical ASTP while stopped; awaiting explicit target', d.axis_id)
            check_stop()
            callback(dict(kind='profile_applied', revision=profile_revision, rpm=rpm,
                          acceleration_rpm_s=acceleration_rpm_s))
            commands.mark_ready()
            callback(dict(kind='continuous_ready', orders=list(commands.orders)))
            began = time.monotonic()
            last_status = 0
            selected_ids = {d.axis_id for d in selected}
            unselected = [d for d in current if d.axis_id not in selected_ids]
            limits = {}
            positions = dict(origins)
            while True:
                cycle_started = time.monotonic()
                check_stop()
                if self.bus_state() != BUS_OP:
                    raise ControlError('PCIe-8332 总线退出 OP。')
                axes = []
                status_due = cycle_started - last_status >= .05
                # One fresh native IO sample serves both limit decoding and faults.
                # Selected axes' 60FD PDO and motion status are checked every cycle.
                io_status = {d.axis_id: api.call('APS_motion_io_status', d.axis_id) for d in selected}
                limits.update(self.read_inputs(selected, bus_ok=True, io_status=io_status))
                for d in selected:
                    sensor = limits[d.order]
                    self.check_limit_feedback(d, sensor)
                    io = io_checked(d, True, io=io_status[d.axis_id])
                    status = api.call('APS_motion_status', d.axis_id)
                    if status_due or sensor.get('triggered') or sensor.get('retraction_triggered'):
                        positions[d.axis_id] = api.value('APS_get_position_f', d.axis_id)
                    position = positions[d.axis_id]
                    for prefix, label, _, direction in d.endpoints:
                        trigger_key = 'triggered' if prefix == 'extension' else 'retraction_triggered'
                        if (sensor.get(trigger_key) and d.axis_id not in limit_stops and
                                ((last_targets.get(d.axis_id, position) - position) * direction > 0
                                 or api.value('APS_get_feedback_velocity_f', d.axis_id) * direction > 0)):
                            api.call('APS_stop_move', d.axis_id)
                            limit_stops.add(d.axis_id)
                            last_targets.pop(d.axis_id, None)
                            if active:
                                active['limit_stopped'] = True
                                active = None
                            callback(dict(kind='limit_blocked', limits=dict(limits),
                                          text=f'电机 {d.order} {label}触发，已停止向该端运动；可提交反向目标'))
                            blocked_message = True
                            status = api.call('APS_motion_status', d.axis_id)
                    if status & ASTP and (d.axis_id not in stale_abnormal_stops or
                                          d.axis_id in last_targets or not status & MDN):
                        code = api.value('APS_get_stop_code', d.axis_id, kind=I32)
                        directional_stop = (any(code == (4 if direction == 1 else 5)
                                                for _, _, _, direction in d.endpoints) or
                                            code == 9 and d.axis_id in limit_stops)
                        if not directional_stop:
                            raise ControlError(f'Axis {d.axis_id} 异常停止，stop_code={code}。')
                    sign = 1 if d.positive_direction else -1
                    axes.append(dict(order=d.order, axis_id=d.axis_id, slave_id=d.slave_id,
                        position=position, enabled=bool(io & SVON), error_code=0,
                        travel_degrees=(position - origins[d.axis_id]) * 360 / d.units_per_rev * sign,
                        motion_status=status, motion_io_status=io))
                now = time.monotonic()
                if status_due:
                    if unselected:
                        limits.update(self.read_inputs(unselected, bus_ok=True))
                    callback(dict(kind='status', axes=axes, limits=dict(limits)))
                    trace.append(dict(elapsed_s=now - began, axes=axes))
                    last_status = now
                if status_due and active and all(a['motion_status'] & MDN and
                                  abs(a['travel_degrees'] - t) <= .2
                                  for a, t in zip(axes, active['targets_deg'])):
                    active['completed'] = True
                    report['completed_commands'] += 1
                    active = None
                revision, new_rpm, new_acceleration = commands.profile()
                if revision != profile_revision:
                    profile_values = native_profile(new_rpm, new_acceleration)
                    fresh_limits = self.read_inputs(selected)
                    for d in selected:
                        self.check_limit_feedback(d, fresh_limits[d.order])
                    for d in selected:
                        check_stop()
                        velocity, acceleration = profile_values[d.axis_id]
                        api.call('APS_set_axis_param_f', d.axis_id, SD_DEC, acceleration)
                        # Retarget only an axis still moving to a previously accepted goal.
                        # A limit stop deletes that goal; a profile change cannot restore it.
                        target = last_targets.get(d.axis_id)
                        if target is None or api.call('APS_motion_status', d.axis_id) & MDN:
                            continue
                        position = api.value('APS_get_position_f', d.axis_id)
                        sensor = fresh_limits[d.order]
                        if any(sensor['triggered' if prefix == 'extension' else 'retraction_triggered']
                               and (target - position) * direction > 0
                               for prefix, _, _, direction in d.endpoints):
                            api.call('APS_stop_move', d.axis_id)
                            last_targets.pop(d.axis_id, None)
                            limit_stops.add(d.axis_id)
                            if active:
                                active['limit_stopped'] = True
                                active = None
                            callback(dict(kind='limit_blocked', limits={**limits, **fresh_limits},
                                          text=f'电机 {d.order} 端点限位触发，已停止；可提交反向目标'))
                            blocked_message = True
                            continue
                        api.call('APS_ptp_all', d.axis_id, 0, target, 0., velocity, 0.,
                                 acceleration, acceleration, 0., None)
                    profile_revision, rpm, acceleration_rpm_s = revision, new_rpm, new_acceleration
                    callback(dict(kind='profile_applied', revision=revision, rpm=rpm,
                                  acceleration_rpm_s=acceleration_rpm_s))
                request = commands.pop()
                if request:
                    # Convert/validate every selected target before submitting any move.
                    targets = [origins[d.axis_id] + value * d.units_per_rev / 360 *
                               (1 if d.positive_direction else -1) for d, value in zip(selected, request.targets)]
                    if not all(math.isfinite(t) and abs(t) <= 2**53 - 1 for t in targets):
                        raise ControlError('目标超出 APS F64 可精确表示的位置范围。')
                    # Re-read immediately before submitting, including a target racing an input edge.
                    fresh_limits = self.read_inputs(selected)
                    blocked = []
                    for d, target in zip(selected, targets):
                        sensor = fresh_limits[d.order]
                        self.check_limit_feedback(d, sensor)
                        if d.endpoints:
                            position = api.value('APS_get_position_f', d.axis_id)
                            if any(sensor['triggered' if prefix == 'extension' else 'retraction_triggered']
                                   and (target - position) * direction > 0
                                   for prefix, _, _, direction in d.endpoints):
                                blocked.append(d.order)
                    if blocked:
                        history.append(dict(number=request.number, targets_deg=list(request.targets),
                                            blocked_orders=blocked, stage='limit_blocked'))
                        callback(dict(kind='limit_blocked', limits={**limits, **fresh_limits},
                                      text=f'电机 {blocked} 端点限位触发，目标已拦截；可提交反向目标'))
                        blocked_message = True
                        commands.wait_for_update(max(0., self.options['poll_interval_s'] -
                                                     (time.monotonic() - cycle_started)))
                        continue  # Discard this request; never resume it when the sensor clears.
                    for d, target in zip(selected, targets):
                        check_stop()
                        if last_targets.get(d.axis_id) == target:
                            continue
                        velocity, acceleration = profile_values[d.axis_id]
                        # Option 0 = absolute + aborting, no buffer, no wait-trigger.
                        # NULL in synchronous mode acknowledges submission, never waits for arrival.
                        api.call('APS_ptp_all', d.axis_id, 0, target, 0., velocity, 0.,
                                 acceleration, acceleration, 0., None)
                        last_targets[d.axis_id] = target
                        limit_stops.discard(d.axis_id)
                    if blocked_message:
                        callback(dict(kind='phase', text='连续使能就绪；已接受新目标'))
                        blocked_message = False
                    report['started_commands'] += 1
                    if active:
                        active['superseded'] = True
                    active = dict(number=request.number, targets_deg=list(request.targets),
                                  completed=False, superseded=False)
                    history.append(active)
                    callback(dict(kind='command', number=request.number, stage='started', pending=commands.pending()))
                commands.wait_for_update(max(0., self.options['poll_interval_s'] -
                                             (time.monotonic() - cycle_started)))
        except Stopped as exc:
            report.update(stopped=True, error=str(exc))
        except Exception as exc:
            report['error'] = str(exc)
            LOG.exception('APS continuous run failed')
        finally:
            commands.close()
            errors = report['cleanup_errors']
            def attempt(name, *args):
                try:
                    api.call(name, *args)
                    return True
                except Exception as exc:
                    errors.append(str(exc))
                    return False
            # Submit stop to ALL owned axes even if one fails. Never touch unselected axes.
            for d in owned:
                if not attempt('APS_stop_move', d.axis_id):
                    attempt('APS_emg_stop', d.axis_id)
            _, rpm, acceleration_rpm_s = commands.profile()
            deadline = time.monotonic() + max(1., rpm / acceleration_rpm_s + .5)
            deadline = min(deadline, time.monotonic() + 5)
            pending = list(owned)
            while pending and time.monotonic() < deadline:
                for d in pending[:]:
                    try:
                        done = api.call('APS_motion_status', d.axis_id) & MDN
                        speed = api.value('APS_get_feedback_velocity_f', d.axis_id)
                        if done and abs(speed) <= max(1., d.units_per_rev * .01 / 60):
                            pending.remove(d)
                    except Exception as exc:
                        errors.append(str(exc))
                        attempt('APS_emg_stop', d.axis_id)
                        pending.remove(d)
                if pending:
                    time.sleep(self.options['poll_interval_s'])
            for d in pending:
                errors.append(f'Axis {d.axis_id} 减速停止确认超时，执行急停并关闭使能。')
                attempt('APS_emg_stop', d.axis_id)
            for d in owned:
                attempt('APS_set_servo_on', d.axis_id, 0)
            pending_disable = list(owned)
            deadline = time.monotonic() + 2
            while pending_disable and time.monotonic() < deadline:
                for d in pending_disable[:]:
                    try:
                        io = api.call('APS_motion_io_status', d.axis_id)
                        if io & ONLINE and not io & SVON and self.bus_state() == BUS_OP:
                            pending_disable.remove(d)
                    except Exception as exc:
                        errors.append(str(exc))
                        # Failed read cannot establish disable; retain it for the final report.
                        deadline = time.monotonic()
                        break
                if pending_disable:
                    time.sleep(self.options['poll_interval_s'])
            report['all_disabled'] = not pending_disable
            if pending_disable:
                errors.append('未能核对所有选中轴关闭使能。')
            for axis, value in saved_deceleration.items():
                attempt('APS_set_axis_param_f', axis, SD_DEC, value)
            for axis, value in saved_limit_mapping.items():
                attempt('APS_set_axis_param', axis, LIMIT_MAP_EN, value)
            # A fault requires a fresh APS lifecycle on the next explicit scan.
            if not report['stopped'] or errors:
                errors.extend(self.close())
            report.update(commands=list(history), trace=list(trace), trace_tail_only=True,
                          command_history_tail_only=True,
                          selected_axes=[dict(order=d.order, axis_id=d.axis_id, slave_id=d.slave_id,
                              units_per_rev=d.units_per_rev) for d in selected])
            if self.log_dir:
                try:
                    self.log_dir.mkdir(parents=True, exist_ok=True)
                    path = self.log_dir / (datetime.now().strftime('aps_%Y%m%d_%H%M%S_%f') + '.json')
                    report['log_path'] = str(path)
                    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
                except Exception as exc:
                    LOG.exception('Unable to save APS report: %s', exc)
        return report
