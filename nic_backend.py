"""SV635N backend using the motherboard NIC, Npcap and the existing SOEM PP loop."""
from dataclasses import asdict, dataclass
from collections import deque
from functools import cached_property
import math
import struct
import threading
import time

from control_common import ControlError, Device, DEFAULT_ADAPTER, adapter_lock
from core import EtherCATController, displacement, read
from continuous_control import _ManualSession
from nic_config import validate_options
from nic_safety import (COMMISSIONING_MAX_RPM, LimitMemory, envelope_bounds, stopping_distance)


@dataclass(frozen=True)
class NICDevice(Device):
    axis_id: int = 0  # Zero-based SOEM slave index, not an APS axis ID.
    slave_id: int = 0
    units_per_rev: float = 0
    extension_input: str = ''
    extension_sign: int = 0
    retraction_input: str = ''
    retraction_sign: int = 0

    @cached_property
    def endpoints(self):
        return tuple((prefix, label, name, sign) for prefix, label, name, sign in
                     [('extension', '防推出限位', self.extension_input, self.extension_sign),
                      ('retraction', '防缩回限位', self.retraction_input, self.retraction_sign)] if name)

    @property
    def identity(self):
        return super().identity + (self.units_per_rev, self.gear_numerator, self.gear_denominator,
                                  self.extension_input, self.extension_sign,
                                  self.retraction_input, self.retraction_sign)


def input_state(device, raw=None, error=None, error_code=0):
    sensor = dict(input=device.extension_input or None, extension_sign=device.extension_sign,
                  retraction_input=device.retraction_input or None, retraction_sign=device.retraction_sign,
                  triggered=None, retraction_triggered=None, conflict=False, valid=raw is not None,
                  state='unconfigured', retraction_state='unconfigured' if device.retraction_input else 'no_sensor',
                  digital_inputs=raw, positive_limit=None, negative_limit=None, di1=None, di2=None)
    if raw is None:
        sensor.update(state='unavailable', error=str(error))
        if device.retraction_input:
            sensor['retraction_state'] = 'unavailable'
    else:
        sensor.update(positive_limit=bool(raw & 2 or error_code == 0x5443),
                      negative_limit=bool(raw & 1 or error_code == 0x5444),
                      di1=bool(raw & (1 << 16)), di2=bool(raw & (1 << 17)))
        for prefix, _, _, sign in device.endpoints:
            stem = '' if prefix == 'extension' else 'retraction_'
            active = sensor['positive_limit' if sign == 1 else 'negative_limit']
            sensor[stem + 'triggered'] = active
            sensor[stem + 'state'] = 'triggered' if active else 'clear'
        sensor['conflict'] = bool(sensor['triggered'] and sensor['retraction_triggered'])
    return sensor


class NICController(EtherCATController):
    backend_name = 'ethercat'

    def __init__(self, adapter=DEFAULT_ADAPTER, log_dir=None, options=None, master_factory=None):
        super().__init__(adapter, log_dir, master_factory)
        self.options = validate_options(options or {})
        # There is no PCIe board emergency input. Drive alarms/quick stop remain monitored.
        self.options['emg_input_connected'] = False
        self.limit_memory = LimitMemory(self.log_dir/'nic-limit-memory.json' if self.log_dir else None)

    def validate_motion_profile(self, commands, rpm=None):
        rpm = commands.profile()[1] if rpm is None else rpm
        if type(rpm) not in (int, float) or not math.isfinite(rpm) or rpm <= 0:
            raise ControlError('速度必须为正有限数值。')
        envelope = getattr(commands, 'travel_envelope', None)
        if envelope is None and rpm > COMMISSIONING_MAX_RPM:
            raise ControlError('未标定固定中位和实测软限位，调试速度最高 5 rpm；请先完成行程标定。')
        if envelope is not None:
            maximum = envelope.get('max_speed_rpm')
            if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum <= 0:
                raise ControlError('软限位缺少已标定的运行速度上限。')
            if rpm > maximum:
                raise ControlError(f'速度超过已标定机构上限 {maximum:g} rpm。')

    def _discover(self, master):
        devices = []
        for device, slave in zip(super()._discover(master), master.slaves):
            if (device.motor_code != 14101 or device.positive_direction not in (0, 1)
                    or device.gear_numerator <= 0 or device.gear_denominator <= 0):
                raise ControlError(f'电机 {device.order} 型号、方向或电子齿轮参数无效。')
            endpoints = {}
            for prefix in ('extension', 'retraction'):
                name = self.options[prefix + '_limits'].get(str(device.order), '')
                function = read(slave, 0x2003, 3 if name == 'di1' else 5) if name else 0
                if name and function not in (14, 15):
                    raise ControlError(f'电机 {device.order} {name.upper()} 必须配置为 P-OT(14) 或 N-OT(15)。')
                endpoints[prefix + '_input'] = name
                endpoints[prefix + '_sign'] = (1 if function == 14 else -1) if name else 0
            if endpoints['extension_sign'] and endpoints['extension_sign'] == endpoints['retraction_sign']:
                raise ControlError(f'电机 {device.order} 两端限位必须分别配置为 P-OT 和 N-OT。')
            devices.append(NICDevice(**asdict(device), axis_id=device.order-1, slave_id=device.order-1,
                units_per_rev=2**23 * device.gear_denominator / device.gear_numerator, **endpoints))
        missing = (set(self.options['extension_limits']) | set(self.options['retraction_limits'])) - {str(d.order) for d in devices}
        if missing:
            raise ControlError(f'端点限位配置中的链路位置不存在：{sorted(missing)}')
        return devices

    def read_inputs(self, devices):
        # Idle reads use SDO in a short, exclusive PRE-OP session. Motion uses PDO only.
        with adapter_lock(self.adapter):
            master = self.master_factory()
            opened = False
            try:
                master.open(self.adapter)
                opened = True
                current = self._discover(master)
                if [d.identity for d in current] != [d.identity for d in devices]:
                    raise ControlError('链路设备已变化，请重新扫描。')
                result = {}
                for d in devices:
                    try:
                        slave = master.slaves[d.slave_id]
                        sensor = input_state(d, read(slave, 0x60FD), error_code=read(slave, 0x603F))
                        result[d.order] = self.limit_memory.protect(
                            self.adapter, d, read(slave, 0x6064, signed=True), sensor)
                    except Exception as exc:
                        result[d.order] = input_state(d, error=exc)
                return result
            finally:
                if opened:
                    master.close()
                self.limit_memory.save()

    def run_continuous(self, devices, commands, stop=None, callback=None):
        try:
            self.validate_motion_profile(commands)
            if not set(commands.orders) <= {d.order for d in devices}:
                raise ControlError('所选电机不在扫描结果中，请重新扫描。')
            with adapter_lock(self.adapter):
                return _NICSession(self, devices, commands, stop or threading.Event(),
                                   callback or (lambda event: None)).run()
        finally:
            commands.close()
            self.limit_memory.save()

    def close(self):
        # Each scan/input read/session owns and closes its own SOEM master.
        return []


class _NICSession(_ManualSession):
    def __init__(self, *args):
        super().__init__(*args)
        self.limits = {}
        self.cleaning = False
        self.recovering_communication = True
        self.limit_pending = set()
        self.limit_halted = False
        self.report.update(backend='ethercat', control_source='udp')

    def check_device_error(self, device):
        # SV635N 603F also contains No.3 warnings. CiA402 bit3 distinguishes a fault.
        if device.error_code == 0x0900:
            raise ControlError(f'电机 {device.order} DI 紧急刹车有效，须释放后再获取控制。')
        if device.statusword & 8 and device.error_code != 0x0E08:
            raise ControlError(f'电机 {device.order} 驱动故障 0x{device.error_code:04X}，须处理后再获取控制。')

    def check_drive_feedback(self, axis):
        code, sw, _ = axis['last']
        if code == 0x0900:
            raise ControlError(f"电机 {axis['device'].order} DI 紧急刹车有效，已停止全部电机。")
        if sw & 8:
            if self.recovering_communication and not self.armed and code == 0x0E08:
                return
            raise ControlError(f"电机 {axis['device'].order} 驱动故障 0x{code:04X}，已停止；重新获取控制前核对故障原因。")
        # Bit11 reports internal limiting (speed/torque/position); directional
        # travel limits are checked separately against 60FD and 5443/5444 below.

    def prepare_op(self):
        # Only an explicit new run reaches this hook. No runtime fault auto-resume.
        # Stable PDO/OP is required before a single reset edge, with all axes disabled.
        self.hold(.05)
        if any(a['last'][1] & 4 for a in self.axes):
            raise ControlError('通信恢复前检测到轴已使能，禁止复位接管。')
        affected = [a for a in self.axes if a['last'][1] & 8 and a['last'][0] == 0x0E08]
        if affected:
            self.emit('phase', text='总线已恢复 OP，正在未使能状态复位残留通信故障 0x0E08')
            self.report['communication_reset'] = dict(orders=[a['device'].order for a in affected],
                                                       attempted=True, cleared=False)
            try:
                for a in affected:
                    a['cw'] = 0
                self.hold(.01)
                for a in affected:
                    a['cw'] = 0x80  # CiA402 fault-reset edge; enable-operation remains clear.
                self.hold(.01)
            finally:
                for a in affected:
                    a['cw'] = 0
            deadline = time.monotonic()+1
            while any(a['last'][1] & 8 for a in affected):
                if time.monotonic() >= deadline:
                    raise ControlError('0x0E08 通信故障复位后仍有效；请检查网线、同步和主站状态，再重新获取控制。')
                self.tick()
            self.report['communication_reset']['cleared'] = True
        self.recovering_communication = False

    def setup(self):
        self.controller.validate_motion_profile(self.commands)
        for device in self.snapshot:
            if device.order in self.commands.orders:
                envelope_bounds(getattr(self.commands, 'travel_envelope', None), device)
        super().setup()
        reference = getattr(self.commands, 'neutral_reference', None)
        if reference:
            for axis in reference['axes']:
                d = next(a['device'] for a in self.axes if a['device'].order == axis['order'])
                if (list(d.identity) != axis['identity'] or
                        abs(displacement(d.position, axis['position'])) > max(2., d.units_per_rev*.2/360)):
                    raise ControlError('三轴尚未回到保存的固定初始位，请先返回固定初始位。')

    def additional_parameters(self, axis):
        # Both the drive-local P/N-OT reaction and PP Halt brake with 6085 and
        # retain position lock. H02.07=4 is required for this vertical assembly.
        return [(0x605D, 2, 2, True), (0x2002, 2, 4, False, 8)]

    def stop_deceleration(self, axis, acceleration):
        value = round(axis['counts'] * self.controller.options['stop_deceleration_rpm_s']/60)
        if not 1 <= value <= 0xFFFFFFFF:
            raise ControlError('独立限位停车减速度超出驱动参数范围。')
        return value

    def apply_profile(self):
        self.controller.validate_motion_profile(self.commands)
        return super().apply_profile()

    def travel_bounds(self, axis):
        d = axis['device']
        origin = axis.get('origin', d.position)
        margin = self.controller.options['position_margin_degrees'] * axis['counts']/360
        low, high = None, None
        envelope = envelope_bounds(getattr(self.commands, 'travel_envelope', None), d)
        if envelope:
            centre, minimum, maximum = envelope
            centre_delta = displacement(centre, origin)
            low, high = centre_delta+minimum+margin, centre_delta+maximum-margin
        memory = self.controller.limit_memory.entry(self.controller.adapter, d)
        if 'positive' in memory:
            edge = displacement(memory['positive'], origin)-margin
            high = edge if high is None else min(high, edge)
        if 'negative' in memory:
            edge = displacement(memory['negative'], origin)+margin
            low = edge if low is None else max(low, edge)
        if low is not None and high is not None and low >= high:
            raise ControlError(f'电机 {d.order} 软限位空间不足或坐标失效，禁止运动。')
        return low, high

    def strict_limits(self):
        return getattr(self.commands, 'stop_at_any_limit', False) or hasattr(self.commands, 'geometry')

    def blocked(self, axis, remaining):
        sensor = self.limits.get(axis['device'].order, {})
        if not sensor.get('valid'):
            raise ControlError('限位反馈失效，停止全部电机。')
        low, high = self.travel_bounds(axis)
        current = axis.get('unwrapped_counts', 0)
        goal = current+remaining
        return (sensor.get('positive_limit') and sensor.get('negative_limit')
                or sensor.get('positive_limit') and remaining > 0
                or sensor.get('negative_limit') and remaining < 0
                or high is not None and goal > high and remaining > 0
                or low is not None and goal < low and remaining < 0)

    def interrupt_motion(self, request=None):
        if self.strict_limits():
            return False
        if request is not None:
            for axis, degrees in zip(self.selected(), request.targets):
                goal = round(axis['counts'] * degrees * axis['direction_factor'] / 360)
                if self.blocked(axis, goal-axis['unwrapped_counts']):
                    self.limit_pending.add(axis['device'].order)
        if not self.limit_pending:
            return False
        orders = sorted(self.limit_pending)
        self.limit_pending.clear()
        self.limit_halted = True
        positions = []
        for axis in self.selected():
            axis['cw'] = 0x10F  # Halt=1; enable-operation stays set; no new-setpoint edge.
            axis['target'] = axis['last'][2]
            axis['command_start'] = axis['command_goal'] = axis['unwrapped_counts']
            axis.pop('velocity_samples', None)
            positions.append(self.travel_degrees(axis))
        self.commands.hold_at(positions)
        self.report['limit_holds'] = self.report.get('limit_holds', 0)+1
        conflict = any(self.limits[n]['positive_limit'] and self.limits[n]['negative_limit'] for n in orders)
        hint = '两端限位冲突，解除后需提交新目标' if conflict else '危险方向已锁存，可提交反向离开目标'
        self.emit('limit_blocked', limits=dict(self.limits),
            axes=[dict(order=a['device'].order, position=a['last'][2],
                       enabled=bool(a['last'][1] & 4), fault=bool(a['last'][1] & 8),
                       error_code=a['last'][0], travel_degrees=self.travel_degrees(a)
                       if 'origin' in a else None) for a in self.axes],
            text=f'电机 {orders} 限位触发；已暂停本次目标，保持使能；{hint}')
        return True

    def command(self, word):
        if word == 0x3F:
            # SV635N does not acknowledge a setpoint while Halt is asserted.
            # motion() has installed a fresh, direction-checked absolute target:
            # release Halt and raise new-setpoint in the SAME PDO, never resume
            # the discarded target first. tick() rechecks limits before sending.
            self.limit_halted = False
        super().command(word)

    def update_feedback(self, axis):
        super().update_feedback(axis)
        d = axis['device']
        raw = struct.unpack_from('<I', axis['slave'].input, 24)[0]
        sensor = self.controller.limit_memory.protect(self.controller.adapter, d, axis['last'][2],
                                                     input_state(d, raw, error_code=axis['last'][0]))
        self.limits[d.order] = sensor
        if self.cleaning or not axis['request']:
            return
        positive, negative = sensor['positive_limit'], sensor['negative_limit']
        if positive and negative:
            if not self.commands.ready or self.strict_limits():
                raise ControlError(f'电机 {d.order} 两端限位同时触发，停止全部电机。')
            if not self.limit_halted:
                self.limit_pending.add(d.order)
        if self.armed and (self.controller.options['limit_inputs_connected'] or d.endpoints):
            if self.strict_limits() and (positive or negative):
                raise ControlError(f'电机 {d.order} 返回固定中位期间触发限位。')
            if self.limit_halted:
                return
            # Holding at enable or after arrival can jitter by a few counts.
            # Only an outstanding motion target can drive into an endpoint;
            # the fresh-setpoint check in tick() still rejects any blocked goal.
            if not self.commands.ready or not axis['cw'] & 0x20:
                return
            remaining = axis.get('command_goal', 0) - axis.get('unwrapped_counts', 0)
            if (positive and remaining > axis['tolerance'] or
                    negative and remaining < -axis['tolerance']):
                self.limit_pending.add(d.order)
            # Predict the stop from measured encoder motion, including host/drive
            # reaction time and position margin. The drive's PP planner is the
            # normal decelerator; this is an independent early-stop backstop.
            samples = axis.setdefault('velocity_samples', deque())
            now = time.monotonic()
            samples.append((now, axis['unwrapped_counts']))
            while len(samples) > 2 and samples[1][0] < now-.02:
                samples.popleft()
            elapsed = now-samples[0][0]
            if elapsed >= .001 and abs(remaining) > axis['tolerance']:
                velocity = (axis['unwrapped_counts']-samples[0][1])/elapsed
                low, high = self.travel_bounds(axis)
                braking = stopping_distance(velocity, self.stop_deceleration(axis, 0),
                    self.controller.options['reaction_time_s'], 0,
                    acceleration=axis['report']['profile_acceleration_counts_s2'],
                    velocity_limit=axis['report']['profile_velocity_counts_s'])
                if (velocity > 0 and high is not None and axis['unwrapped_counts']+braking >= high
                        or velocity < 0 and low is not None and axis['unwrapped_counts']-braking <= low):
                    self.limit_pending.add(d.order)

    def tick(self, checked=True, cancellable=True):
        # Reject a forward target at an already-active endpoint before staging the PDO.
        if checked and self.armed:
            for axis in self.selected():
                if axis['cw'] & 0x10:
                    remaining = displacement(axis['target'], axis['last'][2])
                    if self.blocked(axis, remaining):
                        if self.strict_limits():
                            raise ControlError('目标指向已触发的限位。')
                        self.limit_pending.add(axis['device'].order)
            if self.limit_pending:
                if self.strict_limits():
                    raise ControlError('接近软限位制动边界，停止全部撑杆。')
                for axis in self.selected():
                    axis['cw'] = 0x10F
        super().tick(checked, cancellable)

    def emit(self, kind, **payload):
        if kind == 'status':
            payload['limits'] = dict(self.limits)
        elif kind == 'continuous_ready' and hasattr(self.commands, 'fixed_reference'):
            reference = self.commands.fixed_reference
            targets = []
            for axis in reference['axes']:
                current = next(a for a in self.selected() if a['device'].order == axis['order'])
                d = current['device']
                if list(d.identity) != axis['identity']:
                    raise ControlError('固定初始位与当前设备不匹配。')
                targets.append(displacement(axis['position'], current['origin']) * 360 / d.units_per_rev
                               * current['direction_factor'])
            self.commands.submit(targets)
        super().emit(kind, **payload)

    def cleanup(self):
        self.cleaning = True
        super().cleanup()
