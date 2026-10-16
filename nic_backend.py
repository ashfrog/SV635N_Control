"""SV635N backend using the motherboard NIC, Npcap and the existing SOEM PP loop."""
from dataclasses import asdict, dataclass, replace
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
from nic_safety import (MAX_MOTOR_RPM, LimitMemory, envelope_bounds, manual_bounds, stopping_distance)
from ethercat_timing import DCPhaseClock, cyclic_worker


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
        if rpm > MAX_MOTOR_RPM:
            raise ControlError(f'电机角度调试速度最高 {MAX_MOTOR_RPM:g} rpm。')
        if envelope is not None:
            maximum = envelope.get('max_speed_rpm')
            if type(maximum) not in (int, float) or not math.isfinite(maximum) or maximum <= 0:
                raise ControlError('软限位缺少已标定的运行速度上限。')
            if rpm > maximum:
                raise ControlError(f'速度超过已标定机构上限 {maximum:g} rpm。')

    def limit_settings(self, devices, feedback):
        axes = []
        for device in devices:
            entry = self.limit_memory.axes.get(f'{self.adapter}:{device.order}', {})
            identity_matches = not entry or entry.get('identity') == list(device.identity)
            manual = entry.get('manual', {}) if identity_matches else {}
            current = feedback.get(device.order, {}).get('position', device.position)
            origin = manual.get('origin', current)
            factor = 1 if device.positive_direction else -1
            axes.append(dict(order=device.order, position=current, origin=origin,
                current_degrees=displacement(current, origin)*360/device.units_per_rev*factor,
                enabled=manual.get('enabled', False), lower_degrees=manual.get('lower_degrees'),
                upper_degrees=manual.get('upper_degrees'), learned_positive=entry.get('positive'),
                learned_negative=entry.get('negative'), identity_matches=identity_matches))
        return dict(available=True, axes=axes)

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
                    position = None
                    try:
                        slave = master.slaves[d.slave_id]
                        sensor = input_state(d, read(slave, 0x60FD), error_code=read(slave, 0x603F))
                        position = read(slave, 0x6064, signed=True)
                        result[d.order] = self.limit_memory.protect(
                            self.adapter, d, position, sensor)
                    except Exception as exc:
                        result[d.order] = input_state(d, error=exc)
                    if position is not None:
                        result[d.order]['position'] = position
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

    def run_auto_center(self,devices,commands,options,stop,callback):
        from auto_center_session import AutoCenterSession
        try:
            with adapter_lock(self.adapter):
                return AutoCenterSession(self,devices,commands,stop,callback,options=options).run()
        finally:
            commands.close()

    def close(self):
        # Each scan/input read/session owns and closes its own SOEM master.
        return []


class _NICSession(_ManualSession):
    # SV635N manual section 12.3: 125 us and its integer multiples.
    # PP trajectory runs in the drive; 250 Hz feedback/handshakes leave the
    # Windows worker more scheduling headroom than the former 1 kHz loop.
    # The independent 10 ms PDO recovery budget and stop guards are unchanged.
    cycle_ns = 4_000_000

    def __init__(self, *args):
        super().__init__(*args)
        self.limits = {}
        self.cleaning = False
        self.recovering_communication = True
        self.communication_reset_orders = set()
        self.limit_pending = set()
        self.limit_halted = False
        self.deferred_reverse = None
        self.dc_clock = DCPhaseClock(self.cycle_ns)
        self.network_baseline = {}
        self.report.update(backend='ethercat', control_source='udp', bus_cycle_ms=self.cycle_ns/1_000_000)

    def run(self):
        with cyclic_worker(self.master, self.report):
            return super().run()

    def cycle_adjustment_ns(self):
        adjustment = self.dc_clock.correction(getattr(self.master, 'dc_time', 0))
        self.report['dc_phase_error_ns'] = self.dc_clock.phase_error
        return adjustment

    def retry_pause(self, started, cancellable):
        # Retries retain the setpoint edge, at most once per 1 ms, within the
        # independent 10 ms recovery budget even with a slower normal cycle.
        remaining = started+.001-time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        if cancellable:
            self.check_stop()

    def poll_network_state(self, elapsed):
        # Healthy PDO WKC + per-axis CiA402 enable/fault feedback are checked on
        # every cycle. A blocking read_state transaction must not interrupt DC
        # traffic every 50 ms. OP is explicitly verified during setup; failure
        # diagnostics read AL state only after the stopping PDOs have been sent.
        self.last_state = elapsed

    def capture_network_baseline(self):
        for axis in self.axes:
            try:
                raw = bytes(axis['slave']._fprd(0x0300,20))
                if len(raw) != 20:
                    raise ValueError('ESC counter snapshot length mismatch')
                self.network_baseline[axis['device'].order] = raw
                axis['report']['esc_error_baseline_0300_0313'] = raw.hex()
            except Exception as exc:
                axis['report']['network_baseline_error'] = str(exc)

    def network_diagnostics(self):
        details = []
        try:
            self.master.read_state()
            for axis in self.axes:
                slave = axis['slave']
                item = dict(order=axis['device'].order, id=axis['device'].alias,
                            state=slave.state, al_status=slave.al_status)
                try:
                    raw=slave._fprd(0x0300,20)
                    item['esc_error_registers_0300_0313'] = raw.hex()
                    baseline = self.network_baseline.get(axis['device'].order)
                    item['ports'] = []
                    for p in range(4):
                        port = dict(port=p)
                        offsets = dict(invalid_frames=p*2,physical_rx_errors=p*2+1,
                                       forwarded_rx_errors=8+p,lost_links=16+p)
                        for name,offset in offsets.items():
                            port[name] = raw[offset]
                            # Saturated/reset/missing baselines cannot prove a
                            # zero increase; preserve that uncertainty as null.
                            port['added_'+name] = (raw[offset]-baseline[offset] if baseline is not None
                                and baseline[offset] <= raw[offset] < 255 else None)
                        port['counter_reset_detected'] = bool(baseline is not None and
                            any(raw[o] < baseline[o] for o in offsets.values()))
                        port['counter_saturated'] = any(raw[o] == 255 or baseline is not None and
                            baseline[o] == 255 for o in offsets.values())
                        item['ports'].append(port)
                except Exception as exc:
                    item['counter_read_error'] = str(exc)
                details.append(item)
        except Exception as exc:
            self.report['network_diagnostic_error'] = str(exc)
        self.report['network_diagnostics'] = details

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
            if self.recovering_communication and not self.armed:
                if code == 0x0E08 or code == 0 and axis['device'].order in self.communication_reset_orders:
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
            self.communication_reset_orders = {a['device'].order for a in affected}
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
        self.communication_reset_orders.clear()

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
                tolerance=getattr(self.commands,'neutral_tolerance_degrees',{}).get(d.order,.2)
                if (list(d.identity) != axis['identity'] or
                        abs(displacement(d.position, axis['position'])) > max(2., d.units_per_rev*tolerance/360)):
                    raise ControlError('三轴尚未回到保存的固定初始位，请先返回固定初始位。')

    def additional_parameters(self, axis):
        # SV635N manual p163 recommends H02.07=1 for a vertical axis: the
        # physical overtravel input commands zero speed and retains position
        # lock. Normal predictive software Halt still uses controlled 6085.
        return [(0x605D, 2, 2, True), (0x2002, 2, 1, False, 8)]

    def stop_deceleration(self, axis, acceleration):
        if acceleration == 0 and 'stop_deceleration_counts_s2' in axis:
            return axis['stop_deceleration_counts_s2']
        value = round(axis['counts'] * self.controller.options['stop_deceleration_rpm_s']/60)
        # A high-speed normal PP ramp must not be replaced by a much slower
        # limit-stop ramp. The configured value is a minimum, so lowering the
        # UI acceleration still cannot weaken the stopping floor.
        value = max(value, acceleration)
        if self.limit_halted and not self.stopped_after_limit():
            value = max(value, axis.get('stop_deceleration_counts_s2', 0))
        if not 1 <= value <= 0xFFFFFFFF:
            raise ControlError('独立限位停车减速度超出驱动参数范围。')
        axis['stop_deceleration_counts_s2'] = value
        axis['report']['stop_deceleration_counts_s2'] = value
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
        manual = manual_bounds(memory.get('manual'), d)
        if manual:
            centre, minimum, maximum = manual
            centre_delta = displacement(centre, origin)
            low = centre_delta+minimum if low is None else max(low, centre_delta+minimum)
            high = centre_delta+maximum if high is None else min(high, centre_delta+maximum)
        if 'positive' in memory:
            edge = displacement(memory['positive'], origin)-margin
            high = edge if high is None else min(high, edge)
        if 'negative' in memory:
            edge = displacement(memory['negative'], origin)+margin
            low = edge if low is None else max(low, edge)
        if low is not None and high is not None and low >= high:
            if not self.commands.ready:
                raise ControlError(f'电机 {d.order} 限位位置记录冲突，请停机后“重置限位记录”或重新设置软限位。')
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
        return (low is None or high is None or low >= high
                or axis.get('hold_direction', 0)*remaining > 0
                or sensor.get('positive_limit') and sensor.get('negative_limit')
                or sensor.get('positive_limit') and remaining > 0
                or sensor.get('negative_limit') and remaining < 0
                or high is not None and goal > high and remaining > 0
                or low is not None and goal < low and remaining < 0)

    def stopped_after_limit(self):
        for axis in self.selected():
            samples = axis.get('halt_samples', ())
            if (len(samples) < 2 or samples[-1][0]-samples[0][0] < .05
                    or not axis['last'][1] & 0x400
                    or max(p for _, p in samples)-min(p for _, p in samples) > max(2, axis['counts']*.02/360)):
                return False
        return True

    def next_request(self):
        request = self.commands.pop()
        if not self.limit_halted:
            return request
        if request is not None:
            # Only a fresh, permitted target can survive a limit stop. A newer
            # outward request also cancels a previously queued reverse request.
            if any(self.blocked(a, round(a['counts']*v*a['direction_factor']/360)-a['unwrapped_counts'])
                   for a, v in zip(self.selected(), request.targets)):
                self.deferred_reverse = None
                return request
            self.deferred_reverse = request
        if self.deferred_reverse is None or not self.stopped_after_limit():
            return None
        request, self.deferred_reverse = self.deferred_reverse, None
        return request

    def prepare_request(self, request):
        if self.strict_limits():
            return request
        targets, changed = [], False
        for axis, degrees in zip(self.selected(), request.targets):
            goal = round(axis['counts']*degrees*axis['direction_factor']/360)
            current = axis['unwrapped_counts']
            low, high = self.travel_bounds(axis)
            if low is not None and high is not None and low >= high:
                return request  # Keep enable and hold; do not throw or clamp an invalid interval.
            # The user's slider stays at the requested position. Only the
            # drive's executable endpoint is bounded, so a far target moves
            # smoothly toward the boundary instead of aborting where it is.
            bounded = goal
            if high is not None and current < high and goal > high:
                bounded = math.floor(high)
            if low is not None and current > low and goal < low:
                bounded = math.ceil(low)
            changed |= bounded != goal
            targets.append(bounded*360/axis['counts']*axis['direction_factor'])
        if changed:
            self.report['target_clamps'] = self.report.get('target_clamps', 0)+1
            if not self.limit_halted and all(round(a['counts']*value*a['direction_factor']/360)
                    == a.get('command_goal') for a, value in zip(self.selected(), targets)):
                return None  # Do not restart the same bounded PP trajectory on every mouse event.
            return replace(request, targets=tuple(targets))
        return request

    def interrupt_motion(self, request=None):
        if self.strict_limits():
            return False
        if request is not None:
            for axis, degrees in zip(self.selected(), request.targets):
                goal = round(axis['counts'] * degrees * axis['direction_factor'] / 360)
                if self.blocked(axis, goal-axis['unwrapped_counts']):
                    axis['stop_direction'] = 1 if goal > axis['unwrapped_counts'] else -1
                    self.limit_pending.add(axis['device'].order)
        if not self.limit_pending:
            return False
        orders = sorted(self.limit_pending)
        self.limit_pending.clear()
        already_halted = self.limit_halted
        self.limit_halted = True
        self.deferred_reverse = None
        positions = []
        for axis in self.selected():
            if axis['device'].order in orders:
                direction = axis.pop('stop_direction', 0)
                if not direction:
                    sensor = self.limits[axis['device'].order]
                    if sensor['positive_limit'] != sensor['negative_limit']:
                        direction = 1 if sensor['positive_limit'] else -1
                if direction:
                    axis['hold_direction'] = direction
            if already_halted:
                continue  # Never make the stop target follow ongoing coast motion.
            axis['cw'] = 0x10F  # Halt=1; enable-operation stays set; no new-setpoint edge.
            axis['target'] = axis['last'][2]
            axis['command_start'] = axis['command_goal'] = axis['unwrapped_counts']
            axis.pop('velocity_samples', None)
            axis['halt_samples'] = deque([(time.monotonic(), axis['unwrapped_counts'])])
            positions.append(self.travel_degrees(axis))
        if already_halted:
            return True
        self.commands.hold_at(positions, keep_requested=True)
        self.report['limit_holds'] = self.report.get('limit_holds', 0)+1
        conflict = any(self.limits[n]['positive_limit'] and self.limits[n]['negative_limit'] for n in orders)
        hint = '两端限位冲突，解除后需提交新目标' if conflict else '危险方向已锁存，可提交反向离开目标'
        if any(low is None or high is None for low, high in (self.travel_bounds(a) for a in self.selected())):
            hint = '缺少完整软行程，未执行运动；请关闭使能后设置实测软上限和下限'
        if any(low is not None and high is not None and low >= high
               for low, high in (self.travel_bounds(a) for a in self.selected())):
            hint = '限位位置记录冲突，请关闭使能后重置限位记录或重新设置软限位'
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
            for axis in self.selected():
                if axis.get('hold_direction', 0)*(axis['command_goal']-axis['unwrapped_counts']) < 0:
                    axis.pop('hold_direction', None)
        super().command(word)

    def update_feedback(self, axis):
        super().update_feedback(axis)
        d = axis['device']
        raw = struct.unpack_from('<I', axis['slave'].input, 24)[0]
        sensor = self.controller.limit_memory.protect(self.controller.adapter, d, axis['last'][2],
                                                     input_state(d, raw, error_code=axis['last'][0]))
        self.limits[d.order] = sensor
        if self.limit_halted and axis['request']:
            samples = axis.setdefault('halt_samples', deque())
            now = time.monotonic()
            samples.append((now, axis['unwrapped_counts']))
            while len(samples) > 2 and samples[1][0] < now-.05:
                samples.popleft()
        if self.cleaning or not axis['request']:
            return
        positive, negative = sensor['positive_limit'], sensor['negative_limit']
        if positive and negative:
            if not self.commands.ready or self.strict_limits():
                self.travel_bounds(axis)  # Identify conflicting saved coordinates before physical-input errors.
                raise ControlError(f'电机 {d.order} 两端限位同时触发，停止全部电机。')
            if not self.limit_halted:
                self.limit_pending.add(d.order)
        if self.armed and (self.controller.options['limit_inputs_connected'] or d.endpoints):
            if self.strict_limits() and (positive or negative):
                raise ControlError(f'电机 {d.order} 返回固定中位期间触发限位。')
            if self.limit_halted:
                return
            low, high = self.travel_bounds(axis)
            if low is not None and high is not None and low >= high:
                self.limit_pending.add(d.order)
                return
            # Holding at enable or after arrival can jitter by a few counts.
            # Only an outstanding motion target can drive into an endpoint;
            # the fresh-setpoint check in tick() still rejects any blocked goal.
            if not self.commands.ready or not axis['cw'] & 0x20:
                return
            remaining = axis.get('command_goal', 0) - axis.get('unwrapped_counts', 0)
            if (positive and remaining > axis['tolerance'] or
                    negative and remaining < -axis['tolerance']):
                axis['stop_direction'] = 1 if remaining > 0 else -1
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
                    axis['stop_direction'] = 1 if velocity > 0 else -1
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
        if kind=='continuous_ready' and hasattr(self.commands,'neutral_reference'):
            # Permit small stationary centre offsets without redefining zero or
            # expanding native travel bounds. No setpoint is issued here.
            positions=[]
            for row in self.commands.neutral_reference['axes']:
                axis=next(a for a in self.selected() if a['device'].order==row['order'])
                offset=displacement(axis['last'][2],row['position'])
                axis.update(origin=row['position'],previous_position=axis['last'][2],
                    unwrapped_counts=offset,command_start=offset,command_goal=offset)
                positions.append(self.travel_degrees(axis))
            self.commands.hold_at(positions)
            if hasattr(self.commands,'output'):
                with self.commands.lock:
                    self.commands.output=tuple(positions)
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
