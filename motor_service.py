"""Hardware ownership and lifecycle; socket/UI threads never access APS hardware."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import logging
import math
import threading
import time
import uuid

from control_common import ControlError, MotionQueue
from aps_backend import PCIe8332Controller
from platform_motion import PlatformGeometry, PlatformMotionQueue

LOG = logging.getLogger(__name__)
BUSY = ('scanning', 'enabling', 'enabled', 'stopping')


class MotorService:
    def __init__(self, adapter='PCIe-8332:0', log_dir=None, heartbeat_timeout=.5, controller_factory=None,
                 aps_options=None, platform_options=None, hardware_controller=None):
        if type(heartbeat_timeout) not in (float, int) or not math.isfinite(heartbeat_timeout) or not .1 <= heartbeat_timeout <= 5:
            raise ValueError('heartbeat_timeout 必须为 0.1～5 秒。')
        self.adapter, self.log_dir, self.timeout = adapter, log_dir, heartbeat_timeout
        self.controller_factory = controller_factory
        self.platform = PlatformGeometry(platform_options or {})
        self.control_mode = 'motor'
        self.platform_pose = None
        self.platform_lengths = []
        self.last_pose = 0
        self.tracking_since = None
        if hardware_controller is not None and controller_factory is not None:
            raise ValueError('Specify hardware_controller or controller_factory, not both')
        self.hardware = (hardware_controller if hardware_controller is not None else
                         None if controller_factory else PCIe8332Controller(adapter, log_dir, aps_options))
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='Motor-Hardware')
        self.devices = []
        self.commands = None
        self.stop_event = threading.Event()
        self.shutdown = threading.Event()
        self.phase, self.message = 'idle', '后台已启动，电机未使能'
        self.stop_reason = None
        self.run_id, self.sequence, self.last_heartbeat = None, -1, 0
        self.target_sequence = self.heartbeat_sequence = self.profile_sequence = -1
        self.applied_profile = None
        self.axes, self.result, self.orders = [], None, []
        self.limits = {}
        self.input_poll_pending = False
        self.monitor = threading.Thread(target=self._watchdog, name='Motor-Watchdog', daemon=True)
        self.monitor.start()

    def _controller(self):
        return self.hardware or self.controller_factory(self.adapter, self.log_dir)

    def snapshot(self):
        with self.lock:
            feedback = {a['order']: a for a in self.axes}
            profile = self.commands.profile() if self.commands else None
            return deepcopy(dict(phase=self.phase, message=self.message, adapter=self.adapter,
                control_mode=self.control_mode,
                platform=dict(configured=self.platform.config['enabled'],
                    calibration_id=self.platform.config['calibration_id'],
                    orders=self.platform.orders, pose=self.platform_pose,
                    lengths_mm=self.platform_lengths,
                    commanded_deg=list(self.commands.output) if isinstance(self.commands, PlatformMotionQueue) else [],
                    actual_lengths_mm=[(feedback[leg['order']]['travel_degrees'] * leg['mm_per_rev']
                        / 360 * leg['extension_sign'] if leg['order'] in feedback else None)
                        for leg in self.platform.legs],
                    pose_timeout_s=self.platform.config['pose_timeout_s'],
                    pose_age_s=(max(0., time.monotonic()-self.last_pose)
                                if self.control_mode == 'platform' and self.run_id else None)),
                hardware_backend=getattr(self.hardware, 'backend_name', 'aps') if self.hardware else 'injected',
                input_configuration=({key: self.hardware.options[key] for key in
                    ('limit_inputs_connected', 'emg_input_connected', 'extension_limits',
                     'retraction_limits')} if self.hardware else {}),
                run_id=self.run_id, seq=self.sequence, heartbeat_timeout_s=self.timeout,
                orders=self.orders, targets_deg=list(self.commands.planned) if self.commands else [],
                profile=(dict(revision=profile[0], rpm=profile[1], acceleration_rpm_s=profile[2])
                         if profile else None), applied_profile=self.applied_profile,
                limits=[dict(order=order, **sensor) for order, sensor in self.limits.items()],
                devices=[dict(order=d.order, id=d.alias, name=d.name, motor_code=d.motor_code,
                              positive_direction=d.positive_direction, error_code=d.error_code,
                              position=d.position, **({k: getattr(d, k) for k in
                                  ('axis_id', 'slave_id', 'units_per_rev')} if hasattr(d, 'axis_id') else {}))
                         for d in self.devices], axes=self.axes,
                enabled=bool(self.orders and all(n in feedback and feedback[n]['enabled'] for n in self.orders)),
                result=self.result, stop_reason=self.stop_reason))

    def is_busy(self):
        with self.lock:
            return self.phase in BUSY

    def adapters(self):
        if self.hardware:
            adapters = self.hardware.adapters()
        else:
            from core import EtherCATController
            adapters = EtherCATController.adapters()
        return [dict(name=name, description=description) for name, description in adapters]

    def scan(self, adapter=None):
        with self.lock:
            if self.shutdown.is_set() or self.phase in BUSY:
                raise ControlError('后台忙碌，请先停止本次运行。')
            if adapter is not None:
                if not isinstance(adapter, str) or not adapter or len(adapter) > 512:
                    raise ControlError('控制卡名称无效。')
                if self.hardware and adapter != self.hardware.adapter:
                    raise ControlError('控制卡与后台 aps.board_id 配置不一致。')
                self.adapter = adapter
            self.phase, self.message = 'scanning', f'正在连接并刷新控制卡 {self.adapter}，扫描电机，不使能'
            self.devices, self.axes, self.orders, self.result = [], [], [], None
            self.limits = {}
            self.executor.submit(self._scan)

    def _scan(self):
        try:
            devices = self._controller().scan()
            limits = self.hardware.read_inputs(devices) if self.hardware else {}
            with self.lock:
                self.devices = devices
                self.limits = limits
                self.phase, self.message = 'idle', f'扫描到 {len(devices)} 台电机'
            LOG.info('Scan completed: %s devices', len(devices))
        except Exception as exc:
            if self.hardware:
                errors = self.hardware.close()
                if errors:
                    LOG.error('Scan cleanup errors: %s', errors)
            with self.lock:
                self.phase, self.message = 'fault', str(exc)
            LOG.exception('Scan failed')

    def enable(self, orders, rpm=60, acceleration=120, *, mode='motor', reference_confirmed=False,
               calibration_id=None):
        with self.lock:
            if self.shutdown.is_set() or self.phase in BUSY:
                raise ControlError('后台忙碌，请等待停止和参数恢复完成。')
            if self.phase == 'fault':
                raise ControlError('故障已锁定，请核对设备并重新扫描后再开启。')
            if not isinstance(orders, list) or not orders or any(type(n) is not int for n in orders):
                raise ControlError('orders 必须为非空链路位置整数数组。')
            if mode not in ('motor', 'platform'):
                raise ControlError('Unknown control mode')
            if mode == 'platform':
                if not self.platform.config['enabled']:
                    raise ControlError('Platform mode is not configured')
                if reference_confirmed is not True or calibration_id != self.platform.config['calibration_id']:
                    raise ControlError('Confirm physical neutral and the configured calibration_id for each run')
                if orders != self.platform.orders:
                    raise ControlError('Platform requires all three configured supports in order')
                known_platform = {d.order: d for d in self.devices}
                for leg in self.platform.legs:
                    device = known_platform.get(leg['order'])
                    if (device and hasattr(device, 'extension_sign') and device.extension_sign
                            and leg['extension_sign'] != device.extension_sign * (1 if device.positive_direction else -1)):
                        raise ControlError('Platform extension_sign disagrees with configured drive endpoint direction')
                if self.hardware and self.platform.config['require_endpoint_sensors']:
                    options = self.hardware.options
                    selected = {d.order: str(d.axis_id) for d in self.devices}
                    if (not options['limit_inputs_connected'] or any(
                        selected.get(order) not in options[key] for order in orders
                        for key in ('extension_limits', 'retraction_limits'))):
                        raise ControlError('Platform requires configured dual endpoint sensors on all supports')
                commands = PlatformMotionQueue(self.platform)
            else:
                if self.platform.config['enabled'] and not self.platform.config['allow_motor_debug']:
                    raise ControlError('Direct motor debug is disabled for this configured platform')
                commands = MotionQueue(orders, rpm, acceleration)
            known = {d.order: d for d in self.devices}
            if any(n not in known for n in orders):
                raise ControlError('请先扫描并使用扫描结果中的链路位置。')
            for n in orders:
                device = known[n]
                if device.motor_code != 14101:
                    raise ControlError(f'电机 {n} 型号 {device.motor_code} 不支持，需要 SV635N / 14101。')
                # APS worker rescans and validates current drive alarms before
                # enabling ANY axis. A previous scan's alarm can already be gone.
                if not self.hardware and device.error_code:
                    raise ControlError(f'电机 {n} 驱动报警 0x{device.error_code:04X}；请处理报警后重新扫描。')
            self.commands, self.orders = commands, list(orders)
            self.control_mode = mode
            self.platform_pose, self.platform_lengths = None, []
            self.last_pose = time.monotonic()
            self.tracking_since = None
            self.stop_event = threading.Event()
            self.run_id, self.sequence = uuid.uuid4().hex, -1
            self.target_sequence = self.heartbeat_sequence = self.profile_sequence = -1
            self.applied_profile = None
            self.last_heartbeat = time.monotonic()
            self.phase, self.message = 'enabling', '正在使能，保持当前位置'
            self.axes, self.result = [], None
            self.stop_reason = None
            self.executor.submit(self._run, commands, self.stop_event, list(self.devices))
            LOG.info('Enable run=%s orders=%s rpm=%s acceleration=%s', self.run_id, orders, rpm, acceleration)
            return self.run_id

    def _event(self, event):
        with self.lock:
            if event['kind'] == 'continuous_ready':
                if self.phase == 'enabling' and not self.stop_event.is_set():
                    self.phase, self.message = 'enabled', '连续使能就绪'
                    self.last_pose = time.monotonic()
            elif event['kind'] == 'status':
                self.axes = event['axes']
                if 'limits' in event:
                    self.limits = event['limits']
            elif event['kind'] == 'profile_applied':
                self.applied_profile = {k: event[k] for k in ('revision', 'rpm', 'acceleration_rpm_s')}
            elif event['kind'] == 'limit_blocked':
                self.message = event['text']
                if 'limits' in event:
                    self.limits = event['limits']
                if self.control_mode == 'platform':
                    self.stop('平台端点限位触发，停止全部撑杆')
            elif event['kind'] == 'phase' and self.phase != 'stopping':
                self.message = event['text']
            if self.control_mode == 'platform' and self.phase in ('enabling', 'enabled'):
                for order in self.orders:
                    sensor = self.limits.get(order, {})
                    if (sensor.get('triggered') or sensor.get('retraction_triggered')
                            or sensor.get('conflict') or
                            ((sensor.get('input') or sensor.get('retraction_input')) and not sensor.get('valid'))):
                        self.stop('平台限位触发或反馈无效，停止全部撑杆')
                        break
                if event['kind'] == 'status' and self.phase == 'enabled':
                    feedback = {a['order']: a for a in self.axes}
                    error = any(leg['order'] not in feedback or abs(
                        (feedback[leg['order']]['travel_degrees'] - self.commands.output[index])
                        * leg['mm_per_rev'] / 360) > self.platform.config['max_tracking_error_mm']
                        for index, leg in enumerate(self.platform.legs))
                    now = time.monotonic()
                    if error:
                        if self.tracking_since is None:
                            self.tracking_since = now
                        elif now - self.tracking_since >= self.platform.config['tracking_timeout_s']:
                            self.stop('平台撑杆跟随误差持续超限，停止全部撑杆')
                    else:
                        self.tracking_since = None

    def _run(self, commands, stop, devices):
        try:
            controller = self._controller()
            if hasattr(controller, 'run_continuous'):
                report = controller.run_continuous(devices, commands, stop, self._event)
            else:
                # Legacy direct-NIC controllers remain usable in standalone tools/tests.
                from continuous_control import run_continuous
                report = run_continuous(controller, devices, commands, stop, self._event)
        except Exception as exc:
            LOG.exception('Motor worker failed')
            report = dict(error=str(exc), all_disabled=False)
        with self.lock:
            self.result = {k: report.get(k) for k in ('error', 'stopped', 'all_disabled',
                'cleanup_errors', 'log_path', 'pdo_failed_exchanges', 'pdo_recoveries', 'maximum_pdo_recovery_ms')}
            self.result['stop_reason'] = self.stop_reason
            clean = bool(report.get('all_disabled') and not report.get('cleanup_errors'))
            self.phase = 'idle' if clean and report.get('stopped') else 'fault'
            if clean:
                for a in self.axes:
                    a['enabled'] = False
            self.message = ('已停止并核对关闭使能' if self.phase == 'idle' else
                            '运行故障：' + str(report.get('error') or report.get('cleanup_errors') or '未核对关闭使能'))
            commands.close()
        LOG.info('Run finished phase=%s result=%s', self.phase, self.result)

    def command(self, run_id, sequence, targets=None, profile=None, pose=None):
        with self.lock:
            if not isinstance(run_id, str) or run_id != self.run_id:
                raise ControlError('run_id 已失效，请显式开启新运行。')
            if type(sequence) is not int or not 0 <= sequence <= 2**53 - 1:
                raise ControlError('seq 必须为 0～2^53-1 的整数。')
            if self.phase not in ('enabling', 'enabled') or self.stop_event.is_set():
                raise ControlError('本次运行已停止或未开启，不能续期或更新目标。')
            if time.monotonic() - self.last_heartbeat >= self.timeout:
                self.stop('UDP 心跳超时')
                raise ControlError('UDP 心跳已超时，不能恢复本次运行。')
            if (self.control_mode == 'platform' and self.phase == 'enabled'
                    and time.monotonic() - self.last_pose >= self.platform.config['pose_timeout_s']):
                self.stop('平台姿态流超时')
                raise ControlError('Platform pose stream expired; explicitly arm a new run')
            previous = (self.profile_sequence if profile is not None else
                        self.heartbeat_sequence if targets is None and pose is None else self.target_sequence)
            if sequence <= previous:
                return False
            platform_values = None
            if pose is not None:
                if self.control_mode != 'platform' or targets is not None or profile is not None:
                    raise ControlError('Platform pose requires a platform run')
                if self.phase != 'enabled':
                    raise ControlError('Wait for platform enable before sending poses')
                try:
                    platform_values, lengths, targets = self.platform.pose_to_targets(pose)
                except ControlError:
                    self.stop('平台目标无效或超出标定范围')
                    raise
            elif self.control_mode == 'platform' and (targets is not None or profile is not None):
                raise ControlError('Platform run accepts poses only; raw targets/profiles cannot bypass limits')
            if profile is not None:
                if self.phase != 'enabled':
                    raise ControlError('使能尚未就绪，不能更新速度和加速度。')
                self.commands.update_profile(profile['rpm'], profile['acceleration_rpm_s'])
            if targets is not None:
                if self.phase != 'enabled':
                    raise ControlError('使能尚未就绪，请继续心跳并等待 enabled 状态。')
                if not isinstance(targets, list):
                    raise ControlError('targets_deg 必须是数组。')
                feedback = {a['order']: a for a in self.axes}
                for order, target in zip(self.orders, targets):
                    sensor = self.limits.get(order, {})
                    if sensor.get('input') or sensor.get('retraction_input'):
                        if not sensor.get('valid'):
                            raise ControlError(f'电机 {order} 端点限位反馈失效。')
                        if sensor.get('conflict'):
                            raise ControlError(f'电机 {order} 两端限位同时触发，禁止运动。')
                        axis = feedback.get(order)
                        device = next(d for d in self.devices if d.order == order)
                        for prefix, label, _, direction in device.endpoints:
                            sign = direction * (1 if device.positive_direction else -1)
                            if (sensor.get('triggered' if prefix == 'extension' else 'retraction_triggered')
                                    and axis and type(target) in (int, float)
                                    and (target - axis['travel_degrees']) * sign > 0):
                                allowed = '缩回' if prefix == 'extension' else '推出'
                                raise ControlError(f'电机 {order} {label}触发，只允许{allowed}方向的目标。')
                if tuple(targets) != self.commands.planned:
                    self.commands.submit(targets)
                else:
                    # Validate even unchanged targets (bool is not an angle).
                    if len(targets) != len(self.orders) or any(type(x) not in (int, float) or not math.isfinite(x) for x in targets):
                        raise ControlError('目标必须是每个选中电机的有限数值。')
            if profile is not None:
                self.profile_sequence = sequence
            elif targets is None:
                self.heartbeat_sequence = sequence
            else:
                self.target_sequence = sequence
            if platform_values is not None:
                self.platform_pose, self.platform_lengths = platform_values, lengths
                self.last_pose = time.monotonic()
            self.sequence, self.last_heartbeat = max(self.sequence, sequence), time.monotonic()
            return True

    def stop(self, reason='请求停止'):
        with self.lock:
            if self.phase in ('enabling', 'enabled'):
                self.stop_reason = reason
                self.phase, self.message = 'stopping', reason + '；正在停止并恢复参数'
                self.stop_event.set()
                if self.commands:
                    self.commands.close()
                LOG.info('Stop requested: %s', reason)

    def _watchdog(self):
        last_inputs = 0
        while not self.shutdown.wait(.02):
            with self.lock:
                if self.phase in ('enabling', 'enabled') and time.monotonic() - self.last_heartbeat >= self.timeout:
                    self.stop('UDP 心跳超时')
                if (self.control_mode == 'platform' and self.phase == 'enabled'
                        and time.monotonic() - self.last_pose >= self.platform.config['pose_timeout_s']):
                    self.stop('平台姿态流超时')
                if (self.hardware and self.devices and self.phase not in BUSY
                        and not self.input_poll_pending and time.monotonic() - last_inputs >= .1):
                    self.input_poll_pending = True
                    last_inputs = time.monotonic()
                    self.executor.submit(self._poll_inputs)

    def _poll_inputs(self):
        try:
            with self.lock:
                if self.shutdown.is_set() or self.phase in BUSY:
                    return
                devices = list(self.devices)
            limits = self.hardware.read_inputs(devices)
            with self.lock:
                self.limits = limits
        except Exception as exc:
            with self.lock:
                self.limits = {d.order: dict(input=d.extension_input or None, valid=False,
                    triggered=None, state='unavailable', error=str(exc),
                    retraction_input=d.retraction_input or None, retraction_triggered=None,
                    retraction_state='unavailable' if d.retraction_input else 'no_sensor')
                    for d in self.devices}
        finally:
            with self.lock:
                self.input_poll_pending = False

    def close(self):
        self.shutdown.set()
        self.stop('后台退出')
        self.monitor.join(timeout=1)
        if self.hardware:
            def close_hardware():
                errors = self.hardware.close()
                if errors:
                    LOG.error('APS shutdown errors: %s', errors)
            self.executor.submit(close_hardware)
        # Keep the process alive until the hardware worker has attempted stop,
        # verified disable and restored parameters. Never abandon this worker.
        self.executor.shutdown(wait=True)
