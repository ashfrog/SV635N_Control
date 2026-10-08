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

LOG = logging.getLogger(__name__)
BUSY = ('scanning', 'enabling', 'enabled', 'stopping')


class MotorService:
    def __init__(self, adapter='PCIe-8332:0', log_dir=None, heartbeat_timeout=.5, controller_factory=None,
                 aps_options=None):
        if type(heartbeat_timeout) not in (float, int) or not math.isfinite(heartbeat_timeout) or not .1 <= heartbeat_timeout <= 5:
            raise ValueError('heartbeat_timeout 必须为 0.1～5 秒。')
        self.adapter, self.log_dir, self.timeout = adapter, log_dir, heartbeat_timeout
        self.controller_factory = controller_factory
        self.hardware = None if controller_factory else PCIe8332Controller(adapter, log_dir, aps_options)
        self.lock = threading.RLock()
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='Motor-Hardware')
        self.devices = []
        self.commands = None
        self.stop_event = threading.Event()
        self.shutdown = threading.Event()
        self.phase, self.message = 'idle', '后台已启动，电机未使能'
        self.stop_reason = None
        self.run_id, self.sequence, self.last_heartbeat = None, -1, 0
        self.target_sequence = self.heartbeat_sequence = -1
        self.axes, self.result, self.orders = [], None, []
        self.monitor = threading.Thread(target=self._watchdog, name='Motor-Watchdog', daemon=True)
        self.monitor.start()

    def _controller(self):
        return self.hardware or self.controller_factory(self.adapter, self.log_dir)

    def snapshot(self):
        with self.lock:
            feedback = {a['order']: a for a in self.axes}
            return deepcopy(dict(phase=self.phase, message=self.message, adapter=self.adapter,
                hardware_backend='aps' if self.hardware else 'injected',
                input_configuration=({key: self.hardware.options[key] for key in
                    ('limit_inputs_connected', 'emg_input_connected')} if self.hardware else {}),
                run_id=self.run_id, seq=self.sequence, heartbeat_timeout_s=self.timeout,
                orders=self.orders, targets_deg=list(self.commands.planned) if self.commands else [],
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
            self.phase, self.message = 'scanning', '正在扫描电机，不使能'
            self.devices, self.axes, self.orders, self.result = [], [], [], None
            self.executor.submit(self._scan)

    def _scan(self):
        try:
            devices = self._controller().scan()
            with self.lock:
                self.devices = devices
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

    def enable(self, orders, rpm=60, acceleration=120):
        with self.lock:
            if self.shutdown.is_set() or self.phase in BUSY:
                raise ControlError('后台忙碌，请等待停止和参数恢复完成。')
            if self.phase == 'fault':
                raise ControlError('故障已锁定，请核对设备并重新扫描后再开启。')
            if not isinstance(orders, list) or not orders or any(type(n) is not int for n in orders):
                raise ControlError('orders 必须为非空链路位置整数数组。')
            commands = MotionQueue(orders, rpm, acceleration)
            known = {d.order: d for d in self.devices}
            if any(n not in known for n in orders):
                raise ControlError('请先扫描并使用扫描结果中的链路位置。')
            if any(known[n].motor_code != 14101 or known[n].error_code for n in orders):
                raise ControlError('所选电机的型号或报警状态未通过核对。')
            self.commands, self.orders = commands, list(orders)
            self.stop_event = threading.Event()
            self.run_id, self.sequence = uuid.uuid4().hex, -1
            self.target_sequence = self.heartbeat_sequence = -1
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
            elif event['kind'] == 'status':
                self.axes = event['axes']
            elif event['kind'] == 'phase' and self.phase != 'stopping':
                self.message = event['text']

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

    def command(self, run_id, sequence, targets=None):
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
            previous = self.heartbeat_sequence if targets is None else self.target_sequence
            if sequence <= previous:
                return False
            if targets is not None:
                if self.phase != 'enabled':
                    raise ControlError('使能尚未就绪，请继续心跳并等待 enabled 状态。')
                if not isinstance(targets, list):
                    raise ControlError('targets_deg 必须是数组。')
                if tuple(targets) != self.commands.planned:
                    self.commands.submit(targets)
                else:
                    # Validate even unchanged targets (bool is not an angle).
                    if len(targets) != len(self.orders) or any(type(x) not in (int, float) or not math.isfinite(x) for x in targets):
                        raise ControlError('目标必须是每个选中电机的有限数值。')
            if targets is None:
                self.heartbeat_sequence = sequence
            else:
                self.target_sequence = sequence
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
        while not self.shutdown.wait(.02):
            with self.lock:
                if self.phase in ('enabling', 'enabled') and time.monotonic() - self.last_heartbeat >= self.timeout:
                    self.stop('UDP 心跳超时')

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
