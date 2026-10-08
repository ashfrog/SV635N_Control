"""Local persistent PP control: enable once, execute bounded FIFO commands.

Only the worker touches EtherCAT. The GUI edits and queues immutable targets.
No network transport or heartbeat is needed for an idle manual session.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading

from core import ControlError, Move, Stopped, _Session, adapter_lock, displacement


@dataclass(frozen=True)
class MotionCommand:
    number: int
    targets: tuple[float, ...]
    estimated_seconds: float


class MotionQueue:
    def __init__(self, orders, limit_degrees=360.0, rpm=5.0, acceleration=10.0):
        self.orders = tuple(orders)
        if (not self.orders or len(set(self.orders)) != len(self.orders)
                or self.orders != tuple(sorted(self.orders))):
            raise ControlError('请选择不同的电机，按链路位置升序排列。')
        if (type(limit_degrees) not in (int, float) or not .1 <= limit_degrees <= 3600
                or not math.isfinite(limit_degrees)):
            raise ControlError('连续控制角度限位范围：0.1～3600°。')
        for order in self.orders:
            Move(order, .1, rpm, acceleration_rpm_s=acceleration).validate()
        self.limit, self.rpm, self.acceleration = limit_degrees, rpm, acceleration
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
        if len(values) != len(self.orders) or any(
                type(x) not in (int, float) or abs(x) > 7200 or not math.isfinite(x)
                for x in values):
            raise ControlError('请输入每个选中轴的有限角度数值。')
        with self.lock:
            if not self.ready or self.closed:
                raise ControlError('尚未完成使能，或正在关闭使能。')
            if len(self.commands) >= 32:
                raise ControlError('待执行指令已达 32 条，请等待当前指令完成。')
            targets = tuple(old + change for old, change in zip(self.planned, values)) if relative else values
            if any(abs(x) > self.limit for x in targets):
                raise ControlError(f'指令累计目标超出 ±{self.limit:g}° 限位，未加入队列。')
            duration = max(Move(n, target - old, self.rpm,
                                acceleration_rpm_s=self.acceleration).estimated_seconds
                           for n, target, old in zip(self.orders, targets, self.planned))
            if duration > 120:
                raise ControlError('本条指令预计运动超过 120 秒，请减少位移。')
            self.number += 1
            command = MotionCommand(self.number, targets, duration)
            self.commands.append(command)
            self.planned = targets
            return command

    def pop(self):
        with self.lock:
            return self.commands.popleft() if self.commands else None

    def pending(self):
        with self.lock:
            return len(self.commands)


class _ManualSession(_Session):
    def __init__(self, controller, devices, commands, stop, callback):
        moves = [Move(n, commands.limit, commands.rpm,
                      acceleration_rpm_s=commands.acceleration) for n in commands.orders]
        super().__init__(controller, moves, devices, stop, callback, False)
        self.commands = commands
        self.trace = deque(maxlen=10_000)
        self.history = deque(maxlen=1000)
        self.report.update(continuous=True, control_source='local',
                           limit_degrees=commands.limit, trace_tail_only=True,
                           command_history_tail_only=True, started_commands=0, completed_commands=0)

    def check_stop(self):
        super().check_stop()
        if self.commands.closed:
            raise Stopped('连续使能已关闭。')

    def tick(self, checked=True, cancellable=True):
        super().tick(checked, cancellable)
        if checked and self.armed:
            for a in self.selected():
                if a.get('enable_verified') and a['last'][1] & 0x6F != 0x27:
                    raise ControlError('连续控制期间电机退出使能状态。')
                if abs(displacement(a['last'][2], a['origin'])) > abs(a['delta']) + a['tolerance']:
                    raise ControlError('连续控制反馈超出配置角度限位。')

    def motion(self):
        self.check_stop()
        for a in self.selected():
            a['origin'] = a['last'][2]
            a['target'] = a['origin']
        self.armed = True
        self.emit('phase', text='正在开启连续使能，尚未执行运动指令')
        for cw, state in [(6, 0x21), (7, 0x23), (15, 0x27)]:
            self.command(cw)
            self.wait_all(lambda a: a['last'][1] & 0x6F == state, 1, f'使能状态 {state:#x}')
        for a in self.selected():
            a['enable_verified'] = True
        self.wait_all(lambda a: not a['last'][1] & 0x1000, 1, '清除旧位置确认')
        self.hold(.1)
        if any(abs(displacement(a['last'][2], a['origin'])) > a['tolerance'] for a in self.selected()):
            raise ControlError('使能期间出现意外位移。')
        self.check_stop()
        self.commands.mark_ready()
        self.emit('continuous_ready', orders=list(self.commands.orders))
        self.emit('phase', text='连续使能已就绪；可追加运动指令，空闲保持使能')
        while True:
            self.tick()
            request = self.commands.pop()
            if request is None:
                continue
            self.check_stop()
            entry = {'number': request.number, 'targets_degrees': list(request.targets), 'completed': False}
            self.history.append(entry)
            self.report['started_commands'] += 1
            self.emit('command', number=request.number, stage='started', pending=self.commands.pending())
            deltas = [round(a['counts'] * value * a['direction_factor'] / 360)
                      for a, value in zip(self.selected(), request.targets)]
            for a, delta in zip(self.selected(), deltas):
                # Reuse the already established relative PP handshake. Computing
                # the remaining displacement also avoids a huge signed absolute
                # target jump when the encoder crosses the int32 boundary.
                remaining = delta - displacement(a['last'][2], a['origin'])
                if not -2**31 < remaining < 2**31:
                    raise ControlError('本条指令位移超出可用指令范围。')
                a['target'] = remaining
            self.command(0x5F)  # Relative, single point; FIFO waits for arrival.
            self.wait_all(lambda a: bool(a['last'][1] & 0x1000), 1, '新位置确认')
            self.command(0x4F)
            self.wait_all(lambda a: not a['last'][1] & 0x1000, 1, '清除位置确认')
            expected = {a['device'].order: delta for a, delta in zip(self.selected(), deltas)}
            self.wait_all(lambda a: bool(a['last'][1] & 0x400) and
                          abs(displacement(a['last'][2], a['origin']) - expected[a['device'].order]) <= a['tolerance'],
                          request.estimated_seconds + 5, '到达连续指令目标')
            entry['completed'] = True
            self.report['completed_commands'] += 1
            self.emit('command', number=request.number, stage='completed', pending=self.commands.pending())

    def cleanup(self):
        self.commands.close()  # Reject new GUI commands before disabling/restoring.
        self.report['commands'] = list(self.history)
        super().cleanup()
        # Use the final disabled PDO feedback, including quick-stop deceleration.
        for a in self.selected():
            if 'origin' in a and a['last']:
                a['report']['measured_degrees'] = (displacement(a['last'][2], a['origin']) * 360
                                                   / a['counts'] * a['direction_factor'])


def run_continuous(controller, devices, commands, stop=None, callback=None):
    """Worker entry point. Enabling holds current positions until a GUI command."""
    try:
        if not set(commands.orders) <= {d.order for d in devices}:
            raise ControlError('所选电机不在扫描结果中，请重新扫描。')
        with adapter_lock(controller.adapter):
            return _ManualSession(controller, devices, commands, stop or threading.Event(),
                                  callback or (lambda event: None)).run()
    finally:
        commands.close()
