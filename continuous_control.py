"""Local persistent PP control: enable once, update the latest target immediately.

Only the worker touches EtherCAT. The GUI publishes immutable targets.
No network transport or heartbeat is needed for an idle manual session.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time

from core import (MAX_ACCELERATION_RPM_S, MAX_MOVE_SECONDS, MAX_RPM,
                  ControlError, Move, Stopped, _Session, adapter_lock, displacement)


@dataclass(frozen=True)
class MotionCommand:
    number: int
    targets: tuple[float, ...]
    estimated_seconds: float


class MotionQueue:
    def __init__(self, orders, rpm=MAX_RPM, acceleration=MAX_ACCELERATION_RPM_S):
        self.orders = tuple(orders)
        if (not self.orders or len(set(self.orders)) != len(self.orders)
                or self.orders != tuple(sorted(self.orders))):
            raise ControlError('请选择不同的电机，按链路位置升序排列。')
        for order in self.orders:
            Move(order, .1, rpm, acceleration_rpm_s=acceleration).validate()
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


class _ManualSession(_Session):
    def __init__(self, controller, devices, commands, stop, callback):
        # setup() uses Move for profile parameters; actual travel is set per command.
        moves = [Move(n, .1, commands.rpm,
                      acceleration_rpm_s=commands.acceleration) for n in commands.orders]
        super().__init__(controller, moves, devices, stop, callback, False)
        self.commands = commands
        self.trace = deque(maxlen=10_000)
        self.history = deque(maxlen=1000)
        self.report.update(continuous=True, control_source='local',
                           trace_tail_only=True,
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

    def update_feedback(self, a):
        if 'unwrapped_counts' in a:
            a['unwrapped_counts'] += displacement(a['last'][2], a['previous_position'])
            a['previous_position'] = a['last'][2]

    def travel_degrees(self, a):
        return a['unwrapped_counts'] * 360 / a['counts'] * a['direction_factor']

    def check_travel(self, a):
        # Monitor this command's path, without bounding the session's total travel.
        braking_degrees = self.commands.rpm**2 / (12 * self.commands.acceleration)
        margin = max(a['tolerance'] * 5, round(a['counts'] * (5 + braking_degrees) / 360))
        low = min(a['command_start'], a['command_goal']) - margin
        high = max(a['command_start'], a['command_goal']) + margin
        if not low <= a['unwrapped_counts'] <= high:
            raise ControlError('连续控制反馈偏离当前指令路径。')

    def motion(self):
        self.check_stop()
        for a in self.selected():
            a['origin'] = a['last'][2]
            a['target'] = a['origin']
            a.update(previous_position=a['origin'], unwrapped_counts=0,
                     command_start=0, command_goal=0)
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
        stage, handshake_deadline = 'idle', 0
        active, entry, arrival_deadline = None, None, 0
        expected = {}
        while True:
            self.tick()
            if stage == 'ack':
                if all(a['last'][1] & 0x1000 for a in self.selected()):
                    self.command(0x2F)  # Absolute + change immediately; new-setpoint bit low.
                    stage, handshake_deadline = 'clear', time.monotonic() + 1
                elif time.monotonic() > handshake_deadline:
                    raise ControlError('连续目标确认超时。')
            elif stage == 'clear':
                if all(not a['last'][1] & 0x1000 for a in self.selected()):
                    stage = 'idle'
                elif time.monotonic() > handshake_deadline:
                    raise ControlError('连续目标确认复位超时。')
            if stage != 'idle':
                continue
            request = self.commands.pop()
            if request is None:
                if active is not None:
                    if all(a['last'][1] & 0x400 and
                           abs(a['unwrapped_counts'] - expected[a['device'].order]) <= a['tolerance']
                           for a in self.selected()):
                        entry['completed'] = True
                        for a in self.selected():
                            a['command_start'] = a['unwrapped_counts']
                        self.report['completed_commands'] += 1
                        self.emit('command', number=active.number, stage='completed', pending=0)
                        active = None
                    elif time.monotonic() > arrival_deadline:
                        raise ControlError('等待超时：到达连续指令目标')
                continue
            self.check_stop()
            if active is not None:
                entry['superseded'] = True
            active = request
            entry = {'number': request.number, 'targets_degrees': list(request.targets),
                     'completed': False, 'superseded': False}
            self.history.append(entry)
            self.report['started_commands'] += 1
            self.emit('command', number=request.number, stage='started', pending=self.commands.pending())
            deltas = [round(a['counts'] * value * a['direction_factor'] / 360)
                      for a, value in zip(self.selected(), request.targets)]
            duration = 0
            for a, delta in zip(self.selected(), deltas):
                # Use absolute PP for moving updates: a relative target based on
                # the last feedback would acquire an error while the motor moves
                # between that feedback and the new setpoint's reception.
                remaining = delta - a['unwrapped_counts']
                if not -2**31 <= remaining < 2**31:
                    raise ControlError('本条指令位移超出可用指令范围。')
                a['target'] = (a['origin'] + delta + 2**31) % 2**32 - 2**31
                a['command_start'] = a['unwrapped_counts']
                a['command_goal'] = delta
                duration = max(duration, Move(a['device'].order, abs(remaining) * 360 / a['counts'],
                                              self.commands.rpm,
                                              acceleration_rpm_s=self.commands.acceleration).estimated_seconds)
            self.command(0x3F)  # Absolute + change set immediately + new setpoint.
            stage, handshake_deadline = 'ack', time.monotonic() + 1
            expected = {a['device'].order: delta for a, delta in zip(self.selected(), deltas)}
            arrival_deadline = time.monotonic() + duration + 2 * self.commands.rpm / self.commands.acceleration + 5

    def cleanup(self):
        self.commands.close()  # Reject new GUI commands before disabling/restoring.
        self.report['commands'] = list(self.history)
        super().cleanup()
        # Use the final disabled PDO feedback, including quick-stop deceleration.
        for a in self.selected():
            if 'origin' in a and a['last']:
                a['report']['measured_degrees'] = self.travel_degrees(a)


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
