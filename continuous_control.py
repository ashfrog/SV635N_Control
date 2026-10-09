"""Local persistent PP control: enable once, update the latest target immediately.

Only the worker touches EtherCAT. The GUI publishes immutable targets.
No network transport or heartbeat is needed for an idle manual session.
"""
from collections import deque
import math
from control_common import MotionCommand, MotionQueue
import threading
import time

from core import (ControlError, Move, Stopped, _Session, adapter_lock, displacement)

class _ManualSession(_Session):
    def __init__(self, controller, devices, commands, stop, callback):
        # setup() uses Move for profile parameters; actual travel is set per command.
        moves = [Move(n, .1, commands.rpm,
                      acceleration_rpm_s=commands.acceleration) for n in commands.orders]
        super().__init__(controller, moves, devices, stop, callback, False)
        self.commands = commands
        self.profile_revision = commands.profile()[0]
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
        velocity = a['report']['profile_velocity_counts_s']
        acceleration = a['report']['profile_acceleration_counts_s2']
        margin = max(a['tolerance'] * 5, round(a['counts'] * 5 / 360 + velocity**2 / (2 * acceleration)))
        low = min(a['command_start'], a['command_goal']) - margin
        high = max(a['command_start'], a['command_goal']) + margin
        if not low <= a['unwrapped_counts'] <= high:
            raise ControlError('连续控制反馈偏离当前指令路径。')

    def apply_profile(self):
        revision, rpm, acceleration = self.commands.profile()
        if revision == self.profile_revision:
            return False
        values = []
        for a in self.selected():
            raw = (a['counts'] * (rpm / 60), a['counts'] * (acceleration / 60))
            if not all(math.isfinite(v) for v in raw):
                raise ControlError('速度/加速度超出驱动器 32 位参数范围。')
            velocity, accel = (max(1, round(v)) for v in raw)
            if max(velocity, accel) > 0xFFFFFFFF:
                raise ControlError('速度/加速度超出驱动器 32 位参数范围。')
            values.append((a, velocity, accel))
        for a, velocity, accel in values:
            for index, value in ((0x6081, velocity), (0x6083, accel), (0x6084, accel), (0x6085, accel)):
                self.tick()  # Maintain cyclic PDO traffic between mailbox writes.
                self.write(a, index, 4, value)
            a['report'].update(profile_velocity_counts_s=velocity, profile_acceleration_counts_s2=accel)
        self.profile_revision = revision
        self.emit('profile_applied', revision=revision, rpm=rpm, acceleration_rpm_s=acceleration)
        return True

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
        revision, rpm, acceleration = self.commands.profile()
        self.emit('profile_applied', revision=revision, rpm=rpm, acceleration_rpm_s=acceleration)
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
            profile_changed = self.apply_profile()
            request = self.commands.pop()
            profile_retarget = (request is None and profile_changed and active is not None and
                                not all(a['last'][1] & 0x400 and
                                        abs(a['unwrapped_counts'] - expected[a['device'].order]) <= a['tolerance']
                                        for a in self.selected()))
            if profile_retarget:
                request = active
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
            if active is not None and not profile_retarget:
                entry['superseded'] = True
            active = request
            if not profile_retarget:
                entry = {'number': request.number, 'targets_degrees': list(request.targets),
                         'completed': False, 'superseded': False}
                self.history.append(entry)
                self.report['started_commands'] += 1
                self.emit('command', number=request.number, stage='started', pending=self.commands.pending())
            deltas = [round(a['counts'] * value * a['direction_factor'] / 360)
                      for a, value in zip(self.selected(), request.targets)]
            duration = 0
            braking_seconds = 0
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
                                              a['report']['profile_velocity_counts_s'] * 60 / a['counts'],
                                              acceleration_rpm_s=a['report']['profile_acceleration_counts_s2'] * 60
                                              / a['counts']).estimated_seconds)
                braking_seconds = max(braking_seconds, a['report']['profile_velocity_counts_s'] /
                                      a['report']['profile_acceleration_counts_s2'])
            self.command(0x3F)  # Absolute + change set immediately + new setpoint.
            stage, handshake_deadline = 'ack', time.monotonic() + 1
            expected = {a['device'].order: delta for a, delta in zip(self.selected(), deltas)}
            arrival_deadline = time.monotonic() + duration + 2 * braking_seconds + 5

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
