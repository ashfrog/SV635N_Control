"""UE UDP bridge and persistent three-axis PP session.

Network work stays off the EtherCAT thread. Targets are motor angles relative to
the position captured when enabling, not platform pitch/roll/heave coordinates.
"""
from collections import deque
import json
import math
import socket
import threading
import time
import uuid

from core import ControlError, Move, Stopped, _Session, adapter_lock, displacement


class CommandInbox:
    """One latest absolute target; never accumulate old game frames."""
    def __init__(self, limit_degrees=30.0, timeout=.5):
        if not math.isfinite(limit_degrees) or not .1 <= limit_degrees <= 360:
            raise ControlError('三轴角度限位范围：0.1～360°。')
        if not math.isfinite(timeout) or not .1 <= timeout <= 5:
            raise ControlError('UE 超时范围：0.1～5 秒。')
        self.limit = limit_degrees
        self.timeout = timeout
        self.session = uuid.uuid4().hex
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = -1
        self.stop = threading.Event()
        self.reason = ''

    def halt(self, reason):
        with self.lock:
            if not self.stop.is_set():
                self.reason = reason
                self.stop.set()

    def accept(self, packet, now=None):
        if not isinstance(packet, dict) or packet.get('session') != self.session:
            return False
        seq = packet.get('seq')
        if type(seq) is not int or not 0 <= seq <= 2**53 - 1:
            raise ControlError('seq 必须是 0～2^53-1 的整数。')
        if type(packet.get('enable')) is not bool:
            raise ControlError('enable 必须是 JSON 布尔值。')
        # An authenticated disable takes priority even if UDP reordered it.
        if not packet['enable']:
            self.halt('UE 关闭使能。')
            return True
        targets = packet.get('targets_deg')
        if not isinstance(targets, list) or len(targets) != 3:
            raise ControlError('targets_deg 必须包含三台电机的目标角度。')
        if any(type(x) not in (int, float) or abs(x) > self.limit or not math.isfinite(x)
               for x in targets):
            raise ControlError(f'三轴目标必须为有限数值，且在 ±{self.limit:g}° 内。')
        with self.lock:
            if self.stop.is_set() or seq <= self.sequence:
                return False
            self.sequence = seq
            self.latest = (seq, tuple(targets), time.monotonic() if now is None else now)
        return True

    def get(self):
        with self.lock:
            return self.latest

    def check(self, enabled=False, now=None):
        if self.stop.is_set():
            raise Stopped(self.reason)
        latest = self.get()
        if (enabled or latest is not None) and (latest is None or
                        (time.monotonic() if now is None else now) - latest[2] > self.timeout):
            self.halt('UE 指令超时，停止全部电机并关闭使能。')
            raise Stopped(self.reason)


class UDPBridge:
    """Loopback by default; first hello owns this session's source IP/port."""
    def __init__(self, inbox, orders, host='127.0.0.1', port=5005):
        self.inbox = inbox
        self.orders = list(orders)
        self.host, self.port = host, port
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.feedback = {'phase': 'starting', 'enabled': False}
        self.peer = None

    def __enter__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(.05)
            self.thread = threading.Thread(target=self._receive, name='UE-UDP', daemon=True)
            self.thread.start()
        except Exception:
            self.sock.close()
            raise
        return self

    def __exit__(self, *_):
        self.closed.set()
        self.thread.join(timeout=1)
        self.sock.close()

    def publish(self, event):
        with self.lock:
            if event['kind'] == 'status':
                selected = {a['order']: a for a in event['axes']}
                axes = [selected[n] for n in self.orders]
                self.feedback.update(axes=axes, enabled=all(a['enabled'] for a in axes))
            elif event['kind'] == 'phase':
                self.feedback['phase'] = event['text']
            elif event['kind'] == 'result':
                self.feedback.update(phase='closed', result=event['report'],
                                     disabled_verified=bool(event['report'].get('all_disabled')))
                if event['report'].get('all_disabled'):
                    self.feedback['enabled'] = False

    def _reply(self, peer, error=None):
        with self.lock:
            reply = dict(self.feedback)
        reply.update(type='state', session=self.inbox.session, orders=self.orders,
                     limit_deg=self.inbox.limit, timeout_s=self.inbox.timeout,
                     seq=self.inbox.sequence, stopping=self.inbox.stop.is_set())
        if error:
            reply['error'] = error
        self.sock.sendto(json.dumps(reply, ensure_ascii=False, allow_nan=False).encode('utf-8'), peer)

    def _receive(self):
        try:
            self._receive_loop()
        except Exception as exc:
            self.inbox.halt(f'UE 接收线程异常：{exc}')

    def _receive_loop(self):
        last_reply = 0
        while not self.closed.is_set():
            try:
                data, peer = self.sock.recvfrom(4097)
                packet = json.loads(data.decode('utf-8')) if len(data) <= 4096 else None
                if not isinstance(packet, dict):
                    continue
                if packet.get('type') == 'hello':
                    if self.peer is None:
                        self.peer = peer
                    if peer == self.peer:
                        self._reply(peer)
                elif peer == self.peer and packet.get('type') == 'command':
                    try:
                        self.inbox.accept(packet)
                    except ControlError as exc:
                        self.inbox.halt(f'UE 指令无效：{exc}')
                        self._reply(peer, str(exc))
            except socket.timeout:
                pass
            except (UnicodeError, ValueError):
                pass  # Unidentified/malformed traffic cannot move or renew heartbeat.
            except OSError as exc:
                if not self.closed.is_set():
                    self.inbox.halt(f'UE 通信失败：{exc}')
                return
            if self.peer and time.monotonic() - last_reply >= .1:
                try:
                    self._reply(self.peer)
                except OSError as exc:
                    self.inbox.halt(f'UE 反馈发送失败：{exc}')
                    return
                last_reply = time.monotonic()


class _ContinuousSession(_Session):
    def __init__(self, controller, moves, devices, stop, callback, inbox):
        super().__init__(controller, moves, devices, stop, callback, False)
        self.inbox = inbox
        # Long-running sessions retain the last 10 seconds, not unbounded 1kHz data.
        self.trace = deque(maxlen=10_000)
        self.report.update(continuous=True, session=inbox.session,
                           limit_degrees=inbox.limit, timeout_s=inbox.timeout,
                           trace_tail_only=True,
                           accepted_targets=0)

    def check_stop(self):
        super().check_stop()
        self.inbox.check(self.armed)

    def tick(self, checked=True, cancellable=True):
        super().tick(checked, cancellable)
        if checked and self.armed:
            for a in self.selected():
                if a['last'][1] & 0x6F != 0x27 and a.get('enable_verified'):
                    raise ControlError('连续控制期间电机退出使能状态。')
                if abs(displacement(a['last'][2], a['origin'])) > abs(a['delta']) + a['tolerance']:
                    raise ControlError('三轴反馈超出配置角度限位。')

    def motion(self):
        self.emit('phase', text='UE 三轴已就绪；等待 enable=true，当前未使能')
        while True:
            self.tick()
            latest = self.inbox.get()
            if latest and time.monotonic() - latest[2] <= self.inbox.timeout:
                break
        for a in self.selected():
            a['origin'] = a['last'][2]
            a['target'] = a['origin']
        self.armed = True
        for cw, state in [(6, 0x21), (7, 0x23), (15, 0x27)]:
            self.command(cw)
            self.wait_all(lambda a: a['last'][1] & 0x6F == state, 1, f'使能状态 {state:#x}')
        for a in self.selected():
            a['enable_verified'] = True
            if abs(displacement(a['last'][2], a['origin'])) > a['tolerance']:
                raise ControlError('使能期间出现意外位移。')
        self.wait_all(lambda a: not a['last'][1] & 0x1000, 1, '清除旧位置确认')
        self.emit('phase', text='三轴使能中；持续执行 UE 目标，关闭开关可停止')
        last_seq, last_targets = -1, None
        stage, deadline = 'idle', 0
        while True:
            self.tick()
            if stage == 'ack':
                if all(a['last'][1] & 0x1000 for a in self.selected()):
                    self.command(0x2F)  # absolute, immediate update, bit4 low
                    stage, deadline = 'clear', time.monotonic() + 1
                elif time.monotonic() > deadline:
                    raise ControlError('连续目标确认超时。')
            elif stage == 'clear':
                if all(not a['last'][1] & 0x1000 for a in self.selected()):
                    stage = 'idle'
                elif time.monotonic() > deadline:
                    raise ControlError('连续目标确认复位超时。')
            if stage != 'idle':
                continue
            latest = self.inbox.get()
            if latest[0] == last_seq:
                continue
            last_seq = latest[0]
            targets = tuple(round(a['counts'] * value * a['direction_factor'] / 360)
                            for a, value in zip(self.selected(), latest[1]))
            if targets == last_targets:
                continue  # Heartbeat does not restart a stationary PP profile.
            last_targets = targets
            for a, delta in zip(self.selected(), targets):
                a['target'] = (a['origin'] + delta + 2**31) % 2**32 - 2**31
            self.command(0x3F)  # absolute + change set immediately + new setpoint
            stage, deadline = 'ack', time.monotonic() + 1
            self.report['accepted_targets'] += 1


def run_three_axis(controller, devices, orders, stop=None, callback=None,
                   rpm=5.0, acceleration=10.0, limit_degrees=30.0,
                   host='127.0.0.1', port=5005, timeout=.5):
    """Local call authorizes this session; remote disable/timeout ends it permanently."""
    orders = list(orders)
    if len(orders) != 3 or len(set(orders)) != 3:
        raise ControlError('UE 三轴模式必须选择三台不同的电机。')
    if not set(orders) <= {d.order for d in devices}:
        raise ControlError('三轴映射不在扫描结果中。')
    if orders != sorted(orders):
        raise ControlError('三轴映射须按链路位置升序。')
    inbox = CommandInbox(limit_degrees, timeout)
    moves = [Move(n, limit_degrees, rpm, acceleration_rpm_s=acceleration) for n in orders]
    for move in moves:
        move.validate()
    callback = callback or (lambda event: None)
    stop = stop or threading.Event()
    with adapter_lock(controller.adapter), UDPBridge(inbox, orders, host, port) as bridge:
        def publish(event):
            bridge.publish(event)
            callback(event)
        session = _ContinuousSession(controller, moves, devices, stop, publish, inbox)
        report = session.run()
        bridge.publish({'kind': 'result', 'report': {
            k: report.get(k) for k in ('stopped', 'error', 'all_disabled', 'cleanup_errors')}})
        if bridge.peer:
            try:
                bridge._reply(bridge.peer)
            except OSError:
                pass
        return report
