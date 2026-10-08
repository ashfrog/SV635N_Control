"""Thread-safe UDP SDK: ACK retries, feedback and independent heartbeats."""
import json
import socket
import threading
import time
import uuid

from . import PROTOCOL_VERSION
from .wire import encode


class UDPError(RuntimeError):
    pass


class MotorClient:
    def __init__(self, host='127.0.0.1', port=5005, auth_key='', heartbeat_interval=.1):
        if not .02 <= heartbeat_interval <= .2:
            raise ValueError('heartbeat_interval 须为 0.02～0.2 秒。')
        self.address = (socket.gethostbyname(host), port)
        self.auth_key, self.interval = auth_key, heartbeat_interval
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))
        self.sock.settimeout(.05)
        self.lock = threading.RLock()
        self.control_lock = threading.Lock()
        self.pending = {}
        self.session, self.run_id = None, None
        self.sequence = self.control_sequence = -1
        self.state = {}
        self.state_received = 0
        self.state_serial = -1
        self.server_id = None
        self.background_error = None
        self.closed = threading.Event()
        self.receiver = threading.Thread(target=self._receive, name='MotorClient-RX', daemon=True)
        self.heartbeats = threading.Thread(target=self._heartbeat, name='MotorClient-Heartbeat', daemon=True)
        self.receiver.start()
        self.heartbeats.start()

    def _receive(self):
        while not self.closed.is_set():
            try:
                data, peer = self.sock.recvfrom(65535)
                if peer != self.address:
                    continue
                packet = json.loads(data.decode('utf-8'))
                if not isinstance(packet, dict) or packet.get('v') != PROTOCOL_VERSION:
                    continue
                with self.lock:
                    # Telemetry has a monotonic server counter so late UDP replies
                    # cannot roll the client's run/phase back to an earlier state.
                    serial = packet.get('state_serial', -1)
                    incoming_server = packet.get('server_id')
                    if self.server_id is None and isinstance(incoming_server, str):
                        self.server_id = incoming_server
                    if incoming_server == self.server_id and isinstance(packet.get('state'), dict) and type(serial) is int and serial > self.state_serial:
                        self.state, self.state_serial = packet['state'], serial
                        self.state_received = time.monotonic()
                    reply_id = packet.get('id')
                    entry = self.pending.get(reply_id) if isinstance(reply_id, str) else None
                    if packet.get('type') == 'ack' and entry:
                        entry[1].append(packet)
                        entry[0].set()
            except socket.timeout:
                pass
            except (ValueError, UnicodeError, ConnectionResetError):
                pass
            except OSError:
                if not self.closed.is_set():
                    self.background_error = 'UDP 接收失败'
                return

    def request(self, kind, retries=4, retry_timeout=.15, **payload):
        if self.closed.is_set():
            raise UDPError('客户端已关闭。')
        request_id = uuid.uuid4().hex
        with self.lock:
            packet = dict(v=PROTOCOL_VERSION, type=kind, id=request_id, **payload)
            if self.session:
                packet['session'] = self.session
            if self.auth_key:
                packet['auth_key'] = self.auth_key
            if kind in ('adapters', 'scan', 'enable', 'release'):
                self.control_sequence += 1
                packet['control_seq'] = self.control_sequence
            event, replies = threading.Event(), []
            self.pending[request_id] = (event, replies)
        try:
            data = encode(packet)
            for _ in range(retries):
                self.sock.sendto(data, self.address)
                if event.wait(retry_timeout):
                    reply = replies[0]
                    if not reply.get('ok'):
                        raise UDPError(reply.get('error', '请求被拒绝'))
                    return reply
            raise UDPError(f'{kind} 未收到确认；执行结果未知，请查询状态。')
        except OSError as exc:
            raise UDPError(str(exc)) from exc
        finally:
            with self.lock:
                self.pending.pop(request_id, None)

    def hello(self, claim=True):
        with self.control_lock:
            reply = self.request('hello', claim=claim)
            with self.lock:
                if self.server_id != reply['server_id']:
                    self.server_id = reply['server_id']
                    self.state, self.state_serial = reply['state'], reply['state_serial']
                    self.state_received = time.monotonic()
                if claim:
                    if self.session != reply['session']:
                        self.run_id, self.sequence = None, -1
                        self.control_sequence = reply['control_seq']
                    self.session = reply['session']
                    self.control_sequence = max(self.control_sequence, reply['control_seq'])
            return reply

    def enable(self, orders, rpm=60, acceleration_rpm_s=120):
        with self.control_lock:
            reply = self.request('enable', orders=orders, rpm=rpm, acceleration_rpm_s=acceleration_rpm_s)
            with self.lock:
                self.run_id, self.sequence = reply['run_id'], -1
                self.background_error = None
            return reply

    def _next_sequence(self):
        with self.lock:
            self.sequence += 1
            return self.run_id, self.sequence

    def target(self, targets_deg):
        run_id, seq = self._next_sequence()
        return self.request('target', run_id=run_id, seq=seq, targets_deg=list(targets_deg))

    def heartbeat(self):
        run_id, seq = self._next_sequence()
        return self.request('heartbeat', run_id=run_id, seq=seq, retries=1, retry_timeout=.08)

    def disable(self):
        with self.lock:
            run_id = self.run_id
            # Stop producing heartbeats before sending a disable: if the ACK is
            # lost, the server watchdog still stops this run.
            self.run_id = None
        return self.request('disable', run_id=run_id)

    def release(self):
        with self.control_lock:
            reply = self.request('release')
            with self.lock:
                self.session, self.run_id = None, None
            return reply

    def status(self):
        return self.request('status')['state']

    def wait_for(self, phases, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            state = self.status()
            if state['phase'] in phases:
                return state
            if state['phase'] == 'fault':
                raise UDPError(state['message'])
            time.sleep(.05)
        raise UDPError('等待后台状态超时。')

    def _heartbeat(self):
        while not self.closed.wait(self.interval):
            with self.lock:
                active = self.session and self.run_id and self.state.get('run_id') == self.run_id and self.state.get('phase') in ('enabling', 'enabled')
            if active:
                try:
                    self.heartbeat()
                except UDPError as exc:
                    self.background_error = str(exc)

    def close(self):
        if self.closed.is_set():
            return
        if self.run_id:
            try:
                self.disable()
                self.wait_for(('idle', 'fault'), timeout=10)
            except UDPError:
                pass  # Server's independent watchdog remains responsible.
        if self.session:
            try:
                self.release()
            except UDPError:
                pass
        self.closed.set()
        self.sock.close()
        self.receiver.join(timeout=1)
        self.heartbeats.join(timeout=1)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
