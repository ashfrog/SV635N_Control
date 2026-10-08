"""Versioned UDP RPC, exclusive leases, bounded deduplication and telemetry."""
from collections import OrderedDict
import hmac
import json
import logging
import secrets
import socket
import threading
import time

from core import ControlError

PROTOCOL_VERSION = 1
MAX_PACKET = 8192


def encode(packet):
    return json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode('utf-8')


def decode(data):
    def reject(value):
        raise ValueError('不接受 NaN 或 Infinity')
    if len(data) > MAX_PACKET:
        raise ValueError('数据报过长')
    packet = json.loads(data.decode('utf-8'), parse_constant=reject)
    if not isinstance(packet, dict):
        raise ValueError('数据报必须为 JSON 对象')
    return packet


LOG = logging.getLogger(__name__)


class UDPServer:
    def __init__(self, service, host='127.0.0.1', port=5005, auth_key='', lease_seconds=5):
        if host not in ('127.0.0.1', 'localhost') and not auth_key:
            raise ValueError('非本机监听须配置 auth_key。')
        self.service, self.host, self.port, self.auth_key = service, host, port, auth_key
        self.lease_seconds = lease_seconds
        self.lock = threading.RLock()
        self.owner, self.token, self.last_seen = None, None, 0
        self.control_sequence = -1
        self.released = None
        self.cache = OrderedDict()
        self.closed = threading.Event()
        self.state_serial = 0
        self.server_id = secrets.token_hex(16)
        self.sock = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            self.sock.bind((self.host, self.port))
            self.port = self.sock.getsockname()[1]
            self.sock.settimeout(.02)
            self.thread = threading.Thread(target=self._receive, name='Motor-UDP', daemon=True)
            self.thread.start()
        except Exception:
            self.sock.close()
            raise
        LOG.info('UDP listening %s:%s', self.host, self.port)
        return self

    def _expire(self):
        if self.owner and time.monotonic() - self.last_seen >= self.lease_seconds and not self.service.is_busy():
            self.owner, self.token = None, None
            self.cache.clear()

    def _authorize(self, packet, peer):
        if peer != self.owner or not isinstance(packet.get('session'), str) or not hmac.compare_digest(packet['session'], self.token or ''):
            raise ControlError('不是当前控制客户端，或 session 已失效。')

    def _state(self):
        state = self.service.snapshot()
        self.state_serial += 1
        return dict(state=state, state_serial=self.state_serial, server_id=self.server_id)

    def handle(self, packet, peer):
        with self.lock:
            self._expire()
            request_id = packet.get('id')
            response = dict(type='ack', v=PROTOCOL_VERSION, id=request_id, ok=False)
            try:
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                    raise ControlError('id 必须为 1～64 字符的请求标识。')
                if packet.get('v') != PROTOCOL_VERSION or type(packet.get('v')) is not int:
                    raise ControlError('协议版本不支持，请设置 v=1。')
                if self.auth_key and (not isinstance(packet.get('auth_key'), str) or
                        not hmac.compare_digest(packet['auth_key'].encode('utf-8'), self.auth_key.encode('utf-8'))):
                    raise ControlError('auth_key 不匹配。')
                kind = packet.get('type')
                if kind == 'hello':
                    claim = packet.get('claim', True)
                    if type(claim) is not bool:
                        raise ControlError('claim 必须为布尔值。')
                    if claim:
                        if self.owner is not None and peer != self.owner:
                            raise ControlError('已有客户端持有控制权。')
                        if self.owner is None:
                            self.owner, self.token = peer, secrets.token_urlsafe(32)
                            self.control_sequence = -1
                            self.released = None
                        self.last_seen = time.monotonic()
                        response['session'] = self.token
                        response['control_seq'] = self.control_sequence
                    response['owns_control'] = peer == self.owner
                elif kind == 'status':
                    response['owns_control'] = peer == self.owner
                else:
                    if kind == 'release' and self.released and self.released[:2] == (peer, encode(packet)):
                        return self.released[2]
                    self._authorize(packet, peer)
                    key = (self.token, request_id)
                    fingerprint = encode(packet)
                    if key in self.cache:
                        old_fingerprint, old_response = self.cache[key]
                        if old_fingerprint != fingerprint:
                            raise ControlError('重复 id 的请求内容不同。')
                        return old_response
                    if kind in ('adapters', 'scan', 'enable', 'release'):
                        seq = packet.get('control_seq')
                        if type(seq) is not int or not 0 <= seq <= 2**53 - 1 or seq <= self.control_sequence:
                            raise ControlError('control_seq 必须严格递增；过期控制请求不能再次执行。')
                        self.control_sequence = seq
                    self.last_seen = time.monotonic()
                    if kind == 'adapters':
                        response['adapters'] = self.service.adapters()
                    elif kind == 'scan':
                        self.service.scan(packet.get('adapter'))
                    elif kind == 'enable':
                        response['run_id'] = self.service.enable(packet.get('orders'),
                            packet.get('rpm', 60), packet.get('acceleration_rpm_s', 120))
                    elif kind in ('target', 'heartbeat'):
                        if kind == 'target' and not isinstance(packet.get('targets_deg'), list):
                            raise ControlError('targets_deg 必须是数组。')
                        response['accepted'] = self.service.command(packet.get('run_id'), packet.get('seq'),
                            packet.get('targets_deg') if kind == 'target' else None)
                    elif kind == 'disable':
                        # Stop wins even over reordered sequence numbers, but an
                        # old run must never stop a newly enabled run.
                        if packet.get('run_id') != self.service.run_id or self.service.run_id is None:
                            raise ControlError('run_id 已失效。')
                        self.service.stop('UDP 客户端关闭使能')
                    elif kind == 'release':
                        if self.service.is_busy():
                            raise ControlError('请先停止并等待参数恢复完成，再释放控制权。')
                        self.owner, self.token = None, None
                        self.cache.clear()
                    else:
                        raise ControlError('未知请求类型。')
                    response['ok'] = True
                    response.update(self._state())
                    if kind == 'release':
                        self.released = (peer, fingerprint, response)
                    self.cache[key] = (fingerprint, response)
                    while len(self.cache) > 256:
                        self.cache.popitem(last=False)
                    return response
                response.update(ok=True, **self._state())
            except (ControlError, ValueError, TypeError, OverflowError) as exc:
                response['error'] = str(exc)
            return response

    def _send(self, packet, peer):
        try:
            self.sock.sendto(encode(packet), peer)
        except OSError:
            # A dropped telemetry/ACK is retried by the client; it cannot renew
            # the motor heartbeat. Socket receive failure shuts down below.
            LOG.debug('UDP reply failed', exc_info=True)

    def _receive(self):
        last_publish = 0
        try:
            while not self.closed.is_set():
                try:
                    data, peer = self.sock.recvfrom(MAX_PACKET + 1)
                    try:
                        packet = decode(data)
                    except (ValueError, UnicodeError, RecursionError):
                        continue
                    self._send(self.handle(packet, peer), peer)
                except socket.timeout:
                    pass
                except ConnectionResetError:
                    continue  # Windows UDP ICMP from a client that exited.
                with self.lock:
                    self._expire()
                    if self.owner and time.monotonic() - last_publish >= .1:
                        self._send(dict(type='state', v=PROTOCOL_VERSION, **self._state()), self.owner)
                        last_publish = time.monotonic()
        except Exception:
            if not self.closed.is_set():
                LOG.exception('UDP receive worker failed')
                self.service.stop('UDP 接收服务异常')
            self.closed.set()

    def close(self):
        self.closed.set()
        self.service.stop('UDP 服务关闭')
        if self.sock:
            self.sock.close()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1)
