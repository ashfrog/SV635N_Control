"""Standard-library-only wire encoding shared with clients."""
import json

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
