"""UE protocol demo client: sends three motor targets to an already opened bridge.

This client can move the motors when run_three_axis() is explicitly started.
The current GUI only starts local manual control, not the UE bridge.
"""
import argparse
import json
import math
import socket
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=5005)
    parser.add_argument('--amplitude', type=float, default=5)
    parser.add_argument('--period', type=float, default=10)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--hz', type=float, default=30)
    args = parser.parse_args()
    if (not all(math.isfinite(x) for x in (args.amplitude, args.period, args.seconds, args.hz))
            or not 0 <= args.amplitude <= 360 or args.period <= 0 or args.seconds <= 0
            or not 1 <= args.hz <= 100):
        parser.error('Invalid amplitude/period/duration/rate')
    peer = (socket.gethostbyname(args.host), args.port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(peer)
        sock.settimeout(2)
        sock.send(b'{"type":"hello"}')
        state = json.loads(sock.recv(65535).decode('utf-8'))
        if args.amplitude > state['limit_deg']:
            parser.error('Amplitude exceeds bridge limit')
        if 1 / args.hz >= state['timeout_s']:
            parser.error('Send rate is slower than bridge timeout')
        print(json.dumps(state, ensure_ascii=False))
        session = state['session']
        sock.setblocking(False)
        seq, started = 0, time.monotonic()
        try:
            while time.monotonic() - started < args.seconds:
                elapsed = time.monotonic() - started
                # All axes start at zero; phase relationships produce distinct axes.
                targets = [args.amplitude * math.sin(2 * math.pi * elapsed / args.period * ratio)
                           for ratio in (1, .8, .6)]
                sock.send(json.dumps(dict(type='command', session=session, seq=seq,
                                          enable=True, targets_deg=targets)).encode('utf-8'))
                seq += 1
                try:
                    while True:
                        state = json.loads(sock.recv(65535).decode('utf-8'))
                        if state.get('stopping') or state.get('phase') == 'closed':
                            print(json.dumps(state, ensure_ascii=False))
                            return
                except BlockingIOError:
                    pass
                time.sleep(1 / args.hz)
        except KeyboardInterrupt:
            pass
        finally:
            try:
                sock.send(json.dumps(dict(type='command', session=session, seq=seq,
                                          enable=False)).encode('utf-8'))
            except OSError:
                pass


if __name__ == '__main__':
    main()
