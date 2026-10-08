"""Optional command-line frontend. Dry-run by default; --run means physical motion."""
import argparse
import json
from pathlib import Path
import sys
import threading
from core import DEFAULT_ADAPTER, EtherCATController, Move


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--adapter', default=DEFAULT_ADAPTER)
    parser.add_argument('--ids', nargs='+', type=int, required=True, help='Unique H0E.21 values, e.g. --ids 0 2')
    parser.add_argument('--degrees', type=float, default=30, help='Signed angle: negative = reverse')
    parser.add_argument('--rpm', type=float, default=5)
    parser.add_argument('--acceleration-rpm-s', type=float, default=10,
                        help='Positive acceleration/deceleration in rpm/s, no software upper limit (default: 10)')
    parser.add_argument('--encoder-bits', type=int, default=23, choices=(23, 26))
    parser.add_argument('--run', action='store_true', help='Perform physical motion; default is no enable')
    args = parser.parse_args()
    controller = EtherCATController(args.adapter, Path(__file__).parent / 'logs')
    devices = controller.scan()
    if len({d.alias for d in devices}) != len(devices):
        parser.error('Duplicate IDs; configure unique H0E.21 values first')
    chosen = [d for d in devices if d.alias in args.ids]
    if {d.alias for d in chosen} != set(args.ids):
        parser.error('Requested ID missing from scan')
    stop = threading.Event()
    import signal
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    report = controller.execute([Move(d.order, args.degrees, args.rpm, args.encoder_bits,
                                     acceleration_rpm_s=args.acceleration_rpm_s) for d in chosen],
                                devices, stop, dry_run=not args.run)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['success'] else 1


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
