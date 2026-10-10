"""Frozen entry: select a process role before importing hardware or Tk modules."""
import multiprocessing
import sys


def main():
    multiprocessing.freeze_support()
    if len(sys.argv) > 1 and sys.argv[1] == '--client':
        del sys.argv[1]
        from UdpControl.__main__ import main as run
    elif len(sys.argv) > 1 and sys.argv[1] == '--simulate':
        del sys.argv[1]
        from simulated_backend import main as run
    else:
        from backend import main as run
    return run()


if __name__ == '__main__':
    raise SystemExit(main())
