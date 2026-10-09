"""Compare three-axis command latency using simulated APS calls, never real hardware."""
import argparse
import json
import math
import os
import statistics
import threading
import time
from unittest.mock import patch

from aps_backend import PCIe8332Controller
from control_common import MotionQueue, adapter_lock
from test_aps_backend import FakeAPS


def measure(frames=30, call_cost_s=.0002):
    if type(frames) is not int or frames < 1 or not math.isfinite(call_cost_s) or call_cost_s < 0:
        raise ValueError('frames must be positive and call_cost_s must be finite and nonnegative')
    api = FakeAPS()
    native_call = api.call
    def call(name, *args):
        result = native_call(name, *args)
        until = time.perf_counter() + call_cost_s
        while time.perf_counter() < until:
            pass
        return result
    api.call = call
    controller = PCIe8332Controller(api_factory=lambda: api, options=dict(
        extension_limits={str(axis): 'di2' for axis in api.io},
        retraction_limits={str(axis): 'di1' for axis in api.io}))
    commands, stop = MotionQueue([1, 2, 3]), threading.Event()
    submitted, latencies = {}, []
    start_calls = end_calls = 0
    def submit(number):
        submitted[number] = time.perf_counter()
        commands.submit([number, -number, number])
    def event(e):
        nonlocal start_calls, end_calls
        if e['kind'] == 'continuous_ready':
            start_calls = len(api.calls)
            submit(1)
        elif e['kind'] == 'command' and e['stage'] == 'started':
            latencies.append((time.perf_counter() - submitted[e['number']]) * 1000)
            end_calls = len(api.calls)
            if e['number'] == frames:
                stop.set()
            else:
                submit(e['number'] + 1)
    with patch('aps_backend.adapter_lock', side_effect=lambda adapter:
               adapter_lock(f'{adapter}:benchmark:{os.getpid()}')):
        try:
            devices = controller.scan()
            report = controller.run_continuous(devices, commands, stop, event)
            if not report['stopped'] or not report['all_disabled'] or len(latencies) != frames:
                raise RuntimeError(str(report))
        finally:
            controller.close()
    return dict(frames=frames, simulated_call_us=call_cost_s * 1e6,
                median_submit_to_dispatch_ms=round(statistics.median(latencies), 3),
                max_submit_to_dispatch_ms=round(max(latencies), 3),
                native_calls_per_target=round((end_calls-start_calls)/frames, 2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    print(json.dumps(measure(args.frames), indent=2))
