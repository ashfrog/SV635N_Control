"""UDP platform simulator. Never loads APS DLLs or connects physical motors."""
import argparse
from copy import deepcopy
import math
from pathlib import Path
import signal
import threading
import time

from aps_backend import APSDevice, validate_options
from backend import load_config
from motor_service import MotorService
from platform_motion import PlatformGeometry
from udp_server import UDPServer


class SimulatedController:
    backend_name = 'simulated'

    def __init__(self, geometry):
        self.geometry = geometry
        self.adapter = 'PCIe-8332:0'
        self.options = validate_options(dict(
            extension_limits={str(i):'di2' for i in range(3)},
            retraction_limits={str(i):'di1' for i in range(3)}))
        self.devices = [APSDevice(order=leg['order'], alias=i, name='SIMULATED SV635N',
            vendor=0x00100000, product=0x000C010E, revision=1, state=8,
            statusword=0x40, error_code=0, position=0, gear_numerator=1,
            gear_denominator=1, positive_direction=1, axis_id=i, slave_id=i,
            units_per_rev=360, extension_input='di2', extension_sign=leg['extension_sign'],
            retraction_input='di1', retraction_sign=-leg['extension_sign'])
            for i, leg in enumerate(geometry.legs)]
        self.worker_id = None

    def _thread(self):
        ident = threading.get_ident()
        if self.worker_id is None:
            self.worker_id = ident
        if self.worker_id != ident:
            raise RuntimeError('Simulation accessed outside hardware worker')

    def adapters(self):
        return [(self.adapter, 'SIMULATION ONLY — no physical hardware')]

    def scan(self):
        self._thread()
        return list(self.devices)

    def read_inputs(self, devices):
        self._thread()
        return {d.order: dict(input='di2', retraction_input='di1', state='clear',
            retraction_state='clear', valid=True, triggered=False, retraction_triggered=False,
            conflict=False, extension_sign=d.extension_sign, retraction_sign=d.retraction_sign)
            for d in devices}

    def close(self):
        self._thread()
        return []

    def run_continuous(self, devices, commands, stop, callback):
        self._thread()
        positions, speeds = [0.]*3, [0.]*3
        targets = [0.]*3
        last, last_status = time.monotonic(), 0.
        commands.mark_ready()
        callback(dict(kind='continuous_ready'))
        while not stop.is_set() and not commands.closed:
            now = time.monotonic();dt = min(.05, now-last);last=now
            frame=commands.pop()
            if frame:
                targets=list(frame.targets)
            for i, leg in enumerate(self.geometry.legs):
                gap=targets[i]-positions[i]
                vmax=self.geometry.config['max_leg_speed_mm_s']*360/leg['mm_per_rev']
                acceleration=self.geometry.config['max_leg_acceleration_mm_s2']*360/leg['mm_per_rev']
                velocity=math.copysign(min(vmax,math.sqrt(2*acceleration*abs(gap))),gap) if gap else 0.
                speeds[i]+=max(-acceleration*dt,min(acceleration*dt,velocity-speeds[i]))
                step=speeds[i]*dt
                if gap*step>=0 and abs(step)>=abs(gap):
                    positions[i]=targets[i];speeds[i]=0.
                else:
                    positions[i]+=step
            if now-last_status>=.05:
                callback(dict(kind='status', axes=[dict(order=d.order,axis_id=d.axis_id,
                    slave_id=d.slave_id, position=positions[i], travel_degrees=positions[i],
                    enabled=True,error_code=0) for i,d in enumerate(self.devices)],
                    limits=self.read_inputs(self.devices)))
                last_status=now
            commands.wait_for_update(.005)
        commands.close()
        return dict(stopped=True,all_disabled=True,cleanup_errors=[],error=None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('backend.platform.example.json'))
    parser.add_argument('--port',type=int,default=5006)
    args=parser.parse_args()
    config=load_config(args.config)
    options=deepcopy(config['platform']);options['enabled']=True
    geometry=PlatformGeometry(options)
    controller=SimulatedController(geometry)
    service=MotorService(platform_options=options,hardware_controller=controller)
    server=None;quit_event=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:quit_event.set())
    signal.signal(signal.SIGTERM,lambda *_:quit_event.set())
    try:
        server=UDPServer(service,port=args.port).start()
        service.scan()
        print(f'SIMULATION ONLY: UDP 127.0.0.1:{server.port}, calibration_id={options["calibration_id"]}',flush=True)
        while not quit_event.wait(.1):
            if server.closed.is_set():break
    finally:
        service.shutdown.set()
        if server:server.notify_shutdown()
        service.close()
        if server:server.close()
    return 0


if __name__=='__main__':raise SystemExit(main())
