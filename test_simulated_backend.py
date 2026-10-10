"""Portable simulator exercises the real UDP path without APS DLLs."""
from pathlib import Path
import math
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from backend import load_config
from motor_service import MotorService
from platform_motion import PlatformGeometry
from simulated_backend import SimulatedController
from UdpControl.client import MotorClient
from udp_server import UDPServer


class SimulatorTests(unittest.TestCase):
    def test_real_pose_stream_moves_three_supports_and_stale_input_stops(self):
        options=load_config(Path(__file__).with_name('backend.platform.example.json'))['platform']
        options['enabled']=True
        controller=SimulatedController(PlatformGeometry(options))
        with patch('aps_backend.APSLibrary',side_effect=AssertionError('Simulator loaded hardware DLL')):
            service=MotorService(platform_options=options,hardware_controller=controller)
            self.addCleanup(service.close)
            server=UDPServer(service,port=0).start();self.addCleanup(server.close)
            client=MotorClient(port=server.port);self.addCleanup(client.close)
            client.hello();client.request('scan');client.wait_for(('idle',))
            client.enable_platform([1,2,3],options['calibration_id'],reference_confirmed=True)
            client.wait_for(('enabled',))
            start=time.monotonic()
            while time.monotonic()-start<.7:
                client.pose(.5,.05*math.sin(time.monotonic()-start),0)
                time.sleep(.025)
            state=client.status()
            self.assertEqual(state['hardware_backend'],'simulated')
            self.assertTrue(all(value>0 for value in state['platform']['actual_lengths_mm']))
            self.assertTrue(state['enabled'])
            state=client.wait_for(('idle',))
            self.assertIn('姿态流超时',state['stop_reason'])
            self.assertTrue(state['result']['all_disabled'])

    def test_simulator_entrypoint_uses_standard_library_only(self):
        result=subprocess.run([sys.executable,'-S','simulated_backend.py','--help'],
            cwd=Path(__file__).resolve().parent,capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('Never loads APS DLLs',result.stdout)

    def test_standalone_simulator_process_accepts_v1_platform_commands(self):
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/'simulator.log'
            with output.open('wb') as log:
                process=subprocess.Popen([sys.executable,'-S','-u','simulated_backend.py','--port','0'],
                    cwd=Path(__file__).resolve().parent,stdout=log,stderr=log)
                try:
                    deadline=time.monotonic()+5
                    while time.monotonic()<deadline:
                        text=output.read_text(encoding='utf-8',errors='replace')
                        if 'SIMULATION ONLY: UDP ' in text:break
                        if process.poll() is not None:self.fail(text)
                        time.sleep(.02)
                    else:self.fail('Simulator startup timeout')
                    port=int(text.split('127.0.0.1:',1)[1].split(',',1)[0])
                    with MotorClient(port=port) as client:
                        client.hello();client.wait_for(('idle',))
                        client.enable_platform([1,2,3],'simulation-only-v1',reference_confirmed=True)
                        client.wait_for(('enabled',))
                        client.pose(0,0,0)
                        client.disable()
                        state=client.wait_for(('idle','fault'))
                        self.assertEqual(state['hardware_backend'],'simulated')
                        self.assertTrue(state['result']['all_disabled'])
                        client.release()
                finally:
                    if process.poll() is None:process.terminate()
                    process.wait(timeout=5)


if __name__=='__main__':unittest.main()
