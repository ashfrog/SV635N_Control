import threading
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

from core import ControlError, EtherCATController
from test_platform_control import FakeMaster
from UdpControl.client import MotorClient, UDPError
from motor_service import MotorService
from udp_server import UDPServer


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.masters=[]
        def factory(adapter,log_dir):
            master=FakeMaster()
            self.masters.append(master)
            return EtherCATController('fake',master_factory=lambda:master)
        self.service=MotorService('fake',heartbeat_timeout=.5,controller_factory=factory)
        self.server=UDPServer(self.service,port=0).start()
        self.client=MotorClient(port=self.server.port)
        self.client.hello()

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()

    def scan(self):
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.assertEqual(len(self.client.state['devices']),4)

    def enable(self,orders=None):
        self.scan()
        self.client.enable(orders or [1,2,3],rpm=80,acceleration_rpm_s=120)
        self.client.wait_for(('enabled',),timeout=5)

    def wait_condition(self,condition,timeout=5):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if condition():
                return
            time.sleep(.01)
        self.fail(str(self.service.snapshot()))

    def test_multiple_runs_selection_unlimited_target_and_no_retrigger(self):
        self.enable([1,3])
        master=self.masters[-1]
        self.client.target([10000,-10000])
        self.wait_condition(lambda: all(s.targets for s in (master.slaves[0],master.slaves[2])))
        for _ in range(4):
            self.client.target([10000,-10000])
        self.assertEqual([len(s.targets) for s in master.slaves],[1,0,1,0])
        old=self.client.run_id
        self.client.disable()
        state=self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(state['enabled'])
        self.client.enable([1],rpm=80,acceleration_rpm_s=120)
        self.client.wait_for(('enabled',))
        self.assertNotEqual(old,self.client.run_id)
        with self.assertRaises(UDPError):
            self.client.request('disable',run_id=old)
        self.assertEqual(self.service.phase,'enabled')

    def test_heartbeat_timeout_rejects_late_packets(self):
        self.enable()
        run_id=self.client.run_id
        self.client.run_id=None  # Simulate client death without sending disable.
        self.wait_condition(lambda:self.service.phase in ('stopping','idle'))
        with self.assertRaises(UDPError):
            self.client.request('target',run_id=run_id,seq=10000,targets_deg=[1,2,3])
        state=self.client.wait_for(('idle',))
        self.assertTrue(state['result']['all_disabled'])
        self.assertFalse(any(s.targets for s in self.masters[-1].slaves))
        self.assertEqual(state['stop_reason'],'UDP 心跳超时')

    def test_reorder_heartbeats_do_not_supersede_targets_or_renew_duplicates(self):
        self.enable()
        self.client.run_id=None  # Drive the protocol explicitly.
        run_id=self.service.run_id
        self.assertTrue(self.service.command(run_id,100,None))
        self.assertTrue(self.service.command(run_id,2,[2,3,4]))
        self.assertFalse(self.service.command(run_id,1,[9,9,9]))
        self.assertEqual(self.service.commands.planned,(2,3,4))
        self.assertFalse(self.service.command(run_id,100,None))
        with self.service.lock:
            self.service.last_heartbeat=time.monotonic()-1
        with self.assertRaises(ControlError):
            self.service.command(run_id,101,None)
        self.assertEqual(self.service.phase,'stopping')

    def test_socket_failure_and_fault_require_rescan(self):
        self.enable()
        original=self.masters[-1].receive_processdata
        self.masters[-1].receive_processdata=lambda timeout: -1 if self.service.phase=='enabled' else original(timeout)
        # Recover transport when cleanup starts so disable/restoration can be verified.
        self.wait_condition(lambda:self.service.commands.closed)
        self.masters[-1].receive_processdata=original
        self.wait_condition(lambda:self.service.phase=='fault')
        with self.assertRaises(UDPError):
            self.client.enable([1])
        self.client.run_id=None
        self.scan()
        self.client.enable([1])
        self.client.wait_for(('enabled',))
        self.server.close()
        self.wait_condition(lambda:self.service.phase in ('stopping','idle'))
        self.wait_condition(lambda:self.service.phase=='idle')
        self.assertTrue(self.service.snapshot()['result']['all_disabled'])

    def test_separate_portable_client_process_controls_backend_only_over_udp(self):
        self.client.release()
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/'UdpControl'
            target.mkdir()
            for source in (Path(__file__).resolve().parent/'UdpControl').glob('*.py'):
                shutil.copy2(source,target/source.name)
            script='''
import time
from UdpControl.client import MotorClient
with MotorClient(port=PORT) as client:
    client.hello()
    client.request('scan')
    client.wait_for(('idle',))
    client.enable([2,4],rpm=80,acceleration_rpm_s=120)
    client.wait_for(('enabled',))
    client.target([6,-6])
    time.sleep(.1)
    client.disable()
    state=client.wait_for(('idle',))
    assert state['result']['all_disabled']
    assert state['targets_deg']==[6,-6]
print('separate client completed')
'''.replace('PORT',str(self.server.port))
            result=subprocess.run([sys.executable,'-S','-c',script],cwd=directory,
                                  capture_output=True,text=True,timeout=15)
            self.assertEqual(result.returncode,0,result.stderr)
        self.assertEqual([len(s.targets) for s in self.masters[-1].slaves],[0,1,0,1])
        self.assertEqual(self.service.phase,'idle')


if __name__=='__main__':
    unittest.main()
