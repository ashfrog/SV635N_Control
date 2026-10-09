import threading
import time
import unittest
from unittest.mock import Mock

from core import ControlError
from udp_server import UDPServer, decode, encode
from UdpControl.client import MotorClient, UDPError


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.service=Mock()
        self.service.is_busy.return_value=False
        self.service.snapshot.return_value=dict(phase='idle',message='ready',run_id=None,orders=[],devices=[],axes=[])
        self.service.run_id='run'
        self.service.enable.return_value='run'
        self.service.command.return_value=True
        self.server=UDPServer(self.service,port=0)
        self.peer=('127.0.0.1',12345)
        reply=self.server.handle(dict(v=1,type='hello',id='hello'),self.peer)
        self.token=reply['session']

    def packet(self,kind,id='request',**kw):
        return dict(v=1,type=kind,id=id,session=self.token,**kw)

    def test_duplicate_enable_and_request_collision(self):
        packet=self.packet('enable',control_seq=0,orders=[1,2,3])
        first=self.server.handle(packet,self.peer)
        self.assertTrue(first['ok'])
        self.assertEqual(first,self.server.handle(packet,self.peer))
        self.service.enable.assert_called_once()
        self.assertFalse(self.server.handle(dict(packet,orders=[1]),self.peer)['ok'])

    def test_profile_authorization_and_duplicate_ack(self):
        packet=self.packet('profile',run_id='run',seq=7,rpm=30,acceleration_rpm_s=60)
        self.assertFalse(self.server.handle(packet,('127.0.0.1',12346))['ok'])
        first=self.server.handle(packet,self.peer)
        self.assertTrue(first['ok'])
        self.assertEqual(first,self.server.handle(packet,self.peer))
        self.service.command.assert_called_once_with('run',7,profile=dict(rpm=30,acceleration_rpm_s=60))
        self.assertFalse(self.server.handle(dict(packet,rpm=40),self.peer)['ok'])

    def test_control_sequence_protects_after_cache_eviction(self):
        packet=self.packet('enable',control_seq=0,orders=[1])
        self.assertTrue(self.server.handle(packet,self.peer)['ok'])
        for n in range(260):
            self.server.handle(self.packet('heartbeat',id=str(n),run_id='run',seq=n),self.peer)
        self.assertFalse(self.server.handle(packet,self.peer)['ok'])
        self.service.enable.assert_called_once()

    def test_owner_auth_and_disable_run_isolation(self):
        other=('127.0.0.1',12346)
        self.assertFalse(self.server.handle(self.packet('enable',control_seq=0,orders=[1]),other)['ok'])
        self.assertFalse(self.server.handle(dict(v=1,type='hello',id='claim'),other)['ok'])
        self.assertTrue(self.server.handle(dict(v=1,type='hello',id='observe',claim=False),other)['ok'])
        self.assertFalse(self.server.handle(self.packet('disable',run_id='old'),self.peer)['ok'])
        self.service.stop.assert_not_called()
        self.assertTrue(self.server.handle(self.packet('disable',run_id='run',seq=-1),self.peer)['ok'])
        self.service.stop.assert_called_once()
        with self.assertRaises(ValueError):
            UDPServer(self.service,host='0.0.0.0')
        secured=UDPServer(self.service,auth_key='key',port=0)
        self.assertFalse(secured.handle(dict(v=1,type='hello',id='h'),self.peer)['ok'])
        self.assertTrue(secured.handle(dict(v=1,type='hello',id='h',auth_key='key'),self.peer)['ok'])

    def test_release_retry_and_idle_lease_expiry(self):
        packet=self.packet('release',control_seq=0)
        reply=self.server.handle(packet,self.peer)
        self.assertTrue(reply['ok'])
        self.assertEqual(reply,self.server.handle(packet,self.peer))
        hello=self.server.handle(dict(v=1,type='hello',id='h'),self.peer)
        self.assertNotEqual(hello['session'],self.token)
        self.server.last_seen=time.monotonic()-10
        new=self.server.handle(dict(v=1,type='hello',id='new'),('127.0.0.1',99))
        self.assertTrue(new['ok'])
        self.assertFalse(self.server.handle(self.packet('enable',control_seq=1,orders=[1]),self.peer)['ok'])

    def test_bad_packets_are_rejected(self):
        for raw in (b'[]',b'{',b'{"x":NaN}',b'x'*8193):
            with self.assertRaises(ValueError):
                decode(raw)
        for packet in (self.packet('target',run_id='run',seq=1,targets_deg=None),
                       self.packet('enable',control_seq=True,orders=[1]),
                       dict(v=2,type='hello',id='h')):
            self.assertFalse(self.server.handle(packet,self.peer)['ok'])

    def test_real_udp_lost_ack_retry_executes_enable_once(self):
        self.server.start()
        original=self.server._send
        dropped=threading.Event()
        def send(packet,peer):
            if packet.get('type')=='ack' and packet.get('run_id')=='run' and not dropped.is_set():
                dropped.set()
                return
            original(packet,peer)
        self.server._send=send
        # Reset direct-test ownership; the real client uses a different source port.
        self.server.owner=self.server.token=None
        try:
            with MotorClient(port=self.server.port) as client:
                client.hello()
                reply=client.enable([1,2,3])
                self.assertTrue(reply['ok'])
                self.assertTrue(dropped.is_set())
                self.service.enable.assert_called_once()
        finally:
            self.server.close()

    def test_client_reconnects_after_restart_and_ignores_old_server_feedback(self):
        self.server.owner=self.server.token=None
        self.server.start()
        client=MotorClient(port=self.server.port)
        replacement=None
        try:
            old=client.hello()
            old_token=client.session
            port=self.server.port
            self.server.close()
            self.service.snapshot.return_value['message']='restarted'
            replacement=UDPServer(self.service,port=port).start()
            client.hello()
            self.assertNotEqual(old_token,client.session)
            self.assertNotEqual(old['server_id'],client.server_id)
            self.assertEqual(client.state['message'],'restarted')
            replacement._send(dict(type='state',v=1,server_id=old['server_id'],state_serial=99999,
                                   state=dict(phase='fault',message='stale')),client.sock.getsockname())
            time.sleep(.05)
            self.assertEqual(client.state['message'],'restarted')
        finally:
            client.close()
            self.server.close()
            if replacement:
                replacement.close()


if __name__=='__main__':
    unittest.main()
