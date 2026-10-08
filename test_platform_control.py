"""Hardware-free protocol and CiA402 regression tests."""
import json
import socket
import struct
import threading
import time
import unittest

from core import ControlError, EtherCATController, Move, RX_MAP, TX_MAP, Stopped, _Session
from platform_control import CommandInbox, UDPBridge, _ContinuousSession


class FakeSlave:
    def __init__(self, order):
        self.man, self.id, self.rev = 0x100000, 0xC010E, 0x10000
        self.name, self.state, self.al_status = 'SV635N fake', 2, 0
        self.position = 2**31 - 100 if order == 1 else 1000 * order
        self.sw = 0x40
        self.output, self.input = bytes(12), bytes(28)
        self.params = {(0x200E, 0x16): order, (0x6091, 1): 1, (0x6091, 2): 1,
                       (0x2000, 1): 14101, (0x2002, 3): 1 if order != 2 else 0,
                       (0x2002, 1): 9, (0x200E, 0x20): 1, (0x6502, 0): 1,
                       (0x605A, 0): 2, (0x200E, 2): 1, (0x6060, 0): 0,
                       (0x6081, 0): 42, (0x6083, 0): 43,
                       (0x6084, 0): 44, (0x6085, 0): 45,
                       (0x1C12, 0): 1, (0x1C12, 1): 0x1701,
                       (0x1C13, 0): 1, (0x1C13, 1): 0x1B01,
                       (0x1701, 0): len(RX_MAP), (0x1B01, 0): len(TX_MAP)}
        for pdo, values in ((0x1701, RX_MAP), (0x1B01, TX_MAP)):
            self.params.update({(pdo, n): value for n, value in enumerate(values, 1)})
        self.original = dict(self.params)
        self.targets = []
        self.words = []
        self.previous = 0

    def sdo_read(self, index, sub):
        dynamic = {0x6041: self.sw, 0x603F: 0, 0x6064: self.position,
                   0x6061: self.params[(0x6060, 0)]}
        value = dynamic[index] if sub == 0 and index in dynamic else self.params[(index, sub)]
        return value.to_bytes(4, 'little', signed=index in (0x6064, 0x6060, 0x6061))

    def sdo_write(self, index, sub, data):
        self.params[(index, sub)] = int.from_bytes(data, 'little', signed=index == 0x6060)

    def _fprd(self, address, size):
        return {0x0814: b'\x40', 0x0400: (2498).to_bytes(2, 'little'),
                0x0420: (1).to_bytes(2, 'little'), 0x0981: b'\x03'}[address]

    def dc_sync(self, *_):
        pass

    def exchange(self):
        word, target, _, _ = struct.unpack('<HiHI', self.output)
        self.words.append(word)
        base = word & 0xF
        self.sw = {0: 0x40, 2: 0x7, 6: 0x21, 7: 0x23, 15: 0x27}[base]
        if base == 15:
            if word & 0x10:
                self.sw |= 0x1000
                if not self.previous & 0x10:
                    self.position = (target + (self.position if word & 0x40 else 0)
                                     + 2**31) % 2**32 - 2**31
                    self.targets.append(target)
            self.sw |= 0x400
        self.previous = word
        self.input = struct.pack('<HHi', 0, self.sw, self.position) + bytes(20)


class FakeMaster:
    def __init__(self):
        self.slaves = [FakeSlave(n) for n in range(1, 5)]
        self.expected_wkc = 12
        self.state, self.closed = 2, False

    def open(self, _):
        pass

    def close(self):
        self.closed = True

    def config_init(self):
        return len(self.slaves)

    def state_check(self, *_):
        pass

    def read_state(self):
        pass

    def write_state(self):
        for slave in self.slaves:
            slave.state = self.state

    def config_map(self):
        pass

    def config_dc(self):
        return True

    def send_processdata(self):
        for slave in self.slaves:
            slave.exchange()

    def receive_processdata(self, _):
        return self.expected_wkc


class InboxTests(unittest.TestCase):
    def packet(self, inbox, seq=1, **changes):
        return dict(session=inbox.session, seq=seq, enable=True,
                    targets_deg=changes.pop('targets_deg', [1, 2, 3]), **changes)

    def test_latest_absolute_target_and_reorder(self):
        inbox = CommandInbox()
        self.assertTrue(inbox.accept(self.packet(inbox, 2), now=10))
        self.assertFalse(inbox.accept(self.packet(inbox, 1), now=11))
        self.assertEqual(inbox.get(), (2, (1, 2, 3), 10))
        with self.assertRaises(Stopped):
            inbox.check(True, now=10.6)

    def test_timeout_after_first_command_before_enable(self):
        inbox = CommandInbox()
        inbox.check(False, now=10)  # No client yet: remain ready without enabling.
        inbox.accept(self.packet(inbox), now=10)
        with self.assertRaises(Stopped):
            inbox.check(False, now=10.6)

    def test_validation_and_stale_session(self):
        inbox = CommandInbox()
        stale = self.packet(inbox)
        stale['session'] = 'old'
        self.assertFalse(inbox.accept(stale))
        for values in ([0, 0], [0, 0, float('nan')], [0, 0, float('inf')],
                       [0, True, 0], [31, 0, 0], [10**3000, 0, 0]):
            with self.assertRaises(ControlError):
                inbox.accept(self.packet(inbox, targets_deg=values))
        self.assertIsNone(inbox.get())

    def test_disable_is_latched_even_out_of_order(self):
        inbox = CommandInbox()
        inbox.accept(self.packet(inbox, 10))
        inbox.accept(dict(session=inbox.session, seq=1, enable=False))
        self.assertFalse(inbox.accept(self.packet(inbox, 11)))
        with self.assertRaises(Stopped):
            inbox.check()

    def test_udp_handshake_peer_and_fault(self):
        inbox = CommandInbox()
        with UDPBridge(inbox, [1, 2, 3], port=0) as bridge:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as client:
                client.settimeout(1)
                peer = bridge.sock.getsockname()
                client.sendto(b'{"type":"hello"}', peer)
                state = json.loads(client.recv(4096))
                self.assertEqual(state['session'], inbox.session)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stranger:
                    stranger.sendto(json.dumps(dict(type='command', **self.packet(inbox, 9))).encode(), peer)
                client.sendto(json.dumps(dict(type='command', **self.packet(inbox, 2))).encode(), peer)
                end = time.monotonic() + 1
                while inbox.get() is None and time.monotonic() < end:
                    client.recv(4096)
                self.assertEqual(inbox.get()[0], 2)
                bad = dict(type='command', **self.packet(inbox, 3, targets_deg=[90, 0, 0]))
                client.sendto(json.dumps(bad).encode(), peer)
                self.assertTrue(inbox.stop.wait(1))


class SessionTests(unittest.TestCase):
    def test_single_move_still_completes(self):
        master = FakeMaster()
        controller = EtherCATController('fake', master_factory=lambda: master)
        session = _Session(controller, [Move(n, 1) for n in (1, 2, 3)],
                           controller._discover(master), threading.Event(), lambda _: None, False)
        report = session.run()
        self.assertTrue(report['success'], report)
        self.assertTrue(report['all_disabled'])
        self.assertTrue(all(len(s.targets) == 1 for s in master.slaves[:3]))

    def test_watchdog_wkc_and_limit_fail_closed(self):
        for fault in ('timeout', 'wkc', 'limit', 'local_stop'):
            with self.subTest(fault=fault):
                master = FakeMaster()
                controller = EtherCATController('fake', master_factory=lambda: master)
                stop = threading.Event()
                inbox = CommandInbox(timeout=5)
                inbox.accept(dict(session=inbox.session, seq=1, enable=True, targets_deg=[1, 2, 3]))
                session = _ContinuousSession(controller, [Move(n) for n in (1, 2, 3)],
                                             controller._discover(master), stop, lambda _: None, inbox)
                original_exchange = master.send_processdata
                fired = False
                def exchange():
                    nonlocal fired
                    original_exchange()
                    if master.slaves[0].targets and not fired:
                        fired = True
                        if fault == 'timeout':
                            inbox.accept(dict(session=inbox.session, seq=2, enable=True,
                                              targets_deg=[1, 2, 3]), now=time.monotonic() - 6)
                        elif fault == 'wkc':
                            master.expected_wkc = 0
                        elif fault == 'local_stop':
                            stop.set()
                        else:
                            slave = master.slaves[0]
                            slave.position = (session.axes[0]['origin'] + 2**23 + 2**31) % 2**32 - 2**31
                            slave.input = struct.pack('<HHi', 0, slave.sw, slave.position) + bytes(20)
                    elif fired and fault == 'wkc':
                        master.expected_wkc = 12  # Recover transport for cleanup.
                master.send_processdata = exchange
                report = session.run()
                self.assertFalse(report['success'], report)
                self.assertIn('error', report)
                self.assertTrue(report['all_disabled'], report)
                self.assertNotIn('cleanup_errors', report)
                self.assertTrue(master.closed)
                self.assertTrue(all(a.get('parameters_restored') for a in report['axes'][:3]))

    def test_persistent_motion_wrap_direction_stop_and_restore(self):
        master = FakeMaster()
        controller = EtherCATController('fake', master_factory=lambda: master)
        devices = controller._discover(master)
        inbox = CommandInbox(timeout=5)
        inbox.accept(dict(session=inbox.session, seq=1, enable=True, targets_deg=[1, 2, 3]))
        moves = [Move(n, 30, 5) for n in (1, 2, 3)]
        session = _ContinuousSession(controller, moves, devices, threading.Event(), lambda _: None, inbox)
        original_positions = [s.position for s in master.slaves]
        # Stop/update from PDO feedback on the caller thread, no hardware involved.
        original_exchange = master.send_processdata
        repeat_count = 0
        def exchange():
            nonlocal repeat_count
            original_exchange()
            if len(master.slaves[0].targets) == 1 and all(s.sw & 0x1000 for s in master.slaves[:3]):
                inbox.accept(dict(session=inbox.session, seq=2, enable=True, targets_deg=[1, 2, 3]))
            elif len(master.slaves[0].targets) == 1 and all(not s.sw & 0x1000 for s in master.slaves[:3]):
                repeat_count += 1
                inbox.accept(dict(session=inbox.session, seq=3 + repeat_count, enable=True,
                                  targets_deg=[1, 2, 3] if repeat_count < 5 else [-2, -1, 0]))
            elif len(master.slaves[0].targets) == 2 and all(s.sw & 0x1000 for s in master.slaves[:3]):
                inbox.halt('test stop')
        master.send_processdata = exchange
        report = session.run()
        self.assertTrue(report['stopped'], report)
        self.assertTrue(report['all_disabled'], report)
        self.assertNotIn('cleanup_errors', report)
        self.assertEqual(report['accepted_targets'], 2)
        for index, slave in enumerate(master.slaves[:3]):
            factor = -1 if index == 1 else 1
            delta = round(2**23 * [-2, -1, 0][index] * factor / 360)
            expected = (original_positions[index] + delta + 2**31) % 2**32 - 2**31
            self.assertEqual(slave.position, expected)
            self.assertTrue(report['axes'][index]['parameters_restored'])
            for parameter in (0x6081, 0x6083, 0x6084, 0x6085, 0x6060):
                self.assertEqual(slave.params[(parameter, 0)], slave.original[(parameter, 0)])
            self.assertEqual(slave.params[(0x200E, 2)], 1)
            # There is no disable/re-enable between the two accepted targets.
            first, second = [i for i, word in enumerate(slave.words) if word == 0x3F][:2]
            self.assertTrue(all(word & 0xF == 15 for word in slave.words[first:second]))
        self.assertFalse(master.slaves[3].targets)
        self.assertTrue(all(word == 0 for word in master.slaves[3].words))
        self.assertTrue(master.closed)

    def test_pre_cancel_never_enables(self):
        master = FakeMaster()
        controller = EtherCATController('fake', master_factory=lambda: master)
        inbox = CommandInbox()
        stop = threading.Event()
        stop.set()
        session = _ContinuousSession(controller, [Move(n) for n in (1, 2, 3)],
                                     controller._discover(master), stop, lambda _: None, inbox)
        report = session.run()
        self.assertTrue(report['stopped'])
        self.assertFalse(report['motion_triggered'])
        self.assertFalse(any(s.targets for s in master.slaves))


if __name__ == '__main__':
    unittest.main()
