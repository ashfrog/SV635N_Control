"""Persistent centres, APS return lifecycle and UI; fake hardware only."""
from dataclasses import replace
import gc
import os
from pathlib import Path
import tempfile
import threading
import time
import tkinter as tk
import unittest
from unittest.mock import patch

from control_common import ControlError, adapter_lock
from fixed_reference import FixedReference
from motor_service import MotorService
from platform_motion import PlatformGeometry
from simulated_backend import SimulatedController
from test_aps_backend import FakeAPS
from test_height_control import options
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError
from UdpControl.debug_ui import DebugWindow


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)/'platform.reference.json'
        self.api = FakeAPS()
        self.lock_patch = patch('aps_backend.adapter_lock', side_effect=lambda adapter:
            adapter_lock(f'{adapter}:reference-test:{os.getpid()}'))
        self.lock_patch.start()
        self.service = MotorService(reference_path=self.path, aps_options=dict(
            extension_limits={str(axis):'di2' for axis in self.api.io},
            retraction_limits={str(axis):'di1' for axis in self.api.io}))
        self.service.hardware.api_factory = lambda:self.api
        self.server = UDPServer(self.service, port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()
        self.lock_patch.stop()
        self.directory.cleanup()
        gc.collect()

    def save(self):
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        return self.service.reference.data['id']

    def test_save_has_no_servo_or_motion_and_persists_across_restart(self):
        with self.assertRaises(UDPError):
            self.client.save_reference()
        reference_id = self.save()
        self.assertFalse(any(n in ('APS_set_servo_on','APS_ptp_all') for n,_ in self.api.calls))
        restored = FixedReference(self.path, 'PCIe-8332:0', 'aps')
        self.assertEqual(restored.validate(self.service.devices)['id'], reference_id)
        self.assertEqual([a['position'] for a in restored.data['axes']], [1000]*3)
        self.assertTrue(self.client.status()['fixed_reference']['valid'])
        with self.assertRaises(ControlError):
            restored.validate([replace(d, units_per_rev=d.units_per_rev*2) for d in self.service.devices])

    def test_recreated_service_returns_to_saved_native_coordinates(self):
        reference_id = self.save()
        self.client.close()
        self.server.close()
        self.service.close()
        self.api.positions.update({10:1200, 12:900, 14:1400})
        self.api.calls.clear()
        self.service = MotorService(reference_path=self.path, aps_options=dict(
            extension_limits={str(axis):'di2' for axis in self.api.io},
            retraction_limits={str(axis):'di1' for axis in self.api.io}))
        self.service.hardware.api_factory = lambda:self.api
        self.server = UDPServer(self.service, port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('enabled',))
        deadline = time.monotonic()+2
        while time.monotonic()<deadline and sum(n=='APS_ptp_all' for n,_ in self.api.calls)<3:
            time.sleep(.01)
        self.assertEqual([(a[0],a[2]) for n,a in self.api.calls if n=='APS_ptp_all'],
                         [(10,1000),(12,1000),(14,1000)])

    def test_platform_refuses_off_center_before_any_servo_on(self):
        self.save()
        config = options()
        config['legs'][0]['extension_sign'] = -1
        config['legs'][1]['extension_sign'] = -1
        self.service.platform = PlatformGeometry(config)
        self.api.positions[10] += self.service.devices[0].units_per_rev
        self.client.enable_platform([1,2,3], config['calibration_id'], reference_confirmed=True)
        state = self.client.wait_for(('fault',))
        self.assertIn('尚未回到', state['message'])
        self.assertFalse(any(n in ('APS_set_servo_on','APS_ptp_all') for n,_ in self.api.calls))

    def test_resave_replaces_previous_run_feedback_and_rejects_drift(self):
        reference_id = self.save()
        self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('enabled',))
        self.client.disable()
        self.client.wait_for(('idle',))
        self.api.positions.update({10:2000,12:2000,14:2000})
        replacement = self.save()
        self.assertNotEqual(reference_id, replacement)
        self.assertEqual(self.client.status()['fixed_reference']['heights_mm'], [0,0,0])
        scan = self.service.hardware.scan
        count = 0
        def drifting_scan():
            nonlocal count
            count += 1
            if count == 2:
                self.api.positions[10] += self.service.devices[0].units_per_rev
            return scan()
        with patch.object(self.service.hardware, 'scan', side_effect=drifting_scan):
            self.client.save_reference(physical_center_confirmed=True)
            self.client.wait_for(('fault',))
        self.assertEqual(self.service.reference.data['id'], replacement)

    def test_reference_requests_require_owner_and_retries_do_not_repeat_actions(self):
        with MotorClient(port=self.server.port) as observer:
            observer.hello(claim=False)
            with self.assertRaises(UDPError):
                observer.save_reference(physical_center_confirmed=True)
            with self.assertRaises(UDPError):
                observer.return_reference('missing', position_reference_confirmed=True)
        peer = ('127.0.0.1', self.client.sock.getsockname()[1])
        packet = dict(v=1, type='save_reference', id='save-retry', session=self.client.session,
                      control_seq=self.client.control_sequence+1, physical_center_confirmed=True)
        first = self.server.handle(packet, peer)
        self.assertTrue(first['ok'])
        self.assertEqual(first, self.server.handle(packet, peer))
        self.client.wait_for(('idle',))
        reference_id = self.service.reference.data['id']
        packet = dict(v=1, type='return_reference', id='return-retry', session=self.client.session,
                      control_seq=packet['control_seq']+1, reference_id=reference_id,
                      position_reference_confirmed=True)
        first = self.server.handle(packet, peer)
        self.assertTrue(first['ok'])
        self.assertEqual(first, self.server.handle(packet, peer))
        self.assertEqual(self.service.run_id, first['run_id'])
        self.service.stop('test complete')
        self.client.wait_for(('idle',))

    def test_return_rescans_native_positions_preserves_sign_and_blocks_override(self):
        reference_id = self.save()
        self.api.positions.update({10:1100, 12:800, 14:1300})
        with self.assertRaises(UDPError):
            self.client.return_reference(reference_id)
        with self.assertRaises(UDPError):
            self.client.return_reference('old', position_reference_confirmed=True)
        reply = self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('enabled',))
        end = time.monotonic()+2
        while time.monotonic()<end and sum(n=='APS_ptp_all' for n,_ in self.api.calls)<3:
            time.sleep(.01)
        moves = [a for n,a in self.api.calls if n=='APS_ptp_all']
        self.assertEqual([(a[0],a[2]) for a in moves], [(10,1000),(12,1000),(14,1000)])
        self.assertEqual(self.service.control_mode, 'reference')
        self.assertEqual(self.service.commands.rpm, 5)
        for action in (lambda:self.client.target([0,0,0]), lambda:self.client.heights([0,0,0]),
                       lambda:self.client.set_profile(60,120), lambda:self.client.save_reference(physical_center_confirmed=True)):
            with self.assertRaises(UDPError):
                action()
        self.assertEqual(self.service.phase, 'enabled')
        self.client.disable()
        self.assertTrue(self.client.wait_for(('idle',))['result']['all_disabled'])
        with self.assertRaises(UDPError):
            self.client.request('heartbeat', run_id=reply['run_id'], seq=10000)

    def test_changed_mapping_and_active_endpoint_never_move_to_old_reference(self):
        reference_id = self.save()
        self.api.gear = 2
        self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('fault',))
        self.assertFalse(any(n in ('APS_set_servo_on','APS_ptp_all') for n,_ in self.api.calls))
        self.api.gear = 1
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.api.digital_inputs[10] = 1
        self.client.return_reference(reference_id, position_reference_confirmed=True)
        self.client.wait_for(('fault',))
        self.assertFalse(any(n=='APS_ptp_all' for n,_ in self.api.calls))

    def test_corrupt_saved_file_is_not_used(self):
        self.path.write_text('{broken', encoding='utf-8')
        reference = FixedReference(self.path, 'PCIe-8332:0', 'aps')
        self.assertFalse(reference.status(self.service.devices, {})['valid'])
        with self.assertRaises(ControlError):
            reference.validate(self.service.devices)


class ReferenceUITests(unittest.TestCase):
    def test_stop_with_return_ack_in_flight_cancels_that_exact_run(self):
        with tempfile.TemporaryDirectory() as folder:
            controller = SimulatedController(PlatformGeometry(options()))
            service = MotorService(hardware_controller=controller,
                reference_path=Path(folder)/'reference.json')
            server = UDPServer(service, port=0).start()
            root = tk.Tk()
            ui = DebugWindow(root, '127.0.0.1', server.port)
            held = []
            released = threading.Event()
            send = server._send
            def hold_ack(packet, peer):
                if packet.get('type')=='ack' and packet.get('run_id') and not released.is_set():
                    if not held:held.append((packet,peer))
                else:send(packet,peer)
            def pump(condition):
                deadline = time.monotonic()+5
                while time.monotonic()<deadline:
                    root.update()
                    if condition():return
                    time.sleep(.005)
                self.fail(str(service.snapshot()))
            try:
                pump(lambda:bool(ui.state) and not ui.pending)
                with patch('UdpControl.debug_ui.messagebox.askyesno', return_value=True):
                    ui.save_fixed_reference()
                    pump(lambda:ui.state.get('fixed_reference',{}).get('valid') and not ui.pending)
                    server._send = hold_ack
                    ui.return_fixed_reference()
                    pump(lambda:bool(held))
                    self.assertIsNone(ui.client.run_id)
                    ui.stop()
                    released.set()
                    send(*held[0])
                    pump(lambda:ui.state.get('phase')=='idle' and not ui.pending and not ui.enable_pending)
                self.assertIsNone(ui.client.run_id)
                self.assertTrue(service.snapshot()['result']['all_disabled'])
            finally:
                released.set()
                server._send = send
                ui.close()
                root.destroy()
                server.close()
                service.close()
                del ui,root
                gc.collect()

    def test_unconfigured_height_page_can_save_and_return_fixed_center_after_stop(self):
        with tempfile.TemporaryDirectory() as folder:
            config = options()
            controller = SimulatedController(PlatformGeometry(config))
            service = MotorService(hardware_controller=controller,
                reference_path=Path(folder)/'reference.json')
            server = UDPServer(service, port=0).start()
            root = tk.Tk()
            ui = DebugWindow(root, '127.0.0.1', server.port)
            def pump(condition):
                deadline = time.monotonic()+5
                while time.monotonic()<deadline:
                    root.update()
                    if condition():return
                    time.sleep(.005)
                self.fail(str(service.snapshot()))
            try:
                pump(lambda:bool(ui.state) and not ui.pending)
                self.assertEqual(str(ui.neutral_box.cget('state')), 'disabled')
                ui.claim()
                pump(lambda:service.phase=='enabled' and not ui.pending)
                self.assertEqual(str(ui.save_reference_button.cget('state')), 'disabled')
                ui.stop()
                pump(lambda:ui.state.get('phase')=='idle' and not ui.stop_pending)
                with patch('UdpControl.debug_ui.messagebox.askyesno', return_value=True):
                    ui.save_reference_button.invoke()
                    pump(lambda:ui.state.get('fixed_reference', {}).get('valid') and not ui.pending)
                    controller.devices = [replace(d, position=d.position+36) for d in controller.devices]
                    ui.return_reference_button.invoke()
                    pump(lambda:ui.state.get('phase')=='enabled' and ui.state.get('control_mode')=='reference')
                    pump(lambda:all(abs(value)<.02 for value in ui.state['fixed_reference']['heights_mm']))
                self.assertEqual(str(ui.height_sliders[0].cget('state')), 'disabled')
                self.assertEqual(str(ui.slider.cget('state')), 'disabled')
                self.assertEqual(str(ui.stop_button.cget('state')), 'normal')
                ui.stop()
                pump(lambda:ui.state.get('phase')=='idle')
            finally:
                ui.close()
                root.destroy()
                server.close()
                service.close()
                del ui,root
                gc.collect()


if __name__ == '__main__':unittest.main()
