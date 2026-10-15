"""Commissioning configuration and GUI transition; fake/simulated motors only."""
import gc
import json
import os
from pathlib import Path
import tempfile
import time
import tkinter as tk
import unittest
from unittest.mock import patch
from tkinter import ttk

from backend import load_config
from control_common import adapter_lock
from motor_service import MotorService
from platform_motion import PlatformGeometry
from simulated_backend import SimulatedController
from test_aps_backend import FakeAPS
from test_height_control import options
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError
from UdpControl.debug_ui import DebugWindow


def travel():
    return [dict(order=i,up_mm=50,down_mm=50) for i in (1,2,3)]


class SetupTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name)/'backend.config.json'
        self.original = dict(host='127.0.0.1', port=5005, auth_key='preserve',
            aps=dict(extension_limits={str(i):'di2' for i in (10,12,14)},
                     retraction_limits={str(i):'di1' for i in (10,12,14)}))
        self.path.write_text(json.dumps(self.original),encoding='utf-8')
        self.api = FakeAPS()
        self.lock_patch = patch('aps_backend.adapter_lock', side_effect=lambda adapter:
            adapter_lock(f'{adapter}:setup-test:{os.getpid()}'))
        self.lock_patch.start()
        self.service = MotorService(aps_options=self.original['aps'], configuration_path=self.path,
                                   reference_path=self.path.with_name('platform.reference.json'))
        self.service.hardware.api_factory = lambda:self.api
        self.server = UDPServer(self.service,port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))
        self.client.save_reference(physical_center_confirmed=True)
        self.client.wait_for(('idle',))
        self.reference_id = self.service.reference.data['id']

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()
        self.lock_patch.stop()
        self.directory.cleanup()

    def configure(self):
        return self.client.configure_platform(travel(),self.reference_id,calibration_confirmed=True)

    def test_persist_geometry_direction_and_original_config_without_motion(self):
        self.configure()
        state = self.client.status()
        self.assertTrue(state['platform']['configured'])
        self.assertEqual(state['phase'],'idle')
        self.assertFalse(state['enabled'])
        self.assertFalse(any(n in ('APS_set_servo_on','APS_ptp_all') for n,_ in self.api.calls))
        saved = load_config(self.path)
        for key in ('host','port','auth_key'):
            self.assertEqual(saved[key],self.original[key])
        self.assertEqual(saved['aps']['extension_limits'],self.original['aps']['extension_limits'])
        self.assertEqual(json.loads(self.path.with_name(self.path.name+'.before-platform.bak').read_text()),self.original)
        platform = PlatformGeometry(saved['platform'])
        self.assertEqual([leg['extension_sign'] for leg in platform.legs],[-1,-1,1])
        self.assertEqual(platform.heights_to_targets([1,0,0])[2],[-72,0,0])
        self.assertEqual(platform.heights_to_targets([50,50,50])[2],[-3600,-3600,3600])
        self.assertEqual(platform.config['max_heave_mm'],50)
        self.assertEqual(state['platform']['travel_bounds_mm'],[[-50,50]]*3)

    def test_invalid_unconfirmed_stale_and_active_changes_preserve_configuration(self):
        original = self.path.read_bytes()
        with self.assertRaises(UDPError):
            self.client.configure_platform(travel(),self.reference_id)
        with self.assertRaises(UDPError):
            self.client.configure_platform(travel(),'stale',calibration_confirmed=True)
        for values in ([dict(order=1,up_mm=0,down_mm=50),*travel()[1:]],
                       [dict(order=1,up_mm=float('nan'),down_mm=50),*travel()[1:]],travel()[:2]):
            with self.assertRaises((UDPError,ValueError)):
                self.client.configure_platform(values,self.reference_id,calibration_confirmed=True)
        self.client.enable([1,2,3])
        self.client.wait_for(('enabled',))
        with self.assertRaises(UDPError):self.configure()
        self.assertEqual(self.path.read_bytes(),original)
        self.assertFalse(self.service.platform.config['enabled'])

    def test_write_failure_leaves_runtime_and_file_unchanged(self):
        original = self.path.read_bytes()
        with patch('platform_setup.save_platform_configuration',side_effect=OSError('write denied')):
            with self.assertRaises(UDPError):self.configure()
        self.assertEqual(self.path.read_bytes(),original)
        self.assertFalse(self.service.platform.config['enabled'])

    def test_duplicate_configure_request_does_not_change_calibration_version(self):
        with MotorClient(port=self.server.port) as observer:
            observer.hello(claim=False)
            with self.assertRaises(UDPError):
                observer.configure_platform(travel(),self.reference_id,calibration_confirmed=True)
        packet = dict(v=1,type='configure_platform',id='configure-once',session=self.client.session,
                      control_seq=self.client.control_sequence+1,travel_mm=travel(),
                      reference_id=self.reference_id,calibration_confirmed=True)
        peer = ('127.0.0.1',self.client.sock.getsockname()[1])
        first = self.server.handle(packet,peer)
        self.assertTrue(first['ok'])
        version = self.service.platform.config['calibration_id']
        self.assertEqual(first,self.server.handle(packet,peer))
        self.assertEqual(self.service.platform.config['calibration_id'],version)


class SetupUITests(unittest.TestCase):
    def test_stop_save_setup_confirm_arm_and_drag_heights(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.json'
            path.write_text('{}',encoding='utf-8')
            config = options()
            config['pose_timeout_s'] = .5
            controller = SimulatedController(PlatformGeometry(config))
            service = MotorService(hardware_controller=controller, configuration_path=path,
                                   reference_path=path.with_name('reference.json'))
            server = UDPServer(service,port=0).start()
            root = tk.Tk()
            ui = DebugWindow(root,'127.0.0.1',server.port)
            def pump(predicate):
                end = time.monotonic()+5
                while time.monotonic()<end:
                    root.update()
                    if predicate():return
                    time.sleep(.005)
                self.fail(str(service.snapshot()))
            try:
                pump(lambda:bool(ui.state) and not ui.pending)
                ui.claim()
                pump(lambda:ui.state.get('phase')=='enabled' and not ui.enable_pending)
                with patch('UdpControl.debug_ui.messagebox.showinfo') as info:
                    ui.platform_setup_button.invoke()
                    info.assert_called_once()
                self.assertEqual(str(ui.neutral_box.cget('state')),'disabled')
                ui.stop()
                pump(lambda:ui.state.get('phase')=='idle' and not ui.stop_pending)
                with patch('UdpControl.debug_ui.messagebox.askyesno',return_value=True):
                    ui.save_fixed_reference()
                    pump(lambda:ui.state.get('fixed_reference',{}).get('valid') and not ui.pending)
                ui.open_platform_setup()
                dialog = [child for child in root.winfo_children() if isinstance(child,tk.Toplevel)][0]
                controls = dialog.winfo_children()[0].winfo_children()
                for entry in [child for child in controls if isinstance(child,ttk.Entry)]:
                    entry.insert(0,'50')
                [child for child in controls if isinstance(child,ttk.Checkbutton)][0].invoke()
                [child for child in controls if isinstance(child,ttk.Button)][0].invoke()
                pump(lambda:ui.state.get('platform',{}).get('configured') and not ui.pending)
                self.assertEqual(str(ui.neutral_box.cget('state')),'normal')
                self.assertEqual(ui.controls.select(),str(ui.height_controls))
                # Leave the default backend 250 ms unchanged; prevent first-paint
                # noise in this Windows GUI lifecycle test (covered by UDP tests).
                service.platform.config['pose_timeout_s'] = .5
                ui.neutral_box.invoke()
                ui.claim()
                pump(lambda:ui.height_active and not ui.enable_pending)
                self.assertEqual(str(ui.height_sliders[0].cget('state')),'normal')
                ui.drag_height(0,'1')
                pump(lambda:service.platform_lengths==[1,0,0])
                ui.stop()
                pump(lambda:ui.state.get('phase')=='idle')
                self.assertGreater(ui.log.winfo_height(),20)
            finally:
                ui.close();root.destroy();server.close();service.close()
                del ui,root
                gc.collect()


if __name__=='__main__':unittest.main()
