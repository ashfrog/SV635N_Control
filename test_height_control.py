"""Three-height contract and GUI streaming through UDP; no physical hardware."""
import gc
import math
from pathlib import Path
import time
import tkinter as tk
import unittest

from backend import load_config
from control_common import ControlError
from motor_service import MotorService
from platform_motion import PlatformGeometry
from simulated_backend import SimulatedController
from udp_server import UDPServer
from UdpControl.client import MotorClient, UDPError
from UdpControl.debug_ui import DebugWindow


def options():
    config = load_config(Path(__file__).with_name('backend.platform.example.json'))['platform']
    config['enabled'] = True
    return config


class HeightGeometryTests(unittest.TestCase):
    def test_measured_triangle_and_absolute_signed_heights(self):
        config = options()
        config['legs'][1]['extension_sign'] = -1
        geometry = PlatformGeometry(config)
        for i, j, distance in ((0, 1, 670), (0, 2, 730), (1, 2, 730)):
            a, b = geometry.legs[i], geometry.legs[j]
            self.assertAlmostEqual(math.hypot(a['x_mm']-b['x_mm'], a['y_mm']-b['y_mm']), distance)
        values, heights, degrees = geometry.heights_to_targets([1, -2, 3])
        self.assertEqual(heights, [1, -2, 3])
        self.assertEqual(degrees, [72, 144, 216])
        self.assertAlmostEqual(values['heave_mm'], 2/3)
        self.assertEqual(geometry.heights_to_targets([1, -2, 3]), (values, heights, degrees))
        self.assertEqual(geometry.heights_to_targets([0, 0, 0])[2], [0, 0, 0])
        self.assertEqual(geometry.heights_to_targets([10, 10, 10])[2], [720, -720, 720])

    def test_height_plane_obeys_envelope_with_noncentred_reference(self):
        config = options()
        for leg in config['legs']:
            leg['x_mm'] -= 30
            leg['y_mm'] += 40
        geometry = PlatformGeometry(config)
        for heave, pitch, roll in ((2, 1, -1), (-1, -.5, .75), (10, 0, 0)):
            _, heights, targets = geometry.pose_to_targets(dict(heave_mm=heave, pitch_deg=pitch, roll_deg=roll))
            pose, restored, result = geometry.heights_to_targets(heights)
            self.assertEqual(restored, heights)
            self.assertEqual(result, targets)
            for key, value in zip(('heave_mm', 'pitch_deg', 'roll_deg'), (heave, pitch, roll)):
                self.assertAlmostEqual(pose[key], value)
        # In-range individual supports must still respect the platform envelope.
        for invalid in ([11, 11, 11], [-20, -20, 20], [0, 0], [True, 0, 0],
                        [math.nan, 0, 0], [math.inf, 0, 0], [21, 0, 0], None):
            with self.subTest(invalid=invalid), self.assertRaises(ControlError):
                geometry.heights_to_targets(invalid)


class HeightUDPTests(unittest.TestCase):
    def setUp(self):
        self.options = options()
        self.service = MotorService(platform_options=self.options,
            hardware_controller=SimulatedController(PlatformGeometry(self.options)))
        self.server = UDPServer(self.service, port=0).start()
        self.client = MotorClient(port=self.server.port)
        self.client.hello()
        self.client.request('scan')
        self.client.wait_for(('idle',))

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.service.close()

    def arm(self):
        self.client.enable_platform([1, 2, 3], self.options['calibration_id'], reference_confirmed=True)
        self.client.wait_for(('enabled',))

    def test_heights_move_signed_positions_then_zero_returns_to_same_reference(self):
        self.arm()
        for target in ([.5, -.5, .25], [0, 0, 0]):
            deadline = time.monotonic()+2
            while time.monotonic() < deadline:
                self.client.heights(target)
                state = self.client.status()
                actual = state['platform']['actual_lengths_mm']
                if all(value is not None and abs(value-goal) < .02 for value, goal in zip(actual, target)):
                    break
                time.sleep(.025)
            else:
                self.fail(str(state))
            self.assertEqual(state['platform']['lengths_mm'], target)
            self.assertEqual(state['run_id'], self.client.run_id)
            self.assertTrue(state['enabled'])  # Zero is a target, not servo off.

    def test_sequences_shared_with_pose_and_heartbeats_do_not_renew_height_stream(self):
        self.arm()
        run = self.client.run_id
        self.assertTrue(self.service.command(run, 1000))
        self.assertTrue(self.service.command(run, 5, heights=[.1, -.1, 0]))
        accepted_time = self.service.last_pose
        self.assertFalse(self.service.command(run, 5, heights=[math.inf, 0, 0]))
        self.assertFalse(self.service.command(run, 4, pose=dict(heave_mm=0, pitch_deg=0, roll_deg=0)))
        self.assertEqual(accepted_time, self.service.last_pose)
        state = self.client.wait_for(('idle',))
        self.assertIn('姿态流超时', state['stop_reason'])
        self.assertTrue(state['result']['all_disabled'])
        with self.assertRaises(UDPError):
            self.client.heights([0, 0, 0])

    def test_old_run_invalid_height_cannot_stop_current_run(self):
        self.arm()
        with self.assertRaises(UDPError):
            self.client.request('heights', run_id='old', seq=1, heights_mm=None)
        self.assertEqual(self.service.phase, 'enabled')
        with self.assertRaises(UDPError):
            self.client.heights([0, 0, 0], expected_run_id='old')
        self.client.heights([0, 0, 0])

    def test_missing_null_or_invalid_current_height_frame_stops_all_without_partial_update(self):
        for payload in ({}, dict(heights_mm=None), dict(heights_mm=[0, 0]),
                        dict(heights_mm=[0, 21, 0]), dict(heights_mm=[0, True, 0])):
            with self.subTest(payload=payload):
                self.arm()
                self.client.heights([.1, .2, -.1])
                before = list(self.service.platform_lengths)
                with self.assertRaises(UDPError):
                    self.client.request('heights', run_id=self.client.run_id, seq=100, **payload)
                state = self.client.wait_for(('idle',))
                self.assertEqual(state['platform']['lengths_mm'], before)
                self.assertTrue(state['result']['all_disabled'])


class HeightUITests(unittest.TestCase):
    def setUp(self):
        config = options()
        # Tk painting/scheduling on Windows is not a real-time clock. Exercise
        # UI lifecycle with some headroom; UDP tests cover the default 250 ms.
        config['pose_timeout_s'] = .5
        self.service = MotorService(platform_options=config,
            hardware_controller=SimulatedController(PlatformGeometry(config)))
        self.server = UDPServer(self.service, port=0).start()
        self.root = tk.Tk()
        self.ui = DebugWindow(self.root, '127.0.0.1', self.server.port)
        self.pump(lambda: self.ui.state.get('platform', {}).get('configured') and not self.ui.pending)

    def tearDown(self):
        self.ui.close()
        self.root.destroy()
        self.server.close()
        self.service.close()
        del self.ui, self.root
        gc.collect()

    def pump(self, predicate, timeout=5):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.root.update()
            if predicate():
                return
            time.sleep(.005)
        self.fail(str(self.service.snapshot()))

    def arm(self):
        self.ui.neutral_confirmed.set(True)
        self.ui.claim()
        self.pump(lambda: self.ui.height_active and not self.ui.enable_pending
                  and self.ui.state.get('run_id') == self.ui.client.run_id
                  and self.service.phase == 'enabled' and self.service.platform_lengths == [0, 0, 0])

    def test_three_independent_entries_common_slider_and_zero_keep_stream_alive(self):
        ui = self.ui
        self.assertIsNone(ui.client.session)  # Read-only opening never arms motors.
        self.assertEqual(ui.controls.select(), str(ui.height_controls))
        ui.claim()
        self.assertIsNone(ui.auto_start)
        self.arm()
        ui.height_vars[0].set('1.25')
        ui.height_vars[1].set('-2')
        self.pump(lambda: self.service.platform_lengths == [1.25, -2, 0])
        ui.drag_height(2, '0.5')
        self.pump(lambda: self.service.platform_lengths == [1.25, -2, .5])
        ui.drag_height(3, '-1')
        self.pump(lambda: self.service.platform_lengths == [-1, -1, -1])
        ui.height_zero_button.invoke()
        self.pump(lambda: self.service.platform_lengths == [0, 0, 0])
        self.pump(lambda: all(abs(value) < .02 for value in self.service.snapshot()['platform']['actual_lengths_mm']))
        start = time.monotonic()
        self.pump(lambda: time.monotonic()-start > .45)
        self.assertEqual(self.service.phase, 'enabled')
        self.assertEqual(str(ui.rpm_slider.cget('state')), 'disabled')
        self.assertEqual(ui.tree.heading('target')['text'], '目标高度 mm')
        self.assertTrue(ui.height_zero_button.winfo_viewable())

    def test_invalid_typing_stop_and_ui_hitch_do_not_replay_old_targets(self):
        self.arm()
        ui = self.ui
        ui.height_vars[0].set('nan')
        self.pump(lambda: ui.height_edit_handles[0] is None)
        self.assertEqual(self.service.platform_lengths, [0, 0, 0])
        ui.height_vars[0].set('99')
        ui.commit_height_edit(0)
        self.assertEqual(ui.height_values, [0, 0, 0])
        ui.height_vars[0].set('3')
        ui.stop()
        self.pump(lambda: ui.state.get('phase') == 'idle' and not ui.stop_pending and not ui.height_active)
        self.assertEqual(ui.height_edit_handles, [None]*4)
        self.assertFalse(ui.neutral_confirmed.get())
        self.arm()
        ui.drag_height(0, '1')
        self.pump(lambda: self.service.platform_lengths == [1, 0, 0])
        time.sleep(.65)  # Independent heartbeats cannot hide a stalled UI/height producer.
        self.pump(lambda: ui.state.get('phase') == 'idle')
        ui.claim()
        self.assertIsNone(ui.auto_start)
        self.assertEqual(self.service.phase, 'idle')


if __name__ == '__main__':
    unittest.main()
