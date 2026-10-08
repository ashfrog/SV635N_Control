"""Tk flow tests: real local session with an emulated EtherCAT master."""
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from app import App
from continuous_control import run_continuous
from core import EtherCATController
from test_platform_control import FakeMaster


class AppTests(unittest.TestCase):
    def setUp(self):
        self.master = FakeMaster()
        self.controller = EtherCATController('fake', master_factory=lambda: self.master)
        self.patches = [patch.object(EtherCATController, 'adapters', return_value=[]),
                        patch('app.run_continuous', side_effect=self.run_fake),
                        patch('app.messagebox.showerror')]
        for p in self.patches:
            p.start()
        self.window = App()
        self.window.devices = self.controller._discover(self.master)
        self.window.adapter = lambda: 'fake'
        for device in self.window.devices:
            self.window.tree.insert('', 'end', iid=str(device.order),
                                    values=('☐', device.order, device.alias, 'fake', '1:1', device.position, '0', '未使能'))
            self.window.target_values[device.order] = '0'
        self.window.selected = {1, 2, 3}
        self.window.ready.set(True)
        self.window.render_selection()
        self.window.update()

    def tearDown(self):
        if self.window.winfo_exists():
            if self.window.busy:
                self.window.stop()
                self.pump(lambda: not self.window.busy)
            self.window.close()
        for p in reversed(self.patches):
            p.stop()

    def run_fake(self, ignored_controller, devices, commands, stop, callback):
        return run_continuous(self.controller, devices, commands, stop, callback)

    def pump(self, condition, timeout=5):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            self.window.update()
            time.sleep(.01)
        self.assertTrue(condition(), self.window.status.get())

    def enable(self):
        self.window.continuous_enabled.set(True)
        self.window.toggle_continuous()
        self.pump(lambda: self.window.continuous_ready or not self.window.busy)
        self.assertTrue(self.window.continuous_ready)

    def test_edit_targets_relative_origin_and_disable(self):
        self.enable()
        self.assertFalse(any(slave.targets for slave in self.master.slaves))
        self.assertEqual(str(self.window.rpm_entry.cget('state')), 'disabled')
        self.assertEqual(str(self.window.angle_entry.cget('state')), 'normal')
        self.assertEqual(str(self.window.continuous_switch.cget('state')), 'normal')
        selected = set(self.window.selected)
        x, y, width, height = self.window.tree.bbox('1', 'target')
        self.window.toggle(SimpleNamespace(x=x + width // 2, y=y + height // 2))
        self.assertEqual(self.window.selected, selected)
        self.window.edit_target(SimpleNamespace(x=x + width // 2, y=y + height // 2))
        self.assertIsNotNone(self.window.target_editor)
        editor = self.window.target_editor[1]
        editor.delete(0, 'end')
        editor.insert(0, '2')
        self.window.finish_target_edit()
        self.window.set_target(2, '-3')
        self.window.set_target(3, '4')
        self.window.send_targets()
        commands = self.window.motion_queue
        self.assertEqual(commands.planned, (2, -3, 4))
        self.window.angle.set('1')
        self.window.run_motion(False)
        self.assertEqual(commands.planned, (3, -2, 5))
        self.window.send_targets(origin=True)
        self.assertEqual(commands.planned, (0, 0, 0))
        self.pump(lambda: '指令 #3 已完成' in self.window.queue_text.get())
        self.assertTrue(self.window.continuous_ready)
        self.assertTrue(all(slave.sw & 4 for slave in self.master.slaves[:3]))
        self.window.continuous_enabled.set(False)
        self.window.toggle_continuous()
        self.assertTrue(self.window.stop_token.is_set())
        self.assertFalse(self.window.continuous_ready)
        self.assertEqual(str(self.window.start_button.cget('state')), 'disabled')
        self.pump(lambda: not self.window.busy)
        self.assertFalse(self.window.continuous_active)
        self.assertFalse(self.window.continuous_enabled.get())
        self.assertEqual(commands.pending(), 0)
        self.assertTrue(all(not slave.sw & 4 for slave in self.master.slaves))
        self.assertEqual(str(self.window.rpm_entry.cget('state')), 'normal')

    def test_cancel_preparation_blocks_late_ready_event(self):
        self.window.continuous_enabled.set(True)
        self.window.toggle_continuous()
        self.window.stop()
        self.window.events.put({'kind': 'continuous_ready', 'orders': [1, 2, 3]})
        self.window.poll()
        self.assertFalse(self.window.continuous_ready)
        self.pump(lambda: not self.window.busy)
        self.assertFalse(any(slave.targets for slave in self.master.slaves))

    def test_single_motion_keeps_its_original_behavior(self):
        self.window.angle.set('1')
        with patch('app.EtherCATController', return_value=self.controller):
            self.window.run_motion(False)
        self.assertTrue(self.window.busy)
        self.assertEqual(str(self.window.continuous_switch.cget('state')), 'disabled')
        self.pump(lambda: not self.window.busy)
        self.assertIn('运动完成', self.window.status.get())
        self.assertTrue(all(len(s.targets) == 1 for s in self.master.slaves[:3]))
        self.assertTrue(all(not s.sw & 4 for s in self.master.slaves))
        self.assertFalse(self.window.continuous_ready)
        self.assertEqual(self.window.start_button.cget('text'), '开始单次运行')


if __name__ == '__main__':
    unittest.main()
