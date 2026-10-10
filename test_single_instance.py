"""Single-instance startup checks using local sockets and fake services only."""
import logging
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import unittest
from unittest.mock import Mock, patch

import backend
from UdpControl.client import MotorClient
from UdpControl.instance import PanelInstance
from udp_server import UDPServer, bind_socket


class BackendInstanceTests(unittest.TestCase):
    def setUp(self):
        self.service = Mock()
        self.service.run_id = 'existing-run'
        self.service.is_busy.return_value = True
        self.service.snapshot.return_value = dict(phase='enabled', run_id='existing-run',
                                                  message='running', orders=[1])
        self.server = UDPServer(self.service, port=0, auth_key='instance-test').start()
        self.config = backend.load_config(Path('missing-instance-test.json'))
        self.config.update(port=self.server.port, auth_key='instance-test')

    def tearDown(self):
        self.server.close()

    def test_duplicate_backend_opens_panel_without_new_hardware_or_control_change(self):
        with MotorClient(port=self.server.port, auth_key=self.config['auth_key']) as owner:
            owner.hello()
            original_owner = self.server.owner
            original_session = self.server.token
            with patch('backend.load_config', return_value=self.config), \
                 patch('backend.sys.argv', ['backend.py', '--headless']), \
                 patch('backend.signal.signal'), patch('backend.MotorService') as factory, \
                 patch('backend.RotatingFileHandler') as handler, \
                 patch('backend.subprocess.Popen') as launch, patch('builtins.print'):
                self.assertEqual(backend.main(), 0)
                factory.assert_not_called()
                handler.assert_not_called()
                command = launch.call_args.args[0]
                self.assertEqual(command[1:3], ['-m', 'UdpControl'])
                self.assertEqual(command[command.index('--port') + 1], str(self.server.port))
            self.assertEqual(self.server.owner, original_owner)
            self.assertEqual(self.server.token, original_session)
            self.assertEqual(owner.status()['run_id'], 'existing-run')
            self.service.scan.assert_not_called()
            self.service.enable.assert_not_called()
            self.service.stop.assert_not_called()
            self.service.close.assert_not_called()

    def test_auth_mismatch_does_not_open_panel_or_create_service(self):
        self.config['auth_key'] = 'wrong-key'
        with patch('backend.load_config', return_value=self.config), \
             patch('backend.sys.argv', ['backend.py', '--headless']), \
             patch('backend.signal.signal'), patch('backend.MotorService') as factory, \
             patch('backend.launch_client') as launch, patch('builtins.print'), \
             patch('backend.logging.exception'):
            self.assertEqual(backend.main(), 1)
            launch.assert_not_called()
            factory.assert_not_called()
        self.assertFalse(self.server.clients)

    def test_occupied_foreign_port_is_not_treated_as_our_backend(self):
        with bind_socket('127.0.0.1', 0) as foreign:
            self.config['port'] = foreign.getsockname()[1]
            self.assertFalse(backend.existing_backend(self.config, timeout=.1))

    def test_endpoint_stays_reserved_until_hardware_cleanup_finishes(self):
        config = dict(self.config, port=0)
        service = Mock()
        service.lock = threading.RLock()
        service.is_busy.return_value = False
        service.snapshot.return_value = dict(phase='idle', run_id=None)
        observed = []
        def tray(_service, server, *_):
            observed.append(server)
        def cleanup():
            self.assertTrue(observed[0].stopping.is_set())
            with self.assertRaises(OSError):
                bind_socket('127.0.0.1', observed[0].port)
        service.close.side_effect = cleanup
        with patch('backend.load_config', return_value=config), \
             patch('backend.sys.argv', ['backend.py']), patch('backend.signal.signal'), \
             patch('backend.RotatingFileHandler', return_value=logging.NullHandler()), \
             patch('backend.logging.basicConfig'), \
             patch('backend.MotorService', return_value=service), \
             patch('backend.run_tray', side_effect=tray):
            self.assertEqual(backend.main(), 0)
        with bind_socket('127.0.0.1', observed[0].port):
            pass
        service.scan.assert_called_once()


@unittest.skipUnless(os.name == 'nt', 'Windows panel activation')
class PanelInstanceTests(unittest.TestCase):
    def setUp(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            self.port = sock.getsockname()[1]
        # Separate test namespace; never activate a user's hardware panel.
        self.host = '127.0.0.234'

    def test_activation_is_kept_during_startup_and_lock_can_be_reopened(self):
        first = PanelInstance(self.host, self.port)
        second = None
        try:
            self.assertTrue(first.primary)
            second = PanelInstance(self.host, self.port)
            self.assertFalse(second.primary)
            second.activate()
            self.assertTrue(first.requested())
            self.assertFalse(first.requested())
        finally:
            if second:
                second.close()
            first.close()
            first.close()
        reopened = PanelInstance(self.host, self.port)
        try:
            self.assertTrue(reopened.primary)
        finally:
            reopened.close()

    def test_other_process_signals_original_without_owning_the_panel(self):
        first = PanelInstance(self.host, self.port)
        try:
            script = ('from UdpControl.instance import PanelInstance; '
                      f'instance=PanelInstance({self.host!r},{self.port}); '
                      'assert not instance.primary; instance.activate(); instance.close()')
            result = subprocess.run([sys.executable, '-c', script], capture_output=True,
                                    timeout=5, cwd=Path(__file__).resolve().parent)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(first.requested())
        finally:
            first.close()

    def test_abrupt_process_exit_leaves_no_stale_lock(self):
        script = ('import os; from UdpControl.instance import PanelInstance; '
                  f'instance=PanelInstance({self.host!r},{self.port}); '
                  'assert instance.primary; os._exit(0)')
        result = subprocess.run([sys.executable, '-c', script], capture_output=True,
                                timeout=5, cwd=Path(__file__).resolve().parent)
        self.assertEqual(result.returncode, 0, result.stderr)
        instance = PanelInstance(self.host, self.port)
        try:
            self.assertTrue(instance.primary)
        finally:
            instance.close()

    def test_duplicate_gui_does_not_create_tk_or_a_log_handler(self):
        from UdpControl import __main__ as gui
        first = PanelInstance(self.host, self.port)
        try:
            config = dict(host=self.host, port=self.port, auth_key='')
            with patch.object(gui, 'load_config', return_value=config), \
                 patch.object(sys, 'argv', ['UdpControl']), \
                 patch.object(gui.tk, 'Tk') as root, \
                 patch.object(gui, 'RotatingFileHandler') as handler:
                self.assertEqual(gui.main(), 0)
                root.assert_not_called()
                handler.assert_not_called()
            self.assertTrue(first.requested())
        finally:
            first.close()

    def test_activation_restores_and_foregrounds_existing_gui(self):
        from UdpControl import __main__ as gui
        instance = Mock(primary=True)
        instance.requested.return_value = True
        root, window = Mock(), Mock()
        callbacks = []
        root.after.side_effect = lambda delay, callback: callbacks.append(callback)
        root.mainloop.side_effect = lambda: callbacks[0]()
        config = dict(host=self.host, port=self.port, auth_key='')
        with patch.object(gui, 'load_config', return_value=config), \
             patch.object(sys, 'argv', ['UdpControl']), \
             patch.object(gui, 'PanelInstance', return_value=instance), \
             patch.object(gui.tk, 'Tk', return_value=root), \
             patch.object(gui, 'DebugWindow', return_value=window), \
             patch.object(gui, 'RotatingFileHandler', return_value=logging.NullHandler()), \
             patch.object(gui.logging, 'basicConfig'):
            self.assertEqual(gui.main(), 0)
        window.show.assert_called_once()
        root.attributes.assert_called_with('-topmost', True)
        root.focus_force.assert_called_once()
        instance.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
