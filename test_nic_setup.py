"""The privileged helper never changes a real NIC during tests."""
import base64
from contextlib import nullcontext
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import subprocess
import unittest
from unittest.mock import patch

from control_common import ControlError, DEFAULT_ADAPTER
from nic_setup import optimize_nic
from udp_server import bind_socket


class NICSetupTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(backend='ethercat',adapter=DEFAULT_ADAPTER,host='127.0.0.1',port=0)

    def test_no_admin_or_active_service_never_touches_hardware(self):
        with patch('nic_setup.administrator', return_value=False), patch('nic_setup.inspect_disabled') as scan:
            with self.assertRaisesRegex(ControlError, '管理员'):
                optimize_nic(self.config, 'unused')
            scan.assert_not_called()
        with bind_socket('127.0.0.1', 0) as occupied:
            self.config['port'] = occupied.getsockname()[1]
            with patch('nic_setup.administrator', return_value=True), patch('nic_setup.inspect_disabled') as scan:
                with self.assertRaisesRegex(ControlError, '停止并退出后台'):
                    optimize_nic(self.config, 'unused')
                scan.assert_not_called()

    def test_enabled_drive_refusal_never_runs_network_commands(self):
        with patch('nic_setup.administrator', return_value=True), \
             patch('nic_setup.adapter_lock', return_value=nullcontext()), \
             patch('nic_setup.inspect_disabled', side_effect=ControlError('轴仍使能')), \
             patch('nic_setup.subprocess.run') as command:
            with self.assertRaisesRegex(ControlError, '仍使能'):
                optimize_nic(self.config, 'unused')
            command.assert_not_called()

    def test_success_verifies_bus_after_changes_and_keeps_backup_journal(self):
        devices = [dict(order=1, identity=[1,2,3], statusword=0x40, error_code=0)]
        run_parser = subprocess.run
        with tempfile.TemporaryDirectory() as folder, \
             patch('nic_setup.administrator', return_value=True), \
             patch('nic_setup.adapter_lock', return_value=nullcontext()), \
             patch('nic_setup.inspect_disabled', return_value=devices) as scan, \
             patch('nic_setup.subprocess.run', return_value=SimpleNamespace(returncode=0,stdout='verified',stderr='')) as command:
            backup = optimize_nic(self.config, folder)
            self.assertEqual(scan.call_count, 2)
            script = base64.b64decode(command.call_args.args[0][-1]).decode('utf-16-le')
            self.assertIn('89b39964-a086-4a5d-bc55-d5804ae703c6', script)
            self.assertIn('-NoRestart', script)
            self.assertIn('Restart-NetAdapter -Name $motorNic.Name', script)
            self.assertIn(str(backup.resolve()), script)
            self.assertIn('foreach ($item in $original)', script)  # Rollback is scoped to backed-up properties.
            result = json.loads(next(Path(folder).glob('nic-setup-*.json')).read_text(encoding='utf-8'))
            self.assertTrue(result['success'])
            self.assertEqual(result['before'], result['after'])
            # Parse, never execute, the generated administrator commands.
            parser = '$tokens=$null; $errors=$null; [System.Management.Automation.Language.Parser]::ParseInput([Console]::In.ReadToEnd(), [ref]$tokens, [ref]$errors) > $null; if ($errors) { $errors | Out-String | Write-Output; exit 1 }'
            for content in [script]+[p.read_text(encoding='utf-8').split('-Command "',1)[1].rstrip().rstrip('"')
                                    for p in (Path(__file__).parent/'优化电机网口（管理员）.cmd',
                                              Path(__file__).parent/'packaging'/'优化电机网口（管理员）.cmd')]:
                parsed = run_parser(['powershell.exe','-NoProfile','-NonInteractive','-Command',parser],
                                    input=content, capture_output=True, text=True)
                self.assertEqual(parsed.returncode, 0, parsed.stdout+parsed.stderr)

    def test_network_failure_records_reason_without_faking_success(self):
        with tempfile.TemporaryDirectory() as folder, \
             patch('nic_setup.administrator', return_value=True), \
             patch('nic_setup.adapter_lock', return_value=nullcontext()), \
             patch('nic_setup.inspect_disabled', return_value=[]) as scan, \
             patch('nic_setup.subprocess.run', return_value=SimpleNamespace(returncode=1,stdout='',stderr='Denied')):
            with self.assertRaisesRegex(ControlError, 'Denied'):
                optimize_nic(self.config, folder)
            self.assertEqual(scan.call_count, 1)
            result = json.loads(next(Path(folder).glob('nic-setup-*.json')).read_text(encoding='utf-8'))
            self.assertFalse(result['success'])

    def test_link_renegotiation_is_retried_without_repeating_nic_changes(self):
        devices = [dict(order=1,identity=[1,2,3],statusword=0x40,error_code=0)]
        with tempfile.TemporaryDirectory() as folder, \
             patch('nic_setup.administrator', return_value=True), \
             patch('nic_setup.adapter_lock', return_value=nullcontext()), \
             patch('nic_setup.inspect_disabled', side_effect=[devices,ControlError('扫描到 -1 台设备'),devices]) as scan, \
             patch('nic_setup.time.sleep') as pause, \
             patch('nic_setup.subprocess.run', return_value=SimpleNamespace(returncode=0,stdout='',stderr='')) as command:
            optimize_nic(self.config,folder)
            self.assertEqual(scan.call_count,3)
            command.assert_called_once()
            pause.assert_called_once_with(.5)
