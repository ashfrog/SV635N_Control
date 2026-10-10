"""Frozen launch paths must preserve process roles and external configuration."""
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch

import backend
from runtime_paths import application_dir


class FrozenPathTests(unittest.TestCase):
    def test_config_and_logs_live_beside_executable_not_internal_bundle(self):
        executable=Path('C:/SVServer 测试/SVServer.exe')
        with patch.object(sys,'frozen',True,create=True), patch.object(sys,'executable',str(executable)):
            self.assertEqual(application_dir('C:/SVServer 测试/_internal/backend.py'),executable.parent.resolve())
        source=Path(__file__).resolve().parent/'backend.py'
        self.assertEqual(application_dir(source),source.parent)

    def test_frozen_tray_launches_client_role_without_python_module_flags(self):
        executable=Path('C:/SVServer 测试/SVServer.exe')
        config_path=executable.parent/'backend.config.json'
        with patch.object(sys,'frozen',True,create=True), patch.object(sys,'executable',str(executable)), \
             patch.object(backend,'BASE',executable.parent), patch('backend.subprocess.Popen') as launch:
            backend.launch_client(dict(host='0.0.0.0',port=5005),config_path,port=6123)
        command=launch.call_args.args[0]
        self.assertEqual(command[:2],[str(executable),'--client'])
        self.assertEqual(command[command.index('--host')+1],'127.0.0.1')
        self.assertEqual(command[command.index('--port')+1],'6123')
        self.assertEqual(command[command.index('--config')+1],str(config_path))
        self.assertEqual(launch.call_args.kwargs['cwd'],executable.parent)
        self.assertEqual(launch.call_args.kwargs['stdin'],subprocess.DEVNULL)


if __name__=='__main__':unittest.main()
