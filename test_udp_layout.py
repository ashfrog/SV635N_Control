"""Enforce independent frontend/backend imports and deployment."""
import subprocess
import sys
from pathlib import Path
import shutil
import tempfile
import unittest


class LayoutTests(unittest.TestCase):
    def test_client_folder_runs_without_backend_or_site_packages(self):
        root = Path(__file__).resolve().parent
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)/'UdpControl'
            target.mkdir()
            for source in (root/'UdpControl').glob('*.py'):
                shutil.copy2(source, target/source.name)
            script = '''
import sys
from UdpControl.client import MotorClient
from UdpControl.debug_ui import DebugWindow
from UdpControl.__main__ import load_config
assert not any(name in sys.modules for name in ('core','pysoem','motor_service','udp_server','backend','pystray','PIL'))
print('independent client imports OK')
'''
            result = subprocess.run([sys.executable,'-S','-c',script],cwd=directory,
                                    capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)
            result = subprocess.run([sys.executable,'-S','-m','UdpControl','--help'],cwd=directory,
                                    capture_output=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_backend_does_not_import_frontend(self):
        script = '''
import sys
import backend, motor_service, udp_server
assert not any(name=='UdpControl' or name.startswith('UdpControl.') for name in sys.modules)
assert 'tkinter' not in sys.modules
assert 'pysoem' not in sys.modules
print('independent backend imports OK')
'''
        result = subprocess.run([sys.executable,'-c',script],cwd=Path(__file__).resolve().parent,
                                capture_output=True,text=True,timeout=10)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_wire_format_matches_across_process_components(self):
        import udp_server
        from UdpControl import wire
        for packet in (dict(v=1,type='target',targets_deg=[1,0,-2]),dict(v=1,type='hello',id='hello')):
            self.assertEqual(udp_server.decode(wire.encode(packet)),packet)
            self.assertEqual(wire.decode(udp_server.encode(packet)),packet)


if __name__=='__main__':
    unittest.main()
