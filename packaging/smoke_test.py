"""Verify the built EXE after relocation, without Python on PATH or real APS access."""
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from UdpControl.client import MotorClient, UDPError
from udp_server import UDPServer


def wait_for(predicate, process, timeout=10):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        if predicate():
            return
        if process.poll() is not None:
            raise RuntimeError(f'Packaged process exited early: {process.returncode}')
        time.sleep(.05)
    raise TimeoutError('Packaged process did not become ready')


def stop_process(process):
    if process.poll() is None:
        process.terminate()
    process.wait(timeout=10)


def free_port():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1',0))
        return sock.getsockname()[1]


def main():
    parent=ROOT/'.build'/'svserver'
    parent.mkdir(parents=True,exist_ok=True)
    env=os.environ.copy()
    env['PATH']=str(Path(os.environ['SystemRoot'])/'System32')
    for key in ('PYTHONHOME','PYTHONPATH','VIRTUAL_ENV'):
        env.pop(key,None)
    startup=subprocess.STARTUPINFO()
    startup.dwFlags|=subprocess.STARTF_USESHOWWINDOW
    startup.wShowWindow=0
    results={}
    with tempfile.TemporaryDirectory(prefix='SVServer relocation 测试 ',dir=parent) as directory:
        workspace=Path(directory).resolve()
        assert workspace.is_relative_to(parent.resolve())
        deployed=workspace/'移动后的服务'
        shutil.copytree(ROOT/'SVServer',deployed,ignore=shutil.ignore_patterns(
            'logs','backend.config.json','*.before-platform.bak','platform.reference.json','platform.simulated.reference.json'))
        executable=deployed/'SVServer.exe'
        def start(arguments):
            return subprocess.Popen([str(executable),*arguments],cwd=workspace,env=env,
                stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
                startupinfo=startup)

        # Default configuration must resolve beside the relocated EXE.
        port=free_port()
        process=start(['--simulate','--port',str(port)])
        try:
            with MotorClient(port=port) as client:
                def ready():
                    try:
                        client.hello(claim=False)
                        return client.state.get('phase')=='idle'
                    except UDPError:
                        return False
                wait_for(ready,process)
                client.hello()
                client.save_reference(physical_center_confirmed=True)
                client.wait_for(('idle',))
                client.enable_platform([1,2,3],'simulation-only-v1',reference_confirmed=True)
                client.wait_for(('enabled',))
                reference_id = client.state['fixed_reference']['id']
                for heights in ([.2, -.2, .1], [0, 0, 0], [.2, .2, .2]):
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline:
                        client.heights(heights)
                        state=client.status()
                        actual=state['platform']['actual_lengths_mm']
                        if all(value is not None and abs(value-target)<.02
                               for value,target in zip(actual,heights)):
                            break
                        time.sleep(.025)
                    else:
                        raise AssertionError('Packaged support heights did not reach target')
                    assert state['platform']['lengths_mm']==heights and state['enabled']
                    assert len(state['platform']['travel_bounds_mm'])==3
                client.disable()
                state=client.wait_for(('idle','fault'))
                assert state['hardware_backend']=='simulated' and state['result']['all_disabled']
                client.return_reference(reference_id, position_reference_confirmed=True)
                client.wait_for(('enabled',))
                wait_for(lambda:all(abs(value)<.02 for value in
                    client.status()['fixed_reference']['heights_mm']),process)
                assert client.state['control_mode']=='reference'
                assert client.state['profile']['rpm']==5
                client.disable()
                client.wait_for(('idle',))
                client.release()
                results['relocated_simulation_and_platform_udp']='passed'
                results['relocated_signed_heights_and_return_to_neutral']='passed'
                results['relocated_fixed_reference_low_speed_return']='passed'
        finally:
            stop_process(process)

        saved = deployed/'platform.simulated.reference.json'
        assert saved.exists() and json.loads(saved.read_text(encoding='utf-8'))['id']==reference_id
        process=start(['--simulate','--port',str(port)])
        try:
            with MotorClient(port=port) as client:
                def restored():
                    try:
                        client.hello(claim=False)
                        reference=client.state.get('fixed_reference',{})
                        return reference.get('valid') and reference.get('id')==reference_id
                    except UDPError:
                        return False
                wait_for(restored,process)
                assert client.state['phase']=='idle' and not client.state['enabled']
                results['relocated_fixed_reference_survives_process_restart']='passed'
        finally:
            stop_process(process)

        # Test the actual backend/tray entry, while making DLL access impossible.
        port=free_port()
        missing_dll=workspace/'intentionally-missing-APS.dll'
        assert not missing_dll.exists()
        config=dict(host='127.0.0.1',port=port,aps=dict(board_id=31,dll_path=str(missing_dll)))
        (deployed/'backend.config.json').write_text(json.dumps(config),encoding='utf-8')
        process=start([])
        try:
            with MotorClient(port=port) as client:
                def fault_ready():
                    try:
                        client.hello(claim=False)
                        return client.state.get('phase')=='fault'
                    except UDPError:
                        return False
                wait_for(fault_ready,process)
                assert 'APS' in client.state['message']
                assert not client.state['enabled']
                time.sleep(.3)
                assert process.poll() is None
                assert (deployed/'logs'/'service.log').exists()
                assert not (deployed/'_internal'/'logs').exists()
                results['relocated_backend_tray_external_config_and_logs']='passed'
        finally:
            stop_process(process)  # No APS DLL was loaded and no hardware was initialized.

        # The compiled client connects read-only, then closes on backend shutdown.
        service=Mock()
        service.is_busy.return_value=False
        service.snapshot.return_value=dict(phase='idle',enabled=False,run_id=None,orders=[],
            devices=[],axes=[],adapter='PCIe-8332:0',message='Packaging smoke test')
        service.adapters.return_value=[dict(name='PCIe-8332:0',description='Mock backend')]
        server=UDPServer(service,port=0).start()
        process=start(['--client','--port',str(server.port)])
        try:
            wait_for(lambda:service.adapters.call_count>0,process)
            assert server.owner is None  # Opening a panel must not claim or enable.
            assert (deployed/'logs'/'client.log').exists()
            server.notify_shutdown()
            assert process.wait(timeout=10)==0
            service.enable.assert_not_called()
            results['frozen_client_read_only_connect_and_graceful_shutdown']='passed'
        finally:
            stop_process(process)
            server.close()
    (ROOT/'SVServer'/'verification.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(results,indent=2))


if __name__=='__main__':main()
