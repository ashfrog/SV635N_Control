"""Windows tray process; --headless provides the same service without a UI."""
import argparse
import errno
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import signal
import secrets
import socket
import subprocess
import sys
import threading
import time

from aps_backend import validate_options
from udp_server import UDPServer, bind_socket, encode, decode, PROTOCOL_VERSION, MAX_PACKET
from motor_service import MotorService

BASE=Path(__file__).resolve().parent


def load_config(path):
    config=dict(host='127.0.0.1',port=5005,adapter=None,heartbeat_timeout_s=.5,auth_key='',aps={})
    if path.exists():
        supplied=json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(supplied,dict) or set(supplied)-set(config):
            raise ValueError('配置包含未知字段，或不是 JSON 对象。')
        config.update(supplied)
    if not isinstance(config['host'],str) or type(config['port']) is not int or not 1<=config['port']<=65535:
        raise ValueError('host/port 配置无效。')
    config['aps'] = validate_options(config['aps'])
    configured_adapter = f"PCIe-8332:{config['aps']['board_id']}"
    if config['adapter'] is None:
        config['adapter'] = configured_adapter
    if config['adapter'] != configured_adapter:
        raise ValueError('adapter 请改为 ' + configured_adapter + '；PCIe-8332 后台不使用 NPF 网卡。')
    if not isinstance(config['auth_key'],str):
        raise ValueError('adapter/auth_key 配置无效。')
    return config


def client_host(config):
    return '127.0.0.1' if config['host'] in ('0.0.0.0','localhost') else config['host']


def existing_backend(config, timeout=2):
    """Read-only identification; never claim control or change motor state."""
    peer = (socket.gethostbyname(client_host(config)), config['port'])
    request_id = secrets.token_hex(16)
    packet = encode(dict(v=PROTOCOL_VERSION,type='hello',id=request_id,claim=False,
                         auth_key=config['auth_key']))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(.15)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            sock.sendto(packet, peer)
            try:
                data, sender = sock.recvfrom(MAX_PACKET + 1)
                if sender != peer:
                    continue
                reply = decode(data)
                if (reply.get('v') == PROTOCOL_VERSION and reply.get('type') == 'ack'
                        and reply.get('id') == request_id):
                    return (reply.get('ok') is True and isinstance(reply.get('server_id'), str)
                            and isinstance(reply.get('state'), dict))
            except (OSError, ValueError, UnicodeError, RecursionError):
                continue
    return False


def launch_client(config, config_path, port=None):
    executable = Path(sys.executable)
    windowed = executable.with_name('pythonw.exe')
    if windowed.exists():
        executable = windowed
    return subprocess.Popen([str(executable),'-m','UdpControl',
        '--host',client_host(config),'--port',str(port or config['port']),
        '--config',str(config_path or BASE/'backend.config.json')],
        cwd=BASE,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)


def run_tray(service,server,config,quit_event,config_path=None):
    # The backend owns no Tk objects. Its icon launches a separate UDP client.
    import pystray
    from PIL import Image,ImageDraw
    client_process=None
    launch_lock=threading.Lock()
    def open_client(*_):
        nonlocal client_process
        with launch_lock:
            if quit_event.is_set() or service.shutdown.is_set():
                return
            try:
                # The frontend receives only the endpoint and config file path;
                # it has no service object or hardware reference.
                process=launch_client(config,config_path,server.port)
                # A duplicate GUI launch only foregrounds the existing panel.
                # Keep ownership of our original child for shutdown cleanup.
                if client_process is None or client_process.poll() is not None:
                    client_process=process
            except OSError:
                logging.exception('Unable to launch UDP control UI')
    icon_image=Image.new('RGB',(64,64),'#163a63')
    draw=ImageDraw.Draw(icon_image)
    draw.ellipse((10,10,54,54),outline='white',width=5)
    draw.line((32,20,32,44),fill='#52c5ff',width=6)
    icon=pystray.Icon('SV635N_UDP',icon_image,'SV635N UDP 电机后台',pystray.Menu(
        pystray.MenuItem('打开控制界面',open_client,default=True),
        pystray.MenuItem('停止全部电机',lambda *_:service.stop('托盘停止')),
        pystray.MenuItem('停止并退出后台',lambda *_:quit_event.set())))
    try:
        icon.run_detached()
        while not quit_event.wait(.1):
            if server.closed.is_set():
                break
    finally:
        service.shutdown.set()
        service.stop('后台退出')
        try:
            server.notify_shutdown()
            with launch_lock:
                if client_process is not None and client_process.poll() is None:
                    try:
                        client_process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        logging.warning('Control UI did not exit after shutdown; terminating tray child')
                        client_process.terminate()
                        client_process.wait(timeout=3)
        finally:
            icon.stop()


def main():
    parser=argparse.ArgumentParser(description='SV635N UDP 后台电机控制服务')
    parser.add_argument('--headless',action='store_true',help='不显示托盘，用于控制台/服务宿主')
    parser.add_argument('--config',type=Path,default=BASE/'backend.config.json')
    args=parser.parse_args()
    log_dir=BASE/'logs'
    log_dir.mkdir(exist_ok=True)
    service=server=reserved_socket=None
    quit_event=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:quit_event.set())
    signal.signal(signal.SIGTERM,lambda *_:quit_event.set())
    try:
        config=load_config(args.config)
        try:
            reserved_socket=bind_socket(config['host'],config['port'])
        except OSError as exc:
            # Windows exclusive UDP binds can report access denied (10013)
            # instead of address in use (10048). Verify our protocol first.
            if ((exc.errno in (errno.EADDRINUSE, errno.EACCES)
                    or getattr(exc,'winerror',None) in (10048,10013))
                    and existing_backend(config)):
                launch_client(config,args.config.resolve())
                if args.headless:
                    print('后台已在运行，已打开客户端调试面板。',flush=True)
                return 0
            raise
        handler=RotatingFileHandler(log_dir/'service.log',maxBytes=2_000_000,backupCount=5,encoding='utf-8')
        logging.basicConfig(level=logging.INFO,handlers=[handler],format='%(asctime)s %(levelname)s %(name)s %(message)s')
        service=MotorService(config['adapter'],log_dir,config['heartbeat_timeout_s'],aps_options=config['aps'])
        # Publish the startup scan before the first client can enable or rescan.
        # Bind first so a duplicate backend cannot start accessing the card.
        with service.lock:
            server=UDPServer(service,config['host'],config['port'],config['auth_key']).start(reserved_socket)
            service.scan()
        if args.headless:
            print(f"SV635N UDP 后台已启动：{config['host']}:{server.port}，正在自动扫描 {config['adapter']}，电机未使能。",flush=True)
            while not quit_event.wait(.2):
                if server.closed.is_set():
                    break
        else:
            run_tray(service,server,config,quit_event,args.config.resolve())
    except Exception as exc:
        logging.exception('Service startup/runtime failure')
        if args.headless:
            print(f'后台失败：{exc}',flush=True)
        else:
            import ctypes
            ctypes.windll.user32.MessageBoxW(None,f'{exc}\n\n详细记录：{log_dir / "service.log"}',
                                            'SV635N UDP 后台无法运行',0x10)
        return 1
    finally:
        try:
            if server:
                service.shutdown.set()
                service.stop('后台退出')
                server.notify_shutdown()
            if service:
                service.close()
        finally:
            if server:
                server.close()
            if reserved_socket:
                reserved_socket.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
