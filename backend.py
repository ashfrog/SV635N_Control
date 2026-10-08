"""Windows tray process; --headless provides the same service without a UI."""
import argparse
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import signal
import subprocess
import sys
import threading

from core import DEFAULT_ADAPTER
from udp_server import UDPServer
from motor_service import MotorService

BASE=Path(__file__).resolve().parent


def load_config(path):
    config=dict(host='127.0.0.1',port=5005,adapter=DEFAULT_ADAPTER,heartbeat_timeout_s=.5,auth_key='')
    if path.exists():
        supplied=json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(supplied,dict) or set(supplied)-set(config):
            raise ValueError('配置包含未知字段，或不是 JSON 对象。')
        config.update(supplied)
    if not isinstance(config['host'],str) or type(config['port']) is not int or not 1<=config['port']<=65535:
        raise ValueError('host/port 配置无效。')
    if not isinstance(config['adapter'],str) or not config['adapter'] or not isinstance(config['auth_key'],str):
        raise ValueError('adapter/auth_key 配置无效。')
    return config


def run_tray(service,server,config,quit_event,config_path=None):
    # The backend owns no Tk objects. Its icon launches a separate UDP client.
    import pystray
    from PIL import Image,ImageDraw
    host='127.0.0.1' if config['host'] in ('0.0.0.0','localhost') else config['host']
    client_process=None
    launch_lock=threading.Lock()
    def open_client(*_):
        nonlocal client_process
        with launch_lock:
            if client_process is not None and client_process.poll() is None:
                return
            executable=Path(sys.executable)
            windowed=executable.with_name('pythonw.exe')
            if windowed.exists():
                executable=windowed
            try:
                # The frontend receives only the endpoint and config file path;
                # it has no service object or hardware reference.
                client_process=subprocess.Popen([str(executable),'-m','UdpControl',
                    '--host',host,'--port',str(server.port),'--config',str(config_path or BASE/'backend.config.json')],
                    cwd=BASE,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
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
        service.stop('后台退出')
        icon.stop()


def main():
    parser=argparse.ArgumentParser(description='SV635N UDP 后台电机控制服务')
    parser.add_argument('--headless',action='store_true',help='不显示托盘，用于控制台/服务宿主')
    parser.add_argument('--config',type=Path,default=BASE/'backend.config.json')
    args=parser.parse_args()
    log_dir=BASE/'logs'
    log_dir.mkdir(exist_ok=True)
    handler=RotatingFileHandler(log_dir/'service.log',maxBytes=2_000_000,backupCount=5,encoding='utf-8')
    logging.basicConfig(level=logging.INFO,handlers=[handler],format='%(asctime)s %(levelname)s %(name)s %(message)s')
    service=server=None
    quit_event=threading.Event()
    signal.signal(signal.SIGINT,lambda *_:quit_event.set())
    signal.signal(signal.SIGTERM,lambda *_:quit_event.set())
    try:
        config=load_config(args.config)
        service=MotorService(config['adapter'],log_dir,config['heartbeat_timeout_s'])
        server=UDPServer(service,config['host'],config['port'],config['auth_key']).start()
        if args.headless:
            print(f"SV635N UDP 后台已启动：{config['host']}:{server.port}，电机未使能。",flush=True)
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
        if server:
            server.close()
        if service:
            service.close()
    return 0


if __name__=='__main__':
    raise SystemExit(main())
