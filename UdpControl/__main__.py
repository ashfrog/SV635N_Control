"""Independent GUI process; imports only the standard library and UDP client."""
import argparse
import gc
import json
import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path
import tkinter as tk

from .debug_ui import DebugWindow
from .instance import PanelInstance

BASE = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent


def load_config(path):
    config = dict(host='127.0.0.1', port=5005, auth_key='')
    if path.exists():
        supplied = json.loads(path.read_text(encoding='utf-8-sig'))
        if not isinstance(supplied, dict):
            raise ValueError('配置必须为 JSON 对象。')
        # Also accept a backend configuration passed by its tray launcher;
        # only endpoint/auth fields are consumed by this UDP-only client.
        config.update({k: v for k, v in supplied.items() if k in config})
    if not isinstance(config['host'], str) or not config['host'] or type(config['port']) is not int or not 1 <= config['port'] <= 65535:
        raise ValueError('后台 host/port 配置无效。')
    if not isinstance(config['auth_key'], str):
        raise ValueError('auth_key 必须为字符串。')
    return config


def main():
    parser = argparse.ArgumentParser(description='独立 UDP 电机控制界面')
    parser.add_argument('--config', type=Path, default=BASE/('backend.config.json' if getattr(sys, 'frozen', False) else 'config.json'))
    parser.add_argument('--host', help='后台 UDP 地址，覆盖配置')
    parser.add_argument('--port', type=int, help='后台 UDP 端口，覆盖配置')
    args = parser.parse_args()
    log_dir = BASE/'logs'
    log_dir.mkdir(exist_ok=True)
    root = window = instance = None
    try:
        config = load_config(args.config)
        if args.host is not None:
            config['host'] = args.host
        if args.port is not None:
            if not 1 <= args.port <= 65535:
                raise ValueError('port 须为 1～65535。')
            config['port'] = args.port
        instance = PanelInstance(config['host'], config['port'])
        if not instance.primary:
            instance.activate()
            return 0
        handler = RotatingFileHandler(log_dir/'client.log', maxBytes=1_000_000, backupCount=3, encoding='utf-8')
        logging.basicConfig(level=logging.INFO, handlers=[handler], format='%(asctime)s %(levelname)s %(message)s')
        root = tk.Tk()
        window = DebugWindow(root, config['host'], config['port'], config['auth_key'])
        def check_activation():
            if instance.requested():
                window.show()
                root.attributes('-topmost', True)
                root.after(100, lambda: root.attributes('-topmost', False))
                root.focus_force()
            root.after(100, check_activation)
        root.after(100, check_activation)
        root.mainloop()
    except Exception as exc:
        logging.exception('UDP GUI failure')
        import ctypes
        ctypes.windll.user32.MessageBoxW(None, f'{exc}\n\n记录：{log_dir / "client.log"}', 'UDP 控制界面', 0x10)
        return 1
    finally:
        if window:
            window.close()
        if root:
            root.destroy()
        del window, root
        gc.collect()
        if instance:
            instance.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
