"""Build a portable Windows x64 backend; preserve deployed config and logs."""
from pathlib import Path
import shutil
import subprocess
import sys

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent


def main():
    if sys.platform != 'win32':
        raise SystemExit('Build SVServer on Windows using Python x64.')
    if sys.maxsize <= 2**32:
        raise SystemExit('Python x64 is required for APS168x64.dll.')
    work = ROOT / '.build' / 'svserver'
    work.mkdir(parents=True, exist_ok=True)
    icon = Image.new('RGBA', (256, 256), '#163a63')
    draw = ImageDraw.Draw(icon)
    draw.ellipse((40, 40, 216, 216), outline='white', width=20)
    draw.line((128, 80, 128, 176), fill='#52c5ff', width=24)
    icon.save(work / 'SVServer.ico', sizes=[(16,16), (32,32), (48,48), (64,64), (128,128), (256,256)])
    subprocess.run([sys.executable, '-m', 'PyInstaller', '--noconfirm',
        '--distpath', str(work / 'dist'), '--workpath', str(work / 'work'),
        str(ROOT / 'packaging' / 'SVServer.spec')], cwd=ROOT, check=True)
    target = ROOT / 'SVServer'
    shutil.copytree(work / 'dist' / 'SVServer', target, dirs_exist_ok=True)
    for name in ('backend.config.example.json', 'backend.platform.example.json'):
        shutil.copy2(ROOT / name, target / name)
    config = target / 'backend.config.json'
    if not config.exists():
        source = ROOT / 'backend.config.json'
        shutil.copy2(source if source.exists() else ROOT / 'backend.config.example.json', config)
    print(f'Ready: {target / "SVServer.exe"}')
    print('Copy the entire SVServer folder. Existing deployment config/logs were preserved.')


if __name__ == '__main__':
    main()
