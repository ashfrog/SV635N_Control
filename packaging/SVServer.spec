from pathlib import Path

project = Path(SPECPATH).parent
analysis = Analysis(
    [str(project / 'packaging' / 'server_entry.py')],
    pathex=[str(project)],
    binaries=[],
    datas=[],
    hiddenimports=['pystray._win32'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['pysoem', 'core', 'app', 'cli', 'continuous_control', 'platform_control',
              'unittest', 'pytest', 'test_aps_backend', 'test_platform_control'],
    noarchive=False,
)
archive = PYZ(analysis.pure)
executable = EXE(
    archive, analysis.scripts, [],
    exclude_binaries=True,
    name='SVServer',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    icon=str(project / '.build' / 'svserver' / 'SVServer.ico'),
)
distribution = COLLECT(
    executable, analysis.binaries, analysis.datas,
    strip=False, upx=False, name='SVServer',
)
