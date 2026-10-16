"""Explicit, administrator-only tuning of a stopped, dedicated EtherCAT NIC."""
import base64
import ctypes
from datetime import datetime
import json
from pathlib import Path
import subprocess
import time
import uuid

from control_common import ControlError, adapter_lock
from core import EtherCATController
from udp_server import bind_socket


def administrator():
    return bool(ctypes.windll.shell32.IsUserAnAdmin())


def inspect_disabled(adapter):
    controller = EtherCATController(adapter)
    master = controller.master_factory()
    try:
        master.open(adapter)
        devices = controller._discover(master)  # Caller holds the hardware mutex.
        return [dict(order=d.order, identity=list(d.identity), statusword=d.statusword,
                     error_code=d.error_code) for d in devices]
    finally:
        master.close()


def optimize_nic(config, log_dir):
    if config['backend'] != 'ethercat':
        raise ControlError('只有主板网口直连模式支持此操作。')
    if not administrator():
        raise ControlError('请运行“优化电机网口（管理员）.cmd”，并允许 Windows 管理员提示。')
    adapter = config['adapter']
    if not adapter.startswith('\\Device\\NPF_'):
        raise ControlError('配置中不是有效的 Npcap 网口。')
    guid = str(uuid.UUID(adapter[len('\\Device\\NPF_'):].strip('{}')))
    # Reserve the service endpoint for the ENTIRE operation. A new backend cannot
    # start scanning/enabling while this helper temporarily restarts the NIC.
    try:
        reservation = bind_socket(config['host'], config['port'])
    except OSError as exc:
        raise ControlError('请先在托盘选择“停止并退出后台”，再运行网口优化；不能只关闭控制界面。') from exc
    try:
        with adapter_lock(adapter):
            before = inspect_disabled(adapter)  # Refuses any enabled or unsupported drive.
            folder = Path(log_dir)
            folder.mkdir(parents=True, exist_ok=True)
            tag = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            backup = folder/f'nic-settings-{tag}.json'
            journal = folder/f'nic-setup-{tag}.json'
            result = dict(adapter=adapter, before=before, backup=str(backup), success=False)
            # Both values are validated/literal-quoted, never shell-expanded input.
            backup_literal = str(backup.resolve()).replace("'", "''")
            script = """$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
$motorNic = Get-NetAdapter | Where-Object { [guid]$_.InterfaceGuid -eq [guid]'%GUID%' }
if (-not $motorNic) { throw 'Configured motor NIC was not found' }
$keys = @('*InterruptModeration','EnableGreenEthernet','GigaLite','PowerSavingMode','*SelectiveSuspend')
$original = @(Get-NetAdapterAdvancedProperty -Name $motorNic.Name | Where-Object { $_.RegistryKeyword -in $keys })
if (-not $original.Count) { throw 'NIC has none of the supported tuning properties' }
$original | Select-Object Name,RegistryKeyword,RegistryValue,DisplayName,DisplayValue |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath '%BACKUP%' -Encoding UTF8
try {
    foreach ($item in $original) {
        Set-NetAdapterAdvancedProperty -Name $motorNic.Name -RegistryKeyword $item.RegistryKeyword -RegistryValue '0' -NoRestart
    }
    Restart-NetAdapter -Name $motorNic.Name -Confirm:$false
    $after = @(Get-NetAdapterAdvancedProperty -Name $motorNic.Name | Where-Object { $_.RegistryKeyword -in $keys })
    if ($after.Count -ne $original.Count -or @($after | Where-Object { ($_.RegistryValue -join ',') -ne '0' }).Count) {
        throw 'NIC settings verification failed'
    }
    $after | Select-Object RegistryKeyword,RegistryValue,DisplayValue | ConvertTo-Json -Depth 5
} catch {
    $originalError = $_
    foreach ($item in $original) {
        Set-NetAdapterAdvancedProperty -Name $motorNic.Name -RegistryKeyword $item.RegistryKeyword -RegistryValue $item.RegistryValue -NoRestart
    }
    Restart-NetAdapter -Name $motorNic.Name -Confirm:$false
    throw $originalError
}
""".replace('%GUID%', guid).replace('%BACKUP%', backup_literal)
            try:
                encoded = base64.b64encode(script.encode('utf-16-le')).decode('ascii')
                process = subprocess.run(['powershell.exe','-NoProfile','-NonInteractive','-EncodedCommand',encoded],
                    capture_output=True, text=True, encoding='utf-8', errors='replace',
                    creationflags=subprocess.CREATE_NO_WINDOW, timeout=60)
                result.update(output=process.stdout, stderr=process.stderr)
                if process.returncode:
                    raise ControlError(f'网口优化失败；原设置备份：{backup}。{process.stderr}')
                # Link renegotiation can outlast Restart-NetAdapter returning.
                # Retry only a missing-link discovery, never an enabled drive
                # or an identity/parameter error.
                for attempt in range(10):
                    try:
                        after = inspect_disabled(adapter)
                        break
                    except ControlError as exc:
                        if attempt == 9 or not any(f'扫描到 {n} 台设备' in str(exc) for n in (-1,0)):
                            raise
                        time.sleep(.5)
                if [d['identity'] for d in before] != [d['identity'] for d in after]:
                    raise ControlError('网口优化后驱动器身份发生变化，请检查接线；未使能电机。')
                result.update(after=after, success=True)
                return backup
            except Exception as exc:
                result['error'] = str(exc)
                raise
            finally:
                journal.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    finally:
        reservation.close()
