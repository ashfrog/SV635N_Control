# 独立 UDP 电机控制界面

`UdpControl` 是独立的有界面控制程序。它只通过 UDP 与 `SV635N_Control` 后台通信，不导入 `core.py`、`motor_service.py`、`udp_server.py`，不访问 EtherCAT，也不需要 Npcap、pysoem 或托盘依赖。

## 启动

1. 在后台电脑的 `SV635N_Control` 根目录启动 `启动后台.cmd`。后台的安装、托盘、PCIe-8332 控制卡和心跳配置见 [后台说明](../后台说明.md)。
2. 在 `UdpControl` 中双击 `启动界面.cmd`，或者在父目录执行 `python -m UdpControl`。也可从后台右下角托盘选择“打开控制界面”，后台只启动独立客户端进程，所有界面操作依然走 UDP。
3. 默认连接 `127.0.0.1:5005`。复制 `config.example.json` 为 `config.json` 可修改后台地址、端口和 auth_key；跨电脑时 host 填写后台电脑地址，也可运行 `python -m UdpControl --host 192.168.1.10 --port 5005`。配置中只需要 UDP 连接信息，PCIe-8332 控制卡属于后台配置。
4. 点击“获取调试控制权”，刷新控制卡、扫描电机，在列表中勾选需要转动的电机，设置速度/加减速度，确认运行条件，再开启连续使能。
5. 拖动滑块同时更新所有勾选电机的目标偏移，无需再选一台电机。目标为各轴本次使能起点的绝对偏移，显示范围不是累计限位。
6. “关闭使能 / Esc”经 UDP 优先发送停止请求，不等待普通目标请求结束。等待后台停止并恢复参数后，可释放控制权给其它程序。
7. 关闭窗口会停止本界面开启的运行并关闭客户端，后台继续运行；只读监视不会停止其它程序的运行。客户端进程异常退出时，后台依靠心跳超时停止该运行。

客户端没有开启或关闭后台进程的接口；后台未启动时界面显示连接失败，不会尝试直连电机。只需标准 Python 3.11～3.13（包含 Tkinter），可将整个 `UdpControl` 文件夹复制到另一台电脑，直接运行。启动脚本优先使用父目录的 `.venv`，否则使用系统 `pyw -3`。

## 文件

| 文件 | 用途 |
| --- | --- |
| `__main__.py` | 独立 GUI 启动入口 |
| `debug_ui.py` | 电机选择、使能、目标滑块、UDP 状态显示 |
| `client.py` | 标准库 UDP SDK、确认重试、反馈接收、独立心跳 |
| `wire.py` | 客户端 JSON 协议编解码 |
| `config.example.json` | 后台地址配置示例 |
| `example_client.py` | 只读查询和显式运动的 SDK 示例 |
| `PROTOCOL.md` | C++ / UE / C# 等调用方的 UDP 协议 |

## Python SDK

```python
from UdpControl.client import MotorClient

with MotorClient('127.0.0.1', 5005) as motor:
    motor.hello()
    motor.request('scan')
    motor.wait_for(('idle',))
    motor.enable([1, 2, 3], rpm=80, acceleration_rpm_s=120)
    motor.wait_for(('enabled',))
    motor.target([5, 0, -5])  # 实际运动：相对使能起点的各轴绝对目标
    motor.disable()
    motor.wait_for(('idle', 'fault'))
    motor.release()
```

SDK 自动发送独立心跳，目标停更但程序仍存活时继续保持使能；调用方必须在业务结束时调用 disable/close。后台默认心跳超时 0.5 秒。重复和乱序请求不续期，旧 run_id 无法控制新运行；更多说明见 [协议文档](PROTOCOL.md)。

仅查询后台状态（默认不运动）：

```powershell
python -m UdpControl.example_client
```

明确执行三轴运动示例：

```powershell
python -m UdpControl.example_client --move --orders 1 2 3 --rpm 80 --acceleration 120
```

界面错误记录在 `UdpControl/logs/client.log`。运动日志和硬件异常记录在后台电脑根目录的 `logs`，客户端无需访问这些文件。
