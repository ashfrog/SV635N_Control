# SV635N_Control

SV635N / ADLINK PCIe-8332 电机后台（Python 源码版），通过 UDP 接收单轴/多轴连续运动目标，提供托盘入口和独立控制客户端。

双击根目录的 `启动源码.cmd` 或 `启动后台.cmd` 启动托盘后台，控制界面由 `UdpControl/启动界面.cmd` 或托盘“打开控制界面”启动。运行环境使用本项目的 `.venv`，不依赖临时工作目录。

首次使用或更换电脑时，安装 Python 3.11～3.13 x64 、PCIe-8332 驱动及 APS-SDK-2.3.00.250902，然后在项目根目录执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

示例使用 Python 3.12；使用其他受支持版本时，请相应修改版本号。

文件：`backend.py` 为后台入口，`motor_service.py` / `udp_server.py` 为服务与协议，`aps_backend.py` 为 PCIe-8332 控制核心，`control_common.py` 为共享类型/目标槽，`UdpControl` 为独立 UDP 界面与 SDK。旧 `app.py` 和 `cli.py` 保留为普通网卡直连工具，需要安装 `requirements-legacy.txt`，不能通过 PCIe 卡控制。接线与旧调试操作见 [使用说明.md](使用说明.md)。

常驻 UDP 后台在根目录实现，见 [后台说明](后台说明.md)：双击 `安装后台依赖.cmd` 后运行根目录 `启动后台.cmd`，或执行 `python backend.py --headless`。独立控制界面在 [UdpControl](UdpControl/README.md)，使用 `UdpControl/启动界面.cmd` 或 `python -m UdpControl`，也可由后台右下角托盘打开。界面只通过 UDP 控制后台，可复制到其它电脑使用，无需电机驱动依赖。Python SDK 为 `UdpControl.client.MotorClient`。

连续控制：扫描并选中电机，确认运行条件后开启连续使能。速度与加减速度无软件上下限，只接受正的有限数值；默认值仍为 60 rpm / 120 rpm/s。拖动滑块直接更新列表中所有勾选电机的共同目标偏移。新指令不等待上一目标到位，累计角度不设上限。轨迹和 EtherCAT 周期由控制卡执行，Python 通过 APS 提交绝对位置覆盖目标。UDP 心跳超时和关闭使能都会停止并恢复参数；实际操作见 [客户端说明](UdpControl/README.md)。

后台默认使用 `PCIe-8332:0`，首次使用先在 MCPro2 配好 ESI/ENI 并关闭 MCPro2；具体板卡配置和位置单位见 [后台说明](后台说明.md)。`continuous_control.py` 保留旧普通网卡运动 API。此前 `platform_control.py` 与 `UE/` 组件保留为旧协议参考，与新后台 v1 协议不兼容。完整无硬件回归（需安装 `requirements-legacy.txt`）：

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_aps_backend test_udp_layout test_udp_protocol test_udp_service test_udp_ui test_continuous_control test_app_continuous test_platform_control
```

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
