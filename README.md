# SV635N_Control

SV635N / ADLINK PCIe-8332 电机后台（Python 源码版），通过 UDP 接收单轴/多轴连续运动目标，提供托盘入口和独立控制客户端。

便携 EXE 发布目录为 [SVServer](SVServer/README.md)：双击 `SVServer/SVServer.exe` 或 `SVServer/启动服务.cmd` 启动，复制整个目录即可部署，无需安装 Python。控制卡驱动和 APS SDK 仍需安装。构建方法：安装 `requirements-build.txt` 后运行 `python build_svserver.py`；托盘和客户端各自运行在独立进程中。

双击根目录的 `启动源码.cmd` 或 `启动后台.cmd` 启动托盘后台，控制界面由 `UdpControl/启动界面.cmd` 或托盘“打开控制界面”启动。运行环境使用本项目的 `.venv`，不依赖临时工作目录。

首次使用或更换电脑时，安装 Python 3.11～3.13 x64 、PCIe-8332 驱动及 APS-SDK-2.3.00.250902，然后在项目根目录执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

示例使用 Python 3.12；使用其他受支持版本时，请相应修改版本号。

文件：`backend.py` 为后台入口，`motor_service.py` / `udp_server.py` 为服务与协议，`aps_backend.py` 为 PCIe-8332 控制核心，`control_common.py` 为共享类型/目标槽，`UdpControl` 为独立 UDP 界面与 SDK。旧 `app.py` 和 `cli.py` 保留为普通网卡直连工具，需要安装 `requirements-legacy.txt`，不能通过 PCIe 卡控制。接线与旧调试操作见 [使用说明.md](使用说明.md)。

常驻 UDP 后台在根目录实现，见 [后台说明](后台说明.md)：双击 `安装后台依赖.cmd` 后运行根目录 `启动后台.cmd`，或执行 `python backend.py --headless`。独立控制界面在 [UdpControl](UdpControl/README.md)，使用 `UdpControl/启动界面.cmd` 或 `python -m UdpControl`，也可由后台右下角托盘打开。界面只通过 UDP 控制后台，可复制到其它电脑使用，无需电机驱动依赖。Python SDK 为 `UdpControl.client.MotorClient`。

电机调试模式：点击“获取调试控制权”后自动扫描、选中并使能全部扫描到的电机，保持当前位置和连续使能，无需额外使能按钮；停止或处理故障后，点击“重新获取调试控制”重新扫描并使能全部电机。速度与加减速度只接受正的有限数值；默认值为 60 rpm / 120 rpm/s。使能期间可更新速度、加减速度及每轴目标；新目标覆盖未下发的旧目标。轨迹和 EtherCAT 周期由控制卡执行，Python 通过 APS 提交绝对位置覆盖目标。实际操作见 [客户端说明](UdpControl/README.md)。

三竖向撑杆平台：`platform_motion.py` 将升降毫米、俯仰/横滚角换算为三轴目标，提供标定/行程校验、共同进度插值、速度与加速度约束、独立姿态超时、跟随误差与全轴限位停止。UE 组件已改为后台 UDP v1，见 [UE 接入与实机边界](UE接入.md)。真实平台默认未启用；`backend.platform.example.json` 为虚拟机构示例，必须替换成实测标定。配置平台后默认禁止直接电机调试，防止绕过平台约束。可先运行 `python simulated_backend.py`（仅模拟，端口 5006），通过 UE 或 Python SDK 验证完整流程。

后台默认使用 ADLINK `PCIe-8332:0`，启动后自行初始化现场总线并扫描电机，日常无需先启动 MCPro2；界面自动显示控制卡与扫描结果。冷启动会补建 APS 会话轴映射，暂时性失败最多尝试 3 次，缺失或过期 ENI 时自动扫描生成一次。首次使用需安装正确的 SV635N ESI，并关闭 MCPro2。具体板卡配置和位置单位见 [后台说明](后台说明.md)。`continuous_control.py` 保留旧普通网卡运动 API；`platform_control.py` / `ue_demo.py` 保留旧三轴协议，与当前 UE 组件不兼容。完整无硬件回归（需安装 `requirements-legacy.txt`）：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -v
```

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
