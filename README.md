# SV635N_Control

SV635N 主板网口直连 EtherCAT 电机后台（Python 源码版），通过 UDP 接收单轴/多轴连续运动目标，提供托盘入口和独立控制客户端。

便携 EXE 发布目录为 [SVServer](SVServer/README.md)：双击 `SVServer/SVServer.exe` 或 `SVServer/启动服务.cmd` 启动，复制整个目录即可部署，无需安装 Python。本机需安装 Npcap（启用 WinPcap API 兼容模式）；默认使用主板 Realtek Gaming 2.5GbE 网口。构建方法：安装 `requirements-build.txt` 后运行 `python build_svserver.py`；托盘和客户端各自运行在独立进程中。

双击根目录的 `启动源码.cmd` 或 `启动后台.cmd` 启动托盘后台，控制界面由 `UdpControl/启动界面.cmd` 或托盘“打开控制界面”启动。运行环境使用本项目的 `.venv`，不依赖临时工作目录。

首次使用或更换电脑时，安装 Python 3.11～3.13 x64 和 Npcap（启用 WinPcap API 兼容模式），然后在项目根目录执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

示例使用 Python 3.12；使用其他受支持版本时，请相应修改版本号。

文件：`backend.py` 为后台入口，`motor_service.py` / `udp_server.py` 为服务与协议，`nic_backend.py` / `core.py` / `continuous_control.py` 为网口直连控制核心，`aps_backend.py` 保留可选 PCIe-8332 后端，`control_common.py` 为共享类型/目标槽，`UdpControl` 为独立 UDP 界面与 SDK。`app.py` 和 `cli.py` 保留独立直连调试入口，依赖已包含在 `requirements.txt`。接线与旧调试操作见 [使用说明.md](使用说明.md)。

常驻 UDP 后台在根目录实现，见 [后台说明](后台说明.md)：双击 `安装后台依赖.cmd` 后运行根目录 `启动后台.cmd`，或执行 `python backend.py --headless`。独立控制界面在 [UdpControl](UdpControl/README.md)，使用 `UdpControl/启动界面.cmd` 或 `python -m UdpControl`，也可由后台右下角托盘打开。界面只通过 UDP 控制后台，可复制到其它电脑使用，无需电机驱动依赖。Python SDK 为 `UdpControl.client.MotorClient`。

电机调试模式：点击“获取调试控制权”后自动扫描、选中并使能全部扫描到的电机，保持当前位置和连续使能，无需额外使能按钮；停止或处理故障后，点击“重新获取调试控制”重新扫描并使能全部电机。速度与加减速度只接受正的有限数值；通用默认值为 60 rpm / 120 rpm/s，网口直连未标定时界面改用 5 rpm，后台拒绝超过 5 rpm 的请求。使能期间可更新速度、加减速度及每轴目标；新目标覆盖未下发的旧目标。默认由 pysoem 通过主板网口交换 PDO，驱动器 PP 模式执行轨迹；新绝对目标覆盖旧目标。实际操作见 [客户端说明](UdpControl/README.md)。

网口直连的普通警告和内部限幅不再单独中断控制；`0x5443/0x5444` 超程继续限制相应方向。普通角度调试到达限位后暂停并保持使能，直接提交反向目标即可离开，无需重新获取控制；旧目标丢弃，平台高度模式仍全轴停止。残留 `0x0E08` 通信故障可在点击“重新获取调试控制”后、总线已恢复且全部轴未使能时尝试一次复位；有效故障、急停或持续断联仍会停止。

网口直连未标定固定中位和实测软行程时，后台强制最高 **5 rpm**（界面与 UDP 都受限）。不得用默认 ±50 mm 或未知停车余量解锁高速。已标定模式使用机构速度上限、位置软边界及 `v²/(2a)+v·T+余量` 提前停车。硬限位首次触发的编码器位置保留为方向边界，穿过感应片后信号消失也不解除危险方向；反向退回边界内才能解除锁存，记录的边界仍阻止再越过。停车后位置记忆保存到 `logs/nic-limit-memory.json`，正常重启保留；更换机构、编码器清零或坐标变化必须重新核对标定。限位片应在整个超程区持续有效，并留出实测最坏停车距离；程序保护不能补偿传感器之后没有制动空间。

三竖向撑杆平台：UE 通过 UDP v1 提交 `[前左1号, 前右2号, 后中3号]` 三个相对标定初始位的高度（mm），0 回初始位，正数上推，负数下拉。`platform_motion.py` 提供毫米/电机角度转换、行程与平台姿态范围校验、共同进度插值、速度与加速度约束、独立输入超时、跟随误差与全轴限位停止，兼容旧姿态输入。调试界面的“三点高度 · UE 模拟”页可分别或共同调节三轴，并持续发送约 30 Hz 高度流。UE 接口为 `SetPlatformHeights`，见 [UE 接入与实机边界](UE接入.md)。真实平台默认未启用；`backend.platform.example.json` 采用已确认的 670/730/730 mm 安装结构及每转 5 mm，但方向和行程仍为模拟假设，必须完成实测标定。配置平台后默认禁止直接电机调试，防止绕过平台约束。可先运行 `python simulated_backend.py`（仅模拟，端口 5006），通过界面、UE 或 Python SDK 验证完整流程。

后台默认 `backend: "ethercat"`，通过本机 Realtek Gaming 2.5GbE 网口直连 SV635N，启动只扫描、不使能。接线：主板网口 → 第一台 CN3 IN，CN4 OUT → 下一台 CN3 IN，不经普通交换机。已配置网口为 `\Device\NPF_{89B39964-A086-4A5D-BC55-D5804AE703C6}`；换电脑时在面板刷新网口后选择，或修改 `adapter`。不需要 APS SDK、MCPro2 或 ENI。原 PCIe 后端需显式设置 `backend: "aps"`、`adapter: "PCIe-8332:0"` 及对应 `aps` 配置。`platform_control.py` / `ue_demo.py` 保留旧三轴协议，与当前 UE 组件不兼容。完整无硬件回归：

```powershell
.\.venv\Scripts\python.exe -m unittest discover -v
```

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
