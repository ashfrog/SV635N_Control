# UE 三撑杆平台接入（后台 UDP v1）

`UE/SV635NMotionComponent` 直接连接 `backend.py`，发送三个撑杆相对标定初始位的目标高度，单位毫米，顺序固定为 `[前左1号, 前右2号, 后中3号]`。当前蓝图接口为 `Set Platform Heights`，原 `Set Platform Pose` 接线需要迁移。旧 `platform_control.py` / `ue_demo.py` 使用另一套三轴电机角度协议，不能与本组件混用，也不要同时占用同一控制卡或 UDP 端口。

`[0,0,0]` 表示三杆回到初始位并保持使能；正数向上推，负数向下拉。例如 `[5,-3,0]` 表示前左高于初始位 5 mm、前右低于初始位 3 mm、后中回到初始位。数据是各轴绝对高度偏移，相同数值重复发送不会累加位移。初始位可标定在推杆行程中部，但程序不会根据“大概中心”自动推断位置或行程。UE 世界距离通常以厘米表示，传入接口前需乘 10。

## 架构与坐标

UE → UDP v1 → MotorService → PlatformGeometry / PlatformMotionQueue → pysoem → 主板网口 → 三台 SV635N。可选 `backend: "aps"` 时改由 APS SDK / PCIe-8332 执行。

高度直接换算为电机目标：`motor_deg_i = height_mm_i / mm_per_rev_i * 360 * extension_sign_i`。每帧必须提交全部三轴高度；所有撑杆行程及平台整体升降/倾斜范围通过后才更新目标，任一越界均拒绝整帧并停止当前平台运行。

固定平台坐标：X 向前、Y 向右、Z 向上；后台由三个高度恢复支撑平面，用于校验 `max_heave_mm/max_pitch_deg/max_roll_deg`。正俯仰抬高前端，正横滚抬高右端。后台保留旧 UDP `pose` 请求兼容既有调用方，其公式如下；当前 UE 组件和高度调试页发送 `heights`。

对中位平面上水平位置 `(x_i, y_i)` 的撑杆，模型使用平面与竖直支撑线的交点：

```
delta_mm_i = heave_mm + x_i * tan(pitch) + y_i * tan(roll) / cos(pitch)
motor_deg_i = delta_mm_i / mm_per_rev_i * 360 * extension_sign_i
```

角度在公式中换成弧度；平面法向量由 `Ry(-pitch) Rx(-roll)` 定义。此模型只适用于可通过关节/滑动机构容许倾斜的竖直升降支撑；若实际是固定长度连杆、曲柄或倾斜推杆，需要按真实机构替换逆解，不能直接套用。软件模型不验证结构自由度、载荷分布或侧向受力。

## 先运行无硬件模拟

```powershell
.\.venv\Scripts\python.exe simulated_backend.py
```

模拟后台监听 `127.0.0.1:5006`，不加载 APS DLL，不访问电机。UE 设置 Port=5006、CalibrationId=`simulation-only-v1`、Orders=[1,2,3]。模拟器从 `backend.platform.example.json` 读取模拟配置，在内存中启用平台模式；支点间距、参考原点、电机位置和每转 5 mm 采用下文已确认的数据，伸出方向与行程仍为模拟假设，反馈 `hardware_backend=simulated`。它验证协议、姿态换算、平滑目标和状态流程，不模拟真实制动器、驱动报警、EtherCAT 时序或载荷。

首次模拟联调也须先在调试高度页点击“保存当前位置为固定初始位”，等待保存完成；模拟文件与实机文件隔离，重启后保留。随后勾选中位确认，获取平台控制或释放控制权给 UE。

## UE C++ / 蓝图

将 `UE/SV635NMotionComponent.h` 和 `.cpp` 复制到目标 UE5 C++ 工程的 `Source/你的模块/`，在 `.Build.cs` 增加：

```csharp
PrivateDependencyModuleNames.AddRange(new string[] { "Sockets", "Networking", "Json" });
```

若需要跨模块访问组件，为类声明增加目标模块的 `YOURMODULE_API` 宏。本仓库没有 `.uproject`，组件必须在目标 UE 工程编译与打包验证。

在 Actor 添加组件，并按顺序接线：

1. 每次游戏更新调用 `Set Platform Heights(FrontLeftMillimetres, FrontRightMillimetres, RearMillimetres)`，静止时也持续调用。首次运行先设为 `(0,0,0)`。
2. 等 `Feedback Valid=true`、后台空闲、标定匹配，且已通过调试界面保存并返回固定中位。操作员确认三根撑杆确实处于标定中位，再显式调用 `Arm Platform(true)`。返回 true 仅表示开始异步申请；实际使能看反馈。
3. 默认以 30 Hz 发送当前三个高度。使能准备阶段发送心跳，后台使能就绪后发送高度。`HeightInputTimeout` 默认 0.2 秒，约束蓝图/游戏输入的新鲜度。
4. 停止按钮调用 `Stop Platform`。结束 Play 也提交停止；暂停、游戏卡顿、高度断流、反馈超时、故障或后台重启会撤销运动意图。
5. 故障或停止后不会自动重启。处理问题并重新确认物理中位后才能再次 Arm；后台 fault 状态先通过客户端/SDK显式重新扫描。

组件固定使用同一 UDP socket，控制请求保留相同 id/内容重试，高度请求只发送最新值；使用 server_id/state_serial 拒绝旧实例和乱序状态。停止过程中到达的迟到使能 ACK 会再请求关闭使能。游戏结束时不等待网络确认，后台心跳和平台输入超时负责补充停止。

高度数据包示例（先获取 session、使能平台并等待 enabled）：

```json
{"v":1,"id":"height1","type":"heights","session":"...","run_id":"...","seq":3,"heights_mm":[5,-3,0]}
```

无 UE 时可在调试界面的“三点高度 · UE 模拟”页联调：连接模拟端口 5006，首次先保存固定初始位，勾选初始位确认，点击“获取调试控制权”。三个独立滑块/数字输入只修改对应轴，共同高度同时设定三轴；数字停输 350 ms、Enter 或失焦提交，拖动立即提交。“三轴回到初始位”设定 `[0,0,0]`。该页持续发送完整高度帧，Esc/关闭界面/反馈失联会撤销发送并停止；停止后必须再次确认初始位，未提交的编辑不会带入新运行。

`Hardware Enabled` 和 `Actual Motor Degrees` 是最后观测值，失联时不能认定电机已停止。核对 `Feedback Valid`、后台 `result.all_disabled` 和 `cleanup_errors`；SDK ACK 表示接受请求，不表示运动到位。

## 真实机构配置与标定

### 670/730/730 mm 安装结构

根据提供的俯视图和安装确认，三根竖直撑杆的轴线中心距为：前左与前右 670 mm，两者到后中各 730 mm，因此按等腰三角形建模。前方底边中点到后中支点的距离为 `sqrt(730² - 335²) = 648.594635 mm`。

已确认平台升降与姿态采用三支点几何重心为原点，X 向前、Y 向右；该原点不代表实测质心。电机编号为前左 1、前右 2、后中 3，配置按对应链路位置 `order=1/2/3` 填写。支点坐标如下：

| 支点 | X / mm | Y / mm | 电机编号 / order |
|---|---:|---:|---:|
| 前左 | 216.198212 | -335 | 1 |
| 前右 | 216.198212 | 335 | 2 |
| 后中 | -432.396423 | 0 | 3 |

`order` 表示链路位置，不是 APS axis_id；实机扫描应核对链路位置与上述安装编号一致。`heave_mm` 表示重心参考点相对中位平面的高度变化。若以后将 UE 控制参考点改为相对重心的 `(cx, cy)`，应把每个支点的坐标改为 `(x_i - cx, y_i - cy)`，并更新正式标定版本；改变轴映射也需更新版本。

以重心为原点，纯升降 +1 mm 时三杆均增加 1 mm；纯俯仰 +1° 时前两杆各增加约 3.774 mm、后杆减少约 7.548 mm；纯横滚 +1° 时前左减少约 5.847 mm、前右增加约 5.847 mm、后杆不变。

已确认电机与推杆通过 1:1 皮带连接，电机转两圈推杆移动 10 mm，因此 `mm_per_rev=5`：1 mm 对应电机 72°，10 mm 对应 720°。配置示例的三轴已使用这一数值。皮带比例不决定电机角度的正负方向，`extension_sign` 仍需按实际伸出方向核对。

用户已确认三台从固定中位向上、向下各有 50 mm 可用行程。本机配置预填 ±50 mm，通用模拟示例仍采用 ±20 mm。正式后台可在调试高度页“配置 / 启用高度控制”中完成设置：停止并保存固定中位，核对行程、编号和传感器方向，保存后勾选中位确认并获取平台控制。保存生成新的 `calibration_id`，UE 应使用该新版本；操作不会使能或移动电机，原后台配置自动备份。

示例的方向 +1、行程 ±20 mm 尚未经过实机标定。按这组尺寸，纯俯仰 +3° 已使后杆目标约为 -22.661 mm，超过示例行程；升降与倾斜叠加也可能越界，后台会拒绝整个平台目标。姿态范围不能替代三杆组合行程校验，应依据实测中位和可用行程设置。

复制 `backend.platform.example.json` 为独立的正式配置文件，保留已确认的支点坐标和每转 5 mm，核对扫描轴映射并填写实测伸出方向、中位和上下可用行程后才设置 `platform.enabled=true`。示例保留 `calibration_id=simulation-only-v1`，不可当成完整实机标定；本机 `backend.config.json` 不会被程序自动修改。

| 字段 | 含义 |
|---|---|
| `calibration_id` | 本次机构/中位标定版本，UE 必须完全匹配 |
| `legs[].order` | 链路位置，三个不同编号，升序；不是 APS axis_id |
| `x_mm/y_mm` | 三个支撑点相对平台参考中心的水平坐标，不可共线 |
| `mm_per_rev` | 电机轴一转对应的实测升降毫米，包含丝杆和机械减速比；不是电子齿轮分子 |
| `extension_sign` | 公共电机角度坐标中伸出方向 +1/-1；后台会与配置的推出限位方向核对 |
| `min_mm/max_mm` | 相对实测中位的可用行程，预留机械余量，必须包含 0 |
| `max_heave_mm/max_pitch_deg/max_roll_deg` | 姿态范围；同时校验三个撑杆的组合行程，不分别截断 |
| `max_leg_speed_mm_s/max_leg_acceleration_mm_s2` | 每根撑杆速度/加速度上限；按最严格轴转换共同电机参数 |
| `max_tracking_error_mm/tracking_timeout_s` | 命令中间目标与实际杆长持续偏差的停止阈值 |
| `pose_timeout_s` | 平台高度/兼容姿态新帧到达间隔上限，默认 0.25 秒；普通心跳不能代替高度流（字段名保持兼容） |
| `command_interval_s` | 目标下发最短间隔，默认 0.02 秒 |
| `require_endpoint_sensors` | 默认要求每根撑杆配置有效的推出/缩回传感器 |
| `allow_motor_debug` | 默认 false，防止单轴调试绕过平台约束；仅在机构脱开等维护条件下显式开放 |

默认在 `nic.extension_limits/retraction_limits` 中按链路位置 1/2/3 配置 DI；可选 APS 后端在 `aps.extension_limits/retraction_limits` 中配置实际 APS 轴号和 DI。平台模式任意端点触发都会停止全部撑杆；旧单轴调试的“限位后允许反向离开”不适用于承载平台。平台故障后的脱困应按实际机械维护流程进行。

固定初始位保存在后台配置文件旁的 `platform.reference.json`（模拟为 `platform.simulated.reference.json`，与实机隔离）。首次低速将三根撑杆调至实际中位，关闭使能并停稳，在高度页点“保存当前位置为固定初始位”。后台读取两次停稳位置，保存三轴后端原始位置及轴映射/方向/每转单位（切换后端后重新保存）；保存不使能、不移动。再次保存会覆盖旧中位，改变物理中位后应重新测量可用行程并更新标定版本。

停止或服务重启后，点“低速返回固定初始位”，确认位置计数未重置且返回路径可通行，后台重新扫描、核对轴身份，以 5 rpm / 10 rpm/s 返回保存位置。整个返回期间可用 Esc 停止，返回后保持使能；观察三轴实际高度接近 0，关闭使能后才能再次确认中位并获取平台控制。返回流程禁止覆盖目标或速度，任何已配置端点触发停止全轴，不自动反向脱困。保存文件无效、轴映射/方向/电子齿轮或每转单位变化时拒绝返回。

正式后台和独立模拟服务的平台使能还会核对当前位置与保存中位相符，避免把停止位置自动当作新的 0；普通角度模式的 0 仍仅代表本次使能起点。程序不设置驱动器机械原点，不自动寻找限位。软件文件持久化不能保证控制卡计数在断电、重新初始化、清零或换驱动器后仍保持同一坐标；这类情况必须重新建立物理中位并保存，不能直接沿用旧计数。软件无法识别同型号设备被替换后的所有情况，现场核对仍必需。

启动正式后台：

```powershell
.\.venv\Scripts\python.exe backend.py --headless --config your.calibrated.config.json
```

远程 UE 接入时设置后台 host 和 auth_key，UE 使用相同密钥。密钥用于请求认证，UDP 本身没有加密；实际控制网络应保持隔离。

## 控制能力与实机验证边界

软件把三杆目标按同一进度插值，限制每根杆的目标变化速度；驱动器 PP 轨迹限制加速度，单一硬件线程经主板网口提交三轴绝对目标；可选 APS 后端使用控制卡轨迹。这不是控制卡硬件同步插补，不能承诺三轴同一 EtherCAT 周期开始或平台严格跟踪瞬时姿态。实际跟随误差、相位差和可接受帧率必须实机测量；需要严格同步时应使用经板卡能力核对的多轴插补/CSP，而非在 Python 中宣称同步。

后台停止会减速、关闭使能并恢复参数。竖直承载机构必须具备经过验证的自锁/抱闸及独立硬件急停，避免关闭使能、断电或驱动故障时坠落；本项目没有抱闸专用时序，不能替代该硬件功能。

上线前需要在空载低幅条件验证：三个编号/方向/比例，中位和行程余量，两个端点与急停，丢包、暂停和进程退出，部分使能失败与停止确认，以及低频升降/俯仰/横滚时的实际位置和杆间误差。随后才逐步调整载荷、频率、幅度和误差阈值。

## Python SDK 示例

```python
import time
from UdpControl.client import MotorClient

with MotorClient(port=5006) as client:  # 模拟端口；实机必须使用实测标定
    client.hello()
    client.request('scan')
    client.wait_for(('idle',))
    client.save_reference(physical_center_confirmed=True)  # 仅首次模拟；实机需先实际调整到中位
    client.wait_for(('idle',))
    client.enable_platform([1, 2, 3], 'simulation-only-v1', reference_confirmed=True)
    client.wait_for(('enabled',))
    until = time.monotonic() + 5
    while time.monotonic() < until:
        client.heights([1, -1, 0])  # 前左 +1 mm、前右 -1 mm、后中 0 mm
        time.sleep(1 / 30)
    client.disable()
    state = client.wait_for(('idle', 'fault'))
    assert state['result']['all_disabled'] and not state['result']['cleanup_errors']
    client.release()
```

完整无硬件回归：`python -m unittest discover -v`（旧测试需要 requirements-legacy.txt）。平台专项：`python -m unittest -v test_platform_motion test_simulated_backend test_height_control`，使用标准库、Tk 和模拟 APS，不访问真实电机。
