# UE 三撑杆平台接入（后台 UDP v1）

`UE/SV635NMotionComponent` 现在直接连接 `backend.py`，输入平台升降毫米、俯仰角和横滚角。旧 `platform_control.py` / `ue_demo.py` 使用另一套三轴电机角度协议，不能与本组件混用，也不要同时占用同一控制卡或 UDP 端口。

## 架构与坐标

UE → UDP v1 → MotorService → PlatformGeometry / PlatformMotionQueue → APS SDK → PCIe-8332 → 三台 SV635N。

固定平台坐标：X 向前、Y 向右、Z 向上。升降为毫米，角度为度；正俯仰抬高前端，正横滚抬高右端。UE 世界位置默认厘米，传入升降前需要乘 10。接口的角度采用本文约定，调用方应按实际 UE 姿态来源核对符号，不直接照搬未知坐标系的 Rotator。

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

模拟后台监听 `127.0.0.1:5006`，不加载 APS DLL，不访问电机。UE 设置 Port=5006、CalibrationId=`simulation-only-v1`、Orders=[1,2,3]。模拟器从 `backend.platform.example.json` 读取虚拟机构，在内存中启用平台模式；反馈 `hardware_backend=simulated`。它验证协议、姿态换算、平滑目标和状态流程，不模拟真实制动器、驱动报警、EtherCAT 时序或载荷。

## UE C++ / 蓝图

将 `UE/SV635NMotionComponent.h` 和 `.cpp` 复制到目标 UE5 C++ 工程的 `Source/你的模块/`，在 `.Build.cs` 增加：

```csharp
PrivateDependencyModuleNames.AddRange(new string[] { "Sockets", "Networking", "Json" });
```

若需要跨模块访问组件，为类声明增加目标模块的 `YOURMODULE_API` 宏。本仓库没有 `.uproject`，组件必须在目标 UE 工程编译与打包验证。

在 Actor 添加组件，并按顺序接线：

1. 每次游戏更新调用 `Set Platform Pose(HeaveMillimetres, PitchDegrees, RollDegrees)`，静止时也持续调用。首次运行先发送 `(0,0,0)`。
2. 等 `Feedback Valid=true`、后台空闲、标定匹配。操作员确认三根撑杆确实处于标定中位，再显式调用 `Arm Platform(true)`。返回 true 仅表示开始异步申请；实际使能看反馈。
3. 默认以 30 Hz 发送当前姿态。使能准备阶段发送心跳，后台使能就绪后发送姿态。
4. 停止按钮调用 `Stop Platform`。结束 Play 也提交停止；暂停、游戏卡顿、姿态断流、反馈超时、故障或后台重启会撤销运动意图。
5. 故障或停止后不会自动重启。处理问题并重新确认物理中位后才能再次 Arm；后台 fault 状态先通过客户端/SDK显式重新扫描。

组件固定使用同一 UDP socket，控制请求保留相同 id/内容重试，姿态请求只发送最新值；使用 server_id/state_serial 拒绝旧实例和乱序状态。停止过程中到达的迟到使能 ACK 会再请求关闭使能。游戏结束时不等待网络确认，后台心跳和姿态超时负责补充停止。

`Hardware Enabled` 和 `Actual Motor Degrees` 是最后观测值，失联时不能认定电机已停止。核对 `Feedback Valid`、后台 `result.all_disabled` 和 `cleanup_errors`；SDK ACK 表示接受请求，不表示运动到位。

## 真实机构配置与标定

复制 `backend.platform.example.json` 为独立的正式配置文件，填写实测参数后才设置 `platform.enabled=true`。示例数字和标定名只适合模拟，不可当成实机参数；本机 `backend.config.json` 不会被程序自动修改。

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
| `pose_timeout_s` | 新姿态到达间隔上限，默认 0.25 秒；普通心跳不能代替姿态流 |
| `command_interval_s` | 目标下发最短间隔，默认 0.02 秒 |
| `require_endpoint_sensors` | 默认要求每根撑杆配置有效的推出/缩回传感器 |
| `allow_motor_debug` | 默认 false，防止单轴调试绕过平台约束；仅在机构脱开等维护条件下显式开放 |

在 `aps.extension_limits/retraction_limits` 中配置实际 APS 轴号和 DI。平台模式任意端点触发都会停止全部撑杆；旧单轴调试的“限位后允许反向离开”不适用于承载平台。平台故障后的脱困应按实际机械维护流程进行。

中位是每次使能起点，不是驱动器的持久机械原点。程序不自动回零，不通过端点传感器推断中位，也不能证明操作员确认属实。停止后的当前位置不能直接视为新中位；重新运行必须回到已标定物理中位，或重新测量可用行程并更新标定版本。

启动正式后台：

```powershell
.\.venv\Scripts\python.exe backend.py --headless --config your.calibrated.config.json
```

远程 UE 接入时设置后台 host 和 auth_key，UE 使用相同密钥。密钥用于请求认证，UDP 本身没有加密；实际控制网络应保持隔离。

## 控制能力与实机验证边界

软件把三杆目标按同一进度插值，限制每根杆的目标变化速度；APS 原生轨迹限制加速度，单一硬件线程顺序提交三轴绝对目标。这不是控制卡硬件同步插补，不能承诺三轴同一 EtherCAT 周期开始或平台严格跟踪瞬时姿态。实际跟随误差、相位差和可接受帧率必须实机测量；需要严格同步时应使用经板卡能力核对的多轴插补/CSP，而非在 Python 中宣称同步。

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
    client.enable_platform([1, 2, 3], 'simulation-only-v1', reference_confirmed=True)
    client.wait_for(('enabled',))
    until = time.monotonic() + 5
    while time.monotonic() < until:
        client.pose(1, 0, 0)
        time.sleep(1 / 30)
    client.disable()
    state = client.wait_for(('idle', 'fault'))
    assert state['result']['all_disabled'] and not state['result']['cleanup_errors']
    client.release()
```

完整无硬件回归：`python -m unittest discover -v`（旧测试需要 requirements-legacy.txt）。平台专项：`python -m unittest -v test_platform_motion test_simulated_backend`，只使用标准库和模拟 APS，不访问真实电机。
