# UE 三轴接入参考（暂未接入界面）

当前界面仅提供本地手动连续控制，不启动此 UDP 接口。本文和 `UE/` 组件保留为后续对接参考；要使用网络接口需由独立调用方显式启动文末的 `platform_control.run_three_axis()`。本地界面操作见 [使用说明.md](使用说明.md)。

当前接入层接收**三台电机的目标轴角度**，用于将游戏运动输出连续送给 SV635N。目标不是 Pitch、Roll、Heave；如游戏只有平台姿态，先按实际连杆尺寸、减速比、行程和安装方向做机构逆解，再输出三台电机角度。没有这些参数时，不能把平台姿态直接当成电机角度。

## 操作

1. 通过独立调用方扫描电机，按**链路位置升序**传入三台电机的编号，对应目标数组第 1/2/3 项，握手反馈中的 `orders` 可核对。支持链路上其他未选设备保持未使能。
2. 调用参数提供最高速度、加减速度、限位和超时。方向由目标角度正负表示，从轴端看正为顺时针，程序按各轴 H02.02 换算。
3. 在运行条件核对后显式调用 `run_three_axis()`。默认监听 `127.0.0.1:5005`，默认限位 ±30°、超时 0.5 秒；完成 OP 建链后等待 UE，等待期间不使能。
4. UE 从同一个 UDP socket 握手，并以 30～60 Hz 连续发送 `enable=true` 和三个目标角度。即使角度不变，也持续递增 `seq` 发送，作为心跳。首次使能前连续发送即可，无须等待建链完成。
5. 调用方设置 `stop` 事件、UE 发送 `enable=false` 或有效指令超时，均结束当前会话并停止/关闭使能及恢复参数。重新运行须显式启动新会话并重新握手，旧会话指令不会启用新会话。

每次使能时的实际位置作为三个软件零点，不改写驱动器原点；重启控制后零点会重新捕获。该限位是相对软件零点的电机轴角度限位，不能替代机械行程限位或标定。

## UDP JSON 协议

UTF-8，每个 UDP 数据报一个 JSON 对象，命令最大 4096 字节。默认仅同机；首个 `hello` 的来源 IP/端口绑定会话，后续命令必须使用同一个 socket。session 用于隔离旧报文，不是密码认证。

握手请求（可在等待回复时重发）：

```json
{"type":"hello"}
```

反馈示例（实际 session 每次不同）：

```json
{"type":"state","session":"本次返回的32位字符串","orders":[1,2,3],"limit_deg":30.0,"timeout_s":0.5,"seq":-1,"phase":"starting","enabled":false,"stopping":false}
```

正常目标，三个数均为**相对本次使能起点的绝对偏移角度**；重复 `[5,0,-5]` 不会再累加 5°：

```json
{"type":"command","session":"本次握手返回值","seq":0,"enable":true,"targets_deg":[5.0,0.0,-5.0]}
```

关闭使能：

```json
{"type":"command","session":"本次握手返回值","seq":1,"enable":false}
```

- `seq`：0～2^53-1 的整数，在会话内严格递增。乱序/重复目标与旧 session 不更新心跳；有效关闭使能指令即使乱序也优先停止。
- `enable`：必须为 JSON 布尔值。本地许可和 UE 开启两者同时满足才使能；关闭后当前会话无法用 `true` 自动重新启动。
- `targets_deg`：三个有限数值，可为 0；超出本地配置限位的有效会话指令会停止全部轴，不会截断为可执行目标。
- 状态以约 10 Hz 返回发送端，`axes` 含链路位置、实际编码器位置、实际使能、报警与相对起点的 `travel_degrees`。`enabled` 是驱动器反馈，不是 UE 请求值。
- `seq` 是最新接收的有效序号，不表示目标已经到位；确认位置请读取 `axes`。最终 `result` 包含停止、恢复/关闭使能核对信息，UDP 反馈不保证必达，以本地日志为准。

接收只保留最新目标。三个轴在同一 PDO 周期收到绝对目标和 PP 新位置触发，驱动器确认并清除确认位后可继续更新目标，无须等待上一目标到位；用 PP 的“立即改变位置”位请求跟随新目标，速度/加速度受本地配置限制。相同编码器目标仅更新心跳，不重新触发轨迹。

当前仍是 PP 运动模式，三台电机独立规划，尚未在实物上验证连续立即更新的跟随表现。Windows + 普通网卡不是实时系统，不能承诺高频姿态跟随或三轴机械同步。若需要真正逐周期同步插补，应进一步核对驱动器 CSP 能力、PDO 配置及机构标定。

## UE C++ / 蓝图

提供的 `UE/SV635NMotionComponent.h` 与 `.cpp` 可复制到 UE 项目的 `Source/你的模块/`。在对应模块 `.Build.cs` 的依赖中加入：

```csharp
PrivateDependencyModuleNames.AddRange(new string[] { "Sockets", "Networking", "Json" });
```

组件使用标准 UE Socket API，适用于 UE5 C++ 项目。在 Actor 上添加 `SV635NMotionComponent`，每帧调用蓝图 `Set Motor Targets` 提交三个电机角度，调用 `Set Motor Enable(true)` 开启 UE 发送，组件固定以约 30 Hz 发送。关闭时调用 `Set Motor Enable(false)`；退出游戏也发送关闭使能，突然退出由 PC 超时处理。查看 `Hardware Enabled`、`Last Status`、`Actual Motor Degrees` 和 `Mapped Orders` 获取反馈；`Feedback Valid` 为 false 时这些数值仅代表最后一次收到的反馈。

组件遇到反馈 `stopping` 或 `closed` 会撤销 UE 使能请求；重新运行时先由 PC 调用方启动新会话，再在 UE 重新调用开启。UE 工程不在此仓库，示例需在目标工程中编译。

蓝图纯项目可用具有 UDP 收发功能的插件按上述协议接入；UE 默认蓝图没有通用 UDP JSON 发送节点。

## 示例客户端

独立调用方已启动 `run_three_axis()` 后，以下命令会向桥接程序发送三路缓慢正弦目标，默认幅度 5°，20 秒后发送关闭使能；这是运动客户端：

```powershell
.\.venv\Scripts\python.exe ue_demo.py --amplitude 5 --period 10 --seconds 20
```

不连接硬件的协议与模拟驱动器回归测试：

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_platform_control
```

程序接入示例（实际运动 API，仍须 UE 有效开启命令）：

```python
import threading
from core import EtherCATController
from platform_control import run_three_axis

controller = EtherCATController(log_dir='logs')
devices = controller.scan()
stop = threading.Event()
report = run_three_axis(controller, devices, [1, 2, 3], stop=stop,
                        rpm=5, acceleration=10, limit_degrees=30,
                        host='127.0.0.1', port=5005, timeout=0.5)
```

API 阻塞至会话结束，应放在后台线程；另一线程可调用 `stop.set()`。日志保留最后 10 秒 PDO 与累计目标更新次数，网络线程不读写 EtherCAT。原有 `cli.py` 仍只提供单次验证/调试。
