# UDP 控制协议 v1

UTF-8 JSON，一个数据报一个对象，请求最多 8192 字节。默认目的地 `127.0.0.1:5005`。客户端整个会话须保持同一个 UDP socket / 来源端口；所有请求带 `v:1` 和唯一字符串 `id`（1～64 字符）。配置 auth_key 时每个请求还须带 `auth_key`。

## 获取控制权与只读状态

网口通信故障结果可另含 `maximum_pdo_interval_ms`（真实 PDO 发送间隔最大值）、`dc_phase_error_ns` 和 `network_diagnostics`（逐轴 AL 状态与 ESC 错误寄存器）。这些字段用于诊断，不能把缺少完整反馈解释成已经关闭使能。实际 EE08 通信故障仍停机，不自动重新使能。

```json
{"v":1,"id":"h1","type":"hello","claim":true}
```

返回 ack 的 session 后，后续控制请求均携带它。`claim:false` 只观察、不占控制权；`status` 查询也不占权。独占权空闲 5 秒无有效会话请求后释放；电机仍在扫描/使能/停止时不能被另一客户端接管。

```json
{"v":1,"type":"ack","id":"h1","ok":true,"session":"随机字符串","control_seq":-1,"owns_control":true,"server_id":"本次后台实例","state_serial":100,"state":{"phase":"idle","enabled":false,"orders":[],"run_id":null}}
```

state 的 phase 是 `idle / scanning / enabling / enabled / stopping / fault`。enabled 是电机反馈，不是用户开启意图。所有状态响应还携带随机 server_id，后台进程每次启动时更换。state_serial 在该 server_id 内严格递增；客户端只采用当前服务器更大序号的 state，避免迟到反馈使状态倒退。重新 hello 时采用返回的 server_id，并重置旧反馈序号。后台主动以约 20 Hz 向控制客户端推送 `{"v":1,"type":"state","server_id":"...","state_serial":101,"state":{...}}`。

启用后台 `auto_center.enabled` 时，每次进程启动的首次三轴 motor/platform enable 自动进入 `control_mode:"homing"`，寻找两端、回退复测并回中；此时 `phase:"enabled"` 仅表示标定已使能，客户端仍须持续 heartbeat，禁止提交 target/profile/pose/heights。`auto_center.ready:true` 且已返回请求的 motor/platform 模式、`phase:"enabled"` 后才可提交正常目标。成功后沿用原 run_id，恢复正常 profile，序号仍须递增。平台首次自动寻位不依赖旧中位文件；仍须提供当前有效 calibration_id。失败或取消关闭使能，不自动重试或恢复旧目标。状态 `auto_center` 含 enabled/ready/seek_rpm/result，result.phase 为寻位阶段，成功时 axes 含测得的 up_mm/down_mm、固定 origin 和软上下界。清空限位记录只编辑数据并使 ready=false，下次三轴 enable 重测；同一后台正常停用再使能不重复标定。只读启动不运动，旧坐标记录不用于本次首次标定。

网口故障触发后先立即进入 `stopping`，message/stop_reason 保留最初通信或驱动故障，完成关闭使能核对和清理后进入 `fault`。清理期间发生的心跳中断不会覆盖起因；仍须查看 result.all_disabled，不能将 stopping 理解为已关闭使能。显示范围属于客户端本地视图，不发送任何 target/profile/disable 请求。

result.all_disabled 是完整 PDO 关闭确认，all_disabled_sdo 是逐轴 SDO 确认，all_preop 是逐轴总线状态确认；PDO 不完整时不会伪造 all_disabled=true。停止期间可额外通过 SDO 写6040=0关闭仍使能轴，并逐台重试退回 PRE-OP，不复位或自动重启运行。network_diagnostics[].ports 为各 ESC 端口的 invalid_frames/physical_rx_errors/forwarded_rx_errors/lost_links，在重新扫描清空计数前保存。自动回中允许1 mm位置容差；重建零点时换算软上下界，实际端点不扩展。

网口直连普通角度调试允许自由提交滑块目标，包括超过执行行程的目标。后台将实际 PP 终点约束在有效软边界内，先向边界运行，不因远目标立即停止或重置滑块。触发限位或制动边界后暂停本次运动，保持 `phase=enabled` 和 `run_id`，递增 `target_reset_revision`，但保留用户请求的 `targets_deg`。`target_hold_preserves_request:true` 表示该版本变化只通知停车，客户端不得据此改写滑块、清掉新拖动或覆盖编辑值。其它后端没有此标志时沿用旧版同步规则。限位变清、修改 profile 或继续拖动危险方向不会恢复旧运动，需提交反向目标退出；平台高度模式和固定初始位返回仍保持任一端点触发后全轴停止。

SV635N 的 E950.0 / `0x5443` 表示正向超程警告，E952.0 / `0x5444` 表示反向超程警告；单端触发允许反向离开。驱动在 PP Halt 暂停时不会确认新目标，后台提交合法的新目标时在同一 PDO 中解除 Halt 并置位 new-setpoint，再执行正常确认握手，避免等待暂停中的确认而超时关闭使能。使能时的微小位置抖动不会触发运动限位暂停。

网口后台增加 `motion_safety`，含 `max_rpm:3000`、兼容字段 `unreferenced_max_rpm:3000`、`calibrated`、`stop_deceleration_rpm_s`。角度调试的 enable/profile 接受正有限速度至 3000 rpm（含），超过 3000 返回 `ok:false`，不改变当前使能与目标；未标定时也不再限制为 5 rpm。三点高度模式仍受机构标定速度约束；直接电机调试采用已标定包络还要求本次 `reference_confirmed:true`，禁止将保存文件默认为本次物理坐标已确认。`limits` 新增 `positive_latched` / `negative_latched`，端点状态可能为 `latched`：原始 `digital_inputs` 即使已清除，方向保护仍有效。必须按编码器位置反向退回触发边界内才解除方向锁存；记录的边界始终保留为软边界。断电、坐标重置、存储文件丢失或异常退出后均须重新核对位置和标定，不可据电平清除宣称已安全离开。

## 扫描与使能

`adapters` 是只读的控制卡配置查询，与 `status` 一样无需获取控制权，也不占用 control_seq；配置 auth_key 时仍须携带正确密钥。只读刷新不会重新初始化控制卡、扫描电机、改变控制权或续期电机心跳。

```json
{"v":1,"id":"a1","type":"adapters"}
```

返回 `adapters:[{"name":"PCIe-8332:0","description":"..."}]`，表示后台配置的控制卡。后台启动时自动连接该卡并扫描电机，实际连接/扫描结果由 state.phase、state.message 和 state.devices 反馈。面板连接后台后自动刷新并选中该卡；旧客户端附带 session/control_seq 的 adapters 请求仍可读取列表。

以下操作需要 session 和严格递增的 control_seq（0～2^53-1）：`scan / enable / release / save_reference / return_reference / configure_platform / reset_limits / configure_soft_limits`。从 hello 返回的 control_seq+1 开始递增。重发同一个请求时保留 id、control_seq 和内容；失败后的新请求换 id 并增加 control_seq。

```json
{"v":1,"id":"s1","type":"scan","session":"...","control_seq":1,"adapter":"PCIe-8332:0"}
```

adapter 可省略，使用后台当前控制卡配置；PCIe 后台仅接受匹配 aps.board_id 的卡标签。扫描为异步请求；轮询 status 或接收推送，等 phase=idle 后读取 state.devices。链路位置 order（1 起始）作为控制编号，ID 0 也不等于链路位置 0。

```json
{"v":1,"id":"e1","type":"enable","session":"...","control_seq":2,"orders":[1,2,3],"rpm":80,"acceleration_rpm_s":120}
```

orders 须为不同的升序链路位置，不限定三轴。返回 `run_id`，立即开始心跳，直到 phase=enabled 再提交目标。使能不会自动移动，也不会自动执行上一运行目标。rpm 默认 60，acceleration_rpm_s 默认 120；两者为正的有限数值，使能就绪后可通过 profile 成对更新。

## 网口限位记录与手动软限位

`motion_safety.requires_soft_limits=true`：连续角度模式要求每轴完整有效的软上下界。缺少时 target 仍作为界面意图接受和显示，硬件保持 Halt，不执行运动；message 提示关闭使能并设置实测边界。3000 rpm enable/profile 上限不变。触限后的新反向目标等待驱动零速状态和至少 50 ms 编码器稳定窗口后执行，仅保留最新目标；危险方向更新取消等待目标，停止/重启丢弃它。停车中的重复目标不改变首次固定的 Halt 停车点，降 profile 不降低尚未完成的制动减速度。

仅 EtherCAT 后台支持，仅 `idle/fault` 接受，并在硬件线程重新扫描、确认所有轴未使能后修改。ACK 表示开始，等待 `idle` 或读取 `fault` 原因；不使能、不移动、不清零编码器。SDK 提供 `reset_limits(orders)` 和 `configure_soft_limits(axes)`。

```json
{"v":1,"id":"limits1","type":"reset_limits","session":"...","control_seq":6,"orders":[1,2,3]}
```

仅清除所选轴旧的传感器位置记录，保留手动设置。仍在触发的实际输入会重新记录和阻止危险方向。

```json
{"v":1,"id":"limits2","type":"configure_soft_limits","session":"...","control_seq":7,"axes":[{"order":1,"enabled":true,"origin":1000,"lower_degrees":-1800,"upper_degrees":1800},{"order":2,"enabled":false}]}
```

启用时 `origin` 为扫描到的原生 32 位编码器固定零点，角度符号与该轴电机调试一致，必须有限且下限小于上限。不是每次 enable 重新捕获的原点，也不是毫米数值。仅更改列出的轴；每个列出轴的旧传感器位置记录被替换。整组验证通过后原子保存 `logs/nic-limit-memory.json`，首次编辑前备份 `.before-edit.bak`，重启保留。手动软限位与其它有效包络取交集，不能屏蔽物理输入。

`state.limit_settings` 包含 `available` 和 `axes`；轴项为 `order/position/origin/current_degrees/enabled/lower_degrees/upper_degrees/learned_positive/learned_negative/identity_matches`。`position` 是当前原生计数，`current_degrees` 相对该固定零点；无手动设置时零点预填当前计数且上下限为 null。旧学习记录不匹配轴身份时 `identity_matches=false`，操作员须重新核对并编辑，不能静默当成有效边界。普通角度运行中新形成的冲突暂停并保持使能，提示停机后重置；启用前已有冲突仍拒绝启动。

## 三撑杆平台模式

### 在线配置高度控制

正式后台的 `state.platform.setup_available=true` 时，停止并保存固定中位后，可提交以下请求；必须持有 session，`control_seq` 严格递增：

```json
{"v":1,"id":"setup1","type":"configure_platform","session":"...","control_seq":5,"reference_id":"保存的中位版本","calibration_confirmed":true,"travel_mm":[{"order":1,"up_mm":50,"down_mm":50},{"order":2,"up_mm":50,"down_mm":50},{"order":3,"up_mm":50,"down_mm":50}]}
```

上下行程为实测正毫米数值，操作员确认编号和两端传感器物理对应方向。后台使用本项目已确认的 670/730/730 mm 机构和每转 5 mm，从已扫描轴的端点方向计算各轴角度符号；保存新标定版本并启用高度模式，不使能、不移动。仅 idle 接受，固定中位版本必须匹配。后台配置文件的其它字段保留，首次保存前备份为 `backend.config.json.before-platform.bak`；写入失败不切换运行配置。

初次平台限制为整体升降取三杆上下行程的共同可用范围、倾角 ±3°、速度 5 mm/s、加速度 10 mm/s²；每帧还需满足三杆组合行程。保存后重新确认中位并请求平台使能，不能将当前角度模式直接切换为高度运行。端口 5006 的独立模拟服务已启用模拟配置，无需本操作。

### 固定初始位

以下请求需要控制 session 和严格递增的 control_seq，适用三个链路位置 1/2/3。停止、处于 idle 且三根撑杆确在物理中位后保存：

```json
{"v":1,"id":"centre1","type":"save_reference","session":"...","control_seq":3,"physical_center_confirmed":true}
```

保存异步核对 disabled 三轴的两次静止位置，不使能、不移动；ACK 只表示开始，等待 idle 和 `state.fixed_reference.valid=true`。位置和轴身份写入后台配置旁的文件。重新保存改变 reference id，旧版本返回请求拒绝。返回需要再次明确确认坐标计数未重置且路径可通行：

```json
{"v":1,"id":"return1","type":"return_reference","session":"...","control_seq":4,"reference_id":"保存版本","position_reference_confirmed":true}
```

返回 run_id 后保持 heartbeat，后台采用独立 `control_mode=reference`，重新扫描并校验身份，以 5 rpm / 10 rpm/s 回到保存中位；返回后保持使能，显式 disable 停止。reference 模式禁止 target/heights/pose/profile 覆盖返回路径；端点触发停止全轴。停止或重启后文件保留，但位置计数必须仍属于原坐标。清零、坐标重置或更换硬件需重新标定；服务不自动寻找限位或设置机械原点。

`state.fixed_reference` 包含 `available/saved/valid/id/error` 及 `heights_mm`（相对保存中位的三轴高度，按每转 5 mm 及核对的端点方向转换，方向未知为 null）。valid 表示文件/轴身份通过检查，不证明物理中位或计数坐标未被重置。正式平台使能会再核对当前原始位置与保存位置一致，不能将偏离中位的停止处捕获为新原点。

现有电机模式保持兼容。平台扩展使用同一 v1 协议、认证、控制权和去重规则，必须先完成后台机构配置。请求：

```json
{"v":1,"id":"pe1","type":"enable","session":"...","control_seq":2,"mode":"platform","orders":[1,2,3],"calibration_id":"实测标定版本","reference_confirmed":true}
```

`orders` 必须等于三个配置撑杆的升序编号。`reference_confirmed` 必须为布尔 true，表示操作员已核对物理中位；后台不能验证该人工声明。速度/加速度由后台平台配置生成，平台模式拒绝 target/profile，不能从客户端覆盖。配置平台后普通电机 enable 默认也被禁止，除非维护配置显式允许。

取得 run_id 后开始心跳，等 phase=enabled 再持续发送最新三点高度（当前 UE 与界面使用）：

```json
{"v":1,"id":"height1","type":"heights","session":"...","run_id":"...","seq":3,"heights_mm":[5,-3,0]}
```

`heights_mm` 必须是三个有限数值，顺序与配置 `orders` 一致，本机构为 `[前左1号, 前右2号, 后中3号]`。单位 mm，相对已标定初始位的绝对偏移；0 回到初始位并保持使能，正数向上推，负数向下拉。重复值不累加。每帧原子更新三轴，校验所有撑杆行程与由三点恢复的平台升降/倾斜范围；一轴越界不允许另外两轴部分执行。当前运行缺失、null、长度错误或无效高度均停止全轴；旧 run_id、未授权及重复/乱序报文不会停止当前运行或续期。

`heights` 和兼容 `pose` 共用目标序号通道，与心跳独立。高度有效新帧续期网络心跳及平台输入计时；仅 heartbeat 无法维持平台运行。使能就绪后必须在 `pose_timeout_s` 内收到首帧，静止时也应持续发送 30～60 Hz；高度帧不重试，后续最新帧覆盖旧目标。过期后需重新确认物理初始位并显式使能，不能恢复旧 run。

后台保留姿态输入兼容既有调用方：

```json
{"v":1,"id":"pose1","type":"pose","session":"...","run_id":"...","seq":3,"pose":{"heave_mm":1,"pitch_deg":0.5,"roll_deg":-0.5}}
```

pose 对象只能包含这三个有限数值字段；毫米/度的坐标定义见根目录 UE接入.md。姿态共用 target 的独立序号通道，与 heartbeat 的序号互不覆盖。重复/乱序姿态不续期。有效新姿态同时续期网络心跳和姿态流计时；只有 heartbeat 不能续期姿态。使能完成后 pose_timeout_s 内必须收到首帧，默认 0.25 秒；过期的新姿态也不能恢复旧运行。客户端应发送 30～60 Hz，旧姿态不重试。

当前运行的越界/无效姿态停止全部撑杆；旧 run_id 或未授权请求不能停止当前平台。所有姿态与组合撑杆行程都验证通过才更新目标，不截断单个撑杆。任意端点触发、配置传感器反馈失效或持续跟随误差超限，均停止全轴；驱动器故障沿用后台故障清理流程。停止/重启需要再次确认标定中位，不能把停止位置自动当成中位。

state 新增 `control_mode` 和 `platform`：`configured/calibration_id/orders` 标识平台配置，`pose` 是最近接受或由高度恢复的姿态，`lengths_mm` 是三个期望高度偏移，`travel_bounds_mm` 是每轴可用高度范围 `[min,max]`，`max_heave_mm` 是整体升降上限。`commanded_deg` 是平滑器生成的最近中间轴目标，`actual_lengths_mm` 是由轴反馈换算的杆长偏移（未知为 null），`pose_age_s/pose_timeout_s` 表示高度/兼容姿态流年龄与超时（字段名保持兼容）。`targets_deg` 是最终轴目标，ACK 不表示最终位置已到达。`hardware_backend=simulated` 只表示无硬件模拟，不应作为实机反馈。

## 心跳与目标

从 enable 返回的 run_id 开始新运行。heartbeat、target、profile 各自使用严格递增 seq（0～2^53-1）；可共用全局递增计数，也可分开计数。后台分别记住三种消息的最后 seq。推荐持续 30～60 Hz 目标，目标静止时至少每 100 ms 发送 heartbeat。

```json
{"v":1,"id":"hb1","type":"heartbeat","session":"...","run_id":"...","seq":0}
```

```json
{"v":1,"id":"t1","type":"target","session":"...","run_id":"...","seq":1,"targets_deg":[5,0,-5]}
```

targets_deg 的数量与 orders 相同，依次为相对各轴本次使能起点的绝对偏移，单位 °；可正、负、零。相同目标不累加、也不重新触发轨迹。目标会覆盖尚未下发的旧目标，不积压执行队列。角度累计不设限，但单次剩余位移须可由驱动器原生格式表示。

ack 的 `accepted:true` 表示新序号通过验证；`accepted:false` 表示重复/乱序报文被忽略，不更新目标或心跳。无效请求返回 `ok:false,error:"原因"`，不改变已接受目标，不续期；不能当作已执行。心跳超时默认 0.5 秒，准备阶段也计时，超时后即使后来收到新 seq 也不能恢复该 run。

## 使能期间更新速度和加减速度

```json
{"v":1,"id":"p1","type":"profile","session":"...","run_id":"...","seq":2,"rpm":40,"acceleration_rpm_s":80}
```

仅 phase=enabled 接受，两个参数都必须提供，为正的有限数值。更新同时作用于本次所有选中轴；位置目标、使能起点和 run_id 保持不变。正在运动的轴立即按原目标重新提交原生轨迹参数，空闲轴保持当前位置；被限位停止或拦截的旧目标不会因参数更新而恢复。停止减速度同步更新，停止清理时恢复使能前保存的参数。

ACK 表示请求已接受；state.profile 是最近接受的 `{revision,rpm,acceleration_rpm_s}`，state.applied_profile 是硬件线程最近成功应用的同结构值。两者 revision 一致表示该组参数已应用，不能作为运动到位依据。高频更新合并为最新参数；硬件转换或写入失败会停止并进入故障处理。有效的新序号 profile 也续期电机心跳；重复、乱序、无效或旧运行的请求不续期。

## 停止、释放和重启

```json
{"v":1,"id":"d1","type":"disable","session":"...","run_id":"..."}
```

disable 不要求 seq，匹配当前 run_id 就优先请求停止。ack 仅表示停止请求已经接受；等待 phase=idle/fault，再核对 state.result.all_disabled 和 cleanup_errors。旧 run_id 的 disable 被拒绝。停止后 target/heartbeat/profile 不可重新使能。

```json
{"v":1,"id":"r1","type":"release","session":"...","control_seq":3}
```

停止/扫描结束后才能释放。下一个客户端重新 hello，获取新的 session。干净停止后同一客户端可使用新 id/control_seq 再 enable；fault 必须重新 scan 通过才可 enable。service process 重启后必须重新 hello，旧 session 不可用。

## ACK 和反馈

所有已解析且带合法请求标识的报文都会返回同 id 的 ack；响应不保证送达，客户端在超时后重发**完全相同**的报文。后台有限去重缓存保留最近 256 个控制请求；缓存淘汰后旧 control_seq / target seq 仍禁止重复执行。hello、status 可随时重新查询。

```json
{"v":1,"id":"q1","type":"status"}
```

完整 state 还包括 adapter、message、heartbeat_timeout_s、orders、targets_deg、seq、devices、axes、result、stop_reason。PCIe 后台 state.hardware_backend 为 aps，input_configuration 包含 limit_inputs_connected / emg_input_connected（布尔值），devices 新增 axis_id、slave_id、units_per_rev，axes 项含 order、axis_id、slave_id、position（APS 原始位置单位）、enabled、error_code、travel_degrees（累计偏移）。result 含 stopped、error、all_disabled、cleanup_errors、log_path、stop_reason；旧网卡后端另有 PDO 恢复统计（APS 后台不使用它）。feedback 中 seq 是所接收的最大序号，**不表示到位**。具体轴位置以 axes 为准。

不确定 enable 是否执行时先查 state.run_id 和 phase；若尚在准备，保持当前 run 心跳或明确停止。不要用不同 id 重发 enable。停止失败或反馈失联时不能仅凭 ack 宣称电机已关闭。

PCIe 后台新增 `state.limits`（扫描后持续更新，包括未使能/未选中轴）：每项包含 `order`、`input`（推出端 `di1`/`di2` 或 null）、`state`（`triggered`/`clear`/`unconfigured`/`unavailable`）、`valid`、`triggered`（布尔或 null）、`extension_sign`（APS 原生位置方向 ±1；未配置为 0）、`digital_inputs`（60FD 原始值）、`positive_limit`、`negative_limit`、`di1`、`di2`。双端配置另有 `retraction_input`、`retraction_sign`、`retraction_state`、`retraction_triggered`，含义与推出端相同；未配置缩回端时 `retraction_state:"no_sensor"`。`conflict:true` 表示两端同时触发。未知反馈不代表未触发；通信新鲜度仍由接收时间判断。`input_configuration` 包含逐轴 `extension_limits` / `retraction_limits`，配置见根目录后台说明。

推出端限位触发时只允许缩回，缩回端触发时只允许推出。APS 后端朝已知触发端运动的请求返回 `ok:false`；网口普通角度调试接受该界面意图并保留请求值，由硬件线程约束执行终点或保持限位暂停，不限制滑块。若请求入队后才触发，硬件线程也会停止危险方向的运动并更新 `state.message`，ACK 不代表电机已执行或到位。旧目标不会在传感器解除后自动恢复；触发后需显式提交离开该端的反向目标，并继续心跳。运行中两端同时触发时停止冲突轴运动并保持 `enabled`，两方向目标均拒绝；信号恢复后必须提交新目标，不恢复旧运动。使能前已有双端冲突仍拒绝启动；驱动报警、急停、掉线等仍进入 `fault`。`clear` 不表示到达机械零点，传感器触发也不会自动执行回零。

## 后台退出通知

后台正常退出时向最近连接并通过认证的客户端（包括只读客户端）重复发送 `{"v":1,"type":"shutdown","server_id":"...","message":"后台正在退出"}`。客户端只接受来自配置地址、且 server_id 与当前后台一致的通知；停止心跳及新运动请求，唤醒等待中的 RPC，控制界面关闭窗口并退出进程。SDK 的 `backend_exiting` 事件表示收到退出通知；需要连接新后台时可显式调用 `hello()`，新 server_id 会清除此事件，旧实例的退出通知不能关闭新连接。
