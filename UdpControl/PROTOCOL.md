# UDP 控制协议 v1

UTF-8 JSON，一个数据报一个对象，请求最多 8192 字节。默认目的地 `127.0.0.1:5005`。客户端整个会话须保持同一个 UDP socket / 来源端口；所有请求带 `v:1` 和唯一字符串 `id`（1～64 字符）。配置 auth_key 时每个请求还须带 `auth_key`。

## 获取控制权与只读状态

```json
{"v":1,"id":"h1","type":"hello","claim":true}
```

返回 ack 的 session 后，后续控制请求均携带它。`claim:false` 只观察、不占控制权；`status` 查询也不占权。独占权空闲 5 秒无有效会话请求后释放；电机仍在扫描/使能/停止时不能被另一客户端接管。

```json
{"v":1,"type":"ack","id":"h1","ok":true,"session":"随机字符串","control_seq":-1,"owns_control":true,"server_id":"本次后台实例","state_serial":100,"state":{"phase":"idle","enabled":false,"orders":[],"run_id":null}}
```

state 的 phase 是 `idle / scanning / enabling / enabled / stopping / fault`。enabled 是电机反馈，不是用户开启意图。所有状态响应还携带随机 server_id，后台进程每次启动时更换。state_serial 在该 server_id 内严格递增；客户端只采用当前服务器更大序号的 state，避免迟到反馈使状态倒退。重新 hello 时采用返回的 server_id，并重置旧反馈序号。后台主动以约 20 Hz 向控制客户端推送 `{"v":1,"type":"state","server_id":"...","state_serial":101,"state":{...}}`。

## 扫描与使能

`adapters` 是只读的控制卡配置查询，与 `status` 一样无需获取控制权，也不占用 control_seq；配置 auth_key 时仍须携带正确密钥。只读刷新不会重新初始化控制卡、扫描电机、改变控制权或续期电机心跳。

```json
{"v":1,"id":"a1","type":"adapters"}
```

返回 `adapters:[{"name":"PCIe-8332:0","description":"..."}]`，表示后台配置的控制卡。后台启动时自动连接该卡并扫描电机，实际连接/扫描结果由 state.phase、state.message 和 state.devices 反馈。面板连接后台后自动刷新并选中该卡；旧客户端附带 session/control_seq 的 adapters 请求仍可读取列表。

以下操作需要 session 和严格递增的 control_seq（0～2^53-1）：`scan / enable / release`。从 hello 返回的 control_seq+1 开始递增。重发同一个请求时保留 id、control_seq 和内容；失败后的新请求换 id 并增加 control_seq。

```json
{"v":1,"id":"s1","type":"scan","session":"...","control_seq":1,"adapter":"PCIe-8332:0"}
```

adapter 可省略，使用后台当前控制卡配置；PCIe 后台仅接受匹配 aps.board_id 的卡标签。扫描为异步请求；轮询 status 或接收推送，等 phase=idle 后读取 state.devices。链路位置 order（1 起始）作为控制编号，ID 0 也不等于链路位置 0。

```json
{"v":1,"id":"e1","type":"enable","session":"...","control_seq":2,"orders":[1,2,3],"rpm":80,"acceleration_rpm_s":120}
```

orders 须为不同的升序链路位置，不限定三轴。返回 `run_id`，立即开始心跳，直到 phase=enabled 再提交目标。使能不会自动移动，也不会自动执行上一运行目标。rpm 默认 60，acceleration_rpm_s 默认 120；两者为正的有限数值，使能就绪后可通过 profile 成对更新。

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

推出端限位触发时只允许缩回，缩回端触发时只允许推出。朝已知触发端运动的请求返回 `ok:false`；若请求入队后才触发，由硬件线程丢弃整条多轴目标并更新 `state.message`，先前 ACK 不代表已执行。旧目标不会在传感器解除后自动恢复；触发后需显式提交离开该端的反向目标，并继续心跳。运行中两端同时触发时停止冲突轴运动并保持 `enabled`，两方向目标均拒绝；信号恢复后必须提交新目标，不恢复旧运动。使能前已有双端冲突仍拒绝启动；驱动报警、急停、掉线等仍进入 `fault`。`clear` 不表示到达机械零点，传感器触发也不会自动执行回零。

## 后台退出通知

后台正常退出时向最近连接并通过认证的客户端（包括只读客户端）重复发送 `{"v":1,"type":"shutdown","server_id":"...","message":"后台正在退出"}`。客户端只接受来自配置地址、且 server_id 与当前后台一致的通知；停止心跳及新运动请求，唤醒等待中的 RPC，控制界面关闭窗口并退出进程。SDK 的 `backend_exiting` 事件表示收到退出通知；需要连接新后台时可显式调用 `hello()`，新 server_id 会清除此事件，旧实例的退出通知不能关闭新连接。
