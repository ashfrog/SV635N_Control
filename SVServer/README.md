# SVServer 便携后台（Windows x64）

双击 `SVServer.exe` 或 `启动服务.cmd` 即可启动后台，图标位于 Windows 右下角托盘。托盘可打开独立控制界面、停止全部电机、停止并退出后台。启动只扫描，不使能电机；重复启动会打开已有后台的控制界面。

`打开控制界面.cmd` 仅启动界面，不启动后台。也可执行 `SVServer.exe --client`。控制界面通过 UDP 连接服务，默认 `127.0.0.1:5005`。

本目录已包含 Python 和依赖库，运行不需要安装 Python 或复制项目源码。请复制**整个 SVServer 文件夹**，保留 `_internal/`；不可只复制 EXE。放在有写权限的目录，配置和日志保存在 EXE 旁边，不依赖当前工作目录。

## 控制卡与配置

真实硬件仍需要安装 ADLINK PCIe-8332 驱动、APS SDK x64 和正确的 SV635N ESI。APS DLL 不随本程序分发；必要时在 `backend.config.json` 的 `aps.dll_path` 填写已安装的 `APS168x64.dll` 绝对路径。首次使用须核对控制卡、轴映射、DI 接线和急停配置，关闭其他控制卡占用程序。

`backend.config.json` 是本机实际配置，首次构建从项目现有配置复制；重新打包不会覆盖本目录已有配置和日志。换电脑时必须按实际硬件调整，示例中的轴号、传感器和急停配置不表示新机器的实测值。平台机构参数见项目 `UE接入.md`；`backend.platform.example.json` 只含虚拟机构，默认关闭实机平台模式。

后台记录在 `logs/service.log`，客户端记录在 `logs/client.log`。关闭客户端会停止该客户端的运行，后台继续运行；退出真实后台请使用托盘“停止并退出后台”，等待停止/关闭使能及参数恢复完成。

本程序是托盘常驻进程，未注册为 Windows 系统服务，未修改开机启动。

## 无硬件模拟联调

双击 `启动模拟服务.cmd`，或执行 `SVServer.exe --simulate --port 5006`。此模式不加载 APS DLL、不访问电机，使用虚拟平台配置；控制界面用 `SVServer.exe --client --port 5006`，UE 使用端口 5006 和标定名 `simulation-only-v1`。模拟进程无托盘；只在确认模拟模式时可通过任务管理器结束它。

## 从源码重新构建

在完整项目内双击 `重新打包.cmd`，或者：

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-build.txt
.\.venv\Scripts\python.exe build_svserver.py
```

构建输出为 `SVServer/SVServer.exe` 和 `_internal/`，构建中间文件位于 `.build/svserver/`。发布包未进行代码签名。

重新打包前退出正在使用本目录的 EXE。可运行 `python packaging/smoke_test.py` 验证迁移运行、模拟平台协议、托盘入口和独立客户端；验证不会访问真实 APS 硬件，结果写入 `verification.json`。
