# SV635N_Control

SV635N EtherCAT 电机调试工具（Python 源码版），支持单次调试及界面手动使能、连续执行单轴/多轴指令。

双击根目录的 `启动源码.cmd` 启动界面。运行环境使用本项目的 `.venv`，不依赖临时工作目录。

首次使用或更换电脑时，安装 Python 3.11～3.13 x64 和 Npcap（启用 WinPcap API 兼容模式），然后在项目根目录执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

示例使用 Python 3.12；使用其他受支持版本时，请相应修改版本号。

文件：`app.py` 为界面，`core.py` 为控制核心，`cli.py` 为命令行入口，`requirements.txt` 为依赖清单。接线、操作步骤及支持范围见 [使用说明.md](使用说明.md)。

连续控制：扫描并选中电机，确认运行条件后开启“连续使能”。速度与加减速度无软件上下限，只接受正的有限数值；默认值仍为 60 rpm / 120 rpm/s。在“实时目标位置滑块”页拖动滑块，直接更新列表中所有勾选电机的共同目标偏移；也可使用各轴目标表格和“追加相对指令”。新指令不等待上一目标到位，累计角度不设上限。偶发 WKC 收包异常会保持原指令短时重试，持续异常仍停止。关闭开关、停止全部或 Esc 会停止、清空待更新目标并关闭使能。完整操作见 [使用说明.md](使用说明.md)。

`continuous_control.py` 提供独立的连续使能、最新目标更新和运动 API。界面不启动 UE/UDP 接口；此前的 UE 接入代码保留为后续接入参考。无硬件测试：

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_continuous_control test_app_continuous
```

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
