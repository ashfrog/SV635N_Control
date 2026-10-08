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

连续控制：扫描并选中电机，设置速度、加减速度和累计角度限位，确认运行条件后开启“连续使能”。开启只使能并保持当前位置。双击列表的“目标偏移”设置各轴目标，点击“发送各轴目标”；也可选择方向和角度，点击“追加相对指令”。指令按顺序执行，每条完成后继续保持使能；“回到使能起点”发送零偏移目标。关闭开关、停止全部或 Esc 会停止、清空队列并关闭使能。完整操作见 [使用说明.md](使用说明.md)。

`continuous_control.py` 提供独立的连续使能、指令队列和运动 API。界面不启动 UE/UDP 接口；此前的 UE 接入代码保留为后续接入参考。无硬件测试：

```powershell
.\.venv\Scripts\python.exe -m unittest -v test_continuous_control test_app_continuous
```

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
