# SV635N_Control

SV635N EtherCAT 电机调试工具（Python 源码版）。

双击根目录的 `启动源码.cmd` 启动界面。运行环境使用本项目的 `.venv`，不依赖临时工作目录。

首次使用或更换电脑时，安装 Python 3.11～3.13 x64 和 Npcap（启用 WinPcap API 兼容模式），然后在项目根目录执行：

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

示例使用 Python 3.12；使用其他受支持版本时，请相应修改版本号。

文件：`app.py` 为界面，`core.py` 为控制核心，`cli.py` 为命令行入口，`requirements.txt` 为依赖清单。接线、操作步骤及支持范围见 [使用说明.md](使用说明.md)。

`.venv/`、`logs/`、`settings.json` 和 Python 缓存均为本机运行数据，已加入 Git 忽略规则。`logs/` 和 `settings.json` 会在使用时生成。
