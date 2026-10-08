@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" goto missing_env
".venv\Scripts\python.exe" -B -c "import tkinter, pysoem, core" >nul 2>&1
if errorlevel 1 goto missing_dependencies
start "" ".venv\Scripts\pythonw.exe" -B "app.py"
exit /b 0

:missing_env
echo Please install Python 3.11-3.13 x64, then run in this folder:
echo   py -3.12 -m venv .venv
echo   .venv\Scripts\python.exe -m pip install -r requirements.txt
goto failed

:missing_dependencies
echo Python dependencies could not be loaded. Check Npcap and run:
echo   .venv\Scripts\python.exe -m pip install -r requirements.txt

:failed
pause
exit /b 1
