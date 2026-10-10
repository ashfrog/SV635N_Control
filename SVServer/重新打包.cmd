@echo off
cd /d "%~dp0.."
if not exist ".venv\Scripts\python.exe" (
  echo Please create the project Python x64 environment first.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" -m pip install -r requirements-build.txt
if errorlevel 1 goto failed
".venv\Scripts\python.exe" build_svserver.py
if errorlevel 1 goto failed
exit /b 0
:failed
pause
exit /b 1
