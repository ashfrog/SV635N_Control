@echo off
setlocal
cd /d "%~dp0.."
if exist ".venv\Scripts\pythonw.exe" (
  start "" ".venv\Scripts\pythonw.exe" -m UdpControl
) else (
  start "" pyw -3 -m UdpControl
)
