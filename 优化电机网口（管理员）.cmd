@echo off
set "SV635N_TASK_ROOT=%~dp0"
if not exist "%~dp0.venv\Scripts\pythonw.exe" (
  echo Python environment was not found. Use the launcher in the SVServer folder.
  pause
  exit /b 1
)
powershell.exe -NoProfile -Command "$taskRoot=$env:SV635N_TASK_ROOT; Start-Process -FilePath (Join-Path $taskRoot '.venv\Scripts\pythonw.exe') -ArgumentList @(([char]34+(Join-Path $taskRoot 'backend.py')+[char]34),'--optimize-nic') -WorkingDirectory $taskRoot -Verb RunAs -WindowStyle Hidden"
