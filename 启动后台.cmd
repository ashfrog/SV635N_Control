@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" (
  echo Please create the project Python environment first. See README.md.
  pause
  exit /b 1
)
start "" ".venv\Scripts\pythonw.exe" backend.py
