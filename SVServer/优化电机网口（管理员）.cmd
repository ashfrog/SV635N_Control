@echo off
set "SV635N_TASK_ROOT=%~dp0"
powershell.exe -NoProfile -Command "$taskRoot=$env:SV635N_TASK_ROOT; Start-Process -FilePath (Join-Path $taskRoot 'SVServer.exe') -ArgumentList '--optimize-nic' -WorkingDirectory $taskRoot -Verb RunAs -WindowStyle Hidden"
