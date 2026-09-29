@echo off
set "PYTHONW=C:\Users\Julian\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\pythonw.exe"
if not exist "%PYTHONW%" (
  echo Python runtime was not found at %PYTHONW%
  pause
  exit /b 1
)
start "W11 USB Camera" "%PYTHONW%" "%~dp0usb_camera_viewer.py" %*
