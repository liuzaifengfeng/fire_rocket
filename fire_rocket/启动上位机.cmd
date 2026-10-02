@echo off
setlocal
set "FR_PY="
for %%V in (313 312 311 310) do if not defined FR_PY if exist "%LOCALAPPDATA%\Programs\Python\Python%%V\python.exe" (
  "%LOCALAPPDATA%\Programs\Python\Python%%V\python.exe" -c "import tkinter,serial; tkinter.Tcl()" >nul 2>&1
  if not errorlevel 1 set "FR_PY=%LOCALAPPDATA%\Programs\Python\Python%%V\python.exe"
)
if not defined FR_PY (
  echo Python with tkinter and pyserial was not found.
  echo Install Python 3.10+ with Tcl/Tk, then run: python -m pip install pyserial
  pause
  exit /b 1
)
start "" "%FR_PY:python.exe=pythonw.exe%" "%~dp0upper_computer.py"
endlocal
