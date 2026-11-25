@echo off
setlocal EnableExtensions
title ADASim Launcher

rem ============================================================
rem  ADASim one-click launcher: base station + algorithm process
rem  Default scenario: following_lead.json
rem  Drag a .json onto this file to run a different scenario
rem ============================================================

rem ---- 1. Locate project root (this bat's directory) ----
cd /d "%~dp0"

rem ---- 2. Qt runtime DLLs (edit if your Qt is elsewhere) ----
set "QT_BIN=D:\DownLoad\Qt\6.12.0\mingw_64\bin"
set "MINGW_BIN=D:\DownLoad\Qt\Tools\mingw1310_64\bin"
set "PATH=%QT_BIN%;%MINGW_BIN%;%PATH%"

rem ---- 3. Python (prefer workbuddy bundled, fall back to PATH) ----
set "PY=python"
if exist "C:\Users\26468\.workbuddy\binaries\python\versions\3.13.12\python.exe" (
    set "PY=C:\Users\26468\.workbuddy\binaries\python\versions\3.13.12\python.exe"
)

rem ---- 4. Scenario: use dragged file if any, else default ----
set "SCENE=scenarios\following_lead.json"
if not "%~1"=="" set "SCENE=%~1"

rem ---- 5. Kill leftover processes to free port 45100 ----
echo [START] cleaning up leftover processes...
taskkill /F /IM adasim.exe   >nul 2>&1
taskkill /F /IM python.exe   >nul 2>&1
ping -n 2 127.0.0.1 >nul

rem ---- 6. Launch base station (separate window, server must listen first) ----
echo [START] base station: %SCENE%
start "ADASim Base" build\qt\bin\adasim.exe "%SCENE%"

rem ---- 7. Wait for base station to listen on port 45100 (up to 15s) ----
echo [START] waiting for base station...
set "READY=0"
for /l %%i in (1,1,15) do (
    netstat -ano | findstr ":45100" | findstr "LISTENING" >nul 2>&1
    if not errorlevel 1 (
        set "READY=1"
        goto :base_ready
    )
    ping -n 2 127.0.0.1 >nul
)
:base_ready
if "%READY%"=="0" (
    echo [WARN] port 45100 not listening after 15s, still trying to launch algorithm.
    echo        If the algorithm window reports connection failure, check the base window.
)

rem ---- 8. Launch algorithm process (separate window, cruise 11 m/s, keep running) ----
echo [START] algorithm process (cruise 11 m/s)
start "ADASim Algorithm" "%PY%" python\run.py --cruise 11

echo.
echo Done. Two windows opened:
echo   Base window       - simulation view (HUD shows algorithm online state)
echo   Algorithm window  - prints status every second; close it to stop
echo.
echo To stop: close both windows, or double-click this file again to clean and restart.
echo.
pause
