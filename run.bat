@echo off
setlocal EnableExtensions
chcp 65001 >nul
title ADASim 启动器

rem ============================================================
rem  ADASim 一键启动：双击运行，自动起底座 + 算法
rem  默认场景 following_lead.json，可拖一个 .json 场景到本文件上
rem ============================================================

rem ---- 1. 定位项目根目录（本批处理所在目录） ----
cd /d "%~dp0"

rem ---- 2. Qt 运行库路径（找不到 DLL 就在这里改） ----
set "QT_BIN=D:\DownLoad\Qt\6.12.0\mingw_64\bin"
set "MINGW_BIN=D:\DownLoad\Qt\Tools\mingw1310_64\bin"
set "PATH=%QT_BIN%;%MINGW_BIN%;%PATH%"

rem ---- 3. Python（优先用工作目录里的，其次 PATH 里的） ----
set "PY=python"
if exist "C:\Users\26468\.workbuddy\binaries\python\versions\3.13.12\python.exe" (
    set "PY=C:\Users\26468\.workbuddy\binaries\python\versions\3.13.12\python.exe"
)

rem ---- 4. 场景：优先取拖进来的文件，否则用默认跟驰场景 ----
set "SCENE=scenarios\following_lead.json"
if not "%~1"=="" set "SCENE=%~1"

rem ---- 5. 清理残留进程，避免端口 45100 被旧实例占用 ----
echo [启动] 清理残留的旧进程...
taskkill /F /IM adasim.exe   >nul 2>&1
taskkill /F /IM python.exe   >nul 2>&1
timeout /t 1 /nobreak >nul

rem ---- 6. 起底座（独立窗口，服务端必须先监听） ----
echo [启动] 底座：%SCENE%
start "ADASim 底座" build\qt\bin\adasim.exe "%SCENE%"

rem ---- 7. 等底座就绪：轮询端口 45100，最多等 15 秒 ----
echo [启动] 等待底座就绪...
set /a "READY=0"
for /l %%i in (1,1,15) do (
    netstat -ano | findstr ":45100" | findstr "LISTENING" >nul 2>&1
    if not errorlevel 1 (
        set /a "READY=1"
        goto :base_ready
    )
    timeout /t 1 /nobreak >nul
)
:base_ready
if "%READY%"=="0" (
    echo [警告] 15 秒内未检测到底座监听端口，仍会尝试启动算法。
    echo        若算法窗口提示连接失败，请先检查底座窗口是否正常打开。
)

rem ---- 8. 起算法进程（独立窗口，默认巡航 11 m/s，持续运行） ----
echo [启动] 算法进程（巡航 11 m/s，持续运行）
start "ADASim 算法" "%PY%" python\run.py --cruise 11

echo.
echo 完成。两个窗口已打开：
echo   底座窗口  - 仿真画面（左上角 HUD 显示算法在线状态）
echo   算法窗口  - 每秒打印状态行，Ctrl+C 或关闭窗口即退出
echo.
echo 停止：关闭两个窗口，或再次双击本文件会自动清理并重启。
echo.
pause