@echo off
rem =====================================================================
rem  Episode1 + MuJoCo upper computer -- Windows launcher
rem  Double-click this file. The service runs in THIS window (keep it open).
rem  The browser opens automatically once http://127.0.0.1:<PORT>/api/health answers.
rem  Optional: set UPPER_PORT=8780 to use another port.
rem =====================================================================
setlocal EnableExtensions
chcp 65001 >nul
cd /d "%~dp0"

set "ROOT=%CD%"
set "PYTHON=%ROOT%\.venv\Scripts\python.exe"
set "APP=%ROOT%\运行器\server\app.py"

if not defined UPPER_PORT set "UPPER_PORT=8770"
set "PORT=%UPPER_PORT%"
set "BASE_URL=http://127.0.0.1:%PORT%/v2/base"
set "HEALTH_URL=http://127.0.0.1:%PORT%/api/health"
rem Keep every child python in UTF-8 mode (console + pipes), otherwise Chinese
rem output can raise UnicodeEncodeError on a cp936 console.
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"

if not exist "%PYTHON%" (
    echo [ERROR] .venv not found. Run "1-安装依赖.bat" first.
    pause
    exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -Command "try { $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 1 '%HEALTH_URL%'; if ($r.StatusCode -eq 200) { exit 0 } } catch {}; exit 1"
if not errorlevel 1 (
    echo Service is already running on port %PORT%
    start "" "%BASE_URL%"
    exit /b 0
)

echo Starting upper computer on port %PORT% ...
echo (Keep this window open. Close it or press Ctrl+C to stop the service.)
echo.
start "" /b powershell -NoProfile -WindowStyle Hidden -Command "$h = '%HEALTH_URL%'; $u = '%BASE_URL%'; for ($i = 0; $i -lt 90; $i++) { try { $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 1 $h; if ($r.StatusCode -eq 200) { break } } catch {}; Start-Sleep -Milliseconds 500 }; Start-Process $u"

"%PYTHON%" "%APP%"
set "EXIT_CODE=%ERRORLEVEL%"

echo.
if not "%EXIT_CODE%"=="0" echo Upper computer stopped with exit code %EXIT_CODE%.
pause
exit /b %EXIT_CODE%
