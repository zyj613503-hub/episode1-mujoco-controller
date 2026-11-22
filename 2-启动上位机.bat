@echo off
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
set "PYTHONUTF8=1"

if not exist "%PYTHON%" (
    echo [ERROR] Python virtual environment not found.
    echo.
    echo Create it from Command Prompt in this folder:
    echo   py -3.12 -m venv .venv
    echo   .venv\Scripts\python.exe -m pip install -U pip
    echo   .venv\Scripts\python.exe -m pip install -r requirements.txt
    echo.
    echo If "py" is unavailable, replace it with "python".
    pause
    exit /b 1
)

powershell -NoProfile -ExecutionPolicy Bypass -Command "try { $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 1 '%HEALTH_URL%'; if ($r.StatusCode -eq 200) { exit 0 } } catch {}; exit 1"
if not errorlevel 1 (
    echo Upper computer is already running: %BASE_URL%
    echo The existing service will keep running after this window closes.
    start "" "%BASE_URL%"
    pause
    exit /b 0
)

echo Starting upper computer: %BASE_URL%
start "" /b powershell -NoProfile -WindowStyle Hidden -Command "$u = '%BASE_URL%'; $h = '%HEALTH_URL%'; for ($i = 0; $i -lt 60; $i++) { try { $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 1 $h; if ($r.StatusCode -eq 200) { break } } catch {}; Start-Sleep -Milliseconds 500 }; Start-Process $u"
"%PYTHON%" "%APP%"
set "EXIT_CODE=%ERRORLEVEL%"

echo.
if not "%EXIT_CODE%"=="0" echo Upper computer stopped with exit code %EXIT_CODE%.
pause
exit /b %EXIT_CODE%
