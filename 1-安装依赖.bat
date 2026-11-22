@echo off
rem =====================================================================
rem  Episode1 + MuJoCo upper computer -- Windows installer (run once)
rem  Creates .venv and installs mujoco / mink / numpy.
rem  Requires Python 3.10-3.13 (64-bit). If "py" is missing use "python".
rem =====================================================================
setlocal EnableExtensions
cd /d "%~dp0"

set "VENV_PY=%~dp0.venv\Scripts\python.exe"

echo.
echo [1/3] Looking for Python 3.12 / 3.13 / 3.11 / 3.10 / 3 ...
set "LAUNCHER="
where py >nul 2>nul && set "LAUNCHER=py"
for %%V in (3.12 3.13 3.11 3.10 3) do (
    if not defined FOUND (
        %LAUNCHER% -%%V -c "import sys" >nul 2>nul && (
            set "FOUND=%%V"
            set "PYARGS=-%%V"
        )
    )
)
if not defined FOUND (
    py -c "import sys" >nul 2>nul && (
        set "FOUND=default"
        set "PYARGS="
    )
)
if not defined FOUND (
    python -c "import sys" >nul 2>nul && (
        set "FOUND=python"
        set "PYARGS="
        set "LAUNCHER=python"
    )
)
if not defined FOUND goto :nopython
echo     using %LAUNCHER% %PYARGS%

echo.
echo [2/3] Building virtual environment .venv ...
if exist "%VENV_PY%" (
    echo     .venv already exists, reusing it
) else (
    %LAUNCHER% %PYARGS% -m venv .venv
    if errorlevel 1 goto :fail
)

echo.
echo [3/3] Installing dependencies (needs internet, about 100 MB) ...
"%VENV_PY%" -m pip install -U pip
"%VENV_PY%" -m pip install -r requirements.txt
if errorlevel 1 goto :fail

echo.
echo Verifying ...
"%VENV_PY%" -c "import mujoco, mink, numpy; print('    mujoco', mujoco.__version__, '| mink', getattr(mink, '__version__', 'ok'), '| numpy', numpy.__version__)"
if errorlevel 1 goto :fail

echo.
echo Done. Now double-click "2-启动上位机.bat".
pause
exit /b 0

:nopython
echo [ERROR] Python not found. Install Python 3.12 (64-bit) from python.org,
echo         and tick "Add python.exe to PATH" during setup.
pause
exit /b 1

:fail
echo.
echo [ERROR] Installation failed. See the messages above.
pause
exit /b 1
