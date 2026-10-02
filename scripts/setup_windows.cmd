@echo off
setlocal
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
where uv >nul 2>nul
if errorlevel 1 (
    echo Install uv 0.11.7 first; see docs\environments.md.
    exit /b 1
)
cd /d "%~dp0.."
uv sync --locked
if errorlevel 1 exit /b 1
uv run --no-sync python scripts\check_env.py
if errorlevel 1 exit /b 1
uv run --no-sync python -m unittest discover -s tests -q
if errorlevel 1 exit /b 1
echo Shared control environment and offline tests are ready.
