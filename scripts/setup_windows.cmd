@echo off
setlocal
where uv >nul 2>nul
if errorlevel 1 (
    echo Install uv first: https://docs.astral.sh/uv/getting-started/installation/
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
