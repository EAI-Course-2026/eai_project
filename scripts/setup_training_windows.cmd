@echo off
setlocal
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
set "GPU_CHECK=--require-cuda"
if /i "%~1"=="--software-only" set "GPU_CHECK="
if not "%~1"=="" if /i not "%~1"=="--software-only" (
    echo Usage: setup_training_windows.cmd [--software-only]
    exit /b 2
)
where uv >nul 2>nul
if errorlevel 1 (
    echo Install uv 0.11.7 first; see docs\environments.md.
    exit /b 1
)
cd /d "%~dp0.."
uv sync --locked --project environments/training
if errorlevel 1 exit /b 1
uv run --no-sync --project environments/training python scripts\check_training_env.py %GPU_CHECK%
if errorlevel 1 exit /b 1
if defined GPU_CHECK (
    echo CUDA training software and GPU check passed. No training job or hardware control was run.
) else (
    echo Training software installed. GPU execution and training remain unverified.
)
