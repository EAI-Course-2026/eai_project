# Windows, CUDA and Conda setup

Use this guide with the application repository, `EAI-Course-2026/eai_project`.
The source framework is the course fork at
`6a077907c7989635218969ee78f5436f8faec92b` (LeRobot 0.6.2), not the PyPI
package with the same version number. Keep uv at **0.11.7** and Python at
**3.12**. Dependency changes go through the project files, lockfiles and a PR.

## Choose the environment

| Task | uv project / interpreter on Windows | Compute |
| --- | --- | --- |
| Joint control, browser panel, NumPy FK/IK, black-ball tracking, offline tests | Root / `.venv\Scripts\python.exe` | CPU PyTorch 2.11.0; these application tools do not require CUDA |
| LeRobot model training | `environments/training` / `environments\training\.venv\Scripts\python.exe` | PyTorch 2.11.0+cu128 and CUDA 12.8 on Windows/Linux |
| Mac development or experimental training | Corresponding project / `.venv/bin/python` | Native PyTorch 2.11.0; MPS when supported, no NVIDIA CUDA |

The two projects have separate lockfiles and `.venv` directories. The training
project installs LeRobot training tools; it does not install a second copy of
the application. The root `training` extra was removed: `uv sync --extra
training` is not a CUDA setup command. Linux training wheels are configured,
but Linux application/hardware acceptance is not part of the Mac/Windows CI.

## Windows control setup

Install Git for Windows and use a normal PowerShell terminal. Clone the
application repository, or update your existing checkout after the relevant PR
has merged. From its root, install the pinned uv version if needed:

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/0.11.7/install.ps1 | iex"
```

Open a new terminal if uv is not yet on PATH, then:

```powershell
uv --version
.\scripts\setup_windows.cmd
```

The installer creates the root environment, checks the fork source and runs
the complete offline application suite. It enables UTF-8 for Chinese paths
and diagnostics. Equivalent manual commands are:

```powershell
$env:PYTHONUTF8 = '1'
$env:PYTHONIOENCODING = 'utf-8'
uv sync --locked
uv run --no-sync python scripts/check_env.py
uv run --no-sync python -m unittest discover -s tests -q
```

Select `.venv\Scripts\python.exe` in your editor for application work. There
is no need to activate a virtual environment to use `uv run`.

## Windows NVIDIA training setup

Use a Windows x86-64 machine with a supported NVIDIA GPU and compatible NVIDIA
driver. First run `nvidia-smi` to record the GPU model and driver. Driver
compatibility is different from an installed CUDA Toolkit version; the current
project selects prebuilt CUDA 12.8 PyTorch wheels, not a local source build.
Consult the [NVIDIA compatibility guide](https://docs.nvidia.com/deploy/cuda-compatibility/minor-version-compatibility.html)
when choosing a driver. The actual GPU operation below is the local acceptance
check; do not infer compatibility from the version label alone.

```powershell
nvidia-smi
.\scripts\setup_training_windows.cmd
```

The training installer creates only `environments\training\.venv` and runs
the GPU check. Equivalent manual commands, from the repository root:

```powershell
uv sync --locked --project environments/training
uv run --no-sync --project environments/training python scripts/check_training_env.py --require-cuda
```

The checker verifies the exact fork and locked PyTorch build, reports the GPU
and supported architectures, and performs a small matrix multiplication on
`cuda:0`. It exits with an error if CUDA is unavailable or the operation fails.
It does not move the arm, open a camera, train a model or upload anything.

For installation-only verification on a machine without an NVIDIA GPU:

```powershell
uv run --no-sync --project environments/training python scripts/check_training_env.py
```

Alternatively run `.\scripts\setup_training_windows.cmd --software-only` to
install and check software together. This explicitly reports that GPU execution
is unverified. Windows CI uses this mode to test CUDA-wheel installation, fork provenance and `lerobot-train
--help`; hosted CI has no GPU and is not a GPU training result.

Example single-GPU ACT training command, with a real compatible LeRobot dataset
substituted for `TEAM/DATASET`:

```powershell
uv run --no-sync --project environments/training lerobot-train --dataset.repo_id=TEAM/DATASET --policy.type=act --policy.device=cuda --policy.push_to_hub=false --wandb.enable=false --eval_steps=0 --output_dir=outputs/train/act-first-run
```

Run this only after the GPU check passes. Dataset features, batch size, memory,
video decoding and task-specific training settings still need verification.
Use a new output directory for a new run. This example disables model uploads,
W&B reporting and evaluation, and does not invoke robot control. Select
`environments\training\.venv\Scripts\python.exe` in your editor for training.

## Existing Conda installations

Conda may stay installed, and existing Conda environments may be retained for
comparison. The team does not maintain a second independent Conda dependency
list: the two uv lockfiles define this baseline.

Prefer a fresh PowerShell terminal with no Conda environment activated. If your
terminal automatically activates Conda, run `conda deactivate` until the old
environment is no longer active, then use the commands above. Do not install
project packages with `conda install` or bare `pip` into either uv `.venv`.

If you keep a Conda shell, still use `uv sync --locked` / `uv run --no-sync`
for the chosen project. Do not use `--active`, `--system` or set
`UV_PROJECT_ENVIRONMENT` to a Conda path. Verify the interpreter explicitly:

```powershell
uv run --no-sync python -c "import sys; print(sys.executable)"
uv run --no-sync --project environments/training python -c "import sys; print(sys.executable)"
```

The first path must belong to root `.venv`; the second to the training `.venv`.
An old Conda environment reporting LeRobot 0.6.2 does not establish that it
contains this fork or these locked dependencies. See uv's
[project environment rules](https://docs.astral.sh/uv/concepts/projects/config/#project-environment-path).

## Windows serial ports, keyboard and preview

List serial ports before changing your machine-local configuration:

```powershell
uv run --no-sync python -m serial.tools.list_ports
Copy-Item configs/hardware.example.toml configs/hardware.local.toml
```

Set `[serial].port` to the detected `COM` port. Camera indices and ports belong
in local configuration or command arguments; calibration belongs to the
physical arm and is shared. Start with a read-only check:

```powershell
uv run --no-sync python scripts/arm_serial.py inspect
uv run --no-sync eai-course fk --hardware --port COM5
```

Replace `COM5` with the actual port. A successful software install is not a
hardware acceptance result. Use only one robot control process at a time.

Keyboard and preview tools need an interactive desktop. The vision preview
uses Tk/Pillow, so keep `opencv-python-headless`; do not install a competing
OpenCV wheel to obtain `cv2.imshow`. Check preview dependencies with:

```powershell
uv run --no-sync python -c "import tkinter; from PIL import Image, ImageTk; print('Preview imports OK')"
uv run --no-sync python -m tkinter
```

The second command opens a Tk demo window. Close it, then use
`uv run --no-sync eai-course vision --preview --camera-index 0` for camera-only
detection, replacing the camera index as needed. Omitting `--preview` retains
console detection. No `--execute` means the motor bus is not opened. Mac also
needs camera access and may need keyboard Input Monitoring/Accessibility;
offline imports/help do not require those permissions.

## Troubleshooting and acceptance

| Symptom | Check |
| --- | --- |
| uv version rejected | Install 0.11.7; do not regenerate locks with a different uv version as a setup workaround |
| `torch.version.cuda` is `None` on Windows | You selected the control/Conda interpreter; use the training project and re-sync its lock |
| CUDA wheel installed but CUDA unavailable | Check NVIDIA GPU/driver, interpreter and `nvidia-smi`; repeat `--require-cuda` |
| CUDA kernel/architecture error | Check GPU model/capability and the reported wheel architectures; select compatible wheels through a reviewed configuration/lock change |
| Tk missing or preview cannot open | Verify Tk with the selected interpreter and use a desktop session; console camera detection remains available |
| Serial access denied | Close other robot processes and check the detected COM port and adapter driver |
| FFmpeg missing | Offline control tests do not require the executable; verify video tools separately before recording/training on video datasets |

Acceptance proceeds separately: software installation and offline tests; GPU
tensor operation; a small real dataset training run; camera/keyboard and current
calibration checks on the actual Windows arm. None substitutes for the next.
