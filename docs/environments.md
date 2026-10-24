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

## Command policy on Windows and macOS

| Situation | Command | Behavior |
| --- | --- | --- |
| Everyday control, diagnostics and offline tests | `uv run --locked ...` | Checks the lock and synchronizes this project's environment; an outdated lock errors |
| First installation or after dependency/branch changes | `uv sync --locked` | Installs the committed baseline before hardware work |
| Training commands | `uv run --locked --project environments/training ...` | Uses only the separate training project and its lock |
| Installer/CI execution or a verified fixed hardware session | `uv run --no-sync ...` | Reuses the existing environment without synchronization |

`--locked` does not update the lockfile. `--no-sync` does not establish that an
existing environment matches newly pulled project files. Both platforms follow
the same policy; the selected project still determines CPU/native or CUDA wheels.
Before a hardware session, synchronize and check the environment; complete that
work before enabling motion. Torque-release commands may use `--no-sync` in the
already verified environment to avoid delaying release with an installation.

The installers and CI intentionally retain `uv sync --locked` followed by
`uv run --no-sync`: their install and execution stages are explicit. For a
fixed session, re-run setup when project files, branches or dependencies change.
See [uv's locking and syncing documentation](https://docs.astral.sh/uv/concepts/projects/sync/).

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
uv run --locked --project environments/training python scripts/check_training_env.py
```

Alternatively run `.\scripts\setup_training_windows.cmd --software-only` to
install and check software together. This explicitly reports that GPU execution
is unverified. Windows CI uses this mode to test CUDA-wheel installation, fork provenance and `lerobot-train
--help`; hosted CI has no GPU and is not a GPU training result.

Example single-GPU ACT training command, with a real compatible LeRobot dataset
substituted for `TEAM/DATASET`:

```powershell
uv run --locked --project environments/training lerobot-train --dataset.repo_id=TEAM/DATASET --policy.type=act --policy.device=cuda --policy.push_to_hub=false --wandb.enable=false --eval_steps=0 --output_dir=outputs/train/act-first-run
```

Run this only after the GPU check passes. Dataset features, batch size, memory,
video decoding and task-specific training settings still need verification.
Use a new output directory for a new run. This example disables model uploads,
W&B reporting and evaluation, and does not invoke robot control. Select
`environments\training\.venv\Scripts\python.exe` in your editor for training.

## Conda quick start

uv remains the primary workflow. The Conda alternative is a thin installer,
`scripts/setup_conda.py`, not another dependency resolver or an `environment.yml`
maintained in parallel. Conda supplies Python; pinned uv **0.11.7** installs the
same project and lockfile into the explicitly selected Conda prefix.

Open an initialized Conda terminal (Anaconda Prompt or a shell where `conda` and
`python` work), enter the repository root, and run this on macOS or Windows:

```sh
python scripts/setup_conda.py
```

This creates `eai-control` with Python 3.12 and pip from conda-forge, installs
pinned uv there, runs `uv sync --locked --inexact` against the root project,
checks dependencies and fork provenance, runs the offline suite, and finishes
with the fixed Agent demo. It does not open a motor bus, camera, microphone or
model API. Git and network access are needed for initial downloads.

The script uses a subprocess-scoped `UV_PROJECT_ENVIRONMENT` and an explicit
Python path. It does not export this variable into your shell or change the
repository `.venv`. `--inexact` preserves bootstrap packages such as Conda's
pip and uv while installing required project versions from the lock. Extra
packages are retained; a dependency check rejects conflicts. This is not a
byte-for-byte Conda environment lock, nor a guarantee that arbitrary existing
ML environments are compatible.

### Everyday Conda commands

```sh
conda activate eai-control
python scripts/check_env.py
python -m unittest discover -s tests -q
eai-course agent --demo
eai-course agent-config list
```

Alternatively, without activation:

```sh
conda run --no-capture-output -n eai-control eai-course agent --demo
conda run --no-capture-output -n eai-control python scripts/arm_desk.py
```

The desktop command starts the local panel; connecting/enabling hardware still
requires the existing operator flow. API configuration and explicit dotenv
loading are identical; replace the `uv run --locked` prefix in the
[provider guide](agent_providers.md) with `conda run --no-capture-output -n
 eai-control`, or use the activated environment directly. For example:

```sh
conda run --no-capture-output -n eai-control eai-course agent-config init --preset qwen --model YOUR_MODEL_ID --output configs/agent/qwen.local.toml
```

Do not run bare `uv run` expecting it to target Conda: normal uv commands still
use the repository `.venv`. Select the Conda interpreter in your editor and
verify `python -c "import sys; print(sys.executable)"` after activation.

### Updates, existing environments, and training

The default refuses to modify an existing environment, base/root, or a uv
`.venv`. Rerun with explicit reuse after pulling project/lock changes:

```sh
python scripts/setup_conda.py --reuse
```

For a pre-existing **dedicated project** environment:

```sh
python scripts/setup_conda.py --name my-eai-control --reuse
```

It must be a non-base Conda environment with Python 3.12. Reuse may replace
installed project dependency versions; use a fresh environment for unrelated
notebooks or experiments. Do not add a competing Conda/PyPI PyTorch or LeRobot
installation afterward. Do not globally point `UV_PROJECT_ENVIRONMENT` at Conda
or use `uv sync --active`; let the installer scope target selection explicitly.

Training uses a separate name and the existing training lock:

```sh
python scripts/setup_conda.py --kind training
conda run --no-capture-output -n eai-training python scripts/policy.py doctor
conda run --no-capture-output -n eai-training lerobot-train --help
```

This does not install a second application copy or run root Agent tests in the
training environment. To verify a real NVIDIA GPU, explicitly add
`--require-cuda` (and `--reuse` for an existing training environment). No training
job or motion is started. Never reuse the same Conda environment for control
and training. Their Windows CPU/CUDA wheel choices still come from the original
locks, with no manually exported requirements or duplicate index configuration.

`--skip-tests` skips the full **control** suite for a quick software smoke check;
it retains the dependency/environment check and offline demo. The normal quick
start runs the complete suite. A failed setup leaves its environment available
for diagnosis; fix the cause and explicitly reuse it, or choose a fresh name.

The adapter follows uv's [project environment path](https://docs.astral.sh/uv/concepts/projects/config/#project-environment-path)
and Conda's [run interface](https://docs.conda.io/projects/conda/en/latest/commands/run.html).
Bootstrap Python packages are selected by conda-forge; Python project dependency
versions and sources remain governed by the two committed uv locks.

## Windows serial ports, keyboard and preview

List serial ports before changing your machine-local configuration:

```powershell
uv run --locked python -m serial.tools.list_ports
Copy-Item configs/hardware.example.toml configs/hardware.local.toml
```

Set `[serial].port` to the detected `COM` port. Camera indices and ports belong
in local configuration or command arguments; calibration belongs to the
physical arm and is shared. Start with a read-only check:

```powershell
uv run --locked python scripts/arm_serial.py inspect
uv run --locked eai-course fk --hardware --port COM5
```

Replace `COM5` with the actual port. A successful software install is not a
hardware acceptance result. Use only one robot control process at a time.

Keyboard and preview tools need an interactive desktop. The vision preview
uses Tk/Pillow, so keep `opencv-python-headless`; do not install a competing
OpenCV wheel to obtain `cv2.imshow`. Check preview dependencies with:

```powershell
uv run --locked python -c "import tkinter; from PIL import Image, ImageTk; print('Preview imports OK')"
uv run --locked python -m tkinter
```

The second command opens a Tk demo window. Close it, then use
`uv run --locked eai-course vision --preview --camera-index 0` for camera-only
detection, replacing the camera index as needed. Omitting `--preview` retains
console detection. No `--execute` means the motor bus is not opened. Mac also
needs camera access and may need keyboard Input Monitoring/Accessibility;
offline imports/help do not require those permissions.

## Troubleshooting and acceptance

| Symptom | Check |
| --- | --- |
| uv version rejected | Install 0.11.7; do not regenerate locks with a different uv version as a setup workaround |
| `torch.version.cuda` is `None` on Windows | You selected the control interpreter; use the training uv project or eai-training Conda environment and re-sync its lock |
| CUDA wheel installed but CUDA unavailable | Check NVIDIA GPU/driver, interpreter and `nvidia-smi`; repeat `--require-cuda` |
| CUDA kernel/architecture error | Check GPU model/capability and the reported wheel architectures; select compatible wheels through a reviewed configuration/lock change |
| Tk missing or preview cannot open | Verify Tk with the selected interpreter and use a desktop session; console camera detection remains available |
| Serial access denied | Close other robot processes and check the detected COM port and adapter driver |
| FFmpeg missing | Offline control tests do not require the executable; verify video tools separately before recording/training on video datasets |

Acceptance proceeds separately: software installation and offline tests; GPU
tensor operation; a small real dataset training run; camera/keyboard and current
calibration checks on the actual Windows arm. None substitutes for the next.
