# EAI SCS215 Robot Project

Control a six-joint SO101-style arm built with Feetech SCS215 servos. This team repository combines servo setup, checked joint control, the browser panel, calibrated URDF FK/IK, Cartesian planning, keyboard control and black-ball visual servoing.

LeRobot is fixed to the team's **0.6.2 fork at commit `6a077907c7989635218969ee78f5436f8faec92b`**. The application stays in this repository; the fork supplies framework adaptation. The root uv environment is for control and tests, while `environments/training/` has its own environment and CUDA configuration. See the [shared course baseline](docs/course/README.md) for current Mac/Windows setup, migrated commands, calibration and remaining acceptance work. Recording, training and evaluation are not yet verified end to end for this arm.

## Requirements

- Python 3.12 and [uv 0.11.7](https://docs.astral.sh/uv/getting-started/installation/#installing-a-specific-version)
- A six-servo SCS215 arm with IDs 1–6 and a supported serial adapter
- A suitable servo power supply and a supported arm during motion

From the repository root:

```sh
uv sync --locked
uv run --locked python scripts/check_env.py
uv run --locked python -m unittest discover -s tests -q
```

Windows PowerShell users can run `.\scripts\setup_windows.cmd` for the same
control setup. NVIDIA training uses `.\scripts\setup_training_windows.cmd`
and a **separate** environment. Read the [Windows, CUDA and Conda guide](docs/environments.md)
for the pinned uv installer, driver/GPU verification, Conda coexistence, serial
port selection, camera preview and troubleshooting. Conda environments may be
kept, but the team's reproducible baseline is the two uv lockfiles.

| Use | Environment | Windows PyTorch | macOS PyTorch |
| --- | --- | --- | --- |
| Arm control, classical vision, tests | Root `.venv` | CPU 2.11.0 | Native 2.11.0; MPS available when supported |
| Model training | `environments/training/.venv` | 2.11.0+cu128 (CUDA 12.8) | Native 2.11.0; no NVIDIA CUDA |

Both environments install the exact course fork above, not PyPI LeRobot.
There is no root `training` extra: GPU training uses the separate project.

Copy `configs/hardware.example.toml` to `configs/hardware.local.toml` and set `[serial].port` for your machine. The local file is ignored by Git. Set the camera index in the same local file; there is no shared camera default. You can also pass `--port` before a subcommand. Use the device name reported by your OS, such as a `/dev/...` port on macOS/Linux or `COM5` on Windows; Windows hardware operation has not yet been verified.

## Running commands consistently

Use `uv run --locked ...` for everyday commands on both Windows and macOS.
It checks the lockfile and synchronizes the selected project environment without
updating the lockfile. After first checkout, pulling dependency changes or
switching branches, run `uv sync --locked` before starting hardware work.
An outdated lockfile is an error; resolve it through a reviewed dependency change.

`--no-sync` is reserved for execution after the environment has already been
synchronized and verified, such as installer/CI steps or repeated runs within
a fixed hardware session. It skips environment synchronization and can otherwise
leave a new checkout using old dependencies. See the
[command policy](docs/environments.md#command-policy-on-windows-and-macos).

## Inspect and control

Start with a read-only inspection of all six IDs, models, stored limits, and present positions:

```sh
uv run --locked python scripts/arm_serial.py inspect
uv run --locked python scripts/arm_lerobot.py inspect
```

Both entry points use the same calibration file and control logic. `arm_serial.py` uses the project's SCS protocol implementation; `arm_lerobot.py` uses its SCS215 adapter for the LeRobot Feetech motor bus. The six normalized inputs, each in `[0, 1]`, are ordered as follows:

```text
shoulder_pan  shoulder_lift  elbow_flex  wrist_flex  wrist_roll  gripper
```

Preview a target without opening the serial port or moving the arm:

```sh
uv run --locked python scripts/arm_serial.py control --allow-wide-range --dry-run --values 0.5 0.5 0.5 0.5 0.5 0.5
```

The checked-in calibration snapshot includes a wide-range wrist joint, which requires `--allow-wide-range` for control. It records this shared arm's user-confirmed safe manual ranges and EEPROM readback; powered trajectories and physical FK alignment remain unverified. A six-joint midpoint is not guaranteed to be a collision-free pose. Inspect and calibrate your own hardware before enabling motion. Read the [hardware notes](docs/hardware.md) and [shared calibration contract](calibration/README.md) for the distinction between hardware limits, software ranges, and unverified candidates.

For the local browser control panel:

```sh
uv run --locked python scripts/arm_desk.py
```

The panel supports either backend, position feedback, six joint sliders, a home sequence, demonstration motions, calibration capture, and torque release. Connecting is read-only; motion controls become available after the home sequence. See the [control-panel guide](docs/experiments/scs215_desk.md) and [demo behavior](docs/experiments/scs215_demos.md) before operating hardware.

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/eai_robot/arm/` | Shared calibration, motion checks, control, demonstrations, and web service |
| `src/eai_robot/hardware/` | Native SCS protocol and LeRobot SCS215 motor-bus adapter |
| `src/eai_robot/course/` | Migrated poses, FK/IK, planning, keyboard and visual servo implementation |
| `environments/training/` | Separate locked training environment, using the same fork commit |
| `scripts/` | Servo setup, command-line control, and browser-panel entry points |
| `experiments/servos/` | Single-servo, two-servo, and PID experiments |
| `calibration/` and `configs/` | Arm-specific calibration snapshots, demo poses, and configuration templates |
| `tests/` | Offline protocol, control, and web-service tests |

The [35-file migration inventory](docs/course/COVERAGE.md) maps every file under
the original `examples/eai_course/` to its implementation, asset, test or
historical document here. Voice was a design note in the source, not working
speech recognition. Migrated source coverage does not establish real-arm or
camera acceptance on Windows.

Current hardware validation covers ID/model reads, small synchronized moves with both backends, and motion initiated through the browser panel. Full mechanical travel, rated-load behavior, collision recovery from arbitrary starting poses, camera capture, and the LeRobot dataset pipeline remain open work. The [milestones](docs/milestones.md) track these separately. The project does not currently support `lerobot-record` for this arm.

The longer-term goal is to make the SCS215 implementation suitable for contribution to the LeRobot community, with a complete `Robot` interface, reproducible hardware tests, and a clearly chosen open-source license.
