# Shared course development baseline

The application repository is `EAI-Course-2026/eai_project`. Control, kinematics,
vision, scripts and tests are maintained here. LeRobot is a dependency fixed to
the collaborator fork's commit `6a077907c7989635218969ee78f5436f8faec92b` (package
version 0.6.2); the commit is essential because PyPI 0.6.2 is not this adaptation.
Do not edit installed LeRobot files or follow the old Conda/pip setup alongside
this environment.

## Install and verify on macOS and Windows

Use Python 3.12, **uv 0.11.7**, the root `pyproject.toml` and `uv.lock` on both
systems. The project and CI pin the uv version: the first clean hosted check
found that uv 0.12.22 wanted to rewrite this lock. Tool upgrades must therefore
be reviewed together with regenerated locks. To install the pinned version, use
the official [versioned installation instructions](https://docs.astral.sh/uv/getting-started/installation/#installing-a-specific-version).

```sh
uv sync --locked
uv run --no-sync python scripts/check_env.py
uv run --no-sync python -m unittest discover -s tests -q
```

Windows also has `scripts\setup_windows.cmd`. Existing Conda training environments
can remain on teammates' machines, but the canonical application commands use
uv's project environment. Copy `configs/hardware.example.toml` to
`configs/hardware.local.toml` and set the serial port; the local file is ignored.
Ports, camera indices and compute devices are machine-specific. Calibration is
shared by physical arm, independent of COM port or operating system.
Windows setup enables Python UTF-8 mode so Chinese diagnostics and paths are
consistent with Mac and CI.

## Migrated tools

```sh
uv run --no-sync eai-course --help
uv run --no-sync eai-course mapping
uv run --no-sync eai-course fk
uv run --no-sync eai-course plan --delta-mm 0 0 10
uv run --no-sync eai-course poses
```

These commands run offline. `python scripts/course.py COMMAND` is an equivalent
source-checkout entry point. `fk --hardware --port PORT` reads the arm without
enabling torque. `plan --hardware` reads the starting pose; `--execute` additionally
requires an interactive confirmation. `keyboard` and `vision --execute` retain
their explicit startup confirmations. `vision --preview` uses only the camera.
macOS interactive global keys may require Input Monitoring/Accessibility
permission; imports, tests, help and offline planning do not request it.

The common FK/IK backend is the migrated NumPy implementation on both systems.
Installing Placo no longer silently changes the default solver. The optional
Placo backend can be selected explicitly through `create_fk_solver(...,
backend="placo")`; it is outside this baseline's validated environment.

Task 1's reference poses retain their original calibration binding. Playback
converts through raw encoder targets so changing the current calibration does
not silently change those poses. `record-pose NAME --port PORT` saves to ignored
`configs/poses.local.json`, reads only, and requires torque already off. `poses
--poses configs/poses.local.json --enable --allow-wide-range --port PORT` starts
confirmed control through the existing ArmController. Stop commands are handled
between synchronous moves; they are not a replacement for an independent power
cutoff.

The wrist maintenance command `reindex` was retained. It can move the unloaded
shaft or change persistent limits only through its original confirmation
workflow. Saving requires `--calibration` pointing to a separate local copy;
the shared snapshot is never overwritten by that maintenance command.

## Calibration baseline

`calibration/scs215_so101.json` matches the user-confirmed manual calibration
written and read back on 2026-10-02: IDs 1–6 have limits `169..898`, `60..740`,
`196..772`, `67..681`, `47..974`, `239..640`. Wrist roll retains its previous
command range because the observed `6..1017` approaches both encoder boundaries.
The old EEPROM snapshot remains in `calibration/history/`. See
[CALIBRATION.md](CALIBRATION.md) for the sampling and write receipt.

Control and course entry points use the same 3-count feedback allowance.
Initial hold targets are clamped to calibrated limits; feedback allowance never
expands permitted commands. Natural rest is distinct from a Cartesian working
pose: keyboard/vision motion still checks IK margins before executing.

The existing raw demo home is retained, with its calibration binding updated.
This does not establish that its trajectories are physically validated under
the new limits. The 2026-09-28 inactive candidate remains historical; the new
2026-10-02 inactive candidate uses wrist `67..954`, within current EEPROM bounds.
Candidate verification selects the new files; neither candidate is the default.

The course Robot connects read-only, rejects calibration outside current stored
limits, and requires explicit motion enable. Torque release no longer unlocks
EEPROM; partial enable failures attempt release on every requested motor.
The GUI and migrated course tools still have separate process lifecycles: run
only one hardware control process at a time. A shared GUI/vision/voice action
scheduler remains the next integration task.

The inherited IK mapping assumes calibrated endpoints correspond to URDF
endpoints. Offline tests verify the mathematics, not physical zero alignment.
Recheck joint directions and physical FK alignment after mechanical or
calibration changes before Cartesian motion.

## Separate training environment

```sh
uv sync --locked --project environments/training
uv run --no-sync --project environments/training python -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

This uses `environments/training/.venv` and its own lock. The macOS/Windows
control environment uses native Mac wheels / Windows CPU wheels; the separate
training environment selects Windows/Linux CUDA 12.8 wheels and training
extras. It installs LeRobot's training tools rather than a second application
package. The LeRobot source commit is identical. GPU/driver compatibility and training jobs require
the team's Windows GPU machine; they cannot be accepted on this Mac.

## Source and collaboration

The migrated implementation originates from
[EAI-Course-2026/eai-course-lerobot](https://github.com/EAI-Course-2026/eai-course-lerobot/tree/6a077907c7989635218969ee78f5436f8faec92b/examples/eai_course),
including Week 4 poses, URDF FK, IK, planning, keyboard control, wrist maintenance,
black-ball detection and visual servo. Original source documents are retained
under `docs/course/source/` as historical material; their old paths, commands,
calibration statements and Windows-specific installation instructions are not
the current baseline. The source fork license is retained there.

Voice had documentation only; no working speech implementation existed to
migrate. Data recording, trained policies and end-to-end evaluation are also
not certified by this migration.

Work from the application repository's latest `main`, use a feature branch and
PR, and include offline results plus hardware evidence when applicable. Changes
to the LeRobot dependency must update the commit and lock in a dedicated PR.
macOS and Windows CI run the same offline suite. Do not force-push shared main.
