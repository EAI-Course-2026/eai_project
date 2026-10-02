# SCS215 plugin: direct scripts and standard LeRobot CLI

Run from the repository root with uv 0.11.7. `uv sync --locked` installs the
application, editable plugin and exact course fork `6a077907c7989635218969ee78f5436f8faec92b`.
The separate CUDA project installs the same plugin with its own lock and wheel
selection. Do not replace LeRobot with same-version PyPI software or edit site-packages.
The distribution/import name `lerobot_robot_scs215` retains underscores for this
fork's discovery mechanism. The config type is `scs215_so101_follower`; built-in
`so101_follower` selects the old fork class.

## Direct scripts

```sh
uv run --no-sync python scripts/arm_lerobot.py --port /dev/cu.usbmodem5B910441861 inspect
uv run --no-sync python scripts/arm_lerobot.py control --allow-wide-range --dry-run --values 0.5 0.5 0.5 0.5 0.5 0.5
uv run --no-sync python scripts/arm_desk.py
```

Direct scripts, default web/demo backend and course pose tools share the plugin
bus. `ArmController` keeps the checked motion, homing and 0..1 interface. Course
FK/IK, keyboard and vision use the plugin Robot through compatibility aliases.
Native `arm_serial.py` remains a diagnostics/reference path.

## Standard calibration

macOS/Linux example; Windows accepts the same flags on one line with its local COM port:

```sh
uv run --no-sync lerobot-calibrate \
  --robot.type=scs215_so101_follower \
  --robot.port=/dev/cu.usbmodem5B910441861 \
  --robot.id=scs215_so101 \
  --robot.calibration_path=calibration/scs215_so101.json
```

ENTER verifies that current software ranges fit connected EEPROM limits. To
collect new ranges, type `c`, support the torque-off arm, start sampling, move each
joint slowly through safe travel, and press ENTER to stop. Candidate ranges are
intersected with EEPROM limits. `SAVE` replaces software JSON and backs up old
JSON/metadata; cancellation leaves them unchanged. Already-active torque requires
explicit `RELEASE` before sampling. No EEPROM limits or homing offsets are written;
software calibration cannot extend hardware limits. Experiments should use a
separate local calibration path. New ranges invalidate the old demo home binding
and require FK/IK physical alignment checks.

## Standard teleoperation and recording

A compatible, separately calibrated teleoperator is required. A physical leader
has not been validated for this project. This example uses a compatible SO101
leader with normalized output: **`--teleop.use_degrees=false` is essential**, because
the framework leader defaults to degrees. Project Cartesian keyboard control
remains `eai-course keyboard`; generic framework keyboard actions are not six
joint-position actions.

```sh
uv run --no-sync lerobot-teleoperate \
  --robot.type=scs215_so101_follower \
  --robot.port=FOLLOWER_PORT \
  --robot.calibration_path=calibration/scs215_so101.json \
  --robot.enable_motion=true \
  --robot.max_relative_target=5 \
  --teleop.type=so101_leader \
  --teleop.port=LEADER_PORT \
  --teleop.id=calibrated_leader \
  --teleop.use_degrees=false \
  --fps=15
```

Bring arms to corresponding starting positions first. Relative limits use
normalized units and reject excessive steps without interpolation or silent
clipping. Standard CLI exit releases this Robot's torque. Without the explicit
enable flag, connection is read-only and action attempts raise an error.
Calibration CLI stays torque-off even when the flag is supplied.

For a small local recording, use the same Robot/teleoperator flags above with
`lerobot-record`, replace `--fps=15` with these dataset/camera arguments:

```text
--robot.cameras='{front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 15}}'
--dataset.repo_id=local/scs215_pilot
--dataset.root=outputs/scs215_pilot
--dataset.single_task='pilot task'
--dataset.fps=15
--dataset.num_episodes=1
--dataset.episode_time_s=10
--dataset.reset_time_s=0
--dataset.push_to_hub=false
--dataset.video=false
--play_sounds=false
```

Index `1` is this Mac's example, not a portable camera default. Resolve it per host
and configure RGB dimensions. The pilot stores local images (`video=false`) and
never uploads to the Hub. PowerShell can use one command line with single quotes
around the camera dictionary. Standard framework commands require an appropriate
operator; this integration does not invent a leader device for the existing arm.

Tests run real standard entry points and loops with simulated devices, including
a real local RGB episode. This verifies framework integration and cleanup, not
physical synchronization, latency, leader compatibility, collision clearance,
CUDA training or policy playback. Only one process may own the serial device.

## Package boundary

The plugin is a separate installable package in the same independent Git repo.
It never imports `eai_robot`, so the Robot/bus can run without the web application.
Both locked projects install it locally and retain the exact fork source. The
wheel's version requirement alone cannot prove fork provenance: use the locked
project environments. Plugin installation modifies no fork source files.
