# Vision calibration tools

This package is for camera and hand-eye calibration programs. Calibration
results are durable robot configuration, not source code: reviewed, shared
results belong under the repository-level `calibration/vision/` directory.
Keep both separate from `eai_robot.arm.calibration`, which implements SCS215
joint endpoint calibration.

## Reserved modules

- The command layer should follow LeRobot's split: one small dispatcher selects
  a calibration workflow, while each workflow owns its capture/solve/validate
  procedure. Keep reusable algorithms in this package rather than adding
  calibration logic to the ArUco detector or the robot-motion controller.
- `intrinsics.py` — collect/read checkerboard image samples, solve camera
  intrinsics and distortion, report per-view and overall reprojection error.
- `hand_eye.py` — consume paired robot/tool and camera/target poses, solve the
  hand-eye transform, report residuals and frame conventions.
- `capture.py` — optional camera-only checkerboard capture helpers shared by
  the calibration commands. Any paired robot pose capture must remain read-only
  and must not command arm motion.

Register one `calibrate` command in `src/eai_robot/cli.py` when workflow
inputs and output schemas are agreed. It should dispatch to intrinsics or
hand-eye calibration, much as `lerobot-calibrate` dispatches to a selected
device's `calibrate()` implementation. Keep the workflow-specific procedures
in this package. Use the existing `aruco-preview` command for camera-only
marker detection in the meantime.

## Data and safety boundary

- Keep host camera indices in ignored `configs/hardware.local.toml` or pass
  them on the command line.
- Write raw images, paired pose samples, and candidate results under
  `local_artifacts/experiments/<timestamp>_vision-calibration/`.
- Promote approved camera/rig calibration into `calibration/vision/`, alongside
  the existing joint calibration assets but isolated from them. Keep candidate
  files and raw captures out of this shared runtime location.
- Record units, stable camera/rig identity, capture date, source commit, solver
  version, residuals, and transform direction with each approved result.
- Intrinsics are tied to the camera/lens/resolution configuration. Hand-eye
  calibration is tied to a specific camera-to-tool mounting configuration.
- A paired pose collector may read the robot and camera; it must not move the
  robot. Motion experiments require a separate, explicitly authorized workflow.
