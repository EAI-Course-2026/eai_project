# ArUco visual servoing scaffold

This is the software foundation for the Week 6 image-based visual-servo task.
It is intentionally split into camera observations, pure image-space math,
and a future robot adapter. The preview command opens only the camera; no part
of this scaffold connects to or moves the arm.

## Current modules

- `src/eai_robot/vision/aruco.py` detects a configured marker and can
  estimate its pose from supplied intrinsics and physical marker size.
- Camera and hand-eye calibration belong to the independent
  [calibration capability](../../src/eai_robot/calibration/README.md).
  `vision/calibration/` is retained as the earlier placeholder; neither location
  currently contains camera or hand-eye calibration algorithms.
- `eai-course aruco-preview --camera-index N --marker-id ID` shows a
  camera-only detection preview. It does not connect to the robot.
- `src/eai_robot/control/aruco_ibvs.py` builds the four-corner image
  error, point-feature interaction matrix, and a bounded damped least-squares
  camera-frame velocity command.
- Add offline math and detector regression coverage before connecting this
  scaffold to a robot adapter.

The camera velocity order is `[vx, vy, vz, wx, wy, wz]` in the camera frame.
Marker corners follow OpenCV's top-left, top-right, bottom-right, bottom-left
ordering. The solver returns a camera-frame velocity only; mapping that into
the end-effector/base frame and then into joint targets remains a separate,
reviewed adapter step.

## Intended pipeline

1. Capture an RGB image and identify the configured ArUco dictionary and ID.
2. Use the collaborator-provided camera intrinsics and the printed marker size
   to estimate the marker pose and corner depths.
3. Compare the four detected corners with four configured desired image points.
4. Form the interaction matrix and compute a damped, norm-limited camera twist.
5. Preview the proposed twist and image error before adding any robot adapter.
6. Add hand-eye conversion and bounded Cartesian/IK execution only after the
   calibration files, frame convention, limits and on-hardware validation are
   agreed and reviewed.

Marker ID selection is explicit: if no ID is configured, a frame containing
multiple markers is rejected as ambiguous. Missing markers return no
observation; the eventual runtime must stop and hold rather than reuse stale
error or motion commands.

## Pending integration decisions

- The camera calibration JSON schema and camera asset identifier are being
  prepared separately. The math API accepts `camera_matrix` and `distortion`
  directly so it does not assume that schema.
- Marker dictionary, ID, printed side length and desired corner layout must be
  recorded in a portable configuration, without committing local camera index.
- Hand-eye output must state the transform direction and coordinate frames.
  The robot adapter must convert the camera-frame command using the reviewed
  transform and then use the existing Cartesian planner / IK safety path.
- Raw images, local camera settings and experiment outputs stay in ignored
  local-artifact areas. Only a reviewed, reproducible calibration result should
  be promoted into shared `calibration/` data.

No hardware execution is provided by this scaffold.
