# Vision calibration artifacts

This is the repository location for reviewed, shared camera and hand-eye
calibration results. It is deliberately separate from both the calibration
programs in `src/eai_robot/calibration/` and the SCS215 joint
calibration files directly under `calibration/`.

Reserve these artifact families:

- `cameras/<camera_id>/intrinsics.json` — camera model, image dimensions,
  intrinsic matrix, distortion model and coefficients.
- `rigs/<rig_id>/hand_eye.json` — calibrated transform, explicit source and
  destination frames, units, and the camera/robot mounting identity it applies
  to.

Only the approved `intrinsics.json` and `hand_eye.json` result files are
allowlisted for Git. Keep raw images, pose samples, solver intermediates, and
unapproved candidates in ignored
`local_artifacts/experiments/<timestamp>_vision-calibration/` directories.
Each promoted result needs enough provenance and residual/error information to
judge whether it is valid for the current physical setup. Do not put host camera
indices, usernames, or absolute machine paths in shared files.

## Currently tracked

The collaborator-supplied results are tracked under the flat names that the
servo code and the offline tests load, because a clean checkout otherwise cannot
run either:

- `camera_intrinsics.json` — 640x480 pinhole, 8x6 chessboard with 19.6 mm
  squares, 19 accepted views, RMS 0.41 px, mean reprojection error 0.35 px.
- `hand_eye_result.json` — Park's method, 15 pose pairs, with the board-origin
  residual and consistency figures kept for judging validity.

These two predate the asset-id scheme above, so they are the one exception to
the `cameras/<id>/` and `rigs/<id>/` layout. Promoting them into it is a rename
plus an agreed id, not a recalibration; new results should go straight to the
structured paths.

The hand-eye file arrived with an absolute Windows path in
`preprocessing.source_dataset`. It now reads as a description of the sample set,
and neither file contains a machine path.

The exact schema and asset IDs should be agreed with the calibration
collaborator before the first result is promoted. A resolution, lens, camera,
or mounting change may invalidate the corresponding result.
