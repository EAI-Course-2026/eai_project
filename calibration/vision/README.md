# Vision calibration artifacts

This is the repository location for reviewed, shared camera and hand-eye
calibration results. It is deliberately separate from both the calibration
programs in `src/eai_robot/vision/calibration/` and the SCS215 joint
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

The exact schema and asset IDs should be agreed with the calibration
collaborator before the first result is promoted. A resolution, lens, camera,
or mounting change may invalidate the corresponding result.
