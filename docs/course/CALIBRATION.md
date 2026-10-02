# Manual calibration — 2026-10-02

This receipt is shared hardware evidence, independent of the capture computer.
Host ports and camera indices belong only in ignored local configuration.
See [the shared contract](../../calibration/README.md).

The user placed the torque-off arm in its normal rest pose, then manually moved
all six joints through user-confirmed safe ranges. No powered trajectory was
performed. The fixed LeRobot dependency remains fork commit `6a07790` / 0.6.2.

The range capture recorded 2,240 samples over 237.192 seconds. Models were 1315,
IDs were 1–6, all torque/status registers were zero before and after, and EEPROM
limits were unchanged during sampling. No large encoder discontinuity was detected.

| Joint | Previous command limits | Observed manual travel | New command limits |
| --- | --- | --- | --- |
| shoulder_pan | 183..897 | 169..898 | 169..898 |
| shoulder_lift | 64..737 | 60..740 | 60..740 |
| elbow_flex | 196..771 | 196..772 | 196..772 |
| wrist_flex | 70..681 | 67..681 | 67..681 |
| wrist_roll | 47..974 | 6..1017 | 47..974 |
| gripper | 237..644 | 239..640 | 239..640 |

The user approved this conservative candidate and the persistent write. Five
joint ranges were updated, each with independent readback. Wrist roll was not
written. All six EEPROM locks read 1 afterward; all torque/status registers
remained zero. Previous software calibration and metadata are retained under
`calibration/history/20261002_before_manual_scs215_so101*`; the compact before/after
receipt is `calibration/history/20261002_manual_calibration_receipt.json`.
The 2,240 feedback samples are retained in
`calibration/history/20261002_manual_samples.csv`.

Natural-rest feedback before calibration was shoulder lift 62 and elbow 770..771.
The previous lower bound 64 caused the course entry point to reject shoulder lift.
The new calibration includes the sampled rest pose. Feedback allowance is now
shared at 3 counts by the arm controller, course enable and planner checks;
the initial hold is clamped to valid limits, and command limits never expand.
Larger discrepancies still reject enabling. Cartesian keyboard/vision control
still requires an interior working pose, independently of rest acceptance.

Existing reference-pose raw targets remain within the new ranges. The raw demo
home is unchanged and its binding is updated. Its shoulder entry radius changes
from 110 to 114 counts so the new lower endpoint (60) and its 3-count feedback
allowance are not rejected by the old home-entry envelope. The shoulder entry
is now 57..291; this is an input check, not an expanded motor command range. Manual range sampling does not
prove physical URDF zero/scale or collision-free powered trajectories; those
checks remain pending. Drive modes and zero homing offsets were preserved.

Validation before activation: 95 application tests passed, including four new
rest-feedback regressions. The persistent updater was simulated for success and
three injected failures; old ranges and locks were restored in each failure,
and it never wrote torque or goals. Final checks and rest results are recorded
in the calibration metadata and receipt.
