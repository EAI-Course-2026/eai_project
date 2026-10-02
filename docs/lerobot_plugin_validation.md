# SCS215 plugin validation — 2026-10-02

Implementation lives in the independent `eai_project` repository. The installed
LeRobot remains version 0.6.2 from course fork commit
`6a077907c7989635218969ee78f5436f8faec92b`. No framework/fork source or active
calibration data was changed by this integration.

## Software checks on macOS arm64 / Python 3.12.13

- Root and separate training environments installed with uv 0.11.7 and `--locked`.
- Both locks only add the local plugin; existing locked dependency versions and
  native/CPU/CUDA source selections remain unchanged.
- 125 offline unittest cases pass, including prior protocol, homing, calibration,
  GUI, kinematics and vision regression cases.
- Fresh process discovers the plugin from installed distribution metadata and
  the real dynamic Robot factory resolves `scs215_so101_follower`.
- Standard calibration and teleoperation entry points run with simulated motors.
- Standard recording entry point runs its actual loop with simulated motors,
  leader and RGB camera, writes one local episode and checks six action features,
  RGB shape, episode metadata and Parquet files. No Hub upload is requested.
- Tests cover read-only preservation of external torque, startup goal bounds,
  cancellation/backup of software calibration, camera failure cleanup, rejection
  of excessive relative steps, and releasing only the enabled joint subset.
- Control software check and separate training software check pass on this Mac.
- Application and plugin wheels build. CUDA GPU execution is not claimed here.

## Real arm, read-only

Initial attempts received corrupt packets with both SDK and independent native
protocol readers. After the user power-cycled the arm, the plugin connected and
read six matching SCS215 model/ID pairs. Before/after comparisons verified that
ID, EEPROM limits, torque, lock and status remained unchanged.

| ID | Joint | EEPROM limits | Position | Torque | Lock | Status |
|---|---|---|---|---|---|---|
| 1 | shoulder_pan | 169..898 | 501 | 0 | 1 | 0 |
| 2 | shoulder_lift | 60..740 | 62 | 0 | 1 | 0 |
| 3 | elbow_flex | 196..772 | 773 | 0 | 1 | 0 |
| 4 | wrist_flex | 67..681 | 618 | 0 | 1 | 0 |
| 5 | wrist_roll | 47..974 | 237 | 0 | 1 | 0 |
| 6 | gripper | 239..640 | 249 | 0 | 1 | 0 |

The plugin accepted this observation. Elbow feedback is one count above the
stored limit and fits the existing three-count feedback allowance; command
bounds remain 196..772. This does not establish collision clearance or validate
every possible resting posture.

The installed standard `lerobot-calibrate` executable also loaded the plugin
and verified the shared calibration after ENTER, without EEPROM writes. No new
manual calibration, powered motion, torque enable or actual camera capture was
performed for this integration.

## Remaining acceptance

Cross-platform hosted CI checks software, including the Windows CUDA installation
without a GPU. It cannot validate Windows robot/camera access or CUDA execution.
Next hardware acceptance needs a compatible operator, short real synchronized
camera/action recording, dataset inspection, GPU training and controlled policy
playback. FK/IK physical alignment and arbitrary-pose collision recovery remain
separate milestones.
