> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# Week4 Task 2

This directory contains all six implementation steps.

Current scope: position-only IK for joints 1-5. End-effector orientation is not
constrained, and keyboard Cartesian control does not command gripper ID 6. See
the project [`HANDOFF.md`](../../HANDOFF.md) before adding vision or voice input.

## Step 1: calibration to URDF angles

`joint_mapping.py` converts between three representations:

1. SCS215 raw encoder values (`0..1023`).
2. LeRobot values (`-100..100` for arm joints and `0..100` for the gripper).
3. SO-101 URDF joint angles in degrees or radians.

The five arm joints use limits copied from the official
[`so101_new_calib.urdf`](https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf).
The sixth `gripper` joint is kept separate from the IK chain.

Run the offline report from the active `lerobot` Conda environment:

```cmd
cd /d "<仓库目录>\examples\eai_course\week4\task2"
python check_step1.py
```

Run the unit tests:

```cmd
python -m unittest -v test_joint_mapping.py
```

Neither command opens `COM5` or moves the arm.

## Mapping assumption

The mapping assumes that each calibrated minimum/maximum corresponds to the
matching URDF lower/upper limit. Joint direction and zero alignment still need
to be verified by forward kinematics in step 2.

If a teammate's `wrist_roll` calibration touches either end of the SCS215
encoder range, the checker reports a probable `1023 -> 0` boundary crossing.
Do not use that range for IK motion until that arm has been re-indexed.

## Re-index wrist_roll (ID 5, only when reported)

Run these commands one at a time. Close every other program that may be using
`COM5` first.

Read ID 5 without changing torque or moving it:

```cmd
python reindex_id5.py inspect
```

Power off the arm, detach the wrist horn/linkage from the ID 5 output shaft,
restore power, and center the unloaded shaft:

```cmd
python reindex_id5.py center
```

The program requires the exact confirmation `CENTER ID5 TO 512`. It moves only
ID 5 at low speed and disables torque when finished. Reattach the wrist in its
physical neutral pose without rotating the centered shaft.

Record the new safe range:

```cmd
python reindex_id5.py calibrate
```

Slowly move only `wrist_roll` through the range needed for the assignment and
press Enter. The script rejects a `1023/0` crossing or a range within 32 counts
of either boundary. After a second exact confirmation it will:

1. Back up the existing LeRobot calibration JSON.
2. Write the new ID 5 minimum and maximum to the servo.
3. Update only `wrist_roll` in `scs215_com5.json`.

Finally, rerun the offline checker:

```cmd
python check_step1.py
```

## Step 2: forward kinematics

`so101_new_calib.urdf` uses the official SO-101 URDF geometry and joint limits.
`kinematics_backend.py` uses LeRobot's `RobotKinematics` when Placo is
available. Native Windows currently has no Placo installation, so it selects
the compatible NumPy implementation in `urdf_fk.py`.

Run FK for the URDF zero pose without opening COM5:

```cmd
python check_step2_fk.py
```

Read the current five arm joints through LeRobot and calculate the TCP pose:

```cmd
python check_step2_fk.py --hardware
```

Continuously display joint angles and TCP position:

```cmd
python check_step2_fk.py --watch
```

Hardware modes only read motor positions. They do not change torque or send a
goal position. If the arm already has torque disabled and is mechanically
supported, `--watch` can be used while moving one joint slowly by hand. Compare
the observed Cartesian direction with the positive 5-degree predictions shown
by the one-shot hardware command.

Run all tests implemented so far:

```cmd
python -m unittest -v test_joint_mapping.py test_urdf_fk.py
```

## Steps 3-5: IK, straight lines, and unreachable targets

`position_ik.py` implements position-only IK. It uses LeRobot/Placo when that
backend is available and a damped-least-squares NumPy solver on native Windows.
Every result is checked by running FK and measuring the Cartesian residual.

`cartesian_planner.py` samples a Cartesian line at 3 mm intervals and solves
each waypoint from the previous solution. If a waypoint has no valid IK, it
prints `IK 无解` and uses binary search to retain the farthest feasible point
in the original movement direction.

Offline example, moving the TCP upward by 20 mm:

```cmd
python run_steps3_to5.py --delta-mm 0 0 20
```

Offline unreachable-target example:

```cmd
python run_steps3_to5.py --target-mm 1000 0 1000
```

Use the real arm's current pose as the start, but only calculate the path:

```cmd
python run_steps3_to5.py --hardware --delta-mm 0 0 10
```

Actual movement requires the additional `--execute` flag and an exact
confirmation shown by the program:

```cmd
python run_steps3_to5.py --hardware --delta-mm 0 0 10 --execute
```

Execution seeds every servo goal with its measured position before enabling
torque, limits each Cartesian waypoint to 3 mm and each joint step to 8 degrees,
streams intermediate waypoints every 0.1 seconds without stopping, waits for
feedback only at the final waypoint, and disables torque after the user supports
the arm and presses Enter. The approximate Cartesian command speed is
`step-mm / control-period`; both values can be adjusted explicitly:

```cmd
python run_steps3_to5.py --hardware --delta-mm 0 0 10 --execute --step-mm 2 --control-period 0.1
```

Run all tests for steps 1-5:

```cmd
python -m unittest -v test_joint_mapping.py test_urdf_fk.py test_steps3_to5.py
```

## Step 6: continuous keyboard control

`keyboard_control.py` runs a fixed-rate Cartesian control loop. Held keys
produce a Cartesian velocity; no key is mapped directly to a joint. Every
control cycle uses the IK and failure recovery from steps 3-5.

Controls:

```text
W / S   forward +X / backward -X
A / D   left +Y / right -Y
R / F   up +Z / down -Z
Q/Esc   normal stop
Space   emergency stop and immediate torque-off
```

Start with the default 20 Hz control rate and 20 mm/s Cartesian speed:

```cmd
python keyboard_control.py
```

If every direction reports `IK 无解`, first check whether the measured start
pose is close to a calibrated joint endpoint. The controller deliberately
reserves a 2-degree joint margin and will reject such a start pose.

The program reads the current pose, checks calibration and joint margins, and
requires the exact phrase `START KEYBOARD CONTROL` before enabling torque on
IDs 1-5. ID 6 is neither read nor commanded.

For the first hardware test, use a slower speed:

```cmd
python keyboard_control.py --speed-mm-s 10 --control-hz 20
```

Run the complete test suite:

```cmd
python -m unittest -v test_joint_mapping.py test_urdf_fk.py test_steps3_to5.py test_keyboard_control.py
```
