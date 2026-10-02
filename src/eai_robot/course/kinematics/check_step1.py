"""Offline report for step 1: LeRobot calibration to URDF angle mapping."""

from __future__ import annotations

import argparse
from pathlib import Path

from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, GRIPPER_NAME, SO101JointMapper, URDF_LIMITS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check the SO-101 joint mapping without moving it.")
    parser.add_argument("--port", default=default_port(), help="Stored in the LeRobot config; not opened")
    parser.add_argument("--robot-id", default="scs215_so101", help="LeRobot calibration file ID")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Constructing the robot loads LeRobot's calibration JSON.  Deliberately do
    # not call robot.connect(): this script must not open the port or write motors.
    robot = make_robot(args.port, args.robot_id, calibration_path=args.calibration)
    if not robot.calibration:
        raise FileNotFoundError(f"No LeRobot calibration found at {robot.calibration_fpath}")

    mapper = SO101JointMapper(robot.calibration)

    print("Step 1: calibration -> URDF joint mapping")
    print(f"Calibration: {robot.calibration_fpath}")
    print("Hardware: OFFLINE (COM port was not opened)\n")

    print(
        f"{'JOINT':<16} {'ID':>2} {'RAW RANGE':>12} "
        f"{'URDF RANGE (deg)':>21} {'RAW AT 0 deg':>13}"
    )
    print("-" * 70)
    for name in ARM_JOINT_NAMES:
        cal = robot.calibration[name]
        limit = URDF_LIMITS[name]
        raw_zero = mapper.urdf_degrees_to_raw(name, 0.0)
        print(
            f"{name:<16} {cal.id:>2} {cal.range_min:>4}..{cal.range_max:<4} "
            f"{limit.lower_deg:>8.2f}..{limit.upper_deg:<8.2f} {raw_zero:>13}"
        )

    gripper = robot.calibration[GRIPPER_NAME]
    print(
        f"{GRIPPER_NAME:<16} {gripper.id:>2} {gripper.range_min:>4}..{gripper.range_max:<4} "
        f"{'not in IK (0..100)':>21} {'-':>13}"
    )

    issues = mapper.validate()
    print("\nCalibration checks:")
    if not issues:
        print("  OK: no structural calibration problems found.")
    else:
        for issue in issues:
            print(f"  {issue.level}: {issue.joint}: {issue.message}")

    print("\nMapping assumption:")
    print("  calibrated minimum -> URDF lower limit")
    print("  calibrated maximum -> URDF upper limit")
    print("  Joint signs and zero alignment will be verified with FK in step 2.")

    has_errors = any(issue.level == "ERROR" for issue in issues)
    has_warnings = any(issue.level == "WARNING" for issue in issues)
    if has_errors:
        print("\nRESULT: FAILED - fix calibration errors before step 2.")
        return 1
    if has_warnings:
        print("\nRESULT: REVIEW REQUIRED - do not use the warned joint for IK yet.")
    else:
        print("\nRESULT: STEP 1 MAPPING IS READY FOR FK VERIFICATION.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
