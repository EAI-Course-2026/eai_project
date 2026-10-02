"""Step 2 checker for SO-101 forward kinematics.

The default mode is fully offline.  ``--hardware`` opens the configured port only to read
positions through LeRobot; it does not change torque or send a goal position.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, SO101JointMapper, URDF_LIMITS
from eai_robot.course.kinematics.kinematics_backend import ForwardKinematics, create_fk_solver


DEFAULT_URDF = Path(__file__).with_name("so101_new_calib.urdf")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check SO-101 forward kinematics.")
    parser.add_argument("--hardware", action="store_true", help="Read one pose from the configured port")
    parser.add_argument("--watch", action="store_true", help="Continuously read poses until Ctrl+C")
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--robot-id", default="scs215_so101")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--interval", type=float, default=0.20)
    args = parser.parse_args()
    if args.watch:
        args.hardware = True
    if args.interval < 0.05:
        parser.error("--interval must be at least 0.05 seconds")
    return args


def read_raw_arm(robot: SO101Follower) -> dict[str, int]:
    return {
        name: int(robot.bus.read("Present_Position", name, normalize=False, num_retry=5))
        for name in ARM_JOINT_NAMES
    }


def raw_to_degrees(mapper: SO101JointMapper, raw: dict[str, int]) -> np.ndarray:
    return np.asarray(
        [mapper.raw_to_urdf_degrees(name, raw[name]) for name in ARM_JOINT_NAMES],
        dtype=float,
    )


def print_pose(joint_degrees: np.ndarray, transform: np.ndarray) -> None:
    xyz_mm = transform[:3, 3] * 1000.0
    print("Joint angles (URDF degrees):")
    for name, angle in zip(ARM_JOINT_NAMES, joint_degrees, strict=True):
        print(f"  {name:<16} {angle:>8.2f}")
    print(f"TCP xyz (mm): x={xyz_mm[0]:.2f}, y={xyz_mm[1]:.2f}, z={xyz_mm[2]:.2f}")
    print("TCP rotation matrix:")
    print(np.array2string(transform[:3, :3], precision=5, suppress_small=True))


def print_raw_positions(mapper: SO101JointMapper, raw: dict[str, int]) -> None:
    print("Raw SCS215 positions:")
    for name in ARM_JOINT_NAMES:
        calibration = mapper.calibration[name]
        outside = not calibration.range_min <= raw[name] <= calibration.range_max
        marker = "  OUTSIDE CALIBRATION" if outside else ""
        print(
            f"  {name:<16} {raw[name]:>4}  "
            f"range={calibration.range_min:>4}..{calibration.range_max:<4}{marker}"
        )
    print()


def print_positive_joint_deltas(solver: ForwardKinematics, joint_degrees: np.ndarray) -> None:
    """Show the predicted TCP translation for a small positive joint motion."""
    base_position = solver.forward_kinematics(joint_degrees)[:3, 3]
    print("\nPredicted TCP change for a positive 5 degree joint motion (mm):")

    for index, name in enumerate(ARM_JOINT_NAMES):
        trial = joint_degrees.copy()
        upper_deg = URDF_LIMITS[name].upper_deg
        delta_deg = min(5.0, upper_deg - trial[index])
        if delta_deg <= 0:
            print(f"  {name:<16} at upper limit")
            continue
        trial[index] += delta_deg
        delta_mm = (solver.forward_kinematics(trial)[:3, 3] - base_position) * 1000.0
        print(
            f"  {name:<16} +{delta_deg:>4.1f} deg -> "
            f"dx={delta_mm[0]:>7.2f}, dy={delta_mm[1]:>7.2f}, dz={delta_mm[2]:>7.2f}"
        )


def print_calibration_issues(mapper: SO101JointMapper) -> None:
    issues = mapper.validate()
    if not issues:
        return
    print("Calibration warnings:")
    for issue in issues:
        print(f"  {issue.level}: {issue.joint}: {issue.message}")
    print()


def run_offline(solver: ForwardKinematics, backend_name: str) -> None:
    joint_degrees = np.zeros(len(ARM_JOINT_NAMES), dtype=float)
    transform = solver.forward_kinematics(joint_degrees)
    print(f"FK backend: {backend_name}")
    print("Offline URDF zero pose")
    print_pose(joint_degrees, transform)
    print_positive_joint_deltas(solver, joint_degrees)


def run_hardware(args: argparse.Namespace, solver: ForwardKinematics, backend_name: str) -> None:
    robot = make_robot(args.port, args.robot_id, calibration_path=args.calibration)
    if not robot.calibration:
        raise FileNotFoundError(f"No LeRobot calibration found at {robot.calibration_fpath}")
    mapper = SO101JointMapper(robot.calibration)

    print(f"FK backend: {backend_name}")
    print(f"Calibration: {robot.calibration_fpath}")
    print("Hardware mode is READ-ONLY: torque and goal positions are not changed.\n")
    print_calibration_issues(mapper)

    robot.bus.connect(handshake=False)
    try:
        # Protocol 1 occasionally leaves a partial status packet in the receive
        # buffer after opening the port. Position reads already identify a missing
        # motor, so repeated pings only add failure points here.
        robot.bus.port_handler.clearPort()
        time.sleep(0.10)

        if not args.watch:
            raw = read_raw_arm(robot)
            joint_degrees = raw_to_degrees(mapper, raw)
            transform = solver.forward_kinematics(joint_degrees)
            print_raw_positions(mapper, raw)
            print_pose(joint_degrees, transform)
            print_positive_joint_deltas(solver, joint_degrees)
            return

        print("Watching FK. Press Ctrl+C to stop.")
        while True:
            raw = read_raw_arm(robot)
            joint_degrees = raw_to_degrees(mapper, raw)
            xyz_mm = solver.forward_kinematics(joint_degrees)[:3, 3] * 1000.0
            q_text = ", ".join(f"{value:7.2f}" for value in joint_degrees)
            print(
                f"\rq=[{q_text}] deg  xyz=[{xyz_mm[0]:7.2f}, {xyz_mm[1]:7.2f}, "
                f"{xyz_mm[2]:7.2f}] mm",
                end="",
                flush=True,
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


def main() -> int:
    args = parse_args()
    if not args.urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {args.urdf}")

    solver, backend_name = create_fk_solver(args.urdf)
    if args.hardware:
        run_hardware(args, solver, backend_name)
    else:
        run_offline(solver, backend_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
