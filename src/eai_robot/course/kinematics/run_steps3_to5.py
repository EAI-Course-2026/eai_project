"""Solve one IK target, plan a straight line, and recover from unreachable IK.

Planning is the default and does not open a serial port. ``--hardware`` reads the real
start pose through LeRobot. Actual motion additionally requires ``--execute``
and an exact interactive confirmation.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port

from eai_robot.course.kinematics.cartesian_planner import CartesianLinePlanner, CartesianPlan
from eai_robot.course.kinematics.joint_mapping import ALL_MOTOR_NAMES, ARM_JOINT_NAMES, SO101JointMapper, URDF_LIMITS
from eai_robot.course.kinematics.kinematics_backend import create_fk_solver
from eai_robot.course.kinematics.position_ik import create_position_ik_solver


DEFAULT_URDF = Path(__file__).with_name("so101_new_calib.urdf")
DEFAULT_START_DEG = np.array([0.0, -25.0, 40.0, -15.0, 0.0])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SO-101 steps 3-5: IK and straight-line planning.")
    target_group = parser.add_mutually_exclusive_group()
    target_group.add_argument(
        "--delta-mm",
        nargs=3,
        type=float,
        metavar=("DX", "DY", "DZ"),
        default=(0.0, 0.0, 20.0),
        help="Target displacement in the URDF base frame (default: 0 0 20)",
    )
    target_group.add_argument(
        "--target-mm",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Absolute target in the URDF base frame",
    )
    parser.add_argument("--hardware", action="store_true", help="Read the start pose from the configured port")
    parser.add_argument("--execute", action="store_true", help="Execute the planned path on hardware")
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--robot-id", default="scs215_so101")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--step-mm", type=float, default=3.0)
    parser.add_argument("--max-joint-step-deg", type=float, default=8.0)
    parser.add_argument("--waypoint-timeout", type=float, default=3.0)
    parser.add_argument("--joint-tolerance-deg", type=float, default=2.0)
    parser.add_argument(
        "--control-period",
        type=float,
        default=0.10,
        help="Seconds between streamed Cartesian waypoints (default: 0.10)",
    )
    args = parser.parse_args()

    if args.execute and not args.hardware:
        parser.error("--execute requires --hardware")
    if not 0.5 <= args.step_mm <= 10.0:
        parser.error("--step-mm must be within 0.5..10")
    if not 1.0 <= args.max_joint_step_deg <= 15.0:
        parser.error("--max-joint-step-deg must be within 1..15")
    if args.waypoint_timeout <= 0:
        parser.error("--waypoint-timeout must be positive")
    if not 0.05 <= args.control_period <= 0.50:
        parser.error("--control-period must be within 0.05..0.50 seconds")
    return args


def read_raw_motor(robot: SO101Follower, name: str) -> int:
    last_error: Exception | None = None
    for _ in range(3):
        try:
            return int(robot.bus.read("Present_Position", name, normalize=False, num_retry=5))
        except (ConnectionError, RuntimeError) as exc:
            last_error = exc
            robot.bus.port_handler.clearPort()
            time.sleep(0.08)
    assert last_error is not None
    raise last_error


def read_raw_motors(robot: SO101Follower, names: tuple[str, ...]) -> dict[str, int]:
    return {name: read_raw_motor(robot, name) for name in names}


def arm_degrees_from_raw(mapper: SO101JointMapper, raw: dict[str, int]) -> np.ndarray:
    return np.asarray(
        [mapper.raw_to_urdf_degrees(name, raw[name]) for name in ARM_JOINT_NAMES],
        dtype=float,
    )


def raw_outside_calibration(mapper: SO101JointMapper, raw: dict[str, int]) -> list[str]:
    outside = []
    for name in raw:
        calibration = mapper.calibration[name]
        if not calibration.range_min <= raw[name] <= calibration.range_max:
            outside.append(name)
    return outside


def raw_range_excess(mapper: SO101JointMapper, raw: dict[str, int], name: str) -> int:
    calibration = mapper.calibration[name]
    if raw[name] < calibration.range_min:
        return calibration.range_min - raw[name]
    if raw[name] > calibration.range_max:
        return raw[name] - calibration.range_max
    return 0


def print_plan(plan: CartesianPlan, ik_backend: str) -> None:
    print(f"IK backend: {ik_backend}")
    print(f"Start xyz (mm):     {np.round(plan.start_position * 1000.0, 2)}")
    print(f"Requested xyz (mm): {np.round(plan.requested_target * 1000.0, 2)}")
    print(f"Planned waypoints:  {len(plan.waypoints)}")

    if plan.ik_failed:
        print(plan.message)
    else:
        print(f"Plan result: {plan.message}")

    print(f"Final xyz (mm):     {np.round(plan.final_position * 1000.0, 2)}")
    remaining_mm = float(np.linalg.norm(plan.requested_target - plan.final_position) * 1000.0)
    print(f"Remaining error:    {remaining_mm:.3f} mm")
    if plan.waypoints:
        print("Final joint angles (deg):")
        for name, value in zip(ARM_JOINT_NAMES, plan.waypoints[-1].joint_degrees, strict=True):
            print(f"  {name:<16} {value:>8.2f}")


def ensure_execution_start_is_safe(q_degrees: np.ndarray, margin_deg: float = 2.0) -> None:
    unsafe = []
    for name, value in zip(ARM_JOINT_NAMES, q_degrees, strict=True):
        limit = URDF_LIMITS[name]
        if value <= limit.lower_deg + margin_deg or value >= limit.upper_deg - margin_deg:
            unsafe.append(f"{name}={value:.2f} deg")
    if unsafe:
        raise RuntimeError(
            "Refusing execution because the start pose is within 2 degrees of a joint limit: "
            + ", ".join(unsafe)
        )


def wait_for_waypoint(
    robot: SO101Follower,
    mapper: SO101JointMapper,
    target_degrees: np.ndarray,
    tolerance_deg: float,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while True:
        raw = {name: read_raw_motor(robot, name) for name in ARM_JOINT_NAMES}
        measured = arm_degrees_from_raw(mapper, raw)
        error = float(np.max(np.abs(measured - target_degrees)))
        if error <= tolerance_deg:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Waypoint timeout; maximum joint error is {error:.2f} degrees")
        time.sleep(0.05)


def execute_plan(
    robot: SO101Follower,
    mapper: SO101JointMapper,
    plan: CartesianPlan,
    current_raw: dict[str, int],
    tolerance_deg: float,
    timeout_s: float,
    control_period_s: float,
) -> None:
    if not plan.waypoints:
        print("No movement is required.")
        return

    path_distance_mm = float(np.linalg.norm(plan.final_position - plan.start_position) * 1000.0)
    if path_distance_mm > 100.0:
        raise RuntimeError("Refusing to execute a path longer than 100 mm in one command")

    phrase = f"MOVE ARM THROUGH {len(plan.waypoints)} WAYPOINTS"
    print("\nDANGER: this will enable torque and move all five arm joints.")
    print("Clear the workspace and keep the power switch within reach.")
    if plan.ik_failed:
        print("The requested target is unreachable; motion will stop at the farthest feasible point.")
    if input(f'Type exactly "{phrase}" to continue: ').strip() != phrase:
        print("Confirmation did not match. No movement command was sent.")
        return

    current_normalized = {
        name: mapper.raw_to_normalized(name, current_raw[name]) for name in ALL_MOTOR_NAMES
    }

    torque_was_enabled = False
    try:
        # Seed every goal with the measured pose before enabling torque. This
        # prevents a stale goal from causing a jump at torque-on.
        robot.bus.sync_write("Goal_Position", current_normalized)
        torque_was_enabled = True
        robot.enable_motion()

        gripper_position = current_normalized["gripper"]
        next_send_time = time.perf_counter()
        for index, waypoint in enumerate(plan.waypoints, start=1):
            action = {
                f"{name}.pos": mapper.urdf_degrees_to_normalized(name, value)
                for name, value in zip(ARM_JOINT_NAMES, waypoint.joint_degrees, strict=True)
            }
            action["gripper.pos"] = gripper_position
            robot.send_action(action)

            # Stream the next target before the arm fully settles at this one.
            # Dense Cartesian waypoints then form one continuous motion instead
            # of a stop-and-go sequence. Only the final target is awaited.
            next_send_time += control_period_s
            sleep_time = next_send_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            print(f"\rSent waypoint {index}/{len(plan.waypoints)}", end="", flush=True)

        print()
        wait_for_waypoint(
            robot,
            mapper,
            plan.waypoints[-1].joint_degrees,
            tolerance_deg=tolerance_deg,
            timeout_s=timeout_s,
        )
        print("Reached final waypoint.")

        input("Path complete. Support the arm, then press ENTER to disable torque.")
    finally:
        if torque_was_enabled and robot.bus.is_connected:
            try:
                robot.bus.disable_torque(num_retry=5)
                print("Torque disabled.")
            except Exception as exc:
                print(f"WARNING: could not disable torque: {exc}", file=sys.stderr)
                print("Disconnect servo power immediately.", file=sys.stderr)


def main() -> int:
    args = parse_args()
    if not args.urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {args.urdf}")

    fk, fk_backend = create_fk_solver(args.urdf)
    ik, ik_backend = create_position_ik_solver(fk, fk_backend)
    planner = CartesianLinePlanner(
        fk,
        ik,
        max_cartesian_step_m=args.step_mm / 1000.0,
        max_joint_step_deg=args.max_joint_step_deg,
    )

    robot: SO101Follower | None = None
    mapper: SO101JointMapper | None = None
    current_raw: dict[str, int] | None = None

    try:
        if args.hardware:
            robot = make_robot(args.port, args.robot_id, calibration_path=args.calibration)
            if not robot.calibration:
                raise FileNotFoundError(f"No calibration found at {robot.calibration_fpath}")
            mapper = SO101JointMapper(robot.calibration)
            issues = mapper.validate()
            if issues:
                details = "; ".join(f"{issue.level} {issue.joint}: {issue.message}" for issue in issues)
                raise RuntimeError(f"Calibration check failed: {details}")

            robot.bus.connect(handshake=False)
            robot.bus.port_handler.clearPort()
            time.sleep(0.10)
            required_motors = ALL_MOTOR_NAMES if args.execute else ARM_JOINT_NAMES
            current_raw = read_raw_motors(robot, required_motors)
            outside = raw_outside_calibration(mapper, current_raw)
            if outside:
                details = ", ".join(
                    f"{name}={current_raw[name]} "
                    f"(range {mapper.calibration[name].range_min}..{mapper.calibration[name].range_max})"
                    for name in outside
                )
                largest_excess = max(raw_range_excess(mapper, current_raw, name) for name in outside)
                if args.execute or largest_excess > 5:
                    raise RuntimeError(f"Raw positions outside calibration: {details}")
                print(f"WARNING: clipping read-only endpoint noise: {details}")
            start_degrees = arm_degrees_from_raw(mapper, current_raw)
        else:
            start_degrees = DEFAULT_START_DEG.copy()

        start_position = fk.forward_kinematics(start_degrees)[:3, 3]
        if args.target_mm is not None:
            target_position = np.asarray(args.target_mm, dtype=float) / 1000.0
        else:
            target_position = start_position + np.asarray(args.delta_mm, dtype=float) / 1000.0

        plan = planner.plan(start_degrees, target_position)
        print(f"FK backend: {fk_backend}")
        print_plan(plan, ik_backend)

        if args.execute:
            assert robot is not None and mapper is not None and current_raw is not None
            ensure_execution_start_is_safe(start_degrees)
            execute_plan(
                robot,
                mapper,
                plan,
                current_raw,
                tolerance_deg=args.joint_tolerance_deg,
                timeout_s=args.waypoint_timeout,
                control_period_s=args.control_period,
            )
        return 2 if plan.ik_failed else 0
    finally:
        if robot is not None and robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130) from None
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
