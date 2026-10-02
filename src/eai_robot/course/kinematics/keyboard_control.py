"""Continuous Cartesian keyboard control for the SO-101 follower arm.

Keys command the gripper frame in the URDF base coordinate system. Joint
targets are produced only by IK; no key is mapped directly to a servo.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path
from typing import Iterable

import numpy as np
from eai_robot.course.input import keyboard

from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port

from eai_robot.course.kinematics.cartesian_planner import CartesianLinePlanner
from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, SO101JointMapper
from eai_robot.course.kinematics.kinematics_backend import create_fk_solver
from eai_robot.course.kinematics.position_ik import create_position_ik_solver
from eai_robot.course.kinematics.run_steps3_to5 import (
    arm_degrees_from_raw,
    ensure_execution_start_is_safe,
    raw_outside_calibration,
    read_raw_motors,
)


DEFAULT_URDF = Path(__file__).with_name("so101_new_calib.urdf")
MOVEMENT_KEYS = frozenset({"w", "s", "a", "d", "r", "f"})


def direction_from_keys(keys: Iterable[str]) -> np.ndarray:
    """Return a unit Cartesian direction in the URDF base frame."""
    pressed = set(keys)
    direction = np.array(
        [
            float("w" in pressed) - float("s" in pressed),
            float("a" in pressed) - float("d" in pressed),
            float("r" in pressed) - float("f" in pressed),
        ],
        dtype=float,
    )
    norm = float(np.linalg.norm(direction))
    return direction / norm if norm > 0 else direction


class KeyboardState:
    def __init__(self) -> None:
        self._pressed: set[str] = set()
        self._lock = threading.Lock()
        self.quit_requested = threading.Event()
        self.emergency_requested = threading.Event()

    def on_press(self, key: keyboard.Key | keyboard.KeyCode) -> bool | None:
        if key == keyboard.Key.space:
            self.emergency_requested.set()
            self.quit_requested.set()
            return False
        if key == keyboard.Key.esc:
            self.quit_requested.set()
            return False

        char = getattr(key, "char", None)
        if char is None:
            return None
        char = char.lower()
        if char == "q":
            self.quit_requested.set()
            return False
        if char in MOVEMENT_KEYS:
            with self._lock:
                self._pressed.add(char)
        return None

    def on_release(self, key: keyboard.Key | keyboard.KeyCode) -> None:
        char = getattr(key, "char", None)
        if char is None:
            return
        with self._lock:
            self._pressed.discard(char.lower())

    def snapshot(self) -> set[str]:
        with self._lock:
            return self._pressed.copy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Keyboard Cartesian control for SO-101.")
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--robot-id", default="scs215_so101")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--speed-mm-s", type=float, default=20.0)
    parser.add_argument("--control-hz", type=float, default=20.0)
    parser.add_argument("--max-joint-step-deg", type=float, default=5.0)
    parser.add_argument("--joint-margin-deg", type=float, default=2.0)
    parser.add_argument("--feedback-hz", type=float, default=2.0)
    parser.add_argument("--max-tracking-error-deg", type=float, default=12.0)
    args = parser.parse_args()

    if not 2.0 <= args.speed_mm_s <= 50.0:
        parser.error("--speed-mm-s must be within 2..50")
    if not 5.0 <= args.control_hz <= 30.0:
        parser.error("--control-hz must be within 5..30")
    if not 1.0 <= args.max_joint_step_deg <= 10.0:
        parser.error("--max-joint-step-deg must be within 1..10")
    if not 0.5 <= args.joint_margin_deg <= 10.0:
        parser.error("--joint-margin-deg must be within 0.5..10")
    if not 0.5 <= args.feedback_hz <= 5.0:
        parser.error("--feedback-hz must be within 0.5..5")
    if not 3.0 <= args.max_tracking_error_deg <= 30.0:
        parser.error("--max-tracking-error-deg must be within 3..30")
    return args


def print_controls() -> None:
    print("\nCartesian controls in the URDF base frame:")
    print("  W / S : forward +X / backward -X")
    print("  A / D : left +Y / right -Y")
    print("  R / F : up +Z / down -Z")
    print("  Q/Esc : stop normally")
    print("  Space : emergency stop and torque off")


def run_controller(args: argparse.Namespace) -> int:
    fk, fk_backend = create_fk_solver(args.urdf)
    control_period = 1.0 / args.control_hz
    feedback_period = 1.0 / args.feedback_hz
    cartesian_step_m = (args.speed_mm_s / 1000.0) * control_period
    ik_tolerance_m = min(0.0002, cartesian_step_m * 0.2)
    ik, ik_backend = create_position_ik_solver(
        fk,
        fk_backend,
        tolerance_m=ik_tolerance_m,
        joint_margin_deg=args.joint_margin_deg,
    )

    planner = CartesianLinePlanner(
        fk,
        ik,
        max_cartesian_step_m=cartesian_step_m,
        max_joint_step_deg=args.max_joint_step_deg,
    )

    robot = make_robot(args.port, args.robot_id, calibration_path=args.calibration)
    if not robot.calibration:
        raise FileNotFoundError(f"No calibration found at {robot.calibration_fpath}")
    mapper = SO101JointMapper(robot.calibration)
    issues = mapper.validate()
    if issues:
        details = "; ".join(f"{issue.level} {issue.joint}: {issue.message}" for issue in issues)
        raise RuntimeError(f"Calibration check failed: {details}")

    state = KeyboardState()
    listener: keyboard.Listener | None = None
    torque_enabled = False
    robot.bus.connect(handshake=False)

    try:
        robot.bus.port_handler.clearPort()
        time.sleep(0.10)
        raw = read_raw_motors(robot, ARM_JOINT_NAMES)
        outside = raw_outside_calibration(mapper, raw)
        if outside:
            details = ", ".join(
                f"{name}={raw[name]} "
                f"(range {mapper.calibration[name].range_min}..{mapper.calibration[name].range_max})"
                for name in outside
            )
            raise RuntimeError(f"Raw positions outside calibration: {details}")

        command_q = arm_degrees_from_raw(mapper, raw)
        ensure_execution_start_is_safe(command_q, margin_deg=args.joint_margin_deg)
        command_position = fk.forward_kinematics(command_q)[:3, 3]

        print(f"FK backend: {fk_backend}")
        print(f"IK backend: {ik_backend}")
        print(f"Start TCP xyz (mm): {np.round(command_position * 1000.0, 2)}")
        print(f"Control rate: {args.control_hz:.1f} Hz")
        print(f"Cartesian speed: {args.speed_mm_s:.1f} mm/s")
        print_controls()

        phrase = "START KEYBOARD CONTROL"
        if input(f'\nType exactly "{phrase}" to enable torque: ').strip() != phrase:
            print("Confirmation did not match. Torque was not enabled.")
            return 0

        current_goals = {
            name: mapper.raw_to_normalized(name, raw[name]) for name in ARM_JOINT_NAMES
        }
        robot.bus.sync_write("Goal_Position", current_goals)
        torque_enabled = True
        robot.enable_motion(list(ARM_JOINT_NAMES))

        listener = keyboard.Listener(on_press=state.on_press, on_release=state.on_release)
        listener.start()
        print("Keyboard control active.")

        next_tick = time.perf_counter()
        next_feedback = next_tick + feedback_period
        tracking_paused = False
        last_ik_warning = 0.0

        while not state.quit_requested.is_set():
            now = time.perf_counter()
            direction = direction_from_keys(state.snapshot())

            if now >= next_feedback:
                measured_raw = read_raw_motors(robot, ARM_JOINT_NAMES)
                measured_q = arm_degrees_from_raw(mapper, measured_raw)
                tracking_error = float(np.max(np.abs(measured_q - command_q)))
                tracking_paused = tracking_error > args.max_tracking_error_deg
                if tracking_paused:
                    print(
                        f"\nTracking paused: maximum joint error {tracking_error:.2f} deg",
                        flush=True,
                    )
                next_feedback = now + feedback_period

            if np.any(direction) and not tracking_paused:
                requested_target = command_position + direction * cartesian_step_m
                plan = planner.plan(command_q, requested_target)

                if plan.waypoints:
                    waypoint = plan.waypoints[-1]
                    action = {
                        f"{name}.pos": mapper.urdf_degrees_to_normalized(name, value)
                        for name, value in zip(
                            ARM_JOINT_NAMES, waypoint.joint_degrees, strict=True
                        )
                    }
                    robot.send_action(action)
                    command_q = waypoint.joint_degrees.copy()
                    command_position = waypoint.achieved_position.copy()

                if plan.ik_failed and now - last_ik_warning >= 1.0:
                    print(f"\n{plan.message}", flush=True)
                    last_ik_warning = now

            print(
                f"\rTCP command (mm): x={command_position[0] * 1000:7.2f} "
                f"y={command_position[1] * 1000:7.2f} "
                f"z={command_position[2] * 1000:7.2f}",
                end="",
                flush=True,
            )

            next_tick += control_period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_tick = time.perf_counter()

        print()
        if state.emergency_requested.is_set():
            print("Emergency stop requested. Disabling torque immediately.")
        else:
            input("Control stopped. Support the arm, then press ENTER to disable torque.")
        return 0
    finally:
        if listener is not None:
            listener.stop()
            listener.join(timeout=1.0)
        if torque_enabled and robot.bus.is_connected:
            try:
                robot.bus.disable_torque(list(ARM_JOINT_NAMES), num_retry=5)
                print("Torque disabled on arm joints ID 1-5.")
            except Exception as exc:
                print(f"WARNING: could not disable torque: {exc}", file=sys.stderr)
                print("Disconnect servo power immediately.", file=sys.stderr)
        if robot.bus.is_connected:
            robot.bus.disconnect(disable_torque=False)


def main() -> int:
    args = parse_args()
    if not args.urdf.is_file():
        raise FileNotFoundError(f"URDF not found: {args.urdf}")
    return run_controller(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted. Torque shutdown is handled by the controller.", file=sys.stderr)
        raise SystemExit(130) from None
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise SystemExit(1) from None
