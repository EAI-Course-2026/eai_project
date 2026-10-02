"""Safely re-index and recalibrate the SO-101 wrist-roll servo (ID 5).

Run one subcommand at a time.  ``inspect`` is read-only, ``center`` may move
the unloaded servo shaft, and ``calibrate`` records a new manual range and
updates only wrist_roll in LeRobot's calibration.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from dataclasses import replace
from datetime import datetime
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from eai_robot.course.robot import CourseMotorsBus as FeetechMotorsBus
from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port
from lerobot.utils.utils import enter_pressed


MOTOR_NAME = "wrist_roll"
MOTOR_ID = 5
MOTOR_MODEL = "scs215"
EXPECTED_MODEL_NUMBER = 1315
ENCODER_MIN = 0
ENCODER_MAX = 1023
DEFAULT_CENTER = 512


def make_bus(port: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={MOTOR_NAME: Motor(MOTOR_ID, MOTOR_MODEL, MotorNormMode.RANGE_M100_100)},
        protocol_version=1,
    )


def connect_and_check(bus: FeetechMotorsBus) -> int:
    bus.connect(handshake=False)
    model_number = bus.ping(MOTOR_NAME, num_retry=2, raise_on_error=True)
    if model_number != EXPECTED_MODEL_NUMBER:
        raise RuntimeError(
            f"ID {MOTOR_ID} reports model {model_number}; expected SCS215 model "
            f"{EXPECTED_MODEL_NUMBER}. Refusing to continue."
        )
    return int(model_number)


def read_raw(bus: FeetechMotorsBus) -> int:
    value = int(bus.read("Present_Position", MOTOR_NAME, normalize=False, num_retry=2))
    if not ENCODER_MIN <= value <= ENCODER_MAX:
        raise RuntimeError(f"Invalid SCS215 position: {value}")
    return value


def inspect(bus: FeetechMotorsBus) -> None:
    model_number = connect_and_check(bus)
    position = read_raw(bus)
    print(f"ID={MOTOR_ID}, model={model_number}, raw_position={position}")
    print("Read-only inspection complete. No motor register was written.")


def center_unloaded_servo(
    bus: FeetechMotorsBus,
    center: int,
    velocity: int,
    tolerance: int,
    timeout_s: float,
) -> None:
    model_number = connect_and_check(bus)
    current = read_raw(bus)
    print(f"ID={MOTOR_ID}, model={model_number}, current={current}, target={center}")
    print("\nDANGER: ID 5 will move after confirmation.")
    print("The wrist horn/linkage must already be detached from the servo shaft.")
    print("Keep the power switch or power cable within reach.")

    phrase = f"CENTER ID5 TO {center}"
    if input(f'Type exactly "{phrase}" to continue: ').strip() != phrase:
        print("Confirmation did not match. Nothing was written.")
        return

    try:
        # Match the goal to the measured position before enabling torque so the
        # servo cannot chase a stale command left by an earlier program.
        bus.disable_torque(MOTOR_NAME, num_retry=2)
        bus.write("Goal_Position", MOTOR_NAME, current, normalize=False, num_retry=2)
        bus.write("Goal_Velocity", MOTOR_NAME, velocity, normalize=False, num_retry=2)
        bus.enable_torque(MOTOR_NAME, num_retry=2)
        bus.write("Goal_Position", MOTOR_NAME, center, normalize=False, num_retry=2)

        deadline = time.monotonic() + timeout_s
        while True:
            position = read_raw(bus)
            print(f"\rID 5 position={position:4d}, target={center:4d}", end="", flush=True)
            if abs(position - center) <= tolerance:
                print(f"\nCentered within +/-{tolerance} encoder counts.")
                break
            if time.monotonic() >= deadline:
                raise TimeoutError(f"ID 5 did not reach {center} within {timeout_s:.1f} seconds")
            time.sleep(0.10)
    finally:
        if bus.is_connected:
            bus.disable_torque(MOTOR_NAME, num_retry=2)
            print("Torque disabled. Carefully reattach the wrist at its physical neutral pose.")


def record_range(bus: FeetechMotorsBus, interval_s: float) -> tuple[int, int, bool]:
    connect_and_check(bus)
    bus.disable_torque(MOTOR_NAME, num_retry=2)

    position = read_raw(bus)
    minimum = position
    maximum = position
    previous = position
    boundary_crossed = False

    print("Torque is disabled.")
    print("Slowly move wrist_roll through the intended SAFE range.")
    print("Do not force a mechanical stop. Press ENTER when finished.\n")

    while True:
        position = read_raw(bus)
        minimum = min(minimum, position)
        maximum = max(maximum, position)

        if abs(position - previous) > 512:
            boundary_crossed = True
            print(f"\nBOUNDARY JUMP DETECTED: {previous} -> {position}")

        print(
            f"\rposition={position:4d}  minimum={minimum:4d}  maximum={maximum:4d}",
            end="",
            flush=True,
        )
        previous = position

        if enter_pressed():
            print()
            break
        time.sleep(interval_s)

    return minimum, maximum, boundary_crossed


def save_wrist_calibration(
    bus: FeetechMotorsBus,
    port: str,
    robot_id: str,
    calibration_path: Path,
    minimum: int,
    maximum: int,
) -> Path:
    if calibration_path.resolve() == DEFAULT_CALIBRATION.resolve():
        raise RuntimeError("Maintenance requires --calibration pointing to a local copy; shared baseline is review-only")
    robot = make_robot(port, robot_id, calibration_path=calibration_path)
    if not robot.calibration:
        raise FileNotFoundError(f"No LeRobot calibration found at {robot.calibration_fpath}")
    if MOTOR_NAME not in robot.calibration:
        raise KeyError(f"{MOTOR_NAME} is missing from {robot.calibration_fpath}")

    phrase = f"SAVE ID5 RANGE {minimum} {maximum}"
    print(f"\nNew ID 5 range: {minimum}..{maximum}")
    print(f"Calibration file: {robot.calibration_fpath}")
    if input(f'Type exactly "{phrase}" to update the servo and file: ').strip() != phrase:
        raise RuntimeError("Confirmation did not match. Calibration was not changed.")

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = robot.calibration_fpath.with_suffix(f".{timestamp}.bak.json")
    shutil.copy2(robot.calibration_fpath, backup_path)

    # SCS215 protocol 1 stores ordinary, non-wrapping min/max limits.  Torque
    # must remain disabled while changing these persistent registers.
    bus.disable_torque(MOTOR_NAME, num_retry=2)
    bus.write_verified("Lock", MOTOR_NAME, 0)
    try:
        bus.write_verified("Min_Position_Limit", MOTOR_NAME, minimum)
        bus.write_verified("Max_Position_Limit", MOTOR_NAME, maximum)
    finally:
        bus.write_verified("Lock", MOTOR_NAME, 1)

    old = robot.calibration[MOTOR_NAME]
    robot.calibration[MOTOR_NAME] = replace(old, range_min=minimum, range_max=maximum)
    robot._save_calibration()  # LeRobot owns the JSON format.

    print(f"Backup: {backup_path}")
    print(f"Updated: {robot.calibration_fpath}")
    return robot.calibration_fpath


def calibrate(
    bus: FeetechMotorsBus,
    port: str,
    robot_id: str,
    calibration_path: Path,
    interval_s: float,
    boundary_margin: int,
) -> None:
    minimum, maximum, boundary_crossed = record_range(bus, interval_s)
    print(f"Recorded range: {minimum}..{maximum}")

    if boundary_crossed:
        raise RuntimeError(
            "The recorded motion crossed 1023/0. Re-index the horn again; nothing was saved."
        )
    if minimum < boundary_margin or maximum > ENCODER_MAX - boundary_margin:
        raise RuntimeError(
            f"Range is too close to an encoder boundary. Keep at least {boundary_margin} "
            "counts from both 0 and 1023; nothing was saved."
        )
    if maximum - minimum < 100:
        raise RuntimeError("Recorded range is suspiciously small; nothing was saved.")

    save_wrist_calibration(bus, port, robot_id, calibration_path, minimum, maximum)
    print("ID 5 recalibration complete. Torque remains disabled.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Re-index only the SCS215 wrist-roll servo (ID 5).")
    parser.add_argument("mode", choices=("inspect", "center", "calibrate"))
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--robot-id", default="scs215_so101")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--center", type=int, default=DEFAULT_CENTER)
    parser.add_argument("--velocity", type=int, default=80)
    parser.add_argument("--tolerance", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--boundary-margin", type=int, default=32)
    args = parser.parse_args()

    if not ENCODER_MIN <= args.center <= ENCODER_MAX:
        parser.error("--center must be within 0..1023")
    if not 1 <= args.velocity <= 1000:
        parser.error("--velocity must be within 1..1000; zero can mean maximum speed")
    if not 0 <= args.tolerance <= 50:
        parser.error("--tolerance must be within 0..50")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if not 0.02 <= args.interval <= 1.0:
        parser.error("--interval must be within 0.02..1.0")
    if not 16 <= args.boundary_margin <= 128:
        parser.error("--boundary-margin must be within 16..128")
    return args


def main() -> int:
    args = parse_args()
    bus = make_bus(args.port)

    try:
        if args.mode == "inspect":
            inspect(bus)
        elif args.mode == "center":
            center_unloaded_servo(
                bus,
                center=args.center,
                velocity=args.velocity,
                tolerance=args.tolerance,
                timeout_s=args.timeout,
            )
        else:
            calibrate(
                bus,
                port=args.port,
                robot_id=args.robot_id,
                calibration_path=args.calibration,
                interval_s=args.interval,
                boundary_margin=args.boundary_margin,
            )
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted. Calibration was not saved.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if bus.is_connected:
            # Center/calibrate explicitly disable torque. Inspect must stay
            # read-only, so closing the port must not alter torque state.
            bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    raise SystemExit(main())
