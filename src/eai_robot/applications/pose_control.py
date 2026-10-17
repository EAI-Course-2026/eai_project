"""Portable saved-pose control through the existing checked ArmController."""
from pathlib import Path
import argparse
import time

from eai_robot.calibration.results import JOINTS, load
from eai_robot.motion.poses import POSES_PATH, load_poses
from eai_robot.hardware.backends import LeRobotBackend
from eai_robot.control.arm_controller import ArmController
from eai_robot.config import load_config
from eai_robot.input import read_key
from eai_robot.robots.so101 import DEFAULT_CALIBRATION, default_port

POSE_KEYS = {"1": "stand", "2": "raise", "3": "left", "4": "right"}
ACTION_NAMES = ("raise", "left", "right", "left", "stand")


def main():
    parser = argparse.ArgumentParser(description="Preview saved poses; add --enable for interactive motion")
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--poses", type=Path, default=POSES_PATH)
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--allow-wide-range", action="store_true")
    args = parser.parse_args()
    calibration = load(args.calibration)
    poses = load_poses(args.poses, calibration)
    for name, values in poses.items():
        print(name, {calibration[j].id: calibration[j].position(v) for j, v in zip(JOINTS, values, strict=True)})
    if not args.enable:
        print("Offline preview only. No serial port opened.")
        return 0
    if not args.port:
        raise ValueError("Set --port or configs/hardware.local.toml")
    backend = LeRobotBackend(args.port, load_config()["serial"]["baudrate"])
    arm = None
    try:
        arm = ArmController(backend, calibration, motion="smooth", velocity=80,
                            timeout=8, allow_wide_range=args.allow_wide_range)
        arm.inspect()
        if input('Type "START POSE CONTROL" to enable torque: ').strip() != "START POSE CONTROL":
            return 0
        arm.enable()
        print("1 stand / 2 raise / 3 left / 4 right / w wave / q or Space stop")
        while True:
            key = read_key()
            if key in ("q", " ", "\x1b", "\x03"):
                break
            if key in POSE_KEYS:
                arm.move(poses[POSE_KEYS[key]])
            elif key == "w":
                for pose in ACTION_NAMES:
                    arm.move(poses[pose])
                    time.sleep(0.1)
        return 0
    finally:
        try:
            if arm is not None and arm.torque_touched:
                failures = arm.disable()
                if failures:
                    raise RuntimeError("; ".join(failures))
        finally:
            backend.close()
