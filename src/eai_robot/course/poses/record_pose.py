"""Record a manually positioned pose to a local, calibration-bound file."""
from pathlib import Path
from dataclasses import asdict
import argparse
import json

from eai_robot.arm.backends import LeRobotBackend
from eai_robot.arm.calibration import JOINTS, load
from eai_robot.arm.controller import ArmController
from eai_robot.config import ROOT, load_config
from eai_robot.course.robot import DEFAULT_CALIBRATION, default_port


def main():
    parser = argparse.ArgumentParser(description="Read and save a pose; torque must already be off")
    parser.add_argument("name")
    parser.add_argument("--port", default=default_port())
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--output", type=Path, default=ROOT / "configs/poses.local.json")
    args = parser.parse_args()
    if not args.port:
        raise ValueError("Set --port or configs/hardware.local.toml")
    calibration = load(args.calibration)
    expected = {name: asdict(calibration[name]) for name in JOINTS}
    data = {"schema_version": 1, "calibration": expected, "poses": {}}
    if args.output.exists():
        data = json.loads(args.output.read_text(encoding="utf-8"))
        if data.get("calibration") != expected:
            raise ValueError("Output poses belong to a different calibration; choose a new file")
        if args.name in data["poses"]:
            raise ValueError("Pose already exists; choose a new name")
    backend = LeRobotBackend(args.port, load_config()["serial"]["baudrate"])
    try:
        arm = ArmController(backend, calibration, allow_wide_range=True)
        positions = arm.inspect()
        if any(backend.read(sid, "Torque_Enable") for sid in range(1, 7)):
            raise RuntimeError("Torque must already be off; this recording command is read-only")
        if any(not cal.range_min <= positions[cal.id] <= cal.range_max for cal in calibration.values()):
            raise ValueError("Current pose is outside the calibrated range")
        data["poses"][args.name] = [calibration[n].ratio(positions[calibration[n].id]) for n in JOINTS]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        temporary.replace(args.output)
        print(f"Saved {args.name} to {args.output}; no motor writes")
        return 0
    finally:
        backend.close()
