"""Calibration-bound named joint targets; no device construction or motion."""
from pathlib import Path
import json
import math
from eai_robot.calibration.servo.results import JOINTS, JointCalibration, validate

POSES_PATH = Path(__file__).with_name("poses.reference.json")

def load_poses(path, calibration):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or set(data.get("calibration", {})) != set(JOINTS):
        raise ValueError("Pose file must carry its source calibration")
    old = validate({name: JointCalibration(**data["calibration"][name]) for name in JOINTS})
    result = {}
    for pose, values in data["poses"].items():
        if len(values) != 6 or any(type(v) not in (float, int) or not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError(f"Invalid pose: {pose}")
        converted = []
        for name, value in zip(JOINTS, values, strict=True):
            raw = old[name].position(value)
            if not calibration[name].range_min <= raw <= calibration[name].range_max:
                raise ValueError(f"{pose}/{name}: saved raw target exceeds current calibration")
            converted.append(calibration[name].ratio(raw))
        result[pose] = converted
    return result
