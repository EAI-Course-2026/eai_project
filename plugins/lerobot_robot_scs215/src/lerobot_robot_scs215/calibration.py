"""LeRobot servo calibration workflow; independent of the application package."""
from dataclasses import asdict
from datetime import datetime, timezone
import json
import shutil
from lerobot.motors import MotorCalibration
from .safety import JOINTS


def calibrate_robot(robot):
    if not robot.bus.is_connected:
        raise RuntimeError("Connect read-only before calibrating")
    if any(robot.bus.read("Torque_Enable", n, normalize=False) for n in JOINTS):
        if (
            input("Support the arm. Type RELEASE to disable torque: ").strip()
            != "RELEASE"
        ):
            raise RuntimeError("Calibration cancelled; torque unchanged")
        robot.bus.disable_torque()
        robot.motion_enabled = False
        robot.torque_touched = False
        robot._torque_motors.clear()
    if (
        robot.calibration
        and input(
            "ENTER verifies current calibration; type c for new manual ranges: "
        )
        .strip()
        .lower()
        != "c"
    ):
        if not robot.is_calibrated:
            raise RuntimeError(
                "Current software calibration does not fit EEPROM limits"
            )
        print(f"Verified {robot.calibration_fpath}; no EEPROM writes")
        return
    input(
        "Support the torque-off arm. Move joints slowly through safe, collision-free ranges; ENTER starts sampling: "
    )
    mins, maxes = robot.bus.record_ranges_of_motion()
    result = {}
    for name in JOINTS:
        hwlow, hwhigh = robot.bus.hardware_limits[name]
        low = max(int(mins[name]), hwlow)
        high = min(int(maxes[name]), hwhigh)
        if high - low < 20:
            raise ValueError(
                f"{name}: insufficient safe travel inside EEPROM limits"
            )
        drive = robot.calibration[name].drive_mode if name in robot.calibration else 0
        result[name] = MotorCalibration(
            robot.bus.motors[name].id, drive, 0, low, high
        )
    robot._validate_calibration(result)
    print(json.dumps({n: asdict(c) for n, c in result.items()}, indent=2))
    if (
        input(
            "Type SAVE to replace software calibration (EEPROM unchanged): "
        ).strip()
        != "SAVE"
    ):
        raise RuntimeError("Calibration cancelled; file unchanged")
    for name in JOINTS:
        if robot.bus.read("Torque_Enable", name, normalize=False) or robot.bus.read(
            "Status", name, normalize=False
        ):
            raise RuntimeError(
                f"{name}: torque/alarm changed during calibration; file unchanged"
            )
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    if robot.calibration_fpath.exists():
        shutil.copy2(
            robot.calibration_fpath,
            robot.calibration_fpath.with_suffix(f".{stamp}.bak.json"),
        )
    temporary = robot.calibration_fpath.with_suffix(".json.tmp")
    try:
        temporary.write_text(
            json.dumps({n: asdict(c) for n, c in result.items()}, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(robot.calibration_fpath)
    finally:
        temporary.unlink(missing_ok=True)
    metadata = robot.calibration_fpath.with_suffix(".meta.json")
    if metadata.exists():
        metadata.replace(metadata.with_suffix(f".{stamp}.bak.json"))
    # Retire the old hardware receipt: it describes the previous software file.
    # New manual software calibration makes no claim of EEPROM modification.
    metadata.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "method": "lerobot_plugin_manual_software_ranges",
                "captured": datetime.now(timezone.utc).isoformat(),
                "eeprom_modified": False,
                "manual_endpoints_verified": True,
                "powered_motion_verified": False,
                "ranges": {
                    n: [c.range_min, c.range_max] for n, c in result.items()
                },
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    robot.calibration = result
    robot.bus.calibration = result
    print(f"Software calibration saved: {robot.calibration_fpath}; EEPROM unchanged")
