"""Map LeRobot SCS215 calibration values to SO-101 URDF joint angles.

This module is intentionally hardware-free.  It consumes the calibration that
LeRobot already loaded and never opens the serial port.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees, radians
from typing import Mapping

from lerobot.motors import MotorCalibration


ENCODER_MIN = 0
ENCODER_MAX = 1023

ARM_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
GRIPPER_NAME = "gripper"
ALL_MOTOR_NAMES = ARM_JOINT_NAMES + (GRIPPER_NAME,)


@dataclass(frozen=True)
class JointLimit:
    lower_rad: float
    upper_rad: float

    @property
    def lower_deg(self) -> float:
        return degrees(self.lower_rad)

    @property
    def upper_deg(self) -> float:
        return degrees(self.upper_rad)


# Values copied from so101_new_calib.urdf.  Only the five joints that affect
# gripper_frame_link belong to the arm IK chain.
URDF_LIMITS = {
    "shoulder_pan": JointLimit(-1.91986, 1.91986),
    "shoulder_lift": JointLimit(-1.74533, 1.74533),
    "elbow_flex": JointLimit(-1.69, 1.69),
    "wrist_flex": JointLimit(-1.65806, 1.65806),
    "wrist_roll": JointLimit(-2.74385, 2.84121),
}


@dataclass(frozen=True)
class MappingIssue:
    level: str
    joint: str
    message: str


class SO101JointMapper:
    """Convert raw SCS215 values, LeRobot values, and URDF joint angles.

    LeRobot's adapted SO follower exposes arm positions in [-100, 100] and the
    gripper in [0, 100].  The arm interval is mapped linearly to each joint's
    URDF limits.  This assumes that the recorded physical endpoints correspond
    to the matching lower/upper URDF endpoints.  Step 2 will verify signs and
    zero alignment with forward kinematics before any Cartesian control is used.
    """

    def __init__(self, calibration: Mapping[str, MotorCalibration]):
        missing = [name for name in ALL_MOTOR_NAMES if name not in calibration]
        if missing:
            raise ValueError(f"Calibration is missing motors: {', '.join(missing)}")
        self.calibration = dict(calibration)

    def validate(self) -> list[MappingIssue]:
        """Return calibration errors and warnings without touching hardware."""
        issues: list[MappingIssue] = []
        seen_ids: dict[int, str] = {}

        for expected_id, name in enumerate(ALL_MOTOR_NAMES, start=1):
            cal = self.calibration[name]

            if cal.id in seen_ids:
                issues.append(
                    MappingIssue(
                        "ERROR",
                        name,
                        f"servo ID {cal.id} is also used by {seen_ids[cal.id]}",
                    )
                )
            seen_ids[cal.id] = name

            if cal.id != expected_id:
                issues.append(
                    MappingIssue(
                        "ERROR",
                        name,
                        f"expected servo ID {expected_id}, got {cal.id}",
                    )
                )

            if not (ENCODER_MIN <= cal.range_min <= ENCODER_MAX):
                issues.append(
                    MappingIssue("ERROR", name, f"range_min={cal.range_min} is outside 0..1023")
                )
            if not (ENCODER_MIN <= cal.range_max <= ENCODER_MAX):
                issues.append(
                    MappingIssue("ERROR", name, f"range_max={cal.range_max} is outside 0..1023")
                )
            if cal.range_min >= cal.range_max:
                issues.append(
                    MappingIssue(
                        "ERROR",
                        name,
                        f"range_min={cal.range_min} must be smaller than range_max={cal.range_max}",
                    )
                )
                continue

            # A range touching both encoder ends usually means the joint crossed
            # the 1023 -> 0 discontinuity while calibration was being recorded.
            near_low_end = cal.range_min <= 16
            near_high_end = cal.range_max >= ENCODER_MAX - 16
            covers_most_encoder = cal.range_max - cal.range_min >= 0.80 * ENCODER_MAX
            if near_low_end and near_high_end and covers_most_encoder:
                issues.append(
                    MappingIssue(
                        "WARNING",
                        name,
                        "range touches both 0 and 1023; probable encoder-boundary crossing",
                    )
                )

        return issues

    def raw_to_normalized(self, joint: str, raw: float, *, clip: bool = True) -> float:
        """Apply the same range normalization used by LeRobot's MotorsBus."""
        cal = self._calibration_for(joint)
        value = self._clip(raw, cal.range_min, cal.range_max) if clip else raw
        fraction = (value - cal.range_min) / (cal.range_max - cal.range_min)

        if joint == GRIPPER_NAME:
            normalized = fraction * 100.0
            return 100.0 - normalized if cal.drive_mode else normalized

        normalized = fraction * 200.0 - 100.0
        return -normalized if cal.drive_mode else normalized

    def normalized_to_raw(self, joint: str, normalized: float, *, clip: bool = True) -> int:
        """Convert a LeRobot position command back to an SCS215 encoder value."""
        cal = self._calibration_for(joint)

        if joint == GRIPPER_NAME:
            value = 100.0 - normalized if cal.drive_mode else normalized
            value = self._clip(value, 0.0, 100.0) if clip else value
            fraction = value / 100.0
        else:
            value = -normalized if cal.drive_mode else normalized
            value = self._clip(value, -100.0, 100.0) if clip else value
            fraction = (value + 100.0) / 200.0

        # int() matches LeRobot's current _unnormalize implementation.
        return int(cal.range_min + fraction * (cal.range_max - cal.range_min))

    def normalized_to_urdf_degrees(
        self, joint: str, normalized: float, *, clip: bool = True
    ) -> float:
        """Map a LeRobot arm value in [-100, 100] to a URDF angle in degrees."""
        limit = self._urdf_limit_for(joint)
        value = self._clip(normalized, -100.0, 100.0) if clip else normalized
        fraction = (value + 100.0) / 200.0
        return limit.lower_deg + fraction * (limit.upper_deg - limit.lower_deg)

    def urdf_degrees_to_normalized(
        self, joint: str, angle_deg: float, *, clip: bool = True
    ) -> float:
        """Map a URDF angle in degrees to LeRobot's [-100, 100] arm range."""
        limit = self._urdf_limit_for(joint)
        value = self._clip(angle_deg, limit.lower_deg, limit.upper_deg) if clip else angle_deg
        fraction = (value - limit.lower_deg) / (limit.upper_deg - limit.lower_deg)
        return fraction * 200.0 - 100.0

    def raw_to_urdf_degrees(self, joint: str, raw: float, *, clip: bool = True) -> float:
        normalized = self.raw_to_normalized(joint, raw, clip=clip)
        return self.normalized_to_urdf_degrees(joint, normalized, clip=clip)

    def urdf_degrees_to_raw(self, joint: str, angle_deg: float, *, clip: bool = True) -> int:
        normalized = self.urdf_degrees_to_normalized(joint, angle_deg, clip=clip)
        return self.normalized_to_raw(joint, normalized, clip=clip)

    def raw_to_urdf_radians(self, joint: str, raw: float, *, clip: bool = True) -> float:
        return radians(self.raw_to_urdf_degrees(joint, raw, clip=clip))

    def urdf_radians_to_raw(self, joint: str, angle_rad: float, *, clip: bool = True) -> int:
        return self.urdf_degrees_to_raw(joint, degrees(angle_rad), clip=clip)

    def _calibration_for(self, joint: str) -> MotorCalibration:
        if joint not in self.calibration:
            raise KeyError(f"Unknown motor: {joint}")
        cal = self.calibration[joint]
        if cal.range_min >= cal.range_max:
            raise ValueError(f"Invalid calibration range for {joint}")
        return cal

    @staticmethod
    def _urdf_limit_for(joint: str) -> JointLimit:
        if joint not in URDF_LIMITS:
            raise ValueError(f"{joint} is not part of the five-joint IK chain")
        return URDF_LIMITS[joint]

    @staticmethod
    def _clip(value: float, lower: float, upper: float) -> float:
        return min(upper, max(lower, value))
