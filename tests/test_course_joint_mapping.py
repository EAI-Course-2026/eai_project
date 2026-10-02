from __future__ import annotations

import unittest

from lerobot.motors import MotorCalibration

from eai_robot.course.kinematics.joint_mapping import ALL_MOTOR_NAMES, ARM_JOINT_NAMES, SO101JointMapper, URDF_LIMITS


def make_calibration() -> dict[str, MotorCalibration]:
    ranges = {
        "shoulder_pan": (183, 897),
        "shoulder_lift": (64, 737),
        "elbow_flex": (196, 769),
        "wrist_flex": (70, 681),
        "wrist_roll": (4, 1014),
        "gripper": (237, 644),
    }
    return {
        name: MotorCalibration(index, 0, 0, *ranges[name])
        for index, name in enumerate(ALL_MOTOR_NAMES, start=1)
    }


class JointMappingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = SO101JointMapper(make_calibration())

    def test_arm_raw_endpoints_match_urdf_limits(self) -> None:
        for name in ARM_JOINT_NAMES:
            cal = self.mapper.calibration[name]
            limit = URDF_LIMITS[name]
            self.assertAlmostEqual(
                self.mapper.raw_to_urdf_degrees(name, cal.range_min), limit.lower_deg
            )
            self.assertAlmostEqual(
                self.mapper.raw_to_urdf_degrees(name, cal.range_max), limit.upper_deg
            )

    def test_degree_round_trip_is_within_one_encoder_tick(self) -> None:
        for name in ARM_JOINT_NAMES:
            raw = self.mapper.urdf_degrees_to_raw(name, 0.0)
            one_tick_deg = 300.0 / 1023.0
            self.assertLess(abs(self.mapper.raw_to_urdf_degrees(name, raw)), one_tick_deg)

    def test_gripper_uses_zero_to_one_hundred(self) -> None:
        cal = self.mapper.calibration["gripper"]
        self.assertEqual(self.mapper.raw_to_normalized("gripper", cal.range_min), 0.0)
        self.assertEqual(self.mapper.raw_to_normalized("gripper", cal.range_max), 100.0)

    def test_wrist_roll_boundary_crossing_is_reported(self) -> None:
        warnings = [
            issue
            for issue in self.mapper.validate()
            if issue.level == "WARNING" and issue.joint == "wrist_roll"
        ]
        self.assertEqual(len(warnings), 1)


if __name__ == "__main__":
    unittest.main()
