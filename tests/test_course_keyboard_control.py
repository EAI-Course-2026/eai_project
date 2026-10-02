from __future__ import annotations

import unittest
from unittest.mock import Mock

import numpy as np

from eai_robot.course.kinematics.keyboard_control import (
    direction_from_keys,
    read_motion_feedback,
)
from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES


class KeyboardDirectionTest(unittest.TestCase):
    def test_feedback_rejects_unreceived_goal(self):
        robot = Mock()
        mapper = Mock()
        mapper.urdf_degrees_to_raw.return_value = 500

        def read(register, name, **kwargs):
            return {"Present_Position": 400, "Torque_Enable": 1, "Goal_Position": 400}[
                register
            ]

        robot.bus.read.side_effect = read
        with self.assertRaisesRegex(
            RuntimeError, "goal was not accepted.*sent raw=500"
        ):
            read_motion_feedback(robot, mapper, np.zeros(5))

    def test_feedback_rejects_lost_torque(self):
        robot = Mock()
        robot.bus.read.return_value = 0
        with self.assertRaisesRegex(RuntimeError, "torque feedback is 0"):
            read_motion_feedback(robot, Mock(), np.zeros(5))

    def test_feedback_returns_actual_positions_when_goals_accepted(self):
        robot = Mock()
        mapper = Mock()
        mapper.urdf_degrees_to_raw.return_value = 500
        robot.bus.read.side_effect = lambda register, name, **kwargs: {
            "Present_Position": 400,
            "Torque_Enable": 1,
            "Goal_Position": 500,
        }[register]
        self.assertEqual(
            read_motion_feedback(robot, mapper, np.zeros(5)),
            dict.fromkeys(ARM_JOINT_NAMES, 400),
        )

    def test_single_axis_keys(self) -> None:
        np.testing.assert_allclose(direction_from_keys({"w"}), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"s"}), [-1.0, 0.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"a"}), [0.0, 1.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"d"}), [0.0, -1.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"r"}), [0.0, 0.0, 1.0])
        np.testing.assert_allclose(direction_from_keys({"f"}), [0.0, 0.0, -1.0])

    def test_opposite_keys_cancel(self) -> None:
        np.testing.assert_allclose(
            direction_from_keys({"w", "s", "r", "f"}), [0.0, 0.0, 0.0]
        )

    def test_diagonal_is_normalized(self) -> None:
        direction = direction_from_keys({"w", "a", "r"})
        self.assertAlmostEqual(float(np.linalg.norm(direction)), 1.0)
        self.assertTrue(np.all(direction > 0))


if __name__ == "__main__":
    unittest.main()
