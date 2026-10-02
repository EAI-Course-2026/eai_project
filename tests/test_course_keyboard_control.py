from __future__ import annotations

import unittest

import numpy as np

from eai_robot.course.kinematics.keyboard_control import direction_from_keys


class KeyboardDirectionTest(unittest.TestCase):
    def test_single_axis_keys(self) -> None:
        np.testing.assert_allclose(direction_from_keys({"w"}), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"s"}), [-1.0, 0.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"a"}), [0.0, 1.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"d"}), [0.0, -1.0, 0.0])
        np.testing.assert_allclose(direction_from_keys({"r"}), [0.0, 0.0, 1.0])
        np.testing.assert_allclose(direction_from_keys({"f"}), [0.0, 0.0, -1.0])

    def test_opposite_keys_cancel(self) -> None:
        np.testing.assert_allclose(direction_from_keys({"w", "s", "r", "f"}), [0.0, 0.0, 0.0])

    def test_diagonal_is_normalized(self) -> None:
        direction = direction_from_keys({"w", "a", "r"})
        self.assertAlmostEqual(float(np.linalg.norm(direction)), 1.0)
        self.assertTrue(np.all(direction > 0))


if __name__ == "__main__":
    unittest.main()
