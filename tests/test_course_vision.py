#!/usr/bin/env python

import unittest

import cv2
import numpy as np

from eai_robot.course.vision.ball_detector import BlackBallDetector
from eai_robot.course.vision.visual_math import axis_delta, damped_image_step, validate_image_jacobian


class BlackBallDetectorTest(unittest.TestCase):
    def test_detects_black_circle_in_rgb_frame(self) -> None:
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (430, 170), 55, (0, 0, 0), thickness=-1)
        detection, _ = BlackBallDetector().detect(frame)
        self.assertIsNotNone(detection)
        assert detection is not None
        self.assertAlmostEqual(detection.center_x, 430, delta=2)
        self.assertAlmostEqual(detection.center_y, 170, delta=2)
        self.assertAlmostEqual(detection.radius, 55, delta=3)
        self.assertGreater(detection.confidence, 0.7)

    def test_rejects_long_black_bar(self) -> None:
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        cv2.rectangle(frame, (80, 210), (560, 250), (0, 0, 0), thickness=-1)
        detection, _ = BlackBallDetector().detect(frame)
        self.assertIsNone(detection)

    def test_detects_circle_nested_inside_dark_monitor_surroundings(self) -> None:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # This white screen boundary is deliberately below the old 35% area
        # ceiling and would otherwise outrank the smaller ball by area.
        cv2.rectangle(frame, (180, 100), (460, 380), (255, 255, 255), thickness=-1)
        cv2.circle(frame, (330, 235), 45, (0, 0, 0), thickness=-1)
        detection, _ = BlackBallDetector().detect(frame)
        self.assertIsNotNone(detection)
        assert detection is not None
        self.assertAlmostEqual(detection.center_x, 330, delta=2)
        self.assertAlmostEqual(detection.center_y, 235, delta=2)
        self.assertAlmostEqual(detection.radius, 45, delta=3)

    def test_accepts_configured_150_px_stop_radius(self) -> None:
        frame = np.full((480, 640, 3), 255, dtype=np.uint8)
        cv2.circle(frame, (320, 240), 150, (0, 0, 0), thickness=-1)
        detection, _ = BlackBallDetector().detect(frame)
        self.assertIsNotNone(detection)
        assert detection is not None
        self.assertAlmostEqual(detection.radius, 150, delta=3)


class VisualMathTest(unittest.TestCase):
    def test_axis_delta_uses_requested_axes(self) -> None:
        delta = axis_delta(("y", "z"), np.array([0.002, -0.003]))
        np.testing.assert_allclose(delta, [0.0, 0.002, -0.003])

    def test_damped_step_inverts_image_motion(self) -> None:
        jacobian = np.array([[1000.0, 0.0], [0.0, -1000.0]])
        step = damped_image_step(
            jacobian,
            np.array([10.0, -20.0]),
            gain=1.0,
            damping_px_per_m=0.0,
            max_step_m=0.1,
        )
        np.testing.assert_allclose(step, [0.01, 0.02], atol=1e-9)

    def test_step_is_limited_by_norm(self) -> None:
        step = damped_image_step(
            np.eye(2),
            np.array([3.0, 4.0]),
            gain=1.0,
            damping_px_per_m=0.0,
            max_step_m=0.002,
        )
        self.assertAlmostEqual(float(np.linalg.norm(step)), 0.002, places=12)

    def test_rejects_degenerate_jacobian(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "condition"):
            validate_image_jacobian(np.array([[100.0, 100.0], [0.0, 0.0]]))


if __name__ == "__main__":
    unittest.main()
