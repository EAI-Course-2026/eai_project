"""Project fixed 3-D targets through real URDF FK to validate the joint adapter."""
import unittest

import numpy as np

from eai_robot.config import ROOT
from eai_robot.control.kinematics.kinematics_backend import create_fk_solver
from eai_robot.control.visual_servo.aruco_ibvs import point_interaction_matrix
from eai_robot.control.visual_servo.joint_adapter import (
    camera_joint_jacobian, image_joint_step, validate_transform,
)


class JointServoTests(unittest.TestCase):
    def setUp(self):
        self.fk, _ = create_fk_solver(ROOT / "src/eai_robot/control/kinematics/so101_new_calib.urdf")
        self.mount = np.eye(4)
        self.mount[:3, 3] = [.02, .01, -.06]
        self.K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
        desired = self.fk.forward_kinematics(np.zeros(5)) @ self.mount
        local = np.array([[-.0125, -.0125, .25, 1], [.0125, -.0125, .25, 1],
                          [.0125, .0125, .25, 1], [-.0125, .0125, .25, 1]])
        self.world = (desired @ local.T).T

    def observe(self, q):
        camera = self.fk.forward_kinematics(q) @ self.mount
        local = (np.linalg.inv(camera) @ self.world.T).T[:, :3]
        image = (self.K @ local.T).T
        return image[:, :2] / image[:, 2:], local[:, 2]

    def test_joint_image_jacobian_matches_full_fk_projection(self):
        q = np.array([2., -2, 2, -2, 2])
        pixels, depths = self.observe(q)
        _, jacobian = camera_joint_jacobian(self.fk, q, self.mount)
        image_jacobian = point_interaction_matrix(pixels, depths, self.K) @ jacobian
        epsilon = 1e-6
        for index in range(5):
            offset = np.eye(5)[index] * np.rad2deg(epsilon)
            derivative = ((self.observe(q + offset)[0] - self.observe(q - offset)[0])
                          / (2 * epsilon)).reshape(-1)
            np.testing.assert_allclose(image_jacobian[:, index], derivative, atol=1e-4)

    def test_reachable_goal_converges_with_bounded_steps(self):
        q = np.array([2., -2, 2, -2, 2])
        first_error = None
        for _ in range(120):
            pixels, depths = self.observe(q)
            step = image_joint_step(self.fk, q, self.mount, pixels, depths, self.K,
                                    (640, 480), target_side_px=60)
            first_error = step.error_rms_px if first_error is None else first_error
            self.assertLessEqual(np.max(np.abs(step.target_degrees - q)), .400001)
            self.assertLessEqual(step.camera_translation_m, .001001)
            self.assertLessEqual(step.camera_rotation_rad, .010001)
            self.assertLessEqual(step.predicted_error_rms_px, step.error_rms_px + 1e-9)
            q = step.target_degrees
        self.assertLess(step.error_rms_px, first_error * .06)

    def test_bad_mount_and_start_at_limit_rejected(self):
        for bad in (np.zeros((4, 4)), np.full((4, 4), np.nan)):
            with self.assertRaises(ValueError):
                validate_transform(bad)
        pixels, depths = self.observe(np.zeros(5))
        with self.assertRaisesRegex(ValueError, "margin"):
            image_joint_step(self.fk, [0, 100, 0, 0, 0], self.mount,
                            pixels, depths, self.K, (640, 480))
        with self.assertRaises(ValueError):
            image_joint_step(self.fk, np.zeros(5), self.mount, pixels, depths,
                            self.K, (640, 480), period_s=float("nan"))
