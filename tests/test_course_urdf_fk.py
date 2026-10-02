from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.course.kinematics.urdf_fk import URDFFK


URDF_PATH = Path(__file__).resolve().parents[1] / "src/eai_robot/course/kinematics/so101_new_calib.urdf"


class URDFFKTest(unittest.TestCase):
    def setUp(self) -> None:
        self.fk = URDFFK(URDF_PATH, "gripper_frame_link", ARM_JOINT_NAMES)

    def test_chain_contains_five_arm_joints_and_fixed_tcp(self) -> None:
        self.assertEqual(
            [joint.name for joint in self.fk.chain],
            [*ARM_JOINT_NAMES, "gripper_frame_joint"],
        )

    def test_limits_are_read_from_urdf(self) -> None:
        parsed = self.fk.joint_limits_degrees()
        for name in ARM_JOINT_NAMES:
            self.assertAlmostEqual(parsed[name][0], URDF_LIMITS[name].lower_deg)
            self.assertAlmostEqual(parsed[name][1], URDF_LIMITS[name].upper_deg)

    def test_zero_pose_is_a_rigid_transform(self) -> None:
        transform = self.fk.forward_kinematics(np.zeros(5))
        rotation = transform[:3, :3]
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0, places=6)
        np.testing.assert_allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

    def test_shoulder_pan_preserves_radius_and_height(self) -> None:
        zero_position = self.fk.forward_kinematics(np.zeros(5))[:3, 3]
        turned = np.zeros(5)
        turned[0] = 30.0
        turned_position = self.fk.forward_kinematics(turned)[:3, 3]
        shoulder_axis_xy = self.fk.chain[0].xyz[:2]
        zero_radius = np.linalg.norm(zero_position[:2] - shoulder_axis_xy)
        turned_radius = np.linalg.norm(turned_position[:2] - shoulder_axis_xy)
        self.assertAlmostEqual(float(zero_radius), float(turned_radius), places=6)
        self.assertAlmostEqual(float(zero_position[2]), float(turned_position[2]), places=6)


if __name__ == "__main__":
    unittest.main()
