from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from eai_robot.course.kinematics.cartesian_planner import CartesianLinePlanner
from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.course.kinematics.position_ik import DampedLeastSquaresIK
from eai_robot.course.kinematics.urdf_fk import URDFFK


URDF_PATH = Path(__file__).resolve().parents[1] / "src/eai_robot/course/kinematics/so101_new_calib.urdf"


class Steps3To5Test(unittest.TestCase):
    def setUp(self) -> None:
        self.fk = URDFFK(URDF_PATH, "gripper_frame_link", ARM_JOINT_NAMES)
        self.ik = DampedLeastSquaresIK(self.fk)

    def test_single_reachable_position_ik(self) -> None:
        target_joints = np.array([15.0, -35.0, 45.0, -20.0, 10.0])
        target_position = self.fk.forward_kinematics(target_joints)[:3, 3]
        seed = np.array([10.0, -30.0, 40.0, -15.0, 5.0])
        result = self.ik.solve(target_position, seed)

        self.assertTrue(result.success, result.message)
        self.assertLessEqual(result.position_error_m, 0.001)
        for name, value in zip(ARM_JOINT_NAMES, result.joint_degrees, strict=True):
            self.assertGreaterEqual(value, URDF_LIMITS[name].lower_deg)
            self.assertLessEqual(value, URDF_LIMITS[name].upper_deg)

    def test_reachable_path_tracks_a_straight_line(self) -> None:
        start_joints = np.array([0.0, -25.0, 40.0, -15.0, 0.0])
        start_position = self.fk.forward_kinematics(start_joints)[:3, 3]
        target_position = start_position + np.array([0.0, 0.0, 0.015])
        planner = CartesianLinePlanner(self.fk, self.ik, max_cartesian_step_m=0.003)
        plan = planner.plan(start_joints, target_position)

        self.assertTrue(plan.reached_target, plan.message)
        self.assertGreater(len(plan.waypoints), 1)
        direction = target_position - start_position
        direction /= np.linalg.norm(direction)
        for waypoint in plan.waypoints:
            offset = waypoint.achieved_position - start_position
            perpendicular = offset - np.dot(offset, direction) * direction
            self.assertLess(float(np.linalg.norm(perpendicular)), 0.0015)

    def test_unreachable_target_stops_at_farthest_feasible_point(self) -> None:
        start_joints = np.array([0.0, -25.0, 40.0, -15.0, 0.0])
        start_position = self.fk.forward_kinematics(start_joints)[:3, 3]
        unreachable = start_position + np.array([0.0, 0.0, 1.0])
        planner = CartesianLinePlanner(self.fk, self.ik, max_cartesian_step_m=0.02)
        plan = planner.plan(start_joints, unreachable)

        self.assertFalse(plan.reached_target)
        self.assertTrue(plan.ik_failed)
        self.assertIn("IK 无解", plan.message)
        self.assertGreater(len(plan.waypoints), 0)
        requested_distance = np.linalg.norm(unreachable - start_position)
        achieved_distance = np.linalg.norm(plan.final_position - start_position)
        self.assertLess(achieved_distance, requested_distance)

    def test_live_control_can_move_one_mm_in_all_six_directions(self) -> None:
        start_joints = np.array([0.0, -25.0, 40.0, -15.0, 0.0])
        start_position = self.fk.forward_kinematics(start_joints)[:3, 3]
        live_ik = DampedLeastSquaresIK(
            self.fk,
            tolerance_m=0.0002,
            joint_margin_deg=2.0,
        )
        planner = CartesianLinePlanner(
            self.fk,
            live_ik,
            max_cartesian_step_m=0.001,
            max_joint_step_deg=5.0,
        )

        for direction in (
            np.array([1.0, 0.0, 0.0]),
            np.array([-1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, -1.0, 0.0]),
            np.array([0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, -1.0]),
        ):
            with self.subTest(direction=direction):
                plan = planner.plan(start_joints, start_position + direction * 0.001)
                self.assertTrue(plan.reached_target, plan.message)
                self.assertLess(
                    float(np.max(np.abs(plan.waypoints[-1].joint_degrees - start_joints))),
                    5.0,
                )


if __name__ == "__main__":
    unittest.main()
