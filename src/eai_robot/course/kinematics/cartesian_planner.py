"""Straight Cartesian waypoint planning with IK failure recovery."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Sequence

import numpy as np

from eai_robot.course.kinematics.kinematics_backend import ForwardKinematics
from eai_robot.course.kinematics.position_ik import IKResult, PositionIK


@dataclass(frozen=True)
class CartesianWaypoint:
    requested_position: np.ndarray
    achieved_position: np.ndarray
    joint_degrees: np.ndarray
    position_error_m: float


@dataclass(frozen=True)
class CartesianPlan:
    start_position: np.ndarray
    requested_target: np.ndarray
    waypoints: tuple[CartesianWaypoint, ...]
    reached_target: bool
    ik_failed: bool
    message: str

    @property
    def final_position(self) -> np.ndarray:
        if self.waypoints:
            return self.waypoints[-1].achieved_position
        return self.start_position

    @property
    def joint_path(self) -> tuple[np.ndarray, ...]:
        return tuple(waypoint.joint_degrees for waypoint in self.waypoints)


class CartesianLinePlanner:
    def __init__(
        self,
        fk: ForwardKinematics,
        ik: PositionIK,
        max_cartesian_step_m: float = 0.003,
        max_joint_step_deg: float = 8.0,
        recovery_iterations: int = 12,
        minimum_recovery_progress_m: float = 0.00025,
    ) -> None:
        if max_cartesian_step_m <= 0:
            raise ValueError("max_cartesian_step_m must be positive")
        self.fk = fk
        self.ik = ik
        self.max_cartesian_step_m = max_cartesian_step_m
        self.max_joint_step_deg = max_joint_step_deg
        self.recovery_iterations = recovery_iterations
        self.minimum_recovery_progress_m = minimum_recovery_progress_m

    def plan(self, start_joint_degrees: Sequence[float], target_position: Sequence[float]) -> CartesianPlan:
        seed = np.asarray(start_joint_degrees, dtype=float).reshape(5)
        start_position = self.fk.forward_kinematics(seed)[:3, 3]
        target = np.asarray(target_position, dtype=float).reshape(3)
        displacement = target - start_position
        distance = float(np.linalg.norm(displacement))

        if distance == 0:
            return CartesianPlan(
                start_position,
                target,
                (),
                True,
                False,
                "target equals current position",
            )

        # The subtraction avoids creating two half-steps when floating-point
        # rounding makes an exact one-step distance infinitesimally too large.
        number_of_steps = max(1, ceil(distance / self.max_cartesian_step_m - 1e-9))
        waypoints: list[CartesianWaypoint] = []
        previous_requested = start_position.copy()

        for index in range(1, number_of_steps + 1):
            requested = start_position + displacement * (index / number_of_steps)
            result = self.ik.solve(requested, seed)

            if self._acceptable(result, seed):
                waypoint = self._waypoint(requested, result)
                waypoints.append(waypoint)
                seed = waypoint.joint_degrees
                previous_requested = requested
                continue

            recovered = self._farthest_feasible(previous_requested, requested, seed)
            if recovered is not None:
                waypoints.append(recovered)

            return CartesianPlan(
                start_position,
                target,
                tuple(waypoints),
                False,
                True,
                f"IK 无解: stopped at the farthest feasible point before waypoint {index}/{number_of_steps}",
            )

        return CartesianPlan(
            start_position,
            target,
            tuple(waypoints),
            True,
            False,
            "target reached",
        )

    def _acceptable(self, result: IKResult, previous_joints: np.ndarray) -> bool:
        if not result.success:
            return False
        if not np.all(np.isfinite(result.joint_degrees)):
            return False
        joint_step = float(np.max(np.abs(result.joint_degrees - previous_joints)))
        return joint_step <= self.max_joint_step_deg

    @staticmethod
    def _waypoint(requested: np.ndarray, result: IKResult) -> CartesianWaypoint:
        return CartesianWaypoint(
            requested.copy(),
            result.achieved_position.copy(),
            result.joint_degrees.copy(),
            result.position_error_m,
        )

    def _farthest_feasible(
        self,
        feasible_start: np.ndarray,
        failed_end: np.ndarray,
        seed: np.ndarray,
    ) -> CartesianWaypoint | None:
        low = 0.0
        high = 1.0
        best: CartesianWaypoint | None = None
        best_seed = seed.copy()
        segment = failed_end - feasible_start

        for _ in range(self.recovery_iterations):
            alpha = (low + high) / 2.0
            requested = feasible_start + alpha * segment
            result = self.ik.solve(requested, best_seed)
            if self._acceptable(result, seed):
                low = alpha
                best = self._waypoint(requested, result)
                best_seed = result.joint_degrees
            else:
                high = alpha

        if best is None:
            return None
        progress = float(np.linalg.norm(best.requested_position - feasible_start))
        return best if progress >= self.minimum_recovery_progress_m else None
