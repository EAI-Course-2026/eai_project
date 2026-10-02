"""Position-only inverse kinematics for the five-joint SO-101 arm."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

import numpy as np

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.course.kinematics.kinematics_backend import ForwardKinematics


@dataclass(frozen=True)
class IKResult:
    success: bool
    joint_degrees: np.ndarray
    achieved_position: np.ndarray
    position_error_m: float
    iterations: int
    message: str


class PositionIK(Protocol):
    def solve(self, target_position: Sequence[float], seed_degrees: Sequence[float]) -> IKResult: ...


def joint_limits_arrays() -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray([URDF_LIMITS[name].lower_deg for name in ARM_JOINT_NAMES], dtype=float)
    upper = np.asarray([URDF_LIMITS[name].upper_deg for name in ARM_JOINT_NAMES], dtype=float)
    return lower, upper


class DampedLeastSquaresIK:
    """Numerical IK using a finite-difference Jacobian and damped least squares."""

    def __init__(
        self,
        fk: ForwardKinematics,
        tolerance_m: float = 0.001,
        max_iterations: int = 120,
        damping: float = 0.02,
        max_joint_update_deg: float = 8.0,
        joint_margin_deg: float = 0.0,
    ) -> None:
        self.fk = fk
        self.tolerance_m = tolerance_m
        self.max_iterations = max_iterations
        self.damping = damping
        self.max_joint_update_rad = np.deg2rad(max_joint_update_deg)
        self.lower_deg, self.upper_deg = joint_limits_arrays()
        self.lower_deg = self.lower_deg + joint_margin_deg
        self.upper_deg = self.upper_deg - joint_margin_deg
        if np.any(self.lower_deg >= self.upper_deg):
            raise ValueError("joint_margin_deg leaves an empty joint range")

    def solve(self, target_position: Sequence[float], seed_degrees: Sequence[float]) -> IKResult:
        target = np.asarray(target_position, dtype=float).reshape(3)
        seed = np.asarray(seed_degrees, dtype=float).reshape(len(ARM_JOINT_NAMES))
        seed = np.clip(seed, self.lower_deg, self.upper_deg)

        # Deterministic alternative seeds help the solver leave a locally
        # singular straight-arm pose while keeping the first attempt continuous.
        seed_offsets = (
            np.zeros(5),
            np.array([0.0, 5.0, -10.0, 5.0, 0.0]),
            np.array([0.0, -5.0, 10.0, -5.0, 0.0]),
        )
        results = [
            self._solve_once(target, np.clip(seed + offset, self.lower_deg, self.upper_deg), seed)
            for offset in seed_offsets
        ]
        successful = [result for result in results if result.success]
        if successful:
            # Every candidate already satisfies the Cartesian tolerance. Prefer
            # the one nearest the previous command so a marginally smaller FK
            # residual cannot cause a large branch change in joint space.
            return min(
                successful,
                key=lambda result: (
                    float(np.max(np.abs(result.joint_degrees - seed))),
                    result.position_error_m,
                ),
            )
        return min(results, key=lambda result: result.position_error_m)

    def _solve_once(
        self,
        target: np.ndarray,
        initial_degrees: np.ndarray,
        preferred_degrees: np.ndarray,
    ) -> IKResult:
        q_rad = np.deg2rad(initial_degrees)
        preferred_rad = np.deg2rad(preferred_degrees)
        lower_rad = np.deg2rad(self.lower_deg)
        upper_rad = np.deg2rad(self.upper_deg)
        best_q = q_rad.copy()
        best_error = float("inf")
        stagnant_iterations = 0

        for iteration in range(1, self.max_iterations + 1):
            position = self._position(q_rad)
            error_vector = target - position
            error_norm = float(np.linalg.norm(error_vector))

            if error_norm < best_error:
                best_error = error_norm
                best_q = q_rad.copy()
            if error_norm <= self.tolerance_m:
                return IKResult(
                    True,
                    np.rad2deg(q_rad),
                    position,
                    error_norm,
                    iteration,
                    "converged",
                )

            jacobian = self._numerical_jacobian(q_rad, lower_rad, upper_rad)
            regularized = jacobian @ jacobian.T + (self.damping**2) * np.eye(3)
            try:
                task_step = jacobian.T @ np.linalg.solve(regularized, error_vector)
                pseudo_inverse = jacobian.T @ np.linalg.solve(regularized, np.eye(3))
            except np.linalg.LinAlgError:
                break

            # Preserve the seed posture in the Jacobian null space. This keeps
            # the two redundant DOFs from drifting while position is solved.
            null_space = np.eye(len(q_rad)) - pseudo_inverse @ jacobian
            posture_step = 0.02 * (null_space @ (preferred_rad - q_rad))
            step = task_step + posture_step
            step = np.clip(step, -self.max_joint_update_rad, self.max_joint_update_rad)

            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.1):
                candidate = np.clip(q_rad + scale * step, lower_rad, upper_rad)
                candidate_error = float(np.linalg.norm(target - self._position(candidate)))
                if candidate_error + 1e-9 < error_norm:
                    q_rad = candidate
                    accepted = True
                    stagnant_iterations = 0
                    break

            if not accepted:
                stagnant_iterations += 1
                if stagnant_iterations >= 3:
                    break

        best_position = self._position(best_q)
        return IKResult(
            False,
            np.rad2deg(best_q),
            best_position,
            float(np.linalg.norm(target - best_position)),
            self.max_iterations,
            "position tolerance was not reached",
        )

    def _position(self, q_rad: np.ndarray) -> np.ndarray:
        return self.fk.forward_kinematics(np.rad2deg(q_rad))[:3, 3]

    def _numerical_jacobian(
        self,
        q_rad: np.ndarray,
        lower_rad: np.ndarray,
        upper_rad: np.ndarray,
    ) -> np.ndarray:
        epsilon = 1e-4
        jacobian = np.zeros((3, len(q_rad)), dtype=float)

        for index in range(len(q_rad)):
            q_plus = q_rad.copy()
            q_minus = q_rad.copy()
            q_plus[index] = min(q_plus[index] + epsilon, upper_rad[index])
            q_minus[index] = max(q_minus[index] - epsilon, lower_rad[index])
            denominator = q_plus[index] - q_minus[index]
            if denominator == 0:
                continue
            jacobian[:, index] = (
                self._position(q_plus) - self._position(q_minus)
            ) / denominator
        return jacobian


class LeRobotPlacoPositionIK:
    """Validation wrapper around LeRobot's Placo IK implementation."""

    def __init__(
        self,
        kinematics: ForwardKinematics,
        tolerance_m: float = 0.001,
        joint_margin_deg: float = 0.0,
    ) -> None:
        self.kinematics = kinematics
        self.tolerance_m = tolerance_m
        self.lower_deg, self.upper_deg = joint_limits_arrays()
        self.lower_deg = self.lower_deg + joint_margin_deg
        self.upper_deg = self.upper_deg - joint_margin_deg

    def solve(self, target_position: Sequence[float], seed_degrees: Sequence[float]) -> IKResult:
        target = np.asarray(target_position, dtype=float).reshape(3)
        seed = np.asarray(seed_degrees, dtype=float).reshape(5)
        desired_pose = self.kinematics.forward_kinematics(seed).copy()
        desired_pose[:3, 3] = target
        solution = self.kinematics.inverse_kinematics(
            seed,
            desired_pose,
            orientation_weight=0.0,
            max_iters=30,
        )
        solution = np.asarray(solution[:5], dtype=float)
        achieved = self.kinematics.forward_kinematics(solution)[:3, 3]
        error = float(np.linalg.norm(target - achieved))
        inside_limits = bool(
            np.all(solution >= self.lower_deg) and np.all(solution <= self.upper_deg)
        )
        success = bool(np.all(np.isfinite(solution)) and inside_limits and error <= self.tolerance_m)
        return IKResult(
            success,
            solution,
            achieved,
            error,
            30,
            "converged" if success else "Placo result failed FK residual or joint-limit validation",
        )


def create_position_ik_solver(
    fk_solver: ForwardKinematics,
    backend_name: str,
    tolerance_m: float = 0.001,
    joint_margin_deg: float = 0.0,
) -> tuple[PositionIK, str]:
    if backend_name == "lerobot-placo":
        return (
            LeRobotPlacoPositionIK(fk_solver, tolerance_m, joint_margin_deg),
            "lerobot-placo-position-ik",
        )
    return (
        DampedLeastSquaresIK(
            fk_solver,
            tolerance_m=tolerance_m,
            joint_margin_deg=joint_margin_deg,
        ),
        "numpy-dls-position-ik",
    )
