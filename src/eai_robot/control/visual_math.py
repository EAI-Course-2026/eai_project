"""Pure numerical helpers for image-based visual servoing."""

from __future__ import annotations

import numpy as np


AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def axis_delta(axis_names: tuple[str, str], values_m: np.ndarray) -> np.ndarray:
    if len(set(axis_names)) != 2 or any(axis not in AXIS_INDEX for axis in axis_names):
        raise ValueError("axis_names must contain two different values from x, y, z")
    values = np.asarray(values_m, dtype=float).reshape(2)
    delta = np.zeros(3, dtype=float)
    for axis, value in zip(axis_names, values, strict=True):
        delta[AXIS_INDEX[axis]] = value
    return delta


def damped_image_step(
    jacobian_px_per_m: np.ndarray,
    pixel_error: np.ndarray,
    *,
    gain: float,
    damping_px_per_m: float,
    max_step_m: float,
) -> np.ndarray:
    """Map desired pixel motion to a bounded two-axis Cartesian displacement."""
    jacobian = np.asarray(jacobian_px_per_m, dtype=float).reshape(2, 2)
    error = np.asarray(pixel_error, dtype=float).reshape(2)
    if not np.all(np.isfinite(jacobian)) or not np.all(np.isfinite(error)):
        raise ValueError("jacobian and pixel_error must be finite")
    if gain <= 0 or damping_px_per_m < 0 or max_step_m <= 0:
        raise ValueError("gain/max_step must be positive and damping non-negative")

    regularized = jacobian @ jacobian.T + (damping_px_per_m**2) * np.eye(2)
    try:
        step = gain * jacobian.T @ np.linalg.solve(regularized, error)
    except np.linalg.LinAlgError as exc:
        raise RuntimeError("image Jacobian is singular") from exc

    norm = float(np.linalg.norm(step))
    if norm > max_step_m:
        step *= max_step_m / norm
    return step


def validate_image_jacobian(
    jacobian_px_per_m: np.ndarray,
    *,
    min_column_norm_px_per_m: float = 100.0,
    max_condition_number: float = 80.0,
) -> None:
    jacobian = np.asarray(jacobian_px_per_m, dtype=float).reshape(2, 2)
    column_norms = np.linalg.norm(jacobian, axis=0)
    if np.any(column_norms < min_column_norm_px_per_m):
        raise RuntimeError(
            "探测动作在图像中产生的位移太小；请增大 --probe-mm、调整相机，"
            "或选择另外两个 --control-axes"
        )
    condition = float(np.linalg.cond(jacobian))
    if not np.isfinite(condition) or condition > max_condition_number:
        raise RuntimeError(
            f"图像雅可比矩阵病态（condition={condition:.1f}）；两个控制方向在图像中近似共线"
        )
