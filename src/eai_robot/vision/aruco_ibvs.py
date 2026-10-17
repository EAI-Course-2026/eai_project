"""Pure image-based visual-servo math for a four-corner ArUco target.

Camera velocities are expressed in the camera frame as
``[vx, vy, vz, wx, wy, wz]``. This module does not connect to robot hardware.
"""

from __future__ import annotations

import numpy as np


def desired_square_corners(
    image_width: int,
    image_height: int,
    side_px: float,
) -> np.ndarray:
    """Return centered target corners in OpenCV's TL, TR, BR, BL order."""
    if image_width <= 0 or image_height <= 0 or not np.isfinite(side_px) or side_px <= 0:
        raise ValueError("image dimensions and side_px must be positive")
    if side_px > min(image_width, image_height):
        raise ValueError("target square must fit inside the image")
    cx, cy = image_width / 2.0, image_height / 2.0
    half = side_px / 2.0
    return np.array(
        [[cx - half, cy - half], [cx + half, cy - half],
         [cx + half, cy + half], [cx - half, cy + half]],
        dtype=float,
    )


def corner_error_px(current_corners_px: np.ndarray, target_corners_px: np.ndarray) -> np.ndarray:
    """Flatten current-minus-target corner errors into an eight-vector."""
    current = np.asarray(current_corners_px, dtype=float).reshape(4, 2)
    target = np.asarray(target_corners_px, dtype=float).reshape(4, 2)
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError("corner coordinates must be finite")
    return (current - target).reshape(8)


def point_interaction_matrix(
    points_px: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
) -> np.ndarray:
    """Build the pixel-coordinate interaction matrix for point features.

    The returned matrix maps camera-frame translational/rotational velocity to
    pixel velocity. Depth is required for each point; callers must fail closed
    when pose estimation or calibration is unavailable.
    """
    points = np.asarray(points_px, dtype=float).reshape(-1, 2)
    depths = np.asarray(depths_m, dtype=float).reshape(-1)
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if len(points) == 0 or len(points) != len(depths):
        raise ValueError("each image point must have one positive depth")
    if not np.all(np.isfinite(points)) or not np.all(np.isfinite(depths)):
        raise ValueError("points and depths must be finite")
    if np.any(depths <= 0):
        raise ValueError("point depths must be positive meters")
    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    if not np.all(np.isfinite(matrix)) or fx <= 0 or fy <= 0:
        raise ValueError("camera matrix must contain finite positive focal lengths")

    rows = np.empty((2 * len(points), 6), dtype=float)
    for index, ((u, v), depth) in enumerate(zip(points, depths, strict=True)):
        x, y = (u - cx) / fx, (v - cy) / fy
        rows[2 * index] = [
            -fx / depth, 0.0, fx * x / depth,
            fx * x * y, -fx * (1.0 + x * x), fx * y,
        ]
        rows[2 * index + 1] = [
            0.0, -fy / depth, fy * y / depth,
            fy * (1.0 + y * y), -fy * x * y, -fy * x,
        ]
    return rows


def solve_camera_twist(
    interaction_matrix: np.ndarray,
    error_px: np.ndarray,
    *,
    gain: float = 0.5,
    damping: float = 1e-3,
    max_linear_m_s: float = 0.02,
    max_angular_rad_s: float = 0.15,
) -> np.ndarray:
    """Return a bounded damped-least-squares camera-frame velocity command."""
    jacobian = np.asarray(interaction_matrix, dtype=float)
    error = np.asarray(error_px, dtype=float).reshape(-1)
    if (
        jacobian.ndim != 2
        or jacobian.shape != (len(error), 6)
        or len(error) < 6
        or len(error) % 2
    ):
        raise ValueError("interaction matrix must have shape (2N, 6), with N >= 3")
    if not np.all(np.isfinite(jacobian)) or not np.all(np.isfinite(error)):
        raise ValueError("interaction matrix and image error must be finite")
    parameters = np.asarray(
        [gain, damping, max_linear_m_s, max_angular_rad_s], dtype=float
    )
    if not np.all(np.isfinite(parameters)) or np.any(parameters <= 0):
        raise ValueError("gain, damping, and velocity limits must be positive")

    normal = jacobian.T @ jacobian + damping**2 * np.eye(6)
    try:
        twist = -gain * np.linalg.solve(normal, jacobian.T @ error)
    except np.linalg.LinAlgError as exc:
        raise RuntimeError("could not solve the damped image interaction system") from exc
    if not np.all(np.isfinite(twist)):
        raise RuntimeError("image interaction solve produced a non-finite camera velocity")

    linear_norm = float(np.linalg.norm(twist[:3]))
    if linear_norm > max_linear_m_s:
        twist[:3] *= max_linear_m_s / linear_norm
    angular_norm = float(np.linalg.norm(twist[3:]))
    if angular_norm > max_angular_rad_s:
        twist[3:] *= max_angular_rad_s / angular_norm
    return twist
