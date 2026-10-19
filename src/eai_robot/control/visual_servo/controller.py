"""Pure visual-servo proposals. No device connection or motion execution."""
from dataclasses import dataclass

import numpy as np

from eai_robot.control.visual_servo.aruco_ibvs import (
    corner_error_px,
    desired_square_corners,
    point_interaction_matrix,
    solve_camera_twist,
)
from eai_robot.vision.features import VisualFeatures


@dataclass(frozen=True)
class ServoProposal:
    status: str
    camera_delta_m: np.ndarray | None = None
    camera_twist: np.ndarray | None = None
    error_rms_px: float | None = None


def observation_status(features, *, now, max_age_s=0.25, expected_target=None):
    if not np.isfinite(now) or not np.isfinite(max_age_s) or max_age_s <= 0:
        raise ValueError("time must be finite and max_age_s positive")
    if features is None:
        return "target_lost"
    age = now - features.captured_at
    if age < 0 or age > max_age_s:
        return "stale_frame"
    if expected_target is not None and features.target_id != expected_target:
        return "target_changed"
    return None


def simple_step(
    features: VisualFeatures | None, *, now: float, step_m=0.01,
    max_age_s=0.25, expected_target=None,
) -> ServoProposal:
    """Move on one camera axis per frame; central 20% requests forward motion.

    Camera optical axes: x right, y down, z forward. Outside the central box,
    choose the largest normalized center error (x wins ties). Forward motion
    is only a proposal: a real executor must impose distance/travel limits.
    """
    if not np.isfinite(step_m) or not 0 < step_m <= 0.01:
        raise ValueError("step_m must be within (0, 0.01]")
    status = observation_status(features, now=now, max_age_s=max_age_s,
                                expected_target=expected_target)
    if status:
        return ServoProposal(status)
    center = features.center_px / np.array(features.image_size)
    delta = np.zeros(3)
    outside = (center < 0.4) | (center > 0.6)
    if outside.any():
        scores = np.where(outside, np.abs(center - 0.5), -1)
        axis = int(np.argmax(scores))
        delta[axis] = np.sign(center[axis] - 0.5) * step_m
        return ServoProposal("align", camera_delta_m=delta)
    delta[2] = step_m
    return ServoProposal("approach", camera_delta_m=delta)


def ibvs_step(
    features: VisualFeatures | None, *, now: float, camera_matrix,
    depths_m, target_side_px=120.0, tolerance_px=3.0,
    max_age_s=0.25, expected_target=None,
) -> ServoProposal:
    """Four corresponding, UNDISTORTED pixel features -> camera velocity.

    Intrinsics must describe the supplied image resolution. Distortion removal
    and calibrated depth estimation belong to the observation adapter.
    """
    if not np.isfinite(tolerance_px) or tolerance_px <= 0:
        raise ValueError("tolerance_px must be positive")
    status = observation_status(features, now=now, max_age_s=max_age_s,
                                expected_target=expected_target)
    if status:
        return ServoProposal(status)
    target = desired_square_corners(*features.image_size, target_side_px)
    error = corner_error_px(features.points_px, target)
    # Validate calibration/depth even when the image already meets tolerance.
    matrix = point_interaction_matrix(features.points_px, depths_m, camera_matrix)
    rms = float(np.sqrt(np.mean(error**2)))
    if np.max(np.abs(error)) <= tolerance_px:
        return ServoProposal("aligned", camera_twist=np.zeros(6), error_rms_px=rms)
    return ServoProposal("tracking", camera_twist=solve_camera_twist(matrix, error),
                         error_rms_px=rms)
