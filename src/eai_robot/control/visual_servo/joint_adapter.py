"""Constrained image-to-joint steps for the five-axis arm, without device I/O."""
from dataclasses import dataclass

import numpy as np

from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.control.visual_servo.aruco_ibvs import (
    corner_error_px, desired_square_corners, point_interaction_matrix,
)


def validate_transform(value):
    transform = np.asarray(value, dtype=float)
    if (transform.shape != (4, 4) or not np.all(np.isfinite(transform))
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1., atol=1e-6)):
        raise ValueError("expected a rigid destination_T_source transform in meters")
    return transform


def camera_joint_jacobian(fk, joint_degrees, gripper_T_camera):
    """Map joint rad/s to optical camera [m/s, rad/s] at its own origin.

    FK must return base_T_gripper for the exact frame used by hand-eye. Numeric
    differentiation of base_T_camera includes the camera mounting lever arm.
    """
    q = np.asarray(joint_degrees, dtype=float)
    if q.shape != (5,) or not np.all(np.isfinite(q)):
        raise ValueError("five finite joint angles in degrees are required")
    mount = validate_transform(gripper_T_camera)
    current = validate_transform(fk.forward_kinematics(q)) @ mount
    rotation = current[:3, :3]
    jacobian = np.empty((6, 5))
    epsilon = 1e-5
    for index in range(5):
        offset = np.eye(5)[index] * np.rad2deg(epsilon)
        plus = fk.forward_kinematics(q + offset) @ mount
        minus = fk.forward_kinematics(q - offset) @ mount
        derivative = (plus - minus) / (2 * epsilon)
        jacobian[:3, index] = rotation.T @ derivative[:3, 3]
        spin = rotation.T @ derivative[:3, :3]
        jacobian[3:, index] = [spin[2, 1] - spin[1, 2],
                                spin[0, 2] - spin[2, 0],
                                spin[1, 0] - spin[0, 1]]
        jacobian[3:, index] *= .5
    if not np.all(np.isfinite(jacobian)):
        raise ValueError("FK produced a non-finite camera Jacobian")
    return current, jacobian


@dataclass(frozen=True)
class JointServoStep:
    target_degrees: np.ndarray
    error_rms_px: float
    predicted_error_rms_px: float
    camera_translation_m: float
    camera_rotation_rad: float


def image_joint_step(
    fk, joint_degrees, gripper_T_camera, points_px, depths_m, camera_matrix, image_size,
    *, target_side_px=120., gain=.3, damping=5., period_s=.1,
    max_joint_step_deg=.4, max_camera_step_m=.001, max_camera_rotation_rad=.01,
    margin_deg=2., target_corners_px=None,
):
    """Solve L_image @ J_camera directly; retain all six camera components.

    Returns an offline target proposal only. A device owner must independently
    enforce observation age, feedback, torque/status, total travel and stopping.
    Unreachable image goals are not guaranteed to converge on this 5-DOF arm.
    """
    parameters = np.array([target_side_px, gain, damping, period_s, max_joint_step_deg,
                           max_camera_step_m, max_camera_rotation_rad, margin_deg])
    if not np.all(np.isfinite(parameters)) or np.any(parameters <= 0):
        raise ValueError("solver parameters and limits must be positive and finite")
    q = np.asarray(joint_degrees, dtype=float)
    current, camera_jacobian = camera_joint_jacobian(fk, q, gripper_T_camera)
    lower = np.array([URDF_LIMITS[n].lower_deg + margin_deg for n in ARM_JOINT_NAMES])
    upper = np.array([URDF_LIMITS[n].upper_deg - margin_deg for n in ARM_JOINT_NAMES])
    if np.any(lower >= upper) or np.any(q <= lower) or np.any(q >= upper):
        raise ValueError("current joints must be inside limits with the configured margin")
    desired = (desired_square_corners(*image_size, target_side_px)
               if target_corners_px is None else np.asarray(target_corners_px, dtype=float))
    if desired.shape != (4, 2) or not np.all(np.isfinite(desired)):
        raise ValueError("target corners must be four finite pixel points")
    if np.any(desired < 0) or np.any(desired >= image_size):
        raise ValueError("target corners must fit inside the image")
    error = corner_error_px(points_px, desired)
    image_jacobian = point_interaction_matrix(points_px, depths_m, camera_matrix) @ camera_jacobian
    delta = -gain * period_s * np.linalg.solve(
        image_jacobian.T @ image_jacobian + damping**2 * np.eye(5),
        image_jacobian.T @ error,
    )
    if not np.all(np.isfinite(delta)):
        raise RuntimeError("joint solver produced a non-finite step")
    degrees = np.rad2deg(delta)
    scale = min(1., max_joint_step_deg / max(float(np.max(np.abs(degrees))), 1e-12))
    for index, step in enumerate(degrees):
        if step > 0:
            scale = min(scale, .9 * (upper[index] - q[index]) / step)
        elif step < 0:
            scale = min(scale, .9 * (lower[index] - q[index]) / step)
    # Verify actual FK camera displacement, not just a linearized bound.
    for _ in range(20):
        target = q + scale * degrees
        future = validate_transform(fk.forward_kinematics(target)) @ gripper_T_camera
        translation = float(np.linalg.norm(future[:3, 3] - current[:3, 3]))
        relative_rotation = current[:3, :3].T @ future[:3, :3]
        angle = float(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1, 1)))
        if translation <= max_camera_step_m and angle <= max_camera_rotation_rad:
            predicted = error + image_jacobian @ (delta * scale)
            return JointServoStep(target, float(np.sqrt(np.mean(error**2))),
                                  float(np.sqrt(np.mean(predicted**2))), translation, angle)
        scale *= .5
    raise RuntimeError("could not satisfy camera step limits")
