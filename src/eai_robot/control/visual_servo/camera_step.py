"""Plan one bounded camera-frame translation as a quantized joint target.

Round 1 measured a 2x2 pixel-per-count map over ``shoulder_pan`` and
``wrist_flex`` only. No column of that map changes the apparent marker size, so
the proven centering loop cannot express depth at all: it can re-center, but it
cannot approach. This module adds the missing primitive instead of retuning the
centering loop.

A requested camera translation is converted into its **first-order image
effect** (center shift in pixels plus apparent-size change in pixels) and the
joint step is solved against that effect. Two reasons:

* A lateral 1 cm step cannot be expressed as a camera translation on this arm.
  At the recorded pose 1 cm sideways is about 2.6 ``shoulder_pan`` counts, and
  panning mostly yaws the camera rather than translating it. The marker still
  moves the expected number of pixels, which is what the rule actually wants.
* A forward 1 cm step must be a real translation, and it forces roughly 4
  degrees of ``shoulder_lift`` together with an opposing ``wrist_flex``
  rotation. Without that counter-rotation the camera pitches by about 1 degree,
  moving the marker ~9-20 px, which is six to ten times the depth signal and
  is then "corrected" by the centering loop as if it were a centering error.

The solver therefore also refuses steps whose requested counts fall inside the
measured deadband, and compensates a per-joint, per-direction actual/requested
gain that starts at one and only changes from measured feedback.

Everything here is pure numerics. No camera, serial port, or motion is owned by
this module, and a plan is never evidence that the arm moved.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping, Sequence

import numpy as np

from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.control.visual_servo.aruco_ibvs import point_interaction_matrix
from eai_robot.control.visual_servo.joint_adapter import camera_joint_jacobian

MAX_STEP_M = 0.02
MIN_GAIN, MAX_GAIN = 0.45, 1.25
APPROACH_WEIGHTS = (1.0, 1.0, 3.0)
SHIFT_WEIGHTS = (1.0, 1.0, 3.0)


def counts_per_degree(calibration: Mapping[str, object]) -> np.ndarray:
    """Return encoder counts per URDF degree for the five arm joints.

    The mapper is linear between the calibrated raw range and the URDF limits,
    so this ratio is exact for every raw value inside calibration.
    """
    values = []
    for name in ARM_JOINT_NAMES:
        if name not in calibration:
            raise ValueError(f"calibration is missing {name}")
        item = calibration[name]
        span = float(item.range_max) - float(item.range_min)
        limit = URDF_LIMITS[name]
        width = limit.upper_deg - limit.lower_deg
        if span <= 0 or width <= 0:
            raise ValueError(f"invalid calibration span for {name}")
        values.append(span / width)
    return np.asarray(values, dtype=float)


@dataclass(frozen=True)
class ResponseModel:
    """Bounded actual-over-requested count ratio per joint and direction.

    An empty model assumes the request is reached. Entries are only replaced by
    measured feedback and are clipped to a range in which a partial response
    stays a partial response rather than becoming an invented extrapolation.
    """

    entries: tuple[tuple[str, int, float], ...] = ()

    def gain(self, joint: str, counts: float) -> float:
        if counts == 0:
            return 1.0
        sign = 1 if counts > 0 else -1
        for name, entry_sign, value in self.entries:
            if name == joint and entry_sign == sign:
                return value
        return 1.0

    def gains(self, counts: Sequence[float]) -> np.ndarray:
        return np.asarray(
            [self.gain(name, value) for name, value in zip(ARM_JOINT_NAMES, counts)],
            dtype=float,
        )

    def updated(self, joint: str, sign: int, measured: float) -> "ResponseModel":
        """Return a model with one clipped measurement; never mutates in place."""
        if joint not in ARM_JOINT_NAMES or sign not in (-1, 1):
            raise ValueError("joint and direction must identify a commanded axis")
        if not np.isfinite(measured):
            return self
        value = float(np.clip(measured, MIN_GAIN, MAX_GAIN))
        table = {(name, entry_sign): gain for name, entry_sign, gain in self.entries}
        table[(joint, int(sign))] = value
        return ResponseModel(tuple(sorted((name, entry_sign, gain)
                                          for (name, entry_sign), gain in table.items())))

    def describe(self) -> dict[str, float]:
        return {f"{name}{'+' if sign > 0 else '-'}": round(gain, 4)
                for name, sign, gain in self.entries}


@dataclass(frozen=True)
class ImageGoal:
    """First-order image effect of a camera translation, in pixels."""

    du_px: float
    dv_px: float
    d_side_px: float
    weights: tuple[float, float, float] = SHIFT_WEIGHTS

    def vector(self) -> np.ndarray:
        return np.array([self.du_px, self.dv_px, self.d_side_px], dtype=float)

    def scaled(self, factor: float) -> "ImageGoal":
        return replace(self, du_px=self.du_px * factor,
                       dv_px=self.dv_px * factor, d_side_px=self.d_side_px * factor)

    @property
    def dominant(self) -> str:
        if abs(self.d_side_px) >= max(abs(self.du_px), abs(self.dv_px)):
            return "depth"
        return "horizontal" if abs(self.du_px) >= abs(self.dv_px) else "vertical"


@dataclass(frozen=True)
class CameraStepPlan:
    """A quantized joint target for one camera-frame translation."""

    requested_delta_m: np.ndarray
    goal: ImageGoal
    goal_scale: float
    escalated: bool
    target_degrees: np.ndarray
    requested_counts: np.ndarray
    expected_counts: np.ndarray
    pinned_joints: tuple[str, ...]
    predicted_image_change: np.ndarray
    predicted_translation_m: np.ndarray
    predicted_rotation_rad: np.ndarray
    achieved_fraction: float
    notes: tuple[str, ...] = field(default=())

    @property
    def movable(self) -> bool:
        return bool(np.any(self.requested_counts))

    @property
    def pitch_rad(self) -> float:
        """Camera pitch change, the rotation that most corrupts a depth step."""
        return float(self.predicted_rotation_rad[0])

    def describe(self) -> dict[str, object]:
        return {
            "requested_delta_m": self.requested_delta_m.tolist(),
            "goal_scale": round(self.goal_scale, 3),
            "escalated": self.escalated,
            "requested_counts": {n: int(v) for n, v in
                                 zip(ARM_JOINT_NAMES, self.requested_counts)},
            "expected_counts": {n: round(float(v), 2) for n, v in
                                zip(ARM_JOINT_NAMES, self.expected_counts)},
            "pinned_joints": list(self.pinned_joints),
            "predicted_image_change": np.round(self.predicted_image_change, 2).tolist(),
            "predicted_translation_mm": np.round(self.predicted_translation_m * 1000, 2).tolist(),
            "predicted_rotation_mrad": np.round(self.predicted_rotation_rad * 1000, 2).tolist(),
            "achieved_fraction": round(self.achieved_fraction, 3),
            "notes": list(self.notes),
        }


class StepInfeasible(RuntimeError):
    """No quantized joint target can realize enough of the requested step."""

    def __init__(self, message: str):
        super().__init__(message)


def camera_delta_goal(
    corners_px: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
    delta_m: Sequence[float],
    *,
    side_px: float | None = None,
) -> ImageGoal:
    """First-order image effect of a pure camera translation.

    The center rows come from the point interaction matrix. The apparent-size
    row uses the mean depth: only translation along the optical axis changes the
    marker size, and a fronto-parallel square grows by ``side * dz / depth``.
    """
    points = np.asarray(corners_px, dtype=float).reshape(4, 2)
    depths = np.asarray(depths_m, dtype=float).reshape(4)
    delta = np.asarray(delta_m, dtype=float).reshape(3)
    matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(delta)):
        raise ValueError("camera delta must be finite")
    interaction = point_interaction_matrix(points, depths, matrix)
    translation = interaction @ np.r_[delta, np.zeros(3)]
    center = translation.reshape(4, 2).mean(axis=0)
    size = marker_side_px(points) if side_px is None else float(side_px)
    depth = float(np.mean(depths))
    if depth <= 0 or size <= 0:
        raise ValueError("marker must have a positive size and depth")
    return ImageGoal(float(center[0]), float(center[1]), float(size * delta[2] / depth))


def image_task_matrix(
    corners_px: np.ndarray, depths_m: np.ndarray, camera_matrix: np.ndarray,
    side_px: float | None = None,
) -> np.ndarray:
    """Map a camera twist to ``[du, dv, d(side_px)]``; three rows, six columns."""
    points = np.asarray(corners_px, dtype=float).reshape(4, 2)
    depths = np.asarray(depths_m, dtype=float).reshape(4)
    interaction = point_interaction_matrix(points, depths, np.asarray(camera_matrix))
    matrix = np.zeros((3, 6))
    matrix[0] = interaction[0::2].mean(axis=0)
    matrix[1] = interaction[1::2].mean(axis=0)
    size = marker_side_px(points) if side_px is None else float(side_px)
    matrix[2, 2] = size / float(np.mean(depths))
    return matrix


def _joint_bounds_degrees(margin_deg: float) -> tuple[np.ndarray, np.ndarray]:
    lower = np.asarray([URDF_LIMITS[n].lower_deg + margin_deg for n in ARM_JOINT_NAMES])
    upper = np.asarray([URDF_LIMITS[n].upper_deg - margin_deg for n in ARM_JOINT_NAMES])
    if np.any(lower >= upper):
        raise ValueError("joint margin leaves an empty range")
    return lower, upper


def _solve_counts(task, goal, rate, model, angles, lower, upper, *, weights,
                  damping, max_joint_step_deg, min_effective_counts, max_request_counts):
    """Weighted damped solve plus an active set for deadband and travel limits.

    Returns ``(requested, expected, pinned, notes)`` in encoder counts, or None.
    A joint whose expected motion cannot leave the measured deadband is removed
    from the task and the rest is re-solved, so the prediction describes what
    the arm will do rather than what was asked. A joint that would exceed its
    travel or per-step cap shortens the **whole** step by one common factor:
    scaling a single joint instead would break the lift/wrist balance that keeps
    the camera from pitching.
    """
    weight = np.diag(weights)
    active = np.ones(len(ARM_JOINT_NAMES), dtype=bool)
    pinned = np.zeros(len(ARM_JOINT_NAMES), dtype=bool)
    notes: list[str] = []

    for _ in range(len(ARM_JOINT_NAMES)):
        columns = active
        if not np.any(columns):
            return None
        rows = weight @ task[:, columns]
        normal = rows.T @ rows + damping**2 * np.eye(int(columns.sum()))
        try:
            solution = np.linalg.solve(normal, rows.T @ (weight @ goal))
        except np.linalg.LinAlgError:
            return None
        degrees = np.zeros(len(ARM_JOINT_NAMES))
        degrees[columns] = np.rad2deg(solution)

        desired = degrees * rate
        # Compensation depends on the sign of the request, which the solve
        # decides. Two passes are enough because the sign is stable.
        requested = desired.copy()
        for _ in range(2):
            local = model.gains(requested)
            requested = desired / np.where(local > 0, local, 1.0)
        local = model.gains(requested)
        expected = requested * local

        room = np.where(requested > 0, upper - angles, angles - lower) * rate
        # The cap bounds the *request*, not the expected motion, so a wrong gain
        # estimate can never turn compensation into a larger absolute command
        # than this step limit allows.
        allowed = np.minimum(np.abs(room), max_request_counts)
        moving = np.abs(requested) > 1e-9
        if np.any(moving):
            fraction = float(np.min(allowed[moving] / np.abs(requested[moving])))
            if fraction < 1.0:
                requested = requested * fraction
                expected = expected * fraction
                notes.append(f"step:scaled_to_{fraction:.2f}")
        largest = float(np.max(np.abs(expected) / rate))
        if largest > max_joint_step_deg:
            fraction = max_joint_step_deg / largest
            requested = requested * fraction
            expected = expected * fraction
            notes.append("step:scaled_to_joint_limit")

        requested = np.rint(requested)
        # Rounding can push a request back above the per-step cap that the scale
        # above just enforced (13.6 counts rounds to 14, i.e. 4.12 degrees under a
        # 4 degree cap). Truncate toward zero instead, otherwise the motion guard
        # rejects a step whose plan claims to be legal and the loop stalls on a
        # boundary disagreement rather than on the workspace.
        limit = max_joint_step_deg * rate
        over = np.abs(requested) > limit
        if np.any(over):
            requested = np.where(over, np.trunc(np.sign(requested) * limit), requested)
            notes.append("step:truncated_to_joint_limit")
        expected = requested * local
        dead = active & (np.abs(expected) > 0) & (np.abs(expected) < min_effective_counts)
        if np.any(dead):
            pinned |= dead
            active &= ~dead
            notes.extend(f"{ARM_JOINT_NAMES[index]}:inside_deadband"
                         for index in np.flatnonzero(dead))
            continue
        if np.any(active & (expected == 0)) and np.any(expected):
            notes.append("joint:rounded_to_zero")
        return requested, expected, pinned, notes
    return None


def plan_camera_step(
    fk,
    joint_degrees: Sequence[float],
    gripper_T_camera,
    delta_m: Sequence[float],
    corners_px: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
    *,
    counts_per_deg: Sequence[float],
    gains: ResponseModel | None = None,
    side_px: float | None = None,
    weights: tuple[float, float, float] | None = None,
    joint_margin_deg: float = 2.0,
    max_joint_step_deg: float = 4.0,
    min_effective_counts: float = 5.0,
    max_request_counts: float = 18.0,
    damping: float = 0.05,
    min_achieved_fraction: float = 0.35,
    max_center_drift_px: float = 60.0,
    allow_escalation: bool = True,
    max_goal_scale: float = 2.0,
) -> CameraStepPlan:
    """Turn ``delta_m`` in the camera frame into a bounded joint target.

    The step is reduced until it satisfies the joint and image-drift limits and
    every commanded joint leaves the deadband. Camera pitch is reported rather
    than gated: integer encoder counts put a floor of about one count, roughly
    5 mrad, on how well a lift/wrist pair can cancel it, and its real cost is
    the image drift that the drift limit already bounds. If the request is too small to
    move any joint, it is scaled up to the smallest executable amplitude in the
    same direction (never beyond ``max_goal_scale``) and flagged. When nothing
    executable exists the call raises ``StepInfeasible`` instead of sending a
    step whose effect is unknown.
    """
    request = np.asarray(delta_m, dtype=float).reshape(3)
    angles = np.asarray(joint_degrees, dtype=float).reshape(5)
    rate = np.asarray(counts_per_deg, dtype=float).reshape(5)
    distance = float(np.linalg.norm(request))
    if not np.all(np.isfinite(request)) or not np.all(np.isfinite(angles)):
        raise ValueError("step and joint angles must be finite")
    if not 0 < distance <= MAX_STEP_M:
        raise ValueError(f"camera step must be within (0, {MAX_STEP_M}] m")
    if not np.all(np.isfinite(rate)) or np.any(rate <= 0):
        raise ValueError("counts per degree must be positive")
    if not np.isfinite(max_goal_scale) or max_goal_scale < 1.0:
        raise ValueError("max_goal_scale must be at least one")
    lower, upper = _joint_bounds_degrees(joint_margin_deg)
    if np.any(angles <= lower) or np.any(angles >= upper):
        raise ValueError("current joints must lie inside limits with the margin")

    goal = camera_delta_goal(corners_px, depths_m, camera_matrix, request, side_px=side_px)
    task = image_task_matrix(corners_px, depths_m, camera_matrix, side_px=side_px)
    camera, jacobian = camera_joint_jacobian(fk, angles, gripper_T_camera)
    task_joint = task @ jacobian
    chosen = weights or (APPROACH_WEIGHTS if goal.dominant == "depth" else SHIFT_WEIGHTS)
    model = gains or ResponseModel()

    unit_goal = goal.vector()
    denominator = float(unit_goal @ (np.asarray(chosen) ** 2 * unit_goal))
    if denominator <= 1e-9:
        raise StepInfeasible("camera translation has no first-order image effect")

    rejects: list[str] = []
    # A full one-centimetre step first. Escalation covers the case where every
    # joint sits inside the deadband; the fractional scales cover the case where
    # a measured weak direction, travel cap or deadband can only support part of
    # the step. Partial progress is reported as an achieved fraction, not hidden.
    scales = (1.0, 1.35, 1.7, max_goal_scale, 0.8, 0.65, 0.5, 0.35, 0.25)
    for scale in scales:
        if scale > 1.0 and not allow_escalation:
            break
        candidate = goal.scaled(scale)
        solved = _solve_counts(
            task_joint, candidate.vector(), rate, model, angles, lower, upper,
            weights=chosen, damping=damping, max_joint_step_deg=max_joint_step_deg,
            min_effective_counts=min_effective_counts,
            max_request_counts=max_request_counts,
        )
        if solved is None:
            rejects.append(f"scale {scale:.2f}: no executable joint set")
            continue
        requested, expected, pinned, notes = solved
        if not np.any(requested):
            rejects.append(f"scale {scale:.2f}: every joint inside deadband")
            continue

        degrees_delta = expected / rate
        radians_delta = np.deg2rad(degrees_delta)
        predicted_image = task_joint @ radians_delta
        predicted = jacobian @ radians_delta
        projected = float(predicted_image @ (np.asarray(chosen) ** 2 * unit_goal))
        achieved = projected / denominator
        drift = float(np.linalg.norm(predicted_image[:2]))
        pitch = float(abs(predicted[3]))

        if goal.dominant != "depth" and abs(predicted_image[2]) > 2.0:
            rejects.append(f"scale {scale:.2f}: lateral step changes marker size")
            continue
        if goal.dominant == "depth" and drift > max_center_drift_px:
            rejects.append(f"scale {scale:.2f}: center drift {drift:.1f} px")
            continue
        # A step that the per-joint cap truncated is already the largest legal
        # move in this direction. Refusing it because it realizes only part of
        # the requested centimetre would stall the loop forever at that depth
        # instead of making the progress the arm is actually allowed to make.
        cap_limited = any(note in notes for note in
                          ("step:scaled_to_joint_limit", "step:truncated_to_joint_limit"))
        if achieved < min_achieved_fraction and not cap_limited:
            rejects.append(f"scale {scale:.2f}: achieved {achieved:.2f} of the goal")
            continue

        extra = ["escalated_to_minimum_step"] if scale > 1.0 else (
            [f"partial_step_{scale:.2f}"] if scale < 1.0 else [])
        if cap_limited and achieved < 1.0:
            extra.append(f"cap_limited_to_{achieved:.2f}")
        plan = CameraStepPlan(
            requested_delta_m=request,
            goal=candidate,
            goal_scale=scale,
            escalated=scale > 1.0,
            target_degrees=angles + degrees_delta,
            requested_counts=requested.astype(int),
            expected_counts=expected,
            pinned_joints=tuple(name for name, flag in zip(ARM_JOINT_NAMES, pinned) if flag),
            predicted_image_change=predicted_image,
            predicted_translation_m=predicted[:3],
            predicted_rotation_rad=predicted[3:],
            achieved_fraction=achieved,
            notes=tuple(notes) + tuple(extra),
        )
        return plan

    raise StepInfeasible("no bounded step realizes the requested translation: "
                         + "; ".join(rejects))


def plan_axis_step(
    fk,
    joint_degrees: Sequence[float],
    gripper_T_camera,
    corners_px: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
    *,
    axis: int,
    pixels: float,
    requested_delta_m: Sequence[float],
    counts_per_deg: Sequence[float],
    gains: ResponseModel | None = None,
    joint_margin_deg: float = 2.0,
    max_joint_step_deg: float = 4.0,
    min_effective_counts: float = 5.0,
    max_request_counts: float = 18.0,
    max_cross_fraction: float = 0.35,
    min_achieved_fraction: float = 0.6,
    max_size_change_px: float = 2.0,
    allow_escalation: bool = True,
    max_goal_scale: float = 2.0,
) -> CameraStepPlan:
    """Realize a single image-axis shift with one joint.

    Lateral and vertical corrections use one joint per step on purpose. On this
    arm a one-centimetre shift is a small image motion that any of several joints
    can produce, and a minimum-norm solution mixes three of them into a step that
    is hard to attribute when it fails. One joint with a measured pixel-per-count
    response either moves the marker the expected number of pixels or it does
    not, which is exactly what the verification needs.

    ``axis`` is 0 for horizontal and 1 for vertical. The amplitude grows to the
    deadband floor when the request is too small to move anything.
    """
    angles = np.asarray(joint_degrees, dtype=float).reshape(5)
    rate = np.asarray(counts_per_deg, dtype=float).reshape(5)
    delta = np.asarray(requested_delta_m, dtype=float).reshape(3)
    if axis not in (0, 1):
        raise ValueError("axis must be 0 (horizontal) or 1 (vertical)")
    if not np.isfinite(pixels) or pixels == 0:
        raise ValueError("pixel goal must be finite and non-zero")
    if not np.all(np.isfinite(angles)):
        raise ValueError("joint angles must be finite")
    if not np.all(np.isfinite(rate)) or np.any(rate <= 0):
        raise ValueError("counts per degree must be positive")
    lower, upper = _joint_bounds_degrees(joint_margin_deg)
    if np.any(angles <= lower) or np.any(angles >= upper):
        raise ValueError("current joints must lie inside limits with the margin")

    task = image_task_matrix(corners_px, depths_m, camera_matrix)
    camera, jacobian = camera_joint_jacobian(fk, angles, gripper_T_camera)
    per_count = (task @ jacobian) * (np.pi / 180.0) / rate
    model = gains or ResponseModel()

    rejects: list[str] = []
    for scale in (1.0, 1.35, 1.7, max_goal_scale):
        if scale > 1.0 and not allow_escalation:
            break
        target = float(pixels) * scale
        best = None
        for index, name in enumerate(ARM_JOINT_NAMES):
            response = float(per_count[axis, index])
            if abs(response) < 1e-6:
                continue
            gain = model.gain(name, float(np.sign(target / response)))
            requested = int(round(target / response / gain))
            if requested == 0:
                continue
            expected = requested * gain
            if abs(expected) < min_effective_counts:
                continue
            if abs(requested) > max_request_counts:
                continue
            if abs(expected) / rate[index] > max_joint_step_deg:
                continue
            if not lower[index] < angles[index] + expected / rate[index] < upper[index]:
                continue
            cross = float(per_count[1 - axis, index]) * expected
            if abs(cross) > max_cross_fraction * abs(target):
                continue
            if abs(float(per_count[2, index]) * expected) > max_size_change_px:
                continue
            if best is None or abs(requested) < abs(best[1]):
                best = (index, requested, expected, gain)
        if best is None:
            rejects.append(f"scale {scale:.2f}: no joint reaches the goal outside deadband")
            continue

        index, requested, expected, gain = best
        counts = np.zeros(5, dtype=int)
        counts[index] = requested
        expected_counts = np.zeros(5)
        expected_counts[index] = expected
        joint_delta = np.zeros(5)
        joint_delta[index] = np.deg2rad(expected / rate[index])
        predicted_image = (task @ jacobian) @ joint_delta
        predicted = jacobian @ joint_delta
        achieved = float(predicted_image[axis]) / float(pixels)
        if achieved < min_achieved_fraction:
            rejects.append(f"scale {scale:.2f}: {ARM_JOINT_NAMES[index]} achieves {achieved:.2f}")
            continue
        goal = ImageGoal(float(pixels) * scale, 0.0, 0.0)
        if axis == 1:
            goal = ImageGoal(0.0, float(pixels) * scale, 0.0)
        notes = [f"single_axis:{ARM_JOINT_NAMES[index]}"]
        if scale > 1.0:
            notes.append("escalated_to_minimum_step")
        return CameraStepPlan(
            requested_delta_m=delta,
            goal=goal,
            goal_scale=scale,
            escalated=scale > 1.0,
            target_degrees=angles + expected / rate * np.eye(5)[index],
            requested_counts=counts,
            expected_counts=expected_counts,
            pinned_joints=tuple(name for name, value in zip(ARM_JOINT_NAMES, counts) if value == 0),
            predicted_image_change=predicted_image,
            predicted_translation_m=predicted[:3],
            predicted_rotation_rad=predicted[3:],
            achieved_fraction=achieved,
            notes=tuple(notes),
        )
    raise StepInfeasible("no single joint realizes the requested image shift: "
                         + "; ".join(rejects))


def predict_corners_px(
    corners_px: np.ndarray,
    depths_m: np.ndarray,
    camera_matrix: np.ndarray,
    camera_jacobian: np.ndarray,
    joint_delta_rad: np.ndarray,
) -> np.ndarray:
    """First-order prediction of where the four corners move for a joint step."""
    points = np.asarray(corners_px, dtype=float).reshape(4, 2)
    depths = np.asarray(depths_m, dtype=float).reshape(4)
    interaction = point_interaction_matrix(points, depths, camera_matrix)
    pixel_velocity = interaction @ (np.asarray(camera_jacobian) @ np.asarray(joint_delta_rad))
    return points + pixel_velocity.reshape(4, 2)


def marker_side_px(corners_px: np.ndarray) -> float:
    """Mean edge length of the detected marker square, in pixels."""
    points = np.asarray(corners_px, dtype=float).reshape(4, 2)
    edges = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    return float(np.mean(edges))


def depth_from_side_px(side_px: float, focal_px: float, marker_length_m: float) -> float:
    """Pinhole depth of a fronto-parallel square marker from its apparent side."""
    if not np.isfinite(side_px) or side_px <= 0:
        raise ValueError("marker side must be positive pixels")
    if not np.isfinite(focal_px) or focal_px <= 0:
        raise ValueError("focal length must be positive pixels")
    if not np.isfinite(marker_length_m) or marker_length_m <= 0:
        raise ValueError("marker length must be positive meters")
    return float(focal_px * marker_length_m / side_px)


def relative_side_change(step_m: float, depth_m: float) -> float:
    """Expected fractional growth of the marker for a forward camera step."""
    if not np.isfinite(step_m) or step_m <= 0:
        raise ValueError("forward step must be positive meters")
    if not np.isfinite(depth_m) or depth_m <= step_m:
        raise ValueError("depth must exceed the forward step")
    return float(step_m / (depth_m - step_m))
