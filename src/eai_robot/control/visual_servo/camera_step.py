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

The solver therefore also refuses steps whose requested counts cannot clear the
axis's own absorbed offset plus the two counts the motion guard needs, and
compensates a per-joint, per-direction offset and gain that start neutral and
only change from measured feedback or a declared hardware prior.

Everything here is pure numerics. No camera, serial port, or motion is owned by
this module, and a plan is never evidence that the arm moved.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Mapping, Sequence

import numpy as np

from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS
from eai_robot.control.visual_servo.aruco_ibvs import point_interaction_matrix
from eai_robot.control.visual_servo.empirical import OBSERVED_MOTION_COUNTS
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
    """Measured count response per joint and direction: an offset and a gain.

    Two effects share one measurement and they are not interchangeable:

    * an **offset** is absorbed before the shaft moves at all -- backlash,
      stiction, the servo's own settle band under load;
    * a **gain** scales whatever motion is left after the offset.

    The model is ``moved = max(0, |request| - offset) * gain``, per joint and per
    direction, because the two are direction-dependent: the 2026-10-18 runs lost
    about 6.5 counts on every negative ``wrist_flex`` step of any size, and about
    1 count on the one positive step measured.

    A pure ratio cannot represent a step that produces nothing. Fitting the
    offset into a slope is what turned "7 requested, 0 delivered" into a gain of
    zero and "14 requested, 8 delivered" into 0.57 on the same axis in the same
    pose. An empty model still assumes the request is reached, so every default
    stays neutral and only measured feedback -- or a declared hardware prior --
    changes it.
    """

    entries: tuple[tuple[str, int, float], ...] = ()
    offset_entries: tuple[tuple[str, int, float], ...] = ()

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

    def offset(self, joint: str, counts: float) -> float:
        """Counts this axis absorbs in this direction before it moves."""
        if counts == 0:
            return 0.0
        sign = 1 if counts > 0 else -1
        for name, entry_sign, value in self.offset_entries:
            if name == joint and entry_sign == sign:
                return value
        return 0.0

    def offsets(self, counts: Sequence[float]) -> np.ndarray:
        return np.asarray(
            [self.offset(name, value) for name, value in zip(ARM_JOINT_NAMES, counts)],
            dtype=float,
        )

    def predict(self, counts: Sequence[float]) -> np.ndarray:
        """Expected encoder motion for a request, offset first and gain second."""
        requested = np.asarray(counts, dtype=float)
        return (np.sign(requested)
                * np.maximum(np.abs(requested) - self.offsets(requested), 0.0)
                * self.gains(requested))

    def minimum_request(self, joint: str, counts: float) -> float:
        """Smallest request worth sending on this axis in this direction.

        The offset has to be paid first, and the motion guard needs
        ``OBSERVED_MOTION_COUNTS`` beyond it before it will call the step
        observed rather than timing out. Anything smaller is a command the
        hardware cannot answer and the guard cannot verify.
        """
        return self.offset(joint, counts) + OBSERVED_MOTION_COUNTS

    def minimum_requests(self, counts: Sequence[float]) -> np.ndarray:
        return np.asarray(
            [self.minimum_request(name, value)
             for name, value in zip(ARM_JOINT_NAMES, counts)],
            dtype=float,
        )

    def characterised(self, joint: str) -> bool:
        """Whether any prior or measured entry exists for this axis.

        Used to break ties between candidate axes: an axis nobody has measured
        looks cheaper than a characterised one as soon as offsets are modelled,
        and choosing it is choosing to learn its threshold with a timeout.
        """
        return any(name == joint for name, _, _ in
                   self.entries + self.offset_entries)

    def updated(self, joint: str, sign: int, measured: float) -> "ResponseModel":
        """Return a model with one clipped measurement; never mutates in place."""
        if joint not in ARM_JOINT_NAMES or sign not in (-1, 1):
            raise ValueError("joint and direction must identify a commanded axis")
        if not np.isfinite(measured):
            return self
        value = float(np.clip(measured, MIN_GAIN, MAX_GAIN))
        table = {(name, entry_sign): gain for name, entry_sign, gain in self.entries}
        table[(joint, int(sign))] = value
        return ResponseModel(
            tuple(sorted((name, entry_sign, gain)
                         for (name, entry_sign), gain in table.items())),
            self.offset_entries)

    def with_offset(self, joint: str, sign: int, counts: float) -> "ResponseModel":
        """Return a model with one measured offset; gains are carried across."""
        if joint not in ARM_JOINT_NAMES or sign not in (-1, 1):
            raise ValueError("joint and direction must identify a commanded axis")
        if not np.isfinite(counts) or counts < 0:
            return self
        table = {(name, entry_sign): value
                 for name, entry_sign, value in self.offset_entries}
        table[(joint, int(sign))] = float(counts)
        return ResponseModel(
            self.entries,
            tuple(sorted((name, entry_sign, value)
                         for (name, entry_sign), value in table.items())))

    def describe(self) -> dict[str, float]:
        return {f"{name}{'+' if sign > 0 else '-'}": round(gain, 4)
                for name, sign, gain in self.entries}

    def describe_offsets(self) -> dict[str, float]:
        return {f"{name}{'+' if sign > 0 else '-'}": round(value, 3)
                for name, sign, value in self.offset_entries}


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


def describe_rejects(rejects: Sequence[tuple[float, str, str]]) -> str:
    """Render refusals as ``scale`` plus the rules that fired, each shape once.

    Every refusal has to say *which* rule stopped it -- "no joint reaches the
    goal outside deadband" repeated for four scales names nothing and leaves the
    operator guessing at the bench. Escalation re-solves the same joints against
    the same constraints, so it usually reproduces the same set of rules with
    larger numbers; those are collapsed into one entry that lists the scales and
    keeps the numbers from the smallest one.
    """
    groups: dict[str, list] = {}
    order: list[str] = []
    for scale, key, body in rejects:
        if key not in groups:
            groups[key] = [body, []]
            order.append(key)
        groups[key][1].append(scale)
    parts = []
    for key in order:
        body, scales = groups[key]
        listed = ", ".join(f"{value:.2f}" for value in scales)
        label = "scale" if len(scales) == 1 else "scales"
        parts.append(f"{label} {listed}: {body}")
    return "; ".join(parts)


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


def _solve_counts_active(task, goal, rate, model, angles, lower, upper, *, weights,
                  damping, max_joint_step_deg, min_effective_counts, max_request_counts,
                  active_mask=None):
    """Weighted damped solve plus an active set for deadband and travel limits.

    Returns ``(requested, expected, pinned, notes)`` in encoder counts, or None.
    A joint whose expected motion cannot leave the measured deadband, or whose
    request cannot clear its own offset, is removed from the task and the rest is
    re-solved, so the prediction describes what the arm will do rather than what
    was asked. A joint that would exceed its travel or per-step cap shortens the
    **whole** step by one common factor: scaling a single joint instead would
    break the lift/wrist balance that keeps the camera from pitching.
    """
    weight = np.diag(weights)
    active = (np.ones(len(ARM_JOINT_NAMES), dtype=bool) if active_mask is None
              else np.asarray(active_mask, dtype=bool).copy())
    pinned = ~active.copy()
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
        # decides. Two passes are enough because the sign is stable. The offset
        # is added *after* the gain division: the shaft absorbs it before any of
        # the requested motion appears, so a request that would realize
        # ``desired`` counts is ``desired / gain + offset``, not
        # ``desired / gain``.
        requested = desired.copy()
        for _ in range(2):
            local = model.gains(requested)
            shifted = model.offsets(requested)
            requested = np.sign(desired) * (
                np.abs(desired) / np.where(local > 0, local, 1.0) + shifted)
        expected = model.predict(requested)

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
                # Re-predict rather than scaling the old prediction: the offset
                # does not shrink with the request, so a step shortened below it
                # realizes nothing at all.
                expected = model.predict(requested)
                notes.append(f"step:scaled_to_{fraction:.2f}")
        # The guard checks the *goal* delta, so the request must fit the same cap
        # as the prediction. With an offset the request is larger than what it
        # predicts, and capping only the prediction is how a plan claims to be
        # legal and is then refused as "single joint step 5.52 deg exceeds 5.00".
        #
        # The cap is the guard's *intent* limit and planning may use all of it.
        # The arrival residual is measurement noise that the guard absorbs on its
        # side (``reject_reason(..., step_tolerance=ARRIVAL_TOLERANCE)``), because
        # subtracting it here would cost every step three counts of reach: at a
        # 5 degree cap the forward primitive stops being expressible at all.
        largest = float(np.max(np.maximum(np.abs(requested), np.abs(expected)) / rate))
        if largest > max_joint_step_deg:
            fraction = max_joint_step_deg / largest
            requested = requested * fraction
            expected = model.predict(requested)
            notes.append("step:scaled_to_joint_limit")

        requested = np.rint(requested)
        # Rounding can push a request back above the per-step cap that the scale
        # above just enforced. Truncate toward zero instead, otherwise the motion
        # guard rejects a step whose plan claims to be legal and the loop stalls
        # on a boundary disagreement rather than on the workspace.
        limit = max_joint_step_deg * rate
        over = np.abs(requested) > limit
        if np.any(over):
            requested = np.where(over, np.trunc(np.sign(requested) * limit), requested)
            notes.append("step:truncated_to_joint_limit")
        expected = model.predict(requested)
        dead = active & (np.abs(expected) > 0) & (np.abs(expected) < min_effective_counts)
        # An axis whose *request* cannot clear its own offset plus the counts the
        # motion guard needs is not executable either, however the prediction
        # reads. Without this, a lowered ``min_effective_counts`` would let the
        # planner send a step the guard can only answer with a timeout.
        dead |= (active & (np.abs(requested) > 0)
                 & (np.abs(requested) < model.minimum_requests(requested)))
        if np.any(dead):
            # Limits applied to a discarded solution cannot grant the final
            # solution an exemption from the achieved-fraction requirement.
            notes = [n.replace('step:', 'discarded_step:', 1)
                     if n.startswith('step:') else n for n in notes]
            pinned |= dead
            active &= ~dead
            notes.extend(f"{ARM_JOINT_NAMES[index]}:inside_deadband"
                         for index in np.flatnonzero(dead))
            continue
        if np.any(active & (expected == 0)) and np.any(expected):
            notes.append("joint:rounded_to_zero")
        return requested, expected, pinned, notes
    return None


def _solve_counts(task, goal, rate, model, angles, lower, upper, *,
                  max_center_drift_px=None, **options):
    """Retain the normal solve; search other active sets if it collapses.

    Removing every sub-deadband column together is only a heuristic, not an
    infeasibility proof. With five axes, the remaining 30 nonempty proper
    subsets are cheap enough to check under the same execution constraints.
    """
    weights = np.asarray(options['weights'])
    depth_goal = abs(goal[2]) >= max(abs(goal[0]), abs(goal[1]))

    def score(result):
        if result is None or not np.any(result[0]):
            return None
        image = task @ np.deg2rad(result[1] / rate)
        if float(image @ (weights**2 * goal)) <= 0:
            return None
        if depth_goal and image[2] * goal[2] <= 0:
            return None
        if (depth_goal and max_center_drift_px is not None
                and np.linalg.norm(image[:2]) > max_center_drift_px):
            return None
        return (float(np.linalg.norm(weights * (image - goal))),
                float(np.sum(np.abs(result[0]))))

    original = _solve_counts_active(task, goal, rate, model, angles, lower, upper, **options)
    if score(original) is not None:
        return original
    best, best_score = None, None
    for bits in range(1, (1 << len(ARM_JOINT_NAMES)) - 1):
        active = [bool(bits & (1 << i)) for i in range(len(ARM_JOINT_NAMES))]
        result = _solve_counts_active(task, goal, rate, model, angles, lower, upper,
                                      active_mask=active, **options)
        rank = score(result)
        if rank is not None and (best_score is None or rank < best_score):
            best, best_score = result, rank
    if best is not None:
        requested, expected, pinned, notes = best
        return requested, expected, pinned, notes + ['active_set_fallback']
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
        raise StepInfeasible("该平移在一阶图像效应上为零（no first-order image effect）")

    rejects: list[tuple[float, str, str]] = []
    # A full one-centimetre step first. Escalation covers the case where every
    # joint sits inside the deadband; the fractional scales cover the case where
    # a measured weak direction, travel cap or deadband can only support part of
    # the step. Partial progress is reported as an achieved fraction, not hidden.
    scales = (1.0, 1.35, 1.7, max_goal_scale, 0.8, 0.65, 0.5, 0.35, 0.25)
    for scale in scales:
        if scale > 1.0 and not allow_escalation:
            continue
        candidate = goal.scaled(scale)
        solved = _solve_counts(
            task_joint, candidate.vector(), rate, model, angles, lower, upper,
            weights=chosen, damping=damping, max_joint_step_deg=max_joint_step_deg,
            min_effective_counts=min_effective_counts,
            max_request_counts=max_request_counts,
            max_center_drift_px=max_center_drift_px,
        )
        if solved is None:
            rejects.append((scale, "solve", "候选组合搜索未找到满足当前图像目标与执行约束的解"))
            continue
        requested, expected, pinned, notes = solved
        if not np.any(requested):
            rejects.append((scale, "deadband", "当前候选组合取整/补偿后没有可测位移"))
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
            rejects.append((scale, "size", "横移会改变标记视在尺寸"))
            continue
        if goal.dominant == "depth" and drift > max_center_drift_px:
            rejects.append((scale, "drift", f"中心漂移 {drift:.1f} px 超限"))
            continue
        # A step that the per-joint cap truncated is already the largest legal
        # move in this direction. Refusing it because it realizes only part of
        # the requested centimetre would stall the loop forever at that depth
        # instead of making the progress the arm is actually allowed to make.
        cap_limited = any(note in notes for note in
                          ("step:scaled_to_joint_limit", "step:truncated_to_joint_limit"))
        if achieved <= 0 or (achieved < min_achieved_fraction and not cap_limited):
            rejects.append((scale, "achieved", f"只达成目标的 {achieved:.2f} 倍"))
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

    raise StepInfeasible("没有有界步能实现请求的平移（no bounded step realizes the "
                         "requested translation）：" + describe_rejects(rejects))


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
    excluded_directions: Sequence[tuple[str, int]] = (),
    joint_margin_deg: float = 2.0,
    max_joint_step_deg: float = 4.0,
    min_effective_counts: float = 5.0,
    max_request_counts: float = 18.0,
    max_cross_fraction: float = 0.60,
    cross_error_px: float = 0.0,
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

    ``axis`` is 0 for horizontal and 1 for vertical. The request is built from
    the axis's measured offset plus the counts the pixel goal needs, and grows
    with the goal scale until it is an amplitude the hardware can actually
    execute.

    ``pixels`` is the image shift this one step should produce, but it is not
    the whole error: the marker is usually off *both* axes. ``cross_error_px``
    is what still has to be corrected on the other image axis, so a step that
    also moves the marker along that axis can be told apart from one that drags
    it past the target and back.

    ``max_cross_fraction`` bounds how much the *other* axis's error may grow,
    as a fraction of this step. 0.60 is derived from the arm, not chosen: the
    measured sideways share of a vertical step is 0.16 on this arm at wrist roll
    -92 deg and 0.43-0.48 at -77 deg, because the image Jacobian is rotated with
    the wrist. A limit below the coupling of *every* usable axis does not
    protect the loop, it stops it: at 0.35 the 17:24 frame has no legal joint at
    any escalation, and the closed loop halts after two steps with a 29 px error
    and depth still 80 mm from the standoff. At 0.60 the same frame converges.
    """
    angles = np.asarray(joint_degrees, dtype=float).reshape(5)
    rate = np.asarray(counts_per_deg, dtype=float).reshape(5)
    delta = np.asarray(requested_delta_m, dtype=float).reshape(3)
    if axis not in (0, 1):
        raise ValueError("axis must be 0 (horizontal) or 1 (vertical)")
    if not np.isfinite(pixels) or pixels == 0:
        raise ValueError("pixel goal must be finite and non-zero")
    if not np.isfinite(cross_error_px):
        raise ValueError("cross-axis error must be finite")
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

    rejects: list[tuple[float, str, str]] = []
    for scale in (1.0, 1.35, 1.7, max_goal_scale):
        if scale > 1.0 and not allow_escalation:
            break
        target = float(pixels) * scale
        best = None
        refused: list[tuple[str, str, str]] = []
        for index, name in enumerate(ARM_JOINT_NAMES):
            response = float(per_count[axis, index])
            if abs(response) < 1e-6:
                continue
            direction = float(np.sign(target / response))
            if (name, int(direction)) in excluded_directions:
                refused.append((name, "no_response", f"{name} 该方向本轮对齐无可测响应"))
                continue
            gain = model.gain(name, direction)
            offset = model.offset(name, direction)
            # The offset is not scaled by the goal: it is a property of the axis,
            # so it is added after the pixel goal has been converted to counts.
            requested = int(round(target / response / gain + direction * offset))
            if requested == 0 or int(np.sign(requested)) != int(direction):
                refused.append((name, "zero", f"{name} 取整后为 0"))
                continue
            expected = float(np.sign(requested)) * max(abs(requested) - offset, 0.0) * gain
            if abs(expected) < min_effective_counts:
                refused.append((name, "deadband",
                                f"{name} 预计位移 {abs(expected):.1f} 计数 < 死区"
                                f"{min_effective_counts:.1f}"))
                continue
            if abs(requested) < model.minimum_request(name, requested):
                refused.append((name, "offset",
                                f"{name} 请求 {requested} 计数，未超过自身偏移 "
                                f"{model.minimum_request(name, requested):.1f}"))
                continue
            if abs(requested) > max_request_counts:
                refused.append((name, "count_cap",
                                f"{name} 请求 {requested} 计数 > 上限 "
                                f"{max_request_counts:.0f}"))
                continue
            # Same reason as in the multi-joint solve: the guard measures the
            # written goal, so the request carries the cap, not the prediction.
            if max(abs(requested), abs(expected)) / rate[index] > max_joint_step_deg:
                refused.append(
                    (name, "step_cap",
                     f"{name} 单步 {max(abs(requested), abs(expected)) / rate[index]:.1f}°"
                     f"超过 {max_joint_step_deg:.1f}°"))
                continue
            if not lower[index] < angles[index] + expected / rate[index] < upper[index]:
                refused.append((name, "travel", f"{name} 会走出关节行程"))
                continue
            cross = float(per_count[1 - axis, index]) * expected
            # ``cross`` is the marker motion this joint adds on the *other*
            # axis, and the tolerance is on how much that axis's error may
            # *grow*, not on how much it may change. Measuring the drift against
            # the requested axis alone assumes the other axis is already
            # correct; at 17:24 the residual was (25, -26) px and a vertical
            # step that also carried the marker 13 px back towards the
            # horizontal target was refused as drift in the direction it was
            # supposed to move. With ``cross_error_px == 0`` the rule is exactly
            # the original ``|cross| <= max_cross_fraction * |target|``.
            grown = abs(cross_error_px - cross) - abs(cross_error_px)
            if grown > max_cross_fraction * abs(target):
                refused.append((name, "cross",
                                f"{name} 把另一轴误差推大 {grown:.1f} px > 允许 "
                                f"{max_cross_fraction * abs(target):.1f} px"))
                continue
            if abs(float(per_count[2, index]) * expected) > max_size_change_px:
                refused.append((name, "size", f"{name} 会改变标记视在尺寸"))
                continue
            # A characterised axis wins over an unmeasured one even when it asks
            # for more counts. Once offsets are modelled, an axis with no prior
            # always looks cheapest -- choosing it would spend the next timeout
            # discovering a threshold that the other axes already have on record.
            rank = (0 if model.characterised(name) else 1, abs(requested))
            if best is None or rank < best[0]:
                best = (rank, index, requested, expected, gain)
        if best is None:
            if not refused:
                rejects.append((scale, "none", "没有任何关节能影响这条轴"))
            else:
                rejects.append((
                    scale,
                    ",".join(f"{joint}:{rule}" for joint, rule, _ in refused),
                    "; ".join(body for _, _, body in refused)))
            continue

        _, index, requested, expected, gain = best
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
            rejects.append((scale, f"{ARM_JOINT_NAMES[index]}:achieved",
                            f"{ARM_JOINT_NAMES[index]} 只达成 {achieved:.2f} < "
                            f"{min_achieved_fraction:.2f} 的目标"))
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
    raise StepInfeasible("没有单条关节能实现请求的图像位移（no single joint realizes "
                         "the requested image shift）：" + describe_rejects(rejects))


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
