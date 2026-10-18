"""Fixed-rule discrete servo: five image regions, one centimetre per step.

The rule is deliberately simple and observable:

* marker centre left / right / up / down of the central 40 % band
  -> move the camera 1 cm left / right / up / down;
* marker centre inside the band -> move the camera 1 cm forward;
* stop when the marker reaches a standoff (apparent size or estimated depth).

This module owns only the decision and its verification. It never opens a
camera, a serial port, or a motion lease. The device owner executes the plan and
reports encoder and image feedback, which is the only thing allowed to change
the learned response model or to stop the loop.

Why the rule alone is not enough on this arm, from the 2026-10-18 pose:

* 1 cm forward is about 14 ``shoulder_lift`` counts *plus* 12 opposite
  ``wrist_flex`` counts. The pair is required: lift alone pitches the camera by
  about 1 degree and moves the marker ~20 px, which is six to ten times the
  depth signal and looks like a centering error.
* 1 cm sideways is about 5 ``shoulder_pan`` counts, which is exactly the
  smallest amplitude this arm is known to execute. Below that the request is
  inside the deadband and nothing happens.

The controller therefore asks the planner for the largest executable step in
the requested direction, records when it had to escalate to the deadband floor,
and never treats an unverified step as progress.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from eai_robot.control.visual_servo.camera_step import (
    CameraStepPlan,
    ResponseModel,
    StepInfeasible,
    camera_delta_goal,
    depth_from_side_px,
    marker_side_px,
    plan_axis_step,
    plan_camera_step,
)
from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
from eai_robot.vision.geometry.scale import ExtentScale

REGIONS = ("left", "right", "up", "down", "center")


@dataclass(frozen=True)
class FixedRuleConfig:
    """Rule and guard parameters. All distances are metric, all angles in rad."""

    band: float = 0.4                 # central fraction that requests approach
    step_m: float = 0.01              # the rule's one-centimetre step
    stop_side_px: float = 120.0       # apparent-size standoff
    min_standoff_m: float = 0.10      # independent depth standoff
    approach_gate_px: float = 24.0    # centre error allowed before approaching
    focal_px: float = 554.0
    marker_length_m: float = 0.025
    # Round 1 validated the arm at a 5 degree single step (measured maximum
    # 4.64 deg). A 4 degree cap silently turns the rule's centimetre into
    # about 7.5 mm on a forward step, so the validated value is the default.
    max_joint_step_deg: float = 5.0
    min_effective_counts: float = 5.0
    max_request_counts: float = 18.0
    max_center_drift_px: float = 25.0
    max_step_drift_px: float = 25.0
    stall_limit: int = 2
    disagreement_limit: int = 2
    max_travel_mm: float = 100.0
    max_joint_excursion_deg: float = 40.0

    def validate(self) -> None:
        if not 0.1 <= self.band <= 0.9:
            raise ValueError("band must be within 0.1..0.9 of the image")
        if not 0 < self.step_m <= 0.02:
            raise ValueError("step_m must be within (0, 0.02]")
        if not 0 < self.stop_side_px:
            raise ValueError("stop_side_px must be positive")
        if not 0 < self.min_standoff_m < 1.0:
            raise ValueError("min_standoff_m must be within (0, 1)")
        if not 0 <= self.approach_gate_px <= 200:
            raise ValueError("approach_gate_px must be within 0..200")
        if not 0 < self.marker_length_m <= 0.3:
            raise ValueError("marker_length_m must be within (0, 0.3]")
        if not 0 < self.focal_px:
            raise ValueError("focal_px must be positive")


@dataclass(frozen=True)
class Observation:
    """One tracked target plus the measured joint angles at that instant.

    ``corners_px`` drives the interaction matrix and must be four *ordered*,
    persistently-corresponding points. ``extent_px`` is the apparent size the
    tracker actually measured; it is not recomputed from the corners, because a
    mask's average extent and the mean edge of its four-point approximation are
    different quantities, and using one for depth while the other drives the
    step is how a systematic depth error gets built in. ``None`` means "this
    observation predates the tracker contract -- take the corner mean edge",
    which is exactly the ArUco behaviour the first round validated.
    """

    corners_px: np.ndarray
    image_size: tuple[int, int]
    joint_degrees: np.ndarray
    extent_px: float | None = None
    extent_name: str = "side"

    def validated(self) -> "Observation":
        corners = np.asarray(self.corners_px, dtype=float).reshape(4, 2)
        angles = np.asarray(self.joint_degrees, dtype=float).reshape(5)
        if len(self.image_size) != 2:
            raise ValueError("image size must be two positive integers")
        width, height = self.image_size
        if width <= 0 or height <= 0:
            raise ValueError("image size must be two positive integers")
        if not np.all(np.isfinite(corners)) or not np.all(np.isfinite(angles)):
            raise ValueError("observation must be finite")
        # An object touching the frame edge is trackable, so the bound is the
        # frame size and not width-1. The geometry layer uses the same rule.
        if np.any(corners < 0) or np.any(corners > [width, height]):
            raise ValueError("marker corners must be inside the image")
        if self.extent_px is not None:
            if not np.isfinite(self.extent_px) or self.extent_px <= 0:
                raise ValueError("apparent extent must be positive")
        return Observation(corners, (int(width), int(height)), angles,
                           self.extent_px, self.extent_name)

    @classmethod
    def from_geometry(cls, geometry, joint_degrees) -> "Observation":
        """Adapt any tracker's geometry into the rule's input.

        The four support points come from ``geometry.as_quad``, which is
        order-stable and derived from the tracker's own points, and the extent is
        carried across unchanged so depth and step consult the same measurement.
        """
        return cls(geometry.as_quad, tuple(geometry.image_size),
                   np.asarray(joint_degrees, dtype=float).reshape(5),
                   float(geometry.extent_px), geometry.extent_name)

    @property
    def center_px(self) -> np.ndarray:
        return np.asarray(self.corners_px, dtype=float).reshape(4, 2).mean(axis=0)

    @property
    def side_px(self) -> float:
        if self.extent_px is not None:
            return float(self.extent_px)
        return marker_side_px(self.corners_px)


@dataclass(frozen=True)
class Action:
    """What the device owner should do next."""

    kind: str                      # move | hold | reached
    phase: str                     # align | approach
    region: str
    reason: str = ""
    plan: CameraStepPlan | None = None
    side_px: float = 0.0
    depth_m: float = 0.0
    center_error_px: float = 0.0

    @property
    def movable(self) -> bool:
        return self.kind == "move" and self.plan is not None


@dataclass(frozen=True)
class StepReport:
    """Measured outcome of one executed plan, used for verification only."""

    actual_counts: dict[str, int]
    gain_updates: dict[str, float]
    center_shift_px: np.ndarray
    side_change_px: float
    verdict: str
    reason: str = ""


def image_region(center_px: Sequence[float], image_size: Sequence[int], band: float) -> str:
    """Classify a point into the rule's five regions.

    The largest normalized deviation from the image centre wins, matching the
    rehearsal controller, so a diagonal offset produces one axis of motion.
    """
    if not 0.1 <= band <= 0.9:
        raise ValueError("band must be within 0.1..0.9 of the image")
    width, height = float(image_size[0]), float(image_size[1])
    if width <= 0 or height <= 0:
        raise ValueError("image size must be positive")
    u, v = float(center_px[0]) / width, float(center_px[1]) / height
    low, high = (1.0 - band) / 2.0, (1.0 + band) / 2.0
    outside = (u < low) or (u > high) or (v < low) or (v > high)
    if not outside:
        return "center"
    scores = {"left": low - u, "right": u - high, "up": low - v, "down": v - high}
    return max(REGIONS[:4], key=lambda name: scores[name])


def region_delta(region: str, step_m: float) -> np.ndarray:
    """Camera-frame translation for one region, in the camera's optical axes."""
    if region not in REGIONS:
        raise ValueError(f"unknown region: {region}")
    if not 0 < step_m <= 0.02:
        raise ValueError("step_m must be within (0, 0.02]")
    return {
        "left": np.array([-step_m, 0.0, 0.0]),
        "right": np.array([step_m, 0.0, 0.0]),
        "up": np.array([0.0, -step_m, 0.0]),
        "down": np.array([0.0, step_m, 0.0]),
        "center": np.array([0.0, 0.0, step_m]),
    }[region]


class FixedRuleController:
    """Decide one step at a time and verify what the arm actually did."""

    def __init__(
        self,
        fk,
        gripper_T_camera,
        config: FixedRuleConfig,
        counts_per_deg: Sequence[float],
        camera_matrix: np.ndarray,
        *,
        reference_degrees: Sequence[float],
        origin_m: Sequence[float],
        gains: ResponseModel | None = None,
        scale: ExtentScale | None = None,
    ):
        config.validate()
        self.fk = fk
        self.mount = gripper_T_camera
        self.config = config
        self.rates = np.asarray(counts_per_deg, dtype=float).reshape(5)
        self.camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(3, 3)
        self.reference = np.asarray(reference_degrees, dtype=float).reshape(5)
        self.origin = np.asarray(origin_m, dtype=float).reshape(3)
        self.gains = gains or ResponseModel()
        # ``None`` keeps the first round's pinhole-plus-marker-size route, which
        # is the validated path. A scale measured against a reference distance is
        # the only route available to an object whose real size nobody knows.
        self.scale = scale
        self.steps = 0
        self.halted = ""
        self._stalls = 0
        self._disagreements = 0
        self._depth_progress_mm = 0.0
        self._reported_reached = False

    # -- observation ------------------------------------------------------
    def _camera(self, angles: np.ndarray):
        return self.fk.forward_kinematics(angles) @ self.mount

    def depth_m(self, observation: Observation) -> float:
        """Metric depth from apparent size, reported and never advertised.

        With a scale the only thing that matters is that the constant and the
        extent were measured under the same convention: ``sqrt_area`` and
        ``side`` differ by the square's aspect ratio, and mixing them turns a
        depth into a plausible-looking wrong number.
        """
        side = observation.side_px
        if self.scale is None:
            return depth_from_side_px(side, self.config.focal_px,
                                      self.config.marker_length_m)
        if observation.extent_name != self.scale.extent_name:
            raise ValueError(
                f"extent convention mismatch: tracker reports "
                f"{observation.extent_name!r}, the scale was measured under "
                f"{self.scale.extent_name!r}")
        return self.scale.depth_m(side)

    def use_scale(self, scale: ExtentScale) -> None:
        """Attach the distance scale once the first observation exists.

        The two-phase form is forced by arithmetic, not by taste: the reference
        route's constant is ``depth × extent_px``, so it cannot be known before
        the tracker has reported an extent. Everything that consumes the scale
        (``depth_m``, ``depth_of``) is called after this point.
        """
        self.scale = scale

    def travel_mm(self, angles: Sequence[float]) -> float:
        position = self._camera(np.asarray(angles, dtype=float).reshape(5))[:3, 3]
        return float(np.linalg.norm(position - self.origin) * 1000.0)

    def depth_of(self, observation: Observation) -> float:
        """``depth_m`` without the pairing check, for record-keeping only.

        ``record`` runs after the step, on a target the decision already
        accepted. Re-raising the convention error there would turn a logged
        figure into a mid-motion crash.
        """
        side = observation.side_px
        if self.scale is None:
            return depth_from_side_px(side, self.config.focal_px,
                                      self.config.marker_length_m)
        return self.scale.depth_m(side)

    # -- decision ---------------------------------------------------------
    def decide(self, observation: Observation) -> Action:
        """Return the next action. Pure: no I/O, no hidden state change."""
        observation = observation.validated()
        if self.halted:
            return Action("hold", "hold", "unknown", self.halted,
                          side_px=observation.side_px,
                          depth_m=self.depth_of(observation))
        config = self.config
        side = observation.side_px
        try:
            depth = self.depth_m(observation)
        except ValueError as exc:
            # A tracker whose extent convention does not match the scale cannot
            # produce a depth, and the depth is what the standoff and the
            # approach decision are read from. Hold before motion rather than
            # stepping on a number that means something else.
            self.halted = str(exc)
            return Action("hold", "hold", "unknown", self.halted, side_px=side)
        center = observation.center_px
        width, height = observation.image_size
        error = np.array([width / 2.0, height / 2.0]) - center
        error_norm = float(np.linalg.norm(error))
        region = image_region(center, observation.image_size, config.band)

        if side >= config.stop_side_px or depth <= config.min_standoff_m:
            self._reported_reached = True
            return Action("reached", "approach", region,
                          f"standoff: side {side:.1f} px, depth {depth * 1000:.0f} mm",
                          side_px=side, depth_m=depth, center_error_px=error_norm)

        angles = observation.joint_degrees
        if self.travel_mm(angles) >= config.max_travel_mm:
            self.halted = f"travel budget {config.max_travel_mm:.0f} mm reached"
            return Action("hold", "hold", region, self.halted,
                          side_px=side, depth_m=depth, center_error_px=error_norm)
        if float(np.max(np.abs(angles - self.reference))) >= config.max_joint_excursion_deg:
            self.halted = "joint excursion budget reached"
            return Action("hold", "hold", region, self.halted,
                          side_px=side, depth_m=depth, center_error_px=error_norm)

        phase = "approach" if region == "center" and error_norm <= config.approach_gate_px else "align"
        if phase == "align" and region == "center":
            # Inside the central band but not square enough to spend a forward
            # step on: correct the dominant residual axis first. ``error`` is
            # target minus measured centre, so a positive x error means the
            # marker sits left of centre and the camera must move left.
            if abs(error[0]) >= abs(error[1]):
                region = "left" if error[0] > 0 else "right"
            else:
                region = "up" if error[1] > 0 else "down"
        delta = region_delta(region, config.step_m)
        depths = np.full(4, depth)
        goal = camera_delta_goal(observation.corners_px, depths, self.camera_matrix, delta)
        try:
            if goal.dominant == "depth":
                plan = plan_camera_step(
                    self.fk, angles, self.mount, delta, observation.corners_px, depths,
                    self.camera_matrix,
                    counts_per_deg=self.rates, gains=self.gains,
                    max_joint_step_deg=config.max_joint_step_deg,
                    min_effective_counts=config.min_effective_counts,
                    max_request_counts=config.max_request_counts,
                    max_center_drift_px=config.max_step_drift_px,
                )
            else:
                # Lateral and vertical steps use one joint so the measured pixel
                # response is attributable; a mixed solution cannot be verified
                # from a single frame without per-axis attribution.
                axis = 0 if goal.dominant == "horizontal" else 1
                plan = plan_axis_step(
                    self.fk, angles, self.mount, observation.corners_px, depths,
                    self.camera_matrix, axis=axis,
                    pixels=goal.du_px if axis == 0 else goal.dv_px,
                    requested_delta_m=delta,
                    counts_per_deg=self.rates, gains=self.gains,
                    max_joint_step_deg=config.max_joint_step_deg,
                    min_effective_counts=config.min_effective_counts,
                    max_request_counts=config.max_request_counts,
                )
        except StepInfeasible as exc:
            self.halted = f"no executable step: {exc}"
            return Action("hold", phase, region, self.halted,
                          side_px=side, depth_m=depth, center_error_px=error_norm)
        return Action("move", phase, region, "", plan, side, depth, error_norm)

    # -- verification -----------------------------------------------------
    def record(
        self,
        plan: CameraStepPlan,
        before: Observation,
        after: Observation,
        before_raw: Mapping[str, int],
        after_raw: Mapping[str, int],
    ) -> StepReport:
        """Fold one executed step's feedback into the model and the guards."""
        self.steps += 1
        actual = {}
        updates = {}
        for name, requested in zip(ARM_JOINT_NAMES, plan.requested_counts):
            if requested == 0:
                continue
            moved = int(after_raw[name]) - int(before_raw[name])
            actual[name] = moved
            if abs(requested) >= self.config.min_effective_counts and abs(moved) >= 1:
                # One step confounds a proportional shortfall with the fixed
                # arrival residual, so blend instead of replacing the gain.
                measured = abs(moved) / abs(int(requested))
                before_gain = self.gains.gain(name, requested)
                blended = 0.5 * before_gain + 0.5 * measured
                self.gains = self.gains.updated(name, int(np.sign(requested)), blended)
                updates[name] = round(self.gains.gain(name, requested) - before_gain, 4)

        shift = after.center_px - before.center_px
        side_change = after.side_px - before.side_px
        # Both depths go through the configured scale so that the reported
        # progress is measured the same way the decision was made. Mixing the
        # pinhole route in here would make the two disagree about the sign of
        # progress whenever the tracker's convention is not ``side``.
        depth_change_mm = (self.depth_of(before) - self.depth_of(after)) * 1000.0

        verdict, reason = "ok", ""
        if actual and max(abs(value) for value in actual.values()) < 2:
            self._stalls += 1
            verdict = "stalled"
            reason = "encoder did not move outside noise"
        else:
            self._stalls = 0

        expected_side = float(plan.predicted_image_change[2])
        if plan.goal.dominant == "depth" and abs(expected_side) >= 0.5:
            # Only a cumulative disagreement is actionable: one step's size
            # change is close to the corner-detection noise at 25 mm markers.
            self._depth_progress_mm += depth_change_mm
            if side_change * expected_side < 0:
                self._disagreements += 1
            else:
                self._disagreements = 0

        # Only the part of the shift the plan did not predict is a drift. A
        # one-centimetre vertical step is meant to move the marker about 28 px,
        # so comparing the raw shift against a 25 px limit would flag every
        # successful correction as a fault.
        residual = shift - np.asarray(plan.predicted_image_change[:2], dtype=float)
        if np.linalg.norm(residual) > self.config.max_center_drift_px:
            verdict = "center_drift"
            reason = (f"centre moved {np.linalg.norm(residual):.1f} px more "
                      "than the plan predicted")

        if self._stalls >= self.config.stall_limit:
            self.halted = "encoder stall: two consecutive steps produced no motion"
            verdict, reason = "stall_stop", self.halted
        if self._disagreements >= self.config.disagreement_limit:
            self.halted = ("apparent size disagrees with commanded depth: "
                           "check marker size, focal length and hand-eye axis")
            verdict, reason = "depth_disagreement", self.halted

        return StepReport(actual, updates, shift, side_change, verdict, reason)

    def snapshot(self) -> dict[str, object]:
        return {
            "steps": self.steps,
            "halted": self.halted,
            "response_gains": self.gains.describe(),
            "depth_progress_mm": round(self._depth_progress_mm, 2),
            "stalls": self._stalls,
            "disagreements": self._disagreements,
        }
