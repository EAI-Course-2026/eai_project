"""The five-region rule on any tracked object, on the real arm.

This is the same rule as ``fixed_rule_servo``: centre in the left / right / up /
down region, step the camera 1 cm that way; centre inside the middle band, step
1 cm forward; stop at a standoff. What changes is where the target comes from.

The ArUco runner reads four corners and converts the apparent side into a depth
with the calibrated focal length and the printed marker size. That works because
a marker has a known, square, exactly-decodable outline. An arbitrary object has
none of those properties, so this runner takes three inputs instead:

``--roi``
    A region the operator selected. Class-agnostic texture tracking; nothing
    knows what the object is.
``--prompt`` / ``--label``
    A detector that can find the object again after a loss.
``--reference-depth-m`` *or* ``--target-extent-m``
    One measurement that turns apparent size into distance. Exactly one is
    required: the first says "the object was this far away when its extent was
    this many pixels", the second says "its real size is this many metres and
    the calibration is right". They produce the same constant, as
    ``vision.geometry.scale`` explains.

The tracker is asked for a *target*, not for a marker, so the servo can be run
against a hand, a cup, a fiducial, or the black ball with no change to the
decision layer.

What this runner deliberately does not do:

* It does not relax any guard. The same ``BoundedJointMotion`` owns the
  workspace, joint, travel and step budgets, the fresh-target check before every
  goal write, the torque/status gate and the arrival lease.
* It does not treat a coasted pose as a measurement. A reading is accepted only
  while ``usable`` and inside ``--tracker-max-age-s``; the record says which
  state produced each step.
* It does not claim metric accuracy. The depth comes from one operator-supplied
  measurement, so its relative error is that measurement's relative error. The
  stopping criterion that decides the run is the pixel standoff.
"""
from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from eai_robot.config import ROOT
from eai_robot.control.execution.feedback import (
    arm_degrees_from_raw,
    read_raw_motors,
)
from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
from eai_robot.control.visual_servo.camera_step import ResponseModel, counts_per_degree
from eai_robot.control.visual_servo.fixed_rule import (
    phase_label,
    region_label,
    MEASURED_COUNT_OFFSETS,
    FixedRuleConfig,
    FixedRuleController,
    Observation,
)
from eai_robot.vision.detection.ball import BallDetection
from eai_robot.vision.geometry.scale import ExtentScale
from eai_robot.vision.geometry.targets import TargetGeometry
from eai_robot.vision.perception.async_detector import AsyncDetector
from eai_robot.vision.perception.registry import create as create_detector
from eai_robot.vision.perception.spec import ModelSpec
from eai_robot.vision.tracking.base import TrackerSpec, TrackerUnavailable, stale
from eai_robot.vision.tracking.fused import FusedTracker
from eai_robot.vision.tracking.support_points import SupportPointTracker

from eai_robot.applications.visual_servo.fixed_rule_servo import (
    initial_reference,
    camera_origin,
    load_camera_matrix,
    load_hand_eye,
)
from eai_robot.applications.visual_servo.hold_preview import overlay_label
from eai_robot.applications.visual_servo.joint_center import (
    BoundedJointMotion,
    MotionResponseTimeout,
    TargetLostError,
    wait_for_target,
)
from eai_robot.applications.visual_servo.shared_camera import PreviewFrameUnavailable


class TrackerDetectionAdapter:
    """Give the motion guard and the preview a ``detect``-shaped tracker.

    ``BoundedJointMotion`` was written against a detector that returns
    ``(BallDetection | None, mask)`` and uses it for two things: refusing to
    write a goal unless the target is visible in a frame read immediately
    before, and deciding that a target loss during motion is a hold. Both are
    properties of the *target*, not of a detector, so the adapter answers them
    from the tracker and leaves the guard untouched.

    It returns ``None`` when the geometry is unusable or older than
    ``max_age_s``. That makes staleness a hold before a goal write, which is
    exactly the contract the guard already enforces for a detector that returned
    nothing.
    """

    def __init__(self, tracker: FusedTracker, *, max_age_s: float = 0.25,
                 clock=time.monotonic):
        if not np.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        self.tracker = tracker
        self.max_age_s = float(max_age_s)
        self._clock = clock
        self.last_reason = ""

    def detect(self, frame_rgb):
        now = self._clock()
        reading = self.tracker.update(frame_rgb, now)
        geometry = reading.geometry
        mask = np.zeros(np.asarray(frame_rgb).shape[:2], dtype=np.uint8)
        reason = stale(geometry, now, self.max_age_s)
        if reason is not None or not reading.usable:
            self.last_reason = reason or reading.status.reason or "tracker not usable"
            return None, mask
        self.last_reason = ""
        polygon = np.rint(geometry.as_quad).astype(np.int32)
        cv2.fillConvexPoly(mask, polygon, 255)
        return geometry_annotation(geometry), mask


def geometry_annotation(geometry: TargetGeometry) -> BallDetection:
    """Render any geometry as the annotation type the preview already accepts.

    The equivalent radius uses the square-equivalent area, which is the same
    convention ``from_box`` and ``from_mask`` use, so the drawn circle grows and
    shrinks with the extent the servo is actually closing on.
    """
    area = float(geometry.extent_px) ** 2
    centre = geometry.center_px
    return BallDetection(float(centre[0]), float(centre[1]),
                         float(np.sqrt(area / np.pi)), area, 1.0,
                         float(geometry.confidence))


def detector_options(pairs) -> dict[str, object]:
    """``KEY=VALUE`` list to a dict of strings.

    Detector-specific knobs (a blob threshold, a confidence floor) belong to the
    detector, not to this runner's argument list, so they arrive as an open set
    and are passed through unchanged. Values stay strings and each detector
    coerces what it recognises; guessing types here would silently turn "1e-2"
    into a string a model then treats as a path.
    """
    options: dict[str, object] = {}
    for item in pairs or ():
        if "=" not in item:
            raise ValueError(f"detector option must be KEY=VALUE, got {item!r}")
        key, value = item.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"detector option has an empty key: {item!r}")
        options[key] = value.strip()
    return options


def build_tracker(args) -> FusedTracker:
    """Assemble the tracker the operator asked for, or explain why it is missing."""
    support = SupportPointTracker(
        max_gap_s=max(0.1, 2.0 / max(args.camera_fps, 1)),
        min_inliers=max(4, int(args.tracker_min_inliers)),
    )
    kind = args.tracker
    wants_detector = kind in ("fused", "detector")
    if kind == "support" and args.prompt:
        raise TrackerUnavailable(
            "--prompt needs --tracker fused or --tracker detector: "
            "the support-point tracker has no semantics to match a prompt with")
    if kind == "support" and not args.roi:
        raise TrackerUnavailable(
            "--tracker support needs --roi x,y,w,h: it is class-agnostic and "
            "cannot find the object on its own")
    if kind == "detector" and not (args.prompt or args.label):
        raise TrackerUnavailable(
            "--tracker detector needs --prompt or --label; it has no texture "
            "tracking to fall back on")
    detector = None
    if wants_detector:
        spec = ModelSpec(
            name=args.detector,
            weights=Path(args.detector_weights) if args.detector_weights else None,
            device=args.model_device,
            dtype=args.model_dtype,
            input_size=None,
            allow_download=bool(args.allow_weight_download),
            options=detector_options(args.detector_option),
        )
        detector = AsyncDetector(create_detector(spec),
                                 failure_limit=args.detector_failure_limit)
    return FusedTracker(
        support, detector,
        selection=args.selection or "score",
        max_coast_s=args.tracker_max_coast_s,
        agree_centre_px=args.tracker_agreement,
        agree_extent_ratio=args.tracker_agreement,
        acquire_timeout_s=args.detector_timeout_s,
        seed_margin=args.tracker_seed_margin,
        flow=kind != "detector",
    )


def tracker_spec(args) -> TrackerSpec:
    roi = None
    if args.roi:
        values = [float(item) for item in args.roi.split(",")]
        if len(values) != 4:
            raise ValueError("--roi must be four comma-separated numbers x,y,w,h")
        roi = tuple(values)
    return TrackerSpec(roi_px=roi, prompt=args.prompt, label=args.label,
                       target_id=args.target_id or "object:selection").validate()


def resolve_scale(args, geometry: TargetGeometry,
                  focal_px: float) -> ExtentScale:
    """One measurement turns this tracker's apparent extent into a distance.

    Both routes are accepted because the operator may know either the distance
    they held the object at or the object's real size. Whichever they give, the
    constant is stored with the extent convention it was measured under, so a
    later tracker reporting a different convention is refused instead of
    silently producing a plausible wrong depth.
    """
    if (args.reference_depth_m is None) == (args.target_extent_m is None):
        raise ValueError(
            "give exactly one of --reference-depth-m or --target-extent-m: "
            "the rule needs one distance scale for an arbitrary object")
    if args.reference_depth_m is not None:
        return ExtentScale.from_reference(args.reference_depth_m,
                                          float(geometry.extent_px),
                                          geometry.extent_name)
    return ExtentScale.from_pinhole(focal_px, args.target_extent_m,
                                    geometry.extent_name)


def sample_observation(runtime, tracker, mapper, samples, timeout_s, max_age_s):
    """Median of several stationary frames, paired with the pose read after them.

    One tracked extent on a low-texture object carries two or three pixels of
    noise while the depth signal of one 1 cm step is about 1.5 px, so a single
    frame is not enough evidence to spend a step on. The arm is stationary while
    sampling, so the median is taken over the same physical configuration and
    does not average motion.

    Support points are carried as ``as_quad`` (always four, order-stable) and the
    extent is carried separately, so the depth never consults a number that was
    recomputed from the points afterwards.
    """
    quads, extents, states = [], [], set()
    last = None
    deadline = time.monotonic() + timeout_s
    while len(quads) < samples and time.monotonic() < deadline:
        frame = runtime.read_frame()
        now = time.monotonic()
        reading = tracker.update(frame, now)
        last = (frame, reading)
        if not reading.usable or stale(reading.geometry, now, max_age_s) is not None:
            quads.clear()
            extents.clear()
            continue
        quads.append(np.asarray(reading.geometry.as_quad, dtype=float))
        extents.append(float(reading.geometry.extent_px))
        states.add(reading.status.state)
    if last is None:
        raise TargetLostError("no frame available while sampling the target")
    if len(quads) < samples:
        raise TargetLostError(
            f"only {len(quads)}/{samples} usable tracker frames within {timeout_s:.1f}s "
            f"({last[1].status.reason})")
    quad = np.median(np.stack(quads), axis=0)
    extent = float(np.median(np.asarray(extents)))
    raw = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
    degrees = arm_degrees_from_raw(mapper, raw)
    height, width = last[0].shape[:2]
    geometry = TargetGeometry.from_support_points(
        quad, extent, (width, height), last[1].geometry.captured_at,
        last[1].geometry.target_id, source=last[1].geometry.source,
        extent_name=last[1].geometry.extent_name)
    return Observation.from_geometry(geometry, degrees), raw, last[0], states


def build_config(args, matrix) -> FixedRuleConfig:
    return FixedRuleConfig(
        band=args.rule_band,
        step_m=args.rule_step_mm / 1000.0,
        stop_side_px=args.rule_stop_side_px,
        min_standoff_m=args.rule_min_standoff_m,
        approach_gate_px=args.rule_gate_px,
        focal_px=float(matrix[0, 0]),
        # Unused once a scale is supplied, but the config is shared with the
        # marker path and a validated default keeps one config type.
        marker_length_m=args.marker_length_m,
        max_joint_step_deg=args.max_joint_step_deg,
        min_effective_counts=5.0,
        max_request_counts=float(args.rule_max_request_counts),
        max_travel_mm=args.max_travel_mm,
        max_joint_excursion_deg=args.max_joint_excursion_deg,
    )


def _observe(runtime, tracker, adapter, mapper, args, stop, preview):
    samples = max(3, args.feature_samples)
    try:
        return sample_observation(runtime, tracker, mapper, samples,
                                  args.detection_timeout_s,
                                  args.tracker_max_age_s)
    except (PreviewFrameUnavailable, TimeoutError, TargetLostError) as exc:
        # Re-acquire before waiting. ``wait_for_target`` polls the tracker until
        # it reports a target, so a tracker that cannot recover on its own would
        # make it wait until the operator stops the run. A detector-backed
        # tracker can re-acquire by itself; the region tracker cannot, because
        # its only seed is a selection that no longer exists.
        spec = tracker_spec(args)
        reading = tracker.start(runtime.read_frame(), spec, time.monotonic())
        if not reading.usable:
            if tracker.detector is None:
                raise TargetLostError(
                    f"the region tracker lost the target "
                    f"({reading.status.reason}); re-select --roi") from exc
            wait_for_target(runtime, adapter, preview, stop)
            tracker.start(runtime.read_frame(), spec, time.monotonic())
        return sample_observation(runtime, tracker, mapper, samples,
                                  args.detection_timeout_s,
                                  args.tracker_max_age_s)


def print_tracker_summary(tracker) -> None:
    """One line of tracker truth: state, how often it had to recover, and model health.

    Read with ``.get`` rather than by key: this runs at the end of a run,
    including runs that ended badly, and a summary must never be the thing that
    turns a completed run into a traceback.
    """
    snapshot = tracker.describe()
    print(f"跟踪器（tracker）{snapshot.get('kind', '?')}：状态 "
          f"{snapshot.get('state', '?')}，"
          f"重新捕获 {snapshot.get('reacquisitions', 0)} 次，"
          f"重新锚定 {snapshot.get('reanchors', 0)} 次，"
          f"约定跳过 {snapshot.get('convention_skips', 0)} 次", flush=True)
    health = snapshot.get("detector_health")
    if health:
        print(f"检测器（detector）：p50 {health['latency_p50_s'] * 1000:.0f} ms，"
              f"p95 {health['latency_p95_s'] * 1000:.0f} ms，"
              f"丢弃 {health['dropped']}，失败 {health['failures']}，"
              f"降级 {health['degraded']}", flush=True)


def run_object_rule(runtime, args, stop, preview) -> int:
    """Run the five-region rule against a tracked object until standoff or a guard."""
    mapper = runtime._require_mapper()
    fk = runtime.fk
    if fk is None:
        raise RuntimeError("FK solver is not initialized")

    mount = load_hand_eye(args.hand_eye)
    matrix = load_camera_matrix(args.intrinsics)
    config = build_config(args, matrix)
    config.validate()
    rates = counts_per_degree(mapper.calibration)

    tracker = build_tracker(args)
    adapter = TrackerDetectionAdapter(tracker, max_age_s=args.tracker_max_age_s)
    initial_raw, initial_q = initial_reference(runtime, mapper, args)
    origin = camera_origin(fk, mount, initial_q)

    path = ROOT / "local_artifacts/experiments" / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + "_object-rule.jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    runtime.hold_reason = "USER STOP"

    def record(event, **data):
        with path.open("a") as stream:
            stream.write(json.dumps(dict(time=time.monotonic(), event=event, **data)) + "\n")

    controller = FixedRuleController(
        fk, mount, config, rates, matrix,
        reference_degrees=initial_q, origin_m=origin,
        gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS),
        unresponsive_directions=getattr(runtime, "_alignment_unresponsive", ()))
    bounds = BoundedJointMotion(
        runtime, adapter, mapper, args, initial_q, origin,
        record, preview, stop,
        position_of=lambda q: camera_origin(fk, mount, q),
        joints=ARM_JOINT_NAMES)

    try:
        time.sleep(args.torque_settle_s)
        frame = runtime.read_frame()
        reading = tracker.start(frame, tracker_spec(args), time.monotonic())
        if not reading.usable:
            # Never leave the operator with "acquisition failed: " -- a hold
            # reason that names no cause is unreadable on the bench.
            cause = reading.status.reason or f"tracker reported {reading.status.state}"
            runtime.hold_reason = f"acquisition failed: {cause}"
            record("acquire_failed", reason=cause, tracker=tracker.describe())
            print(f"\n{runtime.hold_reason}；已保持。", flush=True)
            return 0
        scale = resolve_scale(args, reading.geometry, float(matrix[0, 0]))
        controller.use_scale(scale)
        record("acquired", target_id=reading.geometry.target_id,
               centre=reading.geometry.center_px.tolist(),
               extent_px=reading.geometry.extent_px,
               extent_name=reading.geometry.extent_name,
               state=reading.status.state,
               scale=scale.describe(), tracker=tracker.describe())
        print(
            f"\n任意物体规律（object rule）：每步 {config.step_m * 1000:.0f} mm，"
            f"中央带 {config.band:.0%}，边长 {config.stop_side_px:.0f} px 停靠，"
            f"接近门限 {config.approach_gate_px:.0f} px。跟踪器 {args.tracker}，"
            f"尺度量 {reading.geometry.extent_name} "
            f"{reading.geometry.extent_px:.1f} px，深度 "
            f"{scale.depth_m(reading.geometry.extent_px) * 1000:.0f} mm（来自一次"
            f"参考测量）。预算：行程 {config.max_travel_mm:.0f} mm / 关节偏移 "
            f"{config.max_joint_excursion_deg:.0f} 度 / 步数上限 {args.rule_max_steps} 步。日志：{path}", flush=True)
        record("settings", band=config.band, step_mm=config.step_m * 1000,
               stop_side_px=config.stop_side_px, gate_px=config.approach_gate_px,
               focal_px=matrix[0, 0], tracker_kind=args.tracker,
               detector=args.detector, prompt=args.prompt, label=args.label,
               roi=args.roi, selection=args.selection,
               max_coast_s=args.tracker_max_coast_s,
               max_age_s=args.tracker_max_age_s,
               max_travel_mm=config.max_travel_mm,
               max_joint_excursion_deg=config.max_joint_excursion_deg,
               max_joint_step_deg=config.max_joint_step_deg,
               max_steps=args.rule_max_steps, max_runtime_s=args.max_runtime_s,
               intrinsics=str(args.intrinsics), hand_eye=str(args.hand_eye),
               initial=initial_raw, reference=initial_q.tolist())

        observation, raw, frame, states = _observe(
            runtime, tracker, adapter, mapper, args, stop, preview)
        print_tracker_summary(tracker)
        record("observation", centre=observation.center_px.tolist(),
               extent_px=observation.side_px,
               extent_name=observation.extent_name,
               depth_m=controller.depth_m(observation),
               degrees=observation.joint_degrees.tolist(), raw=raw,
               states=sorted(states))

        transient = 0
        while not stop.stop_requested.is_set():
            if args.max_runtime_s > 0 and time.monotonic() - started >= args.max_runtime_s:
                runtime.hold_reason = "TIME LIMIT"
                record("hold", reason=runtime.hold_reason, **controller.snapshot())
                print("\n运行时长到达上限，保持（runtime reached; HOLDING）", flush=True)
                break
            if controller.steps >= args.rule_max_steps:
                runtime.hold_reason = "step budget reached"
                record("hold", reason=runtime.hold_reason, **controller.snapshot())
                print(f"\n已执行 {args.rule_max_steps}/{args.rule_max_steps} 步，步数预算已用尽（HOLDING）", flush=True)
                break

            action = controller.decide(observation)
            status = (
                f"{phase_label(action.phase)} {region_label(action.region)} "
                f"尺度={action.side_px:.0f}px 深度={action.depth_m * 1000:.0f}mm "
                f"误差={action.center_error_px:.0f}px 步数={controller.steps} "
                f"{tracker.status.state}"
            )
            print(f"\r{status:82s}", end="", flush=True)
            if preview is not None:
                label = (f"{action.phase.upper()} {action.region} extent={action.side_px:.0f}px "
                         f"d={action.depth_m * 1000:.0f}mm e={action.center_error_px:.0f}px "
                         f"step={controller.steps}/{args.rule_max_steps}")
                if action.reason:
                    label += " | " + overlay_label(action.reason)
                preview.show(frame, geometry_annotation(TargetGeometry.from_support_points(
                    observation.corners_px, observation.side_px,
                    observation.image_size, time.monotonic(), "object",
                    extent_name=observation.extent_name)), label)

            if action.kind == "reached":
                runtime.hold_reason = "已到停靠（standoff reached）"
                record("reached", reason=action.reason,
                       centre=observation.center_px.tolist(),
                       extent_px=observation.side_px, depth_m=action.depth_m,
                       tracker=tracker.describe(), **controller.snapshot())
                print(f"\n已到停靠（standoff reached）：{action.reason}。保持。", flush=True)
                print_tracker_summary(tracker)
                return 0
            if not action.movable:
                runtime.hold_reason = action.reason or "no executable step"
                record("hold", reason=runtime.hold_reason,
                       tracker=tracker.describe(), **controller.snapshot())
                print(f"\n{runtime.hold_reason}；已保持。", flush=True)
                print_tracker_summary(tracker)
                return 0

            plan = action.plan
            target = dict(raw)
            for name, counts in zip(ARM_JOINT_NAMES, plan.requested_counts):
                target[name] += int(counts)
            if not bounds.valid(target, raw):
                runtime.hold_reason = (
                    "planned step not permitted: "
                    + (bounds.reject_reason(target, raw) or "workspace guard"))
                record("hold", reason=runtime.hold_reason, plan=plan.describe())
                print(f"\n{runtime.hold_reason}；已保持。", flush=True)
                return 0

            record("plan", phase=action.phase, region=action.region,
                   plan=plan.describe(), degrees=observation.joint_degrees.tolist(),
                   tracker_state=tracker.status.state)
            try:
                actual = bounds.move(target)
            except MotionResponseTimeout as exc:
                if action.phase != "align" or not controller.record_no_response(
                        plan, exc.requested_counts, exc.peak_motion):
                    raise
                runtime._alignment_unresponsive = set(controller.unresponsive_directions)
                record("alignment_no_response", requested_counts=exc.requested_counts,
                       peak_motion=exc.peak_motion, **controller.snapshot())
                print("\n该轴此方向无可测响应，已恢复旧保持目标；改选其他对齐轴，"
                      "不增大请求（alignment fallback）。", flush=True)
                observation, raw, frame, states = _observe(
                    runtime, tracker, adapter, mapper, args, stop, preview)
                continue
            except (TargetLostError, PreviewFrameUnavailable, TimeoutError) as exc:
                # A tracker that loses the object mid-step is a normal event for
                # a low-texture target; re-acquire a bounded number of times
                # before giving up. ``_observe`` owns the re-acquisition itself.
                record("transient", reason=str(exc),
                       tracker_reason=adapter.last_reason)
                transient += 1
                if transient > args.tracker_reacquire_attempts:
                    raise
                observation, raw, frame, states = _observe(
                    runtime, tracker, adapter, mapper, args, stop, preview)
                continue

            time.sleep(0.15)
            after, after_raw, frame, states = _observe(
                runtime, tracker, adapter, mapper, args, stop, preview)
            report = controller.record(
                plan, observation, after,
                bounds.last_before or raw, actual)
            transient = 0
            record("feedback", verdict=report.verdict, reason=report.reason,
                   actual_counts=report.actual_counts,
                   raw_before=bounds.last_before or raw, raw_at_settle=actual,
                   raw_after=after_raw,
                   degrees_before=observation.joint_degrees.tolist(),
                   degrees_after=after.joint_degrees.tolist(),
                   predicted_image_change=plan.predicted_image_change.tolist(),
                   gain_updates=report.gain_updates,
                   offset_updates=report.offset_updates,
                   centre_shift_px=np.round(report.center_shift_px, 2).tolist(),
                   extent_change_px=round(report.side_change_px, 3),
                   extent_after_px=after.side_px,
                   depth_after_mm=controller.depth_of(after) * 1000,
                   travel_mm=controller.travel_mm(after.joint_degrees),
                   gains=controller.gains.describe(),
                   offsets=controller.gains.describe_offsets(),
                   steps=controller.steps,
                   tracker=tracker.describe())
            observation, raw = after, after_raw
            if controller.halted:
                raise RuntimeError(controller.halted)
    except RuntimeError as exc:
        runtime.hold_reason = str(exc)
        record("hold", reason=str(exc), **controller.snapshot())
        print(f"\n{exc}；已停止发送运动，扭矩仍然保持，请托住机械臂（torque remains on）",
              flush=True)
    finally:
        runtime._alignment_unresponsive = set(controller.unresponsive_directions)
        if controller.steps:
            record("summary", tracker=tracker.describe(), **controller.snapshot())
        tracker.close()
    return 0
