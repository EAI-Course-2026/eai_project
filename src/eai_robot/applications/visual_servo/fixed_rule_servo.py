"""Fixed-rule one-centimetre servo on the real arm.

The rule under test is deliberately mechanical: with the marker centre in the
left / right / up / down region of the frame the camera moves 1 cm that way;
with the centre inside the middle band it moves 1 cm forward; it stops at a
standoff. Round 1 proved the centering half and never attempted the forward
half, so this module owns the forward primitive's execution -- the joint pair
that produces a real translation, the deadband floor below which a request does
nothing, and the measured-response learning that turns a partial reply into a
corrected request.

Everything that *decides* is in ``control/visual_servo`` (``fixed_rule`` and
``camera_step``) and stays offline-testable. This file owns the device, the log
and the stop conditions. The motion guard is shared with the centering loop
through ``BoundedJointMotion`` so that calibration, workspace and joint budgets,
fresh-detection-before-command, torque and status readback, camera-loss and
target-loss holds and the one-second arrival lease cannot drift apart between
the two entry points.

Honest limits, all of them measured or derived rather than assumed:

* The apparent size at which the loop stops is a pixel criterion and needs no
  calibration. Metric depth comes from the pinhole relation and is therefore
  only as good as the intrinsics file; it is reported, not advertised.
* The marker must stay near the optical axis for the pinhole interaction matrix
  to be valid. Radial distortion is under 0.1 % inside the central band by
  computation, and the align phase is what keeps the target there.
* 1 cm forward is about 14 ``shoulder_lift`` counts plus 12 opposite
  ``wrist_flex`` counts at the recorded pose. If the planner cannot build that
  pair it refuses the step instead of pitching the camera and calling the
  resulting 20 px slide a centering error.
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
    raw_outside_calibration,
    read_raw_motors,
)
from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
from eai_robot.control.visual_servo.camera_step import ResponseModel, counts_per_degree
from eai_robot.control.visual_servo.fixed_rule import (
    MEASURED_COUNT_OFFSETS,
    FixedRuleConfig,
    FixedRuleController,
    Observation,
    phase_label,
    region_label,
)
from eai_robot.control.visual_servo.joint_adapter import validate_transform
from eai_robot.vision.detection.aruco import ArucoMarkerDetector
from eai_robot.vision.detection.ball import BallDetection

from eai_robot.applications.visual_servo.hold_preview import overlay_label
from eai_robot.applications.visual_servo.joint_center import (
    BoundedJointMotion,
    MotionResponseTimeout,
    TargetLostError,
    wait_for_target,
)
from eai_robot.applications.visual_servo.shared_camera import PreviewFrameUnavailable

DEFAULT_INTRINSICS = ROOT / "calibration/vision/camera_intrinsics.json"
DEFAULT_HAND_EYE = ROOT / "calibration/vision/hand_eye_result.json"


def load_camera_matrix(path) -> np.ndarray:
    """Read the measured pinhole matrix; refuse anything that is not one."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    matrix = np.asarray(data["camera_matrix"], dtype=float)
    if (matrix.shape != (3, 3) or not np.all(np.isfinite(matrix))
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0.0, 0.0, 1.0])):
        raise ValueError("invalid pinhole camera matrix")
    return matrix


def load_hand_eye(path) -> np.ndarray:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return validate_transform(data["gripper_T_camera"])


def camera_origin(fk, mount, joint_degrees) -> np.ndarray:
    """Camera optical origin in the base frame for one joint configuration."""
    return (fk.forward_kinematics(np.asarray(joint_degrees, dtype=float)) @ mount)[:3, 3]


class MarkerCornersDetector:
    """ArUco detector that keeps the corners the fixed rule needs.

    The centering loop only ever consumed a centre and an equivalent radius, so
    the marker geometry was thrown away. The rule needs all four corners for the
    apparent side and for the interaction matrix, while the preview still needs a
    ``BallDetection``-shaped annotation, so both views are produced here from a
    single detection rather than from two separate reads of the camera.
    """

    def __init__(self, marker_id=0, dictionary="DICT_4X4_50"):
        self.detector = ArucoMarkerDetector(marker_id=marker_id, dictionary=dictionary)
        self.dictionary, self.marker_id = dictionary, marker_id
        self.last_marker_id = None

    def marker(self, frame_rgb):
        observation = self.detector.detect(frame_rgb)
        self.last_marker_id = None if observation is None else observation.marker_id
        return observation

    def detect(self, frame_rgb):
        """Preview and guard adapter; same contract as the centering detector."""
        observation = self.marker(frame_rgb)
        mask = np.zeros(frame_rgb.shape[:2], dtype=np.uint8)
        if observation is None:
            return None, mask
        polygon = observation.corners_px.astype(np.float32)
        area = abs(float(cv2.contourArea(polygon)))
        centre = observation.center_px
        cv2.fillConvexPoly(mask, np.rint(polygon).astype(np.int32), 255)
        return BallDetection(float(centre[0]), float(centre[1]),
                             float(np.sqrt(area / np.pi)), area, 1.0, 1.0), mask


def preview_detection(corners_px) -> BallDetection:
    polygon = np.asarray(corners_px, dtype=np.float32)
    area = abs(float(cv2.contourArea(polygon)))
    centre = np.asarray(corners_px, dtype=float).mean(axis=0)
    return BallDetection(float(centre[0]), float(centre[1]),
                         float(np.sqrt(area / np.pi)), area, 1.0, 1.0)


def sample_observation(runtime, detector, mapper, samples, timeout_s):
    """Median of several fresh frames, paired with the encoder pose read after them.

    The corners are averaged rather than taken from one frame because a single
    corner set on a 25 mm marker carries a pixel or two of detection noise while
    the depth signal of one step is only about 1.5 px. The joint angles are read
    *after* the frames, so the pose is never older than the observation.
    """
    collected = []
    last_frame, last_marker = None, None
    deadline = time.monotonic() + timeout_s
    while len(collected) < samples and time.monotonic() < deadline:
        frame = runtime.read_frame()
        marker = detector.marker(frame)
        last_frame, last_marker = frame, marker
        if marker is None:
            collected.clear()
        else:
            collected.append(marker.corners_px)
    if last_frame is None:
        raise TargetLostError("no frame available while sampling the marker")
    if len(collected) < samples:
        raise TargetLostError(
            f"only {len(collected)}/{samples} marker frames within {timeout_s:.1f}s")
    corners = np.median(np.stack(collected), axis=0)
    raw = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
    degrees = arm_degrees_from_raw(mapper, raw)
    height, width = last_frame.shape[:2]
    observation = Observation(corners, (width, height), degrees)
    return observation, raw, last_frame, last_marker


def initial_reference(runtime, mapper, args):
    """Reuse the preserved handover pose across a retry; never reset the budget."""
    if not hasattr(runtime, "_fixed_rule_reference"):
        initial = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
        if getattr(args, "hold_reference", None):
            initial = json.loads(Path(args.hold_reference).read_text())
            if (set(initial) != set(ARM_JOINT_NAMES)
                    or any(type(value) is not int for value in initial.values())
                    or raw_outside_calibration(mapper, initial)):
                raise RuntimeError("Invalid preserved handover reference")
        runtime._fixed_rule_reference = (initial, arm_degrees_from_raw(mapper, initial))
    return runtime._fixed_rule_reference


def build_config(args, matrix) -> FixedRuleConfig:
    """One config object owns the rule *and* the budgets the guard enforces."""
    return FixedRuleConfig(
        band=args.rule_band,
        step_m=args.rule_step_mm / 1000.0,
        stop_side_px=args.rule_stop_side_px,
        min_standoff_m=args.rule_min_standoff_m,
        approach_gate_px=args.rule_gate_px,
        focal_px=float(matrix[0, 0]),
        marker_length_m=args.marker_length_m,
        max_joint_step_deg=args.max_joint_step_deg,
        min_effective_counts=5.0,
        max_request_counts=float(args.rule_max_request_counts),
        max_travel_mm=args.max_travel_mm,
        max_joint_excursion_deg=args.max_joint_excursion_deg,
    )


def _observe(runtime, detector, mapper, args, stop, preview):
    """Sample, and wait for a reacquired target rather than dropping the lease."""
    samples = max(3, args.feature_samples)
    try:
        return sample_observation(runtime, detector, mapper, samples,
                                  args.detection_timeout_s)
    except (PreviewFrameUnavailable, TimeoutError, TargetLostError):
        wait_for_target(runtime, detector, preview, stop,
                        on_wait=getattr(runtime, '_fixed_rule_diagnostics', None))
        return sample_observation(runtime, detector, mapper, samples,
                                  args.detection_timeout_s)


def _print_summary(controller, observation):
    snapshot = controller.snapshot()
    print(
        f"步数（steps）{snapshot['steps']}，深度进度（depth progress）"
        f"{snapshot['depth_progress_mm']:.1f} mm，边长（side）{observation.side_px:.1f} px，"
        f"响应增益（gains）{snapshot['response_gains']}", flush=True)
    if snapshot["stalls"] or snapshot["disagreements"]:
        print(f"无位移次数（stalls）{snapshot['stalls']}，"
              f"深度不一致次数（disagreements）{snapshot['disagreements']}", flush=True)


def run_fixed_rule(runtime, args, stop, preview) -> int:
    """Run the five-region rule until standoff, a guard, or the operator stops."""
    mapper = runtime._require_mapper()
    fk = runtime.fk
    if fk is None:
        raise RuntimeError("FK solver is not initialized")

    mount = load_hand_eye(args.hand_eye)
    matrix = load_camera_matrix(args.intrinsics)
    config = build_config(args, matrix)
    config.validate()
    rates = counts_per_degree(mapper.calibration)

    detector = MarkerCornersDetector(args.marker_id, args.dictionary)
    initial_raw, initial_q = initial_reference(runtime, mapper, args)
    origin = camera_origin(fk, mount, initial_q)

    path = ROOT / "local_artifacts/experiments" / (
        datetime.now().strftime("%Y%m%d_%H%M%S") + "_fixed-rule.jsonl")
    path.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    runtime.hold_reason = "USER STOP"

    def record(event, **data):
        with path.open("a") as stream:
            stream.write(json.dumps(dict(time=time.monotonic(), event=event, **data)) + "\n")

    controller = FixedRuleController(
        fk, mount, config, rates, matrix,
        reference_degrees=initial_q, origin_m=origin,
        # The priors are the only place the measured offsets enter a real run.
        # Without them the first step of every axis is again a blind request the
        # shaft absorbs, and the loop pays a timeout to discover what the log
        # already knew.
        gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS),
        unresponsive_directions=getattr(runtime, "_alignment_unresponsive", ()))
    one_approach = getattr(args, 'rule_one_approach', False)
    bounds = BoundedJointMotion(
        runtime, detector, mapper, args, initial_q, origin,
        record, preview, stop,
        position_of=lambda q: camera_origin(fk, mount, q),
        joints=ARM_JOINT_NAMES)
    runtime._fixed_rule_diagnostics = lambda: bounds.record_motor_state('waiting_for_target')

    print(
        f"\n固定规律伺服（fixed rule）：每步 {config.step_m * 1000:.0f} mm，"
        f"中央带 {config.band:.0%}，边长 {config.stop_side_px:.0f} px 停靠，"
        f"接近门限 {config.approach_gate_px:.0f} px。fx={matrix[0, 0]:.1f} px。"
        f"预算：行程 {config.max_travel_mm:.0f} mm / 关节偏移 "
        f"{config.max_joint_excursion_deg:.0f} 度 / 单步 {config.max_joint_step_deg:.1f} 度。"
        f"步数上限 {args.rule_max_steps} 步（命令行覆盖配置文件）；日志：{path}", flush=True)
    if one_approach:
        print("单次前向验证：允许先对齐；首次前向尝试后结束，丢目标也不重试。", flush=True)
    record("settings", band=config.band, step_mm=config.step_m * 1000,
           stop_side_px=config.stop_side_px, gate_px=config.approach_gate_px,
           focal_px=matrix[0, 0], marker_length_m=config.marker_length_m,
           max_travel_mm=config.max_travel_mm,
           max_joint_excursion_deg=config.max_joint_excursion_deg,
           max_joint_step_deg=config.max_joint_step_deg,
           max_steps=args.rule_max_steps, max_runtime_s=args.max_runtime_s,
           one_approach=one_approach,
           intrinsics=str(args.intrinsics), hand_eye=str(args.hand_eye),
           initial=initial_raw, reference=initial_q.tolist())

    transient = 0
    try:
        time.sleep(args.torque_settle_s)
        observation, raw, frame, _ = _observe(runtime, detector, mapper, args, stop, preview)
        depth_mm = controller.depth_m(observation) * 1000.0
        print(
            f"标记中心（marker centre）({observation.center_px[0]:.0f}, "
            f"{observation.center_px[1]:.0f}) px，视在边长（apparent side）"
            f"{observation.side_px:.1f} px，名义深度（nominal depth）{depth_mm:.0f} mm。"
            "动手前请用尺子确认标记确实大约在这个距离——深度数字只和标定一样准。", flush=True)
        # This is a reference replay envelope, not a hardware reachability test.
        if depth_mm > config.max_start_depth_m * 1000.0:
            print(
                f"⚠ 起点 {depth_mm:.0f} mm 超过参考仿真起点 "
                f"{config.max_start_depth_m * 1000:.0f} mm。参考条件为固定起始姿态、"
                "300 mm 行程 / 120 度偏移 / 60 步，不保证当前姿态可达。"
                f"本次实际预算：{config.max_travel_mm:.0f} mm / "
                f"{config.max_joint_excursion_deg:.0f} 度 / {args.rule_max_steps} 步；"
                "小步数或短行程试验可能在停靠前正常结束。", flush=True)
        record("observation", centre=observation.center_px.tolist(),
               side_px=observation.side_px, depth_m=depth_mm / 1000.0,
               degrees=observation.joint_degrees.tolist(), raw=raw,
               marker_id=detector.last_marker_id)

        while not stop.stop_requested.is_set():
            if args.max_runtime_s > 0 and time.monotonic() - started >= args.max_runtime_s:
                runtime.hold_reason = "运行时长到达上限（time limit）"
                record("hold", reason=runtime.hold_reason, **controller.snapshot())
                print("\n运行时长到达上限，保持（runtime reached; HOLDING）", flush=True)
                break
            if controller.steps >= args.rule_max_steps:
                runtime.hold_reason = "步数预算已用尽（step budget reached）"
                record("hold", reason=runtime.hold_reason, **controller.snapshot())
                print(f"\n已执行 {args.rule_max_steps}/{args.rule_max_steps} 步，步数预算已用尽（HOLDING）", flush=True)
                break

            try:
                observation.validated()
            except ValueError as exc:
                transient += 1
                record("invalid_observation", reason=str(exc))
                if transient > 3:
                    raise RuntimeError(f"unusable marker geometry: {exc}") from exc
                observation, raw, frame, _ = _observe(
                    runtime, detector, mapper, args, stop, preview)
                continue
            transient = 0

            action = controller.decide(observation)
            status = (
                f"{phase_label(action.phase)} {region_label(action.region)} "
                f"边长={action.side_px:.0f}px 深度={action.depth_m * 1000:.0f}mm "
                f"误差={action.center_error_px:.0f}px 步数={controller.steps}/{args.rule_max_steps}"
            )
            print(f"\r{status:74s}", end="", flush=True)
            if preview is not None:
                label = (f"{action.phase.upper()} {action.region} side={action.side_px:.0f}px "
                         f"d={action.depth_m * 1000:.0f}mm e={action.center_error_px:.0f}px "
                         f"step={controller.steps}/{args.rule_max_steps}")
                if action.reason:
                    label += " | " + overlay_label(action.reason)
                preview.show(frame, preview_detection(observation.corners_px), label)

            if action.kind == "reached":
                runtime.hold_reason = "已到停靠（standoff reached）"
                record("reached", reason=action.reason,
                       centre=observation.center_px.tolist(),
                       side_px=observation.side_px, depth_m=action.depth_m,
                       **controller.snapshot())
                print(f"\n已到停靠（standoff reached）：{action.reason}。保持。", flush=True)
                _print_summary(controller, observation)
                return 0
            if not action.movable:
                runtime.hold_reason = action.reason or "no executable step"
                record("hold", reason=runtime.hold_reason, **controller.snapshot())
                print(f"\n{runtime.hold_reason}；已保持。", flush=True)
                _print_summary(controller, observation)
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
                _print_summary(controller, observation)
                return 0

            record("plan", phase=action.phase, region=action.region,
                   plan=plan.describe(), degrees=observation.joint_degrees.tolist())
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
                observation, raw, frame, _ = _observe(
                    runtime, detector, mapper, args, stop, preview)
                continue
            except (TargetLostError, PreviewFrameUnavailable, TimeoutError) as exc:
                record("transient", reason=str(exc))
                if one_approach and action.phase == 'approach':
                    runtime.hold_reason = '单次前向验证结束：目标或相机丢失（single approach interrupted）'
                    record('hold', reason=runtime.hold_reason, detail=str(exc),
                           **controller.snapshot())
                    print(f"\n{runtime.hold_reason}；保持，不重试。", flush=True)
                    return 0
                transient += 1
                if transient > 2:
                    raise
                observation, raw, frame, _ = _observe(
                    runtime, detector, mapper, args, stop, preview)
                continue

            time.sleep(0.15)
            after, after_raw, frame, _ = _observe(
                runtime, detector, mapper, args, stop, preview)
            # Attribute the measured response to the delta the guard actually
            # commanded: it re-reads the encoder immediately before the write,
            # so its anchor is the one the servo saw.
            report = controller.record(
                plan, observation, after,
                bounds.last_before or raw, actual)
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
                   side_change_px=round(report.side_change_px, 3),
                   side_after_px=after.side_px,
                   depth_after_mm=controller.depth_m(after) * 1000,
                   travel_mm=controller.travel_mm(after.joint_degrees),
                   gains=controller.gains.describe(),
                   offsets=controller.gains.describe_offsets(),
                   steps=controller.steps)
            observation, raw = after, after_raw
            if controller.halted:
                raise RuntimeError(controller.halted)
            if one_approach and action.phase == 'approach':
                runtime.hold_reason = '单次前向验证完成（single approach complete）'
                record('hold', reason=runtime.hold_reason, **controller.snapshot())
                print(f"\n{runtime.hold_reason}；保持。", flush=True)
                return 0
    except RuntimeError as exc:
        runtime.hold_reason = str(exc)
        record("hold", reason=str(exc), **controller.snapshot())
        print(f"\n{exc}；已停止发送运动，扭矩仍然保持，请托住机械臂（torque remains on）",
              flush=True)
    finally:
        runtime._alignment_unresponsive = set(controller.unresponsive_directions)
        if controller.steps:
            record("summary", **controller.snapshot())
    return 0
