"""Bounded ArUco joint experiment. Read-only unless --execute is explicit."""
import argparse
from datetime import datetime
import hashlib
import itertools
import json
from pathlib import Path
import time

import cv2
import numpy as np

from eai_robot.applications.visual_servo.cli import load_intrinsics
from eai_robot.calibration.servo.results import JOINTS, load
from eai_robot.config import ROOT, default_camera_index, load_config
from eai_robot.control.execution.arm import ArmController
from eai_robot.control.execution.feedback import ensure_execution_start_is_safe
from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES, SO101JointMapper
from eai_robot.control.kinematics.kinematics_backend import create_fk_solver
from eai_robot.control.visual_servo.aruco_ibvs import desired_square_corners
from eai_robot.control.visual_servo.aruco_ibvs import point_interaction_matrix
from eai_robot.control.visual_servo.joint_adapter import camera_joint_jacobian, image_joint_step, validate_transform
from eai_robot.hardware.drivers.backends import SerialBackend
from eai_robot.vision.detection.aruco import ArucoMarkerDetector, estimate_marker_pose


def read_mount(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("transform_convention") != "destination_T_source":
        raise ValueError("hand-eye must declare destination_T_source")
    mount = validate_transform(data["gripper_T_camera"])
    # This collaborator format includes both meters and labeled millimeters.
    if not np.allclose(mount[:3, 3] * 1000, data["camera_translation_in_gripper_mm"]):
        raise ValueError("hand-eye translation units disagree")
    return mount


def nearest_square_target(points, size, side):
    """Choose once, preserving marker-corner identity throughout the run."""
    square = desired_square_corners(*size, side)
    candidates = [np.roll(square, shift, axis=0) for shift in range(4)]
    return min(candidates, key=lambda target: np.linalg.norm(points - target))


def bounded_targets(mapper, calibration, measured, proposed_q, initial_q, initial_camera,
                    fk, mount, *, max_joint_excursion_deg=3., max_travel_m=.01):
    """Recheck limits after quantization to real encoder targets."""
    targets = dict(measured)
    for name, value in zip(ARM_JOINT_NAMES, proposed_q, strict=True):
        normalized = mapper.urdf_degrees_to_normalized(name, value, clip=False)
        targets[calibration[name].id] = calibration[name].position((normalized + 100) / 200)
    for name, cal in calibration.items():
        if not cal.range_min <= targets[cal.id] <= cal.range_max:
            raise RuntimeError("quantized target outside calibration")
    quantized_q = np.array([mapper.raw_to_urdf_degrees(n, targets[calibration[n].id], clip=False)
                            for n in ARM_JOINT_NAMES])
    measured_q = np.array([mapper.raw_to_urdf_degrees(n, measured[calibration[n].id], clip=False)
                           for n in ARM_JOINT_NAMES])
    ensure_execution_start_is_safe(quantized_q)
    if np.max(np.abs(quantized_q - measured_q)) > .75:
        raise RuntimeError("quantization exceeds 0.75 degree joint step")
    if np.max(np.abs(quantized_q - initial_q)) > max_joint_excursion_deg:
        raise RuntimeError("joint excursion budget reached")
    camera = fk.forward_kinematics(quantized_q) @ mount
    if np.linalg.norm(camera[:3, 3] - initial_camera[:3, 3]) > max_travel_m:
        raise RuntimeError("camera travel budget reached")
    current = fk.forward_kinematics(measured_q) @ mount
    if np.linalg.norm(camera[:3, 3] - current[:3, 3]) > .002:
        raise RuntimeError("quantization exceeds 2 mm camera step")
    return targets, quantized_q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=load_config()["serial"]["port"])
    parser.add_argument("--camera-index", type=int, default=default_camera_index())
    parser.add_argument("--intrinsics", type=Path, required=True)
    parser.add_argument("--hand-eye", type=Path, required=True)
    parser.add_argument("--hand-eye-gripper-frame", required=True,
                        help="Operator-confirmed frame used by the hand-eye calibration")
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--marker-length-m", type=float, required=True)
    parser.add_argument("--target-side-px", type=float, default=60.)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--preview", action="store_true", help="Show live features and target corners")
    parser.add_argument("--center-only", action="store_true", help="MVP: center tracking with 20 px tolerance")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.hand_eye_gripper_frame != "gripper_frame_link":
        parser.error("this experiment requires hand-eye calibrated at gripper_frame_link")
    if args.camera_index is None or args.camera_index < 0 or not 1 <= args.steps <= 20:
        parser.error("valid camera index and 1..20 steps required")
    if not np.isfinite(args.marker_length_m) or not 0 < args.marker_length_m <= .3:
        parser.error("marker length must be within (0, 0.3] meters")
    size, K, distortion = load_intrinsics(args.intrinsics)
    desired_square_corners(*size, args.target_side_px)
    mount = read_mount(args.hand_eye)
    calibration = load(ROOT / "calibration/scs215_so101.json")
    mapper = SO101JointMapper(calibration)
    fk, _ = create_fk_solver(ROOT / "src/eai_robot/control/kinematics/so101_new_calib.urdf")
    detector = ArucoMarkerDetector(dictionary=args.dictionary, marker_id=args.marker_id)
    output = args.output or ROOT / "local_artifacts/experiments" / (
        datetime.now().strftime("%Y-%m-%d_%H%M%S_%f") + "_aruco-joint-experiment")
    output.mkdir(parents=True, exist_ok=False)
    backend = camera = controller = None
    target = None
    log = (output / "events.jsonl").open("w", encoding="utf-8")

    def record(kind, **data):
        item = {"event": kind, "monotonic": time.monotonic(), **data}
        value = json.dumps(item, default=lambda x: x.tolist(), allow_nan=False)
        log.write(value + "\n")
        log.flush()
        print(value, flush=True)

    try:
        record("setup", execute=args.execute, camera_index=args.camera_index,
               marker_id=args.marker_id, marker_length_m=args.marker_length_m,
               hand_eye_gripper_frame=args.hand_eye_gripper_frame,
               intrinsics_sha256=hashlib.sha256(args.intrinsics.read_bytes()).hexdigest(),
               hand_eye_sha256=hashlib.sha256(args.hand_eye.read_bytes()).hexdigest())
        backend = SerialBackend(args.port, 1_000_000)
        # Same reviewed wrist interval used by the desktop/demo path. This does
        # not authorize full-range travel: this experiment also enforces a
        # 3-degree excursion around the measured interior start pose.
        wrist = calibration["wrist_roll"]
        reviewed_wrist_range = (wrist.range_min, wrist.range_max) == (47, 974)
        controller = ArmController(backend, calibration, max_step=1, interval=.03,
                                   tolerance=1, timeout=.5, motion="smooth",
                                   allow_wide_range=reviewed_wrist_range)
        measured = controller.inspect()

        def q_from_raw(raw):
            return np.array([mapper.raw_to_urdf_degrees(n, raw[calibration[n].id], clip=False)
                             for n in ARM_JOINT_NAMES])

        def check_devices(enabled):
            for sid in range(1, 7):
                if backend.read(sid, "Status") != 0:
                    raise RuntimeError(f"servo {sid} status alarm")
                if backend.read(sid, "Torque_Enable") != int(enabled):
                    raise RuntimeError(f"servo {sid} unexpected torque state")

        ensure_execution_start_is_safe(q_from_raw(measured))
        check_devices(False)
        camera = cv2.VideoCapture(args.camera_index)
        if not camera.isOpened():
            raise RuntimeError("camera did not open")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])

        def observe(filename):
            # Flush the finite backlog between completed actions. No motion is
            # sent during acquisition. Backend buffering age is not hardware stamped.
            for _ in range(4):
                if not camera.grab():
                    raise RuntimeError("camera stopped returning frames")
            stamp = time.monotonic()
            ok, frame = camera.read()
            if not ok or frame is None or (frame.shape[1], frame.shape[0]) != size:
                raise RuntimeError("invalid camera frame or resolution")
            marker = detector.detect(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            cv2.imwrite(str(output / filename), frame)
            if marker is None:
                raise RuntimeError("target lost; no further motion")
            pose = estimate_marker_pose(marker, K, distortion, args.marker_length_m)
            if np.min(pose.corner_depths_m) < .10:
                raise RuntimeError("target is closer than the 100 mm approach guard")
            points = cv2.undistortPoints(marker.corners_px.reshape(-1, 1, 2),
                                         K, distortion, P=K).reshape(4, 2)
            if np.any(points < 0) or np.any(points >= size):
                raise RuntimeError("target corners outside valid image")
            if args.preview:
                cv2.polylines(frame, [np.rint(marker.corners_px).astype(np.int32)],
                              True, (0, 220, 0), 2)
                message = "ID 0 - PREFLIGHT"
                if target is not None:
                    normalized = np.c_[(target[:, 0] - K[0, 2]) / K[0, 0],
                                       (target[:, 1] - K[1, 2]) / K[1, 1], np.ones(4)]
                    overlay, _ = cv2.projectPoints(normalized, np.zeros(3), np.zeros(3), K, distortion)
                    for pixel in overlay.reshape(4, 2):
                        cv2.drawMarker(frame, tuple(np.rint(pixel).astype(int)),
                                       (0, 200, 255), cv2.MARKER_CROSS, 12, 2)
                    rms = float(np.sqrt(np.mean((points - target)**2)))
                    message = f"ID {args.marker_id}  error {rms:.1f}px  Q/Esc STOP"
                cv2.putText(frame, message, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                            .6, (0, 220, 255), 2)
                cv2.imshow("EAI ArUco LIVE SERVO", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    raise KeyboardInterrupt("preview stop requested")
            if time.monotonic() - stamp > .25:
                raise RuntimeError("observation processing exceeded 250 ms")
            return points, pose.corner_depths_m, stamp

        points, depths, stamp = observe("initial.jpg")
        target = nearest_square_target(points, size, args.target_side_px)
        record("preflight", measured=measured, corners_px=points, depths_m=depths,
               target_corners_px=target)
        if args.execute:
            print("Max 0.4 deg solver step; quantized <=0.75 deg/2 mm; "
                  "total <=3 deg/10 mm from start; <=20 seconds. Ctrl+C stops and releases torque.", flush=True)
            if input('Support arm, clear workspace. Type START ARUCO to enable: ').strip() != "START ARUCO":
                record("cancelled")
                return 0
            # Re-read after human interaction; no stale pose or image is executed.
            measured = controller.inspect()
            ensure_execution_start_is_safe(q_from_raw(measured))
            check_devices(False)
            observe("before_enable.jpg")
            controller.enable()
            record("torque_enabled")
            time.sleep(.3)
        initial_q = q_from_raw(controller.positions())
        initial_camera = fk.forward_kinematics(initial_q) @ mount
        started = time.monotonic()
        previous_error = None
        for index in range(args.steps):
            points, depths, stamp = observe(f"step_{index:03d}.jpg")
            if args.center_only:
                target = points - points.mean(axis=0) + np.array(size) / 2
            measured = controller.positions()
            q = q_from_raw(measured)
            ensure_execution_start_is_safe(q)
            check_devices(args.execute)
            step = image_joint_step(fk, q, mount, points, depths, K, size,
                                    target_side_px=args.target_side_px, target_corners_px=target)
            record("observation", step=index, error_rms_px=step.error_rms_px,
                   corners_px=points, measured=measured, q_degrees=q)
            if previous_error is not None and step.error_rms_px > previous_error + 5:
                record("stopped", reason="image error increased by more than 5 px")
                break
            if np.max(np.abs(points - target)) <= (20 if args.center_only else 3):
                record("aligned")
                break
            targets, quantized = bounded_targets(mapper, calibration, measured,
                step.target_degrees, initial_q, initial_camera, fk, mount)
            _, camera_jacobian = camera_joint_jacobian(fk, q, mount)
            image_jacobian = point_interaction_matrix(points, depths, K) @ camera_jacobian
            error = (points - target).reshape(-1)
            predicted = error + image_jacobian @ np.deg2rad(quantized - q)
            if args.center_only:
                # A one-count request may sit within actuator deadband. The MVP
                # chooses a physical two-count candidate, never an accumulated
                # unmeasured target, under the same travel and joint budgets.
                targets, quantized, predicted = dict(measured), q.copy(), error.copy()
            # Encoder resolution can erase a small continuous solution. Search
            # one/two-count neighbors under the same physical budgets, choosing
            # only a step that predicts lower image error. No accumulating
            # fractional targets or expanding limits to force motion.
            candidates = []
            for name in ARM_JOINT_NAMES:
                for increment in (-2, -1, 1, 2):
                    candidate = q.copy()
                    candidate[ARM_JOINT_NAMES.index(name)] = mapper.raw_to_urdf_degrees(
                        name, measured[calibration[name].id] + increment, clip=False)
                    candidates.append(candidate)
            if args.center_only:
                for first, second in itertools.combinations(range(5), 2):
                    for di, dj in itertools.product((-2, -1, 0, 1, 2), repeat=2):
                        if max(abs(di), abs(dj)) != 2:
                            continue
                        candidate = q.copy()
                        for axis, increment in ((first, di), (second, dj)):
                            name = ARM_JOINT_NAMES[axis]
                            candidate[axis] = mapper.raw_to_urdf_degrees(
                                name, measured[calibration[name].id] + increment, clip=False)
                        candidates.append(candidate)
            for candidate in candidates:
                try:
                    raw_candidate, q_candidate = bounded_targets(mapper, calibration,
                        measured, candidate, initial_q, initial_camera, fk, mount)
                except (ValueError, RuntimeError):
                    continue
                if args.center_only and max(abs(raw_candidate[s] - measured[s]) for s in measured) < 2:
                    continue
                projected = error + image_jacobian @ np.deg2rad(q_candidate - q)
                if np.linalg.norm(projected) < np.linalg.norm(predicted):
                    targets, quantized, predicted = raw_candidate, q_candidate, projected
            record("proposal", step=index, raw_targets=targets, q_target=quantized,
                   predicted_error_rms_px=float(np.sqrt(np.mean(predicted**2))))
            if np.linalg.norm(predicted) >= np.linalg.norm(error):
                record("stopped", reason="quantized step does not predict image improvement")
                break
            if not args.execute:
                break
            if time.monotonic() - stamp > .25 or time.monotonic() - started > 20:
                raise RuntimeError("stale proposal or experiment time budget reached")

            def guard():
                nonlocal stamp
                if time.monotonic() - started > 20:
                    raise RuntimeError("motion lease expired")
                # Keep checking actual new images while an atomic bounded
                # waypoint settles; do not relabel or extend an old frame.
                if time.monotonic() - stamp > .12:
                    _, _, stamp = observe("during_motion.jpg")
                if time.monotonic() - stamp > .25:
                    raise RuntimeError("motion lease expired")
                check_devices(True)

            controller.motion_guard = guard
            ratios = [calibration[n].ratio(targets[calibration[n].id]) for n in JOINTS]
            goals, reached = controller.move(ratios)
            record("motion_complete", step=index, goals=goals, feedback=reached,
                   tolerance_counts=controller.tolerance,
                   exact_target_feedback=all(goals[sid] == reached[sid] for sid in goals))
            previous_error = step.error_rms_px
        if args.execute:
            points, _, _ = observe("final.jpg")
            record("final_error", error_rms_px=float(np.sqrt(np.mean((points - target)**2))),
                   measured=controller.positions())
            input("Motion ended. Support the arm, then press ENTER to release torque: ")
        record("finished", execute=args.execute)
        return 0
    except BaseException as exc:
        record("failed", error=type(exc).__name__, message=str(exc))
        raise
    finally:
        if controller is not None and controller.torque_touched:
            failures = controller.disable()
            record("torque_release", failures=failures)
            if failures:
                print("Torque release not confirmed: support arm and disconnect servo power.", flush=True)
        if backend is not None:
            backend.close()
        if camera is not None:
            camera.release()
        if args.preview:
            cv2.destroyAllWindows()
        log.close()


if __name__ == "__main__":
    raise SystemExit(main())
