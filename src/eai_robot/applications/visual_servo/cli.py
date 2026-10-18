"""ArUco servo rehearsal from a still image or camera, without robot imports."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from eai_robot.config import default_camera_index
from eai_robot.control.visual_servo.controller import ibvs_step, simple_step
from eai_robot.vision.detection.aruco import ArucoMarkerDetector, estimate_marker_pose
from eai_robot.vision.features import VisualFeatures
from eai_robot.vision.tracking.optical_flow import ObjectFlowTracker


def load_intrinsics(path):
    """Read rehearsal or collaborator pinhole results without changing the file."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("intrinsics must be a JSON object")

    def field(primary, alternate):
        if primary in data and alternate in data and data[primary] != data[alternate]:
            raise ValueError(f"conflicting intrinsics fields: {primary} and {alternate}")
        if primary in data:
            return data[primary]
        if alternate in data:
            return data[alternate]
        raise ValueError(f"missing intrinsics field: {primary} or {alternate}")

    if field("distortion_model", "model") != "opencv_pinhole":
        raise ValueError("distortion_model must be opencv_pinhole")
    if "schema_version" in data and (type(data["schema_version"]) is not int
                                     or data["schema_version"] != 1):
        raise ValueError("unsupported intrinsics schema_version (expected 1)")
    try:
        size_data = data["image_size"]
        if isinstance(size_data, dict):
            size = (size_data["width"], size_data["height"])
        elif isinstance(size_data, list):
            size = tuple(size_data)
        else:
            raise ValueError("image_size must be [width, height] or a width/height object")
        matrix = np.asarray(data["camera_matrix"], dtype=float)
        distortion = np.asarray(field("distortion", "distortion_coefficients"), dtype=float)
    except (KeyError, TypeError) as exc:
        raise ValueError("missing or invalid intrinsics fields") from exc
    if (len(size) != 2 or any(type(v) is not int or v <= 0 for v in size)
            or matrix.shape != (3, 3) or not np.all(np.isfinite(matrix))
            or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or matrix[0, 1] != 0 or matrix[1, 0] != 0
            or distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14)
            or not np.all(np.isfinite(distortion))):
        raise ValueError("invalid pinhole intrinsics, distortion, or image_size")
    return size, matrix, distortion


def propose(frame_rgb, detector, args, calibration, captured_at):
    height, width = frame_rgb.shape[:2]
    if calibration is not None and calibration[0] != (width, height):
        raise ValueError("frame resolution differs from intrinsics image_size")
    marker = detector.detect(frame_rgb)
    features = None
    depths = None
    if marker is not None:
        points = marker.corners_px
        if args.mode == "ibvs":
            _, matrix, distortion = calibration
            pose = estimate_marker_pose(marker, matrix, distortion, args.marker_length_m)
            depths = pose.corner_depths_m
            points = cv2.undistortPoints(
                points.reshape(-1, 1, 2), matrix, distortion, P=matrix,
            ).reshape(4, 2)
        # Undistortion can move edge points outside the image. Reject that
        # observation rather than command from an incompletely visible target.
        if np.any(points < 0) or np.any(points >= [width, height]):
            return {"status": "invalid_features", "mode": args.mode}, marker
        features = VisualFeatures(points, (width, height), captured_at,
                                  f"aruco:{marker.marker_id}")
    kwargs = dict(now=time.monotonic(), expected_target=f"aruco:{args.marker_id}")
    if args.mode == "simple":
        proposal = simple_step(features, step_m=args.step_mm / 1000, **kwargs)
    else:
        proposal = ibvs_step(features, camera_matrix=calibration[1], depths_m=depths,
                             target_side_px=args.target_side_px, **kwargs)
    return {
        "status": proposal.status, "mode": args.mode,
        "camera_delta_m": (None if proposal.camera_delta_m is None
                           else proposal.camera_delta_m.tolist()),
        "camera_twist_m_s_rad_s": (None if proposal.camera_twist is None
                                    else proposal.camera_twist.tolist()),
        "error_rms_px": proposal.error_rms_px,
    }, marker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("simple", "ibvs"), default="simple")
    parser.add_argument("--target", choices=("aruco", "object"), default="aruco")
    parser.add_argument("--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                        help="Initial object region; select on first frame if omitted")
    parser.add_argument("--image", type=Path, help="Process one local image and exit")
    parser.add_argument("--camera-index", type=int, default=default_camera_index())
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--intrinsics", type=Path)
    parser.add_argument("--marker-length-m", type=float)
    parser.add_argument("--target-side-px", type=float, default=120)
    parser.add_argument("--step-mm", type=float, default=10)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    if args.target == "aruco" and args.marker_id is None:
        parser.error("ArUco requires an explicit --marker-id")
    if args.target == "object" and args.mode != "simple":
        parser.error("object tracking currently supports simple mode only (no metric depth)")
    if args.target == "object" and args.roi is None and not args.preview:
        parser.error("object tracking requires --roi X Y W H or --preview to select a region")
    if not np.isfinite(args.step_mm) or not 0 < args.step_mm <= 10:
        parser.error("--step-mm must be within (0, 10]")
    if not np.isfinite(args.target_side_px) or args.target_side_px <= 0:
        parser.error("--target-side-px must be positive")
    if args.mode == "ibvs" and (
        args.intrinsics is None or args.marker_length_m is None
        or not np.isfinite(args.marker_length_m) or args.marker_length_m <= 0
    ):
        parser.error("ibvs requires --intrinsics and positive --marker-length-m")
    if args.image is None and (args.camera_index is None or args.camera_index < 0):
        parser.error("set --image or a local --camera-index")
    calibration = load_intrinsics(args.intrinsics) if args.intrinsics else None
    detector = (ArucoMarkerDetector(dictionary=args.dictionary, marker_id=args.marker_id)
                if args.target == "aruco" else None)
    tracker = ObjectFlowTracker() if args.target == "object" else None
    initialized = False
    camera = None
    try:
        if args.image is None:
            camera = cv2.VideoCapture(args.camera_index)
            if not camera.isOpened():
                raise RuntimeError("could not open camera")
            if calibration:
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, calibration[0][0])
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, calibration[0][1])
        while True:
            # Timestamp before acquisition: read/detection latency counts toward
            # freshness. Backends with buffered frames need hardware timestamps
            # before this adapter may be used for real execution.
            captured_at = time.monotonic()
            if args.image is not None:
                frame = cv2.imread(str(args.image))
            else:
                ok, frame = camera.read()
                if not ok:
                    raise RuntimeError("camera stopped returning frames")
            if frame is None:
                raise RuntimeError("could not read image")
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            marker = None
            tracked = None
            if tracker is None:
                result, marker = propose(rgb, detector, args, calibration, captured_at)
            else:
                if not initialized:
                    roi = args.roi
                    if roi is None:
                        roi = cv2.selectROI("Select object, Enter to confirm", frame,
                                            fromCenter=False, showCrosshair=True)
                        cv2.destroyWindow("Select object, Enter to confirm")
                        if roi[2] == 0 or roi[3] == 0:
                            return 0
                        # Selection may take seconds. Reacquire the selected
                        # region from a fresh frame rather than relabel old
                        # image data with a new timestamp.
                        if camera is not None:
                            captured_at = time.monotonic()
                            ok, frame = camera.read()
                            if not ok or frame is None:
                                raise RuntimeError("camera stopped returning frames")
                            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    tracked = tracker.initialize(rgb, roi, captured_at)
                    initialized = True
                else:
                    tracked = tracker.update(rgb, captured_at)
                proposal = simple_step(tracked, now=time.monotonic(),
                                       step_m=args.step_mm / 1000,
                                       expected_target="object:selection")
                result = dict(status=proposal.status, mode="simple", target="object",
                              camera_delta_m=(None if proposal.camera_delta_m is None
                                              else proposal.camera_delta_m.tolist()))
            print(json.dumps(result, allow_nan=False), flush=True)
            if args.preview:
                if marker is not None:
                    cv2.polylines(frame, [np.rint(marker.corners_px).astype(np.int32)],
                                  True, (0, 220, 0), 2)
                if tracked is not None:
                    center = tuple(np.rint(tracked.center_px).astype(int))
                    cv2.drawMarker(frame, center, (0, 220, 0), cv2.MARKER_CROSS, 20, 2)
                cv2.putText(frame, "REHEARSAL: " + result["status"], (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 220), 2)
                cv2.imshow("Visual servo rehearsal", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            if args.image is not None:
                break
    finally:
        if camera is not None:
            camera.release()
        if args.preview:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
