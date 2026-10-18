"""Camera-only ArUco preview; this command never connects to the arm."""

from __future__ import annotations

import argparse
from pathlib import Path
import struct
import time

import cv2
import numpy as np

from eai_robot.config import default_camera_index
from eai_robot.vision.detection.aruco import ArucoMarkerDetector


WINDOW_NAME = "EAI ArUco preview - Q/Esc to close"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only camera preview for ArUco marker detection."
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=default_camera_index(),
        help="Local camera index; set in hardware.local.toml or pass explicitly",
    )
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--marker-id", type=int)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--stream-file", type=Path,
                        help="Publish timestamped raw JPEG frames for a separate controller")
    parser.add_argument("--headless", action="store_true", help="Publish frames in background; only the controller shows a window")
    args = parser.parse_args()
    if args.headless and args.stream_file is None:
        parser.error("--headless requires --stream-file")
    if args.camera_index is None or args.camera_index < 0:
        parser.error("Set --camera-index or [camera].index_or_path in hardware.local.toml")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be positive")
    return args


def main() -> int:
    args = parse_args()
    detector = ArucoMarkerDetector(
        dictionary=args.dictionary,
        marker_id=args.marker_id,
    )
    camera = cv2.VideoCapture(args.camera_index)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Could not open camera index {args.camera_index}")
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print("Background camera publisher; no preview window or robot connection." if args.headless else
          "Camera-only preview. No robot connection or motion. Press Q or Esc to close.")
    if args.stream_file is not None:
        args.stream_file.parent.mkdir(parents=True, exist_ok=True)
    try:
        while True:
            captured_at = time.monotonic()
            ok, frame_bgr = camera.read()
            if not ok or frame_bgr is None:
                raise RuntimeError("Camera stopped returning frames")
            if args.stream_file is not None:
                encoded, jpeg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not encoded:
                    raise RuntimeError("could not publish preview frame")
                temporary = args.stream_file.with_suffix(".tmp")
                temporary.write_bytes(struct.pack("!d", captured_at) + jpeg.tobytes())
                temporary.replace(args.stream_file)
            if args.headless:
                continue
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            observation = detector.detect(frame_rgb)
            if observation is not None:
                polygon = np.rint(observation.corners_px).astype(np.int32).reshape(-1, 1, 2)
                cv2.polylines(frame_bgr, [polygon], True, (0, 220, 0), 2)
                center = tuple(np.rint(observation.center_px).astype(int))
                cv2.drawMarker(frame_bgr, center, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(
                    frame_bgr,
                    f"ArUco ID {observation.marker_id}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 220, 0),
                    2,
                )
            else:
                cv2.putText(
                    frame_bgr,
                    "Configured marker not visible",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                )
            cv2.imshow(WINDOW_NAME, frame_bgr)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
