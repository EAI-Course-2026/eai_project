"""OpenCV detector for the dark circular target used in Week 4."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class BallDetection:
    center_x: float
    center_y: float
    radius: float
    area: float
    circularity: float
    confidence: float

    @property
    def center(self) -> np.ndarray:
        return np.asarray([self.center_x, self.center_y], dtype=float)


class BlackBallDetector:
    """Detect the largest sufficiently circular dark region in an RGB image."""

    def __init__(
        self,
        black_threshold: int = 75,
        min_area_ratio: float = 0.002,
        # A 150 px radius target occupies about 23% of a 640x480 frame.  A 25%
        # ceiling still rejects the large monitor/window boundary observed in
        # the real wrist-camera view while allowing the configured stop size.
        max_area_ratio: float = 0.25,
        min_circularity: float = 0.55,
        min_fill_ratio: float = 0.58,
        morphology_kernel: int = 5,
    ) -> None:
        if not 0 <= black_threshold <= 255:
            raise ValueError("black_threshold must be within 0..255")
        if not 0 < min_area_ratio < max_area_ratio < 1:
            raise ValueError("area ratios must satisfy 0 < min < max < 1")
        self.black_threshold = int(black_threshold)
        self.min_area_ratio = float(min_area_ratio)
        self.max_area_ratio = float(max_area_ratio)
        self.min_circularity = float(min_circularity)
        self.min_fill_ratio = float(min_fill_ratio)
        kernel_size = max(3, int(morphology_kernel) | 1)
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )

    def detect(self, frame_rgb: np.ndarray) -> tuple[BallDetection | None, np.ndarray]:
        if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
            raise ValueError("frame must have shape (height, width, 3)")

        gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        mask = cv2.inRange(gray, 0, self.black_threshold)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        height, width = gray.shape
        frame_area = float(height * width)
        # The target is displayed on a white monitor.  From some wrist-camera
        # poses the monitor is completely surrounded by dark pixels, which
        # makes the ball a nested contour (dark background -> white screen ->
        # dark ball).  RETR_EXTERNAL silently drops that valid ball, so inspect
        # every contour and let the geometry filters below reject backgrounds.
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[tuple[float, BallDetection]] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_ratio = area / frame_area
            if not self.min_area_ratio <= area_ratio <= self.max_area_ratio:
                continue

            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 1.0:
                continue
            circularity = float(4.0 * np.pi * area / (perimeter * perimeter))
            (circle_x, circle_y), radius = cv2.minEnclosingCircle(contour)
            enclosing_area = float(np.pi * radius * radius)
            fill_ratio = area / enclosing_area if enclosing_area > 0 else 0.0
            _, _, box_width, box_height = cv2.boundingRect(contour)
            aspect_ratio = min(box_width, box_height) / max(box_width, box_height)

            if (
                circularity < self.min_circularity
                or fill_ratio < self.min_fill_ratio
                or aspect_ratio < 0.70
            ):
                continue

            moments = cv2.moments(contour)
            if moments["m00"]:
                circle_x = moments["m10"] / moments["m00"]
                circle_y = moments["m01"] / moments["m00"]

            confidence = float(
                np.clip(
                    0.45 * circularity + 0.35 * fill_ratio + 0.20 * aspect_ratio,
                    0.0,
                    1.0,
                )
            )
            detection = BallDetection(
                center_x=float(circle_x),
                center_y=float(circle_y),
                radius=float(radius),
                area=area,
                circularity=circularity,
                confidence=confidence,
            )
            candidates.append((area * confidence * confidence, detection))

        if not candidates:
            return None, mask
        return max(candidates, key=lambda item: item[0])[1], mask


def annotate_frame(
    frame_rgb: np.ndarray,
    detection: BallDetection | None,
    status: str,
) -> np.ndarray:
    """Return a BGR frame suitable for ``cv2.imshow``."""
    canvas = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    height, width = canvas.shape[:2]
    image_center = (width // 2, height // 2)
    cv2.drawMarker(canvas, image_center, (255, 180, 0), cv2.MARKER_CROSS, 28, 2)

    if detection is not None:
        ball_center = (round(detection.center_x), round(detection.center_y))
        cv2.circle(canvas, ball_center, round(detection.radius), (0, 255, 0), 2)
        cv2.line(canvas, image_center, ball_center, (0, 200, 255), 2)
        label = (
            f"ball=({ball_center[0]},{ball_center[1]}) "
            f"r={detection.radius:.1f} conf={detection.confidence:.2f}"
        )
        color = (0, 255, 0)
    else:
        label = "NO BLACK BALL - MOTION FROZEN"
        color = (0, 0, 255)

    cv2.putText(canvas, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.putText(
        canvas,
        status,
        (12, height - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 0),
        2,
    )
    return canvas
