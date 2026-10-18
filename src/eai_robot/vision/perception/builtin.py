"""Zero-dependency reference detector, so the pipeline is testable today.

`dark-blob` is not a foundation model and does not pretend to be. It wraps the
threshold-and-contour detector this repository already ships for the black ball,
which means the whole "any object" path -- asynchronous worker, detection to
geometry conversion, fused tracking, fixed-rule approach -- can be exercised and
verified before a single model dependency is installed.

That ordering matters. When the first real model is added, any regression can be
attributed to the model rather than to the plumbing, because the plumbing was
already proven with a detector whose behaviour is understood.
"""
from __future__ import annotations

import numpy as np

from eai_robot.vision.detection.ball import BlackBallDetector
from eai_robot.vision.perception.detector import Detection


class DarkBlobDetector:
    """Largest sufficiently compact dark region; its mask becomes the extent."""

    name = "dark-blob"
    latency_budget_s = 0.03

    def __init__(self, *, black_threshold: int = 75, min_area_ratio: float = 0.002,
                 max_area_ratio: float = 0.25, min_circularity: float = 0.55,
                 label: str = "dark-blob"):
        self._inner = BlackBallDetector(
            black_threshold=black_threshold, min_area_ratio=min_area_ratio,
            max_area_ratio=max_area_ratio, min_circularity=min_circularity)
        self._label = label
        self._closed = False

    def detect(self, frame_rgb: np.ndarray) -> list[Detection]:
        if self._closed:
            raise RuntimeError("detector is closed")
        found, mask = self._inner.detect(frame_rgb)
        if found is None:
            return []
        radius = float(found.radius)
        return [Detection(
            box_px=(found.center_x - radius, found.center_y - radius,
                    2.0 * radius, 2.0 * radius),
            score=float(found.confidence),
            label=self._label,
            mask=mask > 0 if mask is not None else None,
        )]

    def describe(self) -> dict[str, object]:
        return {"name": self.name, "device": "cpu", "weights": None,
                "label": self._label,
                "threshold": float(self._inner.black_threshold),
                "latency_budget_s": self.latency_budget_s}

    def close(self) -> None:
        self._closed = True
