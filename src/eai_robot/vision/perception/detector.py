"""What every detector must return, and how a detection becomes a target.

The servo loop does not care whether the box came from a fiducial decode, a hand
landmark network or a language-conditioned open-vocabulary detector. It cares
that the answer can be turned into a centre, an apparent extent and four support
points. This module is that conversion, plus the contract that keeps detectors
interchangeable.

Extent convention by evidence quality, in the order the converter prefers them:

``mask``
    ``sqrt(pixel area)``. The most honest measure: it is the only quantity a
    segmentation actually observes, and it does not move when a long object
    rotates inside its box.
``keypoints``
    Mean edge of the quadrilateral spanned by the landmark hull. Right for a
    hand, whose landmarks define the silhouette even with no mask.
``box``
    ``sqrt(width * height)``. The fallback. A box fitted to a rotating stick
    changes area without changing distance, so a box-only tracker must be used
    with a looser standoff or with the pixel criterion only.

All three use the same square-equivalent semantics, so a target that switches
from box to mask keeps its depth scale. That is why ``from_box`` and
``from_mask`` share the ``sqrt_area`` convention.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence, runtime_checkable

import numpy as np

from eai_robot.vision.geometry.targets import (
    TargetGeometry, convex_hull, mean_edge_px, quad_from_points,
)

SELECTION_POLICIES = ("score", "area", "central")


@dataclass(frozen=True)
class Detection:
    """One candidate from any detector, in pixel coordinates."""

    box_px: tuple[float, float, float, float]
    score: float
    label: str = ""
    mask: np.ndarray | None = None
    keypoints_px: np.ndarray | None = None

    def __post_init__(self) -> None:
        box = np.asarray(self.box_px, dtype=float).reshape(4)
        if not np.all(np.isfinite(box)):
            raise ValueError("detection box must be finite")
        if box[2] <= 0 or box[3] <= 0:
            raise ValueError("detection box must have positive size")
        if not np.isfinite(self.score):
            raise ValueError("detection score must be finite")
        object.__setattr__(self, "box_px", tuple(float(value) for value in box))
        if self.keypoints_px is not None:
            points = np.asarray(self.keypoints_px, dtype=float).reshape(-1, 2)
            if not len(points) or not np.all(np.isfinite(points)):
                raise ValueError("keypoints must be finite (N, 2)")
            object.__setattr__(self, "keypoints_px", points)
        if self.mask is not None:
            mask = np.asarray(self.mask)
            if mask.ndim != 2:
                raise ValueError("mask must be two-dimensional")
            object.__setattr__(self, "mask", mask)

    @property
    def area_px(self) -> float:
        return float(self.box_px[2] * self.box_px[3])

    @property
    def center_px(self) -> np.ndarray:
        x, y, width, height = self.box_px
        return np.array([x + width / 2.0, y + height / 2.0])

    def to_geometry(self, image_size, captured_at: float, target_id: str,
                    *, source: str = "detector") -> TargetGeometry:
        """Convert to the servo representation, preferring the richest evidence."""
        if self.mask is not None:
            return TargetGeometry.from_mask(
                self.mask, captured_at, target_id, source=source,
                confidence=float(np.clip(self.score, 0.0, 1.0)))
        if self.keypoints_px is not None and len(self.keypoints_px) >= 4:
            hull = convex_hull(self.keypoints_px)
            quad = quad_from_points(hull)
            return TargetGeometry.from_support_points(
                quad, mean_edge_px(quad), image_size, captured_at, target_id,
                source=source, confidence=float(np.clip(self.score, 0.0, 1.0)),
                extent_name="side")
        return TargetGeometry.from_box(
            self.box_px, image_size, captured_at, target_id, source=source,
            confidence=float(np.clip(self.score, 0.0, 1.0)))

    def describe(self) -> dict[str, object]:
        return {
            "label": self.label,
            "score": round(float(self.score), 4),
            "box_px": [round(float(value), 1) for value in self.box_px],
            "has_mask": self.mask is not None,
            "has_keypoints": self.keypoints_px is not None,
        }


def select_detection(detections: Sequence[Detection], policy: str = "score",
                     image_size: tuple[int, int] | None = None) -> Detection | None:
    """Pick one candidate deterministically.

    Multi-target scenes are the normal case for a language prompt ("a cup" finds
    three), so the choice must be a stated policy rather than ``max(output)`` on
    whatever order the model happened to emit.
    """
    if policy not in SELECTION_POLICIES:
        raise ValueError(f"selection policy must be one of {SELECTION_POLICIES}")
    candidates = [item for item in detections if item is not None]
    if not candidates:
        return None
    if policy == "score":
        return max(candidates, key=lambda item: (item.score, item.area_px))
    if policy == "area":
        return max(candidates, key=lambda item: (item.area_px, item.score))
    if image_size is None:
        raise ValueError("the central policy needs the image size")
    centre = np.array([image_size[0] / 2.0, image_size[1] / 2.0])
    return min(candidates,
               key=lambda item: float(np.linalg.norm(item.center_px - centre)))


@dataclass(frozen=True)
class DetectionBatch:
    """One detector answer, stamped so staleness is measurable, not guessed."""

    detections: tuple[Detection, ...]
    captured_at: float
    latency_s: float
    frame_id: int

    @property
    def age_s(self) -> float:
        import time
        return max(0.0, time.monotonic() - self.captured_at)

    def describe(self) -> dict[str, object]:
        return {
            "count": len(self.detections),
            "captured_at": round(float(self.captured_at), 6),
            "latency_s": round(float(self.latency_s), 4),
            "frame_id": int(self.frame_id),
        }


@runtime_checkable
class ObjectDetector(Protocol):
    """The only surface a model-backed detector has to implement."""

    name: str
    latency_budget_s: float

    def detect(self, frame_rgb: np.ndarray) -> list[Detection]:
        """Return every candidate in the frame; [] is a valid answer."""
        ...

    def describe(self) -> dict[str, object]:
        """Identity for the record: name, weights, device, input size."""
        ...

    def close(self) -> None:
        """Release the model. Must be safe to call twice."""
        ...
