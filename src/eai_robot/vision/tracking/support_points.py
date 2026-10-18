"""Track an arbitrary object as four support points with one apparent extent.

This is the class-agnostic tracker: it needs a region the operator selected and
nothing else. It does not know what the object is, and it never re-detects it.
It locks onto texture inside the region, then carries four support points with a
single similarity transform per frame.

Two deliberate choices make it usable by the fixed rule:

**One similarity transform, not per-point motion.** A 4-DOF transform (translate,
rotate, uniform scale) moves all four support points rigidly, so the points and
the apparent extent can never disagree about how far the object moved. Estimating
each point independently, or refitting a bounding box every frame, produces an
extent whose frame-to-frame noise is the same order as the signal: a one
centimetre step at 300 mm changes the apparent size by about 5 %, which is two or
three pixels on a 70 px object, and a per-frame box jitter of three pixels would
erase it.

**Extent comes from the support points, never from a fresh box.** ``extent_px``
is measured on the same four points the task matrix uses, so "looks bigger" and
"the points spread out" are one fact rather than two.

The tracker reports only ``tracking`` or ``lost``. It has no model to fall behind
on, so it has nothing to coast through; the fused tracker owns ``coasting``
because only it knows that a slow detector is still thinking.
"""
from __future__ import annotations

import cv2
import numpy as np

from eai_robot.vision.geometry.targets import TargetGeometry, mean_edge_px, quad_from_points
from eai_robot.vision.tracking.base import (
    IDLE, LOST, TRACKING, TrackerReading, TrackerSpec, TrackerStatus,
)


class SupportPointTracker:
    """Region-seeded texture tracking: four points, one scale, no semantics."""

    def __init__(
        self,
        *,
        max_points: int = 120,
        min_points: int = 12,
        max_gap_s: float = 0.25,
        forward_backward_px: float = 1.0,
        min_inliers: int = 8,
        min_inlier_fraction: float = 0.5,
        max_scale_step: float = 1.25,
        feature_quality: float = 0.02,
        feature_spacing_px: float = 5.0,
    ):
        if max_points < 8 or min_points < 4 or min_points > max_points:
            raise ValueError("need min_points >= 4 and max_points >= min_points")
        if not np.isfinite(max_gap_s) or max_gap_s <= 0:
            raise ValueError("max_gap_s must be positive")
        if not 0 < min_inlier_fraction <= 1:
            raise ValueError("min_inlier_fraction must be within (0, 1]")
        if not 1.0 < max_scale_step <= 2.0:
            raise ValueError("max_scale_step must be within (1, 2]")
        self.max_points = int(max_points)
        self.min_points = int(min_points)
        self.max_gap_s = float(max_gap_s)
        self.forward_backward_px = float(forward_backward_px)
        self.min_inliers = int(min_inliers)
        self.min_inlier_fraction = float(min_inlier_fraction)
        self.max_scale_step = float(max_scale_step)
        self.feature_quality = float(feature_quality)
        self.feature_spacing_px = float(feature_spacing_px)
        self.target_id = "target"
        self.status = TrackerStatus()
        self._reset_state()

    # -- lifecycle --------------------------------------------------------
    def _reset_state(self) -> None:
        self._gray = None
        self._points = None
        self._anchors = None
        self._extent_px = 0.0
        self._stamp = None
        self._image_size = None
        self._last_scale = 1.0

    def reset(self, reason: str = "") -> None:
        self._reset_state()
        self.status.state = IDLE if reason else LOST
        self.status.reason = reason

    @staticmethod
    def _to_gray(frame_rgb) -> np.ndarray:
        frame = np.asarray(frame_rgb)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("frame must be uint8 RGB with shape (height, width, 3)")
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    # -- acquisition ------------------------------------------------------
    def start(self, frame_rgb, spec: TrackerSpec, captured_at: float) -> TrackerReading:
        spec = spec.validate()
        self.target_id = spec.target_id
        if spec.roi_px is None:
            raise ValueError("the support-point tracker requires a region")
        gray = self._to_gray(frame_rgb)
        height, width = gray.shape
        x, y, box_w, box_h = (float(value) for value in spec.roi_px)
        if x < 0 or y < 0 or box_w <= 0 or box_h <= 0:
            raise ValueError("region must have positive width and height")
        if x + box_w > width or y + box_h > height:
            raise ValueError("region must fit inside the image")
        mask = np.zeros_like(gray)
        mask[int(round(y)):int(round(y + box_h)), int(round(x)):int(round(x + box_w))] = 255

        points = cv2.goodFeaturesToTrack(
            gray, maxCorners=self.max_points, qualityLevel=self.feature_quality,
            minDistance=self.feature_spacing_px, mask=mask, blockSize=5)
        if points is None or len(points) < self.min_points:
            self._reset_state()
            self.status.reset(f"only {0 if points is None else len(points)} features "
                              f"in the selected region")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame_rgb)

        centres = self._cluster_points(points.reshape(-1, 2), 4)
        if centres is None:
            self._reset_state()
            self.status.reset("could not place four support clusters")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame_rgb)

        self._gray = gray
        self._points = points
        self._anchors = quad_from_points(centres)
        self._extent_px = mean_edge_px(self._anchors)
        self._stamp = float(captured_at)
        self._image_size = (width, height)
        self._last_scale = 1.0
        self.status = TrackerStatus(state=TRACKING, frames=1, detections=1,
                                    last_detection_at=float(captured_at))
        return TrackerReading(self._geometry(), self.status, frame_rgb)

    @staticmethod
    def _cluster_points(points: np.ndarray, count: int) -> np.ndarray | None:
        """Four spatially separated seeds, regardless of the object's texture.

        Plain k-means picks up texture density rather than extent and can put
        three centres on one label of a box. Seeding with the extremes of the
        point cloud and refining once keeps the four supports spread over the
        silhouette, which is what makes their mean edge a distance proxy.
        """
        if len(points) < count * 3:
            return None
        hull = points[np.argsort(points[:, 0] + points[:, 1])]
        seeds = np.array([hull[0], hull[-1],
                          hull[np.argmin(hull[:, 0] - hull[:, 1])],
                          hull[np.argmax(hull[:, 0] - hull[:, 1])]], dtype=np.float32)
        if len(np.unique(seeds, axis=0)) < count:
            return None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5)
        _, labels, centres = cv2.kmeans(
            points.astype(np.float32), count, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
        if centres is None or len(centres) < count:
            return None
        return np.asarray(centres, dtype=float)

    # -- tracking ---------------------------------------------------------
    def update(self, frame_rgb, captured_at: float) -> TrackerReading:
        self.status.frames += 1
        if self._gray is None:
            self.status.state = LOST
            self.status.reason = self.status.reason or "not initialized"
            return TrackerReading(None, self.status, frame_rgb)

        gray = self._to_gray(frame_rgb)
        stamp = float(captured_at)
        if (not np.isfinite(stamp) or stamp <= self._stamp
                or stamp - self._stamp > self.max_gap_s
                or gray.shape != self._gray.shape):
            self._reset_state()
            self.status.reset("frame gap or resolution change")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame_rgb)

        forward, status_f, _ = cv2.calcOpticalFlowPyrLK(self._gray, gray, self._points, None)
        if forward is None or status_f is None:
            return self._lose("optical flow failed", frame_rgb)
        backward, status_b, _ = cv2.calcOpticalFlowPyrLK(gray, self._gray, forward, None)
        if backward is None or status_b is None:
            return self._lose("reverse optical flow failed", frame_rgb)

        before = self._points.reshape(-1, 2)
        after = forward.reshape(-1, 2)
        returned = backward.reshape(-1, 2)
        limit = np.array([gray.shape[1], gray.shape[0]], dtype=float)
        valid = ((status_f.reshape(-1) != 0) & (status_b.reshape(-1) != 0)
                 & np.all(np.isfinite(after), axis=1)
                 & np.all(np.isfinite(returned), axis=1)
                 & (np.linalg.norm(returned - before, axis=1) <= self.forward_backward_px)
                 & np.all(after >= 0, axis=1) & np.all(after < limit, axis=1))
        if valid.sum() < max(self.min_inliers, int(self.min_inlier_fraction * len(before))):
            return self._lose(f"only {int(valid.sum())}/{len(before)} points survived "
                              "forward-backward check", frame_rgb)

        transform, inliers = cv2.estimateAffinePartial2D(
            before[valid], after[valid], method=cv2.RANSAC, ransacReprojThreshold=2.0)
        if transform is None or inliers is None or not np.all(np.isfinite(transform)):
            return self._lose("similarity fit failed", frame_rgb)
        keep = inliers.reshape(-1).astype(bool)
        if keep.sum() < self.min_inliers or keep.mean() < self.min_inlier_fraction:
            return self._lose(f"inlier fraction {keep.mean():.2f}", frame_rgb)

        scale = float(np.hypot(transform[0, 0], transform[1, 0]))
        if not np.isfinite(scale) or scale <= 0:
            return self._lose("degenerate scale", frame_rgb)
        if not 1.0 / self.max_scale_step <= scale <= self.max_scale_step:
            # A single frame cannot legitimately change the apparent size by
            # more than this; when it does the flow has slid onto background.
            return self._lose(f"implausible scale change {scale:.2f}", frame_rgb)

        anchors = self._apply(transform, self._anchors)
        limit_row = limit.reshape(1, 2)
        if np.any(anchors < 0) or np.any(anchors > limit_row):
            return self._lose("support points left the frame", frame_rgb)
        extent = mean_edge_px(quad_from_points(anchors))
        if not np.isfinite(extent) or extent <= 0:
            return self._lose("degenerate extent", frame_rgb)

        self._gray = gray
        self._points = after[valid][keep].reshape(-1, 1, 2)
        self._anchors = np.asarray(anchors, dtype=float)
        self._extent_px = float(extent)
        self._stamp = stamp
        self._last_scale = scale
        self.status.state = TRACKING
        self.status.reason = ""
        self.status.coasting_frames = 0
        return TrackerReading(self._geometry(), self.status, frame_rgb,
                              extras={"scale": scale, "inliers": int(keep.sum())})

    @staticmethod
    def _apply(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
        homogeneous = np.c_[points, np.ones(len(points))]
        return (homogeneous @ transform.T).reshape(-1, 2)

    def _lose(self, reason: str, frame_rgb) -> TrackerReading:
        self._reset_state()
        self.status.reset(reason)
        self.status.state = LOST
        return TrackerReading(None, self.status, frame_rgb)

    def _geometry(self) -> TargetGeometry:
        width, height = self._image_size
        return TargetGeometry.from_support_points(
            self._anchors, self._extent_px, (width, height), self._stamp,
            self.target_id, source="support", confidence=self._confidence(),
            extent_name="side")

    def _confidence(self) -> float:
        """Map tracking quality onto 0..1 without inventing a probability."""
        inliers = self.status.frames and self._points is not None
        fraction = 0.0 if not inliers else min(1.0, len(self._points) / self.min_points)
        scale_penalty = 1.0 - min(1.0, abs(self._last_scale - 1.0) / (self.max_scale_step - 1.0))
        return float(np.clip(0.5 * fraction + 0.5 * scale_penalty, 0.0, 1.0))

    def describe(self) -> dict[str, object]:
        return {
            "kind": "support-point",
            "target_id": self.target_id,
            "extent_px": round(float(self._extent_px), 2),
            "last_scale": round(float(self._last_scale), 4),
            "points": 0 if self._points is None else int(len(self._points)),
            **self.status.describe(),
        }
