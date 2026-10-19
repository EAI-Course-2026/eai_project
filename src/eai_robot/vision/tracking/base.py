"""Tracker contract shared by markers, hands and prompt-driven objects.

A tracker is anything that can answer, for one frame, "here is the same target
you saw last time, here is how big it looks, and here is how much I trust that".
Implementations differ wildly -- a fiducial decode is exact and takes a
millisecond, a hand landmark model is a network inference at tens of hertz, a
segmentation model may take a quarter of a second -- so the contract names the
states they must all report instead of pretending they all behave the same.

The states are the whole point of this module:

``tracking``
    The current frame produced a target from the tracker's own measurement.
``coasting``
    The last frame did not, but the tracker still has a recently verified state
    and is extrapolating it. This is what makes a slow detector compatible with
    a fast control loop: the loop keeps closing on flow while the model is still
    thinking, and the model is not allowed to stall the loop.
``lost``
    Nothing usable. The owner must hold and reacquire; it must not command motion
    from a stale pose.

``coasting`` is bounded in time by ``max_coast_s``. A tracker that happily
coasts forever is a tracker that will drive the arm into whatever the drift
points at, so the bound is part of the contract rather than a tunable each
implementation invents.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from eai_robot.vision.geometry.targets import TargetGeometry

TRACKING = "tracking"
COASTING = "coasting"
LOST = "lost"
IDLE = "idle"
STATES = (IDLE, TRACKING, COASTING, LOST)


class TrackerUnavailable(RuntimeError):
    """A required model, weight file or optional dependency is not present."""


@dataclass(frozen=True)
class TrackerSpec:
    """What the operator told the tracker to look for.

    Exactly one selection channel is normally set: a region for a class-agnostic
    tracker, a prompt for a language-conditioned detector, an identifier for a
    fiducial. Keeping them in one object stops each implementation from growing
    its own bespoke ``start`` signature.
    """

    roi_px: tuple[float, float, float, float] | None = None   # x, y, w, h
    prompt: str | None = None                                 # "the red cup"
    marker_id: int | None = None                              # ArUco id
    label: str | None = None                                  # detector class name
    expected_extent_m: float | None = None                    # metric twin of extent
    target_id: str = "target"

    def validate(self) -> "TrackerSpec":
        if not self.target_id:
            raise ValueError("target_id is required")
        channels = [self.roi_px is not None, bool(self.prompt),
                    self.marker_id is not None, bool(self.label)]
        if sum(channels) < 1:
            raise ValueError("specify a region, a prompt, a marker id or a label")
        if self.roi_px is not None:
            values = np.asarray(self.roi_px, dtype=float).reshape(4)
            if not np.all(np.isfinite(values)):
                raise ValueError("region must be finite")
            if values[2] <= 0 or values[3] <= 0:
                raise ValueError("region must have positive width and height")
        if self.expected_extent_m is not None:
            if (not np.isfinite(self.expected_extent_m)
                    or self.expected_extent_m <= 0):
                raise ValueError("expected extent must be positive metres")
        return self


@dataclass
class TrackerStatus:
    """Why the tracker is in the state it reports. Logged, never inferred."""

    state: str = IDLE
    reason: str = ""
    frames: int = 0
    detections: int = 0
    coasting_frames: int = 0
    disagreements: int = 0
    last_detection_at: float | None = None
    last_reason: str = ""

    def reset(self, reason: str = "") -> None:
        self.state, self.reason = IDLE, reason
        self.coasting_frames = 0

    def describe(self) -> dict[str, object]:
        return {
            "state": self.state,
            "reason": self.reason,
            "frames": self.frames,
            "detections": self.detections,
            "coasting_frames": self.coasting_frames,
            "disagreements": self.disagreements,
            "last_detection_at": self.last_detection_at,
        }


@dataclass
class TrackerReading:
    """One tracker answer plus the state it was produced in."""

    geometry: TargetGeometry | None
    status: TrackerStatus
    frame: np.ndarray | None = None
    extras: dict[str, object] = field(default_factory=dict)

    @property
    def usable(self) -> bool:
        return self.geometry is not None and self.status.state in (TRACKING, COASTING)

    @property
    def is_fresh(self) -> bool:
        """True only when this frame itself was measured, not extrapolated."""
        return self.geometry is not None and self.status.state == TRACKING


@runtime_checkable
class TargetTracker(Protocol):
    """The only interface the servo loops are allowed to depend on."""

    target_id: str

    def start(self, frame_rgb: np.ndarray, spec: TrackerSpec,
              captured_at: float) -> TrackerReading:
        """Acquire the target. May fail; failure is a reading, not an exception."""
        ...

    def update(self, frame_rgb: np.ndarray, captured_at: float) -> TrackerReading:
        """Consume one frame and return the current reading."""
        ...

    def reset(self, reason: str = "") -> None:
        """Forget everything; the next update must be preceded by ``start``."""
        ...


def stale(geometry: TargetGeometry | None, now: float,
          max_age_s: float = 0.25) -> str | None:
    """Shared freshness predicate, so every loop rejects staleness identically."""
    if geometry is None:
        return "target_lost"
    if geometry.is_stale(now, max_age_s):
        return "stale_frame"
    return None
