"""A slow semantic detector and a fast texture tracker, fused into one target.

Neither tracker is sufficient on its own for an arbitrary object.

The **detector** knows *what* the target is and can find it again after a loss,
but it is slow: a promptable segmenter needs 150-500 ms on this machine, which
is three to eight camera frames. Closing the servo loop on its output alone
means closing on a pose that is a third of a second old.

The **flow tracker** is exact and per-frame, but it has no idea what it is
following. It drifts, it slides onto background when the target rotates out of
texture, and once it is lost it cannot reacquire.

Fusion assigns each the job it is actually good at:

* the flow tracker produces the pose every frame, so the loop always closes on
  the current image;
* the detector arbitrates *periodically*, re-anchoring the flow when the two
  disagree and reacquiring the target when the flow has died.

That is a supervisor, not a filter. Nothing here averages a slow measurement
into a fast one -- an average of a fresh pose and a stale one is simply a stale
pose with a smaller error bar, and at 1 cm per step a stale pose is a collision
waiting to be scheduled.

Three states, three different promises:

``tracking``
    The flow tracker measured this frame. ``is_fresh`` is true and the servo may
    spend a step on it.
``coasting``
    The flow died this frame, but a detection younger than ``max_coast_s`` still
    locates the target. The geometry is real but it was not measured on this
    frame, so it is reported as coasting rather than as tracking. The bound is
    not optional: an unbounded coast drives the arm at wherever the drift points.
``lost``
    Nothing. The owner must hold.

The disagreement test compares *converted* extents. A segmentation reports
``sqrt(area)``, the flow reports a square-equivalent side; the raw numbers are
different quantities, so ``to_side_equivalent`` converts both before the ratio
is taken. When a convention has no shape-free equivalent the extent check is
skipped and the skip is recorded, because silently comparing a span against a
side would manufacture agreement.
"""
from __future__ import annotations

import time

import numpy as np

from eai_robot.vision.perception.async_detector import AsyncDetector
from eai_robot.vision.perception.detector import DetectionBatch, select_detection
from eai_robot.vision.geometry.targets import TargetGeometry, to_side_equivalent
from eai_robot.vision.tracking.base import (
    COASTING, IDLE, LOST, TRACKING, TrackerReading, TrackerSpec, TrackerStatus,
)
from eai_robot.vision.tracking.support_points import SupportPointTracker

#: Detection geometries are built from a mask or box the model produced, never
#: from the flow, so the two can be compared without circularity.
_ACQUIRE_POLL_S = 0.01


def roi_from_geometry(geometry: TargetGeometry, margin: float,
                      image_size: tuple[int, int]) -> tuple[float, float, float, float]:
    """Grow a geometry's support box into a region a feature detector can use.

    A tight box on a low-texture object yields almost no corners, so the region
    is expanded by ``margin`` and clipped to the frame. Clipping rather than
    refusing keeps an object at the frame edge trackable instead of demanding
    the operator re-centre it before every acquisition.
    """
    if not np.isfinite(margin) or margin < 1.0:
        raise ValueError("margin must be at least 1.0 (no shrink)")
    x, y, width, height = (float(value) for value in geometry.bbox)
    centre_x, centre_y = x + width / 2.0, y + height / 2.0
    half_w, half_h = 0.5 * width * margin, 0.5 * height * margin
    # A single support point has a degenerate box; give it a floor so the region
    # still contains texture to lock onto.
    half_w = max(half_w, 6.0)
    half_h = max(half_h, 6.0)
    image_w, image_h = float(image_size[0]), float(image_size[1])
    left = max(0.0, centre_x - half_w)
    top = max(0.0, centre_y - half_h)
    right = min(image_w, centre_x + half_w)
    bottom = min(image_h, centre_y + half_h)
    if right - left < 4.0 or bottom - top < 4.0:
        raise ValueError("target box is too small or entirely outside the frame")
    return (left, top, right - left, bottom - top)


class FusedTracker:
    """Detector-supervised flow tracking, with a bounded coasting window."""

    def __init__(
        self,
        support: SupportPointTracker,
        detector: AsyncDetector | None = None,
        *,
        selection: str = "score",
        max_coast_s: float = 0.2,
        agree_centre_px: float = 0.35,
        agree_extent_ratio: float = 0.35,
        reanchor_centre_px: float = 6.0,
        acquire_timeout_s: float = 2.0,
        seed_margin: float = 1.25,
        max_disagreements: int = 3,
        flow: bool = True,
        clock=time.monotonic,
    ):
        if not np.isfinite(max_coast_s) or not 0 < max_coast_s <= 1.0:
            raise ValueError("max_coast_s must be within (0, 1] seconds")
        if not 0 < agree_centre_px <= 2.0:
            # Expressed as a fraction of the target's extent, not pixels, because
            # the same physical error is 20 px on a close object and 3 px on a
            # distant one.
            raise ValueError("agree_centre_px must be a fraction within (0, 2]")
        if not 0 < agree_extent_ratio <= 1.0:
            raise ValueError("agree_extent_ratio must be within (0, 1]")
        if not np.isfinite(reanchor_centre_px) or reanchor_centre_px < 0:
            raise ValueError("reanchor_centre_px must be non-negative pixels")
        if not np.isfinite(acquire_timeout_s) or acquire_timeout_s < 0:
            raise ValueError("acquire_timeout_s must be non-negative")
        if not np.isfinite(seed_margin) or seed_margin < 1.0:
            raise ValueError("seed_margin must be at least 1.0")
        if max_disagreements < 1:
            raise ValueError("max_disagreements must be at least one")
        # ``flow=False`` is a real mode, not a degraded one: a landmark or blob
        # model that answers faster than the frame period is a better tracker
        # than flow on the same object, and on a deformable target (a hand)
        # it is the only honest one. What it is not is a *measurement* of the
        # current frame, so its readings are reported as coasting.
        if not flow and detector is None:
            raise ValueError("flow=False needs a detector: there is nothing else")
        self.flow = bool(flow)
        self.support = support
        self.detector = detector
        self.selection = selection
        self.max_coast_s = float(max_coast_s)
        self.agree_centre_px = float(agree_centre_px)
        self.agree_extent_ratio = float(agree_extent_ratio)
        self.reanchor_centre_px = float(reanchor_centre_px)
        self.acquire_timeout_s = float(acquire_timeout_s)
        self.seed_margin = float(seed_margin)
        self.max_disagreements = int(max_disagreements)
        self._clock = clock
        self.target_id = "target"
        self.status = TrackerStatus()
        self._spec: TrackerSpec | None = None
        self._image_size: tuple[int, int] | None = None
        self._last_detection: TargetGeometry | None = None
        self._seen_frame_id = -1
        self._reacquisitions = 0
        self._reanchors = 0
        self._convention_skips = 0
        self._pending_note = ""
        self._last_frame = None
        self._last_reading: TrackerReading | None = None

    # -- lifecycle --------------------------------------------------------
    def reset(self, reason: str = "") -> None:
        self.support.reset(reason)
        self._spec = None
        self._last_detection = None
        self._seen_frame_id = -1
        self._last_frame = None
        self._last_reading = None
        self.status.reset(reason)
        self.status.state = IDLE if reason else LOST

    def _reading(self, geometry: TargetGeometry | None, state: str,
                 frame=None, reason: str | None = None, **extras) -> TrackerReading:
        self.status.state = state
        self.status.coasting_frames = (self.status.coasting_frames + 1
                                       if state == COASTING else 0)
        if reason is not None:
            self.status.reason = reason
        else:
            self.status.reason = self._pending_note
        return TrackerReading(geometry, self.status, frame,
                              dict(extras, note=self.status.reason))

    # -- acquisition ------------------------------------------------------
    def start(self, frame_rgb, spec: TrackerSpec, captured_at: float) -> TrackerReading:
        frame = np.asarray(frame_rgb)
        reading = self._start(frame, spec, captured_at)
        self._last_frame, self._last_reading = frame, reading
        self._last_detection = self._last_detection if reading.usable else None
        return reading

    def _start(self, frame, spec: TrackerSpec, captured_at: float) -> TrackerReading:
        spec = spec.validate()
        self.reset("")
        self.target_id = spec.target_id
        self.support.target_id = spec.target_id
        self._spec = spec
        self._image_size = (int(frame.shape[1]), int(frame.shape[0]))

        if spec.roi_px is not None:
            if not self.flow:
                raise ValueError("a selected region needs flow tracking; "
                                 "the detector-only mode locates the target itself")
            reading = self.support.start(frame, spec, captured_at)
            if reading.usable:
                self._submit(frame, captured_at)
                self.status = TrackerStatus(frames=1)
                return self._reading(reading.geometry, TRACKING, frame,
                                     reason="seeded from the selected region",
                                     source="flow")
            # A region with too little texture is not a dead end: the detector
            # may still see the object inside it.
            if self.detector is None:
                return reading

        if self.detector is None:
            self.status.reset("no region and no detector: nothing to acquire from")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame)

        self._submit(frame, captured_at)
        geometry = self._acquire_geometry(spec)
        if geometry is None:
            self.status.reset(
                f"no detection matching {spec.prompt or spec.label or spec.marker_id!r} "
                f"within {self.acquire_timeout_s:.1f}s")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame)
        self._last_detection = geometry
        self.status = TrackerStatus(frames=1, detections=1,
                                    last_detection_at=geometry.captured_at)
        if not self.flow:
            return self._reading(geometry, COASTING, frame, source="detector",
                                 reason="detector-only mode: the fix is from an "
                                        "earlier frame, not this one")
        seeded = self._seed(geometry, frame, captured_at)
        if seeded is None:
            # The detector located the object but the region had no trackable
            # texture. That is still a usable fix, just not a per-frame one.
            return self._reading(geometry, COASTING, frame, source="detector",
                                 reason="detected but not textured enough "
                                        "to flow-track")
        return self._reading(seeded, TRACKING, frame, reason="acquired",
                             source="flow")

    def _acquire_geometry(self, spec: TrackerSpec) -> TargetGeometry | None:
        """Wait, once and bounded, for a detection this spec actually asked for."""
        batch = self.detector.wait_for_first(self.acquire_timeout_s)
        if batch is None:
            return None
        geometry = self._geometry_from(batch, spec)
        if geometry is not None:
            self._seen_frame_id = batch.frame_id
            return geometry
        deadline = self._clock() + self.acquire_timeout_s
        while self._clock() < deadline:
            batch = self.detector.wait_for_frame(batch.frame_id, _ACQUIRE_POLL_S)
            if batch is None:
                continue
            geometry = self._geometry_from(batch, spec)
            if geometry is not None:
                self._seen_frame_id = batch.frame_id
                return geometry
        return None

    def _geometry_from(self, batch: DetectionBatch,
                       spec: TrackerSpec | None) -> TargetGeometry | None:
        if self._image_size is None:
            return None
        detections = list(batch.detections)
        if spec is not None and spec.label:
            labelled = [item for item in detections if item.label == spec.label]
            detections = labelled
        selection = self.selection
        picked = select_detection(detections, selection, self._image_size)
        if picked is None and selection != "score":
            # The central policy can find nothing near the centre when the object
            # is at the frame edge; falling back keeps acquisition working.
            picked = select_detection(detections, "score", self._image_size)
        if picked is None:
            return None
        try:
            return picked.to_geometry(
                self._image_size, batch.captured_at,
                self.target_id, source=f"detector:{picked.label or 'object'}")
        except ValueError:
            # A detection that cannot be turned into a geometry (empty mask,
            # degenerate box) is a detector problem, not a servo problem.
            return None

    def _seed(self, geometry: TargetGeometry, frame, captured_at: float):
        """Restart the flow tracker on a detected object; None if it will not lock."""
        if self._image_size is None:
            return None
        try:
            roi = roi_from_geometry(geometry, self.seed_margin, self._image_size)
        except ValueError:
            return None
        seed_spec = TrackerSpec(roi_px=roi, target_id=self.target_id)
        reading = self.support.start(frame, seed_spec, captured_at)
        if not reading.usable:
            return None
        return reading.geometry

    def _submit(self, frame, captured_at: float) -> None:
        if self.detector is not None:
            self.detector.submit(frame, captured_at)

    # -- per-frame --------------------------------------------------------
    def update(self, frame_rgb, captured_at: float) -> TrackerReading:
        frame = np.asarray(frame_rgb)
        # Two callers legitimately consume the same frame: the servo samples it
        # and the motion guard re-checks the target on the very frame it is
        # about to command against. Advancing the flow twice on one frame would
        # either double-count the motion or, worse, be rejected as a frame gap
        # and reported as a loss. Identity of the array is the honest test --
        # every camera adapter allocates a new array per read.
        if frame is self._last_frame and self._last_reading is not None:
            return self._last_reading
        self._last_frame = frame
        reading = self._update(frame, captured_at)
        self._last_reading = reading
        return reading

    def _update(self, frame, captured_at: float) -> TrackerReading:
        self.status.frames += 1
        self._pending_note = ""
        self._image_size = (int(frame.shape[1]), int(frame.shape[0]))
        self._submit(frame, captured_at)

        batch = self.detector.latest() if self.detector is not None else None
        new_batch = batch is not None and batch.frame_id != self._seen_frame_id
        new_geometry = None
        if new_batch:
            self._seen_frame_id = batch.frame_id
            self.status.detections += 1
            self.status.last_detection_at = float(batch.captured_at)
            new_geometry = self._geometry_from(batch, self._spec)
            if new_geometry is not None:
                self._last_detection = new_geometry

        if not self.flow:
            # No flow to arbitrate against: the newest detection is the best
            # knowledge available, and it is always from an earlier frame, so it
            # is reported as coasting rather than as a measurement.
            coast = self._coastable(captured_at)
            if coast is not None:
                return self._reading(coast, COASTING, frame, source="detector",
                                     reason="detector-only mode")
            self.status.reset("no detection inside the coasting window")
            self.status.state = LOST
            return TrackerReading(None, self.status, frame)

        flow = self.support.update(frame, captured_at)

        if new_geometry is not None:
            return self._reconcile(flow, new_geometry, frame, captured_at)

        if flow.usable:
            return self._reading(flow.geometry, TRACKING, frame,
                                 source="flow", **flow.extras)

        # Flow died. Coast only while a detection is young enough to mean
        # something; beyond that the honest answer is that the target is lost.
        coast = self._coastable(captured_at)
        if coast is not None:
            return self._reading(coast, COASTING, frame, source="detector")
        self.status.reset(flow.status.reason or "target lost")
        self.status.state = LOST
        return TrackerReading(None, self.status, frame)

    def _reconcile(self, flow: TrackerReading, geometry: TargetGeometry,
                   frame, captured_at: float) -> TrackerReading:
        """The detector has spoken. Keep the flow if it agrees, else re-anchor."""
        if not flow.usable:
            self._reacquisitions += 1
            seeded = self._seed(geometry, frame, captured_at)
            if seeded is not None:
                return self._reading(seeded, TRACKING, frame,
                                     reason="reacquired from detection",
                                     source="flow+reacquire",
                                     reacquisitions=self._reacquisitions)
            # No texture to lock onto: keep reporting the detection, but the
            # detection is not this frame, so it is a coast, not a measurement.
            return self._reading(geometry, COASTING, frame, source="detector",
                                 reason="detected but not textured enough "
                                        "to flow-track")

        verdict, note = self._agreement(flow.geometry, geometry)
        if verdict == "agree":
            # An agreement reached without comparing sizes is still worth
            # recording: the note says which check was skipped and why.
            return self._reading(flow.geometry, TRACKING, frame,
                                 reason=note or None, source="flow", **flow.extras)

        self.status.disagreements += 1
        self._pending_note = note
        if self.status.disagreements >= self.max_disagreements:
            # Two trackers that contradict each other on every frame are not
            # describing a tracking problem, they are describing an ambiguous
            # target -- two similar objects, a reflection, a prompt that matches
            # several things. Re-anchoring again would just pick one of them at
            # random, so stop following the object altogether.
            self.support.reset("tracker disagreement limit")
            self.status.reset(f"{self.status.disagreements} consecutive "
                              "detector/flow disagreements")
            self.status.disagreements = 0
            self.status.state = LOST
            return TrackerReading(None, self.status, frame)
        shift = float(np.linalg.norm(flow.geometry.center_px - geometry.center_px))
        if shift >= self.reanchor_centre_px or verdict == "disagree":
            self._reanchors += 1
            seeded = self._seed(geometry, frame, captured_at)
            if seeded is not None:
                return self._reading(seeded, TRACKING, frame,
                                     source="flow+reanchor",
                                     reanchors=self._reanchors)
        # The detector wins ties: it knows what the object is, the flow only
        # knows that pixels moved. Reporting the detection here keeps the servo
        # from following a drift the model has already contradicted.
        return self._reading(geometry, COASTING, frame, source="detector",
                             reanchors=self._reanchors)

    def _agreement(self, flow: TargetGeometry,
                   detected: TargetGeometry) -> tuple[str, str]:
        """Compare two trackers on the same frame, in one extent convention."""
        scale = max(float(flow.extent_px), float(detected.extent_px))
        distance = float(np.linalg.norm(flow.center_px - detected.center_px))
        limit = self.agree_centre_px * scale
        if distance > limit:
            return "disagree", (f"detector and flow differ by {distance:.1f} px "
                                f"(limit {limit:.1f})")

        flow_side = to_side_equivalent(flow.extent_px, flow.extent_name)
        detected_side = to_side_equivalent(detected.extent_px, detected.extent_name)
        if flow_side is None or detected_side is None:
            # No shape-free equivalent: the sizes are not comparable quantities,
            # so the extent check is skipped and the skip is logged. Pretending
            # they agree would let a rotating object look like an approaching one.
            self._convention_skips += 1
            return "agree", (f"extent not comparable "
                             f"({flow.extent_name} vs {detected.extent_name})")
        ratio = flow_side / detected_side
        if not 1.0 - self.agree_extent_ratio <= ratio <= 1.0 + self.agree_extent_ratio:
            return "disagree", (f"apparent size ratio {ratio:.2f} outside "
                                f"{self.agree_extent_ratio:.2f}")
        return "agree", ""

    def _coastable(self, now: float) -> TargetGeometry | None:
        geometry = self._last_detection
        if geometry is None:
            return None
        age = now - geometry.captured_at
        if age < 0 or age > self.max_coast_s:
            return None
        return geometry

    # -- introspection ----------------------------------------------------
    @property
    def needs_reacquire(self) -> bool:
        """True when the flow tracker has no state and must be seeded again."""
        return self.support.status.state in (IDLE, LOST)

    def describe(self) -> dict[str, object]:
        health = None if self.detector is None else self.detector.health().describe()
        return {
            "kind": "fused",
            "target_id": self.target_id,
            "selection": self.selection,
            "max_coast_s": self.max_coast_s,
            "reacquisitions": self._reacquisitions,
            "reanchors": self._reanchors,
            "convention_skips": self._convention_skips,
            "flow": self.support.describe(),
            "detector_health": health,
            **self.status.describe(),
        }

    def close(self) -> None:
        if self.detector is not None:
            self.detector.close()
