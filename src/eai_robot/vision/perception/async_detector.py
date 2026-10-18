"""Run a slow detector beside a fast control loop without stalling the loop.

The problem this solves is arithmetic, not architecture taste. The wrist camera
delivers 15 frames per second. A hand landmark model answers in 10-20 ms on CPU
and could run inline. An open-vocabulary detector plus a promptable segmenter
answers in 150-500 ms on this machine and cannot. If the control loop waits for
it, the loop becomes a 2-6 Hz loop, and every frame it commands from is already
a third of a second old -- which at 1 cm per step is a collision waiting to be
scheduled.

So the model runs on its own thread and the loop never waits for it:

* **Latest-only, never a queue.** ``submit`` replaces any frame the worker has not
  started yet. A queue would grow without bound whenever the model is slower than
  the camera, and the loop would then close on detections from ten frames ago
  while the queue drains. Replacing instead of enqueueing keeps the detection
  latency equal to the model latency, and counts the discarded frames so the
  loss is a number in the log rather than a feeling.
* **Failures are data.** A detector exception must not propagate into a motion
  loop. The batch is dropped, the failure counter rises, ``latest`` keeps the
  previous successful answer, and ``health`` turns degraded so the caller can
  shrink its coasting window instead of trusting an old box forever.
* **The frame is lent, not copied.** Copying a 640x480 RGB frame per submit costs
  more than some of the detectors. The caller must therefore hand over a frame it
  will not mutate; every camera adapter in this repository allocates a new array
  per read, which satisfies that contract.

Nothing here knows about servoing. It is a bounded-latency single-slot worker.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from eai_robot.vision.perception.detector import DetectionBatch, ObjectDetector


@dataclass(frozen=True)
class DetectorHealth:
    """Everything a control loop needs to decide how much to trust detection."""

    processed: int
    dropped: int
    failures: int
    pending: bool
    last_error: str
    latency_p50_s: float
    latency_p95_s: float
    budget_s: float
    degraded: bool

    def describe(self) -> dict[str, object]:
        return {
            "processed": self.processed,
            "dropped": self.dropped,
            "failures": self.failures,
            "pending": self.pending,
            "last_error": self.last_error,
            "latency_p50_s": round(self.latency_p50_s, 4),
            "latency_p95_s": round(self.latency_p95_s, 4),
            "budget_s": round(self.budget_s, 4),
            "degraded": self.degraded,
        }


class AsyncDetector:
    """Single-slot worker around any :class:`ObjectDetector`.

    The clock defaults to :func:`time.perf_counter`, not ``time.monotonic``:
    Windows implements the latter on ``GetTickCount64``, whose 15.6 ms tick is
    larger than the latency it is supposed to measure, so a fast detector
    reports exactly 0.0 and every percentile becomes a multiple of the tick.
    ``perf_counter`` is the high-resolution monotonic clock on both platforms,
    and this class never compares its readings against a caller's timestamp --
    ``captured_at`` is stored untouched -- so the two clock domains stay apart.
    """

    def __init__(self, detector: ObjectDetector, *, history: int = 32,
                 failure_limit: int = 3, clock=time.perf_counter):
        if not isinstance(detector, ObjectDetector):
            raise TypeError("detector must satisfy the ObjectDetector protocol")
        if failure_limit < 1:
            raise ValueError("failure_limit must be at least one")
        if history < 4:
            raise ValueError("history must hold at least four samples")
        self._detector = detector
        self._clock = clock
        self._failure_limit = int(failure_limit)
        self._condition = threading.Condition()
        self._pending: tuple[np.ndarray, float, int] | None = None
        self._latest: DetectionBatch | None = None
        self._latencies: deque[float] = deque(maxlen=int(history))
        self._closed = False
        self._processed = 0
        self._dropped = 0
        self._failures = 0
        self._last_error = ""
        self._frame_id = 0
        self._thread = threading.Thread(
            target=self._run, name=f"detector:{detector.name}", daemon=True)
        self._thread.start()

    # -- producer side ----------------------------------------------------
    def submit(self, frame_rgb: np.ndarray, captured_at: float) -> bool:
        """Offer a frame. Returns False when an older pending frame was dropped.

        Never blocks on the model. The return value is diagnostic only: a caller
        cannot use it to apply back-pressure, because applying back-pressure is
        exactly what this class exists to prevent.
        """
        frame = np.asarray(frame_rgb)
        with self._condition:
            if self._closed:
                return False
            replaced = self._pending is not None
            if replaced:
                self._dropped += 1
            self._frame_id += 1
            self._pending = (frame, float(captured_at), self._frame_id)
            self._condition.notify()
            return not replaced

    def latest(self) -> DetectionBatch | None:
        """Most recent successful batch; may be old, check ``age_s``."""
        with self._condition:
            return self._latest

    def wait_for_first(self, timeout_s: float) -> DetectionBatch | None:
        """Block once, for acquisition, where waiting is allowed and bounded."""
        deadline = self._clock() + max(0.0, float(timeout_s))
        with self._condition:
            while self._latest is None and not self._closed:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._latest

    def wait_for_frame(self, after_frame_id: int, timeout_s: float) -> DetectionBatch | None:
        """Wait for a batch newer than ``after_frame_id``; used to re-anchor."""
        deadline = self._clock() + max(0.0, float(timeout_s))
        with self._condition:
            while not self._closed:
                if self._latest is not None and self._latest.frame_id > after_frame_id:
                    return self._latest
                remaining = deadline - self._clock()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return None

    def last_frame_id(self) -> int:
        with self._condition:
            return 0 if self._latest is None else self._latest.frame_id

    # -- consumer side ----------------------------------------------------
    def health(self) -> DetectorHealth:
        with self._condition:
            samples = sorted(self._latencies)
            p50 = samples[len(samples) // 2] if samples else 0.0
            p95 = samples[min(len(samples) - 1, int(0.95 * len(samples)))] if samples else 0.0
            budget = float(self._detector.latency_budget_s)
            degraded = (self._failures >= self._failure_limit
                        or (p95 > 0 and p95 > 2.0 * budget))
            return DetectorHealth(
                processed=self._processed, dropped=self._dropped,
                failures=self._failures, pending=self._pending is not None,
                last_error=self._last_error, latency_p50_s=p50,
                latency_p95_s=p95, budget_s=budget, degraded=degraded)

    def describe(self) -> dict[str, object]:
        payload = dict(self._detector.describe())
        payload["async"] = self.health().describe()
        return payload

    # -- worker -----------------------------------------------------------
    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._closed:
                    return
                frame, captured_at, frame_id = self._pending
                self._pending = None
            started = self._clock()
            detections: list = []
            error = ""
            try:
                detections = list(self._detector.detect(frame))
            except Exception as exc:  # a model crash is not a motion-loop crash
                error = f"{type(exc).__name__}: {exc}"
            latency = self._clock() - started
            with self._condition:
                if self._closed:
                    return
                self._latencies.append(latency)
                self._processed += 1
                if error:
                    self._failures += 1
                    self._last_error = error
                else:
                    self._failures = 0
                    self._last_error = ""
                    self._latest = DetectionBatch(
                        tuple(detections), captured_at, latency, frame_id)
                self._condition.notify_all()

    def close(self) -> None:
        """Stop the worker and release the model. Safe to call twice."""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._pending = None
            self._condition.notify_all()
        self._thread.join(timeout=5.0)
        try:
            self._detector.close()
        except Exception:
            # Closing a model must not raise into a shutdown path.
            pass

    def __enter__(self) -> "AsyncDetector":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
