"""RGB/state assembly with local capture intervals and acquisition-age checks."""
from dataclasses import dataclass
import time
import numpy as np

from .contracts import KEYS, vector


@dataclass
class Observation:
    state: np.ndarray
    images: dict[str, np.ndarray]
    task: str
    captured_at: float
    completed_at: float
    camera_times: dict[str, float]

    def validate(self, manifest, *, now=None, max_age_s=None, max_skew_s=None):
        self.state = vector(self.state)
        if not isinstance(self.task, str) or not self.task.strip() or len(self.task) > 4096:
            raise ValueError("Nonempty bounded task text required")
        clocks = np.asarray([self.captured_at, self.completed_at, *self.camera_times.values()], dtype=float)
        if not np.isfinite(clocks).all() or self.completed_at < self.captured_at:
            raise ValueError("Invalid acquisition timestamps")
        required = {c["source"] for c in manifest.data["cameras"].values()}
        if set(self.images) != required or set(self.camera_times) != required:
            raise ValueError("Missing/extra camera or camera acquisition time")
        for name, image in self.images.items():
            if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not 1 <= min(image.shape[:2]) <= max(image.shape[:2]) <= 4096:
                raise ValueError(f"{name}: expected bounded uint8 HWC RGB image")
            if self.camera_times[name] > self.completed_at:
                raise ValueError("Camera time is after completion")
        oldest = min(self.captured_at, *self.camera_times.values())
        if max_skew_s is not None and self.completed_at - oldest > max_skew_s:
            raise ValueError("Image/joint acquisition skew exceeds budget")
        if now is not None and (self.completed_at > now or (max_age_s is not None and now - oldest > max_age_s)):
            raise ValueError("Observation is stale or from the future")
        return oldest

    def model_batch(self, manifest):
        self.validate(manifest)
        import cv2
        import torch
        batch = {"observation.state": torch.from_numpy(self.state.astype(np.float32)), "task": self.task}
        for key, spec in manifest.data["cameras"].items():
            _, height, width = spec["shape"]
            image = self.images[spec["source"]]
            image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
            batch[key] = torch.from_numpy(np.ascontiguousarray(image.transpose(2, 0, 1))).float() / 255.0
        # Official processors own batching, model normalization and tokenization.
        return batch


def load_observation(path, manifest, task):
    with np.load(path, allow_pickle=False) as data:
        images = {c["source"]: data[c["source"]].copy() for c in manifest.data["cameras"].values()}
        state = data["state"].copy()
    now = time.perf_counter()
    obs = Observation(state, images, task, now, now, {name: now for name in images})
    obs.validate(manifest)
    return obs


def fake_observation(manifest, task="Offline synthetic fixture"):
    now = time.perf_counter()
    images = {c["source"]: np.zeros((c["shape"][1], c["shape"][2], 3), dtype=np.uint8) for c in manifest.data["cameras"].values()}
    return Observation(np.array([0, 0, 0, 0, 0, 50.]), images, task, now, now, {name: now for name in images})
