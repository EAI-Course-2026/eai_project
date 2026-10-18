"""Device-independent visual features shared by markers and object trackers."""
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VisualFeatures:
    """Ordered pixel points in one frame; time uses the monotonic clock.

    Four-point IBVS requires persistent corner correspondence. A tracker must
    not reorder points or substitute an axis-aligned box on each frame.
    """

    points_px: np.ndarray
    image_size: tuple[int, int]  # width, height
    captured_at: float
    target_id: str

    def __post_init__(self):
        points = np.array(self.points_px, dtype=float, copy=True)
        width, height = self.image_size
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        if points.ndim != 2 or points.shape[1] != 2 or not len(points):
            raise ValueError("points must have shape (N, 2)")
        if not np.all(np.isfinite(points)) or not np.isfinite(self.captured_at):
            raise ValueError("features and timestamp must be finite")
        if np.any(points < 0) or np.any(points >= [width, height]):
            raise ValueError("features must be inside the image")
        if not self.target_id:
            raise ValueError("target identity is required")
        points.setflags(write=False)
        object.__setattr__(self, "points_px", points)

    @property
    def center_px(self):
        return self.points_px.mean(axis=0)
