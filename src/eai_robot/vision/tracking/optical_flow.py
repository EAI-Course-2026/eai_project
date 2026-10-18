"""User-selected textured-object tracking using forward/backward LK flow."""
import cv2
import numpy as np

from eai_robot.vision.features import VisualFeatures


class ObjectFlowTracker:
    """Track a selected ROI until loss; reinitialization is always explicit.

    Emits a center feature only. Texture points do not define metric corners or
    depth and must not be passed to the marker four-corner IBVS controller.
    """

    def __init__(self, *, max_gap_s=0.25, min_points=6):
        if not np.isfinite(max_gap_s) or max_gap_s <= 0 or min_points < 4:
            raise ValueError("max_gap_s must be positive and min_points >= 4")
        self.max_gap_s = max_gap_s
        self.min_points = min_points
        self.reset()

    def reset(self):
        self._gray = self._points = self._center = None
        self._stamp = None
        self._identity = None

    @staticmethod
    def _to_gray(frame):
        frame = np.asarray(frame)
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.dtype != np.uint8:
            raise ValueError("frame must be uint8 RGB")
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

    def initialize(self, frame_rgb, roi, captured_at, *, target_id="object:selection"):
        self.reset()
        gray = self._to_gray(frame_rgb)
        if not np.isfinite(captured_at) or not target_id:
            raise ValueError("finite timestamp and target identity are required")
        values = np.asarray(roi, dtype=float)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("ROI must be finite x, y, width, height")
        x, y, width, height = values
        if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > gray.shape[1] or y + height > gray.shape[0]:
            raise ValueError("ROI must fit inside the image")
        mask = np.zeros_like(gray)
        mask[int(y):int(y + height), int(x):int(x + width)] = 255
        points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=.02,
                                         minDistance=5, mask=mask, blockSize=5)
        if points is None or len(points) < self.min_points:
            return None
        self._gray, self._points = gray, points
        self._center = np.array([x + width / 2, y + height / 2])
        self._stamp, self._identity = captured_at, target_id
        return self._observation()

    def _observation(self):
        height, width = self._gray.shape
        return VisualFeatures(self._center.reshape(1, 2), (width, height),
                              self._stamp, self._identity)

    def update(self, frame_rgb, captured_at):
        gray = self._to_gray(frame_rgb)
        if self._gray is None:
            return None
        if (not np.isfinite(captured_at) or captured_at <= self._stamp
                or captured_at - self._stamp > self.max_gap_s
                or gray.shape != self._gray.shape):
            self.reset()
            return None
        forward, status_f, _ = cv2.calcOpticalFlowPyrLK(self._gray, gray, self._points, None)
        if forward is None or status_f is None or not np.all(np.isfinite(forward)):
            self.reset()
            return None
        backward, status_b, _ = cv2.calcOpticalFlowPyrLK(gray, self._gray, forward, None)
        if backward is None or status_b is None:
            self.reset()
            return None
        before, after = self._points.reshape(-1, 2), forward.reshape(-1, 2)
        valid = ((status_f.reshape(-1) != 0) & (status_b.reshape(-1) != 0)
                 & np.all(np.isfinite(backward.reshape(-1, 2)), axis=1)
                 & (np.linalg.norm(backward.reshape(-1, 2) - before, axis=1) <= 1.0)
                 & np.all(after >= 0, axis=1)
                 & np.all(after < [gray.shape[1], gray.shape[0]], axis=1))
        if valid.sum() < self.min_points:
            self.reset()
            return None
        # Robust partial affine fit rejects background points and allows modest
        # rotation/scale changes. It does not recover metric object geometry.
        transform, inliers = cv2.estimateAffinePartial2D(before[valid], after[valid],
                                                        method=cv2.RANSAC,
                                                        ransacReprojThreshold=2.)
        if transform is None or inliers is None or not np.all(np.isfinite(transform)):
            self.reset()
            return None
        keep = inliers.reshape(-1).astype(bool)
        if keep.sum() < self.min_points or keep.mean() < .6:
            self.reset()
            return None
        center = transform @ np.r_[self._center, 1.]
        if np.any(center < 0) or np.any(center >= [gray.shape[1], gray.shape[0]]):
            self.reset()
            return None
        self._gray, self._points = gray, after[valid][keep].reshape(-1, 1, 2)
        self._center, self._stamp = center, captured_at
        return self._observation()
