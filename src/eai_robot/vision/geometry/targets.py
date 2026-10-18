"""One target representation shared by markers, hands and arbitrary objects.

The fixed rule was written against four `ArUco` corners plus an apparent side
length. Both of those are marker-specific: a circle has no corners, a hand has
no square, and a segmentation mask has neither. Yet the rule itself only ever
asks three questions -- where is the target, how big does it look, and which way
should the camera move -- so the representation can be general.

`TargetGeometry` answers exactly those three questions and is deliberately
narrower than a full pose estimate:

* ``center_px`` -- where the target is;
* ``extent_px`` -- one scalar length that shrinks with distance;
* ``support_px`` -- the points the interaction matrix is evaluated at.

`extent_px` is a *convention*, not a measurement, and the convention must pair
with the metric size used to turn it into a depth. A square's side paired with a
measured side is valid; a square's side paired with a measured diagonal is 1.41x
wrong. ``extent_name`` carries that convention so the pairing can be checked
instead of assumed.

Two rules keep this honest:

1. ``extent_px`` must come from the same tracker state that produced
   ``support_px``. Recomputing a bounding box every frame would make the depth
   signal jump with detection noise, which is exactly what the fixed rule cannot
   tolerate at a ~2 px per-step signal.
2. ``support_px`` are *tracked* points with persistent correspondence, not
   per-frame extrema. A per-frame ``minAreaRect`` reorders its corners, and an
   interaction matrix built from reordered corners produces a twist in a random
   direction.

Everything here is pure numerics: no camera, no model, no I/O.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# How ``extent_px`` was measured. The metric twin must use the same convention.
EXTENT_CONVENTIONS = ("side", "diameter", "sqrt_area", "span")


#: Conversion from a convention to a square side with the same area. ``None``
#: means the convention cannot be converted without knowing the shape.
_SIDE_EQUIVALENT = {
    "side": lambda extent: extent,
    "sqrt_area": lambda extent: extent,
    "diameter": lambda extent: extent * float(np.sqrt(np.pi)) / 2.0,
    "span": None,
}


def to_side_equivalent(extent_px: float, convention: str) -> float | None:
    """Express an apparent extent as the side of an equal-area square.

    Two trackers on the same object must agree about "how big", but they will
    report different *conventions*: a segmentation reports ``sqrt(area)``, a
    hand reports the edge of its landmark quad, a disc detector reports a
    diameter. Comparing those numbers directly is a comparison of different
    quantities -- a diameter is 1.13x its equal-area side, which is the same
    order as the extent change produced by one 1 cm step at 300 mm.

    ``span`` returns ``None`` on purpose. The longest chord of a silhouette
    depends on its shape, so there is no shape-free square equivalent; a fusion
    step that needs to compare a span against anything else must say so rather
    than quietly applying a factor.
    """
    if convention not in EXTENT_CONVENTIONS:
        raise ValueError(f"unknown extent convention: {convention}")
    value = float(extent_px)
    if not np.isfinite(value) or value <= 0:
        raise ValueError("apparent extent must be positive")
    convert = _SIDE_EQUIVALENT[convention]
    return None if convert is None else float(convert(value))


def polygon_area(points_px) -> float:
    """Shoelace area of a closed polygon, in pixels squared."""
    points = np.asarray(points_px, dtype=float).reshape(-1, 2)
    if len(points) < 3:
        return 0.0
    x, y = points[:, 0], points[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def mean_edge_px(points_px) -> float:
    """Mean closed-polygon edge length, i.e. the ArUco 'apparent side'."""
    points = np.asarray(points_px, dtype=float).reshape(-1, 2)
    if len(points) < 3:
        raise ValueError("a closed polygon needs at least three points")
    edges = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
    return float(np.mean(edges))


def convex_hull(points_px) -> np.ndarray:
    """Andrew monotone chain; counter-clockwise, no repeated first point.

    Hand-written so the geometry layer stays numpy-only and stays testable
    without OpenCV.
    """
    points = np.unique(np.asarray(points_px, dtype=float).reshape(-1, 2), axis=0)
    if len(points) <= 2:
        return points

    def cross(origin, a, b):
        return ((a[0] - origin[0]) * (b[1] - origin[1])
                - (a[1] - origin[1]) * (b[0] - origin[0]))

    lower: list[np.ndarray] = []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[np.ndarray] = []
    for point in points[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1] + upper[:-1], dtype=float)


def _farthest_pair(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    best = (points[0], points[0], 0.0)
    for index in range(len(points)):
        distances = np.linalg.norm(points - points[index], axis=1)
        other = int(np.argmax(distances))
        if distances[other] > best[2]:
            best = (points[index], points[other], float(distances[other]))
    return best


def quad_from_points(points_px, extent_px: float | None = None) -> np.ndarray:
    """Reduce any point set to four ordered points for the interaction matrix.

    Four points are returned unchanged: they already carry correspondence and
    ordering from the tracker. One or two points are expanded into a square of
    ``extent_px`` around their centre, because a single centroid still has a
    well-defined apparent size. Anything else is reduced to the four extreme
    hull points so the result stays inside the silhouette rather than on a
    bounding box that is re-estimated every frame.

    The returned order is clockwise starting from the top-left-most point, which
    matches what OpenCV's ArUco detector produces and therefore what the existing
    task matrices expect.
    """
    points = np.asarray(points_px, dtype=float).reshape(-1, 2)
    if len(points) < 1:
        raise ValueError("at least one support point is required")
    if len(points) == 4:
        return _order_clockwise(points)

    if len(points) < 4:
        if extent_px is None or not np.isfinite(extent_px) or extent_px <= 0:
            raise ValueError("expanding fewer than four points needs a positive extent")
        centre = points.mean(axis=0)
        half = float(extent_px) / 2.0
        square = np.array([[centre[0] - half, centre[1] - half],
                           [centre[0] + half, centre[1] - half],
                           [centre[0] + half, centre[1] + half],
                           [centre[0] - half, centre[1] + half]])
        return _order_clockwise(square)

    hull = convex_hull(points)
    if len(hull) == 4:
        return _order_clockwise(hull)
    if len(hull) < 4:
        return quad_from_points(points, extent_px)
    # Pick four corners spread around the hull: start from the top-left-most and
    # walk the hull in equal strides. Deterministic, ordering-preserving and
    # robust to a hull with six or eight vertices.
    start = int(np.lexsort((hull[:, 1], hull[:, 0]))[0])
    indices = np.rint(np.linspace(0, len(hull), 4, endpoint=False)).astype(int)
    return _order_clockwise(hull[(start + indices) % len(hull)])


def _order_clockwise(points: np.ndarray) -> np.ndarray:
    """Sort four points clockwise from the top-left-most, image coordinates."""
    points = np.asarray(points, dtype=float).reshape(4, 2)
    centre = points.mean(axis=0)
    angles = np.arctan2(points[:, 1] - centre[1], points[:, 0] - centre[0])
    ordered = points[np.argsort(angles)]
    # Image y grows downward: increasing atan2 is clockwise on screen. Rotate so
    # the first point is the top-left-most, i.e. the smallest x+y.
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


@dataclass(frozen=True)
class TargetGeometry:
    """Where a target is, how large it looks, and what the task matrix uses."""

    center_px: np.ndarray
    extent_px: float
    support_px: np.ndarray
    image_size: tuple[int, int]
    captured_at: float
    target_id: str
    source: str
    confidence: float = 1.0
    extent_name: str = "side"

    def __post_init__(self) -> None:
        centre = np.array(self.center_px, dtype=float, copy=True).reshape(2)
        support = np.array(self.support_px, dtype=float, copy=True).reshape(-1, 2)
        object.__setattr__(self, "center_px", centre)
        object.__setattr__(self, "support_px", support)
        self._validate()

    def _validate(self) -> None:
        width, height = self.image_size
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        if not self.target_id or not self.source:
            raise ValueError("target identity and source are required")
        if self.extent_name not in EXTENT_CONVENTIONS:
            raise ValueError(f"unknown extent convention: {self.extent_name}")
        values = np.r_[self.center_px, self.support_px.reshape(-1), self.extent_px,
                       self.confidence, self.captured_at]
        if not np.all(np.isfinite(values)):
            raise ValueError("target geometry must be finite")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be within 0..1")
        if self.extent_px <= 0:
            raise ValueError("apparent extent must be positive")
        if not len(self.support_px):
            raise ValueError("at least one support point is required")
        limit = np.array([width, height], dtype=float)
        # Corners may legitimately sit exactly on the last pixel, so compare
        # against the frame size rather than rejecting at width-1.
        if np.any(self.support_px < 0) or np.any(self.support_px > limit):
            raise ValueError("support points must lie inside the image")

    @property
    def as_quad(self) -> np.ndarray:
        return quad_from_points(self.support_px, self.extent_px)

    @property
    def bbox(self) -> np.ndarray:
        low = self.support_px.min(axis=0)
        high = self.support_px.max(axis=0)
        return np.array([low[0], low[1], high[0] - low[0], high[1] - low[1]])

    def age_s(self, now: float) -> float:
        return float(now - self.captured_at)

    def is_stale(self, now: float, max_age_s: float = 0.25) -> bool:
        age = self.age_s(now)
        return age < 0 or age > max_age_s

    def describe(self) -> dict[str, object]:
        return {
            "target_id": self.target_id,
            "source": self.source,
            "center_px": np.round(self.center_px, 2).tolist(),
            "extent_px": round(float(self.extent_px), 3),
            "extent_name": self.extent_name,
            "support_points": len(self.support_px),
            "confidence": round(float(self.confidence), 3),
        }

    # -- constructors -----------------------------------------------------
    @classmethod
    def from_corners(cls, corners_px, image_size, captured_at, target_id,
                     *, source="aruco", confidence=1.0) -> "TargetGeometry":
        """A detected square: four corners, side convention."""
        corners = np.asarray(corners_px, dtype=float).reshape(4, 2)
        return cls(corners.mean(axis=0), mean_edge_px(corners), corners, image_size,
                   captured_at, target_id, source, confidence, "side")

    @classmethod
    def from_circle(cls, center_px, radius_px, image_size, captured_at, target_id,
                    *, source="circle", confidence=1.0) -> "TargetGeometry":
        """A disc: apparent extent is the diameter, one support point."""
        radius = float(radius_px)
        if not np.isfinite(radius) or radius <= 0:
            raise ValueError("radius must be positive")
        return cls(center_px, 2.0 * radius, np.asarray(center_px, float).reshape(1, 2),
                   image_size, captured_at, target_id, source, confidence, "diameter")

    @classmethod
    def from_box(cls, box_px, image_size, captured_at, target_id,
                 *, source="box", confidence=1.0) -> "TargetGeometry":
        """An axis-aligned box: extent is the geometric mean of its sides.

        The geometric mean is used instead of the arithmetic mean so that a long
        thin object and a square with the same area get the same extent; the
        arithmetic mean would inflate a stick by its length.
        """
        x, y, width, height = np.asarray(box_px, dtype=float).reshape(4)
        if width <= 0 or height <= 0:
            raise ValueError("box must have positive width and height")
        corners = np.array([[x, y], [x + width, y], [x + width, y + height],
                            [x, y + height]])
        centre = np.array([x + width / 2.0, y + height / 2.0])
        return cls(centre, float(np.sqrt(width * height)), corners, image_size,
                   captured_at, target_id, source, confidence, "sqrt_area")

    @classmethod
    def from_mask(cls, mask, captured_at, target_id, *, source="mask",
                  confidence=1.0) -> "TargetGeometry":
        """A segmentation mask: extent is sqrt(area), support is the hull.

        ``sqrt_area`` pairs with the square root of the object's real silhouette
        area, which is the only metric quantity a mask actually measures.
        """
        values = np.asarray(mask)
        if values.ndim != 2:
            raise ValueError("mask must be two-dimensional")
        rows, columns = np.nonzero(values)
        if not len(rows):
            raise ValueError("mask is empty")
        height, width = values.shape
        ys, xs = rows.astype(float), columns.astype(float)
        centre = np.array([xs.mean() + 0.5, ys.mean() + 0.5])
        hull = convex_hull(np.column_stack([xs + 0.5, ys + 0.5]))
        return cls(centre, float(np.sqrt(len(rows))), hull, (width, height),
                   captured_at, target_id, source, confidence, "sqrt_area")

    @classmethod
    def from_support_points(cls, support_px, extent_px, image_size, captured_at,
                            target_id, *, source="support", confidence=1.0,
                            extent_name="side") -> "TargetGeometry":
        """Tracked points plus an extent measured on the same state."""
        points = np.asarray(support_px, dtype=float).reshape(-1, 2)
        return cls(points.mean(axis=0), extent_px, points, image_size,
                   captured_at, target_id, source, confidence, extent_name)
