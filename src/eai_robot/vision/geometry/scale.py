"""Distance from apparent size, without requiring the object's real dimensions.

A pinhole camera turns an apparent extent into a depth through
``depth = focal_px * extent_m / extent_px``. The product ``focal_px * extent_m``
is a single constant for a given target, so the two "knowns" the ArUco path
needs -- a calibrated focal length and a measured marker side -- collapse into
one number that can also be obtained by measuring a *distance* instead of a
size:

    k = depth_reference_m * extent_px_at_reference

That matters for arbitrary objects. Nobody can tell the rule how many metres of
"apparent extent" a coffee cup has, because the extent is a property of the
tracker's point set, not of the cup. But anyone can put the cup at a measured
distance once, read the pixel extent, and divide.

Both routes are exactly equivalent, which is why they share one type:

===================  =============================  =========================
route                constant                       what the operator needs
===================  =============================  =========================
calibrated marker    ``focal_px * extent_m``        intrinsics + real side
reference distance   ``depth_m * extent_px``        a tape measure, once
===================  =============================  =========================

What this does *not* do is make the depth trustworthy. The relative error of a
depth equals the relative error of ``k``, so a 10 mm error on a 200 mm reference
is a 5 % depth error. The fixed rule therefore stops on the pixel criterion,
which needs no calibration at all, and treats depth as a reported figure. Use it
to reason about "roughly how far", never as a measurement.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from eai_robot.vision.geometry.targets import EXTENT_CONVENTIONS


@dataclass(frozen=True)
class ExtentScale:
    """``depth = constant_m_px / extent_px`` for one target, one convention.

    ``extent_name`` records which extent convention the constant was measured
    with. It is not decoration: the same object produces a different number
    under ``side`` and under ``sqrt_area``, so a constant measured under one
    convention silently returns a wrong depth under the other. Keeping the
    convention on the scale lets the caller check the pairing instead of
    discovering it as a depth error.
    """

    constant_m_px: float
    extent_name: str = "side"

    def __post_init__(self) -> None:
        value = float(self.constant_m_px)
        if not np.isfinite(value) or value <= 0:
            raise ValueError("extent constant must be positive metres times pixels")
        if self.extent_name not in EXTENT_CONVENTIONS:
            raise ValueError(f"unknown extent convention: {self.extent_name}")
        object.__setattr__(self, "constant_m_px", value)

    @classmethod
    def from_pinhole(cls, focal_px: float, extent_m: float,
                     extent_name: str = "side") -> "ExtentScale":
        """ArUco route: calibrated focal length and a measured real extent."""
        if not np.isfinite(focal_px) or focal_px <= 0:
            raise ValueError("focal length must be positive pixels")
        if not np.isfinite(extent_m) or extent_m <= 0:
            raise ValueError("extent must be positive metres")
        return cls(float(focal_px) * float(extent_m), extent_name)

    @classmethod
    def from_reference(cls, depth_m: float, extent_px: float,
                       extent_name: str = "side") -> "ExtentScale":
        """Arbitrary-object route: one measured distance and its pixel extent."""
        if not np.isfinite(depth_m) or depth_m <= 0:
            raise ValueError("reference depth must be positive metres")
        if not np.isfinite(extent_px) or extent_px <= 0:
            raise ValueError("reference extent must be positive pixels")
        return cls(float(depth_m) * float(extent_px), extent_name)

    def matches(self, geometry) -> bool:
        """True when this scale was measured with ``geometry``'s convention."""
        return getattr(geometry, "extent_name", None) == self.extent_name

    def depth_m(self, extent_px: float) -> float:
        value = float(extent_px)
        if not np.isfinite(value) or value <= 0:
            raise ValueError("apparent extent must be positive pixels")
        return self.constant_m_px / value

    def extent_px_at(self, depth_m: float) -> float:
        if not np.isfinite(depth_m) or depth_m <= 0:
            raise ValueError("depth must be positive metres")
        return self.constant_m_px / float(depth_m)

    def focal_px_for(self, extent_m: float) -> float:
        """Equivalent focal length if this target also has a known real size."""
        if not np.isfinite(extent_m) or extent_m <= 0:
            raise ValueError("extent must be positive metres")
        return self.constant_m_px / float(extent_m)

    def describe(self) -> dict[str, object]:
        return {"constant_m_px": round(self.constant_m_px, 6),
                "extent_name": self.extent_name}

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ExtentScale):
            return NotImplemented
        return (self.extent_name == other.extent_name
                and bool(np.isclose(self.constant_m_px, other.constant_m_px,
                                    rtol=1e-12, atol=0.0)))

    def __hash__(self) -> int:
        return hash((self.extent_name, round(self.constant_m_px, 12)))


def depth_from_side_px(side_px: float, focal_px: float, marker_length_m: float) -> float:
    """Pinhole depth of a fronto-parallel square marker from its apparent side.

    Retained as the explicit ArUco spelling of :meth:`ExtentScale.depth_m` so
    existing call sites and the round-1 records stay readable.
    """
    return ExtentScale.from_pinhole(focal_px, marker_length_m).depth_m(side_px)


def relative_extent_change(step_m: float, depth_m: float) -> float:
    """Expected fractional growth of the extent for a step toward the target."""
    if not np.isfinite(step_m) or step_m <= 0:
        raise ValueError("approach step must be positive metres")
    if not np.isfinite(depth_m) or depth_m <= step_m:
        raise ValueError("depth must exceed the approach step")
    return float(step_m / (depth_m - step_m))
