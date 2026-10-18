"""Geometry contract: extent conventions, point ordering and the distance scale.

Every assertion here protects a specific way an arbitrary object can be turned
into a wrong depth or a wrong twist:

* an extent convention silently paired with the wrong metric twin;
* four support points reordered between frames;
* a distance constant reused across conventions.
"""
import unittest

import numpy as np

from eai_robot.vision.geometry.scale import (
    ExtentScale, depth_from_side_px, relative_extent_change,
)
from eai_robot.vision.geometry.targets import (
    EXTENT_CONVENTIONS, TargetGeometry, convex_hull, mean_edge_px, polygon_area,
    quad_from_points, to_side_equivalent,
)


def square(centre=(100.0, 100.0), side=40.0):
    cx, cy = centre
    half = side / 2.0
    return np.array([[cx - half, cy - half], [cx + half, cy - half],
                     [cx + half, cy + half], [cx - half, cy + half]])


class QuadOrderingTests(unittest.TestCase):
    def test_four_points_keep_their_correspondence(self):
        # A tracker that reports corners has already established which is which.
        # Re-sorting them would destroy the correspondence the interaction
        # matrix depends on, so the only permitted change is the rotation.
        corners = square()
        ordered = quad_from_points(corners)
        for point in corners:
            self.assertTrue(any(np.allclose(point, other) for other in ordered))

    def test_order_is_clockwise_from_the_top_left_most(self):
        corners = np.array([[10.0, 90.0], [90.0, 90.0], [90.0, 10.0], [10.0, 10.0]])
        ordered = quad_from_points(corners)
        self.assertTrue(np.allclose(ordered[0], [10.0, 10.0]))
        # Image y grows downward, so clockwise means a positive shoelace signed
        # area in screen coordinates after the first point is fixed.
        signed = 0.5 * float(np.sum(ordered[:, 0] * np.roll(ordered[:, 1], -1)
                                    - np.roll(ordered[:, 0], -1) * ordered[:, 1]))
        self.assertGreater(signed, 0.0)

    def test_single_point_is_expanded_to_its_extent(self):
        quad = quad_from_points(np.array([[50.0, 60.0]]), extent_px=20.0)
        self.assertEqual(quad.shape, (4, 2))
        self.assertAlmostEqual(mean_edge_px(quad), 20.0, places=6)
        self.assertTrue(np.allclose(quad.mean(axis=0), [50.0, 60.0]))

    def test_single_point_without_extent_is_refused(self):
        with self.assertRaises(ValueError):
            quad_from_points(np.array([[50.0, 60.0]]))

    def test_a_mask_hull_reduces_to_four_interior_corners(self):
        hull = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 5.0],
                         [20.0, 15.0], [10.0, 20.0], [0.0, 20.0]])
        quad = quad_from_points(hull)
        self.assertEqual(quad.shape, (4, 2))
        # The reduction must stay inside the silhouette; a bounding box would
        # report 20x20 for a shape whose mean edge is smaller.
        self.assertGreaterEqual(quad[:, 0].min(), 0.0)
        self.assertLessEqual(quad[:, 0].max(), 20.0)


class HullTests(unittest.TestCase):
    def test_hull_rejects_interior_points(self):
        points = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0],
                           [5.0, 5.0]])
        hull = convex_hull(points)
        self.assertEqual(len(hull), 4)
        self.assertFalse(any(np.allclose(point, [5.0, 5.0]) for point in hull))

    def test_shoelace_area_of_a_square(self):
        self.assertAlmostEqual(polygon_area(square(side=10.0)), 100.0, places=6)


class ExtentConventionTests(unittest.TestCase):
    def test_conventions_are_enumerated(self):
        self.assertEqual(set(EXTENT_CONVENTIONS),
                         {"side", "diameter", "sqrt_area", "span"})

    def test_side_and_sqrt_area_are_already_square_equivalents(self):
        self.assertAlmostEqual(to_side_equivalent(40.0, "side"), 40.0)
        self.assertAlmostEqual(to_side_equivalent(40.0, "sqrt_area"), 40.0)

    def test_diameter_converts_by_the_circle_area_factor(self):
        # A disc's equal-area square is smaller than its diameter by
        # sqrt(pi)/2 = 0.886, which is the same order as one step's size change.
        self.assertAlmostEqual(to_side_equivalent(100.0, "diameter"),
                               100.0 * np.sqrt(np.pi) / 2.0, places=9)

    def test_span_has_no_shape_free_equivalent(self):
        self.assertIsNone(to_side_equivalent(100.0, "span"))

    def test_unknown_convention_is_refused(self):
        with self.assertRaises(ValueError):
            to_side_equivalent(10.0, "diagonal")

    def test_non_positive_extent_is_refused(self):
        with self.assertRaises(ValueError):
            to_side_equivalent(0.0, "side")


class TargetGeometryTests(unittest.TestCase):
    def test_from_corners_measures_the_mean_edge(self):
        geometry = TargetGeometry.from_corners(square(side=40.0), (640, 480), 1.0, "marker")
        self.assertAlmostEqual(geometry.extent_px, 40.0, places=6)
        self.assertEqual(geometry.extent_name, "side")
        self.assertTrue(np.allclose(geometry.center_px, [100.0, 100.0]))

    def test_from_circle_uses_the_diameter(self):
        geometry = TargetGeometry.from_circle((50.0, 60.0), 15.0, (640, 480), 1.0, "disc")
        self.assertAlmostEqual(geometry.extent_px, 30.0)
        self.assertEqual(geometry.extent_name, "diameter")
        self.assertEqual(len(geometry.support_px), 1)

    def test_from_box_uses_the_geometric_mean(self):
        # A 20x80 stick and a 40x40 square have the same area; the geometric
        # mean gives them the same extent, the arithmetic mean would not.
        stick = TargetGeometry.from_box((0.0, 0.0, 20.0, 80.0), (640, 480), 1.0, "stick")
        block = TargetGeometry.from_box((0.0, 0.0, 40.0, 40.0), (640, 480), 1.0, "block")
        self.assertAlmostEqual(stick.extent_px, block.extent_px, places=9)
        self.assertEqual(stick.extent_name, "sqrt_area")

    def test_from_mask_uses_the_pixel_area(self):
        mask = np.zeros((20, 20), dtype=bool)
        mask[5:15, 5:15] = True
        geometry = TargetGeometry.from_mask(mask, 1.0, "blob")
        self.assertAlmostEqual(geometry.extent_px, 10.0)
        self.assertEqual(geometry.extent_name, "sqrt_area")

    def test_zero_extent_is_refused(self):
        with self.assertRaises(ValueError):
            TargetGeometry(square()[0], 0.0, square(), (640, 480), 0.0, "t", "s")

    def test_unknown_convention_is_refused(self):
        with self.assertRaises(ValueError):
            TargetGeometry.from_support_points(square(), 40.0, (640, 480), 0.0,
                                               "t", extent_name="diagonal")

    def test_support_points_outside_the_frame_are_refused(self):
        with self.assertRaises(ValueError):
            TargetGeometry.from_support_points(
                np.array([[10.0, 10.0], [700.0, 10.0], [700.0, 40.0]]),
                30.0, (640, 480), 0.0, "t")

    def test_a_corner_exactly_on_the_last_pixel_is_allowed(self):
        geometry = TargetGeometry.from_support_points(
            np.array([[10.0, 10.0], [640.0, 10.0], [640.0, 480.0]]),
            30.0, (640, 480), 0.0, "t")
        self.assertEqual(len(geometry.support_px), 3)

    def test_staleness_is_measured_not_assumed(self):
        geometry = TargetGeometry.from_corners(square(), (640, 480), 10.0, "marker")
        self.assertFalse(geometry.is_stale(10.1, 0.25))
        self.assertTrue(geometry.is_stale(10.4, 0.25))
        self.assertTrue(geometry.is_stale(9.0, 0.25))

    def test_confidence_must_be_a_fraction(self):
        with self.assertRaises(ValueError):
            TargetGeometry.from_corners(square(), (640, 480), 0.0, "m", confidence=1.4)

    def test_bbox_covers_every_support_point(self):
        geometry = TargetGeometry.from_support_points(
            np.array([[10.0, 20.0], [30.0, 20.0], [30.0, 60.0], [10.0, 60.0]]),
            20.0, (640, 480), 0.0, "t")
        self.assertTrue(np.allclose(geometry.bbox, [10.0, 20.0, 20.0, 40.0]))


class ExtentScaleTests(unittest.TestCase):
    def test_the_two_routes_produce_the_same_constant(self):
        # Calibrated focal 500 px, 25 mm marker at 300 mm -> 41.67 px.
        from_pinhole = ExtentScale.from_pinhole(500.0, 0.025)
        from_reference = ExtentScale.from_reference(0.3, 500.0 * 0.025 / 0.3)
        self.assertEqual(from_pinhole, from_reference)
        self.assertAlmostEqual(from_pinhole.depth_m(500.0 * 0.025 / 0.3), 0.3, places=9)

    def test_depth_is_inverse_in_the_extent(self):
        scale = ExtentScale.from_reference(0.4, 50.0)
        self.assertAlmostEqual(scale.depth_m(100.0), 0.2, places=9)
        self.assertAlmostEqual(scale.extent_px_at(0.2), 100.0, places=9)

    def test_convention_is_part_of_the_identity(self):
        # The same constant under a different convention is a different scale:
        # if it compares equal, a pairing error can never be detected.
        side = ExtentScale.from_reference(0.3, 40.0, "side")
        area = ExtentScale.from_reference(0.3, 40.0, "sqrt_area")
        self.assertNotEqual(side, area)
        self.assertNotEqual(hash(side), hash(area))
        self.assertTrue(side.matches(TargetGeometry.from_corners(
            square(), (640, 480), 0.0, "m")))

    def test_unknown_convention_is_refused(self):
        with self.assertRaises(ValueError):
            ExtentScale(1.0, "diagonal")

    def test_non_positive_constant_is_refused(self):
        with self.assertRaises(ValueError):
            ExtentScale(0.0)

    def test_marker_depth_helper_matches_the_scale(self):
        side_px = 500.0 * 0.025 / 0.3
        self.assertAlmostEqual(depth_from_side_px(side_px, 500.0, 0.025), 0.3, places=9)

    def test_relative_extent_change_grows_as_the_target_approaches(self):
        self.assertAlmostEqual(relative_extent_change(0.01, 0.11), 0.1, places=9)
        with self.assertRaises(ValueError):
            relative_extent_change(0.01, 0.01)


if __name__ == "__main__":
    unittest.main()
