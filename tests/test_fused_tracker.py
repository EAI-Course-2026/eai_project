"""Fused tracking, the tracker-to-servo adapters, and the coasting bound.

Two trackers on one object must not be averaged into one number. The detector
says *what* the object is and arrives late; the flow tracker says *where* it is
and arrives every frame. These tests pin the contract that keeps them apart:

* the servo closes on the flow, not on a stale detection;
* a detection may stand in for a measurement for a bounded time only;
* when the two disagree, the one that knows semantics wins and the flow is
  re-seeded rather than argued with;
* one camera frame advances the tracker exactly once, however many callers
  consume it.
"""
import threading
import time
import unittest

import cv2
import numpy as np

from eai_robot.control.visual_servo.fixed_rule import (
    FixedRuleConfig, FixedRuleController, Observation,
)
from eai_robot.vision.geometry.scale import ExtentScale
from eai_robot.vision.geometry.targets import TargetGeometry
from eai_robot.vision.perception.async_detector import DetectorHealth
from eai_robot.vision.perception.detector import Detection, DetectionBatch
from eai_robot.vision.tracking.base import (
    COASTING, LOST, TRACKING, TrackerReading, TrackerSpec, TrackerStatus,
)
from eai_robot.vision.tracking.fused import (
    FusedTracker, roi_from_geometry,
)
from eai_robot.vision.tracking.support_points import SupportPointTracker


def box_geometry(centre=(320.0, 240.0), side=60.0, captured_at=0.0,
                 target_id="target", extent_name="side"):
    return TargetGeometry.from_box(
        (centre[0] - side / 2, centre[1] - side / 2, side, side),
        (640, 480), captured_at, target_id, extent_name="sqrt_area"
    ) if extent_name != "side" else TargetGeometry.from_corners(
        np.array([[centre[0] - side / 2, centre[1] - side / 2],
                  [centre[0] + side / 2, centre[1] - side / 2],
                  [centre[0] + side / 2, centre[1] + side / 2],
                  [centre[0] - side / 2, centre[1] + side / 2]]),
        (640, 480), captured_at, target_id)


class StubSupport:
    """SupportPointTracker that answers from a script instead of from pixels."""

    def __init__(self):
        self.target_id = "target"
        self.status = TrackerStatus()
        self.script = []
        self.seeds = []
        self.seed_geometry = None
        self.updates = 0

    def reset(self, reason=""):
        self.status.reset(reason)
        self.status.state = LOST if not reason else "idle"

    def start(self, frame_rgb, spec, captured_at):
        spec.validate()
        self.seeds.append((spec.roi_px, captured_at))
        if self.seed_geometry is None:
            self.status.state = LOST
            self.status.reason = "no texture in the region"
            return TrackerReading(None, self.status, frame_rgb)
        self.status.state = TRACKING
        return TrackerReading(self.seed_geometry, self.status, frame_rgb)

    def update(self, frame_rgb, captured_at):
        self.updates += 1
        geometry = self.script.pop(0) if self.script else None
        if geometry is None:
            self.status.state = LOST
            self.status.reason = "optical flow failed"
            return TrackerReading(None, self.status, frame_rgb)
        self.status.state = TRACKING
        return TrackerReading(geometry, self.status, frame_rgb, extras={"scale": 1.0})

    def describe(self):
        return {"kind": "stub", "seeds": len(self.seeds), "updates": self.updates}


class StubDetector:
    """Stands in for AsyncDetector, with a batch the test can replace."""

    def __init__(self, *batches):
        self.batches = list(batches)
        self.submitted = 0
        self.closed = False

    def submit(self, frame_rgb, captured_at):
        self.submitted += 1
        return True

    def latest(self):
        return None

    def wait_for_first(self, timeout_s):
        return self.batches[0] if self.batches else None

    def wait_for_frame(self, after_frame_id, timeout_s):
        for batch in self.batches:
            if batch.frame_id > after_frame_id:
                return batch
        return None

    def health(self):
        return DetectorHealth(1, 0, 0, False, "", 0.01, 0.02, 0.05, False)

    def close(self):
        self.closed = True


class ScriptedDetector(StubDetector):
    """AsyncDetector stand-in whose ``latest`` the test sets directly."""

    def __init__(self):
        super().__init__()
        self.batch = None

    def latest(self):
        return self.batch

    def wait_for_first(self, timeout_s):
        return self.batch

    def wait_for_frame(self, after_frame_id, timeout_s):
        if self.batch is not None and self.batch.frame_id > after_frame_id:
            return self.batch
        return None


def batch_of(centre=(320.0, 240.0), side=60.0, captured_at=0.0, frame_id=1,
             score=0.9):
    detection = Detection((centre[0] - side / 2, centre[1] - side / 2, side, side),
                          score, "object")
    return DetectionBatch((detection,), captured_at, 0.02, frame_id)


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.support = StubSupport()
        self.detector = ScriptedDetector()
        self.tracker = FusedTracker(self.support, self.detector)

    def test_flow_is_the_per_frame_measurement(self):
        self.support.script = [box_geometry((320.0, 240.0))]
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertTrue(reading.is_fresh)
        self.assertTrue(reading.usable)

    def test_a_detection_can_stand_in_only_inside_the_coasting_window(self):
        self.support.script = [None]
        self.detector.batch = batch_of(captured_at=1.0, frame_id=1)
        coasting = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.1)
        self.assertEqual(coasting.status.state, COASTING)
        self.assertTrue(coasting.usable)
        self.assertFalse(coasting.is_fresh)  # not measured on this frame

        # The same detection, but 0.5 s later, is no longer a pose the arm may
        # move from: an unbounded coast drives at wherever the drift points.
        self.support.script = [None]
        self.detector.batch = None
        self.detector.batch = batch_of(captured_at=1.0, frame_id=1)
        lost = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.6)
        self.assertEqual(lost.status.state, LOST)
        self.assertIsNone(lost.geometry)

    def test_agreement_leaves_the_flow_alone(self):
        self.support.script = [box_geometry((320.0, 240.0), 60.0)]
        self.detector.batch = batch_of((322.0, 241.0), 60.0, 1.0, 5)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(self.tracker.describe()["reanchors"], 0)
        self.assertEqual(self.support.seeds, [])  # never re-seeded on agreement

    def test_disagreement_reanchors_onto_the_detection(self):
        # The flow has drifted 40 px: its pixels moved, but the model knows the
        # object and says otherwise. Semantics wins.
        self.support.script = [box_geometry((320.0, 240.0), 60.0)]
        self.support.seed_geometry = box_geometry((360.0, 240.0), 60.0)
        self.detector.batch = batch_of((360.0, 240.0), 60.0, 1.0, 5)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(len(self.support.seeds), 1)
        self.assertEqual(self.tracker.describe()["reanchors"], 1)
        self.assertGreaterEqual(reading.status.disagreements, 1)
        self.assertIn("60", f"{reading.geometry.extent_px:.0f}")

    def test_repeated_disagreement_stops_the_chase(self):
        # Three consecutive contradictions is not a tracking problem, it is an
        # ambiguous target. Continuing would pick one candidate at random.
        self.support.seed_geometry = box_geometry((360.0, 240.0), 60.0)
        for index in range(2):
            self.support.script = [box_geometry((320.0, 240.0), 60.0)]
            self.detector.batch = batch_of((360.0, 240.0), 60.0, 1.0, 10 + index)
            reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
            self.assertEqual(reading.status.state, TRACKING)
            self.assertEqual(self.tracker.describe()["reanchors"], index + 1)
        self.support.script = [box_geometry((320.0, 240.0), 60.0)]
        self.detector.batch = batch_of((360.0, 240.0), 60.0, 1.0, 20)
        lost = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(lost.status.state, LOST)
        self.assertIn("disagreement", lost.status.reason)

    def test_a_dead_flow_is_reacquired_from_the_detection(self):
        self.support.script = [None]
        self.support.seed_geometry = box_geometry((330.0, 250.0), 62.0)
        self.detector.batch = batch_of((330.0, 250.0), 62.0, 1.0, 4)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(self.tracker.describe()["reacquisitions"], 1)

    def test_a_detected_but_untextured_object_is_reported_as_coasting(self):
        self.support.script = [None]
        self.support.seed_geometry = None  # nothing to lock onto
        self.detector.batch = batch_of((330.0, 250.0), 62.0, 1.0, 4)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, COASTING)
        self.assertIn("textured", reading.status.reason)

    def test_one_frame_is_advanced_once_however_many_callers_consume_it(self):
        frame = np.zeros((480, 640, 3), np.uint8)
        self.support.script = [box_geometry()]
        first = self.tracker.update(frame, 1.0)
        second = self.tracker.update(frame, 1.0)
        self.assertIs(first, second)
        self.assertEqual(self.support.updates, 1)

    def test_extents_are_compared_in_one_convention(self):
        # 53 px of sqrt_area is a 60 px square, so this must read as agreement
        # rather than the disagreement a raw-number comparison would report.
        self.support.script = [box_geometry((320.0, 240.0), 60.0)]
        self.detector.batch = DetectionBatch(
            (Detection((293.5, 213.5, 53.0, 53.0), 0.9, "cup"),), 1.0, 0.02, 6)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(self.tracker.describe()["reanchors"], 0)

    def test_an_incomparable_convention_skips_the_extent_check(self):
        # A span has no shape-free square equivalent. Comparing it against a
        # side would manufacture agreement, so the check is skipped and counted.
        span = TargetGeometry.from_support_points(
            np.array([[290.0, 210.0], [350.0, 210.0], [350.0, 270.0], [290.0, 270.0]]),
            200.0, (640, 480), 1.0, "span", extent_name="span")
        self.support.script = [span]
        self.detector.batch = batch_of((320.0, 240.0), 60.0, 1.0, 7)
        reading = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(self.tracker.describe()["reanchors"], 0)
        self.assertEqual(self.tracker.describe()["convention_skips"], 1)
        self.assertIn("not comparable", reading.status.reason)


class DetectorOnlyTests(unittest.TestCase):
    """A fast landmark/blob model is a better tracker than flow on a hand."""

    def setUp(self):
        self.support = StubSupport()
        self.detector = ScriptedDetector()
        self.tracker = FusedTracker(self.support, self.detector, flow=False)

    def test_it_needs_a_detector_at_all(self):
        with self.assertRaises(ValueError):
            FusedTracker(StubSupport(), None, flow=False)

    def test_it_refuses_to_seed_from_a_region(self):
        self.detector.batch = batch_of((320.0, 240.0), captured_at=1.0, frame_id=1)
        with self.assertRaises(ValueError):
            self.tracker.start(np.zeros((480, 640, 3), np.uint8),
                               TrackerSpec(roi_px=(10.0, 10.0, 40.0, 40.0)), 1.0)

    def test_it_never_runs_the_flow_tracker(self):
        self.detector.batch = batch_of((320.0, 240.0), captured_at=1.0, frame_id=1)
        self.tracker.start(np.zeros((480, 640, 3), np.uint8),
                           TrackerSpec(prompt="hand"), 1.0)
        self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.05)
        self.assertEqual(self.support.updates, 0)
        self.assertEqual(self.support.seeds, [])

    def test_a_detector_answer_is_coasting_not_a_measurement(self):
        self.detector.batch = batch_of((320.0, 240.0), captured_at=1.0, frame_id=1)
        started = self.tracker.start(np.zeros((480, 640, 3), np.uint8),
                                     TrackerSpec(prompt="hand"), 1.0)
        self.assertEqual(started.status.state, COASTING)
        self.assertTrue(started.usable)
        self.assertFalse(started.is_fresh)

    def test_it_stops_coasting_when_the_model_stops_answering(self):
        self.detector.batch = batch_of((320.0, 240.0), captured_at=1.0, frame_id=1)
        self.tracker.start(np.zeros((480, 640, 3), np.uint8),
                           TrackerSpec(prompt="hand"), 1.0)
        lost = self.tracker.update(np.zeros((480, 640, 3), np.uint8), 1.5)
        self.assertEqual(lost.status.state, LOST)
        self.assertIsNone(lost.geometry)


class AcquisitionTests(unittest.TestCase):
    def test_a_selected_region_is_enough_with_no_detector(self):
        support = StubSupport()
        support.seed_geometry = box_geometry()
        tracker = FusedTracker(support, None)
        reading = tracker.start(np.zeros((480, 640, 3), np.uint8),
                                TrackerSpec(roi_px=(290.0, 210.0, 60.0, 60.0)), 0.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(len(support.seeds), 1)

    def test_no_region_and_no_detector_is_an_explicit_loss(self):
        tracker = FusedTracker(StubSupport(), None)
        reading = tracker.start(np.zeros((480, 640, 3), np.uint8),
                                TrackerSpec(prompt="the cup"), 0.0)
        self.assertEqual(reading.status.state, LOST)
        self.assertIn("no detector", reading.status.reason)

    def test_a_prompt_is_acquired_from_the_detector(self):
        support = StubSupport()
        support.seed_geometry = box_geometry((300.0, 200.0), 50.0)
        tracker = FusedTracker(support, StubDetector(batch_of((300.0, 200.0), 50.0)))
        reading = tracker.start(np.zeros((480, 640, 3), np.uint8),
                                TrackerSpec(prompt="the cup", target_id="cup"), 0.0)
        self.assertEqual(reading.status.state, TRACKING)
        self.assertEqual(len(support.seeds), 1)
        # The seed region is grown around the detection, not the exact box: a
        # tight box on a smooth object yields almost no corners to track.
        left, top, width, height = support.seeds[0][0]
        self.assertLess(left, 300.0 - 25.0)
        self.assertGreater(width, 50.0)

    def test_a_spec_with_no_selection_channel_is_refused(self):
        with self.assertRaises(ValueError):
            TrackerSpec().validate()


class RegionTests(unittest.TestCase):
    def test_the_seed_region_grows_and_is_clipped_to_the_frame(self):
        geometry = TargetGeometry.from_box((600.0, 440.0, 30.0, 30.0), (640, 480), 0.0, "t")
        left, top, width, height = roi_from_geometry(geometry, 1.5, (640, 480))
        self.assertGreaterEqual(left, 0.0)
        self.assertLessEqual(left + width, 640.0)
        self.assertLessEqual(top + height, 480.0)

    def test_a_degenerate_point_still_gets_a_region(self):
        geometry = TargetGeometry.from_circle((100.0, 100.0), 1.0, (640, 480), 0.0, "t")
        _, _, width, height = roi_from_geometry(geometry, 1.25, (640, 480))
        self.assertGreaterEqual(width, 6.0)
        self.assertGreaterEqual(height, 6.0)

    def test_an_off_frame_geometry_is_refused(self):
        # The geometry layer already rejects leaving the frame, so the region
        # builder only has to handle the clipped case; assert the guard exists.
        with self.assertRaises(ValueError):
            TargetGeometry.from_support_points(
                np.array([[700.0, 700.0]]), 10.0, (640, 480), 0.0, "t")

    def test_a_target_pinning_the_image_edge_still_gets_a_usable_region(self):
        geometry = TargetGeometry.from_support_points(
            np.array([[636.0, 476.0]]), 8.0, (640, 480), 0.0, "t")
        left, top, width, height = roi_from_geometry(geometry, 1.25, (640, 480))
        self.assertGreaterEqual(width, 4.0)
        self.assertGreaterEqual(height, 4.0)
        self.assertLessEqual(left + width, 640.0)
        self.assertLessEqual(top + height, 480.0)

    def test_a_shrinking_margin_is_refused(self):
        geometry = TargetGeometry.from_box((10.0, 10.0, 30.0, 30.0), (640, 480), 0.0, "t")
        with self.assertRaises(ValueError):
            roi_from_geometry(geometry, 0.5, (640, 480))


class SupportPointTrackerTests(unittest.TestCase):
    def scene(self, centre_x, centre_y, half=40):
        frame = np.full((480, 640, 3), 210, np.uint8)
        step = max(6, int(2 * half / 5))
        for i in range(-2, 3):
            for j in range(-2, 3):
                x, y = centre_x + i * step, centre_y + j * step
                cv2.rectangle(frame, (int(x - 2), int(y - 2)),
                              (int(x + 2), int(y + 2)), (30, 30, 30), -1)
        return frame

    def test_it_follows_a_translating_texture_and_tracks_its_extent(self):
        tracker = SupportPointTracker()
        started = tracker.start(self.scene(300, 240),
                                TrackerSpec(roi_px=(240.0, 180.0, 120.0, 120.0)), 0.0)
        self.assertEqual(started.status.state, TRACKING)
        centres, extents = [], []
        for index in range(1, 5):
            reading = tracker.update(self.scene(300 + 4 * index, 240), index * 0.066)
            self.assertEqual(reading.status.state, TRACKING)
            centres.append(reading.geometry.center_px[0])
            extents.append(reading.geometry.extent_px)
        self.assertEqual(centres, sorted(centres))
        self.assertGreater(centres[-1], centres[0] + 10.0)
        self.assertGreater(max(extents), min(extents))

    def test_a_frame_gap_loses_the_target_instead_of_jumping(self):
        tracker = SupportPointTracker(max_gap_s=0.1)
        tracker.start(self.scene(300, 240),
                      TrackerSpec(roi_px=(240.0, 180.0, 120.0, 120.0)), 0.0)
        reading = tracker.update(self.scene(300, 240), 0.5)
        self.assertEqual(reading.status.state, LOST)
        self.assertIn("gap", reading.status.reason)

    def test_a_region_with_no_texture_is_reported_not_guessed(self):
        tracker = SupportPointTracker()
        blank = np.full((480, 640, 3), 128, np.uint8)
        reading = tracker.start(blank, TrackerSpec(roi_px=(100.0, 100.0, 80.0, 80.0)), 0.0)
        self.assertEqual(reading.status.state, LOST)
        self.assertIsNone(reading.geometry)


class TrackerToServoTests(unittest.TestCase):
    def setUp(self):
        self.controller = FixedRuleController(
            None, np.eye(4), FixedRuleConfig(), np.ones(5) * 10.0, np.eye(3),
            reference_degrees=np.zeros(5), origin_m=np.zeros(3))

    def observation(self, extent_name, extent=60.0):
        geometry = TargetGeometry.from_support_points(
            np.array([[290.0, 210.0], [350.0, 210.0], [350.0, 270.0], [290.0, 270.0]]),
            extent, (640, 480), 0.0, "object", extent_name=extent_name)
        return Observation.from_geometry(geometry, np.zeros(5))

    def test_the_observation_carries_the_trackers_extent_not_a_recomputation(self):
        observation = self.observation("sqrt_area", extent=53.0)
        self.assertAlmostEqual(observation.side_px, 53.0)
        self.assertEqual(observation.extent_name, "sqrt_area")

    def test_a_paired_scale_produces_a_depth(self):
        self.controller.scale = ExtentScale.from_reference(0.30, 60.0, "side")
        observation = self.observation("side", extent=60.0)
        self.assertAlmostEqual(self.controller.depth_m(observation), 0.30, places=9)

    def test_a_mismatched_convention_holds_before_motion(self):
        # The tracker switched to sqrt_area but the scale was measured on a side:
        # the depth would be off by the object's aspect ratio, so the rule must
        # refuse to step rather than step on a plausible wrong number.
        self.controller.scale = ExtentScale.from_reference(0.30, 60.0, "side")
        action = self.controller.decide(self.observation("sqrt_area", extent=60.0))
        self.assertEqual(action.kind, "hold")
        self.assertIn("convention", action.reason)
        self.assertFalse(action.movable)


if __name__ == "__main__":
    unittest.main()
