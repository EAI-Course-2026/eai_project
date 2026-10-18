"""Perception infrastructure: specs, registry, async worker, detector adapters.

The point of these tests is that a machine with no optional model installed must
still be able to import the package, run every control test, and be told exactly
what to install when it asks for a detector it does not have. A
``ModuleNotFoundError`` would technically satisfy "does not crash on import" but
is useless at the bench, so the message is asserted, not just the exception type.
"""
import threading
import time
import unittest
from pathlib import Path

import cv2
import numpy as np

from eai_robot.config import ROOT
from eai_robot.vision.geometry.targets import TargetGeometry
from eai_robot.vision.perception import registry
from eai_robot.vision.perception.async_detector import AsyncDetector
from eai_robot.vision.perception.builtin import DarkBlobDetector
from eai_robot.vision.perception.detector import (
    Detection, DetectionBatch, ObjectDetector, SELECTION_POLICIES, select_detection,
)
from eai_robot.vision.perception.spec import (
    DEVICE_CHOICES, ModelSpec, available_devices, resolve_device,
)
from eai_robot.vision.tracking.base import (
    COASTING, TRACKING, TrackerReading, TrackerStatus, TrackerUnavailable,
)


class GatedDetector:
    """ObjectDetector whose latency and failure mode the test controls."""

    name = "gated"
    latency_budget_s = 0.05

    def __init__(self, detections=(), failure=""):
        self.gate = threading.Event()
        self.gate.set()
        self.entered = threading.Event()
        self.detections = list(detections)
        self.failure = failure
        self.seen = []
        self.closed = False

    def detect(self, frame_rgb):
        self.entered.set()
        self.gate.wait(timeout=5.0)
        self.seen.append(int(np.asarray(frame_rgb).reshape(-1)[0]))
        if self.failure:
            raise RuntimeError(self.failure)
        return list(self.detections)

    def describe(self):
        return {"name": self.name, "device": "cpu", "weights": None}

    def close(self):
        self.closed = True


def frame(value: int) -> np.ndarray:
    return np.full((8, 8, 3), value, dtype=np.uint8)


def wait_until(predicate, timeout_s=5.0, interval_s=0.005):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return False


class ModelSpecTests(unittest.TestCase):
    def test_devices_are_enumerated_with_cpu_always_present(self):
        self.assertIn("cpu", available_devices())
        self.assertIn("cpu", DEVICE_CHOICES)

    def test_cpu_is_always_resolvable(self):
        self.assertEqual(resolve_device("cpu"), "cpu")

    def test_auto_resolves_to_an_available_device(self):
        self.assertIn(resolve_device("auto"), available_devices())

    def test_an_unavailable_device_is_refused_rather_than_silently_replaced(self):
        missing = next((name for name in ("cuda", "mps")
                        if name not in available_devices()), None)
        if missing is None:
            self.skipTest("every optional device is available here")
        with self.assertRaises(RuntimeError):
            resolve_device(missing)

    def test_unknown_device_is_refused(self):
        with self.assertRaises(ValueError):
            ModelSpec("dark-blob", device="tpu")

    def test_a_missing_weight_file_is_an_actionable_error(self):
        # Implicit downloads break both offline benches and reproducibility, so
        # the default is to refuse and say which path was expected.
        missing = Path("/nonexistent/weights/owlv2.bin")
        with self.assertRaises(TrackerUnavailable) as caught:
            ModelSpec("owlv2", weights=missing).validate_weights()
        self.assertIn(str(missing), str(caught.exception))

    def test_allow_download_makes_the_same_spec_acceptable(self):
        spec = ModelSpec("owlv2", weights=Path("/nonexistent/w.bin"),
                         allow_download=True)
        spec.validate_weights()  # must not raise

    def test_resolved_records_the_concrete_device(self):
        spec = ModelSpec("owlv2", device="auto")
        self.assertIn(spec.resolved().device, available_devices())
        self.assertEqual(spec.device, "auto")  # the spec itself is unchanged

    def test_describe_carries_the_identity_of_the_loaded_bytes(self):
        spec = ModelSpec("sam2", weights=Path("/w/sam2.pt"), device="cpu",
                         input_size=(1024, 1024))
        payload = spec.describe()
        self.assertEqual(payload["name"], "sam2")
        self.assertEqual(payload["device"], "cpu")
        self.assertEqual(payload["input_size"], [1024, 1024])


class RegistryTests(unittest.TestCase):
    def test_the_catalogue_names_every_candidate(self):
        self.assertEqual(set(registry.available()),
                         {"dark-blob", "mediapipe-hands", "owlv2", "sam2",
                          "yolo-world"})

    def test_builtin_detector_is_ready_with_no_extra(self):
        self.assertTrue(registry.is_registered("dark-blob"))
        detector = registry.create(ModelSpec("dark-blob"))
        self.assertEqual(detector.name, "dark-blob")

    def test_a_catalogued_but_missing_detector_explains_how_to_get_it(self):
        with self.assertRaises(TrackerUnavailable) as caught:
            registry.create(ModelSpec("sam2"))
        message = str(caught.exception)
        # SAM 2 is distributed only from git, so the message has to carry the
        # URL itself rather than name an extra the lockfile could resolve.
        self.assertIn("facebookresearch/sam2", message)
        # The message must say where to plug the implementation in, otherwise
        # the reader knows the dependency but not the contract.
        self.assertIn("register('sam2', factory)", message)
        self.assertIn("ObjectDetector", message)

    def test_an_unknown_name_lists_what_is_known(self):
        with self.assertRaises(TrackerUnavailable) as caught:
            registry.create(ModelSpec("gpt-vision"))
        self.assertIn("dark-blob", str(caught.exception))

    def test_registering_an_uncatalogued_name_is_refused(self):
        with self.assertRaises(ValueError):
            registry.register("mystery", lambda spec: DarkBlobDetector())

    def test_registration_is_what_decides_availability(self):
        entry = registry.available()["mediapipe-hands"]
        self.assertFalse(entry["implemented"])
        self.assertIn("mediapipe", entry["install"])
        self.assertFalse(entry["needs_weights"])

    def test_every_catalogue_entry_states_its_evidence_and_budget(self):
        for name, entry in registry.available().items():
            with self.subTest(detector=name):
                self.assertIn(entry["evidence"], ("mask", "keypoints", "box"))
                self.assertGreater(entry["latency_budget_s"], 0.0)
                self.assertTrue(entry["summary"])

    def test_model_families_stay_out_of_the_lockfile(self):
        """An extra that cannot resolve breaks ``uv run --locked`` for everyone.

        SAM 2 is distributed only from git and MediaPipe brings its own NumPy,
        so declaring either in ``[project.optional-dependencies]`` makes the
        whole workspace resolution fail -- and with it the servo loop, on
        machines that will never load a model. The exact commands live in the
        catalogue instead, where a failing install stays local to the person
        who asked for the detector.
        """
        import tomllib
        project = tomllib.loads((ROOT / "pyproject.toml").read_text())["project"]
        declared = " ".join(
            f"{name} {' '.join(requirements)}"
            for name, requirements in project.get("optional-dependencies", {}).items()
        ).lower()
        for family in ("mediapipe", "segment-anything", "sam2", "ultralytics",
                       "transformers"):
            self.assertNotIn(family, declared,
                             f"{family} must stay out of the lockfile")
        # And every catalogued detector still says what to install.
        for name, entry in registry.available().items():
            self.assertTrue(entry["install"], name)


class DetectionTests(unittest.TestCase):
    def test_mask_evidence_wins_over_the_box(self):
        mask = np.zeros((40, 40), dtype=bool)
        mask[10:30, 10:30] = True
        detection = Detection((0.0, 0.0, 40.0, 40.0), 0.9, "cup", mask=mask)
        geometry = detection.to_geometry((640, 480), 1.0, "cup")
        self.assertEqual(geometry.extent_name, "sqrt_area")
        self.assertAlmostEqual(geometry.extent_px, 20.0)

    def test_keypoints_become_a_square_equivalent_quad(self):
        points = np.array([[10.0, 10.0], [50.0, 10.0], [50.0, 50.0], [10.0, 50.0]])
        detection = Detection((10.0, 10.0, 40.0, 40.0), 0.8, "hand",
                              keypoints_px=points)
        geometry = detection.to_geometry((640, 480), 1.0, "hand")
        self.assertEqual(geometry.extent_name, "side")
        self.assertAlmostEqual(geometry.extent_px, 40.0, places=6)

    def test_a_box_only_detection_uses_the_geometric_mean(self):
        geometry = Detection((0.0, 0.0, 20.0, 80.0), 0.5, "box").to_geometry(
            (640, 480), 1.0, "box")
        self.assertEqual(geometry.extent_name, "sqrt_area")
        self.assertAlmostEqual(geometry.extent_px, 40.0)

    def test_an_empty_mask_is_an_error_not_a_silent_zero(self):
        with self.assertRaises(ValueError):
            Detection((0.0, 0.0, 10.0, 10.0), 0.5,
                      mask=np.zeros((10, 10), dtype=bool)).to_geometry(
                (640, 480), 1.0, "t")

    def test_selection_policies_are_deterministic(self):
        candidates = [Detection((0.0, 0.0, 10.0, 10.0), 0.4, "a"),
                      Detection((300.0, 220.0, 20.0, 20.0), 0.9, "b"),
                      Detection((100.0, 100.0, 60.0, 60.0), 0.5, "c")]
        self.assertEqual(select_detection(candidates, "score").label, "b")
        self.assertEqual(select_detection(candidates, "area").label, "c")
        self.assertEqual(
            select_detection(candidates, "central", (640, 480)).label, "b")

    def test_no_candidates_is_none_not_an_exception(self):
        self.assertIsNone(select_detection([], "score"))

    def test_unknown_policy_is_refused(self):
        with self.assertRaises(ValueError):
            select_detection([Detection((0.0, 0.0, 1.0, 1.0), 1.0)], "biggest")
        self.assertEqual(set(SELECTION_POLICIES), {"score", "area", "central"})


class DarkBlobDetectorTests(unittest.TestCase):
    def test_a_dark_disc_becomes_a_mask_backed_detection(self):
        scene = np.full((480, 640, 3), 210, dtype=np.uint8)
        cv2.circle(scene, (320, 240), 30, (20, 20, 20), -1)
        detector = DarkBlobDetector()
        found = detector.detect(scene)
        self.assertEqual(len(found), 1)
        self.assertIsNotNone(found[0].mask)
        geometry = found[0].to_geometry((640, 480), 1.0, "ball")
        self.assertAlmostEqual(geometry.center_px[0], 320.0, delta=2.0)
        # sqrt(pi * 30^2) = 53 px for a disc of radius 30, in sqrt_area terms.
        self.assertAlmostEqual(geometry.extent_px, 53.0, delta=3.0)

    def test_an_empty_scene_returns_no_candidates(self):
        self.assertEqual(DarkBlobDetector().detect(
            np.full((480, 640, 3), 210, dtype=np.uint8)), [])

    def test_a_closed_detector_refuses_to_run(self):
        detector = DarkBlobDetector()
        detector.close()
        with self.assertRaises(RuntimeError):
            detector.detect(np.zeros((10, 10, 3), dtype=np.uint8))

    def test_the_detector_satisfies_the_protocol(self):
        self.assertIsInstance(DarkBlobDetector(), ObjectDetector)


class AsyncDetectorTests(unittest.TestCase):
    def test_one_slot_keeps_the_newest_frame_and_counts_the_loss(self):
        inner = GatedDetector()
        inner.gate.clear()  # hold the worker so frames pile up
        worker = AsyncDetector(inner)
        try:
            worker.submit(frame(1), 1.0)
            # Wait until the worker is inside the model, so the next submit is
            # known to land in the single slot rather than racing the pickup.
            self.assertTrue(inner.entered.wait(2.0))
            self.assertTrue(worker.submit(frame(2), 2.0))
            self.assertFalse(worker.submit(frame(3), 3.0))
            inner.gate.set()
            batch = worker.wait_for_first(2.0)
            self.assertIsNotNone(batch)
            health = worker.health()
            self.assertEqual(health.dropped, 1)
            # Frame 2 was never processed: latest-only means the model never
            # works on a backlog, so latency stays equal to model latency.
            self.assertEqual(inner.seen, [1, 3])
        finally:
            worker.close()

    def test_the_batch_carries_the_frame_timestamp_not_the_finish_time(self):
        inner = GatedDetector([Detection((0.0, 0.0, 10.0, 10.0), 0.9, "x")])
        worker = AsyncDetector(inner)
        try:
            worker.submit(frame(1), 123.5)
            batch = worker.wait_for_first(2.0)
            self.assertEqual(batch.captured_at, 123.5)
            self.assertGreater(batch.latency_s, 0.0)
        finally:
            worker.close()

    def test_a_model_crash_is_data_not_a_control_loop_exception(self):
        inner = GatedDetector(failure="weights unreadable")
        worker = AsyncDetector(inner, failure_limit=2)
        try:
            for index in range(3):
                worker.submit(frame(index), float(index))
                wait_until(lambda: worker.health().processed > index)
            health = worker.health()
            self.assertGreaterEqual(health.failures, 2)
            self.assertIn("weights unreadable", health.last_error)
            self.assertTrue(health.degraded)
            self.assertIsNone(worker.latest())
        finally:
            worker.close()

    def test_recovery_clears_degraded_and_publishes_again(self):
        inner = GatedDetector([Detection((0.0, 0.0, 10.0, 10.0), 0.9, "x")],
                              failure="transient")
        worker = AsyncDetector(inner, failure_limit=2)
        try:
            worker.submit(frame(1), 1.0)
            wait_until(lambda: worker.health().processed >= 1)
            self.assertGreaterEqual(worker.health().failures, 1)
            inner.failure = ""
            worker.submit(frame(2), 2.0)
            self.assertIsNotNone(worker.wait_for_first(2.0))
            health = worker.health()
            self.assertEqual(health.failures, 0)
            self.assertFalse(health.degraded)
        finally:
            worker.close()

    def test_wait_for_frame_only_returns_a_newer_batch(self):
        inner = GatedDetector([Detection((0.0, 0.0, 10.0, 10.0), 0.9, "x")])
        worker = AsyncDetector(inner)
        try:
            worker.submit(frame(1), 1.0)
            first = worker.wait_for_first(2.0)
            self.assertIsNone(worker.wait_for_frame(first.frame_id, 0.05))
            worker.submit(frame(2), 2.0)
            second = worker.wait_for_frame(first.frame_id, 2.0)
            self.assertGreater(second.frame_id, first.frame_id)
        finally:
            worker.close()

    def test_close_is_idempotent_and_releases_the_model(self):
        inner = GatedDetector()
        worker = AsyncDetector(inner)
        worker.close()
        worker.close()
        self.assertTrue(inner.closed)
        self.assertFalse(worker.submit(frame(9), 9.0))

    def test_an_object_that_is_not_a_detector_is_refused(self):
        with self.assertRaises(TypeError):
            AsyncDetector(object())


class ObjectRunnerWiringTests(unittest.TestCase):
    """The object runner's own glue: option parsing, scale choice, adapter."""

    def args(self, **overrides):
        import argparse
        defaults = dict(
            camera_fps=15, tracker_min_inliers=8, tracker="fused", prompt=None,
            label=None, roi=None, detector="dark-blob", detector_weights=None,
            model_device="cpu", model_dtype="float32",
            allow_weight_download=False, detector_option=None,
            detector_failure_limit=3, selection="score",
            tracker_max_coast_s=0.2, tracker_agreement=0.35,
            detector_timeout_s=2.0, tracker_seed_margin=1.25,
            reference_depth_m=None, target_extent_m=None)
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_detector_options_pass_through_as_strings(self):
        from eai_robot.applications.visual_servo.object_servo import detector_options
        self.assertEqual(detector_options(["black_threshold=90", "min_area_ratio=0.01"]),
                         {"black_threshold": "90", "min_area_ratio": "0.01"})
        self.assertEqual(detector_options(None), {})

    def test_a_malformed_option_is_refused(self):
        from eai_robot.applications.visual_servo.object_servo import detector_options
        for bad in (["black_threshold"], ["=90"], [" = "]):
            with self.subTest(option=bad):
                with self.assertRaises(ValueError):
                    detector_options(bad)

    def test_the_reference_distance_route_needs_one_number(self):
        from eai_robot.applications.visual_servo.object_servo import resolve_scale
        geometry = TargetGeometry.from_corners(
            np.array([[10.0, 10.0], [70.0, 10.0], [70.0, 70.0], [10.0, 70.0]]),
            (640, 480), 0.0, "obj")
        scale = resolve_scale(self.args(reference_depth_m=0.35), geometry, 554.0)
        self.assertAlmostEqual(scale.depth_m(geometry.extent_px), 0.35, places=9)
        self.assertEqual(scale.extent_name, "side")

    def test_the_real_size_route_uses_the_calibrated_focal_length(self):
        from eai_robot.applications.visual_servo.object_servo import resolve_scale
        geometry = TargetGeometry.from_box((10.0, 10.0, 40.0, 40.0), (640, 480), 0.0, "obj")
        scale = resolve_scale(self.args(target_extent_m=0.05), geometry, 554.0)
        self.assertAlmostEqual(scale.constant_m_px, 554.0 * 0.05, places=9)
        self.assertEqual(scale.extent_name, "sqrt_area")

    def test_two_or_no_measurements_are_both_refused(self):
        from eai_robot.applications.visual_servo.object_servo import resolve_scale
        geometry = TargetGeometry.from_box((0.0, 0.0, 10.0, 10.0), (640, 480), 0.0, "obj")
        for overrides in (dict(reference_depth_m=0.3, target_extent_m=0.05), {}):
            with self.subTest(overrides=overrides):
                with self.assertRaises(ValueError):
                    resolve_scale(self.args(**overrides), geometry, 554.0)

    def test_the_tracker_kind_decides_what_is_required(self):
        from eai_robot.applications.visual_servo.object_servo import build_tracker
        with self.assertRaises(TrackerUnavailable):
            build_tracker(self.args(tracker="support", prompt="cup"))
        with self.assertRaises(TrackerUnavailable):
            build_tracker(self.args(tracker="support"))
        with self.assertRaises(TrackerUnavailable):
            build_tracker(self.args(tracker="detector"))

    def test_a_region_only_run_needs_no_model_at_all(self):
        from eai_robot.applications.visual_servo.object_servo import build_tracker
        tracker = build_tracker(self.args(tracker="support", roi="10,10,40,40"))
        self.assertIsNone(tracker.detector)
        tracker.close()

    def test_the_adapter_refuses_a_stale_pose(self):
        from eai_robot.applications.visual_servo.object_servo import TrackerDetectionAdapter
        from eai_robot.vision.tracking.base import TrackerStatus

        class FixedTracker:
            def __init__(self, reading):
                self.reading = reading

            def update(self, frame_rgb, captured_at):
                return self.reading

        geometry = TargetGeometry.from_box((100.0, 100.0, 40.0, 40.0), (640, 480),
                                           0.0, "obj")
        reading = TrackerReading(geometry, TrackerStatus(state=COASTING,
                                                         reason="coasted"), None)
        adapter = TrackerDetectionAdapter(FixedTracker(reading), max_age_s=0.25)
        detected, mask = adapter.detect(np.zeros((480, 640, 3), np.uint8))
        self.assertIsNone(detected)          # 0.0 s stamp is stale "now"
        self.assertEqual(mask.shape, (480, 640))
        self.assertIn("stale", adapter.last_reason)

    def test_the_adapter_reports_a_fresh_pose_as_an_annotation(self):
        from eai_robot.applications.visual_servo.object_servo import TrackerDetectionAdapter
        from eai_robot.vision.tracking.base import TrackerStatus

        class FixedTracker:
            def __init__(self, reading):
                self.reading = reading

            def update(self, frame_rgb, captured_at):
                return self.reading

        geometry = TargetGeometry.from_box((100.0, 100.0, 40.0, 40.0), (640, 480),
                                          time.monotonic(), "obj")
        reading = TrackerReading(geometry, TrackerStatus(state=TRACKING), None)
        adapter = TrackerDetectionAdapter(FixedTracker(reading))
        detected, mask = adapter.detect(np.zeros((480, 640, 3), np.uint8))
        self.assertIsNotNone(detected)
        self.assertAlmostEqual(detected.center_x, 120.0, places=6)
        self.assertGreater(int(mask.sum()), 0)
        self.assertEqual(adapter.last_reason, "")


class BatchTests(unittest.TestCase):
    def test_batch_age_is_never_negative(self):
        batch = DetectionBatch((), time.monotonic() + 5.0, 0.1, 1)
        self.assertEqual(batch.age_s, 0.0)

    def test_describe_is_json_safe(self):
        batch = DetectionBatch((Detection((1.0, 2.0, 3.0, 4.0), 0.5, "x"),),
                               time.monotonic(), 0.2, 7)
        payload = batch.describe()
        self.assertEqual(payload["count"], 1)
        self.assertEqual(payload["frame_id"], 7)


if __name__ == "__main__":
    unittest.main()
