"""The object runner end to end, against fake devices and a projected target.

Pixels are already covered by ``test_fused_tracker``: the flow tracker was shown
to follow a translating texture and the fusion state machine was shown to
arbitrate. What is untested until now is the *wiring* -- tracker assembly, the
reference-distance scale, the motion guard adapter, the log, and above all
whether the loop makes progress instead of stalling on a boundary.

So this file replaces only two things: the motor bus and the tracker. The
tracker reports geometry projected from the bus's measured pose through the real
URDF, camera mount and pinhole matrix, which is the same scene the ArUco runner
is tested against. Everything between them -- decision, guard, attribution,
record -- is the code that runs on the arm.
"""
import json
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from eai_robot.vision.geometry.targets import TargetGeometry
from eai_robot.vision.tracking.base import (
    TRACKING, TrackerReading, TrackerSpec, TrackerStatus,
)

from test_fixed_rule_servo import (
    K, MARKER_M, RAW, FakeBus, FakeRuntime, Scene, load_fixtures, runner_args,
)


def object_args(**overrides):
    """Runner arguments for the object path, built on the marker runner's set."""
    values = dict(
        tracker="support", roi="280,200,80,80", prompt=None, label=None,
        target_id="object:test", detector="dark-blob", detector_weights=None,
        detector_option=None, model_device="cpu", model_dtype="float32",
        allow_weight_download=False, selection="score",
        tracker_max_coast_s=0.2, tracker_max_age_s=0.25, tracker_agreement=0.35,
        tracker_min_inliers=8, tracker_seed_margin=1.25,
        tracker_reacquire_attempts=2, detector_timeout_s=2.0,
        detector_failure_limit=3, camera_fps=15, reference_depth_m=None,
        target_extent_m=None,
    )
    values.update(overrides)
    return runner_args(**values)


class PoseTracker:
    """Tracker whose geometry is projected from the measured arm pose.

    It reports the same shape of reading as ``FusedTracker`` (status, geometry,
    no extras) so the runner, the guard adapter and the summary printer all see
    the interface they see in production, while the geometry still comes from
    the real FK chain rather than from a hard-coded pixel.
    """

    def __init__(self, scene, mapper, bus, *, extent_name="side", lose=False,
                 flip_after=None):
        self.scene, self.mapper, self.bus = scene, mapper, bus
        self.target_id = "object:test"
        self.status = TrackerStatus(state=TRACKING)
        self.detector = None
        self.extent_name = extent_name
        self.lose = lose
        self.flip_after = flip_after
        self.starts = 0
        self.updates = 0
        self.closed = False

    def _geometry(self, captured_at):
        degrees = np.asarray([self.mapper.raw_to_urdf_degrees(name, self.bus.present[name])
                              for name in self.bus.present], dtype=float)
        observation = self.scene.observe(degrees)
        corners = np.asarray(observation.corners_px, dtype=float)
        if self.extent_name == "side":
            return TargetGeometry.from_corners(corners, (640, 480), captured_at,
                                               self.target_id, source="sim")
        centre = corners.mean(axis=0)
        side = float(np.linalg.norm(corners[1] - corners[0]))
        return TargetGeometry.from_support_points(
            corners, side, (640, 480), captured_at, self.target_id,
            source="sim", extent_name=self.extent_name)

    def start(self, frame_rgb, spec, captured_at):
        spec.validate()
        self.starts += 1
        if self.lose:
            self.status.state = "lost"
            return TrackerReading(None, self.status, frame_rgb)
        return TrackerReading(self._geometry(captured_at), self.status, frame_rgb)

    def update(self, frame_rgb, captured_at):
        self.updates += 1
        if self.lose:
            self.status.state = "lost"
            return TrackerReading(None, self.status, frame_rgb)
        if self.flip_after is not None and self.updates > self.flip_after:
            # A model that starts returning a mask mid-run switches the evidence
            # from a box to a mask, i.e. from sqrt_area to a different number for
            # the same object. This is the reachable form of the pairing hazard.
            self.extent_name = "sqrt_area"
        return TrackerReading(self._geometry(captured_at), self.status, frame_rgb)

    def reset(self, reason=""):
        self.status.reset(reason)

    def describe(self):
        return {"kind": "pose", "starts": self.starts, "updates": self.updates,
                **self.status.describe()}

    def close(self):
        self.closed = True


class ObjectRunnerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        cls.names = ARM_JOINT_NAMES
        calibration, cls.mount = load_fixtures()
        cls.mapper = SO101JointMapper(calibration)
        cls.start = np.array([cls.mapper.raw_to_urdf_degrees(name, RAW[name])
                              for name in ARM_JOINT_NAMES])

    def scene(self, depth_m=0.30):
        return Scene(self.start, self.mount, depth_m=depth_m)

    def run_runner(self, tracker=None, args=None, bus=None, scene=None):
        from eai_robot.applications.visual_servo import object_servo
        scene = scene or self.scene()
        bus = bus or FakeBus(RAW)
        args = args or object_args(reference_depth_m=0.30, rule_max_steps=3,
                                   max_travel_mm=60.0,
                                   max_joint_excursion_deg=40.0)
        runtime = FakeRuntime(scene.fk, self.mapper, bus)
        tracker = tracker or PoseTracker(scene, self.mapper, bus)
        stop = types.SimpleNamespace(stop_requested=threading.Event())
        directory = tempfile.mkdtemp()
        with patch.object(object_servo, "ROOT", Path(directory)), \
                patch.object(object_servo, "build_tracker", lambda a: tracker):
            result = object_servo.run_object_rule(runtime, args, stop, None)
        logs = sorted((Path(directory) / "local_artifacts/experiments").glob("*.jsonl"))
        events = [json.loads(line) for line in logs[-1].read_text().splitlines()]
        return result, runtime, bus, events, tracker

    # -- the loop ---------------------------------------------------------
    def test_the_loop_acquires_decides_and_attributes_each_step(self):
        result, runtime, bus, events, tracker = self.run_runner()
        kinds = [event["event"] for event in events]
        self.assertEqual(result, 0)
        self.assertIn("settings", kinds)
        self.assertIn("acquired", kinds)
        self.assertIn("observation", kinds)
        self.assertIn("plan", kinds)
        self.assertIn("feedback", kinds)
        self.assertEqual(tracker.starts, 1)
        self.assertGreaterEqual(tracker.updates, 3)
        self.assertTrue(tracker.closed)

    def test_the_acquired_record_states_the_convention_and_the_scale(self):
        # Without these two fields a depth in the log cannot be reproduced or
        # even checked: the same object gives a different number per convention.
        _, _, _, events, _ = self.run_runner()
        acquired = [event for event in events if event["event"] == "acquired"]
        self.assertEqual(len(acquired), 1)
        self.assertEqual(acquired[0]["extent_name"], "side")
        # k = depth x extent = 0.30 m x 70.4 px, the round-1 recorded side.
        self.assertAlmostEqual(acquired[0]["scale"]["constant_m_px"], 21.1, delta=0.6)

    def test_a_reference_depth_reproduces_the_true_distance(self):
        # The scene puts the marker at 300 mm. A scale measured at acquisition
        # must read 300 mm back out of the apparent extent, to within the
        # projection arithmetic.
        _, _, _, events, _ = self.run_runner()
        observation = [event for event in events if event["event"] == "observation"][0]
        self.assertAlmostEqual(observation["depth_m"], 0.30, delta=0.01)

    def test_the_bus_actually_moved_and_the_log_says_by_how_much(self):
        _, _, bus, events, _ = self.run_runner()
        self.assertTrue(bus.writes)
        feedback = [event for event in events if event["event"] == "feedback"]
        self.assertTrue(feedback)
        self.assertTrue(any(event["actual_counts"] for event in feedback))

    def test_the_travel_budget_stops_the_run_with_a_reason(self):
        args = object_args(reference_depth_m=0.30, rule_max_steps=40,
                           max_travel_mm=20.0, max_joint_excursion_deg=40.0)
        result, runtime, _, events, _ = self.run_runner(args=args)
        self.assertEqual(result, 0)
        holds = [event for event in events if event["event"] == "hold"]
        self.assertTrue(holds)
        self.assertIn("travel budget", holds[-1]["reason"])

    def test_the_step_budget_stops_the_run(self):
        args = object_args(reference_depth_m=0.30, rule_max_steps=1,
                           max_travel_mm=60.0)
        _, _, _, events, _ = self.run_runner(args=args)
        # One plan per executed step. The bus sees two writes per step -- the
        # goal and the guard's settled hold -- so counting plans is the honest
        # measure of how much motion was authorized.
        self.assertEqual(len([e for e in events if e["event"] == "plan"]), 1)
        self.assertIn("summary", [event["event"] for event in events])

    # -- conventions ------------------------------------------------------
    def test_a_tracker_that_switches_convention_holds_before_any_goal_write(self):
        # The scale is fixed at acquisition, against the convention the tracker
        # used then. If the tracker later starts reporting sqrt_area, every depth
        # would be off by the object's aspect ratio, so the rule must hold rather
        # than step on a plausible wrong number.
        scene = self.scene()
        bus = FakeBus(RAW)
        tracker = PoseTracker(scene, self.mapper, bus, flip_after=3)
        args = object_args(reference_depth_m=0.30, rule_max_steps=4,
                           max_travel_mm=60.0, feature_samples=1)
        _, _, bus, events, _ = self.run_runner(
            tracker=tracker, args=args, bus=bus, scene=scene)
        holds = [event for event in events if event["event"] == "hold"]
        self.assertTrue(holds)
        self.assertIn("convention", holds[-1]["reason"])
        # Whatever was planned was planned before the switch was observed; the
        # run must not keep stepping after the mismatch is known.
        self.assertLessEqual(len([e for e in events if e["event"] == "plan"]), 2)

    def test_a_tracker_that_loses_the_target_holds_without_moving(self):
        scene = self.scene()
        bus = FakeBus(RAW)
        tracker = PoseTracker(scene, self.mapper, bus, lose=True)
        result, runtime, _, events, _ = self.run_runner(
            tracker=tracker, scene=scene, bus=bus)
        self.assertEqual(result, 0)
        self.assertIn("acquire_failed", [event["event"] for event in events])
        self.assertEqual(bus.writes, [])
        self.assertIn("acquisition failed", runtime.hold_reason)


class ObjectRunnerArgumentsTest(unittest.TestCase):
    """What the CLI refuses, and why the refusal is a sentence."""

    def parse(self, argv):
        import sys
        from unittest.mock import patch as mock_patch
        from eai_robot.applications.visual_servo.ball import parse_args
        with mock_patch.object(sys, "argv", ["vision"] + argv):
            return parse_args()

    def test_a_class_agnostic_tracker_must_state_its_selection(self):
        with self.assertRaises(SystemExit):
            self.parse(["--camera-index", "0", "--object-rule",
                        "--reference-depth-m", "0.3"])

    def test_stating_the_selection_is_enough(self):
        args = self.parse(["--camera-index", "0", "--object-rule",
                           "--selection", "central",
                           "--reference-depth-m", "0.3"])
        self.assertTrue(args.object_rule)
        self.assertEqual(args.selection, "central")

    def test_the_object_rule_replaces_the_other_loops(self):
        for extra in (["--fixed-rule"], ["--joint-center"], ["--approach"]):
            with self.subTest(extra=extra):
                with self.assertRaises(SystemExit):
                    self.parse(["--camera-index", "0", "--object-rule",
                                "--selection", "score",
                                "--reference-depth-m", "0.3"] + extra)

    def test_one_distance_measurement_and_only_one(self):
        cases = {
            'none': ["--camera-index", "0", "--object-rule",
                     "--selection", "score"],
            'both': ["--camera-index", "0", "--object-rule",
                     "--selection", "score", "--reference-depth-m", "0.3",
                     "--target-extent-m", "0.05"],
            'negative': ["--camera-index", "0", "--object-rule",
                          "--selection", "score", "--reference-depth-m", "-0.3"],
        }
        for label, argv in cases.items():
            with self.subTest(case=label):
                with self.assertRaises(SystemExit):
                    self.parse(argv)

    def test_a_region_tracker_needs_a_region(self):
        with self.assertRaises(SystemExit):
            self.parse(["--camera-index", "0", "--object-rule",
                        "--tracker", "support", "--reference-depth-m", "0.3"])

    def test_a_detector_only_tracker_needs_semantics(self):
        with self.assertRaises(SystemExit):
            self.parse(["--camera-index", "0", "--object-rule",
                        "--tracker", "detector", "--selection", "score",
                        "--reference-depth-m", "0.3"])

    def test_the_catalogue_is_printable(self):
        from eai_robot.applications.visual_servo.ball import print_detector_catalogue
        self.assertEqual(print_detector_catalogue(), 0)


if __name__ == "__main__":
    unittest.main()
