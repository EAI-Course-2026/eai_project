"""Simulated acceptance for the fixed-rule approach on the real URDF.

The arm, camera mount and marker are the recorded 2026-10-18 pose. The scene is
projected through the actual forward-kinematics chain, so a wrong axis or a
missing wrist counter-rotation shows up as a marker that leaves the frame. The
execution model applies the measured defects: a five-count deadband, a weaker
wrist in the negative direction and a two-count arrival residual.

The last two test classes replace only the device: a fake motor bus and a marker
that is projected from the bus's measured position. Everything between them --
the shared motion guard, the decision loop, the log -- is the code that runs on
the arm.
"""
import json
import tempfile
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from eai_robot.config import ROOT
from eai_robot.control.kinematics.kinematics_backend import create_fk_solver
from eai_robot.control.visual_servo.camera_step import (
    ResponseModel, StepInfeasible, counts_per_degree, depth_from_side_px,
    marker_side_px, plan_camera_step,
)
from eai_robot.control.visual_servo.fixed_rule import (
    FixedRuleConfig, FixedRuleController, Observation, image_region, region_delta,
)
from eai_robot.control.visual_servo.joint_adapter import validate_transform

# Recorded round-1 raw pose and the measured calibration fixtures (read-only).
RAW = {"shoulder_pan": 523, "shoulder_lift": 248, "elbow_flex": 646,
       "wrist_flex": 438, "wrist_roll": 226}
MARKER_M = 0.025


def load_intrinsics():
    import json
    return np.asarray(
        json.loads((ROOT / "calibration/vision/camera_intrinsics.json").read_text())
        ["camera_matrix"], dtype=float)


K = load_intrinsics()


class Scene:
    """A fronto-parallel 25 mm marker fixed in the base frame."""

    def __init__(self, start_degrees, mount, depth_m=0.30, offset_px=(0.0, 0.0),
                 camera_matrix=None):
        self.fk, _ = create_fk_solver(
            ROOT / "src/eai_robot/control/kinematics/so101_new_calib.urdf")
        self.mount = mount
        self.matrix = K if camera_matrix is None else np.asarray(camera_matrix, float)
        camera = self.fk.forward_kinematics(start_degrees) @ mount
        half = MARKER_M / 2.0
        local = np.array([[-half, -half, depth_m, 1.0], [half, -half, depth_m, 1.0],
                          [half, half, depth_m, 1.0], [-half, half, depth_m, 1.0]])
        self.points_base = (camera @ local.T).T[:, :3]
        self.start = np.asarray(start_degrees, dtype=float)

    def observe(self, degrees, shift_px=(0.0, 0.0)):
        camera = self.fk.forward_kinematics(degrees) @ self.mount
        local = (np.linalg.inv(camera) @ np.c_[self.points_base, np.ones(4)].T).T[:, :3]
        image = (self.matrix @ local.T).T
        pixels = image[:, :2] / image[:, 2:]
        return Observation(pixels + np.asarray(shift_px), (640, 480), degrees)

    def camera_z_to_marker(self, degrees):
        camera = self.fk.forward_kinematics(degrees) @ self.mount
        centre = self.points_base.mean(axis=0)
        return float((centre - camera[:3, 3]) @ camera[:3, 2])

    def camera_origin(self, degrees):
        """The point the travel budget must be measured on: the camera, not the flange."""
        return (self.fk.forward_kinematics(degrees) @ self.mount)[:3, 3]


class ExecutionModel:
    """Deadband, directional gain and arrival residual from the round-1 report."""

    def __init__(self, rates, deadband=5, residual=2,
                 gains=None):
        self.rates = np.asarray(rates, dtype=float)
        self.deadband = deadband
        self.residual = residual
        self.gains = gains or {}

    def apply(self, degrees, requested_counts, names):
        moved = {}
        for index, (name, requested) in enumerate(zip(names, requested_counts)):
            if requested == 0 or abs(requested) < self.deadband:
                continue
            gain = self.gains.get((name, int(np.sign(requested))), 1.0)
            actual = int(round(requested * gain)) - int(np.sign(requested)) * self.residual
            if abs(actual) < 1:
                continue
            moved[name] = actual
        updated = np.asarray(degrees, dtype=float).copy()
        for index, name in enumerate(names):
            if name in moved:
                updated[index] += moved[name] / self.rates[index]
        return updated, moved


def load_fixtures():
    import json
    from pathlib import Path
    from lerobot.motors import MotorCalibration
    raw = json.loads((ROOT / "calibration/scs215_so101.json").read_text())
    calibration = {name: MotorCalibration(id=value["id"], drive_mode=value["drive_mode"],
                                          homing_offset=value["homing_offset"],
                                          range_min=value["range_min"],
                                          range_max=value["range_max"])
                   for name, value in raw.items()}
    hand_eye = json.loads((ROOT / "calibration/vision/hand_eye_result.json").read_text())
    return calibration, validate_transform(hand_eye["gripper_T_camera"])


class FixedRuleSimulationTest(unittest.TestCase):
    """Closed loop against the real pose, with the measured execution defects."""

    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        cls.names = ARM_JOINT_NAMES
        calibration, cls.mount = load_fixtures()
        mapper = SO101JointMapper(calibration)
        cls.rates = counts_per_degree(calibration)
        cls.start = np.array([mapper.raw_to_urdf_degrees(name, RAW[name])
                              for name in ARM_JOINT_NAMES])
        cls.scene = Scene(cls.start, cls.mount, depth_m=0.30)

    def config(self, **overrides):
        # A simulation may spend more travel than the hardware profile allows;
        # the budget stop is asserted separately with an explicit override.
        values = dict(focal_px=K[0, 0], marker_length_m=MARKER_M,
                      max_joint_step_deg=5.0, max_joint_excursion_deg=70.0,
                      max_travel_mm=250.0)
        values.update(overrides)
        return FixedRuleConfig(**values)

    def run_loop(self, config=None, gains=None, steps=40, shift_px=(0.0, 0.0),
                 scene=None):
        config = config or self.config()
        scene = scene or self.scene
        controller = FixedRuleController(
            scene.fk, self.mount, config, self.rates, K,
            reference_degrees=self.start,
            origin_m=scene.camera_origin(self.start),
        )
        model = ExecutionModel(self.rates, gains=gains)
        degrees = self.start.copy()
        trace = []
        for _ in range(steps):
            observation = scene.observe(degrees, shift_px)
            action = controller.decide(observation)
            if not action.movable:
                return controller, action, trace, degrees
            plan = action.plan
            before_raw = {name: 500 for name in self.names}
            updated, moved = model.apply(degrees, plan.requested_counts, self.names)
            after_raw = {name: 500 + moved.get(name, 0) for name in self.names}
            after = scene.observe(updated, shift_px)
            report = controller.record(plan, observation, after, before_raw, after_raw)
            trace.append(dict(action=action, plan=plan, report=report,
                              depth=scene.camera_z_to_marker(updated),
                              center=after.center_px))
            degrees = updated
            if controller.halted:
                action = controller.decide(scene.observe(degrees, shift_px))
                return controller, action, trace, degrees
        return controller, controller.decide(scene.observe(degrees, shift_px)), trace, degrees

    def test_marker_outside_the_band_is_approached_until_standoff(self):
        controller, action, trace, degrees = self.run_loop()
        self.assertEqual(action.kind, "reached", action.reason)
        self.assertGreaterEqual(action.side_px, controller.config.stop_side_px)
        # The geometric depth that the pixel criterion implies at this focal
        # length, not a hand-picked millimetre figure.
        expected = K[0, 0] * MARKER_M / controller.config.stop_side_px
        self.assertLess(abs(self.scene.camera_z_to_marker(degrees) - expected), 0.03)
        self.assertGreaterEqual(len(trace), 8)
        phases = {item["action"].phase for item in trace}
        self.assertEqual(phases, {"align", "approach"})

    def test_forward_steps_keep_the_marker_near_the_centre(self):
        _, action, trace, _ = self.run_loop()
        approach = [item for item in trace if item["action"].phase == "approach"]
        self.assertGreaterEqual(len(approach), 5)
        for item in approach:
            center = item["center"]
            self.assertLess(abs(center[0] - 320), 90, item["plan"].describe())
            self.assertLess(abs(center[1] - 240), 90, item["plan"].describe())
        self.assertEqual(action.kind, "reached")

    def test_depth_progress_is_verified_and_monotone_overall(self):
        controller, action, trace, degrees = self.run_loop()
        expected = K[0, 0] * MARKER_M / controller.config.stop_side_px
        self.assertLess(self.scene.camera_z_to_marker(degrees), expected + 0.03)
        self.assertGreater(controller.snapshot()["depth_progress_mm"], 80.0)
        self.assertEqual(controller.snapshot()["disagreements"], 0)
        self.assertEqual(action.kind, "reached")

    def test_weak_wrist_response_is_learned_from_feedback(self):
        controller, action, trace, _ = self.run_loop(
            gains={("wrist_flex", -1): 0.5}, steps=80)
        self.assertEqual(action.kind, "reached", action.reason)
        # The measured gain includes the fixed arrival residual, so it settles
        # just under the true 0.5 and stops at the clip floor, not exactly at it.
        learned = controller.gains.gain("wrist_flex", -1)
        self.assertGreaterEqual(learned, 0.45)
        self.assertLessEqual(learned, 0.55)
        self.assertLess(learned, 0.9)

    def test_every_request_stays_inside_the_declared_step_cap(self):
        _, _, trace, _ = self.run_loop(gains={("wrist_flex", -1): 0.5})
        for item in trace:
            counts = item["plan"].requested_counts
            self.assertLessEqual(int(np.max(np.abs(counts))), 18, item["plan"].describe())

    def test_excursion_budget_stops_the_loop_and_is_reported(self):
        controller, action, trace, degrees = self.run_loop(
            config=self.config(max_joint_excursion_deg=12.0))
        self.assertEqual(action.kind, "hold")
        self.assertIn("excursion", action.reason)
        self.assertGreater(len(trace), 1)
        # Still far from the standoff: the budget, not the pixel criterion, stopped it.
        self.assertGreater(self.scene.camera_z_to_marker(degrees), 0.20)

    def test_stalled_encoder_halts_instead_of_retrying(self):
        # A deadband of 40 counts models a fully stuck joint: nothing the rule
        # asks for can move the arm, and the loop must stop, not spin.
        controller, action, trace, _ = self.run_loop(
            config=self.config(min_effective_counts=60.0))
        self.assertEqual(action.kind, "hold")
        self.assertTrue(action.reason)

    def test_travel_budget_stops_the_loop_before_the_standoff(self):
        controller, action, trace, degrees = self.run_loop(
            config=self.config(max_travel_mm=40.0))
        self.assertEqual(action.kind, "hold")
        self.assertIn("travel budget", action.reason)
        # The pixel criterion was never met: the budget is what stopped it.
        self.assertLess(action.side_px, controller.config.stop_side_px)
        self.assertGreater(self.scene.camera_z_to_marker(degrees), 0.20)
        # Travel is measured on the camera origin, so it must track the closed
        # distance rather than the flange's usually smaller displacement.
        closed_mm = (0.30 - self.scene.camera_z_to_marker(degrees)) * 1000.0
        self.assertLess(abs(controller.travel_mm(degrees) - closed_mm), 0.15 * closed_mm)


class RunnerPlumbingTest(unittest.TestCase):
    """The device owner must hand the planner the same numbers the guard uses."""

    def args(self, **overrides):
        from types import SimpleNamespace
        values = dict(rule_band=0.4, rule_step_mm=10.0, rule_stop_side_px=120.0,
                      rule_min_standoff_m=0.1, rule_gate_px=24.0,
                      marker_length_m=0.025, max_joint_step_deg=4.0,
                      rule_max_request_counts=18.0, max_travel_mm=100.0,
                      max_joint_excursion_deg=40.0)
        values.update(overrides)
        return SimpleNamespace(**values)

    def test_config_mirrors_the_guard_budgets(self):
        from eai_robot.applications.visual_servo.fixed_rule_servo import build_config
        args = self.args()
        config = build_config(args, K)
        config.validate()
        self.assertEqual(config.focal_px, K[0, 0])
        self.assertEqual(config.max_travel_mm, args.max_travel_mm)
        self.assertEqual(config.max_joint_step_deg, args.max_joint_step_deg)
        self.assertEqual(config.max_joint_excursion_deg, args.max_joint_excursion_deg)
        self.assertAlmostEqual(config.step_m, 0.01)

    def test_minimum_effective_request_clears_the_arrival_tolerance(self):
        # The guard refuses a command inside the arrival tolerance, so a planned
        # step below it would be rejected as "no change" instead of executed.
        from eai_robot.control.visual_servo.empirical import ARRIVAL_TOLERANCE
        self.assertGreaterEqual(FixedRuleConfig().min_effective_counts,
                                ARRIVAL_TOLERANCE)

    def test_camera_matrix_loader_rejects_a_non_pinhole_file(self):
        import json
        import tempfile
        from pathlib import Path
        from eai_robot.applications.visual_servo.fixed_rule_servo import load_camera_matrix
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "intrinsics.json"
            path.write_text(json.dumps({"camera_matrix": [[0, 0, 320], [0, 0, 240],
                                                          [0, 0, 1]]}))
            with self.assertRaises(ValueError):
                load_camera_matrix(path)
        self.assertEqual(load_camera_matrix(
            ROOT / "calibration/vision/camera_intrinsics.json").shape, (3, 3))

    def test_camera_origin_follows_the_mount(self):
        from eai_robot.applications.visual_servo.fixed_rule_servo import camera_origin
        calibration, mount = load_fixtures()
        fk, _ = create_fk_solver(
            ROOT / "src/eai_robot/control/kinematics/so101_new_calib.urdf")
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        mapper = SO101JointMapper(calibration)
        degrees = np.array([mapper.raw_to_urdf_degrees(name, RAW[name])
                            for name in ARM_JOINT_NAMES])
        np.testing.assert_allclose(
            camera_origin(fk, mount, degrees),
            (fk.forward_kinematics(degrees) @ mount)[:3, 3])
        # The camera sits ~61 mm off the flange, so the two origins must differ.
        self.assertGreater(
            np.linalg.norm(camera_origin(fk, mount, degrees)
                           - fk.forward_kinematics(degrees)[:3, 3]), 0.03)


class RuleGeometryTest(unittest.TestCase):
    def test_regions_match_the_documented_axes(self):
        cases = [((100, 240), "left"), ((550, 240), "right"),
                 ((320, 80), "up"), ((320, 420), "down"), ((320, 240), "center")]
        for center, region in cases:
            with self.subTest(center=center):
                self.assertEqual(image_region(center, (640, 480), 0.4), region)
        np.testing.assert_allclose(region_delta("left", 0.01), [-0.01, 0, 0])
        np.testing.assert_allclose(region_delta("down", 0.01), [0, 0.01, 0])
        np.testing.assert_allclose(region_delta("center", 0.01), [0, 0, 0.01])

    def test_band_edges_are_inclusive_of_the_central_region(self):
        # band 0.4 keeps 0.3..0.7 of each axis, i.e. 192..448 px horizontally.
        self.assertEqual(image_region((192, 240), (640, 480), 0.4), "center")
        self.assertEqual(image_region((191, 240), (640, 480), 0.4), "left")
        self.assertEqual(image_region((448, 240), (640, 480), 0.4), "center")
        self.assertEqual(image_region((449, 240), (640, 480), 0.4), "right")
        with self.assertRaises(ValueError):
            image_region((320, 240), (640, 480), 1.5)

    def test_no_step_without_a_positive_marker_size(self):
        with self.assertRaises(ValueError):
            depth_from_side_px(0, K[0, 0], 0.025)
        side = K[0, 0] * MARKER_M / 0.30
        self.assertAlmostEqual(depth_from_side_px(side, K[0, 0], MARKER_M), 0.30, places=6)


class ForwardPrimitiveTest(unittest.TestCase):
    """Regression: a forward step must pair lift with an opposing wrist."""

    def setUp(self):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        calibration, self.mount = load_fixtures()
        mapper = SO101JointMapper(calibration)
        self.rates = counts_per_degree(calibration)
        self.names = ARM_JOINT_NAMES
        self.start = np.array([mapper.raw_to_urdf_degrees(name, RAW[name])
                               for name in ARM_JOINT_NAMES])
        self.scene = Scene(self.start, self.mount, depth_m=0.30)
        self.observation = self.scene.observe(self.start)

    def plan(self, delta, gains=None, **kwargs):
        return plan_camera_step(
            self.scene.fk, self.start, self.mount, delta,
            self.observation.corners_px,
            np.full(4, depth_from_side_px(marker_side_px(self.observation.corners_px),
                                          K[0, 0], MARKER_M)),
            K, counts_per_deg=self.rates, gains=gains, **kwargs)

    def test_forward_pairs_lift_with_counter_rotation(self):
        plan = self.plan([0, 0, 0.01])
        counts = dict(zip(self.names, plan.requested_counts))
        self.assertGreater(counts["shoulder_lift"], 0)
        self.assertLess(counts["wrist_flex"], 0)
        # The counter-rotation cannot be exact: one count on either joint of the
        # pair is about 0.3 deg of uncancelled pitch, so the residual is a
        # quantization floor, not a tuning error. What must hold is that the
        # resulting image drift stays far below the drift a centering loop would
        # mistake for its own error.
        self.assertLess(abs(plan.pitch_rad), 0.02)
        # A 13/12 count pair leaves a few pixels of pitch residue; that is the
        # encoder floor, and the standard it must meet is the 66 px a lift-only
        # step would produce, not zero.
        self.assertLess(np.linalg.norm(plan.predicted_image_change[:2]), 6.0)
        self.assertGreater(plan.predicted_image_change[2], 0.8)

    def test_one_count_residual_exceeds_the_depth_signal(self):
        # Records the physical reason the forward primitive needs a verified
        # pair: integer encoder counts put a floor of about 0.3 deg, i.e. ~5 px,
        # on how well the pitch can be cancelled, which is already twice the
        # apparent-size change of a one-centimetre step at 300 mm.
        from eai_robot.control.visual_servo.joint_adapter import camera_joint_jacobian
        from eai_robot.control.visual_servo.camera_step import image_task_matrix
        _, jacobian = camera_joint_jacobian(self.scene.fk, self.start, self.mount)
        task = image_task_matrix(self.observation.corners_px, np.full(4, 0.30), K)
        one_count = np.zeros(5)
        one_count[1] = np.deg2rad(1.0 / self.rates[1])
        floor_px = float(np.linalg.norm((task @ jacobian @ one_count)[:2]))
        depth_signal = float(self.plan([0, 0, 0.01]).predicted_image_change[2])
        self.assertGreater(floor_px, 2.0 * depth_signal)

    def test_lift_alone_would_move_the_marker_far_more_than_the_depth_signal(self):
        # Documents the failure mode this primitive exists to avoid.
        from eai_robot.control.visual_servo.joint_adapter import camera_joint_jacobian
        _, jacobian = camera_joint_jacobian(self.scene.fk, self.start, self.mount)
        from eai_robot.control.visual_servo.camera_step import image_task_matrix
        task = image_task_matrix(self.observation.corners_px,
                                 np.full(4, 0.30), K)
        lift_only = task @ jacobian @ np.array([0, np.deg2rad(4.0), 0, 0, 0])
        depth_signal = self.plan([0, 0, 0.01]).predicted_image_change[2]
        self.assertGreater(abs(lift_only[1]), 5 * abs(depth_signal))

    def test_deadband_joint_is_not_commanded(self):
        plan = self.plan([0, 0, 0.01])
        counts = dict(zip(self.names, plan.requested_counts))
        for name, value in counts.items():
            self.assertFalse(0 < abs(value) < 5, f"{name}={value} inside deadband")

    def test_requested_direction_is_preserved(self):
        # A forward step's size signal is only a couple of pixels, so the
        # dominant *predicted* component can be the quantization residue on the
        # other axis; what must hold is that the rule's own signal has the right
        # sign and that the residue stays bounded.
        forward = self.plan([0, 0, 0.01])
        self.assertEqual(forward.goal.dominant, "depth")
        self.assertGreater(forward.predicted_image_change[2], 0.8)
        self.assertLess(abs(forward.predicted_image_change[1]), 5.0)
        for delta, axis, sign in (([0.01, 0, 0], 0, -1), ([-0.01, 0, 0], 0, 1),
                                  ([0, -0.01, 0], 1, 1), ([0, 0.01, 0], 1, -1)):
            with self.subTest(delta=delta):
                plan = self.plan(delta)
                dominant = int(np.argmax(np.abs(plan.predicted_image_change)))
                self.assertEqual(dominant, axis)
                self.assertEqual(np.sign(plan.predicted_image_change[axis]), sign)

    def test_step_larger_than_the_budget_is_rejected(self):
        with self.assertRaises(ValueError):
            self.plan([0, 0, 0.05])

    def test_no_image_effect_is_rejected(self):
        with self.assertRaises(StepInfeasible):
            plan_camera_step(
                self.scene.fk, self.start, self.mount, [0, 0, 0.01],
                self.observation.corners_px, np.full(4, 0.30), K,
                counts_per_deg=self.rates, max_joint_step_deg=0.02)

    def test_response_model_is_bounded_and_immutable(self):
        from eai_robot.control.visual_servo.camera_step import MIN_GAIN
        model = ResponseModel()
        updated = model.updated("wrist_flex", -1, 0.05)
        self.assertEqual(model.entries, ())
        self.assertGreaterEqual(updated.gain("wrist_flex", -1), MIN_GAIN)
        self.assertEqual(model.gain("wrist_flex", -1), 1.0)


class FakeBus:
    """Motor bus stand-in that applies the measured defects on every goal write.

    ``Present_Position`` moves only when a write leaves the five-count deadband,
    then loses the two-count arrival residual, which is exactly the behaviour the
    round-1 log shows. ``Goal_Position`` echoes the write so the guard's readback
    check still has something real to compare.
    """

    def __init__(self, raw, deadband=5, residual=2, gains=None):
        self.present = {name: int(value) for name, value in raw.items()}
        self.goal = dict(self.present)
        self.deadband = deadband
        self.residual = residual
        self.gains = gains or {}
        self.writes = []
        self.port_handler = types.SimpleNamespace(clearPort=lambda: None)

    def read(self, register, motor, normalize=True, num_retry=1):
        if register == "Present_Position":
            return self.present[motor]
        if register == "Goal_Position":
            return self.goal[motor]
        if register == "Torque_Enable":
            return 1
        if register == "Status":
            return 0
        raise KeyError(register)

    def sync_write(self, register, values, normalize=True, num_retry=1):
        if register != "Goal_Position":
            raise KeyError(register)
        self.writes.append({name: int(value) for name, value in values.items()})
        for name, value in values.items():
            self.goal[name] = int(value)
        for name, value in values.items():
            delta = int(value) - self.present[name]
            if abs(delta) < self.deadband:
                continue
            sign = 1 if delta > 0 else -1
            gain = self.gains.get((name, sign), 1.0)
            moved = int(round(abs(delta) * gain)) - self.residual
            if moved < 1:
                continue
            self.present[name] += sign * moved


class SimMarkerDetector:
    """Marker projected from the bus's measured pose instead of from pixels."""

    def __init__(self, scene, mapper, bus, marker_id=0, dictionary="DICT_4X4_50"):
        from eai_robot.vision.detection.ball import BallDetection
        self.scene, self.mapper, self.bus = scene, mapper, bus
        self.marker_id, self.dictionary = marker_id, dictionary
        self.last_marker_id = marker_id
        self._detection = BallDetection

    def marker(self, frame_rgb):
        from eai_robot.vision.detection.aruco import MarkerObservation
        degrees = np.asarray([self.mapper.raw_to_urdf_degrees(name, self.bus.present[name])
                              for name in self.bus.present], dtype=float)
        observation = self.scene.observe(degrees)
        return MarkerObservation(self.marker_id, observation.corners_px)

    def detect(self, frame_rgb):
        observation = self.marker(frame_rgb)
        centre = observation.center_px
        mask = np.zeros(frame_rgb.shape[:2], dtype=np.uint8)
        return self._detection(float(centre[0]), float(centre[1]), 10.0, 314.0, 1.0, 1.0), mask


class FakeRuntime:
    """Only what the runner and the shared guard actually touch."""

    def __init__(self, fk, mapper, bus):
        self.fk = fk
        self.mapper = mapper
        self.robot = types.SimpleNamespace(bus=bus)
        self.command_q = None
        self.command_position = None
        self.hold_reason = ""
        self._frame = np.zeros((480, 640, 3), dtype=np.uint8)

    def _require_mapper(self):
        return self.mapper

    def read_frame(self):
        return self._frame


def runner_args(**overrides):
    from eai_robot.applications.visual_servo.fixed_rule_servo import (
        DEFAULT_HAND_EYE, DEFAULT_INTRINSICS,
    )
    values = dict(
        intrinsics=DEFAULT_INTRINSICS, hand_eye=DEFAULT_HAND_EYE,
        marker_id=0, dictionary="DICT_4X4_50", marker_length_m=MARKER_M,
        rule_band=0.4, rule_step_mm=10.0, rule_stop_side_px=120.0,
        rule_min_standoff_m=0.1, rule_gate_px=24.0, rule_max_steps=40,
        rule_max_request_counts=18.0, max_travel_mm=100.0,
        max_joint_excursion_deg=40.0, max_joint_step_deg=5.0,
        joint_margin_deg=2.0, torque_settle_s=0.0, feature_samples=1,
        detection_timeout_s=2.0, max_runtime_s=0.0, hold_reference=None,
    )
    values.update(overrides)
    return types.SimpleNamespace(**values)


class RunnerIntegrationTest(unittest.TestCase):
    """Run the real decision loop and the real guard against fake devices."""

    @classmethod
    def run_runner(cls, scene, bus=None, args=None, gains=None):
        from eai_robot.applications.visual_servo import fixed_rule_servo
        from eai_robot.control.execution.feedback import (
            arm_degrees_from_raw, read_raw_motors,
        )
        args = args or runner_args()
        bus = bus or FakeBus(RAW, gains=gains)
        runtime = FakeRuntime(scene.fk, cls.mapper, bus)
        detector = SimMarkerDetector(scene, cls.mapper, bus)
        stop = types.SimpleNamespace(stop_requested=threading.Event())
        directory = tempfile.mkdtemp()
        patches = (
            patch.object(fixed_rule_servo, "ROOT", Path(directory)),
            patch.object(fixed_rule_servo, "MarkerCornersDetector",
                         lambda *a, **k: detector),
        )
        with patches[0], patches[1]:
            result = fixed_rule_servo.run_fixed_rule(runtime, args, stop, None)
        logs = sorted((Path(directory) / "local_artifacts/experiments").glob("*.jsonl"))
        events = [json.loads(line) for line in logs[-1].read_text().splitlines()]
        degrees = arm_degrees_from_raw(cls.mapper,
                                       read_raw_motors(runtime.robot, cls.names))
        return result, runtime, bus, events, degrees

    @classmethod
    def scene(cls, depth_m=0.25):
        from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
        start = np.array([cls.mapper.raw_to_urdf_degrees(name, RAW[name])
                          for name in ARM_JOINT_NAMES])
        return Scene(start, cls.mount, depth_m=depth_m)

    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        calibration, cls.mount = load_fixtures()
        cls.mapper = SO101JointMapper(calibration)
        cls.names = ARM_JOINT_NAMES
        # One full closed-loop run serves the assertions that do not vary an
        # argument; each additional run costs a real 0.2 s arrival lease per
        # step, which is the dominant cost of this class. The excursion budget
        # is widened for this run only: the shipped 40 degrees stops a 250 mm
        # approach mid-way, which is asserted separately.
        cls.shared = cls.run_runner(cls.scene(),
                                    args=runner_args(max_joint_excursion_deg=70.0))

    def test_forward_step_writes_every_joint_the_planner_asked_for(self):
        # The shared guard was written for a two-joint centering loop. If it
        # filters the writable set, a forward step still reports arrival while
        # the lift was never sent -- the arm would only pitch its wrist.
        _, _, bus, events, degrees = self.shared
        plans = [event for event in events if event["event"] == "plan"]
        forward = [item for item in plans if item["phase"] == "approach"]
        self.assertTrue(forward, "no forward step was planned")
        lift = sum(abs(write.get("shoulder_lift", 0)) for write in bus.writes)
        self.assertGreater(lift, 0, "shoulder_lift was never written")
        self.assertLess(self.scene().camera_z_to_marker(degrees), 0.25,
                        "the camera did not move forward")

    def test_runner_reaches_standoff_and_logs_every_stage(self):
        result, runtime, _, events, degrees = self.shared
        self.assertEqual(result, 0)
        names = [event["event"] for event in events]
        self.assertEqual(names[0], "settings")
        for stage in ("observation", "plan", "motion", "feedback"):
            self.assertIn(stage, names)
        self.assertEqual(names[-2], "reached")
        self.assertEqual(names[-1], "summary")
        self.assertGreaterEqual(events[-2]["side_px"], 120.0)
        self.assertLess(self.scene().camera_z_to_marker(degrees), 0.20)
        self.assertEqual(runtime.hold_reason, "USER STOP")

    def test_runner_never_exceeds_the_declared_request_cap(self):
        _, _, _, events, _ = self.shared
        plans = [event["plan"] for event in events if event["event"] == "plan"]
        self.assertTrue(plans, "the loop never planned a step")
        for plan in plans:
            for name, value in plan["requested_counts"].items():
                self.assertLessEqual(abs(value), 18,
                                     f"{name}={value} exceeded the request cap")

    def test_runner_logs_the_marker_identity_it_actually_saw(self):
        _, _, _, events, _ = self.shared
        observation = [event for event in events if event["event"] == "observation"][0]
        self.assertEqual(observation["marker_id"], 0)
        self.assertAlmostEqual(observation["side_px"],
                               K[0, 0] * MARKER_M / 0.25, delta=1.0)

    def test_runner_stops_at_the_shipped_excursion_budget(self):
        scene = self.scene()
        _, runtime, _, events, degrees = self.run_runner(scene, args=runner_args())
        self.assertEqual(events[-2]["event"], "hold")
        self.assertIn("excursion budget", events[-2]["reason"])
        self.assertIn("excursion", runtime.hold_reason)
        # The pixel criterion was never satisfied: the budget ended the run.
        last_side = [event["side_after_px"] for event in events
                     if event["event"] == "feedback"][-1]
        self.assertLess(last_side, 120.0)
        self.assertGreater(scene.camera_z_to_marker(degrees), 0.17)

    def test_runner_holds_and_reports_when_the_travel_budget_runs_out(self):
        scene = self.scene()
        args = runner_args(max_travel_mm=25.0)
        result, runtime, _, events, degrees = self.run_runner(scene, args=args)
        self.assertEqual(result, 0)
        self.assertEqual(events[-2]["event"], "hold")
        self.assertIn("travel budget", events[-2]["reason"])
        self.assertIn("travel budget", runtime.hold_reason)
        # It stopped short of the standoff, with the marker still visible.
        self.assertGreater(scene.camera_z_to_marker(degrees), 0.19)


class BaseVersionCompatibilityTest(unittest.TestCase):
    """Pins the recorded-demo path against the any-object additions.

    The second round made ``extent_px`` optional on ``Observation`` and
    ``scale`` optional on the controller. The ArUco demo builds its input from
    three positional arguments and never attaches a scale, so those two shapes
    are asserted here: if a later change starts *reading* an extent the old path
    never sets, or makes the scale mandatory, this class fails before a recorded
    demo does. Nothing here measures the arm; every assertion is about which
    arithmetic the old inputs reach.
    """

    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        calibration, cls.mount = load_fixtures()
        mapper = SO101JointMapper(calibration)
        cls.names = ARM_JOINT_NAMES
        cls.rates = counts_per_degree(calibration)
        cls.start = np.array([mapper.raw_to_urdf_degrees(name, RAW[name])
                              for name in ARM_JOINT_NAMES])
        cls.scene = Scene(cls.start, cls.mount, depth_m=0.30)

    def controller(self, **overrides):
        values = dict(focal_px=K[0, 0], marker_length_m=MARKER_M,
                      max_joint_step_deg=5.0, max_joint_excursion_deg=70.0,
                      max_travel_mm=250.0)
        values.update(overrides)
        return FixedRuleController(
            self.scene.fk, self.mount, FixedRuleConfig(**values), self.rates, K,
            reference_degrees=self.start,
            origin_m=self.scene.camera_origin(self.start))

    def test_the_aruco_config_defaults_still_validate(self):
        # No new argument is required to describe the first round's task.
        FixedRuleConfig(focal_px=K[0, 0], marker_length_m=MARKER_M).validate()

    def test_three_argument_observation_keeps_the_aruco_side(self):
        observation = self.scene.observe(self.start)
        self.assertIsNone(observation.extent_px)
        self.assertEqual(observation.extent_name, "side")
        expected = marker_side_px(observation.corners_px)
        self.assertAlmostEqual(observation.side_px, expected, places=9)
        # The value is the recorded one for this pose, not merely self
        # consistent: the 300 mm fixture projects to a rectangle, because fx and
        # fy differ by 0.1 %, so the mean edge answers to the mean focal length.
        mean_focal = float(np.mean([K[0, 0], K[1, 1]]))
        self.assertAlmostEqual(expected, mean_focal * MARKER_M / 0.30, places=6)

    def test_no_scale_reproduces_the_round_one_depth(self):
        controller = self.controller()
        observation = self.scene.observe(self.start)
        expected = depth_from_side_px(observation.side_px, K[0, 0], MARKER_M)
        # 0.3 m to the same fx/fy-order tolerance; the route identity below is
        # what is asserted exactly, since the millimetre value is a projection.
        self.assertAlmostEqual(expected, 0.30, delta=5e-4)
        self.assertAlmostEqual(controller.depth_m(observation), expected, places=12)
        # ``record`` and ``decide`` must read the same route; two independent
        # implementations drifting apart is how a log contradicts a decision.
        self.assertAlmostEqual(controller.depth_of(observation), expected, places=12)
        action = controller.decide(observation)
        self.assertEqual(action.kind, "move", action.reason)
        self.assertAlmostEqual(action.depth_m, expected, places=12)

    def test_corners_may_touch_the_frame_edge_but_not_cross_it(self):
        corners = np.array([[0.0, 100.0], [60.0, 100.0], [60.0, 160.0], [0.0, 160.0]])
        Observation(corners, (640, 480), self.start).validated()
        with self.assertRaises(ValueError):
            Observation(corners + [-1.0, 0.0], (640, 480), self.start).validated()

    def test_the_geometry_path_agrees_with_the_legacy_corner_path(self):
        from eai_robot.vision.geometry.targets import TargetGeometry
        legacy = self.scene.observe(self.start)
        geometry = TargetGeometry.from_corners(legacy.corners_px, (640, 480),
                                               0.0, "marker")
        bridged = Observation.from_geometry(geometry, self.start)
        self.assertAlmostEqual(bridged.side_px, legacy.side_px, places=9)
        np.testing.assert_allclose(bridged.center_px, legacy.center_px, atol=1e-9)
        self.assertEqual(bridged.extent_name, legacy.extent_name)
        # Same target, two entry points: the step the arm is asked for must be
        # the same step, otherwise the new path has quietly retuned the demo.
        old = self.controller().decide(legacy)
        new = self.controller().decide(bridged)
        self.assertEqual((old.kind, old.phase, old.region),
                         (new.kind, new.phase, new.region))
        np.testing.assert_array_equal(old.plan.requested_counts,
                                      new.plan.requested_counts)


if __name__ == "__main__":
    unittest.main()
