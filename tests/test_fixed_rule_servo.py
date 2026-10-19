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
    marker_side_px, plan_axis_step, plan_camera_step,
)
from eai_robot.control.visual_servo.empirical import OBSERVED_MOTION_COUNTS
from eai_robot.control.visual_servo.fixed_rule import (
    MEASURED_COUNT_OFFSETS, FixedRuleConfig, FixedRuleController, Observation,
    image_region, region_delta,
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

    def test_visual_prediction_residual_is_diagnostic_and_allows_correction(self):
        # 19:05 bench regression: the marker moved toward the target, but
        # less than the model predicted. A 27.3 px prediction residual must
        # remain a diagnostic rather than veto the next correction.
        from dataclasses import replace
        controller = FixedRuleController(
            self.scene.fk, self.mount, self.config(), self.rates, K,
            reference_degrees=self.start,
            origin_m=self.scene.camera_origin(self.start))
        before = self.scene.observe(self.start)
        plan = controller.decide(before).plan
        plan = replace(plan, predicted_image_change=np.array([-19.4, -28.8, 0.]))
        after = self.scene.observe(self.start, shift_px=(-7.8, -4.0))
        before_raw = {name: 500 for name in self.names}
        after_raw = {name: 500 + int(count) for name, count in
                     zip(self.names, plan.requested_counts)}
        report = controller.record(plan, before, after, before_raw, after_raw)
        self.assertEqual(report.verdict, 'center_drift')
        self.assertIn('预测', report.reason)
        self.assertIn('实测', report.reason)
        self.assertTrue(controller.decide(after).movable)
        self.assertEqual(controller.snapshot()['halted'], '')

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

    def test_alignment_fallback_is_directional_and_requires_no_measurable_motion(self):
        from types import SimpleNamespace
        controller = FixedRuleController(
            self.scene.fk, self.mount, self.config(), self.rates, K,
            reference_degrees=self.start, origin_m=self.scene.camera_origin(self.start))
        plan = SimpleNamespace(goal=SimpleNamespace(dominant='vertical'),
                               requested_counts=np.array([0, -10, 0, 0, 0]))
        # An axis that moved and returned must not be retried as a deadband.
        self.assertFalse(controller.record_no_response(
            plan, {'shoulder_lift': -10}, {'shoulder_lift': 4}))
        self.assertTrue(controller.record_no_response(
            plan, {'shoulder_lift': -10}, {'shoulder_lift': 1}))
        self.assertEqual(controller.unresponsive_directions, {('shoulder_lift', -1)})
        self.assertNotIn(('shoulder_lift', 1), controller.unresponsive_directions)
        self.assertEqual(controller.steps, 1)
        self.assertFalse(controller.halted)
        plan.goal.dominant = 'depth'
        self.assertFalse(controller.record_no_response(
            plan, {'shoulder_lift': -10}, {'shoulder_lift': 0}))
        self.assertEqual(controller.steps, 1)

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


class ShippedBudgetTest(unittest.TestCase):
    """The profile's budgets must express the approach the bench actually does.

    Raised on 2026-10-18 at the operator's request, then checked by replaying
    the loop here. These assertions exist because "2-3x the old value" is a
    multiplier, not a derivation: the numbers below are what the arm costs.
    """

    @classmethod
    def setUpClass(cls):
        FixedRuleSimulationTest.setUpClass()
        cls.sim = FixedRuleSimulationTest(
            'test_marker_outside_the_band_is_approached_until_standoff')

    def shipped(self, **overrides):
        """The values in configs/visual_servo/aruco_fixed_rule.toml."""
        values = dict(focal_px=K[0, 0], marker_length_m=MARKER_M,
                      max_joint_step_deg=12.0, max_joint_excursion_deg=120.0,
                      max_travel_mm=300.0, max_request_counts=40.0)
        values.update(overrides)
        return FixedRuleConfig(**values)

    def replay(self, depth_m, config):
        scene = Scene(self.sim.start, self.sim.mount, depth_m=depth_m)
        return self.sim.run_loop(config=config, steps=60, scene=scene)

    def forward_plan(self, **overrides):
        from eai_robot.control.visual_servo.camera_step import (
            ResponseModel, plan_camera_step,
        )
        from eai_robot.control.visual_servo.fixed_rule import (
            MEASURED_COUNT_OFFSETS,
        )
        scene = Scene(self.sim.start, self.sim.mount, depth_m=0.30)
        observation = scene.observe(self.sim.start)
        return plan_camera_step(
            scene.fk, self.sim.start, self.sim.mount,
            np.array([0.0, 0.0, 0.01]), observation.corners_px,
            np.full(4, 0.30), K, counts_per_deg=self.sim.rates,
            max_joint_step_deg=overrides.get('max_joint_step_deg', 12.0),
            max_request_counts=overrides.get('max_request_counts', 40.0),
            min_effective_counts=5.0,
            gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS))

    def test_the_shipped_caps_deliver_a_whole_centimetre(self):
        # 0.987, not 1.000: integer encoder counts and the offset model leave a
        # little on the table. What must not happen is the 0.76 that the old
        # pair of caps delivered while still reporting a legal step.
        self.assertGreaterEqual(self.forward_plan().achieved_fraction, 0.95)

    def test_the_five_degree_cap_cannot_deliver_the_centimetre(self):
        # This is why the ceiling moved. It is also why the offset model made
        # the forward step worse without any parameter changing: the same 16
        # counts now pay a 7-count wrist offset first.
        plan = self.forward_plan(max_joint_step_deg=5.0, max_request_counts=18.0)
        self.assertLess(plan.achieved_fraction, 0.85)
        self.assertGreater(plan.achieved_fraction, 0.70)

    def test_raising_only_the_degree_cap_still_clips(self):
        # 22 counts is itself above the old 18-count request cap, so a profile
        # that raised the degrees alone would have changed nothing: the count
        # cap binds at 18 and the step still falls short of the whole
        # centimetre.
        plan = self.forward_plan(max_joint_step_deg=12.0, max_request_counts=18.0)
        self.assertLessEqual(int(np.max(np.abs(plan.requested_counts))), 18)
        self.assertLess(plan.achieved_fraction,
                        self.forward_plan().achieved_fraction - 0.05)

    def test_the_excursion_ceiling_covers_a_400mm_start(self):
        controller, action, _, _ = self.replay(0.40, self.shipped())
        self.assertEqual(action.kind, "reached", action.reason)
        self.assertGreaterEqual(action.side_px, controller.config.stop_side_px)

    def test_a_420mm_start_is_stopped_by_the_excursion_budget(self):
        # The ceiling is the binding constraint here, not the arm: the message
        # has to name the budget so the operator knows to move the marker.
        controller, action, _, degrees = self.replay(0.42, self.shipped())
        self.assertEqual(action.kind, "hold")
        self.assertIn("excursion", action.reason)
        self.assertLess(action.side_px, controller.config.stop_side_px)
        self.assertGreater(controller.travel_mm(degrees), 200.0)

    def test_a_440mm_start_cannot_reach_the_standoff_at_any_budget(self):
        # Past ~142 degrees the forward primitive runs shoulder_lift into its
        # own mechanical end (+100 deg from a -44.7 deg start), and no
        # candidate axis clears the count deadband. That end is *not* a
        # forward-reach limit: on its own the axis gets the camera to 330.6 mm
        # at +13 deg and then swings past the marker, so at +100 deg the camera
        # is already behind it. Raising the ceiling further does not buy
        # approach distance -- it only changes which message reports the stop.
        results = {}
        for ceiling in (120.0, 160.0):
            _, action, _, degrees = self.replay(
                0.44, self.shipped(max_joint_excursion_deg=ceiling))
            self.assertEqual(action.kind, "hold")
            results[ceiling] = action.side_px
        self.assertLess(results[160.0], 120.0)
        self.assertLess(results[160.0], results[120.0] + 15.0)

    def test_the_mechanical_end_is_not_the_forward_limit(self):
        # The distinction the operator had to correct: shoulder_lift reaching
        # +100 deg says the joint ran out of travel, not that the camera cannot
        # get closer. Alone it closes to 330.6 mm around +13 deg, then retreats
        # until at +100 deg the camera sits behind the marker plane.
        from eai_robot.control.visual_servo.camera_step import URDF_LIMITS
        scene = Scene(self.sim.start, self.sim.mount, depth_m=0.44)
        marker = scene.points_base.mean(axis=0)

        def distance(lift_deg):
            angles = self.sim.start.copy()
            angles[1] = lift_deg
            return float(np.linalg.norm(marker - scene.camera_origin(angles))) * 1000.0

        scan = {angle: distance(angle) for angle in range(-100, 101, 1)}
        closest_angle = min(scan, key=lambda angle: scan[angle])
        self.assertAlmostEqual(scan[closest_angle], 330.6, delta=2.0)
        self.assertAlmostEqual(closest_angle, 13.0, delta=2.0)
        # Well short of the 120 px standoff, so the approach cannot be this axis
        # alone -- it is the lift/wrist pair that approaches.
        self.assertGreater(scan[closest_angle], 300.0)
        # And the end of the range is on the far side of the marker, not in
        # front of it: this is a mechanically folded pose, not a reach limit.
        upper = URDF_LIMITS['shoulder_lift'].upper_deg
        self.assertAlmostEqual(upper, 100.0, delta=0.01)
        self.assertLess(scene.camera_z_to_marker(
            np.concatenate([[self.sim.start[0]], [upper], self.sim.start[2:]])), 0.0)
        self.assertGreater(scan[int(round(upper))], 500.0)


class ElbowShareTest(unittest.TestCase):
    """Why the forward step does not share its load with ``elbow_flex``.

    ``elbow_flex`` keeps 82 deg of travel unused on a 440 mm approach and its
    forward efficiency per degree (0.723 px/deg) matches ``shoulder_lift``'s
    (0.714), so on paper it is an idle spare axis. It was implemented and
    measured: a standing cost of 1.5 on lift splits a 300 mm step into 2.79 /
    2.69 deg instead of 5.15 / 0.00, and the shorter steps that follow make a
    400 mm run miss the standoff entirely (60 steps, 119.6 px) where the
    minimum-norm split arrives in 29.

    The reason is not the split itself, it is the *pairing*: every count of
    ``elbow_flex`` moves the marker vertically 1.32x as far as a count of
    ``shoulder_lift`` (7.48 vs 4.87 px), and the wrist has to cancel all of it.
    Spending less lift therefore costs more wrist, and ``wrist_flex`` is
    already the most-used axis of the pair. These assertions pin that trade so
    the "use the idle axis" idea is not re-tried from the efficiency table.
    """

    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        cls.names = ARM_JOINT_NAMES
        calibration, cls.mount = load_fixtures()
        mapper = SO101JointMapper(calibration)
        cls.rates = counts_per_degree(calibration)
        cls.start = np.array([mapper.raw_to_urdf_degrees(n, RAW[n])
                              for n in cls.names])
        cls.scene = Scene(cls.start, cls.mount, depth_m=0.30)
        observation = cls.scene.observe(cls.start)
        from eai_robot.control.visual_servo.camera_step import (
            camera_delta_goal, image_task_matrix,
        )
        from eai_robot.control.visual_servo.joint_adapter import (
            camera_joint_jacobian,
        )
        task = image_task_matrix(observation.corners_px, np.full(4, 0.30), K)
        _, jacobian = camera_joint_jacobian(cls.scene.fk, cls.start, cls.mount)
        cls.per_count = (task @ jacobian) * (np.pi / 180.0) / cls.rates
        cls.goal = camera_delta_goal(observation.corners_px, np.full(4, 0.30),
                                     K, [0.0, 0.0, 0.01]).vector()

    def split_for_elbow(self, elbow_counts):
        """The lift/wrist counts that serve the same goal with this much elbow."""
        lift = self.names.index('shoulder_lift')
        wrist = self.names.index('wrist_flex')
        elbow = self.names.index('elbow_flex')
        # dv and dsize rows only: du is served by other axes and the goal has none.
        rows = [1, 2]
        matrix = self.per_count[np.ix_(rows, [lift, wrist])]
        rhs = np.asarray(self.goal)[rows] - self.per_count[rows, elbow] * elbow_counts
        solution = np.linalg.solve(matrix, rhs)
        return float(solution[0]), float(solution[1])

    def test_elbow_moves_the_marker_vertically_more_than_lift_per_count(self):
        # The premise of the idea: elbow is not the weak axis.
        lift = self.names.index('shoulder_lift')
        elbow = self.names.index('elbow_flex')
        ratio = abs(self.per_count[1, elbow]) / abs(self.per_count[1, lift])
        self.assertGreater(ratio, 1.3)
        # And it is the most size-efficient axis of the three that can lift.
        self.assertGreater(self.per_count[2, elbow], self.per_count[2, lift])

    def test_spending_elbow_costs_more_wrist_not_less(self):
        # Same goal, more elbow: lift falls, wrist grows. The step's binding
        # axis therefore moves *to* the wrist rather than away from lift.
        wrist_loads = []
        for elbow_counts in (0.0, 6.0, 12.0):
            lift, wrist = self.split_for_elbow(elbow_counts)
            wrist_loads.append(abs(wrist))
            self.assertGreater(abs(lift), 0.0)
        self.assertLess(wrist_loads[0], wrist_loads[1])
        self.assertLess(wrist_loads[1], wrist_loads[2])
        # At the arm's own split (no elbow) the wrist load is the smallest of
        # the family, which is why the minimum-norm solve keeps elbow out.
        self.assertAlmostEqual(wrist_loads[0], 15.0, delta=1.5)

    def test_the_planner_keeps_the_wrist_sparing_split(self):
        # Behaviour, not arithmetic: a real forward plan at the recorded pose
        # puts the step on lift and leaves elbow in the deadband.
        from eai_robot.control.visual_servo.camera_step import (
            ResponseModel, plan_camera_step,
        )
        from eai_robot.control.visual_servo.fixed_rule import MEASURED_COUNT_OFFSETS
        observation = self.scene.observe(self.start)
        plan = plan_camera_step(
            self.scene.fk, self.start, self.mount, np.array([0.0, 0.0, 0.01]),
            observation.corners_px, np.full(4, 0.30), K,
            counts_per_deg=self.rates, max_joint_step_deg=12.0,
            max_request_counts=40.0, min_effective_counts=5.0,
            gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS))
        counts = dict(zip(self.names, plan.requested_counts))
        self.assertNotEqual(counts['shoulder_lift'], 0)
        self.assertEqual(counts['elbow_flex'], 0)
        self.assertGreaterEqual(plan.achieved_fraction, 0.95)


class DryStepGuardTest(unittest.TestCase):
    """A run that stops making progress must stop, and say why.

    At a 440 mm start the loop used to burn ten steps realising 0.12-0.32 of a
    forward step with the request flipping sign each time, then end on the step
    budget with no reason at all. The encoder moved and the image responded, so
    neither the stall guard nor the disagreement guard saw anything wrong.
    """

    @classmethod
    def setUpClass(cls):
        FixedRuleSimulationTest.setUpClass()
        cls.sim = FixedRuleSimulationTest(
            'test_marker_outside_the_band_is_approached_until_standoff')

    def shipped(self, **overrides):
        values = dict(focal_px=K[0, 0], marker_length_m=MARKER_M,
                      max_joint_step_deg=12.0, max_joint_excursion_deg=120.0,
                      max_travel_mm=300.0, max_request_counts=40.0)
        values.update(overrides)
        return FixedRuleConfig(**values)

    def replay(self, depth_m, **overrides):
        scene = Scene(self.sim.start, self.sim.mount, depth_m=depth_m)
        controller, action, trace, degrees = self.sim.run_loop(
            config=self.shipped(**overrides), steps=60, scene=scene)
        return controller, action, trace, degrees, scene

    def test_a_440mm_start_halts_with_a_named_reason(self):
        # Deliberately not the dry-step guard. Measured: at 140 deg the run
        # keeps closing about 4.6 mm per step (208.9 -> 185.7 mm over five
        # steps) even after lift saturates, so the guard must stay inert -- the
        # earlier reading of "the marker barely moves" compared *before* side
        # values and was wrong. What stops a 440 mm run is the excursion
        # budget, and that message is already specific.
        controller, action, trace, degrees, scene = self.replay(0.44)
        self.assertEqual(action.kind, "hold")
        self.assertIn("excursion", action.reason)
        self.assertEqual(controller.snapshot()["dry_steps"], 0)
        # The loop is still making progress, not thrashing: it ends 231 mm of
        # travel in, not at the pose it started the slowdown from.
        self.assertLess(scene.camera_z_to_marker(degrees) * 1000, 210.0)
        self.assertGreater(len(trace), 20)

    def forward_plan(self):
        """A depth-dominant plan at the recorded pose, built the same way the
        runner builds one, so the guard can be tested without depending on
        which region the marker happens to fall in first."""
        from eai_robot.control.visual_servo.camera_step import (
            ResponseModel, plan_camera_step,
        )
        from eai_robot.control.visual_servo.fixed_rule import (
            MEASURED_COUNT_OFFSETS,
        )
        scene = Scene(self.sim.start, self.sim.mount, depth_m=0.40)
        observation = scene.observe(self.sim.start)
        return observation, plan_camera_step(
            scene.fk, self.sim.start, self.sim.mount, np.array([0.0, 0.0, 0.01]),
            observation.corners_px, np.full(4, 0.40), K,
            counts_per_deg=self.sim.rates, max_joint_step_deg=12.0,
            max_request_counts=40.0, min_effective_counts=5.0,
            max_center_drift_px=60.0,
            gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS))

    def controller_for(self, observation):
        scene = Scene(self.sim.start, self.sim.mount, depth_m=0.40)
        return FixedRuleController(
            scene.fk, self.sim.mount, self.shipped(), self.sim.rates, K,
            reference_degrees=self.sim.start,
            origin_m=scene.camera_origin(self.sim.start))

    def test_the_guard_fires_only_when_the_depth_stops_closing(self):
        observation, plan = self.forward_plan()
        self.assertEqual(plan.goal.dominant, "depth")
        controller = self.controller_for(observation)
        # The encoder moves (so this is not a stall) and the marker does not
        # grow (so this is not approach): three of these halt the loop. The
        # executed pose must carry the offset on shoulder_lift, because that
        # is the axis the halt reason reports as spent -- with a pose equal
        # to the reference every axis reads 0.0 deg and argmax names the
        # first one, which says nothing about which joint ran out. Only the
        # joint angles change: moving the camera would also move the marker's
        # apparent size, and the guard measures depth from that size.
        lift_shift = np.zeros(len(self.sim.names))
        lift_shift[self.sim.names.index('shoulder_lift')] = 40.0
        after = Observation(observation.corners_px, observation.image_size,
                            self.sim.start + lift_shift)
        before_raw = {n: 500 for n in self.sim.names}
        after_raw = dict(before_raw)
        after_raw['shoulder_lift'] += 10
        for _ in range(self.shipped().dry_step_limit):
            report = controller.record(plan, observation, after,
                                       before_raw, after_raw)
        self.assertTrue(controller.halted)
        self.assertIn("前向步已无法再接近", controller.halted)
        self.assertIn("shoulder_lift", controller.halted)
        self.assertIn("forward step no longer closes distance", controller.halted)
        self.assertEqual(report.verdict, "dry_step")

    def test_one_productive_step_resets_the_counter(self):
        observation, plan = self.forward_plan()
        controller = self.controller_for(observation)
        before_raw = {n: 500 for n in self.sim.names}
        after_raw = dict(before_raw)
        after_raw['shoulder_lift'] += 10
        controller.record(plan, observation, observation, before_raw, after_raw)
        self.assertEqual(controller.snapshot()["dry_steps"], 1)
        # The same plan, now with a marker that actually grew.
        closer = Scene(self.sim.start, self.sim.mount, depth_m=0.38).observe(
            self.sim.start)
        controller.record(plan, observation, closer, before_raw, after_raw)
        self.assertEqual(controller.snapshot()["dry_steps"], 0)
        self.assertEqual(controller.halted, "")

    def test_the_guard_does_not_fire_on_a_reachable_start(self):
        # The guard is a no-op on every start the arm can actually finish, so it
        # cannot shorten a demo that would have reached the standoff.
        controller, action, _, _, _ = self.replay(0.38)
        self.assertEqual(action.kind, "reached", action.reason)
        self.assertEqual(controller.halted, "")
        self.assertLess(controller.snapshot()["dry_steps"], 3)

    def test_a_400mm_start_still_reaches(self):
        controller, action, _, _, _ = self.replay(0.40)
        self.assertEqual(action.kind, "reached", action.reason)

    def test_the_threshold_matches_a_measured_step(self):
        # One rule's step closes 8-10 mm. A threshold above a quarter of the
        # rule's own step would refuse a legal partial step.
        config = self.shipped()
        config.validate()
        self.assertLessEqual(config.min_approach_progress_mm, 0.25 * 10.0)
        self.assertGreater(config.min_approach_progress_mm, 0.0)
        self.assertGreaterEqual(config.dry_step_limit, 2)

    def test_the_start_depth_announcement_has_a_measured_basis(self):
        # 0.40 m is the measured reach with a 120 deg budget, not a preference.
        config = self.shipped()
        config.validate()
        self.assertAlmostEqual(config.max_start_depth_m, 0.40)
        controller, action, _, _, _ = self.replay(0.41)
        # Just past the announcement the run still stops on travel, which is
        # exactly what the startup warning says will happen.
        self.assertEqual(action.kind, "hold")
        self.assertTrue(action.reason)


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

    def test_folded_pose_searches_other_joint_sets_without_lowering_deadband(self):
        # 22:30:53 ending pose. The previous greedy active set deleted all
        # useful columns, despite legal alternatives. Rebuild a fronto-parallel
        # marker at its recorded centre/depth; actual corners were not logged.
        q = np.array([-5.281201143977398, -9.705886512646302,
                      -11.431303787575388, 68.07806136638918, -133.73742708702616])
        depth = .3039956931988343
        scene = Scene(q, self.mount, depth_m=depth)
        camera = scene.fk.forward_kinematics(q) @ self.mount
        offset = np.array([(320.25-K[0, 2])*depth/K[0, 0],
                           (225.-K[1, 2])*depth/K[1, 1], 0.])
        scene.points_base += camera[:3, :3] @ offset
        observation = scene.observe(q)
        model = self.priors().updated('wrist_flex', 1, 1.1708)
        model = model.updated('shoulder_pan', -1, .875).updated('wrist_roll', -1, .9583)
        plan = plan_camera_step(
            scene.fk, q, self.mount, [0., 0., .01], observation.corners_px,
            np.full(4, depth), K, counts_per_deg=self.rates, gains=model,
            max_joint_step_deg=12., max_request_counts=40., min_effective_counts=5.,
            max_center_drift_px=20.)
        self.assertIn('active_set_fallback', plan.notes)
        self.assertGreater(plan.predicted_translation_m[2], 0.)
        self.assertGreater(plan.predicted_image_change[2], 0.)
        self.assertGreater(plan.achieved_fraction, .35)
        self.assertLessEqual(np.max(np.abs(plan.requested_counts)), 40)
        self.assertLessEqual(np.max(np.abs(plan.requested_counts)/self.rates), 12.)
        for request, expected, name in zip(plan.requested_counts, plan.expected_counts, self.names):
            if request:
                self.assertGreaterEqual(abs(expected), 5.)
                self.assertGreaterEqual(abs(request), model.minimum_request(name, request))

    def test_capped_reverse_candidate_is_not_accepted_as_forward(self):
        # The 22:24 log accepted negative progress under cap_limited. Simulate
        # that solver result at every scale so it cannot be waved through.
        forward = self.plan([0, 0, .01])
        wrong = (-forward.requested_counts, -forward.expected_counts, np.zeros(5, dtype=bool),
                 ['step:scaled_to_joint_limit'])
        with patch('eai_robot.control.visual_servo.camera_step._solve_counts',
                   return_value=wrong):
            with self.assertRaises(StepInfeasible):
                self.plan([0, 0, .01])

    def test_disabling_escalation_keeps_smaller_step_candidates(self):
        forward = self.plan([0, 0, .01])
        solved = (forward.requested_counts, forward.expected_counts,
                  np.zeros(5, dtype=bool), [])
        with patch('eai_robot.control.visual_servo.camera_step._solve_counts',
                   side_effect=[None, solved]) as solve:
            plan = self.plan([0, 0, .01], allow_escalation=False)
        self.assertEqual(solve.call_count, 2)
        self.assertEqual(plan.goal_scale, .8)

    def priors(self):
        return ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS)

    def assert_requests_clear_the_offset(self, plan, model):
        commanded = 0
        for name, requested, expected in zip(self.names, plan.requested_counts,
                                             plan.expected_counts):
            if requested == 0:
                continue
            commanded += 1
            self.assertGreaterEqual(
                abs(int(requested)), model.minimum_request(name, requested),
                f"{name} requested {requested}, below its own offset")
            self.assertGreaterEqual(
                abs(expected), OBSERVED_MOTION_COUNTS,
                f"{name} predicted {expected}, below what the guard can observe")
            # The inversion is exact: the offset is not a slope, so the request
            # exceeds the predicted motion by exactly the offset.
            self.assertAlmostEqual(abs(int(requested)) - abs(expected),
                                   model.offset(name, requested), delta=0.51)
        return commanded

    def test_forward_requests_clear_the_measured_offset(self):
        plan = self.plan([0, 0, 0.01], gains=self.priors())
        self.assertGreaterEqual(self.assert_requests_clear_the_offset(plan, self.priors()), 2)

    def test_vertical_align_requests_clear_the_measured_offset(self):
        # The step that timed out three times is a vertical align, so it is
        # planned here at the pixel goals that produced it (30 to 50 px).
        model = self.priors()
        planned = 0
        for pixels in (-50.0, -43.0, -31.0, 31.0, 43.0, 50.0):
            try:
                plan = plan_axis_step(
                    self.scene.fk, self.start, self.mount,
                    self.observation.corners_px, np.full(4, 0.30), K,
                    axis=1, pixels=pixels, requested_delta_m=[0, 0.01, 0],
                    counts_per_deg=self.rates, gains=model,
                    max_joint_step_deg=5.0, max_request_counts=18.0,
                    min_effective_counts=5.0)
            except StepInfeasible:
                # A refusal is an acceptable answer; a step the hardware cannot
                # answer is not. Only silence below the threshold is forbidden.
                continue
            planned += self.assert_requests_clear_the_offset(plan, model)
        self.assertGreater(planned, 0, "no vertical step was plannable at all")

    def test_zero_response_teaches_the_offset_not_the_gain(self):
        from dataclasses import replace
        model = self.priors()
        controller = FixedRuleController(
            self.scene.fk, self.mount,
            FixedRuleConfig(focal_px=K[0, 0], marker_length_m=MARKER_M),
            self.rates, K, reference_degrees=self.start,
            origin_m=self.scene.camera_origin(self.start), gains=model)
        # The exact failing command: seven negative wrist counts, nothing moves.
        plan = replace(self.plan([0, 0, 0.01], gains=model),
                       requested_counts=np.array([0, 0, 0, -7, 0]))
        before = {name: 400 for name in self.names}
        report = controller.record(plan, self.observation, self.observation,
                                   before, dict(before))
        self.assertEqual(report.actual_counts["wrist_flex"], 0)
        self.assertAlmostEqual(controller.gains.offset("wrist_flex", -1), 6.75,
                               places=3)
        self.assertEqual(controller.gains.gain("wrist_flex", -1), 1.0)
        self.assertIn("wrist_flex", report.offset_updates)

    def test_response_model_is_bounded_and_immutable(self):
        from eai_robot.control.visual_servo.camera_step import MIN_GAIN
        model = ResponseModel()
        updated = model.updated("wrist_flex", -1, 0.05)
        self.assertEqual(model.entries, ())
        self.assertGreaterEqual(updated.gain("wrist_flex", -1), MIN_GAIN)
        self.assertEqual(model.gain("wrist_flex", -1), 1.0)


class MeasuredCountOffsetTest(unittest.TestCase):
    """The eight wrist steps measured on 2026-10-18, as a model specification.

    Every value below is a (requested, observed) pair from
    ``local_artifacts/experiments/20261018_*_fixed-rule.jsonl``. They are the
    reason the response model carries an offset: the same axis, in the same pose,
    delivered 8 counts for a 14-count request and 0 for a 7-count one.
    """

    # (requested counts, observed counts) on wrist_flex, negative direction
    # unless noted. Three of these are the timeouts that stopped the loop.
    MEASURED = (
        (-7, 0), (-7, 0), (-8, -2), (-11, -5),
        (-14, -8), (-15, -9), (-16, -10), (5, 4),
    )

    def setUp(self):
        self.model = ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS)

    def predict(self, requested):
        vector = np.zeros(5)
        vector[3] = requested
        return float(self.model.predict(vector)[3])

    def test_every_measured_step_is_reproduced_within_one_count(self):
        for requested, observed in self.MEASURED:
            predicted = self.predict(requested)
            self.assertLessEqual(
                abs(predicted - observed), 1.0,
                f"requested {requested}, observed {observed}, predicted {predicted}")

    def test_a_ratio_cannot_explain_a_step_that_produces_nothing(self):
        # One gain fitted to the 14-count step is 0.57, which predicts 4 counts
        # for a 7-count request -- more than twice the guard's two-count floor,
        # for a step the hardware answers with zero. The offset model predicts
        # half a count there and needs no separate explanation.
        ratio = 8 / 14
        self.assertGreater(ratio * 7, 3.5)
        self.assertLess(abs(self.predict(-7)), 1.0)

    def test_minimum_request_pays_the_offset_and_the_guard_floor(self):
        self.assertAlmostEqual(
            self.model.minimum_request("wrist_flex", -1),
            6.5 + OBSERVED_MOTION_COUNTS)
        # All three timeouts were a 7-count negative wrist request.
        self.assertLess(7, self.model.minimum_request("wrist_flex", -1))
        # An axis nobody has measured keeps a neutral floor, so the model can be
        # introduced without changing the default behaviour of every caller.
        self.assertEqual(self.model.minimum_request("shoulder_pan", 1),
                         OBSERVED_MOTION_COUNTS)

    def test_offset_updates_keep_the_gain_and_vice_versa(self):
        moved = self.model.updated("wrist_flex", -1, 0.9)
        self.assertEqual(moved.describe_offsets(), self.model.describe_offsets())
        shifted = self.model.with_offset("shoulder_pan", 1, 12.0)
        self.assertEqual(shifted.describe(), self.model.describe())
        self.assertEqual(shifted.offset("shoulder_pan", 1), 12.0)

    def test_an_unmeasured_axis_keeps_a_two_count_floor(self):
        # Documented limit, and the reason it is not papered over: shoulder_pan
        # was never commanded in the 2026-10-18 runs, so nobody knows what it
        # absorbs. If it has a threshold like the wrist's, a six-count pan request
        # will still be sent and will still time out. The model describes three
        # measured axes, not the arm.
        self.assertEqual(self.model.minimum_request("shoulder_pan", -1),
                         OBSERVED_MOTION_COUNTS)
        self.assertEqual(self.model.offset("shoulder_pan", -1), 0.0)


class DiagonalErrorTest(unittest.TestCase):
    """The frame that ended the 17:24 run: the marker is off *both* axes.

    Recorded observation ``20261018_172452``: centre (295.0, 266.25), side
    49.55 px, depth 426 mm at joints [-8.90, -74.12, 52.45, 39.30, -77.47]. The
    residual is (25.0, -26.25) px, so the vertical axis wins the "dominant" test
    by 1.25 px while the horizontal axis is just as far off.

    Every joint that can move the marker vertically also drags it sideways by
    43-48 % of the vertical motion, because at this wrist roll the camera's
    image Jacobian is rotated about 25 degrees. The old cross-axis rule measured
    that sideways motion against the *vertical* goal alone, so it refused all
    four usable joints, at every escalation, and halted the loop with a 36 px
    error and a legal step on the table.
    """

    # Recorded joint angles for the pose that refused the step.
    POSE = np.array([-8.902596214133311, -74.1176788238445, 52.449511495934146,
                     39.29960815241559, -77.4698330078938])
    CENTRE = np.array([295.0, 266.25])
    DEPTH_M = 0.426

    def setUp(self):
        from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
        calibration, self.mount = load_fixtures()
        self.names = ARM_JOINT_NAMES
        self.rates = counts_per_degree(calibration)
        self.scene = Scene(self.POSE, self.mount, depth_m=self.DEPTH_M)
        self.base = self.scene.observe(self.POSE)
        self.observation = self.scene.observe(self.POSE, self.CENTRE - self.base.center_px)
        self.error = np.array([320.0, 240.0]) - self.observation.center_px

    def plan(self, cross_error_px=0.0, **kwargs):
        from eai_robot.control.visual_servo.camera_step import camera_delta_goal
        delta = region_delta("down", 0.010)
        depths = np.full(4, self.DEPTH_M)
        goal = camera_delta_goal(self.observation.corners_px, depths, K, delta)
        values = dict(max_joint_step_deg=5.0, max_request_counts=18.0,
                      min_effective_counts=5.0)
        values.update(kwargs)
        return plan_axis_step(
            self.scene.fk, self.POSE, self.mount, self.observation.corners_px,
            depths, K, axis=1, pixels=goal.dv_px, requested_delta_m=delta,
            counts_per_deg=self.rates, cross_error_px=cross_error_px,
            gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS), **values)

    def test_the_recorded_frame_is_off_both_axes(self):
        # Guards the test itself: the horizontal error has to be as large as the
        # vertical one, otherwise the scenario is not the one that failed.
        self.assertAlmostEqual(self.error[0], 25.0, delta=0.1)
        self.assertAlmostEqual(self.error[1], -26.25, delta=0.1)

    def test_a_diagonal_error_justifies_crossing_the_other_axis(self):
        # The sideways motion this joint adds is not drift: it is the correction
        # the horizontal axis is still owed, in the direction it is owed.
        plan = self.plan(cross_error_px=float(self.error[0]))
        shift = np.asarray(plan.predicted_image_change, dtype=float)[:2]
        self.assertEqual(np.sign(shift[0]), np.sign(self.error[0]))
        before = float(np.linalg.norm(self.error))
        after = float(np.linalg.norm(self.error - shift))
        self.assertGreater(before, 30.0)
        self.assertLess(after, 0.5 * before, f"{before:.1f} -> {after:.1f} px")
        # The step still has to be something the arm can execute.
        counts = dict(zip(self.names, plan.requested_counts))
        self.assertNotEqual(counts["shoulder_lift"], 0)

    def test_the_coupling_limit_is_above_the_arms_own_coupling(self):
        # The default is not a preference, it is a bracket. Below the coupling of
        # every usable axis the loop halts with a legal step available; above the
        # coupling of an axis that mostly moves sideways it would command a
        # diagonal step and call it a vertical correction.
        from eai_robot.control.visual_servo.camera_step import image_task_matrix
        from eai_robot.control.visual_servo.joint_adapter import camera_joint_jacobian
        task = image_task_matrix(self.observation.corners_px,
                                 np.full(4, self.DEPTH_M), K)
        _, jacobian = camera_joint_jacobian(self.scene.fk, self.POSE, self.mount)
        per_count = (task @ jacobian) * (np.pi / 180.0) / self.rates
        share = {name: abs(per_count[0, index]) / abs(per_count[1, index])
                 for index, name in enumerate(self.names)}
        usable = ("shoulder_lift", "elbow_flex", "wrist_flex")
        for name in usable:
            self.assertLess(max(share[name] for name in usable), 0.60,
                            f"{name} coupling {share[name]:.2f} exceeds the limit")
        for name in ("shoulder_pan", "wrist_roll"):
            self.assertGreater(share[name], 0.60,
                               f"{name} moves mostly sideways at {share[name]:.2f}")

    def test_a_vertical_error_is_served_by_an_axis_that_moves_the_marker_down(self):
        # With nothing owed on the other axis the same frame is still plannable,
        # and the joint chosen is one whose sideways motion the next step can
        # afford -- never the pan, whose sideways motion is twice its vertical.
        plan = self.plan(cross_error_px=0.0)
        counts = dict(zip(self.names, plan.requested_counts))
        self.assertEqual(counts["shoulder_pan"], 0)
        self.assertEqual(counts["wrist_roll"], 0)
        self.assertLess(plan.predicted_image_change[1], 0.0)

    def test_the_controller_passes_the_other_axis_error(self):
        # The planner change is worthless if the loop still hands it a single
        # axis. This is the frame that halted; it must now move.
        controller = FixedRuleController(
            self.scene.fk, self.mount,
            FixedRuleConfig(focal_px=K[0, 0], marker_length_m=MARKER_M,
                            max_joint_step_deg=5.0, max_joint_excursion_deg=70.0,
                            max_travel_mm=250.0),
            self.rates, K, reference_degrees=self.POSE,
            origin_m=self.scene.camera_origin(self.POSE),
            gains=ResponseModel(offset_entries=MEASURED_COUNT_OFFSETS))
        action = controller.decide(self.observation)
        self.assertEqual(action.kind, "move", action.reason)
        self.assertEqual(action.region, "down")
        self.assertFalse(controller.halted)

    def test_the_refusal_names_the_rule_that_fired(self):
        # A refusal is the loop's last word before it holds, so it has to say
        # which constraint bound. Four copies of "no joint reaches the goal
        # outside deadband" -- one per escalation -- say nothing.
        with self.assertRaises(StepInfeasible) as caught:
            self.plan(cross_error_px=0.0, max_joint_step_deg=0.02)
        message = str(caught.exception)
        self.assertNotIn("no joint reaches the goal outside deadband", message)
        # Bodies read Chinese-first, as the operator asked; the joint keeps its
        # canonical English identifier so a log line still greps. ``单步`` is the
        # per-step cap and ``°`` its unit -- asserting the old English ``step``
        # and ``deg`` would only re-test the localization, not the rule.
        self.assertIn("shoulder_lift 单步", message)
        self.assertIn("°", message)
        # The header carries the English gloss the operator reads second.
        self.assertIn("no single joint realizes the requested image shift", message)
        # Escalation reproduces the same rules with larger numbers, so identical
        # rule sets share one entry instead of repeating the whole line.
        self.assertIn("scales 1.35, 1.70", message)
        self.assertEqual(message.count("shoulder_lift"), 3)


class FakeBus:
    """Motor bus stand-in that applies the measured defects on every goal write.

    ``Present_Position`` moves only when a write leaves the five-count deadband,
    then loses the two-count arrival residual, which is exactly the behaviour the
    round-1 log shows. ``Goal_Position`` echoes the write so the guard's readback
    check still has something real to compare.

    ``deadbands`` overrides the threshold per axis. The offset model has a prior
    for three axes only, so a *global* threshold is a stronger claim than any
    measurement supports: applying it to ``shoulder_pan`` tests the driver, not
    the model.
    """

    def __init__(self, raw, deadband=5, residual=2, gains=None, deadbands=None):
        self.present = {name: int(value) for name, value in raw.items()}
        self.goal = dict(self.present)
        self.deadband = deadband
        self.deadbands = dict(deadbands or {})
        self.residual = residual
        self.gains = gains or {}
        self.writes = []
        self.port_handler = types.SimpleNamespace(clearPort=lambda: None)

    def threshold(self, name):
        return self.deadbands.get(name, self.deadband)

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
            if abs(delta) < self.threshold(name):
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
    def run_runner(cls, scene, bus=None, args=None, gains=None, runtime=None):
        from eai_robot.applications.visual_servo import fixed_rule_servo
        from eai_robot.control.execution.feedback import (
            arm_degrees_from_raw, read_raw_motors,
        )
        args = args or runner_args()
        bus = bus or FakeBus(RAW, gains=gains)
        runtime = runtime or FakeRuntime(scene.fk, cls.mapper, bus)
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
        self.assertEqual(runtime.hold_reason, "已到停靠（standoff reached）")

    def test_runner_never_exceeds_the_declared_request_cap(self):
        _, _, _, events, _ = self.shared
        plans = [event["plan"] for event in events if event["event"] == "plan"]
        self.assertTrue(plans, "the loop never planned a step")
        for plan in plans:
            for name, value in plan["requested_counts"].items():
                self.assertLessEqual(abs(value), 18,
                                     f"{name}={value} exceeded the request cap")

    def test_explicit_short_step_budget_is_logged_and_stops_the_runner(self):
        args = runner_args(rule_max_steps=1)
        _, runtime, _, events, _ = self.run_runner(self.scene(), args=args)
        self.assertEqual(events[0]['max_steps'], 1)
        self.assertEqual(sum(e['event'] == 'plan' for e in events), 1)
        self.assertEqual(events[-2]['event'], 'hold')
        self.assertIn('step budget', events[-2]['reason'])
        self.assertIn('step budget', runtime.hold_reason)

    def test_one_approach_allows_alignment_then_ends_after_first_forward_step(self):
        _, runtime, _, events, _ = self.run_runner(
            self.scene(), args=runner_args(rule_one_approach=True))
        plans = [e for e in events if e['event'] == 'plan']
        self.assertEqual(plans[0]['phase'], 'align')
        self.assertEqual(plans[-1]['phase'], 'approach')
        self.assertEqual(sum(e['phase'] == 'approach' for e in plans), 1)
        self.assertIn('single approach complete', runtime.hold_reason)
        self.assertTrue(events[0]['one_approach'])

    def test_unresponsive_axis_falls_back_without_larger_requests(self):
        raw = dict(zip(self.names, [508, 325, 631, 327, 157]))
        start = np.array([self.mapper.raw_to_urdf_degrees(n, raw[n]) for n in self.names])
        scene = Scene(start, self.mount, depth_m=.312)
        camera = scene.fk.forward_kinematics(start) @ self.mount
        offset = np.array([(370-K[0, 2])*.312/K[0, 0],
                           (361-K[1, 2])*.312/K[1, 1], 0.])
        scene.points_base += camera[:3, :3] @ offset
        class NoResponseBus(FakeBus):
            failed = None

            def sync_write(self, register, values, **kwargs):
                if self.failed is None and len(values) == 1:
                    name, value = next(iter(values.items()))
                    if abs(value - self.present[name]) > 3:
                        self.failed = (name, 1 if value > self.present[name] else -1)
                        self.deadbands[name] = 10000
                return super().sync_write(register, values, **kwargs)

        bus = NoResponseBus(raw)
        args = runner_args(rule_one_approach=True, rule_max_steps=2,
                           max_joint_step_deg=12., rule_max_request_counts=40.,
                           max_joint_excursion_deg=120., max_travel_mm=300.)
        _, runtime, _, events, _ = self.run_runner(scene, bus=bus, args=args)
        fallback = next(i for i, e in enumerate(events)
                        if e['event'] == 'alignment_no_response')
        name, sign = bus.failed
        self.assertEqual(set(events[fallback]['requested_counts']), {name})
        self.assertEqual(events[fallback]['peak_motion'], {name: 0})
        later = [e for e in events[fallback+1:] if e['event'] == 'plan'
                 and e['phase'] == 'align']
        self.assertTrue(later)
        self.assertTrue(all(e['plan']['requested_counts'][name] * sign <= 0
                            for e in later))
        self.assertTrue(any(e['event'] == 'motion' for e in events[fallback+1:]))
        # R in the same device session carries the failed direction forward.
        self.assertIn((name, sign), runtime._alignment_unresponsive)
        _, _, _, retry_events, _ = self.run_runner(
            scene, bus=bus, runtime=runtime, args=runner_args(
                rule_max_steps=1, max_joint_step_deg=12.,
                rule_max_request_counts=40., max_joint_excursion_deg=120.,
                max_travel_mm=300.))
        for e in retry_events:
            if e['event'] == 'plan' and e['phase'] == 'align':
                self.assertLessEqual(e['plan']['requested_counts'][name] * sign, 0)

    def test_one_approach_does_not_reacquire_or_repeat_a_lost_forward_step(self):
        from eai_robot.applications.visual_servo.joint_center import (
            BoundedJointMotion, TargetLostError,
        )
        original = BoundedJointMotion.move

        def lose_on_forward(guard, target):
            changed = sum(abs(target[n] - guard.runtime.robot.bus.present[n]) > 3
                          for n in self.names)
            if changed > 1:
                raise TargetLostError('synthetic forward target loss')
            return original(guard, target)

        with patch.object(BoundedJointMotion, 'move', new=lose_on_forward):
            _, runtime, _, events, _ = self.run_runner(
                self.scene(), args=runner_args(rule_one_approach=True))
        self.assertEqual(sum(e['event'] == 'plan' and e['phase'] == 'approach'
                             for e in events), 1)
        self.assertIn('single approach interrupted', runtime.hold_reason)
        self.assertEqual(events[-2]['event'], 'hold')

    def test_initial_standoff_reports_reached_without_motion_or_user_stop(self):
        _, runtime, bus, events, _ = self.run_runner(self.scene(depth_m=.17))
        self.assertEqual(bus.writes, [])
        self.assertFalse(any(e['event'] == 'plan' for e in events))
        self.assertEqual(events[-1]['event'], 'reached')
        self.assertEqual(events[-1]['steps'], 0)
        self.assertIn('standoff reached', runtime.hold_reason)
        self.assertNotIn('USER STOP', runtime.hold_reason)

    def test_visual_residual_is_logged_without_shortening_the_step_budget(self):
        original = FixedRuleController.record

        def mismatched_record(controller, plan, before, after, *raw):
            shifted = Observation(after.corners_px + np.array([0., 80.]),
                                  after.image_size, after.joint_degrees)
            return original(controller, plan, before, shifted, *raw)

        with patch.object(FixedRuleController, 'record', new=mismatched_record):
            _, runtime, _, events, _ = self.run_runner(
                self.scene(), args=runner_args(rule_max_steps=2))
        self.assertEqual(sum(e['event'] == 'plan' for e in events), 2)
        self.assertIn('step budget', runtime.hold_reason)
        self.assertEqual(events[-2]['event'], 'hold')
        self.assertIn('step budget', events[-2]['reason'])
        feedback = next(e for e in events if e['event'] == 'feedback')
        self.assertEqual(feedback['verdict'], 'center_drift')
        for field in ('raw_before', 'raw_at_settle', 'raw_after',
                      'degrees_before', 'degrees_after', 'predicted_image_change'):
            self.assertIn(field, feedback)

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


class MeasuredThresholdRunnerTest(unittest.TestCase):
    """The whole loop against a bus that refuses small steps.

    The 2026-10-18 logs are the counter-example: the planner asked the wrist for
    seven counts, the shaft gave zero, and the guard timed out three times in a
    row. The same threshold is applied here to the three axes that were actually
    measured, with no threshold anywhere else, because a global deadband is a
    claim the log does not support.
    """

    MEASURED_THRESHOLD = {"shoulder_lift": 7, "wrist_flex": 7, "wrist_roll": 7}

    @classmethod
    def setUpClass(cls):
        from eai_robot.control.kinematics.joint_mapping import (
            ARM_JOINT_NAMES, SO101JointMapper,
        )
        calibration, cls.mount = load_fixtures()
        cls.mapper = SO101JointMapper(calibration)
        cls.names = ARM_JOINT_NAMES
        scene = RunnerIntegrationTest.scene.__func__(cls)
        cls.threshold = RunnerIntegrationTest.run_runner.__func__(
            cls, scene, bus=FakeBus(RAW, deadband=0, residual=2,
                                    deadbands=cls.MEASURED_THRESHOLD),
            args=runner_args(max_joint_excursion_deg=70.0, max_travel_mm=250.0))

    def test_no_arrival_timeout_against_the_measured_threshold(self):
        _, runtime, _, events, _ = self.threshold
        names = [event["event"] for event in events]
        self.assertNotIn("arrival_timeout", names)
        self.assertNotIn("Arrival not confirmed", runtime.hold_reason)
        self.assertGreaterEqual(len([name for name in names if name == "motion"]), 8)

    def test_the_measured_priors_reach_the_real_entry_point(self):
        # The offsets are only useful if the runner actually passes them to the
        # controller; a model that stays neutral would behave like the old one.
        _, _, _, events, _ = self.threshold
        feedback = [event for event in events if event["event"] == "feedback"]
        self.assertTrue(feedback)
        self.assertIn("wrist_flex-", feedback[0]["offsets"])
        self.assertEqual(feedback[0]["offsets"]["wrist_flex-"], 6.5)

    def test_every_request_to_a_measured_axis_clears_its_threshold(self):
        _, _, _, events, _ = self.threshold
        for event in events:
            if event["event"] != "plan":
                continue
            for name, value in event["plan"]["requested_counts"].items():
                if value and name in self.MEASURED_THRESHOLD:
                    self.assertGreaterEqual(
                        abs(value), self.MEASURED_THRESHOLD[name],
                        f"{name}={value} inside the measured deadband")


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
