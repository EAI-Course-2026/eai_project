import unittest
import numpy as np
from eai_robot.control.visual_servo.empirical import choose_step, motion_confirmed, commanded_joints, center_tolerance, joint_step_options


class JointCenterTest(unittest.TestCase):
    def test_target_wait_honours_runtime_deadline(self):
        import threading
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        import eai_robot.applications.visual_servo.joint_center as module

        runtime = SimpleNamespace(read_frame=Mock(
            side_effect=module.PreviewFrameUnavailable('no fresh frame')))
        detector = SimpleNamespace(detect=Mock())
        stop = SimpleNamespace(stop_requested=threading.Event())
        clock = iter((0.0, 0.5, 1.0))
        with patch.object(module.time, 'monotonic', side_effect=lambda: next(clock)), \
             patch.object(module.time, 'sleep'):
            with self.assertRaisesRegex(module.RuntimeLimitReached,
                                        'Runtime limit reached'):
                module.wait_for_target(runtime, detector, None, stop,
                                       deadline=0.75)
        detector.detect.assert_not_called()

    def test_probe_start_uses_marker_edge_margin_not_central_half(self):
        from eai_robot.applications.visual_servo.joint_center import target_has_probe_margin
        from eai_robot.vision.detection.ball import BallDetection
        target = lambda x, y: BallDetection(x, y, 20, 1200, 1, 1)
        # This is outside the old central half but leaves the complete marker
        # and a ten-percent probe margin inside a 640x480 frame.
        self.assertTrue(target_has_probe_margin((480, 640, 3), target(100, 100)))
        self.assertFalse(target_has_probe_margin((480, 640, 3), target(55, 100)))
        self.assertFalse(target_has_probe_margin((480, 640, 3), target(100, 425)))

    def test_wrist_never_inherits_small_base_steps(self):
        options=joint_step_options(15,12)
        self.assertEqual(options,([-15,-10,0,10,15],[-12,0,12]))
        for error in (np.array([0.,52.]),np.array([0.,120.])):
            step=choose_step(np.diag([7.,5.5]),error,lambda s:True,options)
            self.assertIsNotNone(step)
            self.assertEqual(abs(step[1]),12)

    def test_smaller_safe_step_is_used_when_ten_counts_crosses_bound(self):
        step=choose_step(np.diag([7.,5.5]),np.array([4.,-52.]),lambda s:s[1]>=-5)
        np.testing.assert_array_equal(step,[0,-5])
    def test_course_central_twenty_percent_region_is_rectangular(self):
        np.testing.assert_allclose(center_tolerance((480,640,3),30,.2),[64,48])
        np.testing.assert_allclose(center_tolerance((480,640,3),30,0),[30,30])
    def test_unselected_axis_encoder_jitter_does_not_create_micro_command(self):
        before = {'shoulder_pan': 517, 'wrist_flex': 430}
        target = {'shoulder_pan': 527, 'wrist_flex': 429}
        self.assertEqual(commanded_joints(before, target), {'shoulder_pan':527})
    def test_quantized_move_reduces_predicted_error(self):
        j = np.array([[-4., 0.], [0., 3.]])
        error = np.array([40., -30.])
        step = choose_step(j, error, lambda _: True)
        np.testing.assert_array_equal(step, [-10, -10])
        self.assertLess(np.linalg.norm(error-j@step), np.linalg.norm(error))

    def test_does_not_force_minimum_move_that_overshoots(self):
        self.assertIsNone(choose_step(np.eye(2)*10, np.array([2., 3.]), lambda _: True))

    def test_workspace_bound_filters_better_candidate(self):
        step = choose_step(np.eye(2), np.array([20., 20.]), lambda s: s[0] == 0)
        np.testing.assert_array_equal(step, [0, 10])
        self.assertIsNone(choose_step(np.eye(2), np.array([20., 20.]), lambda _: False))

    def test_no_motion_is_never_arrival(self):
        self.assertFalse(motion_confirmed({'a':640}, {'a':639}, {'a':640}))
        self.assertFalse(motion_confirmed({'a':640}, {'a':645}, {'a':640}))
        self.assertFalse(motion_confirmed({'a':640}, {'a':645}, {'a':639}))

    def test_actual_movement_and_strictly_smaller_tolerance_required(self):
        self.assertTrue(motion_confirmed({'a':640}, {'a':645}, {'a':643}))
        self.assertTrue(motion_confirmed({'a':525}, {'a':515}, {'a':518}))
        self.assertFalse(motion_confirmed({'a':640}, {'a':645}, {'a':641}))
        self.assertFalse(motion_confirmed({'a':640,'b':100}, {'a':645,'b':105}, {'a':645,'b':100}))

class JointCenterRuntimeTest(unittest.TestCase):
    def exercise_runtime(self, stale=False, residual=3, target_motion=False, hold_offset=0):
        import tempfile
        import threading
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        import eai_robot.applications.visual_servo.joint_center as module
        from eai_robot.vision.detection.ball import BallDetection
        names = module.ARM_JOINT_NAMES
        state = dict(zip(names, [500, 300, 500, 400, 300]))
        initial_goals = {name: value + hold_offset for name, value in state.items()}
        goal_register = dict(initial_goals)
        writes = []
        moves = [0]
        clock = [0.]
        stop = SimpleNamespace(stop_requested=threading.Event())
        def detection():
            return BallDetection(260+4*(state['shoulder_pan']-500) - (100 if target_motion and moves[0]>=5 else 0),
                                 200+3*(state['wrist_flex']-400), 30, 1000, 1, 1)
        def send(register, goals, **kwargs):
            self.assertEqual(register, 'Goal_Position')
            writes.append(dict(goals))
            goal_register.update(goals)
            if len(writes) > 1 and all(goals[n] == initial_goals[n] for n in goals):
                return  # timeout restores the old load-bearing goal
            if any(goal != state[name] for name, goal in goals.items()): moves[0] += 1
            for name, goal in goals.items():
                if goal == state[name]: continue  # measured hold after a partial response
                self.assertGreater(abs(goal-state[name]), 3)
                sign = np.sign(goal-state[name])
                state[name] = int(goal - residual*sign)
            # Realistic one-count unselected-axis drift, on the first probe.
            if len(writes) == 1:
                state['wrist_flex'] += 1
        def read(register, name, **kwargs):
            if register == 'Torque_Enable': return 1
            if register == 'Status': return 0
            return goal_register[name]
        bus = SimpleNamespace(sync_write=send, read=read)
        runtime = SimpleNamespace(
            robot=SimpleNamespace(bus=bus), _require_mapper=lambda: object(),
            fk=SimpleNamespace(forward_kinematics=lambda q: np.eye(4)),
            read_frame=Mock(side_effect=RuntimeError('stale frame')) if stale else lambda: np.zeros((480,640,3),np.uint8),
        )
        detector = SimpleNamespace(detect=lambda frame: (detection(),None))
        calls = [0]
        def show(*args):
            calls[0] += 1
            if moves[0] >= 6: stop.stop_requested.set()
        args = SimpleNamespace(joint_margin_deg=2, max_joint_step_deg=5,
                               max_travel_mm=40,torque_settle_s=0,
                               feature_samples=1,max_runtime_s=6,
                               deadband_px=30,stable_frames=3)
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(module,'ROOT',Path(directory)), \
             patch.object(module,'read_raw_motors',side_effect=lambda *a: dict(state)), \
             patch.object(module,'arm_degrees_from_raw',side_effect=lambda m,r: np.array([r[n]*.3 for n in names])), \
             patch.object(module,'ensure_execution_start_is_safe'), \
             patch.object(module,'raw_outside_calibration',return_value=[]), \
             patch.object(module.time,'sleep'), \
             patch.object(module.time,'monotonic',side_effect=lambda: clock.__setitem__(0,clock[0]+.06) or clock[0]), \
             patch('eai_robot.applications.visual_servo.ball.median_detection',side_effect=lambda *a: detection()):
            try:
                module.run_joint_center(runtime,detector,args,stop,SimpleNamespace(show=show))
            except RuntimeError:
                pass
            records = '\n'.join(p.read_text() for p in Path(directory).rglob('*.jsonl'))
        return writes, records

    def test_target_motion_does_not_reverse_learned_motor_direction(self):
        import json
        writes, records = self.exercise_runtime(target_motion=True)
        self.assertIn('"model_updated": false',records)
        commands = [json.loads(line) for line in records.splitlines()
                    if json.loads(line)['event'] == 'command']
        self.assertGreater(commands[-1]['goal']['shoulder_pan'],
                           commands[-1]['before']['shoulder_pan'])

    def test_stale_frame_prevents_motor_write(self):
        writes, records = self.exercise_runtime(stale=True)
        self.assertEqual(writes, [])
        self.assertIn('stale frame', records)

    def test_probes_returns_and_control_use_actual_feedback_with_three_count_residual(self):
        writes, records = self.exercise_runtime()
        self.assertGreaterEqual(len(writes),5)
        self.assertIn('calibration',records)
        self.assertIn('response',records)
        self.assertNotIn('arrival_timeout',records)

    def test_five_count_residual_is_partial_feedback_not_false_arrival(self):
        writes, records = self.exercise_runtime(residual=5)
        self.assertIn('partial_response', records)
        self.assertIn('calibration', records)
        self.assertIn('response', records)
        self.assertNotIn('arrival_timeout', records)

    def test_no_response_stops_before_learning_or_continuous_control(self):
        writes, records = self.exercise_runtime(residual=10)
        self.assertEqual(len(writes),2)
        self.assertNotEqual(writes[0], writes[1])
        self.assertIn('hold_goals', records)
        self.assertIn('actual_counts', records)
        self.assertIn('arrival_timeout',records)
        self.assertNotIn('calibration',records)

    def test_timeout_restores_previous_load_bearing_goal_instead_of_sagged_pose(self):
        import json
        writes, records = self.exercise_runtime(residual=9, hold_offset=8)
        timeout = next(json.loads(line) for line in records.splitlines()
                       if json.loads(line)['event'] == 'arrival_timeout')
        self.assertEqual(writes[-1], timeout['hold_goals'])
        self.assertEqual(timeout['hold_goals'], timeout['previous_goals'])
        self.assertNotEqual(timeout['hold_goals'],
                            {name: timeout['actual'][name] for name in timeout['goal']})
        self.assertTrue(all(v == 1 for v in timeout['torque_enabled'].values()))
        self.assertTrue(all(abs(n) == 1 for n in timeout['actual_counts'].values()))
        self.assertNotIn('calibration', records)

class ForwardMotionEvidenceTest(unittest.TestCase):
    def test_detection_is_paused_during_bounded_motion(self):
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        import threading
        import eai_robot.applications.visual_servo.joint_center as module
        names = module.ARM_JOINT_NAMES
        before = dict(zip(names, [481, 341, 588, 358, 252]))
        state = dict(before)
        goals = dict(before)
        writes, events = [], []

        def write(register, values, **kwargs):
            writes.append(dict(values))
            goals.update(values)
            if len(writes) == 1:
                state.update(values)

        def read(register, name, **kwargs):
            return {'Goal_Position': goals[name], 'Torque_Enable': 1,
                    'Status': 0}[register]

        runtime = SimpleNamespace(
            robot=SimpleNamespace(bus=SimpleNamespace(sync_write=write, read=read)),
            read_frame=Mock(return_value=np.zeros((2, 2, 3))),
            fk=SimpleNamespace(forward_kinematics=lambda q: np.eye(4)))
        detector = SimpleNamespace(detect=Mock(return_value=(object(), None)))
        guard = module.BoundedJointMotion(
            runtime, detector, object(), SimpleNamespace(), np.zeros(5),
            np.zeros(3), lambda event, **data: events.append(dict(event=event, **data)),
            None, SimpleNamespace(stop_requested=threading.Event()),
            joints=('wrist_flex',), position_of=lambda q: np.zeros(3))
        target = dict(before, wrist_flex=370)
        clock = [0.0]
        with patch.object(guard, 'reject_reason', return_value=None), \
             patch.object(module, 'read_raw_motors', side_effect=lambda *a: dict(state)), \
             patch.object(module, 'arm_degrees_from_raw', return_value=np.zeros(5)), \
             patch.object(module.time, 'monotonic',
                          side_effect=lambda: clock.__setitem__(0, clock[0] + .11) or clock[0]):
            actual = guard.move(target)
        self.assertEqual(actual['wrist_flex'], 370)
        self.assertEqual(len(writes), 2)
        self.assertEqual(writes[0], {'wrist_flex': 370})
        self.assertEqual(writes[1], {'wrist_flex': 370})
        runtime.read_frame.assert_called_once_with()
        detector.detect.assert_called_once()
        samples = [e for e in events if e['event'] == 'motion_sample']
        self.assertTrue(samples)
        self.assertTrue(all(e['target_check'] == 'deferred_until_settled'
                            for e in samples))
        self.assertFalse(any(e['event'] == 'target_lost_hold' for e in events))
        snapshots = [e for e in events if e['event'] == 'motor_state']
        self.assertEqual([e['stage'] for e in snapshots],
                         ['before_command', 'after_settled_hold'])
        for snapshot in snapshots:
            self.assertEqual(set(snapshot['goals']), set(names))
            self.assertTrue(all(v == 1 for v in snapshot['torque_enabled'].values()))


class GuardStepToleranceTest(unittest.TestCase):
    """The guard must not reject a step it just allowed the planner to make.

    Both checks in ``BoundedJointMotion.move`` use ``reject_reason``. The
    pre-write one compares intent with intent and keeps the strict cap; the
    arrival poll compares a *measurement* with intent, and that measurement
    carries ARRIVAL_TOLERANCE counts of legitimate residual. Checking it
    against the strict cap is how the bench saw "Joint/workspace bound
    reached" while 0.16 counts below a 5 degree ceiling.
    """

    def guard(self, mapper=None, **overrides):
        from types import SimpleNamespace
        import eai_robot.applications.visual_servo.joint_center as module
        args = SimpleNamespace(joint_margin_deg=2.0, max_joint_step_deg=5.0,
                               max_travel_mm=400.0, max_joint_excursion_deg=120.0)
        for key, value in overrides.items():
            setattr(args, key, value)
        return module.BoundedJointMotion(
            None, None, mapper or SimpleNamespace(calibration=None), args,
            np.zeros(5), np.zeros(3), None, None, None,
            position_of=lambda q: np.zeros(3))

    def reject(self, guard, raw, reference, **kwargs):
        from unittest.mock import patch
        import eai_robot.applications.visual_servo.joint_center as module
        with patch.object(module, 'raw_outside_calibration', return_value=[]), \
             patch.object(module, 'ensure_execution_start_is_safe'), \
             patch.object(module, 'arm_degrees_from_raw',
                          side_effect=lambda m, r: np.array(r, dtype=float)):
            return guard.reject_reason(raw, reference, **kwargs)

    def test_a_mapper_without_calibration_does_not_crash_the_guard(self):
        # The test doubles in this file publish no calibration table; the
        # count-to-degree conversion must have a conservative fallback rather
        # than an AttributeError.
        guard = self.guard()
        self.assertIsNone(self.reject(guard, np.full(5, 5.5), np.zeros(5),
                                      step_tolerance=3.0))

    def test_the_arrival_residual_does_not_trip_the_strict_cap(self):
        guard = self.guard()
        over = np.full(5, 5.5)          # 0.5 deg past a 5.0 deg cap
        self.assertIsNotNone(self.reject(guard, over, np.zeros(5)))
        self.assertIsNone(self.reject(guard, over, np.zeros(5), step_tolerance=3.0))

    def test_a_step_far_past_the_cap_is_still_rejected_with_tolerance(self):
        # Widening by the arrival residual must not turn the cap into no cap.
        guard = self.guard()
        message = self.reject(guard, np.full(5, 40.0), np.zeros(5),
                              step_tolerance=3.0)
        self.assertIsNotNone(message)
        self.assertIn('shoulder_pan', message)
        self.assertIn('到位容差', message)

    def test_the_pre_write_message_does_not_pretend_to_allowance(self):
        guard = self.guard()
        message = self.reject(guard, np.full(5, 6.0), np.zeros(5))
        self.assertIn('单步', message)
        self.assertNotIn('到位容差', message)


class HoldHandoverTest(unittest.TestCase):
    def test_stationary_hold_is_read_only_and_faults_are_rejected(self):
        from unittest.mock import Mock, patch
        from types import SimpleNamespace
        import eai_robot.applications.visual_servo.joint_center as module
        raw = {n:100 for n in module.ARM_JOINT_NAMES}
        for fault in (None,'torque','status','goal','drift'):
            bus = Mock()
            def read(reg,name,**kw):
                if reg=='Torque_Enable': return 0 if fault=='torque' else 1
                if reg=='Status': return 1 if fault=='status' else 0
                return 110 if fault=='goal' else 100
            bus.read.side_effect=read
            robot=SimpleNamespace(bus=bus)
            poses = [raw]+[{n:103 for n in raw}]*5 if fault=='drift' else [raw]*6
            with patch.object(module,'read_raw_motors',side_effect=poses), patch.object(module.time,'sleep'):
                if fault:
                    with self.assertRaises(RuntimeError): module.verify_existing_hold(robot)
                else:
                    module.verify_existing_hold(robot)
            bus.write.assert_not_called()
            bus.sync_write.assert_not_called()
            bus.enable_torque.assert_not_called()
            bus.disable_torque.assert_not_called()


if __name__ == '__main__':
    unittest.main()
