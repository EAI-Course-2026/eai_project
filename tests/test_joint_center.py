import unittest
import numpy as np
from eai_robot.control.visual_servo.empirical import choose_step, motion_confirmed, commanded_joints, center_tolerance, joint_step_options


class JointCenterTest(unittest.TestCase):
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
    def exercise_runtime(self, stale=False, residual=3, target_motion=False):
        import tempfile
        import threading
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        import eai_robot.applications.visual_servo.joint_center as module
        from eai_robot.vision.detection.ball import BallDetection
        names = module.ARM_JOINT_NAMES
        state = dict(zip(names, [500, 300, 500, 400, 300]))
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
            return writes[-1][name]
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
        writes, records = self.exercise_runtime(target_motion=True)
        self.assertIn('"model_updated": false',records)
        self.assertGreater(writes[-1]['shoulder_pan'],writes[-2]['shoulder_pan'])

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
        self.assertEqual(len(writes),1)
        self.assertIn('arrival_timeout',records)
        self.assertNotIn('calibration',records)

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
