import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from eai_robot.applications.visual_servo.joint_center import wait_for_target


class ReacquisitionTest(unittest.TestCase):
    def test_loss_resets_consecutive_count_and_never_writes_motion(self):
        marker=object()
        observations=iter([None,marker,marker,None,marker,marker,marker])
        robot=Mock()
        runtime=SimpleNamespace(read_frame=lambda:np.zeros((2,2,3)),robot=robot)
        detector=SimpleNamespace(detect=lambda f:(next(observations),None))
        stop=SimpleNamespace(stop_requested=threading.Event())
        preview=Mock()
        _,detected=wait_for_target(runtime,detector,preview,stop,required_frames=3)
        self.assertIs(detected,marker)
        self.assertEqual(preview.show.call_count,7)
        self.assertEqual(robot.mock_calls,[])

    def test_user_stop_remains_effective_while_target_missing(self):
        runtime=SimpleNamespace(read_frame=lambda:np.zeros((2,2,3)),robot=Mock())
        detector=SimpleNamespace(detect=lambda f:(None,None))
        stop=SimpleNamespace(stop_requested=threading.Event())
        preview=SimpleNamespace(show=lambda *a:stop.stop_requested.set())
        with self.assertRaisesRegex(RuntimeError,'User stop'):
            wait_for_target(runtime,detector,preview,stop)
        self.assertEqual(runtime.robot.mock_calls,[])

    def test_camera_gap_requires_fresh_consecutive_detections_before_return(self):
        from eai_robot.applications.visual_servo.shared_camera import PreviewFrameUnavailable
        marker=object()
        frames=iter([np.zeros((2,2,3)),PreviewFrameUnavailable('stale')]+[np.zeros((2,2,3))]*3)
        def read():
            frame=next(frames)
            if isinstance(frame,Exception): raise frame
            return frame
        runtime=SimpleNamespace(read_frame=read,robot=Mock())
        detector=SimpleNamespace(detect=lambda f:(marker,None))
        preview=Mock()
        with patch('eai_robot.applications.visual_servo.joint_center.time.sleep'):
            _,detected=wait_for_target(runtime,detector,preview,SimpleNamespace(stop_requested=threading.Event()),3)
        self.assertIs(detected,marker)
        self.assertEqual(preview.show.call_count,5)
        self.assertIn('CAMERA WAIT',preview.show.call_args_list[1].args[2])
        self.assertEqual(runtime.robot.mock_calls,[])
