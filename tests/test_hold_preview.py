import io
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from eai_robot.applications.visual_servo.hold_preview import wait_while_holding


class HoldPreviewTest(unittest.TestCase):
    def setup_runtime(self):
        stop=SimpleNamespace(emergency_requested=threading.Event(),resume_requested=threading.Event())
        frames=iter([np.full((2,2,3),n,np.uint8) for n in range(1,10)])
        runtime=SimpleNamespace(read_frame=lambda:next(frames),robot=Mock(),hold_reason='RANGE')
        detector=SimpleNamespace(detect=lambda f:(None,None))
        return runtime,detector,stop

    def test_waiting_for_input_continues_rendering_new_frames_without_motion(self):
        runtime,detector,stop=self.setup_runtime()
        shown=[]
        def show(f,d,s):
            shown.append(int(f[0,0,0]))
            self.assertIn('PAUSED',s)
            if len(shown)==3: stop.resume_requested.set()
        result=wait_while_holding(runtime,detector,SimpleNamespace(show=show),stop,
                                  stream=io.StringIO(),poll=lambda *a:([],[],[]))
        self.assertEqual(result,'r')
        self.assertEqual(shown,[1,2,3])
        self.assertEqual(runtime.robot.mock_calls,[])

    def test_terminal_eof_does_not_release_or_exit(self):
        runtime,detector,stop=self.setup_runtime()
        calls=[]
        def show(*a):
            calls.append(1)
            if len(calls)==3: stop.emergency_requested.set()
        with patch('eai_robot.applications.visual_servo.hold_preview.time.sleep'):
            result=wait_while_holding(runtime,detector,SimpleNamespace(show=show),stop,
                                     stream=io.StringIO(''),poll=lambda r,*a:(r,[],[]))
        self.assertEqual(result,'emergency')
        self.assertEqual(len(calls),3)
        self.assertEqual(runtime.robot.mock_calls,[])

    def test_camera_failure_displays_placeholder_instead_of_stale_image(self):
        runtime,detector,stop=self.setup_runtime()
        runtime.read_frame=Mock(side_effect=RuntimeError('stale'))
        def show(f,d,s):
            self.assertIn('CAMERA UNAVAILABLE',s)
            self.assertFalse(f.any())
            stop.emergency_requested.set()
        with patch('eai_robot.applications.visual_servo.hold_preview.time.sleep'):
            self.assertEqual(wait_while_holding(runtime,detector,SimpleNamespace(show=show),stop,
                                              stream=io.StringIO(),poll=lambda *a:([],[],[])),'emergency')
