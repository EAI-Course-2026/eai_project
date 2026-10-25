import io
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np
from eai_robot.applications.visual_servo.hold_preview import (
    overlay_label, wait_while_holding,
)
from eai_robot.applications.visual_servo.ball import PreviewWindow, StopState


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
            self.assertTrue(s.isascii())
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
            self.assertTrue(s.isascii())
            self.assertFalse(f.any())
            stop.emergency_requested.set()
        with patch('eai_robot.applications.visual_servo.hold_preview.time.sleep'):
            self.assertEqual(wait_while_holding(runtime,detector,SimpleNamespace(show=show),stop,
                                              stream=io.StringIO(),poll=lambda *a:([],[],[])),'emergency')

    def test_windows_console_does_not_use_socket_only_select(self):
        runtime,detector,stop=self.setup_runtime()
        shown=[]
        def show(*args):
            shown.append(1)
        with patch('eai_robot.applications.visual_servo.hold_preview._use_windows_console',
                   return_value=True), patch(
                       'eai_robot.applications.visual_servo.hold_preview._read_windows_console_line',
                       side_effect=[None, 'r']):
            result=wait_while_holding(runtime,detector,SimpleNamespace(show=show),stop,
                                      stream=io.StringIO(),poll=None)
        self.assertEqual(result,'r')
        self.assertEqual(len(shown),2)


class OverlayLabelTest(unittest.TestCase):
    """OpenCV overlay labels must stay ASCII."""

    def test_the_english_gloss_is_used_for_opencv(self):
        self.assertEqual(
            overlay_label('关节累计偏移预算已用尽（joint excursion budget 120 deg '
                          'reached）：已偏 122.9 度'),
            'joint excursion budg')

    def test_the_check_point_prefix_is_not_what_the_operator_reads(self):
        # The guard's arrival-poll sentence starts with "measured pose outside
        # permitted bounds" and then names the bound that actually fired.
        self.assertEqual(
            overlay_label('实测位姿越界（measured pose outside permitted bounds）：'
                          '行程预算已用尽（travel budget 300 mm reached）：相机已移动 '
                          '301.4 mm'),
            'travel budget 300 mm')

    def test_a_reason_without_a_gloss_is_passed_through(self):
        self.assertEqual(overlay_label('控制已暂停 CONTROL PAUSED'),
                         'CONTROL PAUSED')


class PreviewWindowKeyTest(unittest.TestCase):
    def test_tk_keys_only_update_the_owned_stop_state(self):
        stop = StopState()
        preview = object.__new__(PreviewWindow)
        preview._stop_state = stop
        preview._on_key(SimpleNamespace(keysym='r'))
        self.assertTrue(stop.resume_requested.is_set())
        preview._on_key(SimpleNamespace(keysym='space'))
        self.assertTrue(stop.emergency_requested.is_set())
        self.assertTrue(stop.stop_requested.is_set())
