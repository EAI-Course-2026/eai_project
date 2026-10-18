import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
from eai_robot.applications.visual_servo import aruco_preview


class HeadlessCameraTest(unittest.TestCase):
    def test_only_publishes_frames_without_creating_a_second_window(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'latest.frame'
            args=SimpleNamespace(camera_index=0,dictionary='DICT_4X4_50',marker_id=0,
                                 width=640,height=480,stream_file=path,headless=True)
            camera=Mock()
            camera.isOpened.return_value=True
            camera.read.side_effect=[(True,np.zeros((8,8,3),np.uint8)),(False,None)]
            with patch.object(aruco_preview,'parse_args',return_value=args), \
                 patch.object(aruco_preview,'ArucoMarkerDetector') as detector, \
                 patch.object(aruco_preview.cv2,'VideoCapture',return_value=camera), \
                 patch.object(aruco_preview.cv2,'imshow') as show, \
                 patch.object(aruco_preview.cv2,'waitKey') as wait:
                with self.assertRaisesRegex(RuntimeError,'stopped'):
                    aruco_preview.main()
                show.assert_not_called()
                wait.assert_not_called()
                detector.return_value.detect.assert_not_called()
            self.assertGreater(len(path.read_bytes()),8)
            camera.release.assert_called_once()
