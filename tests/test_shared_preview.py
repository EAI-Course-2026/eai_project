"""Persistent preview adapter freshness and ownership checks without devices."""
from pathlib import Path
import struct
import tempfile
import time
import unittest

import cv2
import numpy as np

from eai_robot.applications.visual_servo.shared_camera import SharedPreviewCamera


class SharedPreviewTests(unittest.TestCase):
    def publish(self, path, age=0.):
        bgr = np.zeros((48, 64, 3), np.uint8)
        bgr[:, :, 2] = 255
        ok, jpeg = cv2.imencode('.jpg', bgr)
        self.assertTrue(ok)
        path.write_bytes(struct.pack('!d', time.monotonic()-age) + jpeg.tobytes())

    def test_reads_rgb_and_disconnect_does_not_remove_or_stop_publisher(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path)
            original = path.read_bytes()
            camera = SharedPreviewCamera(path)
            camera.connect()
            self.publish(path)
            rgb = camera.async_read()
            self.assertEqual(rgb.shape, (48, 64, 3))
            self.assertGreater(rgb[10, 10, 0], 250)
            camera.disconnect()
            self.assertTrue(path.exists())
            self.assertFalse(camera.is_connected)

    def test_stale_frames_rejected_before_control(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path, age=1.)
            camera = SharedPreviewCamera(path)
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                camera.connect()
            self.assertFalse(camera.is_connected)

    def test_connected_reader_rejects_stale_then_recovers_on_new_fresh_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'latest.frame'
            self.publish(path)
            camera=SharedPreviewCamera(path)
            camera.connect()
            self.publish(path,age=1.)
            with self.assertRaisesRegex(RuntimeError,'stale'):
                camera.async_read()
            self.assertTrue(camera.is_connected)
            self.publish(path)
            self.assertEqual(camera.async_read().shape,(48,64,3))
