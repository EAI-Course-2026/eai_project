"""Persistent preview adapter freshness and ownership checks without devices."""
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from eai_robot.applications.visual_servo import shared_camera
from eai_robot.applications.visual_servo.shared_camera import SharedPreviewCamera


class ManualClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class SharedPreviewTests(unittest.TestCase):
    def setUp(self):
        self.clock = ManualClock()
        clock_patch = patch.object(shared_camera, 'time', self.clock)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)

    def publish(self, path, age=0.):
        bgr = np.zeros((48, 64, 3), np.uint8)
        bgr[:, :, 2] = 255
        ok, jpeg = cv2.imencode('.jpg', bgr)
        self.assertTrue(ok)
        path.write_bytes(struct.pack('!d', self.clock.monotonic()-age) + jpeg.tobytes())

    def test_reads_rgb_and_disconnect_does_not_remove_or_stop_publisher(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path)
            camera = SharedPreviewCamera(path)
            camera.connect()
            self.clock.sleep(.01)
            self.publish(path)
            published = path.read_bytes()
            rgb = camera.async_read()
            self.assertEqual(rgb.shape, (48, 64, 3))
            self.assertGreater(rgb[10, 10, 0], 250)
            camera.disconnect()
            self.assertEqual(path.read_bytes(), published)
            self.assertFalse(camera.is_connected)

    def test_reader_retries_a_brief_windows_writer_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path)
            original_read = Path.read_bytes
            attempts = []
            def read(source):
                attempts.append(1)
                if len(attempts) == 1:
                    raise PermissionError('synthetic sharing violation')
                return original_read(source)
            camera = SharedPreviewCamera(path)
            with patch.object(Path, 'read_bytes', autospec=True, side_effect=read):
                camera.connect()
            self.assertTrue(camera.is_connected)
            self.assertEqual(len(attempts), 2)

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
            path = Path(directory)/'latest.frame'
            self.publish(path)
            camera = SharedPreviewCamera(path)
            camera.connect()
            self.clock.sleep(.01)
            self.publish(path, age=1.)
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                camera.async_read()
            self.assertTrue(camera.is_connected)
            self.clock.sleep(.01)
            self.publish(path)
            self.assertEqual(camera.async_read().shape, (48, 64, 3))

    def test_repeated_timestamp_is_not_a_new_frame_and_later_frame_recovers(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path)
            camera = SharedPreviewCamera(path)
            camera.connect()
            # Fast publications can share a clock tick, particularly on Windows.
            self.publish(path)
            with self.assertRaisesRegex(TimeoutError, 'no new frame'):
                camera.async_read(timeout_ms=10)
            self.assertTrue(camera.is_connected)
            self.clock.sleep(.01)
            self.publish(path)
            self.assertEqual(camera.async_read().shape, (48, 64, 3))

    def test_future_timestamp_is_rejected_before_control(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'latest.frame'
            self.publish(path, age=-1.)
            camera = SharedPreviewCamera(path)
            with self.assertRaisesRegex(RuntimeError, 'stale'):
                camera.connect()
            self.assertFalse(camera.is_connected)
