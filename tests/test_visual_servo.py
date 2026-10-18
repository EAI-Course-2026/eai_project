"""Offline direction, closed-loop geometry and rehearsal acceptance checks."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

import cv2
import numpy as np

from eai_robot.applications.visual_servo.cli import load_intrinsics, propose
from eai_robot.control.visual_servo.aruco_ibvs import desired_square_corners, point_interaction_matrix
from eai_robot.control.visual_servo.controller import ibvs_step, simple_step
from eai_robot.vision.detection.aruco import ArucoMarkerDetector
from eai_robot.vision.features import VisualFeatures
from eai_robot.vision.tracking.optical_flow import ObjectFlowTracker


K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])


def features(points, stamp=1., identity="aruco:0"):
    return VisualFeatures(np.array(points), (640, 480), stamp, identity)


class ServoTests(unittest.TestCase):
    def test_simple_camera_directions_and_center(self):
        for point, expected in [([100, 240], [-.01, 0, 0]),
                                ([550, 240], [.01, 0, 0]),
                                ([320, 80], [0, -.01, 0]),
                                ([320, 420], [0, .01, 0]),
                                ([320, 240], [0, 0, .01])]:
            with self.subTest(point=point):
                result = simple_step(features([point]), now=1.)
                np.testing.assert_allclose(result.camera_delta_m, expected)

    def test_no_proposal_for_lost_stale_future_or_changed_target(self):
        for obs, status in [(None, "target_lost"),
                            (features([[320, 240]], stamp=.5), "stale_frame"),
                            (features([[320, 240]], stamp=2), "stale_frame"),
                            (features([[320, 240]], identity="other"), "target_changed")]:
            for controller in (simple_step, ibvs_step):
                kwargs = dict(camera_matrix=K, depths_m=np.ones(4)) if controller == ibvs_step else {}
                result = controller(obs, now=1., expected_target="aruco:0", **kwargs)
                self.assertEqual(result.status, status)
                self.assertIsNone(result.camera_delta_m)
                self.assertIsNone(result.camera_twist)

    def test_features_copy_input_and_reject_invalid_values(self):
        source = np.array([[100., 120.]])
        obs = features(source)
        source[:] = 0
        np.testing.assert_equal(obs.points_px, [[100, 120]])
        for point in ([[np.nan, 1]], [[640, 1]], [[-1, 1]]):
            with self.assertRaises(ValueError):
                features(point)

    def test_interaction_matrix_matches_camera_motion_projection(self):
        points = np.array([[-.05, -.05, .4], [.05, -.05, .4],
                           [.05, .05, .4], [-.05, .05, .4]])
        def project(x):
            homogeneous = (K @ x.T).T
            return homogeneous[:, :2] / homogeneous[:, 2:]
        pixels = project(points)
        matrix = point_interaction_matrix(pixels, points[:, 2], K)
        dt = 1e-6
        for axis in range(6):
            twist = np.eye(6)[axis]
            rotation, _ = cv2.Rodrigues(-twist[3:] * dt)
            moved = (rotation @ (points - twist[:3] * dt).T).T
            derivative = ((project(moved) - pixels) / dt).reshape(-1)
            np.testing.assert_allclose(derivative, matrix[:, axis], atol=.002)

    def test_ibvs_reduces_error_over_synthetic_closed_loop(self):
        # A fixed square seen by a moving camera; integrate inverse camera
        # motion in 3-D and reproject, rather than testing L @ velocity alone.
        points = np.array([[-.025, -.07, .5], [.075, -.07, .5],
                           [.075, .03, .5], [-.025, .03, .5]])
        errors = []
        for _ in range(120):
            homogeneous = (K @ points.T).T
            pixels = homogeneous[:, :2] / homogeneous[:, 2:]
            proposal = ibvs_step(features(pixels), now=1., camera_matrix=K,
                                 depths_m=points[:, 2], tolerance_px=.1)
            errors.append(proposal.error_rms_px)
            twist = proposal.camera_twist
            self.assertLessEqual(np.linalg.norm(twist[:3]), .020001)
            self.assertLessEqual(np.linalg.norm(twist[3:]), .150001)
            rotation, _ = cv2.Rodrigues(-twist[3:] * .1)
            points = (rotation @ (points - twist[:3] * .1).T).T
        self.assertLess(errors[-1], errors[0] * .05)

    def test_aligned_and_invalid_depth(self):
        obs = features(desired_square_corners(640, 480, 120))
        proposal = ibvs_step(obs, now=1., camera_matrix=K, depths_m=np.ones(4))
        self.assertEqual(proposal.status, "aligned")
        np.testing.assert_equal(proposal.camera_twist, np.zeros(6))
        with self.assertRaises(ValueError):
            ibvs_step(obs, now=1., camera_matrix=K, depths_m=np.zeros(4))

    def test_distorted_marker_rehearsal_and_resolution_rejection(self):
        # Render a genuine marker; exercise detector -> PnP -> undistort -> IBVS.
        image = np.full((480, 640, 3), 255, np.uint8)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dictionary, 0, 120)
        image[180:300, 300:420] = marker[:, :, None]
        args = SimpleNamespace(mode="ibvs", marker_length_m=.05, marker_id=0,
                               target_side_px=120, step_mm=10)
        detector = ArucoMarkerDetector(marker_id=0)
        calibration = ((640, 480), K, np.array([.1, -.02, .001, 0, 0]))
        result, detected = propose(image, detector, args, calibration, time.monotonic())
        self.assertIsNotNone(detected)
        self.assertEqual(result["status"], "tracking")
        self.assertTrue(np.all(np.isfinite(result["camera_twist_m_s_rad_s"])))
        with self.assertRaisesRegex(ValueError, "resolution"):
            propose(image, detector, args, ((320, 240), K, np.zeros(5)), time.monotonic())

    def test_intrinsics_reject_unsupported_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "intrinsics.json"
            path.write_text(json.dumps(dict(image_size=[640, 480], camera_matrix=K.tolist(),
                                             distortion=[0]*5, distortion_model="opencv_pinhole")))
            self.assertEqual(load_intrinsics(path)[0], (640, 480))
            path.write_text('{"distortion_model": "fisheye"}')
            with self.assertRaisesRegex(ValueError, "opencv_pinhole"):
                load_intrinsics(path)

    def test_cli_still_image_without_camera_or_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "blank.png"
            cv2.imwrite(str(path), np.full((480, 640, 3), 255, np.uint8))
            code = '''
import sys
from eai_robot.cli import main
sys.argv = ['eai-course', 'visual-servo', '--image', sys.argv[1], '--marker-id', '0']
assert main() == 0
assert not any(n.startswith(('lerobot', 'serial', 'eai_robot.hardware.robots', 'eai_robot.hardware')) for n in sys.modules)
'''
            result = subprocess.run([sys.executable, "-c", code, str(path)],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(result.stdout)["status"], "target_lost")

    def test_collaborator_intrinsics_and_full_ibvs_cli(self):
        # Portable example of the collaborator's schema; no dependency on
        # ignored machine-local calibration files.
        data = dict(schema_version=1, model="opencv_pinhole",
                    image_size=dict(width=640, height=480), camera_matrix=K.tolist(),
                    distortion_coefficients=[.01, -.02, .001, 0, 0],
                    pattern=dict(square_size_mm=19.6))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "intrinsics.json"
            original = json.dumps(data)
            path.write_text(original)
            size, matrix, distortion = load_intrinsics(path)
            self.assertEqual(size, (640, 480))
            np.testing.assert_equal(matrix, K)
            np.testing.assert_equal(distortion, data["distortion_coefficients"])
            image = np.full((480, 640, 3), 255, np.uint8)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(dictionary, 0, 120)
            image[180:300, 300:420] = marker[:, :, None]
            image_path = Path(directory) / "marker.png"
            cv2.imwrite(str(image_path), image)
            code = '''
import sys
from eai_robot.cli import main
sys.argv = ['eai-course', 'visual-servo', '--mode', 'ibvs', '--image', sys.argv[1],
            '--marker-id', '0', '--marker-length-m', '0.05', '--intrinsics', sys.argv[2]]
assert main() == 0
assert not any(n.startswith(('lerobot', 'serial', 'eai_robot.hardware')) for n in sys.modules)
'''
            result = subprocess.run([sys.executable, "-c", code, str(image_path), str(path)],
                                    capture_output=True, text=True, timeout=30)
            self.assertEqual(result.returncode, 0, result.stderr)
            proposal = json.loads(result.stdout)
            self.assertEqual(proposal["status"], "tracking")
            self.assertTrue(np.all(np.isfinite(proposal["camera_twist_m_s_rad_s"])))
            self.assertEqual(path.read_text(), original)

    def test_collaborator_intrinsics_reject_invalid_or_conflicting_fields(self):
        base = dict(schema_version=1, model="opencv_pinhole",
                    image_size=dict(width=640, height=480), camera_matrix=K.tolist(),
                    distortion_coefficients=[0]*5)
        invalid = [dict(model="fisheye"), dict(schema_version=2),
                   dict(image_size=dict(width=640)), dict(image_size=[640, 0]),
                   dict(image_size=dict(width=True, height=480)),
                   dict(distortion_coefficients=[0]*3),
                   dict(distortion_coefficients=[0, 0, float("nan"), 0, 0]),
                   dict(distortion_model="fisheye"), dict(distortion=[1]*5)]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "intrinsics.json"
            for replacement in invalid:
                with self.subTest(replacement=replacement):
                    path.write_text(json.dumps(base | replacement))
                    with self.assertRaises(ValueError):
                        load_intrinsics(path)


class ObjectTrackerTests(unittest.TestCase):
    @staticmethod
    def scene():
        rng = np.random.default_rng(42)
        frame = np.full((240, 320, 3), 180, np.uint8)
        texture = rng.integers(0, 256, size=(80, 80), dtype=np.uint8)
        frame[70:150, 90:170] = texture[:, :, None]
        return frame

    def test_tracks_translation_and_drives_shared_controller(self):
        first = self.scene()
        moved = cv2.warpAffine(first, np.float32([[1, 0, 9], [0, 1, -6]]), (320, 240))
        tracker = ObjectFlowTracker()
        initial = tracker.initialize(first, (90, 70, 80, 80), 1.)
        self.assertIsNotNone(initial)
        tracked = tracker.update(moved, 1.1)
        self.assertIsNotNone(tracked)
        np.testing.assert_allclose(tracked.center_px, [139, 104], atol=.6)
        proposal = simple_step(tracked, now=1.1, expected_target="object:selection")
        self.assertIsNotNone(proposal.camera_delta_m)

    def test_loss_is_latched_until_explicit_reselection(self):
        first = self.scene()
        tracker = ObjectFlowTracker()
        tracker.initialize(first, (90, 70, 80, 80), 1.)
        self.assertIsNone(tracker.update(np.full_like(first, 180), 1.1))
        self.assertIsNone(tracker.update(first, 1.2))
        self.assertIsNotNone(tracker.initialize(first, (90, 70, 80, 80), 1.3))

    def test_timestamp_gap_and_resolution_changes_lose_target(self):
        first = self.scene()
        for frame, stamp in ((first, 1.5), (first, .9), (first[:200], 1.1)):
            tracker = ObjectFlowTracker()
            tracker.initialize(first, (90, 70, 80, 80), 1.)
            self.assertIsNone(tracker.update(frame, stamp))
            self.assertIsNone(tracker.update(first, 1.6))

    def test_textureless_or_invalid_roi_cannot_start_tracking(self):
        tracker = ObjectFlowTracker()
        self.assertIsNone(tracker.initialize(np.full((240, 320, 3), 180, np.uint8),
                                             (90, 70, 80, 80), 1.))
        with self.assertRaises(ValueError):
            tracker.initialize(self.scene(), (300, 70, 80, 80), 1.)
