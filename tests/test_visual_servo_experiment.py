"""Simulated-device checks for experiment entry, motion and failure cleanup."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from eai_robot.applications.visual_servo import experiment
from eai_robot.calibration.servo.results import load
from eai_robot.config import ROOT
from test_arm import FakeArm


class FakeCamera:
    def __init__(self, backend, failure=None):
        self.backend, self.failure = backend, failure
        self.reads, self.closed = 0, False
        self.image = np.full((480, 640, 3), 255, np.uint8)
        if failure != "missing":
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(dictionary, 0, 60)
            self.image[210:270, 320:380] = marker[:, :, None]

    def isOpened(self): return True
    def set(self, *_): return True
    def grab(self): return True
    def release(self): self.closed = True

    def read(self):
        self.reads += 1
        if self.reads >= 3:
            if self.failure == "torque":
                self.backend.registers[2]["Torque_Enable"] = 0
            elif self.failure == "alarm":
                self.backend.registers[2]["Status"] = 1
            elif self.failure == "slow":
                time.sleep(.3)
            elif self.failure == "lost":
                return True, np.full_like(self.image, 255)
        return True, self.image.copy()


class ExperimentTests(unittest.TestCase):
    def run_case(self, root, *, execute=False, answer="START ARUCO", failure=None, center=False):
        arm = FakeArm()
        if failure == "stalled":
            arm.follow = False
        calibration = load(ROOT / "calibration/scs215_so101.json")
        for cal in calibration.values():
            arm.registers[cal.id]["Present_Position"] = (cal.range_min + cal.range_max) // 2
        camera = FakeCamera(arm, failure)
        intrinsic = root / "intrinsics.json"
        intrinsic.write_text(json.dumps(dict(model="opencv_pinhole", image_size=dict(width=640,height=480),
            camera_matrix=[[600,0,320],[0,600,240],[0,0,1]], distortion_coefficients=[0]*5)))
        mount = root / "mount.json"
        mount.write_text(json.dumps(dict(transform_convention="destination_T_source",
            gripper_T_camera=np.eye(4).tolist(),camera_translation_in_gripper_mm=[0]*3)))
        args = ["aruco-experiment", "--camera-index", "0", "--port", "FAKE",
                "--intrinsics", str(intrinsic), "--hand-eye", str(mount),
                "--hand-eye-gripper-frame", "gripper_frame_link", "--marker-length-m", ".025",
                "--output", str(root / "run")]
        if execute:
            args.append("--execute")
        if center:
            args.append("--center-only")
        error = None
        with patch.object(sys, "argv", args), patch.object(experiment, "SerialBackend", return_value=arm), \
             patch.object(experiment.cv2, "VideoCapture", return_value=camera), \
             patch("builtins.input", side_effect=[answer, ""]), contextlib.redirect_stdout(io.StringIO()):
            try:
                experiment.main()
            except (RuntimeError, TimeoutError) as exc:
                error = exc
        events = [json.loads(line) for line in (root / "run/events.jsonl").read_text().splitlines()]
        self.assertTrue(arm.closed)
        self.assertTrue(camera.closed)
        return arm, events, error

    def test_read_only_and_cancel_never_write_motor_registers(self):
        for execute, answer in ((False, ""), (True, "cancel")):
            with tempfile.TemporaryDirectory() as directory:
                arm, _, error = self.run_case(Path(directory), execute=execute, answer=answer)
                self.assertIsNone(error)
                self.assertEqual(arm.writes, [])
                self.assertEqual(arm.packets, [])

    def test_missing_marker_rejects_before_torque(self):
        with tempfile.TemporaryDirectory() as directory:
            arm, _, error = self.run_case(Path(directory), execute=True, failure="missing")
            self.assertIn("target lost", str(error))
            self.assertEqual(arm.writes, [])
            self.assertEqual(arm.packets, [])

    def test_mvp_center_mode_uses_a_two_count_step_within_existing_limits(self):
        with tempfile.TemporaryDirectory() as directory:
            arm, events, error = self.run_case(Path(directory), execute=True, center=True)
            self.assertIsNone(error)
            observation = next(e for e in events if e["event"] == "observation")
            proposal = next(e for e in events if e["event"] == "proposal")
            delta = [abs(proposal["raw_targets"][sid] - observation["measured"][sid])
                     for sid in observation["measured"]]
            self.assertEqual(max(delta), 2)
            self.assertIn("motion_complete", [e["event"] for e in events])

    def test_one_simulated_step_and_confirmed_torque_release(self):
        with tempfile.TemporaryDirectory() as directory:
            arm, events, error = self.run_case(Path(directory), execute=True)
            self.assertIsNone(error)
            self.assertIn("motion_complete", [e["event"] for e in events])
            complete = next(e for e in events if e["event"] == "motion_complete")
            self.assertEqual(complete["tolerance_counts"], 1)
            proposals = [e for e in events if e["event"] == "proposal"]
            observed = next(e["measured"] for e in events if e["event"] == "observation")
            self.assertTrue(any(abs(proposals[0]["raw_targets"][sid] - observed[sid]) == 2
                                for sid in observed))
            self.assertTrue(all(arm.read(i, "Torque_Enable") == 0 for i in range(1,7)))
            # First packet holds the measured pose; later packets keep gripper fixed.
            self.assertTrue(all(packet[6] == arm.packets[0][6] for packet in arm.packets))

    def test_torque_loss_or_alarm_rejects_before_motion_and_releases(self):
        for mode in ("torque", "alarm", "slow", "lost"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                arm, events, error = self.run_case(Path(directory), execute=True, failure=mode)
                self.assertIsNotNone(error)
                self.assertEqual(len(arm.packets), 1)  # startup hold only
                self.assertNotIn("motion_complete", [e["event"] for e in events])
                self.assertTrue(all(arm.read(i, "Torque_Enable") == 0 for i in range(1,7)))

    def test_nearest_target_keeps_marker_identity_for_rotated_marker(self):
        square = experiment.desired_square_corners(640, 480, 60)
        upside_down = np.roll(square, 2, axis=0) + [20, 10]
        target = experiment.nearest_square_target(upside_down, (640, 480), 60)
        np.testing.assert_equal(target, np.roll(square, 2, axis=0))

    def test_stalled_arm_times_out_with_fresh_camera_checks_and_releases(self):
        with tempfile.TemporaryDirectory() as directory:
            arm, events, error = self.run_case(Path(directory), execute=True,
                                               center=True, failure="stalled")
            self.assertIsNotNone(error)
            self.assertNotIn("motion_complete", [e["event"] for e in events])
            self.assertTrue(all(arm.read(i, "Torque_Enable") == 0 for i in range(1, 7)))
