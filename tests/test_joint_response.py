"""Response timing and motion detection must be separate from arrival tolerance."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from eai_robot.applications.tools import joint_response as app
from eai_robot.calibration.servo.results import JointCalibration
from test_arm import FakeArm


class Clock:
    def __init__(self): self.now = 0.
    def read(self): return self.now
    def sleep(self, amount): self.now += amount


class DelayedMotor:
    def __init__(self, clock, latency=.08, deadzone=0):
        self.clock, self.latency, self.deadzone = clock, latency, deadzone
        self.position = self.target = 500
    def sync_positions(self, targets):
        self.target = targets[1]
        self.sent = self.clock.now
    def read(self, sid, register):
        if register == "Goal_Position": return self.target
        if register == "Torque_Enable": return 1
        if register == "Status": return 0
        if (self.clock.now-self.sent >= self.latency
                and abs(self.target-self.position) > self.deadzone):
            self.position = self.target
        return self.position


class JointResponseTests(unittest.TestCase):
    def test_reports_actual_200ms_response(self):
        clock = Clock(); motor = DelayedMotor(clock)
        result, rows = app.measure_step(motor, 1, 500, 520,
                                       clock=clock.read, sleep=clock.sleep)
        self.assertTrue(result["moved_within_200ms"])
        self.assertTrue(result["arrived_within_200ms"])
        self.assertGreaterEqual(result["first_motion_s"], .08)
        self.assertGreater(len(rows), 50)

    def test_delayed_response_is_not_reported_as_200ms_success(self):
        clock = Clock(); motor = DelayedMotor(clock, latency=.3)
        result, _ = app.measure_step(motor, 1, 500, 520,
                                    clock=clock.read, sleep=clock.sleep)
        self.assertTrue(result["observed_motion"])
        self.assertFalse(result["moved_within_200ms"])

    def test_deadzone_is_not_false_arrival(self):
        clock = Clock(); motor = DelayedMotor(clock, deadzone=3)
        result, _ = app.measure_step(motor, 1, 500, 502,
                                    clock=clock.read, sleep=clock.sleep)
        self.assertFalse(result["observed_motion"])
        self.assertIsNone(result["first_arrival_s"])
        self.assertLess(result["tolerance_counts"], abs(result["delta_counts"]))

    def test_validates_whole_excursion_before_power(self):
        cal = JointCalibration(1, 0, 0, 169, 898)
        app.validate_targets(500, cal, (169, 898), [30, 20, 2])
        for origin, counts in ((180, [30]), (500, [50]), (500, [1])):
            with self.assertRaises(ValueError):
                app.validate_targets(origin, cal, (169, 898), counts)

    def test_read_only_makes_no_register_or_position_writes(self):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(app, "SerialBackend", return_value=arm), \
             contextlib.redirect_stdout(io.StringIO()):
            app.main(["--port", "FAKE", "--output", str(Path(directory)/"run")])
        self.assertEqual(arm.writes, [])
        self.assertEqual(arm.packets, [])
        self.assertTrue(arm.closed)

    def test_cancel_makes_no_motor_writes(self):
        arm = FakeArm()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(app, "SerialBackend", return_value=arm), \
             patch("builtins.input", return_value="cancel"), \
             contextlib.redirect_stdout(io.StringIO()):
            app.main(["--port", "FAKE", "--execute", "--output", str(Path(directory)/"run")])
        self.assertEqual(arm.writes, [])
        self.assertEqual(arm.packets, [])

    def test_execute_only_powers_selected_axis_and_releases_on_fault(self):
        for fail in (False, True):
            arm = FakeArm()
            def fake_measure(backend, sid, start, target, **kwargs):
                backend.sync_positions({sid: target})
                if fail:
                    raise RuntimeError("injected feedback failure")
                row = dict(elapsed_s=.08, position=target, target=target)
                kwargs["on_sample"](row)
                return dict(start=start, target=target, delta_counts=target-start,
                            final_position=target, final_error_counts=0, observed_motion=True), [row]
            with tempfile.TemporaryDirectory() as directory, \
                 patch.object(app, "SerialBackend", return_value=arm), \
                 patch.object(app, "measure_step", side_effect=fake_measure), \
                 patch("builtins.input", return_value="START JOINT TEST"), \
                 contextlib.redirect_stdout(io.StringIO()):
                args = ["--port", "FAKE", "--execute", "--counts", "20", "2",
                        "--output", str(Path(directory)/"run")]
                if fail:
                    with self.assertRaisesRegex(RuntimeError, "injected"):
                        app.main(args)
                else:
                    app.main(args)
            self.assertTrue(all(sid == 1 for sid, _, _ in arm.writes))
            self.assertTrue(all(set(packet) == {1} for packet in arm.packets))
            self.assertEqual(arm.read(1, "Torque_Enable"), 0)
            self.assertTrue(arm.closed)

    def test_return_offset_is_recorded_and_next_shot_uses_measured_origin(self):
        arm = FakeArm()
        calls = []
        def static_offset(backend, sid, start, target, **kwargs):
            sign = 1 if target > start else -1
            actual = target - sign * 2
            backend.sync_positions({sid: target})
            backend.registers[sid]["Present_Position"] = actual
            calls.append((start, target, actual))
            kwargs["on_sample"](dict(elapsed_s=1., position=actual, target=target))
            return dict(final_position=actual, final_error_counts=2,
                        observed_motion=actual != start), []
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(app, "SerialBackend", return_value=arm), \
             patch.object(app, "measure_step", side_effect=static_offset), \
             patch("builtins.input", return_value="START JOINT TEST"), \
             contextlib.redirect_stdout(io.StringIO()):
            app.main(["--port", "FAKE", "--execute", "--counts", "20", "5", "2",
                      "--output", str(Path(directory)/"run")])
        self.assertEqual(calls[2][0], calls[1][2])
        self.assertEqual(calls[2][1] - calls[2][0], -20)
        self.assertTrue(all(430 <= target <= 470 for _, target, _ in calls))
