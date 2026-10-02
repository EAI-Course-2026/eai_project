"""No hardware: test wire bytes, measured ranges and failure behavior."""
import contextlib
import copy
import io
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from eai_robot.arm.calibration import JOINTS, JointCalibration, load, save, parse_ratios
from eai_robot.arm.controller import ArmController
from eai_robot.arm.cli import calibrate, import_hardware_calibration, main
from eai_robot.hardware.scs import Bus, ProtocolError


def calibration():
    return {name: JointCalibration(sid, sid % 2, 0, 100, 800)
            for sid, name in enumerate(JOINTS, 1)}


class FakeArm:
    def __init__(self):
        self.registers = {sid: {"Model_Number": 1315, "ID": sid,
                              "Min_Position_Limit": 0, "Max_Position_Limit": 1023,
                              "Present_Position": 450, "Goal_Position": 999,
                              "Torque_Enable": 0, "Running_Time": 321, "Goal_Velocity": 0,
                              "Present_Voltage": 50, "Present_Temperature": 20, "Status": 0}
                          for sid in range(1, 7)}
        self.writes, self.packets = [], []
        self.fail_enable = None
        self.follow = True
        self.closed = False

    def read(self, sid, register):
        return self.registers[sid][register]

    def write(self, sid, register, value):
        self.writes.append((sid, register, value))
        if register == "Torque_Enable" and value == 1 and sid == self.fail_enable:
            raise OSError("disconnected during enable")
        self.registers[sid][register] = value

    def sync_positions(self, targets):
        self.packets.append(dict(targets))
        for sid, value in targets.items():
            self.registers[sid]["Goal_Position"] = value
            if self.follow:
                self.registers[sid]["Present_Position"] = value

    def close(self):
        self.closed = True


class FakeSerial:
    def __init__(self):
        self.packets = []
        self.replies = b""

    def write(self, packet):
        self.packets.append(bytes(packet))
        return len(packet)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def read(self, size):
        reply, self.replies = self.replies[:size], self.replies[size:]
        return reply


class ArmTests(unittest.TestCase):
    def test_calibration_roundtrip_and_reversal(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "measured.json"
            save(path, calibration())
            cal = load(path)
        self.assertEqual(cal[JOINTS[0]].position(0), 800)
        self.assertEqual(cal[JOINTS[0]].position(1), 100)
        self.assertEqual(cal[JOINTS[1]].position(0, 10), 110)
        self.assertEqual(cal[JOINTS[1]].position(1, 10), 790)
        self.assertAlmostEqual(cal[JOINTS[0]].ratio(625), 0.25)
        with self.assertRaises(ValueError):
            cal[JOINTS[0]].bounds(350)

    def test_reject_invalid_calibration(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "bad.json"
            save(path, calibration())
            good = json.loads(path.read_text())
            for field, value in (("id", 2), ("range_min", 100.1), ("range_max", 1024),
                                 ("homing_offset", 1), ("drive_mode", True)):
                bad = copy.deepcopy(good)
                bad[JOINTS[0]][field] = value
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    load(path)
            path.write_text("[]")
            with self.assertRaises(ValueError):
                load(path)

    def test_reject_invalid_inputs_before_writes(self):
        bus = FakeArm()
        arm = ArmController(bus, calibration())
        arm.enable()
        writes, packets = len(bus.writes), len(bus.packets)
        for text in ("0 0", "0 0 0 0 0 nan", "0 0 0 0 0 inf", "0 0 0 0 0 1.1"):
            with self.assertRaises(ValueError):
                parse_ratios(text)
        with self.assertRaises(ValueError):
            arm.move([0.5] * 5 + [float("nan")])
        self.assertEqual((len(bus.writes), len(bus.packets)), (writes, packets))
        self.assertEqual(parse_ratios("0, 0.2, 0.4, 0.6, 0.8, 1"), [0, .2, .4, .6, .8, 1])

    def test_model_mode_and_limits_stop_all_motion(self):
        for sid, field, value in ((3, "Model_Number", 777), (1, "ID", 2),
                                  (2, "Max_Position_Limit", 0), (4, "Min_Position_Limit", 200),
                                  (5, "Present_Position", 999), (6, "Torque_Enable", 2)):
            bus = FakeArm()
            bus.registers[sid][field] = value
            arm = ArmController(bus, calibration())
            with self.assertRaises(RuntimeError):
                arm.enable()
            self.assertEqual(bus.writes, [])
            self.assertEqual(bus.packets, [])

    def test_enable_preloads_pose_clears_time_and_partial_failure_releases_every_id(self):
        bus = FakeArm()
        bus.fail_enable = 3
        arm = ArmController(bus, calibration(), velocity=100)
        with self.assertRaises(OSError):
            arm.enable()
        self.assertEqual(bus.packets, [{sid: 450 for sid in range(1, 7)}])
        self.assertTrue(all(bus.read(s, "Running_Time") == 0 for s in range(1, 7)))
        self.assertTrue(all(bus.read(s, "Goal_Velocity") == 100 for s in range(1, 7)))
        self.assertTrue(all(bus.read(s, "Torque_Enable") == 0 for s in range(1, 7)))
        self.assertEqual(bus.writes[-6:], [(sid, "Torque_Enable", 0) for sid in range(1, 7)])

    def test_startup_recovery_clamps_hardware_hold_and_enters_software_range(self):
        old = load(ROOT/'calibration/scs215_so101.json')
        candidate = load(ROOT/'calibration/scs215_safe_candidate_20261002.json')
        bus = FakeArm()
        for name in JOINTS:
            c = old[name]
            bus.registers[c.id]['Min_Position_Limit'] = c.range_min
            bus.registers[c.id]['Max_Position_Limit'] = c.range_max
        for sid, raw in enumerate((360, 63, 664, 162, 789, 420), 1):
            bus.registers[sid]['Present_Position'] = raw
        arm = ArmController(bus, candidate, tolerance=10, timeout=.2, allow_wide_range=True)
        trace = arm.recover_startup()
        self.assertTrue(arm.enabled)
        self.assertEqual(bus.packets[0][2], 64)
        self.assertGreaterEqual(bus.read(2, 'Present_Position'), candidate['shoulder_lift'].range_min)
        self.assertTrue(any(item['phase'] == 'recover' for item in trace))
        for packet in bus.packets:
            for name in JOINTS:
                c = old[name]
                self.assertTrue(c.range_min <= packet[c.id] <= c.range_max)
        for a, b in zip(bus.packets, bus.packets[1:]):
            self.assertLessEqual(max(abs(a[s] - b[s]) for s in a), 10)
        arm.disable()
        self.assertEqual({bus.read(s, 'Torque_Enable') for s in range(1, 7)}, {0})

    def test_startup_recovery_rejects_large_overtravel_without_writing(self):
        old = load(ROOT/'calibration/scs215_so101.json')
        candidate = load(ROOT/'calibration/scs215_safe_candidate_20261002.json')
        bus = FakeArm()
        for name in JOINTS:
            c = old[name]
            bus.registers[c.id]['Min_Position_Limit'] = c.range_min
            bus.registers[c.id]['Max_Position_Limit'] = c.range_max
        bus.registers[2]['Present_Position'] = 55
        arm = ArmController(bus, candidate, allow_wide_range=True)
        with self.assertRaisesRegex(RuntimeError, '距 EEPROM 限位过远'):
            arm.recover_startup()
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.packets, [])

    def test_startup_recovery_stall_releases_all_torque(self):
        old = load(ROOT/'calibration/scs215_so101.json')
        candidate = load(ROOT/'calibration/scs215_safe_candidate_20261002.json')
        bus = FakeArm()
        bus.follow = False
        for name in JOINTS:
            c = old[name]
            bus.registers[c.id]['Min_Position_Limit'] = c.range_min
            bus.registers[c.id]['Max_Position_Limit'] = c.range_max
        for sid, raw in enumerate((360, 63, 664, 162, 789, 420), 1):
            bus.registers[sid]['Present_Position'] = raw
        arm = ArmController(bus, candidate, tolerance=10, timeout=.1, allow_wide_range=True)
        with self.assertRaises(TimeoutError):
            arm.recover_startup()
        self.assertFalse(arm.enabled)
        self.assertEqual({bus.read(s, 'Torque_Enable') for s in range(1, 7)}, {0})

    def test_sync_interpolation_reaches_complete_command(self):
        bus = FakeArm()
        arm = ArmController(bus, calibration(), max_step=5, motion="smooth")
        arm.enable()
        values = [0, .2, .4, .6, .8, 1]
        with patch("eai_robot.arm.controller.time.sleep"):
            targets, reached = arm.move(values)
        self.assertEqual(targets, reached)
        self.assertEqual(targets, {sid: calibration()[n].position(v)
                                  for sid, (n, v) in enumerate(zip(JOINTS, values), 1)})
        for before, after in zip(bus.packets, bus.packets[1:]):
            self.assertEqual(set(after), set(range(1, 7)))
            self.assertLessEqual(max(abs(after[sid] - before[sid]) for sid in after), 5)
        arm.disable()
        self.assertFalse(arm.enabled)

    def test_tracking_failure_and_arrival_timeout_release_all(self):
        for value, error in ((0, RuntimeError), (.55, TimeoutError)):
            bus = FakeArm()
            arm = ArmController(bus, calibration(), timeout=.1, motion="smooth")
            arm.enable()
            bus.follow = False
            with patch("eai_robot.arm.controller.time.sleep"):
                with self.assertRaises(error):
                    arm.move([value] * 6)
            self.assertTrue(all(bus.read(s, "Torque_Enable") == 0 for s in range(1, 7)))

    def test_calibrate_manual_endpoints_and_preserve_file_on_interrupt(self):
        bus = FakeArm()
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
            path = Path(d) / "measured.json"
            count = 0
            def answer(prompt):
                nonlocal count
                if "RELEASE" in prompt:
                    return "RELEASE"
                sid = count // 2 + 1
                bus.registers[sid]["Present_Position"] = 750 if count % 2 == 0 else 150
                count += 1
                return ""
            calibrate(bus, path, input_fn=answer)
            self.assertEqual(count, 12)
            self.assertTrue(all(c.drive_mode == 1 for c in load(path).values()))
            original = path.read_bytes()
            def interrupt(prompt):
                if "RELEASE" in prompt:
                    return "RELEASE"
                raise KeyboardInterrupt()
            with self.assertRaises(KeyboardInterrupt):
                calibrate(bus, path, overwrite=True, input_fn=interrupt)
            self.assertEqual(path.read_bytes(), original)

    def test_inspect_does_not_write_and_cli_closes(self):
        bus = FakeArm()
        with patch("eai_robot.arm.backends.SerialBackend", return_value=bus), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main("serial", ["--port", "FAKE", "inspect"]), 0)
        self.assertEqual(bus.writes, [])
        self.assertEqual(bus.packets, [])
        self.assertTrue(bus.closed)

    def test_dry_run_does_not_open_either_backend(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "measured.json"
            save(path, calibration())
            with patch("eai_robot.arm.backends.SerialBackend", side_effect=AssertionError()), \
                    patch("eai_robot.arm.backends.LeRobotBackend", side_effect=AssertionError()), \
                    contextlib.redirect_stdout(io.StringIO()):
                for mode in ("serial", "lerobot"):
                    self.assertEqual(main(mode, ["--calibration", str(path), "control", "--dry-run",
                                                 "--values", "0", ".2", ".4", ".6", ".8", "1"]), 0)

    def test_plain_backend_never_imports_lerobot_or_sdk(self):
        code = ("import sys; sys.path.insert(0, 'src'); from eai_robot.arm.backends import SerialBackend; "
                "assert not any(n.startswith(('lerobot', 'scservo_sdk')) for n in sys.modules)")
        subprocess.run([sys.executable, "-c", code], cwd=ROOT, check=True)

    def test_scs_sync_packet_exact_big_endian_and_checksum(self):
        ser = FakeSerial()
        targets = {sid: 0x0100 + sid for sid in range(1, 7)}
        Bus(ser).sync_positions(targets)
        expected = bytes.fromhex("ff ff fe 16 83 2a 02 01 01 01 02 01 02 03 01 03 "
                                 "04 01 04 05 01 05 06 01 06 0c")
        # Independent checksum check, as well as the literal packet.
        self.assertEqual(sum(ser.packets[0][2:]) & 255, 255)
        self.assertEqual(ser.packets[0], expected)

    def test_lerobot_and_plain_backend_produce_identical_sync_packet(self):
        from eai_robot.hardware.lerobot_scs215 import SCS215MotorsBus
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors.feetech.tables import MODEL_NUMBER_TABLE
        bus = SCS215MotorsBus("FAKE")
        self.assertEqual(bus.protocol_version, 1)
        self.assertEqual(bus.model_number_table["scs_series"], 1315)
        self.assertNotIn("scs_series", FeetechMotorsBus.model_number_table)
        self.assertNotIn("scs_series", MODEL_NUMBER_TABLE)
        self.assertEqual(bus.model_resolution_table["scs_series"], 1024)
        captured = []
        bus.port_handler.is_open = True
        bus.port_handler.writePort = lambda p: captured.append(bytes(p)) or len(p)
        bus.port_handler.clearPort = lambda: None
        targets = {sid: 0x0100 + sid for sid in range(1, 7)}
        bus.sync_write("Goal_Position", {n: targets[s] for s, n in enumerate(JOINTS, 1)}, normalize=False)
        ser = FakeSerial()
        Bus(ser).sync_positions(targets)
        self.assertEqual(captured, ser.packets)
        with self.assertRaises(NotImplementedError):
            bus.sync_read("Present_Position")
        # Torque management never writes Lock or any EEPROM register.
        with patch.object(bus, "write_verified") as write:
            bus.disable_torque()
            self.assertEqual([c.args for c in write.call_args_list],
                             [("Torque_Enable", n, 0) for n in JOINTS])
        bus.port_handler.is_open = False

    def test_read_only_control_preserves_torque(self):
        bus = FakeArm()
        for sid in range(1, 7):
            bus.registers[sid]["Torque_Enable"] = 1
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "measured.json"
            save(path, calibration())
            with patch("eai_robot.arm.backends.SerialBackend", return_value=bus), \
                    contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main("serial", ["--port", "FAKE", "--calibration", str(path), "control"]), 0)
        self.assertTrue(bus.closed)
        self.assertEqual(bus.writes, [])
        self.assertTrue(all(bus.read(s, "Torque_Enable") == 1 for s in range(1, 7)))

    def test_failed_shutdown_returns_error_and_still_closes_port(self):
        bus = FakeArm()
        real_write = bus.write
        def failing_release(sid, register, value):
            if register == "Torque_Enable" and value == 0 and sid == 2:
                raise OSError("shutdown not confirmed")
            real_write(sid, register, value)
        bus.write = failing_release
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "measured.json"
            save(path, calibration())
            with patch("eai_robot.arm.backends.SerialBackend", return_value=bus), \
                    patch("eai_robot.arm.controller.time.sleep"), \
                    contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main("serial", ["--port", "FAKE", "--calibration", str(path), "control",
                                                 "--enable", "--values", *([".5"] * 6)]), 1)
        self.assertTrue(bus.closed)
        self.assertTrue(all(bus.read(s, "Torque_Enable") == 0 for s in (1, 3, 4, 5, 6)))

    def test_numbering_verifies_model_collision_and_relocks_after_failure(self):
        spec = importlib.util.spec_from_file_location("numbering_test", ROOT / "scripts/servo_setup.py")
        setup = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(setup)
        class NumberBus:
            def __init__(self):
                self.sid, self.model, self.lock, self.torque = 1, 1315, 1, 1
                self.collision = self.fail = False
                self.writes = []
            def identity(self, sid):
                if sid != self.sid:
                    raise TimeoutError()
                return sid
            def read_word(self, sid, address):
                return self.model
            def request(self, sid, instruction, params, count):
                if sid != self.sid and not self.collision:
                    raise TimeoutError()
                return b""
            def verified_write(self, sid, address, value):
                self.writes.append((sid, address, value))
                if address == 48:
                    self.lock = value
                if address == 40:
                    self.torque = value
            def write(self, sid, address, value):
                if self.fail:
                    raise OSError("ID write failed")
                self.sid = value
        with contextlib.redirect_stdout(io.StringIO()):
            bus = NumberBus()
            setup.rename(bus, 1, 6)
            self.assertEqual((bus.sid, bus.lock, bus.torque), (6, 1, 0))
            for bad in ("model", "collision"):
                bus = NumberBus()
                if bad == "model":
                    bus.model = 777
                else:
                    bus.collision = True
                with self.assertRaises(RuntimeError):
                    setup.rename(bus, 1, 6)
                self.assertEqual(bus.writes, [])
            bus = NumberBus()
            bus.fail = True
            with self.assertRaises(OSError):
                setup.rename(bus, 1, 6)
            self.assertEqual((bus.sid, bus.lock), (1, 1))

    def test_lerobot_verified_write_discards_ack_before_read(self):
        from eai_robot.hardware.lerobot_scs215 import SCS215MotorsBus
        bus = SCS215MotorsBus("FAKE")
        class AckSerial:
            def __init__(self):
                self.pending = b"old reply"
                self.resets = 0
            def reset_input_buffer(self):
                self.pending = b""
                self.resets += 1
            def flush(self):
                # SDK clearPort has precisely this behavior: TX only.
                pass
        ser = AckSerial()
        bus.port_handler.ser = ser
        def write_ack(port, sid, address, size, data):
            self.assertEqual(ser.pending, b"")
            self.assertEqual((sid, address, size, data), (1, 44, 2, [0, 0]))
            ser.pending = bytes.fromhex("ff ff 01 02 00 fc")
            return 0
        def read_value(register, motor, normalize):
            self.assertEqual(ser.pending, b"")
            self.assertEqual((register, motor, normalize), ("Running_Time", JOINTS[0], False))
            return 0
        with patch.object(bus.packet_handler, "writeTxOnly", side_effect=write_ack), \
                patch.object(bus, "read", side_effect=read_value), \
                patch("eai_robot.hardware.lerobot_scs215.time.sleep"):
            bus.write_verified("Running_Time", JOINTS[0], 0)
        self.assertEqual(ser.resets, 2)

    def test_direct_control_preserves_speed_and_uses_one_complete_target_frame(self):
        bus = FakeArm()
        for sid in range(1, 7):
            bus.registers[sid]["Goal_Velocity"] = sid * 100
        arm = ArmController(bus, calibration())
        arm.enable()
        with patch("eai_robot.arm.controller.time.sleep"):
            targets, reached = arm.move([.6] * 6)
        self.assertEqual(targets, reached)
        self.assertEqual(len(bus.packets), 2)  # preload, then six-joint final target
        self.assertEqual(bus.packets[-1], targets)
        self.assertTrue(all(bus.read(s, "Goal_Velocity") == s * 100 for s in range(1, 7)))
        self.assertFalse(any(register == "Goal_Velocity" for _, register, _ in bus.writes))

    def test_feedback_slack_clamps_hold_but_does_not_expand_goals(self):
        bus = FakeArm()
        bus.registers[1]["Present_Position"] = 98
        arm = ArmController(bus, calibration())
        arm.enable()
        self.assertEqual(bus.packets[0][1], 100)
        self.assertTrue(all(100 <= p <= 800 for p in bus.packets[0].values()))
        bus = FakeArm()
        bus.registers[1]["Present_Position"] = 96
        with self.assertRaises(RuntimeError):
            ArmController(bus, calibration()).enable()
        self.assertEqual(bus.writes, [])

    def test_import_existing_hardware_calibration_is_read_only_and_tracks_source(self):
        bus = FakeArm()
        with tempfile.TemporaryDirectory() as d, contextlib.redirect_stdout(io.StringIO()):
            path = Path(d) / "hardware.json"
            import_hardware_calibration(bus, path, reverse=[JOINTS[1]])
            cal = load(path)
            self.assertEqual(cal[JOINTS[1]].drive_mode, 1)
            self.assertEqual(cal[JOINTS[0]].range_max, 1023)
            meta = json.loads(path.with_suffix(".meta.json").read_text())
            self.assertEqual(meta["method"], "existing_hardware_limits")
            self.assertFalse(meta["manual_endpoints_verified"])
            self.assertEqual(bus.writes, [])
            with self.assertRaises(ValueError):
                ArmController(bus, cal)
            ArmController(bus, cal, allow_wide_range=True)

    def test_short_sync_write_is_error(self):
        ser = FakeSerial()
        ser.write = lambda packet: len(packet) - 1
        with self.assertRaises(ProtocolError):
            Bus(ser).sync_positions({1: 256})


if __name__ == "__main__":
    unittest.main()
