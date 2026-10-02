"""Exercise the real plugin factory and standard CLI loops with simulated hardware."""

from contextlib import ExitStack, contextmanager, redirect_stdout
from dataclasses import asdict
import importlib
import importlib.metadata
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators.so_leader import SO101Leader
from lerobot_robot_scs215 import (
    SCS215MotorsBus,
    SCS215SO101Follower,
    SCS215SO101FollowerConfig,
)
from lerobot_robot_scs215.safety import JOINTS, feedback_in_range
from eai_robot.arm.calibration import load
from eai_robot.course.robot import DEFAULT_CALIBRATION, CourseFollower, CourseMotorsBus

ROOT = Path(__file__).resolve().parents[1]


class FakeLeader(SO101Leader):
    def __init__(self):
        self.connected = False

    @property
    def is_connected(self):
        return self.connected

    def connect(self, calibrate=True):
        self.connected = True

    def disconnect(self):
        self.connected = False

    def get_action(self):
        return {f"{name}.pos": 50.0 if name == "gripper" else 0.0 for name in JOINTS}


class SimulatedHardware:
    def __init__(self):
        self.calibration = load(DEFAULT_CALIBRATION)
        self.positions = {
            n: (c.range_min + c.range_max) // 2 for n, c in self.calibration.items()
        }
        self.torque = dict.fromkeys(JOINTS, 0)
        self.writes = []
        self.camera = Mock(height=2, width=3, is_connected=False)
        self.camera.async_read.return_value = np.zeros((2, 3, 3), dtype=np.uint8)
        self.camera.connect.side_effect = lambda: setattr(
            self.camera, "is_connected", True
        )
        self.camera.disconnect.side_effect = lambda: setattr(
            self.camera, "is_connected", False
        )

    def open(self, bus, **kwargs):
        bus.port_handler.is_open = True
        bus.port_handler.ser = Mock()
        bus.port_handler.closePort = lambda: setattr(bus.port_handler, "is_open", False)

    def read(self, bus, register, name, normalize=True, **kwargs):
        c = self.calibration[name]
        values = {
            "ID": c.id,
            "Min_Position_Limit": c.range_min,
            "Max_Position_Limit": c.range_max,
            "Present_Position": self.positions[name],
            "Torque_Enable": self.torque[name],
            "Status": 0,
            "Lock": 1,
        }
        raw = values[register]
        return (
            bus._normalize({c.id: raw})[c.id]
            if register == "Present_Position" and normalize
            else raw
        )

    def write(self, bus, register, name, value):
        self.writes.append((register, name, value))
        if register == "Torque_Enable":
            self.torque[name] = value

    def sync(self, bus, register, values, normalize=True, **kwargs):
        raw = (
            bus._unnormalize({bus.motors[n].id: v for n, v in values.items()})
            if normalize
            else {bus.motors[n].id: v for n, v in values.items()}
        )
        for name in values:
            value = raw[bus.motors[name].id]
            self.writes.append((register, name, value))
            self.positions[name] = value

    @contextmanager
    def active(self):
        with ExitStack() as stack:
            stack.enter_context(
                patch.object(
                    FeetechMotorsBus, "connect", autospec=True, side_effect=self.open
                )
            )
            stack.enter_context(patch.object(SCS215MotorsBus, "set_baudrate"))
            stack.enter_context(
                patch.object(SCS215MotorsBus, "ping", return_value=1315)
            )
            stack.enter_context(
                patch.object(
                    SCS215MotorsBus, "read", autospec=True, side_effect=self.read
                )
            )
            stack.enter_context(
                patch.object(
                    SCS215MotorsBus,
                    "write_verified",
                    autospec=True,
                    side_effect=self.write,
                )
            )
            stack.enter_context(
                patch.object(
                    SCS215MotorsBus, "sync_write", autospec=True, side_effect=self.sync
                )
            )
            stack.enter_context(
                patch(
                    "lerobot_robot_scs215.robot.make_cameras_from_configs",
                    side_effect=lambda cfg: {name: self.camera for name in cfg},
                )
            )
            yield self


class PluginTests(unittest.TestCase):
    def config(self, **kwargs):
        return SCS215SO101FollowerConfig(
            port="FAKE", calibration_path=DEFAULT_CALIBRATION, **kwargs
        )

    def test_fresh_process_discovers_installable_plugin_and_dynamic_factory(self):
        code = """
from lerobot.utils.import_utils import register_third_party_plugins
register_third_party_plugins()
from lerobot.robots import RobotConfig, make_robot_from_config
cls=RobotConfig.get_choice_class('scs215_so101_follower')
robot=make_robot_from_config(cls(port='FAKE'))
assert robot.__class__.__module__ == 'lerobot_robot_scs215.robot'
assert not robot.is_connected
"""
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            importlib.metadata.distribution("lerobot_robot_scs215").metadata["Name"],
            "lerobot_robot_scs215",
        )

    def test_course_and_direct_bus_share_the_plugin_classes(self):
        from eai_robot.hardware.lerobot_scs215 import SCS215MotorsBus as DirectBus

        self.assertIs(DirectBus, SCS215MotorsBus)
        self.assertIs(CourseMotorsBus, SCS215MotorsBus)
        self.assertIs(CourseFollower, SCS215SO101Follower)
        from eai_robot.arm import calibration as native

        self.assertEqual(native.JOINTS, JOINTS)
        for raw in (-1, 0, 57, 59, 60, 740, 743, 744, 1023, 1024, True):
            self.assertEqual(
                native.feedback_in_range(raw, 60, 740), feedback_in_range(raw, 60, 740)
            )

    def test_direct_backend_keeps_original_connection_error_and_configured_baud(self):
        from eai_robot.arm.backends import LeRobotBackend

        bus = Mock(is_connected=False)
        bus.connect.side_effect = OSError("device unavailable")
        with patch(
            "eai_robot.hardware.lerobot_scs215.SCS215MotorsBus", return_value=bus
        ) as factory:
            with self.assertRaisesRegex(OSError, "device unavailable"):
                LeRobotBackend("FAKE", 500_000)
            factory.assert_called_once_with("FAKE", baudrate=500_000)
        bus.port_handler.closePort.assert_not_called()

    def test_default_connect_observe_disconnect_preserves_external_torque(self):
        hardware = SimulatedHardware()
        hardware.torque = dict.fromkeys(JOINTS, 1)
        with hardware.active():
            robot = make_robot_from_config(self.config())
            robot.connect()
            obs = robot.get_observation()
            self.assertEqual(set(obs), set(robot.action_features))
            robot.disconnect()
        self.assertEqual(hardware.writes, [])
        self.assertEqual(set(hardware.torque.values()), {1})

    def test_relative_target_rejects_before_write_and_disconnect_releases(self):
        hardware = SimulatedHardware()
        with hardware.active():
            robot = make_robot_from_config(
                self.config(enable_motion=True, max_relative_target=5)
            )
            robot.connect()
            hardware.writes.clear()
            with self.assertRaisesRegex(ValueError, "max_relative_target"):
                robot.send_action({"shoulder_pan.pos": 50.0})
            self.assertEqual(hardware.writes, [])
            robot.disconnect()
        self.assertEqual(set(hardware.torque.values()), {0})
        self.assertTrue(all(r == "Torque_Enable" for r, _, _ in hardware.writes))

    def test_subset_enable_does_not_release_preexisting_gripper_torque(self):
        hardware = SimulatedHardware()
        hardware.torque["gripper"] = 1
        with hardware.active():
            robot = make_robot_from_config(self.config())
            robot.connect()
            robot.enable_motion(list(JOINTS[:-1]))
            robot.send_action({"shoulder_pan.pos": np.float32(0)})
            robot.disconnect()
        self.assertEqual(hardware.torque["gripper"], 1)
        self.assertEqual({hardware.torque[n] for n in JOINTS[:-1]}, {0})
        self.assertFalse(any(n == "gripper" for _, n, _ in hardware.writes))

    def test_out_of_range_observation_is_not_silently_clipped(self):
        hardware = SimulatedHardware()
        with hardware.active():
            robot = make_robot_from_config(self.config())
            robot.connect()
            hardware.positions["shoulder_lift"] = 56
            with self.assertRaisesRegex(RuntimeError, "observation outside"):
                robot.get_observation()
            robot.disconnect()
        self.assertEqual(hardware.writes, [])

    def test_camera_failure_closes_bus_before_motion_enable(self):
        hardware = SimulatedHardware()
        hardware.camera.connect.side_effect = OSError("camera absent")
        with hardware.active():
            robot = make_robot_from_config(
                self.config(
                    enable_motion=True,
                    cameras={
                        "front": OpenCVCameraConfig(
                            index_or_path=0, width=3, height=2, fps=100
                        )
                    },
                )
            )
            with self.assertRaisesRegex(OSError, "camera absent"):
                robot.connect()
            self.assertFalse(robot.bus.is_connected)
        self.assertEqual(hardware.writes, [])

    def cli_args(self, **flags):
        return [
            "cli",
            "--robot.type=scs215_so101_follower",
            "--robot.port=FAKE",
            f"--robot.calibration_path={DEFAULT_CALIBRATION}",
            *[f"--{key}={value}" for key, value in flags.items()],
        ]

    def test_standard_calibrate_cli_verifies_current_file_without_any_write(self):
        module = importlib.import_module("lerobot.scripts.lerobot_calibrate")
        hardware = SimulatedHardware()
        with (
            hardware.active(),
            patch.object(sys, "argv", self.cli_args(**{"robot.enable_motion": "true"})),
            patch("builtins.input", return_value=""),
            redirect_stdout(io.StringIO()),
        ):
            module.main()
        self.assertEqual(hardware.writes, [])

    def test_standard_teleoperate_cli_runs_real_loop_and_releases(self):
        module = importlib.import_module("lerobot.scripts.lerobot_teleoperate")
        hardware = SimulatedHardware()
        flags = {
            "robot.enable_motion": "true",
            "teleop.type": "so101_leader",
            "teleop.port": "FAKE",
            "teleop.use_degrees": "false",
            "teleop_time_s": "0.01",
            "fps": "100",
        }
        with (
            hardware.active(),
            patch.object(sys, "argv", self.cli_args(**flags)),
            patch.object(
                module, "make_teleoperator_from_config", return_value=FakeLeader()
            ),
            redirect_stdout(io.StringIO()),
        ):
            module.main()
        self.assertGreater(
            len([w for w in hardware.writes if w[0] == "Goal_Position"]), 6
        )
        self.assertEqual(set(hardware.torque.values()), {0})
        self.assertFalse(
            any(
                r in {"Lock", "Min_Position_Limit", "Max_Position_Limit"}
                for r, _, _ in hardware.writes
            )
        )

    def test_standard_record_cli_saves_local_episode_and_rgb_frames(self):
        module = importlib.import_module("lerobot.scripts.lerobot_record")
        hardware = SimulatedHardware()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "dataset"
            flags = {
                "robot.enable_motion": "true",
                "teleop.type": "so101_leader",
                "teleop.port": "FAKE",
                "teleop.use_degrees": "false",
                "robot.cameras": "{front: {type: opencv, index_or_path: 0, width: 3, height: 2, fps: 100}}",
                "dataset.repo_id": "local/plugin_test",
                "dataset.root": str(output),
                "dataset.single_task": "simulated control",
                "dataset.push_to_hub": "false",
                "dataset.num_episodes": "1",
                "dataset.episode_time_s": "0.03",
                "dataset.reset_time_s": "0",
                "dataset.fps": "100",
                "dataset.video": "false",
                "play_sounds": "false",
            }
            events = {
                "exit_early": False,
                "stop_recording": False,
                "rerecord_episode": False,
            }
            with (
                hardware.active(),
                patch.object(sys, "argv", self.cli_args(**flags)),
                patch.object(
                    module, "make_teleoperator_from_config", return_value=FakeLeader()
                ),
                patch.object(
                    module, "init_keyboard_listener", return_value=(None, events)
                ),
                redirect_stdout(io.StringIO()),
            ):
                module.main()
            info = json.loads((output / "meta/info.json").read_text())
            self.assertEqual(info["total_episodes"], 1)
            self.assertGreater(info["total_frames"], 0)
            self.assertEqual(info["robot_type"], "scs215_so101_follower")
            self.assertEqual(
                info["features"]["observation.images.front"]["shape"], [2, 3, 3]
            )
            self.assertEqual(info["features"]["action"]["shape"], [6])
            self.assertTrue(list(output.rglob("*.parquet")))
        self.assertEqual(set(hardware.torque.values()), {0})

    def test_manual_calibration_is_backed_up_and_never_writes_eeprom(self):
        hardware = SimulatedHardware()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "arm.json"
            original = DEFAULT_CALIBRATION.read_bytes()
            path.write_bytes(original)
            path.with_suffix(".meta.json").write_text('{"method":"old_snapshot"}')
            config = SCS215SO101FollowerConfig(port="FAKE", calibration_path=path)
            mins = {n: c.range_min + 1 for n, c in hardware.calibration.items()}
            maxes = {n: c.range_max - 1 for n, c in hardware.calibration.items()}
            with (
                hardware.active(),
                patch.object(
                    SCS215MotorsBus,
                    "record_ranges_of_motion",
                    return_value=(mins, maxes),
                ),
                patch("builtins.input", side_effect=["c", "", "SAVE"]),
                redirect_stdout(io.StringIO()),
            ):
                robot = make_robot_from_config(config)
                robot.connect(calibrate=False)
                robot.calibrate()
                robot.disconnect()
            self.assertEqual(load(path)["shoulder_lift"].range_min, 61)
            backups = [
                p
                for p in Path(directory).glob("arm.*.bak.json")
                if ".meta." not in p.name
            ]
            self.assertEqual(backups[0].read_bytes(), original)
            metadata = json.loads(path.with_suffix(".meta.json").read_text())
            self.assertFalse(metadata["eeprom_modified"])
            self.assertEqual(hardware.writes, [])

    def test_cancelled_calibration_keeps_file_and_active_ranges(self):
        hardware = SimulatedHardware()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "arm.json"
            path.write_bytes(DEFAULT_CALIBRATION.read_bytes())
            original = path.read_bytes()
            mins = {n: c.range_min + 1 for n, c in hardware.calibration.items()}
            maxes = {n: c.range_max - 1 for n, c in hardware.calibration.items()}
            with (
                hardware.active(),
                patch.object(
                    SCS215MotorsBus,
                    "record_ranges_of_motion",
                    return_value=(mins, maxes),
                ),
                patch("builtins.input", side_effect=["c", "", "cancel"]),
                redirect_stdout(io.StringIO()),
            ):
                robot = make_robot_from_config(
                    SCS215SO101FollowerConfig(port="FAKE", calibration_path=path)
                )
                robot.connect(calibrate=False)
                with self.assertRaisesRegex(RuntimeError, "cancelled"):
                    robot.calibrate()
                robot.disconnect()
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(hardware.writes, [])
