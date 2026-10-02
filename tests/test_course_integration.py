"""Migration boundaries: hardware-free imports, calibration, enable cleanup and dependency identity."""
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch, PropertyMock

from eai_robot.arm.calibration import JOINTS, load
from eai_robot.course.robot import CourseMotorsBus, CourseFollower, DEFAULT_CALIBRATION, make_robot
from eai_robot.course.poses.control_presets import load_poses, POSES_PATH
from eai_robot.course.kinematics.kinematics_backend import create_fk_solver
from eai_robot.course.kinematics.check_step2_fk import DEFAULT_URDF
from eai_robot.course.kinematics.joint_mapping import SO101JointMapper
from eai_robot.course.cli import COMMANDS
from lerobot.motors.feetech import FeetechMotorsBus

ROOT = Path(__file__).resolve().parents[1]
FORK_COMMIT = "6a077907c7989635218969ee78f5436f8faec92b"


class IntegrationTests(unittest.TestCase):
    def test_installed_framework_is_pinned_fork_not_pypi(self):
        dist = importlib.metadata.distribution("lerobot")
        self.assertEqual(dist.version, "0.6.2")
        provenance = json.loads(dist.read_text("direct_url.json"))
        self.assertEqual(provenance["vcs_info"]["commit_id"], FORK_COMMIT)

    def test_current_snapshot_matches_expected_ids_and_limits(self):
        cal = load(DEFAULT_CALIBRATION)
        self.assertEqual([(cal[j].range_min, cal[j].range_max) for j in JOINTS],
                         [(169,898),(60,740),(196,772),(67,681),(47,974),(239,640)])

    def test_constructing_course_robot_uses_shared_file_without_serial(self):
        with patch.object(CourseMotorsBus, "connect", side_effect=AssertionError("serial opened")):
            robot = make_robot("FAKE", "scs215_com5")
            self.assertEqual(robot.calibration_fpath, DEFAULT_CALIBRATION)
            self.assertEqual([m.model for m in robot.bus.motors.values()], ["scs215"]*6)
            self.assertEqual(robot.bus.protocol_version, 1)
            self.assertFalse(robot.motion_enabled)
            self.assertEqual(robot.action_features, {f"{n}.pos": float for n in JOINTS})

    def test_read_only_connect_checks_limits_without_writes(self):
        robot = make_robot("FAKE")
        bus = robot.bus
        cal = robot.calibration
        def read(register, name, **kwargs):
            if register == "ID":
                return cal[name].id
            return cal[name].range_min if register == "Min_Position_Limit" else cal[name].range_max
        with patch.object(FeetechMotorsBus,"connect"), patch.object(bus,"set_baudrate"), \
             patch.object(bus,"ping",return_value=1315), patch.object(bus,"read",side_effect=read), \
             patch.object(bus,"write",side_effect=AssertionError("register write")), \
             patch.object(bus,"write_verified",side_effect=AssertionError("register write")):
            bus.connect()

    def test_connect_rejects_stale_calibration_and_closes_port(self):
        robot = make_robot("FAKE")
        bus = robot.bus
        bus.port_handler.ser = Mock()
        def read(register, name, **kwargs):
            if register == "ID":
                return robot.calibration[name].id
            if name == "wrist_roll":
                return 100 if register == "Min_Position_Limit" else 900
            cal = robot.calibration[name]
            return cal.range_min if register == "Min_Position_Limit" else cal.range_max
        with patch.object(FeetechMotorsBus,"connect"), patch.object(bus,"set_baudrate"), \
             patch.object(bus,"ping",return_value=1315), patch.object(bus,"read",side_effect=read), \
             patch.object(bus.port_handler,"closePort") as close:
            with self.assertRaisesRegex(RuntimeError,"exceeds"):
                bus.connect()
            close.assert_called_once()

    def test_partial_torque_enable_releases_every_requested_motor(self):
        bus = make_robot("FAKE").bus
        calls = []
        def write(register,name,value):
            calls.append((register,name,value))
            if name == "shoulder_lift" and value == 1:
                raise OSError("disconnect")
        with patch.object(bus,"write_verified",side_effect=write):
            with self.assertRaises(OSError):
                bus.enable_torque(list(JOINTS))
        self.assertEqual([name for _,name,value in calls if value==0],list(JOINTS))
        self.assertTrue(all(register=="Torque_Enable" for register,_,_ in calls))

    def test_torque_release_continues_after_one_failure(self):
        bus = make_robot("FAKE").bus
        with patch.object(bus,"write_verified",side_effect=[OSError("missing"),None,None,None,None,None]) as write:
            with self.assertRaises(RuntimeError):
                bus.disable_torque()
            self.assertEqual(write.call_count,6)

    def test_motion_requires_enable_and_valid_normalized_values(self):
        robot = make_robot("FAKE")
        with self.assertRaisesRegex(RuntimeError,"enable_motion"):
            robot.send_action({"shoulder_pan.pos":0})
        robot.motion_enabled=True
        with patch.object(CourseMotorsBus,"is_connected",new_callable=PropertyMock,return_value=True):
            for action in ({"shoulder_pan.pos":float("nan")},{"gripper.pos":-1},{"shoulder_pan.pos":101},{"bad.pos":0}):
                with self.assertRaises(ValueError):
                    robot.send_action(action)
        robot.motion_enabled=False

    def test_pose_conversion_preserves_original_raw_targets(self):
        data=json.loads(POSES_PATH.read_text())
        current=load(DEFAULT_CALIBRATION)
        converted=load_poses(POSES_PATH,current)
        for pose, values in data["poses"].items():
            for name, old_value, new_value in zip(JOINTS,values,converted[pose],strict=True):
                old=data["calibration"][name]
                expected=round(old["range_min"]+old_value*(old["range_max"]-old["range_min"]))
                self.assertEqual(current[name].position(new_value),expected)

    def test_raw_normalized_and_ratio_conventions_agree(self):
        robot=make_robot("FAKE")
        mapper=SO101JointMapper(robot.calibration)
        current=load(DEFAULT_CALIBRATION)
        for name in JOINTS:
            for ratio in (0,.25,.5,.75,1):
                raw=current[name].position(ratio)
                normalized=mapper.raw_to_normalized(name,raw)
                expected=100*current[name].ratio(raw)
                if name!="gripper": expected=2*expected-100
                self.assertAlmostEqual(normalized,expected)

    def test_default_fk_does_not_change_when_placo_is_installed(self):
        with patch("importlib.util.find_spec",side_effect=AssertionError("platform-dependent auto-selection")):
            _,backend=create_fk_solver(DEFAULT_URDF)
        self.assertEqual(backend,"numpy-urdf-fallback")

    def test_all_command_help_paths_are_headless_and_hardware_free(self):
        for command in COMMANDS:
            with self.subTest(command=command):
                result=subprocess.run([sys.executable,str(ROOT/"scripts/course.py"),command,"--help"],
                                      capture_output=True,text=True,timeout=30)
                self.assertEqual(result.returncode,0,result.stderr)
