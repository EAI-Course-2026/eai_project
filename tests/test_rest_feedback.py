"""Natural-rest feedback allowance must not permit out-of-range motor goals."""
import unittest
from unittest.mock import patch, PropertyMock
from lerobot_robot_scs215 import SCS215MotorsBus
from eai_robot.arm.calibration import feedback_in_range
from eai_robot.course.robot import make_robot
from eai_robot.course.kinematics.joint_mapping import SO101JointMapper
from eai_robot.course.kinematics.run_steps3_to5 import raw_outside_calibration


class RestFeedbackTests(unittest.TestCase):
    def test_course_start_clamps_small_drift_without_expanding_goals(self):
        for boundary, drift in (("range_min", -3), ("range_max", 3)):
            with self.subTest(boundary=boundary):
                robot = make_robot("FAKE")
                robot.bus.hardware_limits={n:(c.range_min,c.range_max) for n,c in robot.calibration.items()}
                cal = robot.calibration["shoulder_lift"]
                raw = getattr(cal, boundary) + drift
                def read(register, name, **kw):
                    if register == "Present_Position":
                        c = robot.calibration[name]
                        return raw if name == "shoulder_lift" else (c.range_min+c.range_max)//2
                    return 0
                with patch.object(SCS215MotorsBus,"is_connected",new_callable=PropertyMock,return_value=True), \
                     patch.object(robot.bus,"read",side_effect=read), \
                     patch.object(robot.bus,"write_verified"), \
                     patch.object(robot.bus,"sync_write") as sync, \
                     patch.object(robot.bus,"enable_torque"):
                    robot.enable_motion()
                    self.assertTrue(robot.motion_enabled)
                    goals = sync.call_args.args[1]
                    self.assertEqual(goals["shoulder_lift"], getattr(cal,boundary))
                    for name, goal in goals.items():
                        c=robot.calibration[name]
                        self.assertTrue(c.range_min <= goal <= c.range_max)

    def test_excess_drift_rejected_before_any_motor_write(self):
        for drift in (-4,4):
            robot = make_robot("FAKE")
            robot.bus.hardware_limits={n:(c.range_min,c.range_max) for n,c in robot.calibration.items()}
            def read(register,name,**kw):
                c=robot.calibration[name]
                if register == "Present_Position":
                    return (c.range_min+drift if drift<0 else c.range_max+drift)
                return 0
            with patch.object(SCS215MotorsBus,"is_connected",new_callable=PropertyMock,return_value=True), \
                 patch.object(robot.bus,"read",side_effect=read), \
                 patch.object(robot.bus,"write_verified") as write, \
                 patch.object(robot.bus,"sync_write") as sync, \
                 patch.object(robot.bus,"enable_torque") as enable:
                with self.assertRaisesRegex(RuntimeError,"outside calibration"):
                    robot.enable_motion()
                write.assert_not_called()
                sync.assert_not_called()
                enable.assert_not_called()

    def test_planner_feedback_check_uses_same_bounded_allowance(self):
        robot=make_robot("FAKE")
        mapper=SO101JointMapper(robot.calibration)
        c=robot.calibration["shoulder_lift"]
        self.assertEqual(raw_outside_calibration(mapper,{"shoulder_lift":c.range_min-3}),[])
        self.assertEqual(raw_outside_calibration(mapper,{"shoulder_lift":c.range_min-4}),["shoulder_lift"])
        self.assertEqual(raw_outside_calibration(mapper,{"shoulder_lift":-1}),["shoulder_lift"])

    def test_invalid_encoder_values_are_never_accepted_as_slack(self):
        for raw in (-1,1024,True,1.5,float("nan")):
            self.assertFalse(feedback_in_range(raw,0,1023))
