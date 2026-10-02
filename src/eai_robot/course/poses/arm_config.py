"""Load per-robot joint ranges from LeRobot's local calibration file."""

from eai_robot.course.robot import CourseFollower as SO101Follower, make_robot, DEFAULT_CALIBRATION, default_port


JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def load_calibrated_ranges(port: str, robot_id: str) -> dict[int, tuple[int, int]]:
    """Return servo ID -> raw minimum/maximum without opening the serial port."""
    robot = make_robot(port, robot_id)
    if not robot.calibration:
        raise FileNotFoundError(f"No calibration found at {robot.calibration_fpath}")

    return {
        servo_id: (
            robot.calibration[joint].range_min,
            robot.calibration[joint].range_max,
        )
        for servo_id, joint in enumerate(JOINTS, start=1)
    }
