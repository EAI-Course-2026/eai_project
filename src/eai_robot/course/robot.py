"""Course compatibility entry points use the same registered Robot as standard LeRobot CLIs."""

from pathlib import Path
from lerobot_robot_scs215 import (
    SCS215MotorsBus,
    SCS215SO101Follower,
    SCS215SO101FollowerConfig,
)
from eai_robot.config import ROOT, load_config
from eai_robot.arm.calibration import load

DEFAULT_CALIBRATION = ROOT / "calibration/scs215_so101.json"
CourseMotorsBus = SCS215MotorsBus
CourseFollower = SCS215SO101Follower


def default_port():
    return load_config()["serial"]["port"]


def make_robot(
    port, robot_id="scs215_so101", *, calibration_path=DEFAULT_CALIBRATION, cameras=None
):
    path = Path(calibration_path)
    load(path)
    return SCS215SO101Follower(
        SCS215SO101FollowerConfig(
            port=port,
            id=robot_id,
            calibration_path=path,
            baudrate=load_config()["serial"]["baudrate"],
            cameras=cameras or {},
            enable_motion=False,
        )
    )
