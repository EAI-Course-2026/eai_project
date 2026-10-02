"""Automatically discovered by the pinned LeRobot third-party plugin loader."""

from .bus import SCS215MotorsBus
from .robot import SCS215SO101Follower, SCS215SO101FollowerConfig

__all__ = ["SCS215MotorsBus", "SCS215SO101Follower", "SCS215SO101FollowerConfig"]
