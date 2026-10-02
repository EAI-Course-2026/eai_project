"""Choose LeRobot/Placo FK when available, otherwise use the NumPy backend."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Protocol, Sequence

import numpy as np

from eai_robot.course.kinematics.joint_mapping import ARM_JOINT_NAMES
from eai_robot.course.kinematics.urdf_fk import URDFFK


class ForwardKinematics(Protocol):
    def forward_kinematics(self, joint_pos_deg: Sequence[float] | np.ndarray) -> np.ndarray: ...


def create_fk_solver(
    urdf_path: str | Path,
    target_frame_name: str = "gripper_frame_link",
    *, backend: str = "numpy",
) -> tuple[ForwardKinematics, str]:
    """Return a solver and a short backend name for diagnostics."""
    if backend not in ("numpy", "placo"):
        raise ValueError("FK backend must be numpy or placo")
    if backend == "placo":
        if importlib.util.find_spec("placo") is None:
            raise RuntimeError("Placo was explicitly requested but is not installed")
        from lerobot.model.kinematics import RobotKinematics

        solver = RobotKinematics(
            urdf_path=str(urdf_path),
            target_frame_name=target_frame_name,
            joint_names=list(ARM_JOINT_NAMES),
        )
        return solver, "lerobot-placo"

    solver = URDFFK(
        urdf_path=urdf_path,
        target_frame_name=target_frame_name,
        joint_names=ARM_JOINT_NAMES,
    )
    return solver, "numpy-urdf-fallback"
