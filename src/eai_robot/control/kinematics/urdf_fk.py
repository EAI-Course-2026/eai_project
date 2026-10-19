"""Small URDF forward-kinematics backend for environments without Placo.

The public ``forward_kinematics`` method intentionally follows LeRobot's
``RobotKinematics`` convention: joint inputs are degrees and the result is a
4x4 homogeneous transform in the URDF base frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence
from xml.etree import ElementTree

import numpy as np


@dataclass(frozen=True)
class URDFJoint:
    name: str
    joint_type: str
    parent: str
    child: str
    xyz: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray
    lower_rad: float | None
    upper_rad: float | None


def _parse_vector(text: str | None, default: str) -> np.ndarray:
    values = [float(value) for value in (text or default).split()]
    if len(values) != 3:
        raise ValueError(f"Expected three values, got: {text!r}")
    return np.asarray(values, dtype=float)


def _translation(xyz: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=float)
    transform[:3, 3] = xyz
    return transform


def _rpy_rotation(rpy: np.ndarray) -> np.ndarray:
    """Return the URDF fixed-axis roll-pitch-yaw rotation matrix."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def _origin_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    transform = _translation(xyz)
    transform[:3, :3] = _rpy_rotation(rpy)
    return transform


def _axis_angle_transform(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    norm = float(np.linalg.norm(axis))
    if norm == 0:
        raise ValueError("A revolute joint cannot have a zero rotation axis")
    x, y, z = axis / norm
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    one_minus_c = 1.0 - c

    rotation = np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ],
        dtype=float,
    )
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    return transform


class URDFFK:
    """Forward kinematics for one serial chain selected from a URDF tree."""

    def __init__(
        self,
        urdf_path: str | Path,
        target_frame_name: str,
        joint_names: Sequence[str],
    ) -> None:
        self.urdf_path = Path(urdf_path)
        self.target_frame_name = target_frame_name
        self.joint_names = list(joint_names)
        self._joints = self._load_joints()
        self.chain = self._build_chain()

        chain_active_names = [joint.name for joint in self.chain if joint.joint_type != "fixed"]
        if chain_active_names != self.joint_names:
            raise ValueError(
                "URDF active chain does not match requested joint order: "
                f"chain={chain_active_names}, requested={self.joint_names}"
            )

    def _load_joints(self) -> list[URDFJoint]:
        root = ElementTree.parse(self.urdf_path).getroot()
        joints: list[URDFJoint] = []

        for node in root.findall("joint"):
            name = node.attrib["name"]
            joint_type = node.attrib["type"]
            parent_node = node.find("parent")
            child_node = node.find("child")
            if parent_node is None or child_node is None:
                raise ValueError(f"Joint {name} is missing parent or child")

            origin_node = node.find("origin")
            xyz = _parse_vector(origin_node.get("xyz") if origin_node is not None else None, "0 0 0")
            rpy = _parse_vector(origin_node.get("rpy") if origin_node is not None else None, "0 0 0")

            axis_node = node.find("axis")
            axis = _parse_vector(axis_node.get("xyz") if axis_node is not None else None, "1 0 0")

            limit_node = node.find("limit")
            lower = float(limit_node.get("lower")) if limit_node is not None and limit_node.get("lower") else None
            upper = float(limit_node.get("upper")) if limit_node is not None and limit_node.get("upper") else None

            joints.append(
                URDFJoint(
                    name=name,
                    joint_type=joint_type,
                    parent=parent_node.attrib["link"],
                    child=child_node.attrib["link"],
                    xyz=xyz,
                    rpy=rpy,
                    axis=axis,
                    lower_rad=lower,
                    upper_rad=upper,
                )
            )
        return joints

    def _build_chain(self) -> list[URDFJoint]:
        child_to_joint = {joint.child: joint for joint in self._joints}
        chain_reversed: list[URDFJoint] = []
        link = self.target_frame_name

        while link in child_to_joint:
            joint = child_to_joint[link]
            chain_reversed.append(joint)
            link = joint.parent

        if not chain_reversed:
            raise ValueError(f"Target frame {self.target_frame_name!r} is not in the URDF joint tree")
        return list(reversed(chain_reversed))

    def forward_kinematics(
        self, joint_pos_deg: Sequence[float] | np.ndarray | Mapping[str, float]
    ) -> np.ndarray:
        if isinstance(joint_pos_deg, Mapping):
            positions = {name: float(joint_pos_deg[name]) for name in self.joint_names}
        else:
            values = np.asarray(joint_pos_deg, dtype=float).reshape(-1)
            if len(values) < len(self.joint_names):
                raise ValueError(
                    f"Expected at least {len(self.joint_names)} positions, got {len(values)}"
                )
            positions = dict(zip(self.joint_names, values, strict=True))

        transform = np.eye(4, dtype=float)
        for joint in self.chain:
            transform = transform @ _origin_transform(joint.xyz, joint.rpy)
            if joint.joint_type in {"revolute", "continuous"}:
                angle_rad = np.deg2rad(positions[joint.name])
                transform = transform @ _axis_angle_transform(joint.axis, angle_rad)
            elif joint.joint_type != "fixed":
                raise NotImplementedError(f"Unsupported URDF joint type: {joint.joint_type}")
        return transform

    def joint_limits_degrees(self) -> dict[str, tuple[float, float]]:
        limits: dict[str, tuple[float, float]] = {}
        for joint in self.chain:
            if joint.name not in self.joint_names:
                continue
            if joint.lower_rad is None or joint.upper_rad is None:
                raise ValueError(f"Joint {joint.name} has no finite URDF limits")
            limits[joint.name] = (
                float(np.rad2deg(joint.lower_rad)),
                float(np.rad2deg(joint.upper_rad)),
            )
        return limits
