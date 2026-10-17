"""Shared feedback checks for the LeRobot execution path; no device ownership."""
from __future__ import annotations
import time
from typing import TYPE_CHECKING
import numpy as np
from eai_robot.calibration.results import feedback_in_range
from eai_robot.kinematics.joint_mapping import ARM_JOINT_NAMES, URDF_LIMITS, SO101JointMapper
if TYPE_CHECKING:
    from eai_robot.robots.so101 import SO101Follower

def read_raw_motor(robot: SO101Follower, name: str) -> int:
    last_error: Exception | None = None
    for _ in range(3):
        try:
            return int(robot.bus.read("Present_Position", name, normalize=False, num_retry=5))
        except (ConnectionError, RuntimeError) as exc:
            last_error = exc
            robot.bus.port_handler.clearPort()
            time.sleep(0.08)
    assert last_error is not None
    raise last_error


def read_raw_motors(robot: SO101Follower, names: tuple[str, ...]) -> dict[str, int]:
    return {name: read_raw_motor(robot, name) for name in names}


def arm_degrees_from_raw(mapper: SO101JointMapper, raw: dict[str, int]) -> np.ndarray:
    return np.asarray(
        [mapper.raw_to_urdf_degrees(name, raw[name]) for name in ARM_JOINT_NAMES],
        dtype=float,
    )


def raw_outside_calibration(mapper: SO101JointMapper, raw: dict[str, int]) -> list[str]:
    outside = []
    for name in raw:
        calibration = mapper.calibration[name]
        if not feedback_in_range(raw[name], calibration.range_min, calibration.range_max):
            outside.append(name)
    return outside


def raw_range_excess(mapper: SO101JointMapper, raw: dict[str, int], name: str) -> int:
    calibration = mapper.calibration[name]
    if raw[name] < calibration.range_min:
        return calibration.range_min - raw[name]
    if raw[name] > calibration.range_max:
        return raw[name] - calibration.range_max
    return 0


def ensure_execution_start_is_safe(q_degrees: np.ndarray, margin_deg: float = 2.0) -> None:
    unsafe = []
    for name, value in zip(ARM_JOINT_NAMES, q_degrees, strict=True):
        limit = URDF_LIMITS[name]
        if value <= limit.lower_deg + margin_deg or value >= limit.upper_deg - margin_deg:
            unsafe.append(f"{name}={value:.2f} deg")
    if unsafe:
        raise RuntimeError(
            "Refusing execution because the start pose is within 2 degrees of a joint limit: "
            + ", ".join(unsafe)
        )


def wait_for_waypoint(
    robot: SO101Follower,
    mapper: SO101JointMapper,
    target_degrees: np.ndarray,
    tolerance_deg: float,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    while True:
        raw = {name: read_raw_motor(robot, name) for name in ARM_JOINT_NAMES}
        measured = arm_degrees_from_raw(mapper, raw)
        error = float(np.max(np.abs(measured - target_degrees)))
        if error <= tolerance_deg:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Waypoint timeout; maximum joint error is {error:.2f} degrees")
        time.sleep(0.05)
