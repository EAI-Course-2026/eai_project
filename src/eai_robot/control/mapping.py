"""Compatibility helper for SCS215 ratios; canonical mapping uses JointCalibration."""
from eai_robot.calibration.results import JointCalibration


def ratio_to_position(ratio, minimum, maximum, direction=1):
    if not 0 <= minimum < maximum <= 1023 or direction not in (-1, 1):
        raise ValueError("SCS215 范围应为 0..1023 内的递增区间，方向为 ±1")
    return JointCalibration(1, int(direction == -1), 0, minimum, maximum).position(ratio)
