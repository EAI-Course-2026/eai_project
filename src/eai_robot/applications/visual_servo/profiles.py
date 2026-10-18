"""Portable servo parameters; profiles never grant hardware execution authority."""
from pathlib import Path
import math
import tomllib


_FIELDS = {
    'target': str, 'joint_center': bool, 'marker_id': int, 'dictionary': str,
    'center_region_fraction': float, 'deadband_px': float, 'stable_frames': int,
    'base_step_counts': int, 'wrist_step_counts': int,
    'max_travel_mm': float, 'max_joint_excursion_deg': float,
    'max_joint_step_deg': float, 'joint_margin_deg': float,
    'max_runtime_s': float, 'control_hz': float, 'feedback_hz': float,
    'feature_samples': int, 'torque_settle_s': float,
}


def load_profile(path: Path) -> dict:
    document = tomllib.loads(Path(path).read_text())
    if type(document.get('schema_version')) is not int or document['schema_version'] != 1:
        raise ValueError('servo profile requires schema_version = 1')
    if set(document) - {'schema_version', 'description', 'servo'}:
        raise ValueError('unknown servo profile section')
    values = document.get('servo')
    if not isinstance(values, dict):
        raise ValueError('servo profile requires a [servo] table')
    for name, value in values.items():
        if name not in _FIELDS:
            raise ValueError(f'profile field is not allowed: {name}')
        expected = _FIELDS[name]
        if expected is float:
            valid = type(value) in (int, float)
        else:
            valid = type(value) is expected
        if not valid:
            raise ValueError(f'invalid type for profile field: {name}')
        if expected is float and not math.isfinite(value):
            raise ValueError(f'non-finite profile field: {name}')
    if values.get('target', 'aruco') not in ('aruco', 'black-ball'):
        raise ValueError('unsupported profile target')
    return values
