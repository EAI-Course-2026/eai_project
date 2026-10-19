"""Portable servo parameters; profiles never grant hardware execution authority.

A profile is a portable record of the numbers a run used, so two things are
deliberately absent from it:

* calibration and hand-eye file paths -- a profile must not be able to repoint
  the measurements it relies on;
* detector weight paths -- weights are machine-local bytes, and letting a profile
  name them would make "the same" configuration mean a different model on a
  different machine. ``detector`` names *which* model, and the operator supplies
  the file.

``[perception]`` is optional. A marker-only profile keeps working unchanged, and
a profile that sets ``object_rule = true`` only has to add the fields of the
tracker it actually uses.
"""
from pathlib import Path
import math
import tomllib


_SERVO_FIELDS = {
    'target': str, 'joint_center': bool, 'marker_id': int, 'dictionary': str,
    'center_region_fraction': float, 'deadband_px': float, 'stable_frames': int,
    'base_step_counts': int, 'wrist_step_counts': int,
    'max_travel_mm': float, 'max_joint_excursion_deg': float,
    'max_joint_step_deg': float, 'joint_margin_deg': float,
    'max_runtime_s': float, 'control_hz': float, 'feedback_hz': float,
    'feature_samples': int, 'torque_settle_s': float,
    # Fixed-rule servo. Calibration file paths stay out of profiles on purpose:
    # a profile must be portable and cannot repoint the measurements it relies on.
    'fixed_rule': bool, 'marker_length_m': float,
    'rule_band': float, 'rule_step_mm': float, 'rule_stop_side_px': float,
    'rule_min_standoff_m': float, 'rule_gate_px': float,
    'rule_max_steps': int, 'rule_max_request_counts': float,
}

#: The object path: what to track, how the tracker is allowed to behave, and the
#: one measurement that turns apparent size into a distance.
_PERCEPTION_FIELDS = {
    'object_rule': bool, 'tracker': str, 'roi': str, 'prompt': str, 'label': str,
    'target_id': str, 'selection': str,
    'detector': str, 'model_device': str, 'model_dtype': str,
    'allow_weight_download': bool, 'detector_failure_limit': int,
    'detector_timeout_s': float,
    'tracker_max_coast_s': float, 'tracker_max_age_s': float,
    'tracker_agreement': float, 'tracker_min_inliers': int,
    'tracker_seed_margin': float, 'tracker_reacquire_attempts': int,
    'reference_depth_m': float, 'target_extent_m': float,
}

_ALLOWED_SECTIONS = {'schema_version', 'description', 'servo', 'perception'}
_TRACKERS = ('support', 'fused', 'detector')
_DEVICES = ('auto', 'cpu', 'mps', 'cuda')


def _check(table: str, values: dict, fields: dict) -> None:
    if not isinstance(values, dict):
        raise ValueError(f'profile [{table}] must be a table')
    for name, value in values.items():
        if name not in fields:
            raise ValueError(f'profile field is not allowed: {table}.{name}')
        expected = fields[name]
        valid = (type(value) in (int, float) if expected is float
                 else type(value) is expected)
        if not valid:
            raise ValueError(f'invalid type for profile field: {table}.{name}')
        if expected is float and not math.isfinite(value):
            raise ValueError(f'non-finite profile field: {table}.{name}')


def load_profile(path: Path) -> dict:
    """Flatten the profile's tables into one set of argparse defaults."""
    document = tomllib.loads(Path(path).read_text())
    if type(document.get('schema_version')) is not int or document['schema_version'] != 1:
        raise ValueError('servo profile requires schema_version = 1')
    if set(document) - _ALLOWED_SECTIONS:
        raise ValueError('unknown servo profile section')
    values = document.get('servo')
    if not isinstance(values, dict):
        raise ValueError('servo profile requires a [servo] table')
    _check('servo', values, _SERVO_FIELDS)
    if values.get('target', 'aruco') not in ('aruco', 'black-ball'):
        raise ValueError('unsupported profile target')

    perception = document.get('perception', {})
    _check('perception', perception, _PERCEPTION_FIELDS)
    overlap = set(values) & set(perception)
    if overlap:
        # Two tables setting one option makes the effective value depend on
        # merge order, which is exactly the kind of ambiguity a record must not
        # have.
        raise ValueError(f'field set in both [servo] and [perception]: {sorted(overlap)}')
    if perception.get('tracker') is not None and perception['tracker'] not in _TRACKERS:
        raise ValueError(f'unsupported profile tracker: {perception["tracker"]}')
    if perception.get('model_device') is not None and perception['model_device'] not in _DEVICES:
        raise ValueError(f'unsupported profile device: {perception["model_device"]}')
    object_rule = perception.get('object_rule', False)
    if object_rule and values.get('fixed_rule', False):
        raise ValueError('profile cannot enable both fixed_rule and object_rule')
    if object_rule and perception.get('tracker', 'fused') == 'support' \
            and not perception.get('roi'):
        raise ValueError('profile tracker "support" requires roi')
    for name in ('reference_depth_m', 'target_extent_m'):
        if object_rule and name in perception and perception[name] <= 0:
            raise ValueError(f'profile {name} must be positive')
    if object_rule and ('reference_depth_m' in perception) == ('target_extent_m' in perception):
        raise ValueError('profile object_rule needs exactly one of '
                         'reference_depth_m or target_extent_m')
    return {**values, **perception}
