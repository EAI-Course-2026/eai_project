"""Pure pixel-to-encoder selection and feedback predicates for the MVP servo.

No cameras, serial ports, or hardware ownership belong in this module.
"""
from __future__ import annotations

import itertools

import numpy as np

JOINTS = ("shoulder_pan", "wrist_flex")
STEP = 10  # Historical numerical-helper default; live axes have separate steps.
ARRIVAL_TOLERANCE = 3


def commanded_joints(before, target):
    """Ignore unselected-axis jitter instead of producing corrective micro-steps."""
    return {
        name: int(target[name])
        for name in JOINTS
        if abs(target[name] - before[name]) > ARRIVAL_TOLERANCE
    }


def center_tolerance(shape, deadband, fraction):
    """Return horizontal/vertical half-widths of the accepted image region."""
    return np.maximum(deadband, np.array([shape[1], shape[0]]) * fraction / 2)


def joint_step_options(base_counts, wrist_counts):
    """Keep wrist starting amplitude independent of the base's smaller option."""
    base = sorted({-base_counts, -min(10, base_counts), 0,
                   min(10, base_counts), base_counts})
    return base, [-wrist_counts, 0, wrist_counts]


def choose_step(jacobian, error, allowed, step_options=None):
    """Select a permitted quantized move predicting at least 1 px improvement."""
    best = None
    best_cost = float(np.linalg.norm(error))
    options = step_options or ((-STEP, -5, 0, 5, STEP),) * 2
    for values in itertools.product(*options):
        step = np.asarray(values, dtype=int)
        if not np.any(step) or not allowed(step):
            continue
        cost = float(np.linalg.norm(error - jacobian @ step))
        if cost < best_cost - 1.0:
            best, best_cost = step, cost
    return best


def motion_confirmed(before, target, actual):
    """Arrival requires direction, actual motion, and a strictly smaller band."""
    changed = [name for name in target if target[name] != before[name]]
    return bool(changed) and all(
        abs(target[name] - before[name]) > ARRIVAL_TOLERANCE
        and (actual[name] - before[name]) * (target[name] - before[name]) > 0
        and abs(actual[name] - before[name]) >= 2
        and abs(target[name] - actual[name]) <= ARRIVAL_TOLERANCE
        for name in changed
    )


def motion_observed(before, target, actual):
    """A partial settled response is useful feedback, but is not goal arrival."""
    changed = [name for name in target if target[name] != before[name]]
    return bool(changed) and all(
        abs(actual[name] - before[name]) >= 2
        and (actual[name] - before[name]) * (target[name] - before[name]) > 0
        for name in changed
    )
