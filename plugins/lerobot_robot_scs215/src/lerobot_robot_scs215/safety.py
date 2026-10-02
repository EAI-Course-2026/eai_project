"""Shared feedback contract, independent of either application's entry point."""

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
MODEL_NUMBER = 1315
DEFAULT_FEEDBACK_SLACK = 3


def feedback_in_range(raw, low, high, slack=DEFAULT_FEEDBACK_SLACK):
    return type(raw) is int and 0 <= raw <= 1023 and low - slack <= raw <= high + slack
