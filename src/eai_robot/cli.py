"""Portable command dispatcher for the EAI robot application."""
import importlib
import sys

COMMANDS = {
    "mapping": "applications.tools.check_mapping",
    "fk": "applications.tools.check_fk",
    "plan": "applications.tools.cartesian_plan",
    "keyboard": "applications.teleoperation.keyboard",
    "vision": "applications.visual_servo.ball",
    "aruco-preview": "applications.visual_servo.aruco_preview",
    "visual-servo": "applications.visual_servo.cli",
    "aruco-experiment": "applications.visual_servo.experiment",
    "voice": "applications.teleoperation.voice",
    "poses": "applications.teleoperation.poses",
    "record-pose": "applications.teleoperation.record_pose",
    "reindex": "applications.tools.reindex"
}


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print("Usage: eai-course COMMAND [options]\nCommands: " + ", ".join(COMMANDS))
        return 0
    command = sys.argv.pop(1)
    if command not in COMMANDS:
        print(f"Unknown command: {command}", file=sys.stderr)
        return 2
    try:
        return importlib.import_module("eai_robot." + COMMANDS[command]).main() or 0
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
