"""Portable command dispatcher for the EAI robot application."""
import importlib
import sys

COMMANDS = {
    "mapping": "applications.check_mapping",
    "fk": "applications.check_fk",
    "plan": "applications.cartesian_plan",
    "keyboard": "applications.keyboard_control",
    "vision": "applications.visual_closed_loop",
    "aruco-preview": "applications.aruco_preview",
    "voice": "applications.voice_cli",
    "poses": "applications.pose_control",
    "record-pose": "applications.record_pose",
    "reindex": "applications.reindex_id5",
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
