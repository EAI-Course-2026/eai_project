"""One portable command dispatcher for the migrated course tools."""
import importlib
import sys

COMMANDS = {
    "mapping": "kinematics.check_step1",
    "fk": "kinematics.check_step2_fk",
    "plan": "kinematics.run_steps3_to5",
    "keyboard": "kinematics.keyboard_control",
    "vision": "vision.visual_closed_loop",
    "poses": "poses.control_presets",
    "record-pose": "poses.record_pose",
    "reindex": "kinematics.reindex_id5",
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
        return importlib.import_module("eai_robot.course." + COMMANDS[command]).main() or 0
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
