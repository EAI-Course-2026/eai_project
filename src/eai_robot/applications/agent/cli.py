"""Provider-configurable text agent; first release supports dry-run tools only."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from threading import Lock

from eai_robot.calibration.servo.results import JOINTS, JointCalibration, load, validate
from eai_robot.control.planning.poses import POSES_PATH, load_poses
from eai_robot.policy.model_io.contracts import ProviderError

from .demo import DemoProvider
from .configuration import load_provider
from .session import create_robot_session, is_stop, run_stdin
from .skills import DryRunSkills


def reference_poses(path, calibration_path=None):
    try:
        if calibration_path is not None:
            calibration = load(calibration_path)
        else:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            calibration = validate({name: JointCalibration(**data["calibration"][name]) for name in JOINTS})
        return load_poses(path, calibration)
    except (KeyError, TypeError):
        raise ValueError("Invalid calibration-bound pose file") from None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--text", help="One text turn")
    source.add_argument("--stdin", action="store_true", help="UTF-8, one final transcript per line")
    source.add_argument("--list-tools", action="store_true", help="Print standard OpenAI tool definitions without inference")
    source.add_argument("--demo", action="store_true", help="Fixed offline fixture; does not call a model")
    parser.add_argument("--provider-config", type=Path, help="User-selected TOML; credentials via environment")
    parser.add_argument("--env-file", type=Path, help="Explicit dotenv file; existing process environment takes precedence")
    parser.add_argument("--poses", type=Path, default=POSES_PATH)
    parser.add_argument("--calibration", type=Path, help="Optional target calibration; otherwise use the pose file binding")
    parser.add_argument("--strict-tools", action=argparse.BooleanOptionalAction, default=None,
                        help="Explicit server strict mode; omitted by default for compatible services; local validation always applies")
    parser.add_argument("--max-rounds", type=int, default=8)
    parser.add_argument("--max-calls", type=int, default=16)
    parser.add_argument("--turn-timeout", type=float, default=60.)
    args = parser.parse_args(argv)
    if (args.demo or args.list_tools) and args.provider_config:
        parser.error("--demo/--list-tools do not use a provider config")
    if args.env_file and not args.provider_config:
        parser.error("--env-file requires --provider-config")
    if not args.demo and not args.list_tools and not args.provider_config:
        parser.error("--provider-config is required for a real API call")
    output_lock = Lock()
    def output(value):
        with output_lock:
            print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
    provider = session = None
    try:
        # Validate before instantiating a provider; imports/assembly never open devices.
        skills = DryRunSkills(reference_poses(args.poses, args.calibration), strict=args.strict_tools)
        if args.list_tools:
            output({"tools": list(skills.registry().definitions), "mode": "dry_run", "hardware_connected": False})
            return 0
        provider = DemoProvider() if args.demo else load_provider(args.provider_config, args.env_file)
        session = create_robot_session(provider, skills, max_rounds=args.max_rounds,
                               max_calls=args.max_calls, timeout_s=args.turn_timeout,
                               emit=lambda event: output({"event": event}))
        output({"mode": "dry_run", "provider": provider.describe(), "hardware_connected": False})
        if args.stdin:
            return run_stdin(sys.stdin, session, output)
        text = "离线固定 stand 演练" if args.demo else args.text
        if is_stop(text):
            output({"status": "cancelled", "stop": asdict(session.cancel())})
            return 0
        result = session.run_turn(text)
        output(asdict(result))
        return 0 if result.status in {"completed", "cancelled"} else 1
    except ProviderError as exc:
        print(f"Agent provider error: {exc.code}", file=sys.stderr)
        return 1
    except (OSError, ValueError):
        # Never print input files, credentials, endpoint URLs or remote error bodies.
        print("Agent configuration/provider error; check config and credential environment.", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        if session is not None:
            session.close()
        elif provider is not None:
            provider.close()


if __name__ == "__main__":
    raise SystemExit(main())
