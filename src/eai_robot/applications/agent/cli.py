"""Provider-configurable text agent; first release supports dry-run tools only."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import tomllib

from eai_robot.calibration.servo.results import JOINTS, JointCalibration, load, validate
from eai_robot.control.planning.poses import POSES_PATH, load_poses
from eai_robot.policy.agents.contracts import ProviderError
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.providers import ChatCompletionsProvider, ProviderConfig

from .demo import DemoProvider
from .session import is_stop, run_stdin
from .skills import DryRunSkills


def load_provider(path):
    with Path(path).open("rb") as stream:
        config = tomllib.load(stream)
    if set(config) != {"provider"} or not isinstance(config["provider"], dict):
        raise ValueError("Expected only a [provider] section")
    settings = dict(config["provider"])
    if settings.pop("adapter", None) != "chat_completions":
        raise ValueError("Unsupported provider adapter")
    try:
        spec = ProviderConfig(**settings)
    except TypeError:
        raise ValueError("Invalid provider fields; store credentials in the named environment variable") from None
    return ChatCompletionsProvider(spec)


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
    source.add_argument("--demo", action="store_true", help="Fixed offline fixture; does not call a model")
    parser.add_argument("--provider-config", type=Path, help="User-selected TOML; credentials via environment")
    parser.add_argument("--poses", type=Path, default=POSES_PATH)
    parser.add_argument("--calibration", type=Path, help="Optional target calibration; otherwise use the pose file binding")
    parser.add_argument("--max-rounds", type=int, default=8)
    parser.add_argument("--max-calls", type=int, default=16)
    parser.add_argument("--turn-timeout", type=float, default=60.)
    args = parser.parse_args(argv)
    if args.demo and args.provider_config:
        parser.error("--demo does not use a provider config")
    if not args.demo and not args.provider_config:
        parser.error("--provider-config is required for a real API call")
    def output(value):
        print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
    provider = session = None
    try:
        # Validate before instantiating a provider; imports/assembly never open devices.
        skills = DryRunSkills(reference_poses(args.poses, args.calibration))
        provider = DemoProvider() if args.demo else load_provider(args.provider_config)
        session = AgentSession(provider, skills.registry(), max_rounds=args.max_rounds,
                               max_calls=args.max_calls, timeout_s=args.turn_timeout,
                               emit=lambda event: output({"event": event}))
        output({"mode": "dry_run", "provider": provider.describe(), "hardware_connected": False})
        if args.stdin:
            return run_stdin(sys.stdin, session, output)
        text = "离线固定 stand 演练" if args.demo else args.text
        if is_stop(text):
            output({"status": "cancelled", "stop": asdict(session.stop())})
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
