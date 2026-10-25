"""Provider-configurable text agent with dry-run and explicitly armed visual hardware modes."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from threading import Lock
import time

from eai_robot.calibration.servo.results import JOINTS, JointCalibration, load, validate
from eai_robot.control.planning.poses import POSES_PATH, load_poses
from eai_robot.policy.model_io.contracts import ProviderError

from .demo import DemoProvider, VisualDemoProvider
from .capabilities import HARDWARE_VISUAL_TOOLS, RobotCapabilities
from .configuration import load_provider
from .hardware_visual import (
    HARDWARE_ACKNOWLEDGEMENT, ArucoCenterHardwareBackend, HardwareVisualSkills,
)
from .session import create_robot_session, is_stop, run_stdin
from .skills import DryRunSkills


ACTIVE_HARDWARE_STATES = {"starting", "calibrating", "moving", "running", "stopping"}


def wait_for_hardware_completion(skills, output_fn=print, sleep_fn=time.sleep,
                                 poll_s=0.2):
    """Keep a successful one-turn CLI alive while its hardware worker runs."""
    previous = None
    while True:
        snapshot = skills.backend.snapshot()
        state = snapshot.get("tracking")
        if state != previous:
            output_fn({"hardware_monitor": snapshot})
            previous = state
        if state not in ACTIVE_HARDWARE_STATES:
            return snapshot
        sleep_fn(poll_s)


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


def release_hardware(skills, input_fn=None, output_fn=None):
    """Stop the worker, then require a local human before torque release."""
    input_fn = input if input_fn is None else input_fn
    output_fn = print if output_fn is None else output_fn
    try:
        skills.stop({})
    except BaseException:
        output_fn("自动停止没有完成；请立即按实体急停或切断舵机电源，并继续托住机械臂。",
                  flush=True)
        return False
    snapshot = skills.backend.snapshot()
    if snapshot.get("torque_enabled"):
        output_fn("追踪已停止，扭矩仍保持。请先用手托住机械臂。", flush=True)
        while True:
            try:
                reply = input_fn('托稳后输入 "RELEASE TORQUE" 并回车：').strip()
            except KeyboardInterrupt:
                output_fn("尚未释放扭矩；请继续托住机械臂并完成确认。", flush=True)
                continue
            except EOFError:
                output_fn("无法读取释放确认；扭矩状态未确认，请立即按实体急停或切断舵机电源。",
                          flush=True)
                return False
            if reply == "RELEASE TORQUE":
                break
            output_fn("确认文字不匹配，扭矩保持；请托住机械臂后重新输入。", flush=True)
    try:
        skills.release()
    except BaseException:
        output_fn("自动释放没有完成；请继续托住机械臂，并立即按实体急停或切断舵机电源。",
                  flush=True)
        return False
    output_fn("真实视觉后端已关闭，扭矩释放流程完成。", flush=True)
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--text", help="One text turn")
    source.add_argument("--stdin", action="store_true", help="UTF-8, one final transcript per line")
    source.add_argument("--list-tools", action="store_true", help="Print standard OpenAI tool definitions without inference")
    source.add_argument("--demo", action="store_true", help="Fixed offline fixture; does not call a model")
    source.add_argument("--visual-demo", action="store_true",
                        help="Fixed detect/track/stop fixture; no model, camera or robot")
    parser.add_argument("--provider-config", type=Path, help="User-selected TOML; credentials via environment")
    parser.add_argument("--env-file", type=Path, help="Explicit dotenv file; existing process environment takes precedence")
    parser.add_argument("--poses", type=Path, default=POSES_PATH)
    parser.add_argument("--calibration", type=Path, help="Optional target calibration; otherwise use the pose file binding")
    parser.add_argument("--strict-tools", action=argparse.BooleanOptionalAction, default=None,
                        help="Explicit server strict mode; omitted by default for compatible services; local validation always applies")
    parser.add_argument("--max-rounds", type=int, default=8)
    parser.add_argument("--max-calls", type=int, default=16)
    parser.add_argument("--turn-timeout", type=float, default=60.)
    parser.add_argument("--visual-hardware-profile", type=Path,
                        help="Enable the operator-armed ArUco centre-tracking backend")
    parser.add_argument("--device-port", help="Explicit SCS215 serial port for hardware Agent mode")
    parser.add_argument("--camera-index", type=int, help="Explicit wrist-camera index for hardware Agent mode")
    parser.add_argument("--shared-frame", type=Path,
                        help="Read fresh frames from a separate persistent preview process")
    parser.add_argument("--hardware-max-runtime", type=float, default=60.,
                        help="Tracking runtime cap in seconds (5..120)")
    parser.add_argument("--enable-visual-motion", action="store_true",
                        help="Required explicit authority for hardware visual tracking")
    parser.add_argument("--acknowledge", help="Required hardware-mode operator acknowledgement")
    args = parser.parse_args(argv)
    hardware_mode = args.visual_hardware_profile is not None
    if (args.demo or args.visual_demo or args.list_tools) and args.provider_config:
        parser.error("--demo/--visual-demo/--list-tools do not use a provider config")
    if args.env_file and not args.provider_config:
        parser.error("--env-file requires --provider-config")
    if not args.demo and not args.visual_demo and not args.list_tools and not args.provider_config:
        parser.error("--provider-config is required for a real API call")
    hardware_fields = (args.enable_visual_motion, args.acknowledge is not None,
                       args.device_port is not None, args.camera_index is not None,
                       args.shared_frame is not None)
    if not hardware_mode and any(hardware_fields):
        parser.error("hardware options require --visual-hardware-profile")
    if hardware_mode:
        if args.text is None or args.stdin or args.demo or args.visual_demo or args.list_tools:
            parser.error("first hardware acceptance supports exactly one --text turn")
        if (not args.enable_visual_motion or args.acknowledge != HARDWARE_ACKNOWLEDGEMENT
                or args.device_port is None or args.camera_index is None):
            parser.error("hardware mode requires explicit motion flag, acknowledgement, port and camera")
        if not sys.stdin.isatty():
            parser.error("hardware mode requires an interactive terminal for torque release")
    output_lock = Lock()
    def output(value):
        with output_lock:
            print(json.dumps(value, ensure_ascii=False, allow_nan=False), flush=True)
    provider = session = skills = None
    exit_code = 1
    try:
        # Validate before instantiating a provider; imports/assembly never open devices.
        if hardware_mode:
            backend = ArucoCenterHardwareBackend(
                args.visual_hardware_profile, port=args.device_port,
                camera_index=args.camera_index, max_runtime_s=args.hardware_max_runtime,
                shared_frame=args.shared_frame)
            skills = HardwareVisualSkills(backend, strict=args.strict_tools)
            capabilities = RobotCapabilities(
                allowed_tools=HARDWARE_VISUAL_TOOLS, mode="aruco_center_hardware",
                physical_execution=True,
                evidence="Operator-armed 2026-10-18 ArUco centre-region MVP envelope")
        else:
            skills = DryRunSkills(reference_poses(args.poses, args.calibration), strict=args.strict_tools)
            capabilities = None
        if args.list_tools:
            output({"tools": list(skills.registry().definitions), "mode": "dry_run", "hardware_connected": False})
            exit_code = 0
            return exit_code
        provider = (DemoProvider() if args.demo else VisualDemoProvider() if args.visual_demo
                    else load_provider(args.provider_config, args.env_file))
        session = create_robot_session(provider, skills, capabilities=capabilities,
                               hardware_visual=hardware_mode, max_rounds=args.max_rounds,
                               max_calls=args.max_calls, timeout_s=args.turn_timeout,
                               emit=lambda event: output({"event": event}))
        output({"mode": "aruco_center_hardware" if hardware_mode else "dry_run",
                "provider": provider.describe(), "hardware_connected": False,
                "motion_authorized": hardware_mode})
        if args.stdin:
            exit_code = run_stdin(sys.stdin, session, output)
            return exit_code
        text = ("离线固定 stand 演练" if args.demo else
                "离线 ArUco 视觉追踪演练" if args.visual_demo else args.text)
        if is_stop(text):
            output({"status": "cancelled", "stop": asdict(session.cancel())})
            exit_code = 0
        else:
            result = session.run_turn(text)
            output(asdict(result))
            if hardware_mode and result.status == "completed":
                wait_for_hardware_completion(skills, output)
            exit_code = 0 if result.status in {"completed", "cancelled"} else 1
    except ProviderError as exc:
        print(f"Agent provider error: {exc.code}", file=sys.stderr)
        exit_code = 1
    except (OSError, ValueError):
        # Never print input files, credentials, endpoint URLs or remote error bodies.
        print("Agent configuration/provider error; check config and credential environment.", file=sys.stderr)
        exit_code = 1
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        if session is not None:
            session.close()
        elif provider is not None:
            provider.close()
        if hardware_mode and skills is not None:
            if not release_hardware(skills):
                exit_code = 2
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
