"""Explicit offline, data, remote and hardware commands for the three policy families."""
import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import sys
import time

import numpy as np

from .contracts import Manifest
from .observations import fake_observation, load_observation
from .runtime import check_fork, make_runtime


def emit(data, path=None):
    text = json.dumps(data, indent=2, allow_nan=False)
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(text + "\n")
    print(text)


def doctor(require_cuda=False):
    import torch
    check_fork()
    report = {"python": platform.python_version(), "system": platform.system(), "machine": platform.machine(), "torch": torch.__version__, "cuda_runtime": torch.version.cuda, "cuda_available": torch.cuda.is_available(), "mps_available": torch.backends.mps.is_available(), "policies": {}, "hardware_opened": False}
    from lerobot.policies.factory import get_policy_class
    for family in ("act", "smolvla", "pi05"):
        try:
            report["policies"][family] = {"import": get_policy_class(family).__name__, "status": "software-only"}
        except Exception as exc:
            report["policies"][family] = {"status": "blocked", "reason": f"{type(exc).__name__}: {exc}"}
    if report["cuda_available"]:
        report["gpus"] = []
        for i in range(torch.cuda.device_count()):
            props = torch.cuda.get_device_properties(i)
            x = torch.ones((32, 32), device=f"cuda:{i}")
            result = x @ x
            torch.cuda.synchronize(i)
            if not torch.all(result == 32).item():
                raise RuntimeError("CUDA operation failed")
            report["gpus"].append({"name": props.name, "total_memory_bytes": props.total_memory, "capability": [props.major, props.minor], "matmul": "passed"})
    report["passed"] = all(p["status"] == "software-only" for p in report["policies"].values()) and (not require_cuda or report["cuda_available"])
    if require_cuda and not report["cuda_available"]:
        report["blocker"] = "NVIDIA GPU host/access is required"
    return report


def dry_run(args):
    manifest = Manifest.load(args.manifest)
    obs = load_observation(args.observation, manifest, args.task) if args.observation else fake_observation(manifest, args.task)
    if args.remote:
        from .transport import RemoteRuntime
        runtime = RemoteRuntime(manifest, args.remote, timeout=args.timeout)
    else:
        runtime = make_runtime(manifest, args.device)
    for _ in range(args.warmup):
        runtime.predict(obs)
    if args.device == "cuda" and not args.remote:
        import torch
        torch.cuda.reset_peak_memory_stats()
    latencies = []
    for _ in range(args.repeats):
        actions, elapsed = runtime.predict(obs)
        latencies.append(elapsed)
    report = {"policy_type": manifest.data["policy_type"], "manifest_fingerprint": manifest.fingerprint, "device": "remote" if args.remote else args.device, "synthetic_input": not bool(args.observation), "shape": list(actions.shape), "finite": bool(np.isfinite(actions).all()), "range_per_joint": {name: [float(actions[:, i].min()), float(actions[:, i].max())] for i, name in enumerate(manifest.data["action_order"])}, "samples": len(latencies), "warmup": args.warmup, "latency_s": {"p50": float(np.percentile(latencies, 50)), "p95": float(np.percentile(latencies, 95)), "max": max(latencies)}, "action_horizon_s": len(actions) / manifest.data["action_hz"], "hardware_opened": False, "robot_compatibility_verified": False}
    if args.device == "cuda" and not args.remote:
        report["peak_cuda_allocated_bytes"] = torch.cuda.max_memory_allocated()
    emit(report, args.output)


def hardware(args):
    from .execution import HardwareOwner, execute_chunk
    from .recording import Journal
    from .safety import Budgets, Gate
    from .transport import RemoteRuntime
    manifest = Manifest.load(args.manifest)
    motion = args.command == "run"
    if motion and (not args.enable_motion or args.acknowledge != "MATCHED_CHECKPOINT_WITH_OPERATOR_STOP"):
        raise ValueError("run requires --enable-motion and --acknowledge MATCHED_CHECKPOINT_WITH_OPERATOR_STOP")
    budgets = Budgets(**json.loads(Path(args.budgets).read_text()))
    owner = HardwareOwner(manifest, args.hardware_config)
    # Validate before loading a model or connecting anything.
    manifest.require_robot_semantics(owner.config["calibration"], motion=motion)
    runtime = RemoteRuntime(manifest, args.remote, timeout=budgets.response_timeout_s) if args.remote else make_runtime(manifest, args.device)
    journal = Journal(args.output, manifest, mode="rollout" if motion else "shadow")
    gate = Gate(manifest, budgets)
    complete = False
    try:
        owner.connect(motion=motion)
        if motion:
            gate.arm(owner.feedback())
        stop_at = time.perf_counter() + args.duration_s
        while time.perf_counter() < stop_at:
            obs = owner.observe(args.task)
            obs.validate(manifest, now=time.perf_counter(), max_age_s=budgets.max_observation_age_s, max_skew_s=budgets.max_acquisition_skew_s)
            if motion:
                execute_chunk(owner, gate, obs, runtime, journal, stop_at=stop_at)
            else:
                actions, elapsed = runtime.predict(obs)
                journal.append(obs, requested=actions[0])
                print(json.dumps({"shadow": True, "inference_s": elapsed, "first_action": actions[0].tolist(), "writes": 0}, allow_nan=False))
        complete = True
    finally:
        gate.stop("end of finite run or operator interruption")
        try:
            owner.close()
        finally:
            journal.close(complete=complete)


def record_demo(args):
    from .execution import HardwareOwner
    from .recording import Journal
    from .safety import Budgets
    from .teleoperation import KeyboardJog, record_demonstration
    if not args.enable_motion or args.acknowledge != "OPERATOR_DEMONSTRATION_WITH_STOP":
        raise ValueError("Collection requires --enable-motion and --acknowledge OPERATOR_DEMONSTRATION_WITH_STOP")
    manifest = Manifest.load(args.manifest)
    budgets = Budgets(**json.loads(Path(args.budgets).read_text()))
    owner = HardwareOwner(manifest, args.hardware_config)
    manifest.require_robot_semantics(owner.config["calibration"])
    source = KeyboardJog(step=budgets.max_step / 2)
    journal = Journal(args.output, manifest, mode="demonstration", scene_id=args.scene_id)
    complete = False
    try:
        source.start()  # Check keyboard permission before enabling hardware.
        owner.connect(motion=True, operator_collection=True)
        print("Jog +/−: Q/A, W/S, E/D, R/F, T/G, Y/H. ESC stops. Support the arm before release.", flush=True)
        record_demonstration(owner, manifest, source, journal, budgets, args.task, args.duration_s)
        complete = True
    finally:
        source.close()
        try:
            owner.close()
        finally:
            journal.close(complete=complete)
    # Never infer task success from motion. Label only after the finite session has ended.
    outcome = input("Arm released. Type SUCCESS only if the demonstration completed the task: ").strip()
    if outcome == "SUCCESS":
        path = Path(args.output) / "episode.json"
        header = json.loads(path.read_text())
        header["outcome"] = "success"
        path.write_text(json.dumps(header, indent=2) + "\n")


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    d = sub.add_parser("doctor", help="Software/GPU inventory; never opens hardware")
    d.add_argument("--require-cuda", action="store_true")
    d.add_argument("--output")
    fetch = sub.add_parser("fetch", help="Explicit fixed-revision metadata/weight preparation")
    fetch.add_argument("--model", required=True)
    fetch.add_argument("--revision")
    fetch.add_argument("--directory", required=True)
    fetch.add_argument("--weights", action="store_true")
    fetch.add_argument("--manifest-output")
    info = sub.add_parser("checkpoint-info")
    info.add_argument("--checkpoint", required=True)
    info.add_argument("--output")
    freeze = sub.add_parser("freeze", help="Inventory an existing complete local checkpoint")
    freeze.add_argument("--checkpoint", required=True)
    freeze.add_argument("--revision", required=True)
    freeze.add_argument("--output", required=True)
    freeze.add_argument("--action-hz", type=float, required=True)
    freeze.add_argument("--assets", help="JSON of pinned local VLM/tokenizer assets")
    dry = sub.add_parser("dry-run")
    dry.add_argument("--manifest", required=True)
    dry.add_argument("--observation", help="NPZ containing state and uint8 HWC RGB camera sources")
    dry.add_argument("--task", default="Offline fixture")
    dry.add_argument("--device", choices=["cpu", "mps", "cuda"], default="cpu")
    dry.add_argument("--warmup", type=int, default=1)
    dry.add_argument("--repeats", type=int, default=5)
    dry.add_argument("--remote")
    dry.add_argument("--timeout", type=float, default=10)
    dry.add_argument("--output")
    serve = sub.add_parser("serve")
    serve.add_argument("--manifest", required=True)
    serve.add_argument("--device", choices=["cpu", "mps", "cuda"], default="cuda")
    serve.add_argument("--port", type=int, default=8081)
    for name in ("shadow", "run", "record-demo"):
        cmd = sub.add_parser(name, help="Explicit real device operation; review the deployment guide")
        cmd.add_argument("--manifest", required=True)
        cmd.add_argument("--hardware-config", required=True)
        cmd.add_argument("--budgets", required=True)
        cmd.add_argument("--task", required=True)
        cmd.add_argument("--duration-s", type=float, required=True)
        cmd.add_argument("--output", required=True, help="New episode directory")
        cmd.add_argument("--remote")
        cmd.add_argument("--device", choices=["cpu", "mps", "cuda"], default="cpu")
        if name in {"run", "record-demo"}:
            cmd.add_argument("--enable-motion", action="store_true")
            cmd.add_argument("--acknowledge")
        if name == "record-demo":
            cmd.add_argument("--scene-id", required=True)
    validate = sub.add_parser("validate-episode")
    validate.add_argument("--manifest", required=True)
    validate.add_argument("--episode", required=True)
    validate.add_argument("--demonstrations", action="store_true")
    export = sub.add_parser("export-dataset")
    export.add_argument("--manifest", required=True)
    export.add_argument("--episodes", nargs="+", required=True)
    export.add_argument("--root", required=True)
    export.add_argument("--repo-id", required=True)
    export.add_argument("--split", required=True)
    recipe = sub.add_parser("training-recipe")
    recipe.add_argument("--policy-type", choices=["act", "smolvla", "pi05"], required=True)
    recipe.add_argument("--dataset-root", required=True)
    recipe.add_argument("--repo-id", required=True)
    recipe.add_argument("--base-bundle")
    recipe.add_argument("--output-dir", required=True)
    recipe.add_argument("--output")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == "doctor":
            report = doctor(args.require_cuda)
            emit(report, args.output)
            return 0 if report["passed"] else 1
        elif args.command == "fetch":
            from .artifacts import fetch_checkpoint
            emit(fetch_checkpoint(args.model, args.directory, revision=args.revision, weights=args.weights, manifest_path=args.manifest_output))
        elif args.command == "checkpoint-info":
            from .artifacts import checkpoint_info
            emit(checkpoint_info(args.checkpoint), args.output)
        elif args.command == "freeze":
            from .artifacts import freeze_checkpoint
            assets = json.loads(Path(args.assets).read_text()) if args.assets else None
            emit({"manifest": str(freeze_checkpoint(args.checkpoint, args.output, revision=args.revision, action_hz=args.action_hz, assets=assets).path)})
        elif args.command == "dry-run":
            if args.repeats < 1 or args.warmup < 0:
                raise ValueError("repeats must be positive and warmup nonnegative")
            dry_run(args)
        elif args.command == "serve":
            from .transport import policy_server, token_from_env
            manifest = Manifest.load(args.manifest)
            token = token_from_env()
            server = policy_server(make_runtime(manifest, args.device), args.port, token=token)
            print(json.dumps({"binding": "127.0.0.1", "port": server.server_port, "manifest": manifest.fingerprint, "hardware_opened": False}), flush=True)
            try:
                server.serve_forever()
            finally:
                server.server_close()
        elif args.command in {"shadow", "run", "record-demo"}:
            if not np.isfinite(args.duration_s) or not 0 < args.duration_s <= 60:
                raise ValueError("Hardware commands require a finite duration in (0, 60] seconds")
            record_demo(args) if args.command == "record-demo" else hardware(args)
        elif args.command == "validate-episode":
            from .recording import validate_episode
            report = validate_episode(args.episode, Manifest.load(args.manifest), demonstrations=args.demonstrations)
            emit({k: v for k, v in report.items() if k != "rows"})
        elif args.command == "export-dataset":
            from .datasets import export_dataset
            emit(export_dataset(args.episodes, Manifest.load(args.manifest), root=args.root, repo_id=args.repo_id, split_file=args.split))
        elif args.command == "training-recipe":
            from .datasets import training_recipe
            emit(training_recipe(args.policy_type, args.dataset_root, args.repo_id, output_dir=args.output_dir, base_bundle=args.base_bundle), args.output)
        return 0
    except KeyboardInterrupt:
        print("Stopped by operator", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
