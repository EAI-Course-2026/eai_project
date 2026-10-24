"""Thin Conda bootstrap; project dependencies still come from the uv lockfiles."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
UV_VERSION = "0.11.7"
PROBE = ("import json,os,sys; print(json.dumps({'prefix':sys.prefix,'python':sys.executable,"
         "'version':list(sys.version_info[:2]),'conda':os.path.isdir(os.path.join(sys.prefix,'conda-meta'))}))")


def child_environment():
    env = os.environ.copy()
    # Selection is explicit below, independent of the shell's active venv/project.
    for key in ("VIRTUAL_ENV", "PYTHONPATH", "PYTHONHOME", "UV_PROJECT_ENVIRONMENT", "UV_PYTHON",
                "UV_FROZEN", "UV_ACTIVE", "UV_SYSTEM_PYTHON"):
        env.pop(key, None)
    env.update(PYTHONUTF8="1", PYTHONIOENCODING="utf-8", PYTHONNOUSERSITE="1")
    return env


def setup(*, name, kind="control", reuse=False, skip_tests=False, require_cuda=False):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", name) or name.lower() in {"base", "root"}:
        raise ValueError("Choose a dedicated Conda environment name, not base/root")
    if kind not in {"control", "training"} or (require_cuda and kind != "training"):
        raise ValueError("--require-cuda is only valid for training")
    conda = os.environ.get("CONDA_EXE") or shutil.which("conda.exe") or shutil.which("conda")
    if not conda and os.environ.get("CONDA"):
        candidate = Path(os.environ["CONDA"]) / ("Scripts/conda.exe" if os.name == "nt" else "bin/conda")
        if candidate.is_file():
            conda = str(candidate)
    if not conda:
        raise ValueError("Conda not found; run from an initialized Conda terminal")
    env = child_environment()

    def run(args, *, capture=False, check=True, environment=None):
        return subprocess.run([conda, *args], cwd=ROOT, env=environment or env, check=check,
                              text=True, encoding="utf-8", capture_output=capture)

    def target(args, **options):
        return run(["run", "--no-capture-output", "--name", name, *args], **options)

    base = Path(run(["info", "--base"], capture=True).stdout.strip()).resolve()
    known = json.loads(run(["env", "list", "--json"], capture=True).stdout)["envs"]
    protected = {(ROOT / ".venv").resolve(), (ROOT / "environments/training/.venv").resolve()}
    matches = [Path(path).resolve() for path in known if Path(path).name.casefold() == name.casefold()]
    if len(matches) > 1:
        raise ValueError("Environment name is ambiguous; choose a unique name")
    if matches and (matches[0] == base or matches[0] in protected):
        raise ValueError("Refusing base or a uv-managed .venv")
    if matches and not reuse:
        raise ValueError("Environment exists; choose a new name or explicitly use --reuse for a dedicated project env")
    if not matches:
        run(["create", "--yes", "--name", name, "--override-channels", "--channel", "conda-forge",
             "python=3.12", "pip"])
    probe = json.loads(target(["python", "-c", PROBE], capture=True).stdout)
    prefix = Path(probe["prefix"]).resolve()
    if prefix == base or prefix in protected or not probe["conda"] or probe["version"] != [3, 12]:
        raise ValueError("Target must be a non-base Conda environment with Python 3.12")
    version = target(["python", "-m", "uv", "--version"], capture=True, check=False)
    if version.returncode or version.stdout.split()[:2] != ["uv", UV_VERSION]:
        target(["python", "-m", "pip", "install", "--disable-pip-version-check", f"uv=={UV_VERSION}"])
    project = ROOT if kind == "control" else ROOT / "environments/training"
    scoped = dict(env, UV_PROJECT_ENVIRONMENT=str(prefix))
    # Inexact sync retains Conda/pip/uv bootstrap packages. uv owns the locked deps.
    target(["python", "-m", "uv", "sync", "--locked", "--inexact", "--no-managed-python",
            "--python", probe["python"], "--project", str(project)], environment=scoped)
    target(["python", "-m", "uv", "pip", "check", "--python", probe["python"]])
    if kind == "control":
        target(["python", "scripts/check_env.py"])
        if not skip_tests:
            target(["python", "-m", "unittest", "discover", "-s", "tests", "-q"])
        target(["python", "-m", "eai_robot.applications.agent.cli", "--demo"])
    else:
        target(["python", "scripts/check_training_env.py", *(["--require-cuda"] if require_cuda else [])])
        target(["lerobot-train", "--help"])
    print(f"Ready: {name} ({kind}), Python: {probe['python']}")
    print(f"Use: conda activate {name}")
    print("Only software checks were run; no robot, camera or model API was opened.")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("control", "training"), default="control")
    parser.add_argument("--name", help="Default: eai-control or eai-training")
    parser.add_argument("--reuse", action="store_true", help="Explicitly update an existing dedicated project env; may replace dependencies")
    parser.add_argument("--skip-tests", action="store_true", help="Skip full control suite; retain environment and offline demo checks")
    parser.add_argument("--require-cuda", action="store_true", help="Training only: require a real GPU tensor check")
    args = parser.parse_args(argv)
    try:
        return setup(name=args.name or f"eai-{args.kind}", kind=args.kind, reuse=args.reuse,
                     skip_tests=args.skip_tests, require_cuda=args.require_cuda)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"Conda setup failed: {exc}")
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
