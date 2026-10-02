"""Verify the fork and training runtime; --require-cuda also runs a small GPU operation.

This standalone script can run in the training environment, which intentionally
does not install the eai_robot application. It never opens robot/camera devices.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import platform
import sys

FORK_COMMIT = "6a077907c7989635218969ee78f5436f8faec92b"
FORK_URL = "https://github.com/EAI-Course-2026/eai-course-lerobot.git"


def check(*, require_cuda: bool = False) -> int:
    print(f"Python: {sys.version.split()[0]} ({sys.executable})")
    system = platform.system()
    print(f"Platform: {system} {platform.machine()}")
    try:
        if not (3, 12) <= sys.version_info < (3, 13):
            raise RuntimeError("The shared training environment requires Python 3.12.")
        dist = importlib.metadata.distribution("lerobot")
        source = json.loads(dist.read_text("direct_url.json") or "{}")
        commit = source.get("vcs_info", {}).get("commit_id")
        print(f"LeRobot: {dist.version}; source: {source.get('url')}; commit: {commit}")
        if dist.version != "0.6.2" or commit != FORK_COMMIT or source.get("url") != FORK_URL:
            raise RuntimeError("Expected the pinned course fork, not a PyPI or other Git installation.")

        import torch

        print(f"PyTorch: {torch.__version__}; compiled CUDA runtime: {torch.version.cuda}")
        if system in ("Windows", "Linux") and (
            str(torch.__version__) != "2.11.0+cu128" or torch.version.cuda != "12.8"
        ):
            raise RuntimeError("Expected the locked PyTorch 2.11.0+cu128 training wheel.")
        if system == "Darwin" and str(torch.__version__) != "2.11.0":
            raise RuntimeError("Expected the locked native Mac PyTorch 2.11.0 wheel.")
        available = torch.cuda.is_available()
        print(f"CUDA available: {available}")
        if not require_cuda:
            print("Software installation verified; GPU execution and training are NOT verified.")
            print("On the GPU machine, repeat with --require-cuda before starting training.")
            return 0
        if not available:
            raise RuntimeError(
                "CUDA is unavailable. Check the NVIDIA GPU/driver with nvidia-smi and "
                "use environments/training, not the root CPU environment. Mac does not use CUDA."
            )
        print(f"GPU 0: {torch.cuda.get_device_name(0)}")
        print(f"Compute capability: {torch.cuda.get_device_capability(0)}")
        print(f"Wheel architectures: {torch.cuda.get_arch_list()}")
        x = torch.ones((32, 32), device="cuda:0")
        y = x @ x
        torch.cuda.synchronize()
        if not torch.all(y == 32).item():
            raise RuntimeError("CUDA matrix multiplication produced an unexpected result.")
        print("CUDA matrix multiplication: OK (GPU 0). Training/data loading are not yet verified.")
        return 0
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-cuda", action="store_true", help="Require and exercise NVIDIA GPU 0")
    return check(require_cuda=parser.parse_args().require_cuda)


if __name__ == "__main__":
    raise SystemExit(main())
