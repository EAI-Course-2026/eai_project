"""Run application policy tools in either locked project, without installing a second app."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.policy.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
