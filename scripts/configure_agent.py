"""Configure Agent API profiles; never sends model requests."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.applications.agent.configure import main

if __name__ == "__main__":
    raise SystemExit(main())
