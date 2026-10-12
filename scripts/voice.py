"""Run the voice command console without installing a second application."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.voice.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
