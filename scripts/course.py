"""Run migrated course tools without requiring an editable installation."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.course.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
