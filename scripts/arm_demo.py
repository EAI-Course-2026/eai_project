"""Bounded coordinated six-joint demo; importing does not open a serial port."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.arm.demo import main

if __name__ == "__main__":
    raise SystemExit(main())
