"""SCS215 arm workflow using only pyserial (no LeRobot or servo SDK)."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.arm.cli import main

if __name__ == "__main__":
    raise SystemExit(main("serial"))
