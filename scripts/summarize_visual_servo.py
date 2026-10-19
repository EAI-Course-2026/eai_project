"""Summarize saved visual-servo evidence without camera or robot access."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from eai_robot.applications.visual_servo.reporting import main

if __name__ == '__main__':
    raise SystemExit(main())
