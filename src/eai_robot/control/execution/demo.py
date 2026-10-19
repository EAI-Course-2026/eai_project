"""Feedback-paced execution of prevalidated joint-space stages."""
import math
import time
from eai_robot.calibration.servo.results import JOINTS

def check_status(backend):
    for sid in range(1, 7):
        status = backend.read(sid, 'Status')
        if status:
            raise RuntimeError(f'ID {sid} 状态寄存器报警：0x{status:02x}')


def run_stage(arm, item, max_step=20, on_progress=None):
    """Bound intermediate lag to 25 counts; enforce final arrival separately."""
    current = arm.positions()
    arm._check_pose(current)
    origin = {cal.id: max(cal.range_min, min(cal.range_max, current[cal.id]))
              for cal in arm.calibration.values()}
    goal = item['targets']
    steps = max(1, math.ceil(max(abs(goal[s] - origin[s]) for s in goal) / max_step))
    item['substeps'] = []
    for index in range(1, steps + 1):
        check_status(arm.backend)
        target = {sid: round(origin[sid] + (goal[sid] - origin[sid]) * index / steps) for sid in goal}
        ratios = [arm.calibration[name].ratio(target[arm.calibration[name].id]) for name in JOINTS]
        entry = {'targets': target, 'reached': False}
        item['substeps'].append(entry)
        started = time.monotonic()
        arrival_tolerance = arm.tolerance
        arm.tolerance = arrival_tolerance if index == steps else max(arrival_tolerance, 25)
        entry['tolerance_counts'] = arm.tolerance
        try:
            _, feedback = arm.move(ratios)
        except Exception as failure:
            # Retain exact feedback and per-joint error for a failed substep.
            try:
                feedback = getattr(failure, 'feedback', None)
                entry['feedback_source'] = 'at_failure_before_release' if feedback is not None else 'after_release'
                if feedback is None:
                    feedback = arm.positions()
                entry['feedback'] = feedback
                entry['errors'] = {sid: feedback[sid] - target[sid] for sid in target}
            except Exception as exc:
                entry['read_error'] = str(exc)
            raise
        finally:
            arm.tolerance = arrival_tolerance
        entry.update(feedback=feedback, reached=True, duration_seconds=round(time.monotonic() - started, 3))
        check_status(arm.backend)
        if on_progress is not None:
            on_progress()
        time.sleep(.08)
    return feedback
