"""Refresh live preview while awaiting an explicit retry or supported release."""
from __future__ import annotations
import select
import sys
import time
import numpy as np


def wait_while_holding(runtime, detector, preview, stop, stream=None, poll=None):
    stream = sys.stdin if stream is None else stream
    poll = select.select if poll is None else poll
    print('\nPAUSED - TORQUE HOLD. R in preview/terminal retries; support arm then terminal ENTER releases.', flush=True)
    print('Pause reason: '+getattr(runtime, 'hold_reason', 'CONTROL PAUSED'),flush=True)
    reason = getattr(runtime,'hold_reason','PAUSED')
    label = ('RANGE/STEP LIMIT' if 'No safe' in reason else reason)[:20]
    frames = 0
    while not stop.emergency_requested.is_set():
        if preview is not None:
            try:
                frame = runtime.read_frame()
                detected, _ = detector.detect(frame)
                detail = '' if detected is None else f' x={detected.center_x:.0f} y={detected.center_y:.0f}'
                frames += 1
                status = f'PAUSED {label} | R | LIVE #{frames}' + detail
                preview.show(frame, detected, status)
            except (RuntimeError, TimeoutError):
                frame = np.zeros((480,640,3),dtype=np.uint8)
                preview.show(frame, None, 'CAMERA UNAVAILABLE - TORQUE HOLD')
                time.sleep(.05)
        if stop.emergency_requested.is_set():
            return 'emergency'
        if stop.resume_requested.is_set():
            return 'r'
        if stream is not None and poll([stream],[],[],.02)[0]:
            line = stream.readline()
            if line == '':
                # Losing terminal stdin is not authorization to release torque.
                stream = None
                print('Terminal input closed; holding with live preview.',flush=True)
            else:
                command = line.strip().lower()
                if command in ('','r'):
                    return command
                print('Use R to retry, or support the arm then ENTER to release.',flush=True)
        elif stream is None or preview is None:
            time.sleep(.02)
    return 'emergency'
