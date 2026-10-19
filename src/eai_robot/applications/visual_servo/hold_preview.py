"""Refresh live preview while awaiting an explicit retry or supported release."""
from __future__ import annotations
import select
import sys
import time
import numpy as np


def overlay_label(reason, width=20):
    """ASCII pause label for OpenCV's Hershey font; terminal keeps Unicode."""
    import re
    text = str(reason).strip()
    if text.startswith('计划步不被允许') or text.startswith('实测位姿越界'):
        text = text.split('：', 1)[-1]
    glosses = re.findall(r'（([^）]*)）', text)
    english = next((g for g in glosses if g.isascii()), None)
    if english:
        text = english
    else:
        text = ''.join(c for c in text if c.isascii()).strip()
    return text[:width].rstrip() or 'CONTROL PAUSED'


def wait_while_holding(runtime, detector, preview, stop, stream=None, poll=None):
    stream = sys.stdin if stream is None else stream
    poll = select.select if poll is None else poll
    print('\n已暂停 · 扭矩保持（PAUSED - TORQUE HOLD）', flush=True)
    print('  预览窗口或终端按 R 重试；托住机械臂后在终端按 ENTER 释放扭矩。', flush=True)
    print('  R in preview/terminal retries; support arm then terminal ENTER releases.', flush=True)
    print('暂停原因（Pause reason）: '
          + getattr(runtime, 'hold_reason', '控制已暂停 CONTROL PAUSED'), flush=True)
    label = overlay_label(getattr(runtime, 'hold_reason', 'PAUSED'))
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
                print('终端输入已关闭，继续保持扭矩并保留预览。',flush=True)
                print('Terminal input closed; holding with live preview.',flush=True)
            else:
                command = line.strip().lower()
                if command in ('','r'):
                    return command
                print('按 R 重试；或托住机械臂后按 ENTER 释放。'
                      ' (Use R to retry, or support the arm then ENTER to release.)',flush=True)
        elif stream is None or preview is None:
            time.sleep(.02)
    return 'emergency'
