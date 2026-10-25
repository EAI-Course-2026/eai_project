"""Read fresh frames published by the persistent camera preview process."""
from pathlib import Path
import struct
import time

import cv2
import numpy as np


class PreviewFrameUnavailable(RuntimeError):
    """No usable fresh image; callers must hold, then explicitly reacquire."""


class SharedPreviewCamera:
    def __init__(self, path):
        self.path = Path(path)
        self.is_connected = False
        self.last_stamp = float("-inf")

    def connect(self):
        self.is_connected = True
        try:
            self.async_read(timeout_ms=1000)
        except BaseException:
            self.is_connected = False
            raise

    def async_read(self, timeout_ms=1000):
        if not self.is_connected:
            raise RuntimeError("shared preview camera is disconnected")
        deadline = time.monotonic() + timeout_ms / 1000
        unavailable_reason = None
        while time.monotonic() < deadline:
            try:
                payload = self.path.read_bytes()
            except (FileNotFoundError, PermissionError):
                # Atomic publication can briefly hide or lock the destination
                # on Windows. Retry within the existing freshness lease; never
                # return an older frame to work around the sharing violation.
                unavailable_reason = 'shared camera frame is not available'
                time.sleep(.005)
                continue
            if len(payload) <= 8:
                unavailable_reason = "invalid shared camera frame"
                time.sleep(.005)
                continue
            stamp, = struct.unpack("!d", payload[:8])
            age = time.monotonic() - stamp
            if not 0 <= age <= .25:
                unavailable_reason = "persistent camera preview stopped or frame is stale"
                time.sleep(.005)
                continue
            if stamp > self.last_stamp:
                bgr = cv2.imdecode(np.frombuffer(payload[8:], dtype=np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    unavailable_reason = "shared frame JPEG could not be decoded"
                    time.sleep(.005)
                    continue
                self.last_stamp = stamp
                return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            unavailable_reason = None
            time.sleep(.005)
        if unavailable_reason is not None:
            raise PreviewFrameUnavailable(unavailable_reason)
        raise TimeoutError("no new frame from persistent preview")

    def disconnect(self):
        # This adapter owns no capture device or preview window.
        self.is_connected = False
