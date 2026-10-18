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
        while time.monotonic() < deadline:
            try:
                payload = self.path.read_bytes()
            except FileNotFoundError as exc:
                raise PreviewFrameUnavailable('shared camera frame is not available') from exc
            if len(payload) <= 8:
                raise PreviewFrameUnavailable("invalid shared camera frame")
            stamp, = struct.unpack("!d", payload[:8])
            age = time.monotonic() - stamp
            if not 0 <= age <= .25:
                raise PreviewFrameUnavailable("persistent camera preview stopped or frame is stale")
            if stamp > self.last_stamp:
                bgr = cv2.imdecode(np.frombuffer(payload[8:], dtype=np.uint8), cv2.IMREAD_COLOR)
                if bgr is None:
                    raise PreviewFrameUnavailable("shared frame JPEG could not be decoded")
                self.last_stamp = stamp
                return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            time.sleep(.005)
        raise TimeoutError("no new frame from persistent preview")

    def disconnect(self):
        # This adapter owns no capture device or preview window.
        self.is_connected = False
