"""Explicit headless device ownership and read-only timestamped observation capture."""
from contextlib import contextmanager
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import numpy as np

from .contracts import JOINTS, KEYS, vector
from .observations import Observation


class PortLease:
    """Policy-runner process exclusion; existing GUI ownership is checked separately."""
    def __init__(self, port):
        key = hashlib.sha256(os.path.realpath(port).encode()).hexdigest()
        self.path = Path(tempfile.gettempdir()) / f"eai-policy-port-{key}.lock"
        self.stream = None

    def acquire(self):
        self.stream = self.path.open("a+b")
        try:
            if os.name == "nt":
                import msvcrt
                self.stream.seek(0)
                if self.path.stat().st_size == 0:
                    self.stream.write(b"0")
                    self.stream.flush()
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError):
            self.stream.close()
            self.stream = None
            raise RuntimeError("Another policy runner owns this serial port")

    def release(self):
        if self.stream:
            self.stream.close()
            self.stream = None


class HardwareOwner:
    def __init__(self, manifest, hardware_config):
        self.manifest = manifest
        self.config = json.loads(Path(hardware_config).read_text())
        self.robot = None
        self.thread_id = None
        self.lease = PortLease(self.config["port"])

    def _thread(self):
        if self.thread_id != threading.get_ident():
            raise RuntimeError("All device access must use the owning hardware thread")

    def connect(self, *, motion=False, operator_collection=False):
        if self.robot is not None:
            raise RuntimeError("Already connected")
        calibration = Path(self.config["calibration"])
        self.manifest.require_robot_semantics(calibration, motion=motion and not operator_collection)
        required = {s["source"] for s in self.manifest.data["cameras"].values()}
        if set(self.config.get("cameras", {})) != required:
            raise ValueError("Hardware config must supply each distinct required real camera")
        devices = [str(c["index_or_path"]) for c in self.config["cameras"].values()]
        if len(set(devices)) != len(devices):
            raise ValueError("Duplicate physical camera device")
        self.lease.acquire()
        try:
            if os.name != "nt":
                # Fail closed if process ownership cannot be inspected on Unix.
                check = subprocess.run(["lsof", "-t", "--", self.config["port"]], capture_output=True, text=True)
                if check.stdout.strip():
                    raise RuntimeError("Serial port is already open; disconnect GUI/other controllers first")
                if check.returncode not in (0, 1):
                    raise RuntimeError("Cannot inspect existing serial ownership")
            from lerobot.cameras.opencv import OpenCVCameraConfig
            from lerobot_robot_scs215 import SCS215SO101Follower, SCS215SO101FollowerConfig
            cameras = {name: OpenCVCameraConfig(**camera) for name, camera in self.config["cameras"].items()}
            robot = SCS215SO101Follower(SCS215SO101FollowerConfig(port=self.config["port"], baudrate=self.config.get("baudrate", 1_000_000), calibration_path=calibration, cameras=cameras, enable_motion=False))
            self.robot = robot
            self.thread_id = threading.get_ident()
            robot.connect()
            # Enforce exclusive mode on supported serial implementations, after SDK opens.
            if os.name != "nt":
                import fcntl
                import termios
                if not hasattr(termios, "TIOCEXCL"):
                    raise RuntimeError("OS does not provide exclusive terminal device ownership")
                fcntl.ioctl(robot.bus.port_handler.ser.fileno(), termios.TIOCEXCL)
                robot.bus.port_handler.ser.exclusive = True
            if motion:
                robot.enable_motion()
        except BaseException:
            self.close()
            raise

    def feedback(self):
        self._thread()
        bus = self.robot.bus
        values = [float(bus.read("Present_Position", name, num_retry=0)) for name in JOINTS]
        return vector(values)

    def health(self):
        self._thread()
        bus = self.robot.bus
        torques = [bus.read("Torque_Enable", n, normalize=False, num_retry=0) for n in JOINTS]
        alarms = [bus.read("Status", n, normalize=False, num_retry=0) for n in JOINTS]
        return {"torque_ok": all(value == 1 for value in torques), "alarm": any(value != 0 for value in alarms)}

    def observe(self, task):
        self._thread()
        started = time.perf_counter()
        state = self.feedback()
        images, camera_times = {}, {}
        for name, camera in self.robot.cameras.items():
            camera.async_read()  # Starts the upstream background capture if needed.
            with camera.frame_lock:
                frame, stamp = camera.latest_frame, camera.latest_timestamp
                if frame is None or stamp is None:
                    raise RuntimeError(f"{name}: no timestamped camera frame")
                images[name] = frame.copy()
                camera_times[name] = stamp
        obs = Observation(state, images, task, started, time.perf_counter(), camera_times)
        obs.validate(self.manifest)
        return obs

    def send(self, action, *, before_write):
        self._thread()
        vector(action)
        sent = self.robot.send_action(dict(zip(KEYS, map(float, action))), before_write=before_write)
        # Verify the target register separately from encoder tracking.
        for name, value in zip(JOINTS, action):
            goal = self.robot.bus.read("Goal_Position", name, num_retry=0)
            # One encoder tick plus rounding tolerance in normalized units.
            c = self.robot.calibration[name]
            tolerance = (100 if name == "gripper" else 200) / (c.range_max - c.range_min) + 1e-6
            if abs(goal - value) > tolerance:
                raise RuntimeError(f"{name}: goal register verification failed")
        return sent

    def close(self):
        try:
            if self.robot is not None:
                self._thread()
                self.robot.disconnect()
        finally:
            self.robot = None
            self.lease.release()


def execute_chunk(owner, gate, observation, runtime, journal=None, *, stop_at=None):
    """One synchronous chunk: caller owns connection, operator stop and finite duration."""
    from .transport import RemoteRuntime
    ticket = gate.request(observation)
    try:
        actions, elapsed = runtime.predict(observation, ticket) if isinstance(runtime, RemoteRuntime) else runtime.predict(observation)
        gate.accept(ticket, actions)
        while gate.queue:
            state = owner.feedback()
            action = gate.next_action(state, **owner.health())
            if action is None:
                time.sleep(min(0.005, 1 / gate.manifest.data["action_hz"]))
                continue

            def before_write():
                if not gate.active or gate.clock() >= ticket.expires_at or (stop_at is not None and time.perf_counter() >= stop_at):
                    gate._reject("Lease expired during hardware validation")

            sent = owner.send(action, before_write=before_write)
            feedback = owner.feedback()
            if journal:
                # The originating observation labels this prediction, not a newly acquired frame.
                # Rollout journals cannot be exported as operator demonstrations.
                journal.append(observation, requested=action, sent=sent, feedback=feedback, feedback_at=time.perf_counter())
        return elapsed
    except BaseException as exc:
        gate.stop(f"Execution stopped: {type(exc).__name__}: {exc}")
        if journal:
            journal.append(observation, rejection=gate.reason)
        raise
