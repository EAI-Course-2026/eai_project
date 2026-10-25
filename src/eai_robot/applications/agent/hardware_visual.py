"""Explicitly armed adapter for the validated ArUco centre-tracking MVP.

Importing this module does not open a camera, serial port, or motor bus.  Device
imports and connections happen only inside ``detect``/``start`` after the CLI
has checked the operator acknowledgement.  Tracking stops in torque-hold; only
the local operator-facing release path may disable torque.
"""
from __future__ import annotations

import math
from pathlib import Path
from threading import Event, RLock, Thread, current_thread
import time
import uuid

from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.tools import Tool, ToolRegistry

from .skills import normalize_target_description, parameters


HARDWARE_ACKNOWLEDGEMENT = "ARUCO_CENTER_MVP_WITH_OPERATOR_STOP"
_VALIDATED_CENTER_PROFILE = {
    "target": "aruco", "joint_center": True, "marker_id": 0,
    "dictionary": "DICT_4X4_50", "center_region_fraction": 0.2,
    "deadband_px": 30.0, "stable_frames": 8, "base_step_counts": 15,
    "wrist_step_counts": 12, "max_travel_mm": 100.0,
    "max_joint_excursion_deg": 40.0, "max_joint_step_deg": 5.0,
    "joint_margin_deg": 2.0, "max_runtime_s": 0.0, "control_hz": 5.0,
    "feedback_hz": 2.0, "feature_samples": 3, "torque_settle_s": 1.0,
}


def _validate_profile(path: Path) -> None:
    from eai_robot.applications.visual_servo.profiles import load_profile

    values = load_profile(path)
    if values != _VALIDATED_CENTER_PROFILE:
        raise ValueError("Hardware Agent requires the unchanged validated ArUco centre MVP profile")


class ArucoCenterHardwareBackend:
    """Own one camera/bus runtime and expose bounded observe/start/stop/release."""

    def __init__(self, profile: Path, *, port: str, camera_index: int,
                 max_runtime_s: float = 60.0, shared_frame: Path | None = None):
        profile = Path(profile)
        if not profile.is_file():
            raise ValueError("Visual-servo profile does not exist")
        _validate_profile(profile)
        if not isinstance(port, str) or not port.strip() or len(port) > 128:
            raise ValueError("A bounded hardware serial port is required")
        if type(camera_index) is not int or camera_index < 0:
            raise ValueError("A non-negative camera index is required")
        if (type(max_runtime_s) not in (int, float) or not math.isfinite(max_runtime_s)
                or not 5 <= max_runtime_s <= 120):
            raise ValueError("max_runtime_s must be within [5, 120]")
        self.profile = profile
        self.port = port.strip()
        self.camera_index = camera_index
        self.max_runtime_s = float(max_runtime_s)
        self.shared_frame = None if shared_frame is None else Path(shared_frame)
        self._lock = RLock()
        self._thread = None
        self._preview_thread = None
        self._preview_stop = Event()
        self._ready = None
        self._cancel_requested = Event()
        self._runtime = None
        self._stop_state = None
        self._state = "idle"
        self._hardware_sent = False
        self._motion_command_count = 0
        self._motion_confirmed_count = 0
        self._torque_enabled = False
        self._connected = False
        self._error_code = None
        self._hold_reason = None
        self._frame = None
        self._frame_sequence = 0

    def _publish_frame(self, frame, detection, status):
        import cv2
        from eai_robot.vision.detection.ball import annotate_frame

        rendered = annotate_frame(frame, detection, status)
        ok, encoded = cv2.imencode(".jpg", rendered, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if ok:
            with self._lock:
                self._frame = encoded.tobytes()
                self._frame_sequence += 1

    class _PreviewSink:
        def __init__(self, owner):
            self.owner = owner

        def show(self, frame, detection, status):
            self.owner._publish_frame(frame, detection, status)

        def close(self):
            pass

    def frame(self):
        with self._lock:
            return self._frame, self._frame_sequence

    def _args(self, execute: bool):
        from eai_robot.applications.visual_servo.ball import parse_args

        argv = ["--profile", str(self.profile), "--port", self.port,
                "--camera-index", str(self.camera_index),
                "--max-runtime-s", str(self.max_runtime_s)]
        if self.shared_frame is not None:
            argv.extend(["--shared-frame", str(self.shared_frame)])
        if execute:
            argv.append("--execute")
        return parse_args(argv)

    def snapshot(self):
        with self._lock:
            return {
                "mode": "aruco_center_hardware",
                "connected": self._connected,
                "tracking": self._state,
                "hardware_sent": self._hardware_sent,
                "motion_command_count": self._motion_command_count,
                "motion_confirmed_count": self._motion_confirmed_count,
                "torque_enabled": self._torque_enabled,
                "requires_operator_release": self._torque_enabled,
                "error_code": self._error_code,
                "hold_reason": self._hold_reason,
                "camera_preview": bool(self._preview_thread and self._preview_thread.is_alive()),
                "frame_sequence": self._frame_sequence,
                "frame_source": "shared" if self.shared_frame is not None else "camera",
            }

    def _motion_command_sent(self, _goals):
        with self._lock:
            self._hardware_sent = True
            self._motion_command_count += 1
            self._state = "moving"

    def _motion_confirmed(self, _goals, _actual):
        with self._lock:
            self._motion_confirmed_count += 1
            self._state = "running"
            if self._ready is not None:
                self._ready.set()

    def start_preview(self):
        with self._lock:
            if self._torque_enabled or self._thread is not None and self._thread.is_alive():
                raise RuntimeError("tracking_or_torque_active")
            if self._preview_thread is not None and self._preview_thread.is_alive():
                return self.snapshot()
            self._preview_stop.clear()
            self._error_code = None
            self._preview_thread = Thread(target=self._run_preview,
                                          name="agent-camera-preview", daemon=True)
            self._preview_thread.start()
            return self.snapshot()

    def _run_preview(self):
        from eai_robot.applications.visual_servo.ball import (
            ArucoCenterDetector, StopState, VisionArmRuntime,
        )
        runtime = VisionArmRuntime(self._args(False), StopState())
        try:
            runtime.connect(camera_only=True)
            detector = ArucoCenterDetector(0, "DICT_4X4_50")
            while not self._preview_stop.is_set():
                frame = runtime.read_frame()
                detected, _ = detector.detect(frame)
                status = "ARUCO 0" if detected else "WAITING FOR ARUCO 0"
                self._publish_frame(frame, detected, status)
        except Exception:
            with self._lock:
                self._error_code = "camera_preview_failed"
        finally:
            runtime.shutdown()

    def stop_preview(self, wait_s=3.0):
        with self._lock:
            self._preview_stop.set()
            thread = self._preview_thread
        if thread is not None and thread.is_alive() and thread is not current_thread():
            thread.join(max(0.0, wait_s))
        with self._lock:
            if thread is None or not thread.is_alive():
                self._preview_thread = None
            return self.snapshot()

    def detect(self):
        from eai_robot.applications.visual_servo.ball import (
            ArucoCenterDetector, StopState, VisionArmRuntime, median_detection,
        )

        preview = self.stop_preview()
        if preview["camera_preview"]:
            raise RuntimeError("camera_preview_stop_timeout")
        with self._lock:
            if ((self._thread is not None and self._thread.is_alive())
                    or self._torque_enabled):
                raise RuntimeError("tracking_or_torque_active")
            self._state = "observing"
            self._error_code = None
        args = self._args(False)
        stop = StopState()
        runtime = VisionArmRuntime(args, stop)
        try:
            runtime.connect(camera_only=True)
            detector = ArucoCenterDetector(args.marker_id, args.dictionary)
            observed = median_detection(runtime, detector, args.feature_samples)
            if detector.last_marker_id != args.marker_id:
                raise RuntimeError("marker_identity_mismatch")
            frame = runtime.read_frame()
            detected, _ = detector.detect(frame)
            self._publish_frame(frame, detected, "ARUCO 0 DETECTED")
            diameter = max(1.0, observed.radius * 2.0)
            return {
                "target_type": "aruco marker", "label": f"aruco:{args.marker_id}",
                "detector": "ArucoCenterDetector", "center_px": observed.center.tolist(),
                "bbox_px": [observed.center_x - observed.radius,
                            observed.center_y - observed.radius, diameter, diameter],
                "confidence": observed.confidence, "camera_observed": True,
                "hardware_sent": False,
            }
        finally:
            runtime.shutdown()
            with self._lock:
                if self._state == "observing":
                    self._state = "idle"

    def start(self, startup_timeout_s: float):
        preview = self.stop_preview()
        if preview["camera_preview"]:
            return {**self.snapshot(), "error_code": "camera_preview_stop_timeout"}
        with self._lock:
            if ((self._thread is not None and self._thread.is_alive())
                    or self._torque_enabled):
                return {**self.snapshot(), "error_code": "tracking_busy"}
            self._ready = Event()
            self._cancel_requested.clear()
            self._state = "starting"
            self._error_code = None
            self._hold_reason = None
            self._hardware_sent = False
            self._motion_command_count = 0
            self._motion_confirmed_count = 0
            self._thread = Thread(target=self._run, name="agent-aruco-center", daemon=False)
            thread, ready = self._thread, self._ready
            thread.start()
        if not ready.wait(max(0.1, startup_timeout_s)):
            self.stop(wait_s=5.0)
            with self._lock:
                self._error_code = "hardware_start_timeout"
            return self.snapshot()
        return self.snapshot()

    def _run(self):
        from eai_robot.applications.visual_servo.ball import (
            ArucoCenterDetector, StopRequestedError, StopState, VisionArmRuntime,
            median_detection,
        )
        from eai_robot.applications.visual_servo.joint_center import run_joint_center

        runtime = None
        try:
            args = self._args(True)
            stop = StopState()
            runtime = VisionArmRuntime(args, stop)
            runtime.on_motion_command = self._motion_command_sent
            runtime.on_motion_confirmed = self._motion_confirmed
            with self._lock:
                self._runtime, self._stop_state = runtime, stop
            runtime.connect(camera_only=False)
            with self._lock:
                self._connected = True
            detector = ArucoCenterDetector(args.marker_id, args.dictionary)
            median_detection(runtime, detector, args.feature_samples)
            if detector.last_marker_id != args.marker_id:
                raise RuntimeError("marker_identity_mismatch")
            if stop.stop_requested.is_set() or self._cancel_requested.is_set():
                raise StopRequestedError("stop requested before torque enable")
            runtime.enable_motion()
            stop.armed.set()
            with self._lock:
                self._state = "calibrating"
                self._torque_enabled = True
            run_joint_center(runtime, detector, args, stop, self._PreviewSink(self))
            with self._lock:
                self._hold_reason = getattr(runtime, "hold_reason", "tracking stopped")
                if self._motion_confirmed_count:
                    self._state = "holding"
                else:
                    self._state = "failed_holding"
                    self._error_code = "tracking_never_started"
        except StopRequestedError:
            with self._lock:
                self._state = "holding" if self._torque_enabled else "cancelled"
                self._hold_reason = "operator_stop"
        except Exception:
            with self._lock:
                self._error_code = "hardware_tracking_failed"
                self._state = "failed_holding" if self._torque_enabled else "failed"
        finally:
            with self._lock:
                if self._ready is not None:
                    self._ready.set()
                close_now = runtime is not None and not self._torque_enabled
            if close_now:
                try:
                    runtime.shutdown()
                finally:
                    with self._lock:
                        if runtime.torque_enabled:
                            self._runtime = runtime
                            self._torque_enabled = True
                            self._state = "failed_holding"
                            self._error_code = "torque_release_unconfirmed"
                        else:
                            self._runtime = None
                        self._connected = False

    def stop(self, wait_s: float = 10.0):
        with self._lock:
            stop, thread = self._stop_state, self._thread
            self._cancel_requested.set()
            if stop is not None:
                stop.stop_requested.set()
            if thread is not None and thread.is_alive():
                self._state = "stopping"
        if thread is not None and thread.is_alive() and thread is not current_thread():
            thread.join(max(0.0, wait_s))
        with self._lock:
            if thread is not None and thread.is_alive():
                self._error_code = "tracking_stop_timeout"
            elif self._torque_enabled and self._state == "stopping":
                self._state = "holding"
                self._hold_reason = "operator_stop"
            return self.snapshot()

    def release(self):
        self.stop_preview()
        self.stop(wait_s=20.0)
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("tracking worker did not stop; use physical emergency procedure")
            runtime = self._runtime
        if runtime is not None:
            runtime.shutdown()
            if runtime.torque_enabled:
                with self._lock:
                    self._state = "release_failed"
                    self._error_code = "torque_release_unconfirmed"
                    self._torque_enabled = True
                raise RuntimeError("torque release was not confirmed")
        with self._lock:
            self._runtime = None
            self._connected = False
            self._torque_enabled = False
            self._state = "released"
            return self.snapshot()


class HardwareVisualSkills:
    """Session-bound tools around an operator-armed hardware backend."""

    def __init__(self, backend, *, strict=None, clock=None, object_ttl_s=30.0):
        if strict is not None and type(strict) is not bool:
            raise ValueError("strict must be bool or omitted")
        if (type(object_ttl_s) not in (int, float) or not math.isfinite(object_ttl_s)
                or not 1 <= object_ttl_s <= 60):
            raise ValueError("object_ttl_s must be within [1, 60]")
        self.backend, self.strict = backend, strict
        self._clock = clock or time.monotonic
        self.object_ttl_s = float(object_ttl_s)
        self._lock = RLock()
        self._objects = {}
        self.tasks = {}
        self._active_task_id = None

    @staticmethod
    def _session(context):
        return context.session_id if context is not None else "direct"

    def state(self, _, context=None):
        snapshot = self.backend.snapshot()
        return ToolResult(True, "observed", {**snapshot,
                          "feedback_available": snapshot["connected"]})

    def detect(self, args, context=None):
        if context is not None:
            context.check_active()
        if normalize_target_description(args["description"]) != "aruco marker":
            return ToolResult(False, "rejected", {"found": False}, "unsupported_description")
        try:
            observed = self.backend.detect()
        except Exception:
            return ToolResult(False, "failed", {"found": False}, "detection_failed")
        captured = self._clock()
        object_id = uuid.uuid4().hex
        with self._lock:
            self._objects[object_id] = {"owner": self._session(context), "captured": captured,
                                        "observation": dict(observed)}
        return ToolResult(True, "observed", {"found": True, "object_id": object_id,
                          "expires_in_s": self.object_ttl_s, **observed})

    def start_tracking(self, args, context=None):
        if context is not None:
            context.check_active()
        with self._lock:
            record = self._objects.get(args["object_id"])
            if record is None or record["owner"] != self._session(context):
                return ToolResult(False, "rejected", error_code="unknown_object")
            age = self._clock() - record["captured"]
            if not math.isfinite(age) or age < 0 or age > self.object_ttl_s:
                self._objects.pop(args["object_id"], None)
                return ToolResult(False, "rejected", error_code="stale_object")
            if self._active_task_id is not None:
                state = self.backend.snapshot()["tracking"]
                if state not in {"released", "failed", "cancelled", "idle"}:
                    return ToolResult(False, "rejected", error_code="tracking_busy")
            task_id = uuid.uuid4().hex
            self._active_task_id = task_id
        timeout = 20.0 if context is None else max(0.1, context.deadline - time.monotonic())
        snapshot = self.backend.start(min(20.0, timeout))
        task = {"task_id": task_id, "task_type": "tracking",
                "object_id": args["object_id"], **snapshot}
        with self._lock:
            self.tasks[task_id] = task
        if snapshot.get("error_code"):
            return ToolResult(False, snapshot["tracking"], task, snapshot["error_code"])
        return ToolResult(True, snapshot["tracking"], task)

    def task(self, args, context=None):
        with self._lock:
            task = self.tasks.get(args["task_id"])
            if task is None:
                return ToolResult(False, "rejected", error_code="unknown_task")
            task.update(self.backend.snapshot())
            return ToolResult(True, task["tracking"], dict(task))

    def stop_tracking(self, _, context=None):
        snapshot = self.backend.stop()
        with self._lock:
            task_id = self._active_task_id
            if task_id in self.tasks:
                self.tasks[task_id].update(snapshot)
        data = {"stopped_task_id": task_id, **snapshot}
        if snapshot.get("error_code"):
            return ToolResult(False, snapshot["tracking"], data, snapshot["error_code"])
        return ToolResult(True, snapshot["tracking"], data)

    def stop(self, _, context=None):
        return self.stop_tracking({}, context)

    def release(self):
        return self.backend.release()

    def registry(self, allowed_tools=None):
        definitions = [
            ({"type": "function", "function": {"name": "get_robot_state",
              "description": "读取已授权 ArUco 跟随后端的连接、追踪与扭矩状态。",
              "parameters": parameters()}}, self.state, False),
            ({"type": "function", "function": {"name": "detect_object",
              "description": "使用腕部相机检测 ArUco marker；只读，不发送电机命令。",
              "parameters": parameters(description={"type": "string", "minLength": 1, "maxLength": 128})}}, self.detect, False),
            ({"type": "function", "function": {"name": "start_tracking",
              "description": "使用刚检测到的 object_id 启动已授权的 ArUco 中央区域真机跟随。",
              "parameters": parameters(object_id={"type": "string", "minLength": 1, "maxLength": 128})}}, self.start_tracking, False),
            ({"type": "function", "function": {"name": "get_task_status",
              "description": "查询真机追踪任务、连接与扭矩保持状态。",
              "parameters": parameters(task_id={"type": "string", "minLength": 1, "maxLength": 128})}}, self.task, False),
            ({"type": "function", "function": {"name": "stop_tracking",
              "description": "停止追踪指令并保持扭矩；仍需现场操作者托住机械臂后释放扭矩。",
              "parameters": parameters()}}, self.stop_tracking, False),
            ({"type": "function", "function": {"name": "stop_motion",
              "description": "立即取消本轮并请求停止真机运动；不会代替现场托住和释放扭矩。",
              "parameters": parameters()}}, self.stop, True),
        ]
        if self.strict is not None:
            for spec, _, _ in definitions:
                spec["function"]["strict"] = self.strict
        return ToolRegistry([Tool(spec, handler, interrupt) for spec, handler, interrupt in definitions
                             if allowed_tools is None or spec["function"]["name"] in allowed_tools])
