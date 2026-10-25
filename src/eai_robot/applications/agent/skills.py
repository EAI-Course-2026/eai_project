"""Hardware-free planning and visual-tracking rehearsal handlers.

The visual tools intentionally model a known fixture instead of opening a
camera.  They exercise the same detect -> opaque object id -> long-running task
contract that a future hardware adapter must implement, while keeping all robot
and camera I/O outside this module.
"""
import math
import re
from threading import RLock
import time
import uuid

from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.tools import Tool, ToolRegistry


def parameters(**properties):
    return {"type": "object", "properties": properties,
            "required": list(properties), "additionalProperties": False}


_TARGET_ALIASES = {
    "aruco": "aruco marker",
    "aruco marker": "aruco marker",
    "aruco tag": "aruco marker",
    "aruco 标记": "aruco marker",
    "aruco 标签": "aruco marker",
}


def normalize_target_description(value: str) -> str | None:
    """Map ordinary ArUco descriptions to the one hardware-authorized target.

    Model wording is not required to equal a tiny alias table.  A conflicting
    marker number or an explicit negation remains rejected: hardware mode is
    still fixed to marker 0 and cannot broaden the operator's authorization.
    """
    description = " ".join(value.strip().casefold().split())
    target = _TARGET_ALIASES.get(description)
    if target is not None:
        return target
    if "aruco" not in description:
        return None
    if any(token in description for token in ("not aruco", "non-aruco", "非 aruco", "不是 aruco")):
        return None
    if any(number != "0" for number in re.findall(r"\d+", description)):
        return None
    return "aruco marker"


class DryRunSkills:
    def __init__(self, poses: dict[str, list[float]], *, strict: bool | None = None,
                 clock=None, object_ttl_s: float = 5.0):
        if (not poses or any(not isinstance(name, str) or not 1 <= len(name) <= 128
                             for name in poses)
                or any(len(values) != 6 or any(type(v) not in (int, float)
                    or not math.isfinite(v) or not 0 <= v <= 1 for v in values)
                       for values in poses.values())):
            raise ValueError("Expected named six-joint 0..1 poses")
        if strict is not None and type(strict) is not bool:
            raise ValueError("strict must be bool or omitted")
        if (type(object_ttl_s) not in (int, float) or not math.isfinite(object_ttl_s)
                or not .1 <= object_ttl_s <= 60):
            raise ValueError("object_ttl_s must be within [0.1, 60]")
        self.strict = strict
        self.poses = {name: list(values) for name, values in poses.items()}
        self._clock = clock or time.monotonic
        if not callable(self._clock):
            raise ValueError("clock must be callable")
        self.object_ttl_s = float(object_ttl_s)
        self._lock = RLock()
        self._frame_id = 0
        self._objects = {}
        self._active_tracking_task_id = None
        self.tasks = {}

    @staticmethod
    def _session_id(context):
        return context.session_id if context is not None else "direct"

    def _prune(self):
        while len(self._objects) > 128:
            del self._objects[next(iter(self._objects))]
        while len(self.tasks) > 128:
            task_id = next((candidate for candidate in self.tasks
                            if candidate != self._active_tracking_task_id), next(iter(self.tasks)))
            del self.tasks[task_id]
            if self._active_tracking_task_id == task_id:
                self._active_tracking_task_id = None

    def _active_tracking(self):
        task = self.tasks.get(self._active_tracking_task_id)
        return task if task is not None and task.get("task_type") == "tracking" \
            and task.get("state") == "running" else None

    def _stop_tracking(self, reason):
        task = self._active_tracking()
        if task is None:
            self._active_tracking_task_id = None
            return None
        task["state"] = "cancelled"
        task["reason"] = reason
        task["active"] = False
        self._active_tracking_task_id = None
        return task["task_id"]

    def state(self, _, context=None):
        with self._lock:
            active = self._active_tracking()
            return ToolResult(True, "simulated", {
                "mode": "dry_run", "connected": False,
                "feedback_available": False, "joint_positions": None,
                "tracking": "running" if active else "idle",
                "active_task_id": active["task_id"] if active else None,
                "hardware_error": "not_observed", "hardware_sent": False,
            })

    def list_poses(self, _, context=None):
        return ToolResult(True, "simulated", {"poses": sorted(self.poses)})

    def move(self, args, context=None):
        if context is not None:
            context.check_active()
        with self._lock:
            task_id = uuid.uuid4().hex
            plan = {"task_id": task_id, "task_type": "pose", "pose_name": args["pose_name"],
                    "status": "simulated", "targets": self.poses[args["pose_name"]].copy(),
                    "units": "normalized_0_1", "hardware_sent": False}
            self.tasks[task_id] = plan
            self._prune()
            return ToolResult(True, "simulated", plan.copy())

    def detect(self, args, context=None):
        if context is not None:
            context.check_active()
        target = normalize_target_description(args["description"])
        if target is None:
            return ToolResult(False, "rejected", {
                "found": False, "description": args["description"],
                "supported_descriptions": ["ArUco marker"], "hardware_sent": False,
            }, "unsupported_description")
        with self._lock:
            self._frame_id += 1
            object_id = uuid.uuid4().hex
            captured_at = self._clock()
            if type(captured_at) not in (int, float) or not math.isfinite(captured_at):
                raise ValueError("clock must return a finite number")
            record = {
                "object_id": object_id, "target_type": target,
                "label": "aruco:0", "detector": "dry_run_aruco_fixture",
                "frame_id": self._frame_id, "bbox_px": [240, 160, 160, 160],
                "captured_at": captured_at, "confidence": 1.0,
                "owner_session_id": self._session_id(context),
            }
            self._objects[object_id] = record
            self._prune()
            return ToolResult(True, "simulated", {
                "found": True, "object_id": object_id, "target_type": target,
                "label": record["label"], "detector": record["detector"],
                "frame_id": record["frame_id"], "bbox_px": record["bbox_px"].copy(),
                "confidence": record["confidence"], "expires_in_s": self.object_ttl_s,
                "mode": "dry_run", "hardware_sent": False,
            })

    def start_tracking(self, args, context=None):
        if context is not None:
            context.check_active()
        with self._lock:
            record = self._objects.get(args["object_id"])
            if record is None or record["owner_session_id"] != self._session_id(context):
                return ToolResult(False, "rejected", error_code="unknown_object")
            age = self._clock() - record["captured_at"]
            if not math.isfinite(age) or age < 0 or age > self.object_ttl_s:
                del self._objects[args["object_id"]]
                return ToolResult(False, "rejected", error_code="stale_object")
            active = self._active_tracking()
            if active is not None:
                return ToolResult(False, "rejected", {
                    "active_task_id": active["task_id"], "hardware_sent": False,
                }, "tracking_busy")
            task_id = uuid.uuid4().hex
            task = {
                "task_id": task_id, "task_type": "tracking", "state": "running",
                "active": True, "object_id": record["object_id"],
                "target_type": record["target_type"], "source_frame_id": record["frame_id"],
                "mode": "dry_run", "hardware_sent": False,
            }
            self.tasks[task_id] = task
            self._active_tracking_task_id = task_id
            self._prune()
            return ToolResult(True, "simulated", task.copy())

    def task(self, args, context=None):
        with self._lock:
            plan = self.tasks.get(args["task_id"])
            if plan is None:
                return ToolResult(False, "rejected", error_code="unknown_task")
            return ToolResult(True, "simulated", plan.copy())

    def stop_tracking(self, _, context=None):
        if context is not None:
            context.check_active()
        with self._lock:
            task_id = self._stop_tracking("stop_tracking")
            return ToolResult(True, "simulated", {
                "mode": "dry_run", "stopped_task_id": task_id,
                "active_tracking": False, "hardware_sent": False,
            })

    def stop(self, _, context=None):
        with self._lock:
            task_id = self._stop_tracking("stop_motion")
            return ToolResult(True, "simulated", {
                "mode": "dry_run", "hardware_sent": False,
                "active_motion": False, "stopped_tracking_task_id": task_id,
            })

    def registry(self, allowed_tools=None):
        # These dictionaries are the actual tools payload accepted by the SDK.
        definitions = [
            ({"type": "function", "function": {
                "name": "get_robot_state", "description": "读取演练状态；无真实反馈。",
                "parameters": parameters()}}, self.state, False),
            ({"type": "function", "function": {
                "name": "list_poses", "description": "列出保存的演练姿态。",
                "parameters": parameters()}}, self.list_poses, False),
            ({"type": "function", "function": {
                "name": "move_to_pose", "description": "生成命名姿态演练计划，不驱动硬件。",
                "parameters": parameters(pose_name={"type": "string", "enum": sorted(self.poses)})}}, self.move, False),
            ({"type": "function", "function": {
                "name": "detect_object",
                "description": "在固定演练场景中查找白名单目标；当前仅支持 ArUco marker，不打开相机。",
                "parameters": parameters(description={"type": "string", "minLength": 1, "maxLength": 128})}}, self.detect, False),
            ({"type": "function", "function": {
                "name": "start_tracking",
                "description": "使用本会话中未过期的 object_id 启动视觉追踪演练；不驱动硬件。",
                "parameters": parameters(object_id={"type": "string", "minLength": 1, "maxLength": 128})}}, self.start_tracking, False),
            ({"type": "function", "function": {
                "name": "get_task_status", "description": "查询本进程的演练任务。",
                "parameters": parameters(task_id={"type": "string", "minLength": 1, "maxLength": 128})}}, self.task, False),
            ({"type": "function", "function": {
                "name": "stop_tracking", "description": "停止当前视觉追踪演练；没有活动任务时也可重复调用。",
                "parameters": parameters()}}, self.stop_tracking, False),
            ({"type": "function", "function": {
                "name": "stop_motion", "description": "取消本轮演练并废弃旧响应；不卸扭矩。",
                "parameters": parameters()}}, self.stop, True),
        ]
        if self.strict is not None:
            for spec, _, _ in definitions:
                spec["function"]["strict"] = self.strict
        return ToolRegistry([Tool(spec, handler, interrupt) for spec, handler, interrupt in definitions
                             if allowed_tools is None or spec["function"]["name"] in allowed_tools])
