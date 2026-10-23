"""Hardware-free planning handlers. All robot I/O remains outside this module."""
import math
import uuid

from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.tools import Tool, ToolRegistry


def parameters(**properties):
    return {"type": "object", "properties": properties,
            "required": list(properties), "additionalProperties": False}


class DryRunSkills:
    def __init__(self, poses: dict[str, list[float]], *, strict: bool | None = None):
        if (not poses or any(not isinstance(name, str) or not 1 <= len(name) <= 128
                             for name in poses)
                or any(len(values) != 6 or any(type(v) not in (int, float)
                    or not math.isfinite(v) or not 0 <= v <= 1 for v in values)
                       for values in poses.values())):
            raise ValueError("Expected named six-joint 0..1 poses")
        if strict is not None and type(strict) is not bool:
            raise ValueError("strict must be bool or omitted")
        self.strict = strict
        self.poses = {name: list(values) for name, values in poses.items()}
        self.tasks = {}

    def state(self, _, context=None):
        return ToolResult(True, "simulated", {"mode": "dry_run", "connected": False,
                          "feedback_available": False, "joint_positions": None,
                          "tracking": "unavailable", "hardware_error": "not_observed"})

    def list_poses(self, _, context=None):
        return ToolResult(True, "simulated", {"poses": sorted(self.poses)})

    def move(self, args, context=None):
        if context is not None:
            context.check_active()
        task_id = uuid.uuid4().hex
        plan = {"task_id": task_id, "pose_name": args["pose_name"],
                "status": "simulated", "targets": self.poses[args["pose_name"]].copy(),
                "units": "normalized_0_1", "hardware_sent": False}
        self.tasks[task_id] = plan
        # Bound long-running stdin sessions; expired IDs are explicitly unknown.
        if len(self.tasks) > 128:
            del self.tasks[next(iter(self.tasks))]
        return ToolResult(True, "simulated", plan.copy())

    def task(self, args, context=None):
        plan = self.tasks.get(args["task_id"])
        if plan is None:
            return ToolResult(False, "rejected", error_code="unknown_task")
        return ToolResult(True, "simulated", plan.copy())

    def stop(self, _, context=None):
        return ToolResult(True, "simulated", {"mode": "dry_run", "hardware_sent": False,
                                               "active_motion": False})

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
                "name": "get_task_status", "description": "查询本进程的演练任务。",
                "parameters": parameters(task_id={"type": "string", "minLength": 1, "maxLength": 128})}}, self.task, False),
            ({"type": "function", "function": {
                "name": "stop_motion", "description": "取消本轮演练并废弃旧响应；不卸扭矩。",
                "parameters": parameters()}}, self.stop, True),
        ]
        if self.strict is not None:
            for spec, _, _ in definitions:
                spec["function"]["strict"] = self.strict
        return ToolRegistry([Tool(spec, handler, interrupt) for spec, handler, interrupt in definitions
                             if allowed_tools is None or spec["function"]["name"] in allowed_tools])
