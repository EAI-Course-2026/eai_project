"""Application-owned robot capabilities. Unknown physical limits stay unknown."""
from dataclasses import asdict, dataclass
import json
import time

from eai_robot.policy.agents.context import ContextSnapshot
from eai_robot.policy.agents.contracts import ToolResult

TOOLS = ("get_robot_state", "list_poses", "move_to_pose", "detect_object",
         "start_tracking", "get_task_status", "stop_tracking", "stop_motion")
HARDWARE_VISUAL_TOOLS = ("get_robot_state", "detect_object", "start_tracking",
                         "get_task_status", "stop_tracking", "stop_motion")


@dataclass(frozen=True)
class RobotCapabilities:
    allowed_tools: tuple[str, ...] = TOOLS
    mode: str = "dry_run"
    physical_execution: bool = False
    evidence: str = "Validated reference poses and visual fixture; no physical task acceptance"

    def __post_init__(self):
        dry_run = (self.mode == "dry_run" and self.physical_execution is False
                   and set(self.allowed_tools) <= set(TOOLS))
        visual_hardware = (self.mode == "aruco_center_hardware"
                           and self.physical_execution is True
                           and set(self.allowed_tools) <= set(HARDWARE_VISUAL_TOOLS))
        if (not (dry_run or visual_hardware) or "stop_motion" not in self.allowed_tools
                or (visual_hardware and "stop_tracking" not in self.allowed_tools)):
            raise ValueError("Unsupported robot capability set")

    def describe(self):
        return {**asdict(self), "physical_limits": {
            "payload_kg": None, "workspace_mm": None, "position_error_mm": None,
        }, "limits_evidence": "unmeasured in this runtime; not inferred from model text"}

    def context(self):
        return ContextSnapshot("Current declared capabilities (not joint feedback):\n" +
                               json.dumps(self.describe(), ensure_ascii=False), time.monotonic(), 5.)

    def before_tool(self, tool, arguments, context):
        context.check_active()
        if tool.name not in self.allowed_tools:
            return ToolResult(False, "rejected", error_code="capability_unavailable")
        return None
