"""Application-owned robot capabilities. Unknown physical limits stay unknown."""
from dataclasses import asdict, dataclass
import json
import time

from eai_robot.policy.agents.context import ContextSnapshot
from eai_robot.policy.agents.contracts import ToolResult

TOOLS = ("get_robot_state", "list_poses", "move_to_pose", "get_task_status", "stop_motion")


@dataclass(frozen=True)
class RobotCapabilities:
    allowed_tools: tuple[str, ...] = TOOLS
    mode: str = "dry_run"
    physical_execution: bool = False
    evidence: str = "Validated reference poses; no physical task acceptance"

    def __post_init__(self):
        if (self.mode != "dry_run" or self.physical_execution is not False
                or not set(self.allowed_tools) <= set(TOOLS)
                or "stop_motion" not in self.allowed_tools):
            raise ValueError("Only supported dry-run capabilities are implemented")

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
