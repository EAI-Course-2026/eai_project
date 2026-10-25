"""Explicit offline fixture: a fixed stand-pose walkthrough, not model inference."""
import json

from eai_robot.policy.model_io.contracts import ChatReply, ModelEvent, ToolCall


class DemoProvider:
    def __init__(self):
        self.index = 0

    def describe(self):
        return {"adapter": "offline_demo", "execution": "in_process", "real_model": False}

    def complete(self, request, *, cancel):
        calls = [ToolCall("demo-state", "get_robot_state", {}),
                 ToolCall("demo-poses", "list_poses", {}),
                 ToolCall("demo-move", "move_to_pose", {"pose_name": "stand"})]
        if self.index < len(calls):
            result = ChatReply(request.request_id, request.session_id, request.generation,
                               tool_calls=(calls[self.index],), finish_reason="tool_calls")
        elif self.index == 3:
            data = json.loads(request.messages[-1]["content"])["data"]
            result = ChatReply(request.request_id, request.session_id, request.generation,
                               tool_calls=(ToolCall("demo-task", "get_task_status",
                                                    {"task_id": data["task_id"]}),), finish_reason="tool_calls")
        else:
            result = ChatReply(request.request_id, request.session_id, request.generation,
                               "离线固定演示完成：已生成 stand 姿态计划，没有调用模型或移动机械臂。")
        self.index += 1
        return result

    def stream(self, request, *, cancel):
        yield ModelEvent("done", reply=self.complete(request, cancel=cancel))

    def close(self):
        pass


class VisualDemoProvider(DemoProvider):
    """Deterministic visual-tool walkthrough; still not model inference."""

    def complete(self, request, *, cancel):
        if self.index == 0:
            call = ToolCall("visual-state", "get_robot_state", {})
        elif self.index == 1:
            call = ToolCall("visual-detect", "detect_object", {"description": "ArUco marker"})
        elif self.index == 2:
            data = json.loads(request.messages[-1]["content"])["data"]
            call = ToolCall("visual-start", "start_tracking", {"object_id": data["object_id"]})
        elif self.index == 3:
            data = json.loads(request.messages[-1]["content"])["data"]
            call = ToolCall("visual-status", "get_task_status", {"task_id": data["task_id"]})
        elif self.index == 4:
            call = ToolCall("visual-stop", "stop_tracking", {})
        else:
            self.index += 1
            return ChatReply(
                request.request_id, request.session_id, request.generation,
                "离线视觉追踪演练完成：检测、启动、查询和停止均为模拟，没有调用模型、相机或机械臂。",
            )
        self.index += 1
        return ChatReply(request.request_id, request.session_id, request.generation,
                         tool_calls=(call,), finish_reason="tool_calls")
