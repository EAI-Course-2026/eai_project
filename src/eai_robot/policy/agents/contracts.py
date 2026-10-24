"""Agent execution contracts, independent of model transport and robot hardware."""
from dataclasses import dataclass, field
import json
from threading import Event
import time
from typing import Callable


class ToolInterrupted(RuntimeError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class ToolResult:
    ok: bool
    status: str
    data: dict = field(default_factory=dict)
    error_code: str | None = None


@dataclass(frozen=True)
class ToolContext:
    session_id: str
    generation: int
    deadline: float
    cancel: Event
    is_current: Callable[[], bool]
    on_progress: Callable[[dict], None]

    def check_active(self):
        if self.cancel.is_set() or not self.is_current():
            raise ToolInterrupted("cancelled")
        if time.monotonic() >= self.deadline:
            raise ToolInterrupted("tool_timeout")

    def progress(self, data: dict):
        self.check_active()
        raw = json.dumps(data, allow_nan=False)
        if not isinstance(data, dict) or len(raw) > 16384:
            raise ValueError("Progress must be a bounded JSON object")
        self.on_progress(json.loads(raw))


@dataclass(frozen=True)
class AgentEvent:
    type: str
    sequence: int
    session_id: str
    generation: int
    run_id: str | None
    at: float
    data: dict = field(default_factory=dict)


@dataclass(frozen=True)
class TurnResult:
    status: str
    text: str = ""
    error_code: str | None = None
