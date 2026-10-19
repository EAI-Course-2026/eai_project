"""Small, text-only first-version chat and tool contracts."""
from dataclasses import dataclass, field
from threading import Event
from typing import Protocol


class ProviderError(RuntimeError):
    """Sanitized error: never include a provider response body or credentials."""

    def __init__(self, code: str):
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass(frozen=True)
class ChatRequest:
    request_id: str
    session_id: str
    generation: int
    messages: tuple[dict, ...]
    tools: tuple[dict, ...]
    timeout_s: float


@dataclass(frozen=True)
class ChatReply:
    request_id: str
    session_id: str
    generation: int
    text: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    finish_reason: str = "stop"

    def message(self) -> dict:
        import json
        result = {"role": "assistant", "content": self.text or None}
        if self.tool_calls:
            result["tool_calls"] = [
                {"id": call.id, "type": "function", "function": {
                    "name": call.name,
                    "arguments": json.dumps(call.arguments, ensure_ascii=False, allow_nan=False),
                }} for call in self.tool_calls
            ]
        return result


@dataclass(frozen=True)
class ToolResult:
    ok: bool
    status: str
    data: dict = field(default_factory=dict)
    error_code: str | None = None


class ChatProvider(Protocol):
    def describe(self) -> dict: ...
    def complete(self, request: ChatRequest, *, cancel: Event) -> ChatReply: ...
    def close(self) -> None: ...
