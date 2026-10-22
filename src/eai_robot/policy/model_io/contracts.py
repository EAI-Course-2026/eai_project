"""Provider-neutral text model contracts. Wire formats belong in adapters."""
from dataclasses import dataclass
from threading import Event
from typing import Iterator, Protocol


class ProviderError(RuntimeError):
    """Stable sanitized code, never a raw provider response or credential."""
    def __init__(self, code: str):
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass(frozen=True)
class ToolDefinition:
    name: str
    description: str
    parameters: dict


@dataclass(frozen=True)
class Message:
    role: str
    text: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    tool_call_id: str | None = None


@dataclass(frozen=True)
class ChatRequest:
    request_id: str
    session_id: str
    generation: int
    messages: tuple[Message, ...]
    tools: tuple[ToolDefinition, ...]
    timeout_s: float


@dataclass(frozen=True)
class ChatReply:
    request_id: str
    session_id: str
    generation: int
    text: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    finish_reason: str = "stop"

    def message(self) -> Message:
        return Message("assistant", self.text, self.tool_calls)


@dataclass(frozen=True)
class ModelEvent:
    type: str
    text: str = ""
    reply: ChatReply | None = None


class ChatProvider(Protocol):
    def describe(self) -> dict: ...
    def stream(self, request: ChatRequest, *, cancel: Event) -> Iterator[ModelEvent]: ...
    def complete(self, request: ChatRequest, *, cancel: Event) -> ChatReply: ...
    def close(self) -> None: ...
