"""OpenAI message/tool types plus local execution identity and event envelopes."""
from dataclasses import dataclass, field
from copy import deepcopy
import json

from openai.types.chat import ChatCompletionMessageParam, ChatCompletionFunctionToolParam
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


# Aliases, not a second message or schema representation.
Message = ChatCompletionMessageParam
ToolDefinition = ChatCompletionFunctionToolParam


@dataclass(frozen=True)
class ChatRequest:
    request_id: str
    session_id: str
    generation: int
    messages: tuple[Message, ...]
    tools: tuple[ToolDefinition, ...]
    timeout_s: float
    provider_state: tuple[dict, ...] = ()


@dataclass(frozen=True)
class ChatReply:
    request_id: str
    session_id: str
    generation: int
    text: str = ""
    tool_calls: tuple[ToolCall, ...] = ()
    finish_reason: str = "stop"

    usage: dict = field(default_factory=dict)
    assistant_message: dict | None = None
    provider_state: dict | None = None

    def message(self) -> Message:
        if self.assistant_message is not None:
            return deepcopy(self.assistant_message)
        message = {"role": "assistant", "content": self.text or None}
        if self.tool_calls:
            message["tool_calls"] = [{"id": c.id, "type": "function", "function": {
                "name": c.name, "arguments": json.dumps(c.arguments, allow_nan=False),
            }} for c in self.tool_calls]
        return message


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
