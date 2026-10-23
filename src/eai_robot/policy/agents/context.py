"""Bounded dialogue memory and short-lived, application-supplied context."""
from dataclasses import dataclass
from copy import deepcopy
import json
import math
import time

from eai_robot.policy.model_io.contracts import Message


@dataclass(frozen=True)
class ContextSnapshot:
    text: str
    captured_at: float
    max_age_s: float

    def validate(self):
        now = time.monotonic()
        if (not isinstance(self.text, str) or len(self.text) > 32768
                or type(self.captured_at) not in (int, float) or not math.isfinite(self.captured_at)
                or type(self.max_age_s) not in (int, float) or not math.isfinite(self.max_age_s)
                or not 0 < self.max_age_s <= 300 or self.captured_at > now
                or now - self.captured_at > self.max_age_s):
            raise ValueError("Invalid or stale application context")


class ConversationHistory:
    """Completed dialogue and final assistant metadata; never tool queues."""
    def __init__(self, max_turns=8, max_chars=32768):
        if type(max_turns) is not int or not 0 <= max_turns <= 32 or type(max_chars) is not int or not 1024 <= max_chars <= 262144:
            raise ValueError("Invalid history budget")
        self.max_turns, self.max_chars = max_turns, max_chars
        self._turns = []

    def messages(self) -> tuple[Message, ...]:
        return tuple(deepcopy(message) for pair in self._turns for message in pair)

    def commit(self, user_text, assistant_text, *, assistant_message=None):
        assistant = deepcopy(assistant_message) if assistant_message is not None else {"role": "assistant", "content": assistant_text}
        if assistant.get("role") != "assistant" or assistant.get("tool_calls") or assistant.get("function_call"):
            raise ValueError("Only a final assistant message can enter dialogue history")
        self._turns.append(({"role": "user", "content": deepcopy(user_text)}, assistant))
        while self._turns and (len(self._turns) > self.max_turns or
                              sum(len(json.dumps(m, ensure_ascii=False)) for pair in self._turns for m in pair) > self.max_chars):
            self._turns.pop(0)

    def clear(self):
        self._turns.clear()
