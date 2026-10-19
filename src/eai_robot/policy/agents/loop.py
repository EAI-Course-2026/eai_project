"""Bounded tool loop, injected handlers and generation-based cancellation."""
from dataclasses import asdict, dataclass
from importlib.resources import files
import json
import math
from threading import Event, Lock, RLock
import time
import uuid

from .contracts import ChatRequest, ProviderError, ToolCall, ToolResult
from .tools import ToolRegistry


@dataclass(frozen=True)
class TurnResult:
    status: str
    text: str = ""
    error_code: str | None = None


class AgentSession:
    def __init__(self, provider, tools: ToolRegistry, *, max_rounds=8,
                 max_calls=16, timeout_s=60., emit=None):
        if (type(max_rounds) is not int or not 1 <= max_rounds <= 16
                or type(max_calls) is not int or not 1 <= max_calls <= 64
                or type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
                or not .1 <= timeout_s <= 300):
            raise ValueError("Invalid agent budgets")
        self.provider, self.tools = provider, tools
        self.max_rounds, self.max_calls, self.timeout_s = max_rounds, max_calls, timeout_s
        self.emit = emit or (lambda event: None)
        self.session_id = uuid.uuid4().hex
        self.generation = 0
        self._lock = RLock()
        self._turn = Lock()
        self._cancel = Event()
        self._closed = False

    def _event(self, kind, **data):
        # Metadata only: no user transcript, provider body, endpoint or credentials.
        self.emit({"type": kind, "session_id": self.session_id,
                   "generation": self.generation, **data})

    def stop(self) -> ToolResult:
        self._cancel.set()
        with self._lock:
            self._cancel.set()
            self.generation += 1
            try:
                result = self.tools.invoke(ToolCall(uuid.uuid4().hex, "stop_motion", {}))
            except Exception:
                result = ToolResult(False, "failed", error_code="stop_failed")
            self._event("stop", status=result.status)
            return result

    def run_turn(self, text: str, *, ready: Event | None = None) -> TurnResult:
        if not isinstance(text, str) or not text.strip() or len(text) > 8192:
            if ready is not None:
                ready.set()
            return TurnResult("rejected", error_code="invalid_text")
        if not self._turn.acquire(blocking=False):
            if ready is not None:
                ready.set()
            return TurnResult("rejected", error_code="busy")
        try:
            with self._lock:
                if self._closed:
                    return TurnResult("rejected", error_code="closed")
                cancel = self._cancel = Event()
                # A fresh explicit input also invalidates identities from older turns.
                self.generation += 1
                generation = self.generation
                if ready is not None:
                    ready.set()
            messages = [{"role": "system", "content": files(__package__).joinpath("prompts/system.txt").read_text(encoding="utf-8")},
                        {"role": "user", "content": text}]
            deadline = time.monotonic() + self.timeout_s
            seen, count = set(), 0
            for _ in range(self.max_rounds):
                if cancel.is_set():
                    return TurnResult("cancelled")
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ProviderError("turn_timeout")
                request = ChatRequest(uuid.uuid4().hex, self.session_id, generation,
                                      tuple(messages), self.tools.schemas, left)
                self._event("request", request_id=request.request_id)
                reply = self.provider.complete(request, cancel=cancel)
                with self._lock:
                    if cancel.is_set() or generation != self.generation or self._closed:
                        return TurnResult("cancelled")
                    if time.monotonic() >= deadline:
                        raise ProviderError("turn_timeout")
                    if (reply.request_id, reply.session_id, reply.generation) != (request.request_id, self.session_id, generation):
                        raise ProviderError("identity_mismatch")
                    if reply.finish_reason not in {"stop", "tool_calls"}:
                        raise ProviderError("incomplete_response")
                    ids = [call.id for call in reply.tool_calls]
                    if len(set(ids)) != len(ids) or seen.intersection(ids):
                        raise ProviderError("duplicate_tool_call")
                    if count + len(ids) > self.max_calls:
                        raise ProviderError("call_budget")
                    seen.update(ids)
                    count += len(ids)
                    messages.append(reply.message())
                    # A valid stop anywhere in a batch preempts even earlier moves.
                    if any(call.name == "stop_motion" and call.arguments == {} for call in reply.tool_calls):
                        result = self.stop()
                        return (TurnResult("cancelled", "已停止演练任务。") if result.ok
                                else TurnResult("failed", error_code="stop_failed"))
                    for call in reply.tool_calls:
                        if cancel.is_set():
                            return TurnResult("cancelled")
                        result = self.tools.invoke(call)
                        self._event("tool_result", tool_call_id=call.id, tool=call.name,
                                    status=result.status, ok=result.ok)
                        content = {**asdict(result), "session_id": self.session_id,
                                   "generation": generation, "tool_call_id": call.id}
                        messages.append({"role": "tool", "tool_call_id": call.id,
                                         "content": json.dumps(content, ensure_ascii=False, allow_nan=False)})
                    if not reply.tool_calls:
                        self._event("turn_complete")
                        return TurnResult("completed", reply.text)
            raise ProviderError("round_budget")
        except ProviderError as exc:
            if self._cancel.is_set():
                return TurnResult("cancelled")
            self.stop()
            self._event("turn_failed", error_code=exc.code)
            return TurnResult("failed", error_code=exc.code)
        except Exception:
            self.stop()
            self._event("turn_failed", error_code="internal_error")
            return TurnResult("failed", error_code="internal_error")
        finally:
            if ready is not None:
                ready.set()
            self._turn.release()

    def close(self):
        with self._lock:
            self._closed = True
        self.stop()
        self.provider.close()
