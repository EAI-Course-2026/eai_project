"""Transport-neutral harness: dialogue, events, tool lifecycle and cancellation."""
from dataclasses import asdict, replace
import json
import math
from queue import Empty, Queue
from threading import Event, Lock, RLock, Thread
import time
import uuid
from copy import deepcopy

from openai.types.chat import ChatCompletionContentPartParam
from pydantic import TypeAdapter, ValidationError

_USER_CONTENT = TypeAdapter(list[ChatCompletionContentPartParam])

from eai_robot.policy.model_io.contracts import ChatProvider, ChatRequest, Message, ProviderError
from .context import ConversationHistory
from .contracts import AgentEvent, ToolContext, ToolInterrupted, ToolResult, TurnResult
from .tools import Tool, ToolRegistry


class AgentSession:
    def __init__(self, provider: ChatProvider, tools: ToolRegistry, *, system_prompt="Use the provided tools and report their results.",
                 context_provider=None, on_cancel=None, before_tool=None, after_tool=None,
                 max_rounds=8, max_calls=16, timeout_s=60., history_turns=8, emit=None):
        if (type(max_rounds) is not int or not 1 <= max_rounds <= 16
                or type(max_calls) is not int or not 1 <= max_calls <= 64
                or type(timeout_s) not in (int, float) or not math.isfinite(timeout_s)
                or not .1 <= timeout_s <= 300 or not isinstance(system_prompt, str)
                or not 1 <= len(system_prompt) <= 8192):
            raise ValueError("Invalid agent budgets/prompt")
        self.provider, self.tools = provider, tools
        self.system_prompt, self.context_provider, self.on_cancel = system_prompt, context_provider, on_cancel
        self.before_tool, self.after_tool = before_tool, after_tool
        self.max_rounds, self.max_calls, self.timeout_s = max_rounds, max_calls, timeout_s
        self.emit = emit or (lambda event: None)
        self.session_id, self.generation = uuid.uuid4().hex, 0
        self._lock, self._events = RLock(), RLock()
        self._turn, self._tool_slot = Lock(), Lock()
        self._cancel, self._closed = Event(), False
        self._idle = Event()
        self._idle.set()
        self._history = ConversationHistory(max_turns=history_turns)
        self._running, self._pending_settlement = False, None
        self._active_tools, self._sequence, self._observer_errors = set(), 0, 0
        self._run_id = None

    def _event(self, kind, run_id, generation, **data):
        # Observers cannot alter policy or prevent cancellation by raising.
        with self._events:
            with self._lock:
                self._sequence += 1
                event = AgentEvent(kind, self._sequence, self.session_id, generation, run_id, time.monotonic(), data)
            try:
                self.emit(asdict(event))
            except Exception:
                with self._lock:
                    self._observer_errors += 1

    def snapshot(self):
        with self._lock:
            return {"session_id": self.session_id, "generation": self.generation,
                    "status": "running" if self._running else "draining" if self._active_tools or self._pending_settlement is not None else "idle",
                    "active_tools": sorted(self._active_tools),
                    "history_messages": len(self._history.messages()), "observer_errors": self._observer_errors,
                    "closed": self._closed}

    def wait_for_idle(self, timeout_s=5.) -> bool:
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 <= timeout_s <= 300:
            raise ValueError("Invalid idle wait budget")
        return self._idle.wait(timeout_s)

    def clear_history(self):
        with self._lock:
            if self._running or self._active_tools or self._pending_settlement is not None:
                raise RuntimeError("busy")
            self._history.clear()
            self.generation += 1

    def cancel(self) -> ToolResult:
        self._cancel.set()
        with self._lock:
            self._cancel.set()
            generation, run_id = self.generation, self._run_id
            self.generation += 1
        # Application cancellation is deliberately outside the state/tool lock.
        try:
            result = self.on_cancel() if self.on_cancel else ToolResult(True, "cancelled")
            if not isinstance(result, ToolResult):
                raise TypeError("Invalid cancel result")
            json.dumps(asdict(result), allow_nan=False)
        except Exception:
            result = ToolResult(False, "failed", error_code="cancel_failed")
        self._event("cancel", run_id, generation, status=result.status, ok=result.ok)
        return result

    def _current(self, generation, cancel):
        with self._lock:
            return not self._closed and generation == self.generation and not cancel.is_set()

    def _settle_if_idle(self):
        with self._events:
            with self._lock:
                if self._running or self._active_tools or self._pending_settlement is None:
                    return
                run_id, generation, status = self._pending_settlement
                self._pending_settlement = None
                if self._run_id == run_id:
                    self._run_id = None
                self._idle.set()
            self._event("run_settled", run_id, generation, status=status)

    def _invoke(self, call, context, run_id):
        if not self._tool_slot.acquire(blocking=False):
            raise ProviderError("tool_still_running")
        result, accepting_updates = Queue(maxsize=1), Event()
        accepting_updates.set()
        def progress(data):
            with self._events:
                if accepting_updates.is_set() and self._current(context.generation, context.cancel):
                    context.on_progress(data)
        execution_context = replace(context, on_progress=progress)
        with self._lock:
            self._active_tools.add(call.id)
        def execute():
            try:
                value = self.tools.invoke(call, execution_context, before=self.before_tool, after=self.after_tool)
                outcome = (True, value)
            except Exception as exc:
                outcome = (False, exc)
            finally:
                with self._events:
                    accepting_updates.clear()
                    with self._lock:
                        self._active_tools.discard(call.id)
                    self._tool_slot.release()
                    self._settle_if_idle()
            result.put(outcome)
        Thread(target=execute, name="agent-tool", daemon=True).start()
        try:
            while True:
                context.check_active()
                try:
                    ok, value = result.get(timeout=min(.02, max(.001, context.deadline - time.monotonic())))
                except Empty:
                    continue
                context.check_active()
                if not ok:
                    raise value
                return value
        except ToolInterrupted:
            raise

    def run_turn(self, text: str | list[ChatCompletionContentPartParam], *, ready: Event | None = None) -> TurnResult:
        try:
            if isinstance(text, str):
                valid = bool(text.strip()) and len(text) <= 8192
            elif isinstance(text, list):
                text = _USER_CONTENT.validate_python(text, strict=True)
                valid = bool(text) and len(json.dumps(text, allow_nan=False).encode()) <= 1024 * 1024
            else:
                valid = False
        except (ValueError, TypeError, ValidationError):
            valid = False
        if not valid:
            if ready is not None:
                ready.set()
            return TurnResult("rejected", error_code="invalid_text")
        if not self._turn.acquire(blocking=False):
            if ready is not None:
                ready.set()
            return TurnResult("rejected", error_code="busy")
        outcome, run_id, generation, started = TurnResult("failed", error_code="internal_error"), None, 0, time.monotonic()
        try:
            with self._lock:
                if self._closed:
                    return TurnResult("rejected", error_code="closed")
                if self._active_tools or self._pending_settlement is not None:
                    return TurnResult("rejected", error_code="tool_still_running")
                cancel = self._cancel = Event()
                self.generation += 1
                generation = self.generation
                run_id = self._run_id = uuid.uuid4().hex
                self._running = True
                self._idle.clear()
                history = self._history.messages()
            if ready is not None:
                ready.set()
            self._event("run_start", run_id, generation)
            messages = [*history, {"role": "user", "content": deepcopy(text)}]
            provider_state = []
            deadline, seen, count = started + self.timeout_s, set(), 0
            for _ in range(self.max_rounds):
                if not self._current(generation, cancel):
                    raise ToolInterrupted("cancelled")
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ProviderError("turn_timeout")
                leading = [{"role": "system", "content": self.system_prompt}]
                if self.context_provider:
                    snapshot = self.context_provider()
                    try:
                        snapshot.validate()
                    except (ValueError, AttributeError):
                        raise ProviderError("stale_context") from None
                    leading.append({"role": "system", "content": snapshot.text})
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ProviderError("turn_timeout")
                if not self._current(generation, cancel):
                    raise ToolInterrupted("cancelled")
                request = ChatRequest(uuid.uuid4().hex, self.session_id, generation,
                                      tuple(leading + messages), self.tools.definitions, left, tuple(provider_state))
                self._event("model_start", run_id, generation, request_id=request.request_id)
                reply = None
                stream = self.provider.stream(request, cancel=cancel)
                try:
                    for event in stream:
                        if not self._current(generation, cancel):
                            raise ToolInterrupted("cancelled")
                        if time.monotonic() >= deadline:
                            raise ProviderError("turn_timeout")
                        if event.type == "text_delta":
                            self._event("model_delta", run_id, generation, text=event.text)
                        elif event.type == "done" and event.reply is not None:
                            reply = event.reply
                            break
                        else:
                            raise ProviderError("invalid_model_event")
                finally:
                    if hasattr(stream, "close"):
                        stream.close()
                if reply is None:
                    raise ProviderError("incomplete_response")
                if (reply.request_id, reply.session_id, reply.generation) != (request.request_id, self.session_id, generation):
                    raise ProviderError("identity_mismatch")
                if reply.finish_reason not in {"stop", "tool_calls"}:
                    raise ProviderError("incomplete_response")
                self._event("model_end", run_id, generation, request_id=request.request_id, usage=reply.usage)
                ids = [call.id for call in reply.tool_calls]
                if len(set(ids)) != len(ids) or seen.intersection(ids):
                    raise ProviderError("duplicate_tool_call")
                if count + len(ids) > self.max_calls:
                    raise ProviderError("call_budget")
                seen.update(ids)
                count += len(ids)
                if reply.provider_state is not None:
                    provider_state.append(deepcopy(reply.provider_state))
                messages.append(reply.message())
                # Interrupt semantics are tool metadata, not a domain name in the core.
                for call in reply.tool_calls:
                    prepared = self.tools.prepare(call)
                    if isinstance(prepared, Tool) and prepared.interrupt:
                        self._event("tool_start", run_id, generation, tool=call.name, tool_call_id=call.id)
                        result = self.cancel()
                        self._event("tool_end", run_id, generation, tool=call.name, tool_call_id=call.id, status=result.status, ok=result.ok)
                        outcome = TurnResult("cancelled") if result.ok else TurnResult("failed", error_code="cancel_failed")
                        return outcome
                for call in reply.tool_calls:
                    context = ToolContext(self.session_id, generation, deadline, cancel,
                                          lambda: self._current(generation, cancel),
                                          lambda data, c=call: self._event("tool_progress", run_id, generation,
                                                                          tool=c.name, tool_call_id=c.id, progress=data))
                    self._event("tool_start", run_id, generation, tool=call.name, tool_call_id=call.id)
                    try:
                        result = self._invoke(call, context, run_id)
                    except Exception:
                        self._event("tool_end", run_id, generation, tool=call.name, tool_call_id=call.id,
                                    status="cancelled" if cancel.is_set() else "failed", ok=False)
                        raise
                    self._event("tool_end", run_id, generation, tool=call.name, tool_call_id=call.id, status=result.status, ok=result.ok)
                    content = {**asdict(result), "session_id": self.session_id,
                               "generation": generation, "tool_call_id": call.id}
                    messages.append({"role": "tool", "content": json.dumps(content, ensure_ascii=False, allow_nan=False), "tool_call_id": call.id})
                if not reply.tool_calls:
                    with self._lock:
                        if not self._current(generation, cancel):
                            raise ToolInterrupted("cancelled")
                        self._history.commit(text, reply.text, assistant_message=reply.message())
                    outcome = TurnResult("completed", reply.text)
                    return outcome
            raise ProviderError("round_budget")
        except (ProviderError, ToolInterrupted) as exc:
            if self._cancel.is_set() or getattr(exc, "code", None) == "cancelled":
                if not self._cancel.is_set():
                    self.cancel()
                outcome = TurnResult("cancelled")
            else:
                self.cancel()
                outcome = TurnResult("failed", error_code=exc.code)
            return outcome
        except Exception:
            self.cancel()
            outcome = TurnResult("failed", error_code="internal_error")
            return outcome
        finally:
            if ready is not None:
                ready.set()
            if run_id is not None:
                self._event("run_end", run_id, generation, status=outcome.status,
                            error_code=outcome.error_code, duration_s=round(time.monotonic() - started, 6))
                with self._lock:
                    self._running = False
                    self._pending_settlement = (run_id, generation, outcome.status)
                self._settle_if_idle()
            self._turn.release()

    def close(self):
        with self._lock:
            self._closed = True
        self.cancel()
        self.provider.close()
