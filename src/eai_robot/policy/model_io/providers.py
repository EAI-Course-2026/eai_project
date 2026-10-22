"""Text/tool Chat Completions adapter, bounded JSON or SSE; no agent imports."""
from dataclasses import dataclass
import json
import math
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

from .auth import EnvironmentCredential
from .contracts import ChatReply, ChatRequest, Message, ModelEvent, ProviderError, ToolCall

MAX_BODY = 1024 * 1024


def strict_json(value):
    def invalid(_):
        raise ValueError("Non-finite JSON number")
    def unique(pairs):
        result = {}
        for key, item in pairs:
            if key in result:
                raise ValueError("Duplicate JSON key")
            result[key] = item
        return result
    return json.loads(value, parse_constant=invalid, object_pairs_hook=unique)


@dataclass(frozen=True)
class ProviderConfig:
    endpoint: str
    model: str
    execution: str
    credential_env: str | None = None
    timeout_s: float = 30.0
    max_tokens: int = 512
    streaming: bool = False

    def __post_init__(self):
        if not isinstance(self.endpoint, str):
            raise ValueError("endpoint must be a URL string")
        parsed = urlsplit(self.endpoint)
        if (parsed.scheme not in {"http", "https"} or not parsed.hostname
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or (parsed.scheme == "http" and parsed.hostname not in {"localhost", "127.0.0.1", "::1"})):
            raise ValueError("Use HTTPS, or loopback HTTP without URL credentials/query/fragment")
        if self.execution not in {"local_process", "self_hosted", "managed_api"}:
            raise ValueError("Invalid execution location")
        if self.execution == "managed_api" and parsed.scheme != "https":
            raise ValueError("Managed APIs require HTTPS")
        if not isinstance(self.model, str) or not self.model.strip() or len(self.model) > 256:
            raise ValueError("A bounded model ID is required")
        EnvironmentCredential(self.credential_env)
        if (type(self.timeout_s) not in (int, float) or not math.isfinite(self.timeout_s)
                or not .1 <= self.timeout_s <= 120 or type(self.streaming) is not bool
                or type(self.max_tokens) is not int or not 1 <= self.max_tokens <= 8192):
            raise ValueError("Invalid provider budgets")


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        fp.close()
        raise ProviderError("redirect_refused")


def encode_message(message: Message) -> dict:
    if not isinstance(message, Message) or message.role not in {"system", "user", "assistant", "tool"}:
        raise ProviderError("invalid_message")
    result = {"role": message.role, "content": message.text}
    if message.tool_calls:
        result["tool_calls"] = [{"id": call.id, "type": "function", "function": {
            "name": call.name, "arguments": json.dumps(call.arguments, allow_nan=False, ensure_ascii=False),
        }} for call in message.tool_calls]
    if message.tool_call_id is not None:
        result["tool_call_id"] = message.tool_call_id
    return result


def parse_reply(request, text, calls, reason):
    if not isinstance(text, str) or len(text) > 32768 or not isinstance(calls, list) or len(calls) > 16:
        raise ValueError("Invalid response")
    parsed = []
    for call in calls:
        fn = call["function"]
        if (call.get("type") != "function" or not isinstance(call["id"], str)
                or not 1 <= len(call["id"]) <= 128 or not isinstance(fn["name"], str)
                or not 1 <= len(fn["name"]) <= 128 or not isinstance(fn["arguments"], str)
                or len(fn["arguments"]) > 4096):
            raise ValueError("Invalid tool call")
        args = strict_json(fn["arguments"])
        if not isinstance(args, dict):
            raise ValueError("Arguments must be an object")
        parsed.append(ToolCall(call["id"], fn["name"], args))
    if (reason not in {"stop", "tool_calls"} or (not parsed and not text)
            or (reason == "tool_calls" and not parsed)):
        raise ValueError("Incomplete response")
    return ChatReply(request.request_id, request.session_id, request.generation, text, tuple(parsed), reason)


class ChatCompletionsProvider:
    """Cancellation ends local waiting; remote computation may continue.

    One bounded I/O worker prevents unbounded abandoned requests. The core only
    receives parsed final tool calls; partial SSE argument JSON never executes.
    """
    def __init__(self, config: ProviderConfig):
        self.config = config
        self._credential = EnvironmentCredential(config.credential_env)
        self._credential.resolve()
        self._closed = Event()
        self._active = Lock()

    def describe(self):
        return {"adapter": "chat_completions", "model": self.config.model,
                "model_revision": "unknown", "execution": self.config.execution,
                "inputs": ["text"], "outputs": ["text", "tool_calls"],
                "streaming": self.config.streaming, "cancel": "local_wait_only"}

    def stream(self, request: ChatRequest, *, cancel: Event):
        deadline = time.monotonic() + min(request.timeout_s, self.config.timeout_s)
        if self._closed.is_set() or cancel.is_set():
            raise ProviderError("cancelled")
        if not self._active.acquire(blocking=False):
            raise ProviderError("request_still_in_flight")
        queue, abandoned = Queue(maxsize=64), Event()
        def publish(value):
            while not abandoned.is_set() and not self._closed.is_set():
                if time.monotonic() >= deadline:
                    raise ProviderError("timeout")
                try:
                    queue.put(value, timeout=.02)
                    return
                except Full:
                    pass
            raise ProviderError("cancelled")
        def exchange():
            try:
                reply = self._exchange(request, lambda event: publish((True, event)), deadline)
                final = (True, ModelEvent("done", reply=reply))
            except Exception as exc:
                final = (False, exc if isinstance(exc, ProviderError) else ProviderError("protocol_error"))
            finally:
                self._active.release()
            try:
                publish(final)
            except ProviderError:
                pass
        Thread(target=exchange, name="model-http", daemon=True).start()
        try:
            while True:
                if cancel.is_set() or self._closed.is_set():
                    raise ProviderError("cancelled")
                left = deadline - time.monotonic()
                if left <= 0:
                    raise ProviderError("timeout")
                try:
                    ok, value = queue.get(timeout=min(.02, left))
                except Empty:
                    continue
                if cancel.is_set() or self._closed.is_set():
                    raise ProviderError("cancelled")
                if not ok:
                    raise value
                yield value
                if value.type == "done":
                    return
        finally:
            abandoned.set()

    def complete(self, request, *, cancel):
        for event in self.stream(request, cancel=cancel):
            if event.type == "done":
                return event.reply
        raise ProviderError("incomplete_response")

    def _exchange(self, request, publish, deadline):
        body = json.dumps({"model": self.config.model,
                           "messages": [encode_message(m) for m in request.messages],
                           "tools": [{"type": "function", "function": {
                               "name": t.name, "description": t.description, "parameters": t.parameters,
                           }} for t in request.tools],
                           "stream": self.config.streaming,
                           "max_tokens": self.config.max_tokens}, allow_nan=False).encode()
        if len(body) > MAX_BODY:
            raise ProviderError("request_too_large")
        headers = {"Content-Type": "application/json"}
        key = self._credential.resolve()
        if key:
            headers["Authorization"] = "Bearer " + key
        wire = Request(self.config.endpoint, data=body, headers=headers, method="POST")
        try:
            with build_opener(NoRedirect()).open(wire, timeout=min(request.timeout_s, self.config.timeout_s)) as response:
                chunks = self._chunks(response, deadline)
                if self.config.streaming:
                    return self._sse(request, chunks, publish)
                raw = b"".join(chunks)
            choice = strict_json(raw)["choices"][0]
            message = choice["message"]
            if message.get("role") != "assistant" or message.get("refusal"):
                raise ValueError("Invalid assistant response")
            return parse_reply(request, message.get("content") if message.get("content") is not None else "",
                               message.get("tool_calls") if message.get("tool_calls") is not None else [],
                               choice["finish_reason"])
        except HTTPError as exc:
            status = exc.code
            exc.close()
            raise ProviderError("authentication" if status in (401, 403) else
                                "rate_limited" if status == 429 else "http_error") from None
        except TimeoutError:
            raise ProviderError("timeout") from None
        except URLError as exc:
            raise ProviderError("timeout" if isinstance(exc.reason, TimeoutError) else "transport_error") from None
        except OSError:
            raise ProviderError("transport_error") from None
        except (ValueError, TypeError, KeyError, IndexError, UnicodeError):
            raise ProviderError("protocol_error") from None

    def _chunks(self, response, deadline):
        size = 0
        while size <= MAX_BODY:
            if self._closed.is_set():
                raise ProviderError("cancelled")
            if time.monotonic() >= deadline:
                raise ProviderError("timeout")
            chunk = response.read1(min(65536, MAX_BODY + 1 - size))
            if not chunk:
                return
            size += len(chunk)
            if size > MAX_BODY:
                raise ProviderError("response_too_large")
            yield chunk

    def _sse(self, request, chunks, publish):
        buffer, lines, text, calls, reason = b"", [], "", {}, None
        for chunk in chunks:
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.rstrip(b"\r")
                if line.startswith(b"data:"):
                    lines.append(line[5:].lstrip(b" "))
                elif line == b"" and lines:
                    payload, lines = b"\n".join(lines), []
                    if payload == b"[DONE]":
                        return parse_reply(request, text, [calls[i] for i in sorted(calls)], reason)
                    data = strict_json(payload)
                    if data.get("error") or not isinstance(data.get("choices"), list):
                        raise ValueError("Invalid stream packet")
                    if not data["choices"]:  # Optional usage-only final packet.
                        continue
                    choice = data["choices"][0]
                    delta = choice.get("delta", {})
                    if not isinstance(delta, dict) or delta.get("refusal") or delta.get("role") not in (None, "assistant"):
                        raise ValueError("Invalid delta")
                    content, fragments = delta.get("content"), delta.get("tool_calls", [])
                    if reason is not None and (content or fragments):
                        raise ValueError("Content after finish")
                    if content is not None:
                        if not isinstance(content, str) or len(text) + len(content) > 32768:
                            raise ValueError("Invalid text delta")
                        text += content
                        if content:
                            publish(ModelEvent("text_delta", text=content))
                    if not isinstance(fragments, list):
                        raise ValueError("Invalid tool fragments")
                    for fragment in fragments:
                        index = fragment["index"]
                        if type(index) is not int or not 0 <= index < 16:
                            raise ValueError("Invalid tool index")
                        target = calls.setdefault(index, {"id": "", "type": "function",
                                                        "function": {"name": "", "arguments": ""}})
                        if fragment.get("type") not in (None, "function"):
                            raise ValueError("Invalid tool type")
                        for field, value in (("id", fragment.get("id")),
                                             ("name", fragment.get("function", {}).get("name")),
                                             ("arguments", fragment.get("function", {}).get("arguments"))):
                            if value is not None:
                                if not isinstance(value, str):
                                    raise ValueError("Invalid fragment")
                                dest = target if field == "id" else target["function"]
                                dest[field] += value
                                if len(dest[field]) > (4096 if field == "arguments" else 128):
                                    raise ValueError("Oversized fragment")
                    if choice.get("finish_reason") is not None:
                        if reason is not None:
                            raise ValueError("Duplicate finish")
                        reason = choice["finish_reason"]
        raise ValueError("Missing complete stream terminator")

    def close(self):
        self._closed.set()
