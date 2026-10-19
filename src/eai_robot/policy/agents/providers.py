"""Bounded non-streaming Chat Completions HTTP adapter, without vendor SDKs."""
from dataclasses import dataclass
import json
import math
import os
from queue import Empty, Queue
import re
from threading import Event, Lock, Thread
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener

from .contracts import ChatReply, ChatRequest, ProviderError, ToolCall

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

    def __post_init__(self):
        if not isinstance(self.endpoint, str):
            raise ValueError("endpoint must be a URL string")
        parsed = urlsplit(self.endpoint)
        if (parsed.scheme not in {"http", "https"} or not parsed.hostname
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or (parsed.scheme == "http" and parsed.hostname not in {"localhost", "127.0.0.1", "::1"})):
            raise ValueError("Use HTTPS, or loopback HTTP; no URL credentials/query/fragment")
        if self.execution not in {"local_process", "self_hosted", "managed_api"}:
            raise ValueError("Invalid execution location")
        if self.execution == "managed_api" and parsed.scheme != "https":
            raise ValueError("Managed APIs require HTTPS")
        if not isinstance(self.model, str) or not self.model.strip() or len(self.model) > 256:
            raise ValueError("A bounded model ID is required")
        if self.credential_env is not None and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", self.credential_env):
            raise ValueError("credential_env must be an environment variable name")
        if (type(self.timeout_s) not in (int, float) or not math.isfinite(self.timeout_s)
                or not .1 <= self.timeout_s <= 120
                or type(self.max_tokens) is not int or not 1 <= self.max_tokens <= 8192):
            raise ValueError("Invalid provider budgets")


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        fp.close()
        raise ProviderError("redirect_refused")


class ChatCompletionsProvider:
    """Cancellation ends the local wait; it cannot undo remote computation/billing.

    A daemon I/O worker is bounded by socket timeout and response size. At most one
    exchange remains in flight; a cancelled connection cannot accumulate workers.
    """
    def __init__(self, config: ProviderConfig):
        self.config = config
        self._closed = Event()
        self._active = Lock()
        self._key = os.environ.get(config.credential_env, "") if config.credential_env else ""
        if config.credential_env and not self._key.strip():
            raise ProviderError("missing_credential")
        if "\n" in self._key or "\r" in self._key:
            raise ProviderError("invalid_credential")

    def describe(self) -> dict:
        # Do not expose the endpoint (which may contain private identifiers) or key.
        return {"adapter": "chat_completions", "model": self.config.model,
                "model_revision": "unknown", "execution": self.config.execution,
                "inputs": ["text"], "outputs": ["text", "tool_calls"],
                "streaming": False, "cancel": "local_wait_only"}

    def complete(self, request: ChatRequest, *, cancel: Event) -> ChatReply:
        deadline = time.monotonic() + min(request.timeout_s, self.config.timeout_s)
        if self._closed.is_set() or cancel.is_set():
            raise ProviderError("cancelled")
        if not self._active.acquire(blocking=False):
            raise ProviderError("request_still_in_flight")
        result = Queue(maxsize=1)
        def exchange():
            try:
                result.put((True, self._exchange(request)))
            except Exception as exc:
                result.put((False, exc if isinstance(exc, ProviderError) else ProviderError("transport_error")))
            finally:
                self._active.release()
        Thread(target=exchange, name="agent-http", daemon=True).start()
        while True:
            if cancel.is_set() or self._closed.is_set():
                raise ProviderError("cancelled")
            left = deadline - time.monotonic()
            if left <= 0:
                raise ProviderError("timeout")
            try:
                ok, value = result.get(timeout=min(.02, left))
            except Empty:
                continue
            if cancel.is_set() or self._closed.is_set():
                raise ProviderError("cancelled")
            if not ok:
                raise value
            return value

    def _exchange(self, request: ChatRequest) -> ChatReply:
        body = json.dumps({"model": self.config.model, "messages": request.messages,
                           "tools": request.tools, "stream": False,
                           "max_tokens": self.config.max_tokens}, allow_nan=False).encode()
        if len(body) > MAX_BODY:
            raise ProviderError("request_too_large")
        headers = {"Content-Type": "application/json"}
        if self._key:
            headers["Authorization"] = "Bearer " + self._key
        wire = Request(self.config.endpoint, data=body, headers=headers, method="POST")
        deadline = time.monotonic() + min(request.timeout_s, self.config.timeout_s)
        try:
            with build_opener(NoRedirect()).open(wire, timeout=min(request.timeout_s, self.config.timeout_s)) as response:
                chunks, size = [], 0
                while size <= MAX_BODY:
                    if time.monotonic() >= deadline or self._closed.is_set():
                        raise ProviderError("timeout" if not self._closed.is_set() else "cancelled")
                    chunk = response.read1(min(65536, MAX_BODY + 1 - size))
                    if not chunk:
                        break
                    chunks.append(chunk)
                    size += len(chunk)
                raw = b"".join(chunks)
        except HTTPError as exc:
            status = exc.code
            exc.close()
            code = ("authentication" if status in (401, 403) else
                    "rate_limited" if status == 429 else "http_error")
            raise ProviderError(code) from None
        except TimeoutError:
            raise ProviderError("timeout") from None
        except URLError as exc:
            raise ProviderError("timeout" if isinstance(exc.reason, TimeoutError) else "transport_error") from None
        except OSError:
            raise ProviderError("transport_error") from None
        if len(raw) > MAX_BODY:
            raise ProviderError("response_too_large")
        try:
            data = strict_json(raw)
            choice = data["choices"][0]
            message = choice["message"]
            if message.get("role") != "assistant" or message.get("refusal"):
                raise ValueError("Invalid assistant response")
            text = message.get("content")
            text = "" if text is None else text
            if not isinstance(text, str) or len(text) > 32768:
                raise ValueError("Invalid text")
            calls = message.get("tool_calls")
            calls = [] if calls is None else calls
            if not isinstance(calls, list) or len(calls) > 16:
                raise ValueError("Invalid tool batch")
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
            reason = choice["finish_reason"]
            if (reason not in {"stop", "tool_calls"} or (not parsed and not text)
                    or (reason == "tool_calls" and not parsed)):
                raise ValueError("Incomplete or empty response")
            return ChatReply(request.request_id, request.session_id, request.generation,
                             text, tuple(parsed), reason)
        except (ValueError, TypeError, KeyError, IndexError, UnicodeError):
            raise ProviderError("protocol_error") from None

    def close(self) -> None:
        self._closed.set()
        self._key = ""
