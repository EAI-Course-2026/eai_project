"""Official OpenAI SDK transport; standard Chat Completions messages and tools.

Only robotics-specific budgets, cancellation and sanitized errors live here.
HTTP, SSE decoding and streamed tool argument accumulation belong to the SDK.
"""
from dataclasses import dataclass, field
import json
import math
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import time
from urllib.parse import urlsplit

import httpx
import openai
from openai import OpenAI
from openai.types.chat import ChatCompletion

from .auth import EnvironmentCredential
from .contracts import ChatReply, ChatRequest, ModelEvent, ProviderError, ToolCall

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
    # endpoint is a deprecated complete Chat Completions URL, retained for migration.
    endpoint: str | None = None
    model: str = ""
    execution: str = "managed_api"
    credential_env: str | None = None
    timeout_s: float = 30.0
    max_tokens: int = 512
    streaming: bool = False
    base_url: str | None = None
    token_limit_parameter: str = "max_completion_tokens"
    model_options: dict = field(default_factory=dict)
    extra_body: dict = field(default_factory=dict)

    def __post_init__(self):
        if (self.base_url is None) == (self.endpoint is None):
            raise ValueError("Specify base_url, or the legacy endpoint, exclusively")
        url = self.base_url if self.base_url is not None else self.endpoint
        if not isinstance(url, str):
            raise ValueError("Expected a URL")
        parsed = urlsplit(url)
        if (parsed.scheme not in {"http", "https"} or not parsed.hostname
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or (parsed.scheme == "http" and parsed.hostname not in {"localhost", "127.0.0.1", "::1"})):
            raise ValueError("Use HTTPS or loopback HTTP without URL credentials/query/fragment")
        if self.execution not in {"local_process", "self_hosted", "managed_api"}:
            raise ValueError("Invalid execution location")
        if self.execution == "managed_api" and parsed.scheme != "https":
            raise ValueError("Managed APIs require HTTPS")
        if not isinstance(self.model, str) or not self.model.strip() or len(self.model) > 256:
            raise ValueError("A bounded model ID is required")
        EnvironmentCredential(self.credential_env)
        if (type(self.timeout_s) not in (int, float) or not math.isfinite(self.timeout_s)
                or not .1 <= self.timeout_s <= 120 or type(self.streaming) is not bool
                or type(self.max_tokens) is not int or not 1 <= self.max_tokens <= 8192
                or self.token_limit_parameter not in {"max_tokens", "max_completion_tokens"}):
            raise ValueError("Invalid provider budgets")
        supported = {"temperature", "top_p", "reasoning_effort", "tool_choice", "parallel_tool_calls",
                     "response_format", "seed", "stream_options", "reasoning", "text", "include"}
        if not isinstance(self.model_options, dict) or self.model_options.keys() - supported:
            raise ValueError("Unsupported model options")
        json.dumps(self.model_options, allow_nan=False)
        protected = {"model", "messages", "input", "tools", "stream", "max_tokens", "max_completion_tokens",
                     "max_output_tokens", "api_key", "base_url", "endpoint", "store", "previous_response_id", "conversation"}
        if not isinstance(self.extra_body, dict) or self.extra_body.keys() & (protected | supported):
            raise ValueError("extra_body cannot override routing, messages, tools, budgets or state")
        json.dumps(self.extra_body, allow_nan=False)
        if "stream_options" in self.model_options and not self.streaming:
            raise ValueError("stream_options requires streaming")

    @property
    def api_base_url(self):
        if self.base_url is not None:
            return self.base_url.rstrip("/") + "/"
        return self.endpoint.removesuffix("/chat/completions").removesuffix("/responses").rstrip("/") + "/"


class _BoundedBody(httpx.SyncByteStream):
    """Enforce total bytes/deadline without parsing or interpreting SSE."""
    def __init__(self, stream, deadline, closed):
        self.stream, self.deadline, self.closed = stream, deadline, closed

    def __iter__(self):
        size = 0
        for chunk in self.stream:
            if self.closed.is_set():
                raise ProviderError("cancelled")
            if time.monotonic() >= self.deadline:
                raise ProviderError("timeout")
            size += len(chunk)
            if size > MAX_BODY:
                raise ProviderError("response_too_large")
            yield chunk

    def close(self):
        self.stream.close()


def parse_reply(request, completion):
    # SDK Pydantic models, with required protocol fields checked before dispatch.
    data = completion.model_dump(exclude_none=True, warnings=False)
    completion = ChatCompletion.model_validate(data)
    if len(completion.choices) != 1:
        raise ValueError("Expected one choice")
    choice = completion.choices[0]
    message, reason = choice.message, choice.finish_reason
    text = message.content or ""
    if (message.refusal or reason not in {"stop", "tool_calls"}
            or not isinstance(text, str) or len(text) > 32768):
        raise ValueError("Incomplete or refused response")
    calls = message.tool_calls or []
    if len(calls) > 16:
        raise ValueError("Too many tool calls")
    parsed = []
    for call in calls:
        if (call.type != "function" or not 1 <= len(call.id) <= 128
                or not 1 <= len(call.function.name) <= 64 or len(call.function.arguments) > 65536):
            raise ValueError("Invalid function call")
        args = strict_json(call.function.arguments)
        if not isinstance(args, dict):
            raise ValueError("Function arguments must be an object")
        parsed.append(ToolCall(call.id, call.function.name, args))
    if (not text and not parsed) or (reason == "tool_calls" and not parsed):
        raise ValueError("Empty response")
    assistant = message.model_dump(exclude_none=True)
    assistant.pop("parsed", None)
    for call in assistant.get("tool_calls", []):
        call.get("function", {}).pop("parsed_arguments", None)
    return ChatReply(request.request_id, request.session_id, request.generation, text, tuple(parsed), reason,
                     completion.usage.model_dump() if completion.usage else {}, assistant)


class ChatCompletionsProvider:
    """One bounded SDK worker. Cancellation stops local waiting, not remote billing."""
    _adapter = "chat_completions"
    _inputs = ["text", "image_url", "input_audio"]
    _allowed_options = {"temperature", "top_p", "reasoning_effort", "tool_choice", "parallel_tool_calls",
                        "response_format", "seed", "stream_options"}
    def __init__(self, config: ProviderConfig, *, credential_values=None):
        if config.model_options.keys() - self._allowed_options:
            raise ValueError("Options do not belong to the selected API")
        self.config = config
        self._credential = EnvironmentCredential(config.credential_env, credential_values)
        self._credential.resolve()
        self._closed, self._active = Event(), Lock()

    def describe(self):
        return {"adapter": self._adapter, "sdk": "openai", "sdk_version": openai.__version__,
                "model": self.config.model, "model_revision": "unknown", "execution": self.config.execution,
                "inputs": self._inputs, "capabilities_verified": False, "outputs": ["text", "tool_calls"],
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
        kwargs = self._request_kwargs(request)
        if len(json.dumps(kwargs, allow_nan=False).encode()) > MAX_BODY:
            raise ProviderError("request_too_large")
        def bound(response):
            response.stream = _BoundedBody(response.stream, deadline, self._closed)
            if not self.config.streaming and response.is_success:
                try:
                    strict_json(response.read())
                except (ValueError, UnicodeError):
                    raise ProviderError("protocol_error") from None
        # Never discover OPENAI_API_KEY when no credential_env was selected.
        key = self._credential.resolve()
        transport = httpx.Client(follow_redirects=False, event_hooks={"response": [bound]})
        try:
            with OpenAI(api_key=key or "not-required", base_url=self.config.api_base_url,
                        max_retries=0, timeout=min(request.timeout_s, self.config.timeout_s),
                        http_client=transport) as client:
                return self._sdk_exchange(client, request, kwargs, publish)
        except openai.APIStatusError as exc:
            status = exc.status_code
            raise ProviderError("authentication" if status in (401, 403) else
                                "rate_limited" if status == 429 else
                                "redirect_refused" if 300 <= status < 400 else
                                "invalid_request" if status in (400, 422) else
                                "endpoint_or_model_not_found" if status == 404 else
                                "service_unavailable" if 500 <= status < 600 else "http_error") from None
        except (openai.APITimeoutError, httpx.TimeoutException, TimeoutError):
            raise ProviderError("timeout") from None
        except openai.APIConnectionError as exc:
            cause = exc.__cause__
            if isinstance(cause, ProviderError):
                raise cause from None
            raise ProviderError("transport_error") from None
        except (ValueError, TypeError, KeyError, IndexError, AttributeError, AssertionError,
                openai.APIResponseValidationError, openai.LengthFinishReasonError,
                openai.ContentFilterFinishReasonError):
            raise ProviderError("protocol_error") from None
        finally:
            transport.close()

    def _request_kwargs(self, request):
        kwargs = {"model": self.config.model, "messages": list(request.messages),
                  self.config.token_limit_parameter: self.config.max_tokens,
                  **self.config.model_options}
        if self.config.extra_body:
            kwargs["extra_body"] = self.config.extra_body
        if request.tools:
            kwargs["tools"] = list(request.tools)
            kwargs.setdefault("parallel_tool_calls", False)
        return kwargs

    def _sdk_exchange(self, client, request, kwargs, publish):
        if self.config.streaming:
            with client.chat.completions.stream(**kwargs) as stream:
                for event in stream:
                    if event.type == "content.delta":
                        publish(ModelEvent("text_delta", text=event.delta))
                completion = stream.get_final_completion()
        else:
            completion = client.chat.completions.create(**kwargs)
        return parse_reply(request, completion)

    def close(self):
        self._closed.set()
