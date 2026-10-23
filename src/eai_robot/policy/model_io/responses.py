"""Official Responses API adapter over the same standard function registry.

The only conversion is between two documented OpenAI API wire shapes. Complete
provider output (including encrypted reasoning) is carried opaquely by the core
within this turn, and never restored as an executable queue on another turn.
"""
from copy import deepcopy

from .contracts import ChatReply, ModelEvent, ProviderError, ToolCall
from .providers import ChatCompletionsProvider, strict_json


class ResponsesProvider(ChatCompletionsProvider):
    _adapter = "responses"
    _inputs = ["text", "image_url"]
    _allowed_options = {"temperature", "top_p", "reasoning", "text", "include", "tool_choice", "parallel_tool_calls"}

    def _request_kwargs(self, request):
        items, states = [], iter(request.provider_state)
        for message in request.messages:
            role, content = message["role"], message.get("content")
            if role == "tool":
                items.append({"type": "function_call_output", "call_id": message["tool_call_id"], "output": content})
            elif role == "assistant" and message.get("tool_calls"):
                state = next(states, None)
                if not isinstance(state, dict) or not isinstance(state.get("output"), list):
                    raise ProviderError("missing_provider_state")
                items.extend(deepcopy(state["output"]))
            else:
                if isinstance(content, list):
                    converted = []
                    for part in content:
                        if part["type"] == "text":
                            converted.append({"type": "input_text", "text": part["text"]})
                        elif part["type"] == "image_url":
                            converted.append({"type": "input_image", "image_url": part["image_url"]["url"],
                                              "detail": part["image_url"].get("detail", "auto")})
                        else:
                            raise ProviderError("unsupported_modality")
                    content = converted
                items.append({"role": role, "content": content or ""})
        kwargs = {"model": self.config.model, "input": items, "max_output_tokens": self.config.max_tokens,
                  "store": False, "include": ["reasoning.encrypted_content"], **self.config.model_options}
        if self.config.extra_body:
            kwargs["extra_body"] = deepcopy(self.config.extra_body)
        if request.tools:
            kwargs["tools"] = [{"type": "function", **deepcopy(t["function"])} for t in request.tools]
            kwargs.setdefault("parallel_tool_calls", False)
        return kwargs

    def _sdk_exchange(self, client, request, kwargs, publish):
        if self.config.streaming:
            with client.responses.stream(**kwargs) as stream:
                for event in stream:
                    if event.type == "response.output_text.delta":
                        publish(ModelEvent("text_delta", text=event.delta))
                response = stream.get_final_response()
        else:
            response = client.responses.create(**kwargs)
        if response.status != "completed" or response.error or response.incomplete_details:
            raise ProviderError("incomplete_response")
        calls = []
        for item in response.output:
            if item.type == "function_call":
                if (not 1 <= len(item.call_id) <= 128 or not 1 <= len(item.name) <= 64
                        or len(item.arguments) > 65536 or item.status not in (None, "completed")):
                    raise ProviderError("protocol_error")
                arguments = strict_json(item.arguments)
                if not isinstance(arguments, dict):
                    raise ProviderError("protocol_error")
                calls.append(ToolCall(item.call_id, item.name, arguments))
            elif item.type == "message":
                if item.status != "completed" or any(p.type == "refusal" for p in item.content):
                    raise ProviderError("refused_response")
            elif item.type != "reasoning":
                raise ProviderError("unsupported_output")
        text = response.output_text
        if len(calls) > 16 or len(text) > 32768 or (not calls and not text):
            raise ProviderError("protocol_error")
        return ChatReply(request.request_id, request.session_id, request.generation, text, tuple(calls),
                         "tool_calls" if calls else "stop", response.usage.model_dump(exclude_none=True, warnings=False) if response.usage else {},
                         provider_state={"output": [item.model_dump(exclude_none=True, exclude={"parsed_arguments"}, warnings=False) for item in response.output]})
