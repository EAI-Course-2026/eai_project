"""Compatibility at the SDK/wire boundary, not an emulation of model intelligence."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
from threading import Event, Thread
import unittest
from unittest.mock import patch

import openai
from pydantic import BaseModel, ConfigDict, Field

from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.tools import Tool, ToolRegistry
from eai_robot.policy.model_io.contracts import ChatRequest, ToolCall
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from test_agent import answer, request, service, tool_context
from test_agent_layers import packet, streaming_service


class Target(BaseModel):
    model_config = ConfigDict(extra="forbid")
    object_id: int = Field(ge=0)
    offset_mm: list[float] = Field(min_length=3, max_length=3)
    track: bool
    label: str | None


class Command(BaseModel):
    model_config = ConfigDict(extra="forbid")
    target: Target


def fixture_tool(handler=lambda a, c: ToolResult(True, "simulated", a)):
    return Tool(openai.pydantic_function_tool(Command, name="target", description="Plan only"), handler)


class StandardCompatibilityTests(unittest.TestCase):
    def test_official_pydantic_tool_is_registered_and_sent_without_translation(self):
        spec = openai.pydantic_function_tool(Command, name="target", description="Plan only")
        registry = ToolRegistry([Tool(spec, lambda a, c: ToolResult(True, "simulated", a))])
        # Definitions are standard dictionaries; $defs/$ref and strict are preserved.
        self.assertEqual(registry.definitions[0], spec)
        self.assertIn("$defs", spec["function"]["parameters"])
        with service(lambda _: (200, answer())) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'),
                                                             model="chosen-model", execution="self_hosted"))
            self.addCleanup(provider.close)
            req = request()
            provider.complete(ChatRequest(req.request_id, req.session_id, req.generation,
                                          req.messages, registry.definitions, req.timeout_s), cancel=Event())
            self.assertEqual(requests[0]["tools"], [spec])
            self.assertFalse(requests[0]["parallel_tool_calls"])
            self.assertEqual(requests[0]["model"], "chosen-model")
            self.assertEqual(requests[0]["max_completion_tokens"], 512)
            self.assertNotIn("max_tokens", requests[0])
        # Caller mutation cannot change the executable registry.
        spec["function"]["parameters"]["$defs"]["Target"]["properties"]["object_id"]["minimum"] = -99
        self.assertEqual(registry.prepare(ToolCall("bad", "target", {"target": {
            "object_id": -1, "offset_mm": [1, 2, 3], "track": True, "label": None}})).error_code, "invalid_arguments")

    def test_numeric_boolean_nested_nullable_and_array_constraints_before_handler(self):
        executed = []
        registry = ToolRegistry([fixture_tool(lambda a, c: executed.append(a) or ToolResult(True, "simulated", a))])
        valid = {"target": {"object_id": 7, "offset_mm": [1, 2.5, -3], "track": True, "label": None}}
        result = registry.invoke(ToolCall("ok", "target", valid), tool_context())
        self.assertTrue(result.ok)
        self.assertEqual(executed, [valid])
        for changes in ({"object_id": True}, {"object_id": -1}, {"object_id": "7"},
                        {"offset_mm": [1, 2]}, {"offset_mm": [1, float('nan'), 3]},
                        {"track": "true"}, {"extra": 1}):
            invalid = deepcopy(valid)
            invalid["target"].update(changes)
            with self.subTest(changes=changes):
                self.assertEqual(registry.invoke(ToolCall("bad", "target", invalid), tool_context()).error_code,
                                 "invalid_arguments")
        self.assertEqual(len(executed), 1)

    def test_non_strict_schema_optional_fields_and_anyof_are_not_rewritten(self):
        spec = {"type": "function", "function": {"name": "flexible", "strict": False, "parameters": {
            "type": "object", "properties": {"value": {"anyOf": [{"type": "integer"}, {"type": "null"}]}},
            "additionalProperties": False}}}
        registry = ToolRegistry([Tool(spec, lambda a, c: ToolResult(True, "accepted"))])
        self.assertEqual(registry.definitions[0], spec)
        for args in ({}, {"value": 4}, {"value": None}):
            self.assertTrue(registry.invoke(ToolCall("x", "flexible", args), tool_context()).ok)
        self.assertEqual(registry.invoke(ToolCall("x", "flexible", {"value": "4"}), tool_context()).error_code,
                         "invalid_arguments")

    def test_external_schema_reference_never_fetches_remote_content(self):
        spec = {"type": "function", "function": {"name": "ref", "parameters": {
            "type": "object", "properties": {"x": {"$ref": "https://example.invalid/schema.json"}}, "required": ["x"]}}}
        registry = ToolRegistry([Tool(spec, lambda a, c: self.fail("Unresolved reference executed"))])
        self.assertEqual(registry.invoke(ToolCall("x", "ref", {"x": 1}), tool_context()).error_code, "invalid_arguments")

    def test_standard_multimodal_user_content_and_completed_history_are_preserved(self):
        parts = [{"type": "text", "text": "Describe without moving"},
                 {"type": "image_url", "image_url": {"url": "data:image/png;base64,ZmFrZQ==", "detail": "low"}},
                 {"type": "input_audio", "input_audio": {"data": "ZmFrZQ==", "format": "wav"}}]
        with service(lambda _: (200, answer("fixture response"))) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'),
                                                             model="multimodal-fixture", execution="self_hosted"))
            session = AgentSession(provider, ToolRegistry([]))
            self.addCleanup(session.close)
            self.assertEqual(session.run_turn(parts).status, "completed")
            self.assertEqual(requests[0]["messages"][-1], {"role": "user", "content": parts})
            parts[0]["text"] = "Caller mutation"
            self.assertEqual(session.run_turn("Follow up").status, "completed")
            self.assertEqual(requests[-1]["messages"][1]["content"][0]["text"], "Describe without moving")
            self.assertNotIn("tools", requests[0])
            self.assertEqual(session.run_turn([{"type": "image_url", "image_url": {"url": 123}}]).error_code, "invalid_text")
            self.assertEqual(len(requests), 2)

    def test_sdk_interleaves_multiple_tools_and_reports_usage_after_final_packet(self):
        schema = {"type": "function", "function": {"name": "work", "strict": False, "parameters": {
            "type": "object", "properties": {"n": {"type": "integer"}}, "required": ["n"], "additionalProperties": False}}}
        body = packet({"role": "assistant", "tool_calls": [
            {"index": 0, "id": "a", "type": "function", "function": {"name": "work", "arguments": '{"n":'}},
            {"index": 1, "id": "b", "type": "function", "function": {"name": "work", "arguments": '{"n":'}}]})
        body += packet({"tool_calls": [{"index": 1, "function": {"arguments": '2}'}},
                                       {"index": 0, "function": {"arguments": '1}'}}]})
        body += packet(reason="tool_calls")
        body += b'data: {"id":"fixture","object":"chat.completion.chunk","created":0,"model":"fixture","choices":[],"usage":{"prompt_tokens":10,"completion_tokens":4,"total_tokens":14}}\n\n'
        body += b'data: [DONE]\n\n'
        invoked, events = [], []
        def responder(data, index):
            return ([body], None) if index == 1 else ([packet({"content": "Done"}) + packet(reason="stop") + b'data: [DONE]\n\n'], None)
        with streaming_service(responder) as (url, requests):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True,
                                                             model_options={"stream_options": {"include_usage": True}}))
            registry = ToolRegistry([Tool(schema, lambda a, c: invoked.append(a["n"]) or ToolResult(True, "simulated"))])
            session = AgentSession(provider, registry, emit=events.append)
            self.addCleanup(session.close)
            self.assertEqual(session.run_turn("Fixture").status, "completed")
            self.assertEqual(invoked, [1, 2])
            self.assertEqual([m["tool_call_id"] for m in requests[1]["messages"] if m["role"] == "tool"], ["a", "b"])
            self.assertEqual(next(e for e in events if e["type"] == "model_end")["data"]["usage"]["total_tokens"], 14)

    def test_reasoning_parameters_and_legacy_token_field_are_explicit(self):
        with service(lambda _: (200, answer())) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "operator-selected", "self_hosted",
                token_limit_parameter="max_tokens", model_options={"reasoning_effort": "low", "temperature": .2}))
            self.addCleanup(provider.close)
            provider.complete(request(), cancel=Event())
            self.assertEqual(requests[0]["reasoning_effort"], "low")
            self.assertEqual(requests[0]["temperature"], .2)
            self.assertEqual(requests[0]["max_tokens"], 512)
            self.assertNotIn("max_completion_tokens", requests[0])
        with self.assertRaises(ValueError):
            ProviderConfig(base_url="https://example.invalid/v1", model="m", model_options={"messages": []})

    def test_sdk_disables_retries_and_does_not_discover_unselected_credentials(self):
        for code in (429, 500):
            with service(lambda _: (code, {"error": {"message": "PRIVATE BODY"}})) as (url, requests, headers):
                provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted"))
                self.addCleanup(provider.close)
                with patch.dict(os.environ, {"OPENAI_API_KEY": "MUST_NOT_BE_DISCOVERED"}):
                    with self.assertRaises(Exception) as caught:
                        provider.complete(request(), cancel=Event())
                self.assertEqual(len(requests), 1)
                self.assertNotIn("PRIVATE BODY", str(caught.exception))
                self.assertNotIn("MUST_NOT_BE_DISCOVERED", json.dumps(headers))

# Full documented Responses objects, used only by the local HTTP fixture.
def response_fixture(output, status="completed"):
    return {"id": "resp_fixture", "object": "response", "created_at": 0, "model": "fixture",
            "status": status, "error": None, "incomplete_details": None, "parallel_tool_calls": False,
            "tool_choice": "auto", "tools": [], "output": output,
            "usage": {"input_tokens": 10, "output_tokens": 3, "total_tokens": 13,
                      "input_tokens_details": {"cached_tokens": 0}, "output_tokens_details": {"reasoning_tokens": 1}}}


def response_message(text):
    return {"type": "message", "id": "msg_fixture", "role": "assistant", "status": "completed",
            "content": [{"type": "output_text", "text": text, "annotations": []}]}


class ResponsesCompatibilityTests(unittest.TestCase):
    def test_responses_keeps_reasoning_and_call_ids_in_stateless_tool_loop(self):
        from eai_robot.policy.model_io.responses import ResponsesProvider
        reasoning = {"type": "reasoning", "id": "rs_fixture", "summary": [], "encrypted_content": "opaque-fixture"}
        call = {"type": "function_call", "id": "fc_fixture", "call_id": "call_fixture",
                "name": "target", "arguments": '{"target":{"object_id":2,"offset_mm":[0,0,0],"track":false,"label":null}}',
                "status": "completed"}
        def respond(data):
            if not any(i.get("type") == "function_call_output" for i in data["input"]):
                return 200, response_fixture([reasoning, call])
            return 200, response_fixture([response_message("Planned, not moved")])
        with service(respond) as (url, requests, _):
            provider = ResponsesProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'),
                model="responses-fixture", execution="self_hosted", model_options={"reasoning": {"effort": "low"}}))
            events = []
            session = AgentSession(provider, ToolRegistry([fixture_tool()]), emit=events.append)
            self.addCleanup(session.close)
            self.assertEqual(session.run_turn("Plan").status, "completed")
            first, second = requests
            self.assertFalse(first["store"])
            self.assertEqual(first["max_output_tokens"], 512)
            self.assertEqual(first["reasoning"], {"effort": "low"})
            self.assertEqual(first["tools"][0]["type"], "function")
            self.assertNotIn("function", first["tools"][0])
            self.assertTrue(first["tools"][0]["strict"])
            self.assertIn(reasoning, second["input"])
            self.assertIn(call, second["input"])
            result = next(i for i in second["input"] if i.get("type") == "function_call_output")
            self.assertEqual(result["call_id"], "call_fixture")
            self.assertTrue(json.loads(result["output"])["ok"])
            self.assertEqual(next(e for e in events if e["type"] == "model_end")["data"]["usage"]["total_tokens"], 13)
            session.run_turn("Another")
            # A new user turn must not replay encrypted state, tool outputs or calls.
            self.assertFalse(any(i.get("type") in {"reasoning", "function_call", "function_call_output"} for i in requests[2]["input"]))

    def test_responses_rejects_incomplete_before_tool_dispatch(self):
        from eai_robot.policy.model_io.responses import ResponsesProvider
        executed = []
        call = {"type": "function_call", "id": "fc_fixture", "call_id": "call_fixture", "name": "target",
                "arguments": '{}', "status": "completed"}
        body = response_fixture([call], status="incomplete")
        body["incomplete_details"] = {"reason": "max_output_tokens"}
        with service(lambda _: (200, body)) as (url, _, _):
            provider = ResponsesProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'), model="fixture", execution="self_hosted"))
            session = AgentSession(provider, ToolRegistry([fixture_tool(lambda a,c: executed.append(a))]))
            self.addCleanup(session.close)
            self.assertEqual(session.run_turn("Fixture").error_code, "incomplete_response")
            self.assertEqual(executed, [])

    def test_responses_sdk_stream_requires_completed_event(self):
        from eai_robot.policy.model_io.responses import ResponsesProvider
        def event(kind, **data):
            return ("event: " + kind + "\ndata: " + json.dumps({"type": kind, "sequence_number": 0, **data}) + "\n\n").encode()
        created = response_fixture([], status="in_progress")
        completed = response_fixture([response_message("Streamed")])
        raw = event("response.created", response=created)
        raw += event("response.output_item.added", output_index=0, item={**response_message(""), "content": [], "status": "in_progress"})
        raw += event("response.content_part.added", output_index=0, content_index=0, item_id="msg_fixture", part={"type": "output_text", "text": "", "annotations": []})
        raw += event("response.output_text.delta", output_index=0, content_index=0, item_id="msg_fixture", delta="Streamed", logprobs=[])
        ending = event("response.completed", response=completed)
        for complete in (True, False):
            with self.subTest(complete=complete), streaming_service(lambda *_: ([raw + (ending if complete else b'')], None)) as (url, _):
                provider = ResponsesProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'), model="fixture",
                                                            execution="self_hosted", streaming=True))
                self.addCleanup(provider.close)
                if complete:
                    events = list(provider.stream(request(), cancel=Event()))
                    self.assertEqual(events[-1].reply.text, "Streamed")
                    self.assertEqual("".join(e.text for e in events), "Streamed")
                else:
                    with self.assertRaisesRegex(Exception, "protocol_error"):
                        provider.complete(request(), cancel=Event())

    def test_responses_streamed_function_dispatch_waits_for_terminal_response(self):
        from eai_robot.policy.model_io.responses import ResponsesProvider
        gate, seen = Event(), Event()
        executed = []
        arguments = '{"target":{"object_id":2,"offset_mm":[0,0,0],"track":false,"label":null}}'
        call = {"type": "function_call", "id": "fc_fixture", "call_id": "call_fixture", "name": "target",
                "arguments": arguments, "status": "completed"}
        def event(kind, **data):
            return ("data: " + json.dumps({"type": kind, "sequence_number": 0, **data}) + "\n\n").encode()
        def respond(data, index):
            beginning = event("response.created", response=response_fixture([], "in_progress"))
            if index == 1:
                beginning += event("response.output_item.added", output_index=0,
                                   item={**call, "arguments": "", "status": "in_progress"})
                beginning += event("response.function_call_arguments.delta", output_index=0,
                                   item_id="fc_fixture", delta=arguments)
                seen.set()
                return ([beginning, event("response.completed", response=response_fixture([call]))], gate)
            return ([beginning + event("response.completed", response=response_fixture([response_message("Done")]))], None)
        with streaming_service(respond) as (url, requests):
            provider = ResponsesProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'), model="fixture",
                                                        execution="self_hosted", streaming=True, timeout_s=10))
            session = AgentSession(provider, ToolRegistry([fixture_tool(lambda a,c: executed.append(a) or ToolResult(True, "simulated"))]))
            self.addCleanup(session.close)
            results = []
            worker = Thread(target=lambda: results.append(session.run_turn("Fixture")), daemon=True)
            worker.start()
            self.assertTrue(seen.wait(5))
            self.assertEqual(executed, [])
            gate.set()
            worker.join(5)
            self.assertFalse(worker.is_alive())
            self.assertEqual(results[0].status, "completed")
            self.assertEqual(len(executed), 1)
            replay = next(i for i in requests[1]["input"] if i.get("type") == "function_call")
            self.assertNotIn("parsed_arguments", replay)

    def test_responses_image_shape_and_unsupported_audio_fail_explicitly(self):
        from eai_robot.policy.model_io.responses import ResponsesProvider
        with service(lambda _: (200, response_fixture([response_message("Fixture")]))) as (url, requests, _):
            provider = ResponsesProvider(ProviderConfig(base_url=url.removesuffix('/chat/completions'), model="fixture", execution="self_hosted"))
            session = AgentSession(provider, ToolRegistry([]))
            self.addCleanup(session.close)
            content = [{"type": "image_url", "image_url": {"url": "data:image/png;base64,ZmFrZQ==", "detail": "low"}}]
            self.assertEqual(session.run_turn(content).status, "completed")
            self.assertEqual(requests[0]["input"][-1]["content"], [{"type": "input_image", "image_url": content[0]["image_url"]["url"], "detail": "low"}])
            self.assertEqual(session.run_turn([{"type": "input_audio", "input_audio": {"data": "ZmFrZQ==", "format": "wav"}}]).error_code,
                             "unsupported_modality")
            self.assertEqual(len(requests), 1)


class AgentCLIContractTests(unittest.TestCase):
    def test_list_tools_needs_no_service_or_credentials(self):
        from jsonschema import Draft202012Validator
        root = Path(__file__).resolve().parents[1]
        proc = subprocess.run([sys.executable, "-m", "eai_robot.applications.agent.cli", "--list-tools"],
                              cwd=root, capture_output=True, text=True, timeout=15,
                              env=dict(os.environ, PYTHONPATH=str(root / "src")))
        self.assertEqual(proc.returncode, 0, proc.stderr)
        data = json.loads(proc.stdout)
        self.assertFalse(data["hardware_connected"])
        self.assertEqual(len(data["tools"]), 5)
        for spec in data["tools"]:
            self.assertEqual(set(spec), {"type", "function"})
            Draft202012Validator.check_schema(spec["function"]["parameters"])
        self.assertNotIn("handler", proc.stdout)


if __name__ == "__main__":
    unittest.main()
