"""Acceptance runner against real loopback HTTP/SSE using the production SDK."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import re
import tempfile
from threading import Event
import unittest
from unittest.mock import patch

from eai_robot.applications.agent.acceptance import probe, run_case
from eai_robot.applications.agent.configuration import load_provider
from eai_robot.applications.agent.configure import generate, main
from eai_robot.policy.agents.context import ConversationHistory
from eai_robot.policy.model_io.contracts import ToolCall
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.policy.model_io.responses import ResponsesProvider
from test_agent import answer, service
from test_agent_layers import packet, streaming_service
from test_agent_standard import response_fixture, response_message


def reply_to(data, *, require_reasoning=True):
    """A protocol fixture: errors if continuation/history loses vendor metadata."""
    messages = data.get("messages", data.get("input"))
    for item in messages:
        if require_reasoning and item.get("role") == "assistant" and "reasoning_content" not in item:
            raise AssertionError("Lost completed reasoning metadata")
    last = messages[-1]
    if last.get("role") == "tool" or last.get("type") == "function_call_output":
        return json.loads(last.get("content", last.get("output")))["data"]["value"], ()
    if last["content"].startswith("Call lookup_probe"):
        key = re.search(r"with key ([a-f0-9]+)", last["content"])[1]
        return "", (ToolCall("probe-call", "lookup_probe", {"key": key}),)
    if last["content"].startswith("Repeat exactly"):
        return next(m["content"] for m in reversed(messages[:-1]) if m.get("role") == "assistant"), ()
    return last["content"].split(": ")[-1], ()


def chat_body(data):
    text, calls = reply_to(data)
    body = answer(text, calls)
    body["choices"][0]["message"]["reasoning_content"] = "OPAQUE_REASONING_NOT_FOR_REPORT"
    return body


def chat_sse(data, _):
    body = chat_body(data)
    message = body["choices"][0]["message"]
    delta = {"role": "assistant", "reasoning_content": message["reasoning_content"]}
    if message["tool_calls"]:
        delta["tool_calls"] = [{"index": i, **call} for i, call in enumerate(message["tool_calls"])]
    else:
        delta["content"] = message["content"]
    return [packet(delta) + packet(reason=body["choices"][0]["finish_reason"]) + b'data: [DONE]\n\n'], None


class AcceptanceTests(unittest.TestCase):
    def test_chat_json_and_sse_prove_tool_result_and_cross_turn_reasoning_without_replay(self):
        for streaming in (False, True):
            context = streaming_service(chat_sse) if streaming else service(lambda data: (200, chat_body(data)))
            with self.subTest(streaming=streaming), context as result:
                url, requests = result[:2]
                for case, count in (("text", 1), ("conversation", 2), ("tools", 3)):
                    provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=streaming))
                    row = run_case(provider, case, strict_tools=True)
                    self.assertEqual(row["status"], "passed", row)
                    self.assertEqual(row["model_requests"], count)
                    self.assertEqual(row["tool_calls"], 1 if case == "tools" else 0)
                    self.assertNotIn("OPAQUE_REASONING", json.dumps(row))
                # Final follow-up includes dialogue metadata, never previous calls.
                history = requests[-1]["messages"]
                self.assertFalse(any(m.get("tool_calls") or m["role"] == "tool" for m in history))
                self.assertTrue(next(m for m in history if m["role"] == "assistant")["reasoning_content"])
                self.assertTrue(requests[-2]["tools"][0]["function"]["strict"])

    def test_responses_json_tool_continuation_and_followup_use_same_acceptance_case(self):
        def responder(data):
            text, calls = reply_to(data, require_reasoning=False)
            output = ([{"type": "function_call", "id": "fc_probe", "call_id": c.id, "name": c.name,
                        "arguments": json.dumps(c.arguments), "status": "completed"} for c in calls]
                      if calls else [response_message(text)])
            return 200, response_fixture(output)
        with service(responder) as (url, requests, _):
            provider = ResponsesProvider(ProviderConfig(url, "fixture", "self_hosted"))
            row = run_case(provider, "tools")
            self.assertEqual(row["status"], "passed", row)
            self.assertEqual(row["model_requests"], 3)
            self.assertIn("function_call_output", [m.get("type") for m in requests[1]["input"]])
            self.assertFalse(any(m.get("type") == "function_call" for m in requests[2]["input"]))

    def test_responses_sse_tool_continuation_uses_completed_output_not_fragments(self):
        def event(kind, **data):
            return ("data: " + json.dumps({"type": kind, "sequence_number": 0, **data}) + "\n\n").encode()
        def responder(data, _):
            text, calls = reply_to(data, require_reasoning=False)
            beginning = event("response.created", response=response_fixture([], "in_progress"))
            if calls:
                call = calls[0]
                item = {"type": "function_call", "id": "fc_probe", "call_id": call.id, "name": call.name,
                        "arguments": json.dumps(call.arguments), "status": "completed"}
                beginning += event("response.output_item.added", output_index=0, item={**item, "arguments": "", "status": "in_progress"})
                beginning += event("response.function_call_arguments.delta", output_index=0, item_id=item["id"], delta=item["arguments"])
                output = [item]
            else:
                item = response_message(text)
                beginning += event("response.output_item.added", output_index=0, item={**item, "content": [], "status": "in_progress"})
                beginning += event("response.content_part.added", output_index=0, content_index=0, item_id=item["id"],
                                   part={"type": "output_text", "text": "", "annotations": []})
                beginning += event("response.output_text.delta", output_index=0, content_index=0, item_id=item["id"], delta=text, logprobs=[])
                output = [item]
            return [beginning + event("response.completed", response=response_fixture(output))], None
        with streaming_service(responder) as (url, _):
            row = run_case(ResponsesProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True)), "tools")
            self.assertEqual(row["status"], "passed", row)
            self.assertEqual(row["tool_calls"], 1)
            self.assertGreater(row["text_deltas"], 0)

    def test_cancel_after_actual_stream_delta_returns_without_waiting_for_remote_end(self):
        gate = Event()
        with streaming_service(lambda *_: ([packet({"content": "First"}), packet(reason="stop") + b'data: [DONE]\n\n'], gate)) as (url, requests):
            row = run_case(ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True)), "cancel")
            self.assertEqual(row["status"], "passed", row)
            self.assertFalse(gate.is_set())
            self.assertEqual(row["model_requests"], 1)
            self.assertEqual(row["tool_calls"], 0)
            self.assertIsNotNone(row["cancel_wait_ms"])
            self.assertEqual(len(requests), 1)

    def test_profile_matrix_retains_failures_sanitizes_reports_and_never_retries(self):
        with tempfile.TemporaryDirectory() as folder, service(lambda _: (503, b'PRIVATE_REMOTE_BODY')) as (bad_url, bad_requests, _), service(lambda d: (200, chat_body(d))) as (good_url, good_requests, _):
            paths = []
            for name, url in (("bad", bad_url), ("good", good_url)):
                path = Path(folder) / (name + '.toml')
                generate(preset="custom", model="fixture", output=path, base_url=url.removesuffix('/chat/completions'),
                         execution="self_hosted", no_auth=True, streaming=False)
                paths.append(path)
            report = probe(paths, transport="json", cases=("text",))
            self.assertEqual(report["status"], "failed")
            self.assertEqual((report["passed"], report["failed"]), (1, 1))
            self.assertEqual(report["results"][0]["error_code"], "service_unavailable")
            self.assertEqual(len(bad_requests), 1)
            self.assertEqual(len(good_requests), 1)
            self.assertNotIn("PRIVATE_REMOTE_BODY", json.dumps(report))
            self.assertNotIn(bad_url, json.dumps(report))
            self.assertFalse(report["hardware_connected"])

    def test_model_answer_without_tool_cannot_pass_tool_case(self):
        with service(lambda _: (200, answer("I used the tool"))) as (url, _, _):
            row = run_case(ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted")), "tools")
            self.assertEqual(row["error_code"], "tool_not_used")
            self.assertEqual(row["status"], "failed")

    def test_stream_override_is_explicit_does_not_mutate_profile_and_missing_key_stays_offline(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'chat.toml'
            result = generate(preset="deepseek", model="fixture", output=path)
            with path.open('a') as stream:
                stream.write('\n[provider.model_options.stream_options]\ninclude_usage = true\n')
            original = path.read_bytes()
            env = Path(result["env_file"])
            env.write_text('DEEPSEEK_API_KEY=PRIVATE_TEST_KEY\n')
            provider = load_provider(path, env, streaming=False)
            try:
                self.assertFalse(provider.config.streaming)
                self.assertNotIn("stream_options", provider.config.model_options)
                self.assertEqual(path.read_bytes(), original)
            finally:
                provider.close()
            with patch.dict(os.environ, {}, clear=True), patch("httpx.Client.send", side_effect=AssertionError("Must not request without key")):
                stdout = io.StringIO()
                with redirect_stdout(stdout):
                    code = main(["probe", "--provider-config", str(path), "--case", "text"])
                self.assertEqual(code, 1)
                report = json.loads(stdout.getvalue())
                self.assertEqual(report["failed"], 2)
                self.assertTrue(all(r["error_code"] == "missing_credential" for r in report["results"]))
                self.assertNotIn("PRIVATE_TEST_KEY", stdout.getvalue())

    def test_history_retains_bounded_copied_final_metadata_but_rejects_tool_queue(self):
        history = ConversationHistory(max_turns=1)
        message = {"role": "assistant", "content": "Done", "reasoning_content": "opaque"}
        history.commit("a", "Done", assistant_message=message)
        message["reasoning_content"] = "mutated"
        self.assertEqual(history.messages()[1]["reasoning_content"], "opaque")
        with self.assertRaises(ValueError):
            history.commit("b", "", assistant_message={"role": "assistant", "tool_calls": [{"id": "never-replay"}]})
        history.commit("c", "Final")
        self.assertEqual(len(history.messages()), 2)
        self.assertNotIn("reasoning_content", history.messages()[1])


if __name__ == "__main__":
    unittest.main()
