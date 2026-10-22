"""Three-layer contracts, SSE framing, conversation and local task cancellation."""
import ast
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import sys
import subprocess
import tempfile
from threading import Event, Thread
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from eai_robot.policy.model_io.contracts import ChatReply, Message, ModelEvent, ToolCall
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.policy.agents.context import ContextSnapshot
from eai_robot.policy.agents.contracts import ToolInterrupted, ToolResult
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.agents.tools import Tool, ToolRegistry
from eai_robot.applications.agent.capabilities import RobotCapabilities
from eai_robot.applications.agent.demo import DemoProvider
from eai_robot.applications.agent.session import create_robot_session
from eai_robot.applications.agent.skills import DryRunSkills
from test_agent import answer, request, service

EMPTY = {"type": "object", "properties": {}, "additionalProperties": False}


def packet(delta=None, reason=None):
    delta = dict(delta or {})
    if "content" in delta or any("id" in t for t in delta.get("tool_calls", [])):
        delta["role"] = "assistant"
    return ("data: " + json.dumps({"id": "fixture", "object": "chat.completion.chunk", "created": 0,
                                  "model": "fixture", "choices": [{"index": 0, "delta": delta, "finish_reason": reason}]},
                                  ensure_ascii=False) + "\n\n").encode()


@contextmanager
def streaming_service(responder):
    requests, gates = [], []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_POST(self):
            data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            requests.append(data)
            chunks, gate = responder(data, len(requests))
            if gate is not None:
                gates.append(gate)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Content-Length", str(sum(len(c) for c in chunks)))
            self.end_headers()
            try:
                for index, chunk in enumerate(chunks):
                    if index and gate is not None:
                        gate.wait(10)
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    thread = Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}/chat/completions", requests
    finally:
        for gate in gates:
            gate.set()
        server.shutdown()
        server.server_close()
        thread.join(2)


class SingleToolProvider(DemoProvider):
    def __init__(self, name="work"):
        super().__init__()
        self.name, self.requests = name, []
    def complete(self, req, *, cancel):
        self.requests.append(req)
        self.index += 1
        return ChatReply(req.request_id, req.session_id, req.generation,
                         tool_calls=(ToolCall("work-1", self.name, {}),), finish_reason="tool_calls") if self.index == 1 else ChatReply(req.request_id, req.session_id, req.generation, "finished")


class LayersTests(unittest.TestCase):
    def test_import_boundaries_are_enforced(self):
        for layer, blocked in (("model_io", ("eai_robot.policy.agents", "eai_robot.applications", "eai_robot.control", "eai_robot.hardware", "eai_robot.vision")),
                               ("agents", ("eai_robot.applications", "eai_robot.control", "eai_robot.hardware", "eai_robot.vision", "eai_robot.policy.model_io.providers", "eai_robot.policy.model_io.auth"))):
            for file in (ROOT / "src/eai_robot/policy" / layer).rglob("*.py"):
                for node in ast.walk(ast.parse(file.read_text(encoding="utf-8"))):
                    names = ([node.module or ""] if isinstance(node, ast.ImportFrom) else
                             [a.name for a in node.names] if isinstance(node, ast.Import) else [])
                    for name in names:
                        self.assertFalse(name.startswith(blocked), f"{file}: {name}")
        self.assertFalse((ROOT / "src/eai_robot/policy/agents/prompts").exists())

    def test_sse_is_live_and_tool_arguments_do_not_execute_until_final(self):
        gate, text_seen = Event(), Event()
        skills = DryRunSkills({"stand": [.5] * 6})
        first = packet({"content": "准备", "tool_calls": [{"index": 0, "id": "stream-call", "type": "function",
                         "function": {"name": "move_to_pose", "arguments": '{"pose_name":'}}]})
        last = packet({"tool_calls": [{"index": 0, "function": {"arguments": '"stand"}'}}]}) + packet(reason="tool_calls") + b'data: [DONE]\n\n'
        def responder(data, index):
            return ([first, last], gate) if index == 1 else ([packet({"content": "仅演练"}) + packet(reason="stop") + b'data: [DONE]\n\n'], None)
        with streaming_service(responder) as (url, requests):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", timeout_s=10, streaming=True))
            events, results = [], []
            def emit(event):
                events.append(event)
                if event["type"] == "model_delta":
                    text_seen.set()
            session = create_robot_session(provider, skills, emit=emit, timeout_s=10)
            self.addCleanup(session.close)
            worker = Thread(target=lambda: results.append(session.run_turn("演练")), daemon=True)
            worker.start()
            self.assertTrue(text_seen.wait(5))
            self.assertEqual(skills.tasks, {})
            self.assertTrue(worker.is_alive())
            gate.set()
            worker.join(5)
            self.assertFalse(worker.is_alive())
            self.assertEqual(results[0].status, "completed")
            self.assertEqual(len(skills.tasks), 1)
            self.assertTrue(all(item["stream"] for item in requests))
            kinds = [e["type"] for e in events]
            self.assertLess(kinds.index("model_delta"), kinds.index("tool_start"))
            self.assertEqual(kinds[-2:], ["run_end", "run_settled"])
            self.assertEqual([e["sequence"] for e in events], list(range(1, len(events) + 1)))

    def test_unfinished_stream_never_dispatches_a_complete_looking_tool(self):
        body = packet({"tool_calls": [{"index": 0, "id": "x", "type": "function", "function": {
            "name": "move_to_pose", "arguments": '{"pose_name":"stand"}'}}]})
        skills = DryRunSkills({"stand": [.5] * 6})
        with service(lambda _: (200, body)) as (url, _, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True))
            session = create_robot_session(provider, skills)
            self.addCleanup(session.close)
            result = session.run_turn("演练")
        self.assertEqual(result.error_code, "protocol_error")
        self.assertEqual(skills.tasks, {})

    def test_sse_utf8_framing_and_invalid_tool_json(self):
        raw = packet({"content": "你好"}) + packet(reason="stop") + b'data: [DONE]\n\n'
        with streaming_service(lambda *_: ([bytes([b]) for b in raw], None)) as (url, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True))
            self.addCleanup(provider.close)
            events = list(provider.stream(request(), cancel=Event()))
            self.assertEqual(events[-1].reply.text, "你好")
            self.assertEqual("".join(e.text for e in events), "你好")
        invalid = packet({"tool_calls": [{"index": 0, "id": "x", "type": "function", "function": {
            "name": "work", "arguments": '{"incomplete":'}}]}) + packet(reason="tool_calls") + b'data: [DONE]\n\n'
        with streaming_service(lambda *_: ([invalid], None)) as (url, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", streaming=True))
            self.addCleanup(provider.close)
            with self.assertRaisesRegex(Exception, "protocol_error"):
                provider.complete(request(), cancel=Event())

    def test_credentials_are_resolved_per_request_and_never_in_messages(self):
        with service(lambda _: (200, answer())) as (url, requests, headers), patch.dict(os.environ, {"ROTATING_TEST_KEY": "first"}):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted", "ROTATING_TEST_KEY"))
            self.addCleanup(provider.close)
            provider.complete(request(), cancel=Event())
            os.environ["ROTATING_TEST_KEY"] = "second"
            provider.complete(request(), cancel=Event())
            self.assertEqual([h["Authorization"] for h in headers], ["Bearer first", "Bearer second"])
            self.assertNotIn("second", json.dumps(requests))

    def test_completed_dialogue_survives_with_bounded_memory_but_tool_queue_does_not(self):
        class Provider(DemoProvider):
            def __init__(self):
                super().__init__()
                self.requests = []
            def complete(self, req, *, cancel):
                self.requests.append(req)
                return ChatReply(req.request_id, req.session_id, req.generation, "answer " + req.messages[-1]["content"])
        provider = Provider()
        session = AgentSession(provider, ToolRegistry([]), history_turns=1)
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("first").status, "completed")
        self.assertEqual(session.run_turn("second").status, "completed")
        self.assertEqual([m["content"] for m in provider.requests[-1].messages[1:]], ["first", "answer first", "second"])
        session.run_turn("third")
        self.assertEqual([m["content"] for m in provider.requests[-1].messages[1:]], ["second", "answer second", "third"])
        self.assertTrue(all(isinstance(m, dict) for m in provider.requests[-1].messages))
        session.clear_history()
        self.assertEqual(session.snapshot()["history_messages"], 0)

    def test_tool_results_stay_out_of_next_user_turn(self):
        provider = SingleToolProvider()
        session = AgentSession(provider, ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, lambda a, ctx: ToolResult(True, "accepted", {"task_id": "pending"}))]))
        self.addCleanup(session.close)
        session.run_turn("first")
        self.assertTrue(any(m["role"] == "tool" for m in provider.requests[-1].messages))
        session.run_turn("second")
        self.assertFalse(any(m["role"] == "tool" or m.get("tool_calls") for m in provider.requests[-1].messages))

    def test_context_is_refreshed_for_every_model_request_and_stale_is_rejected(self):
        calls = []
        def context():
            calls.append(True)
            return ContextSnapshot("fresh declaration", time.monotonic() if len(calls) == 1 else time.monotonic() - 10, 1.)
        provider = SingleToolProvider()
        session = AgentSession(provider, ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, lambda a, ctx: ToolResult(True, "simulated"))]), context_provider=context)
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("test").error_code, "stale_context")
        self.assertEqual(len(provider.requests), 1)
        self.assertEqual(session.snapshot()["history_messages"], 0)

    def test_cancel_does_not_wait_for_tool_and_draining_prevents_overlap(self):
        entered, release, stopped, settled = Event(), Event(), Event(), Event()
        self.addCleanup(release.set)
        progress, results, events = [], [], []
        def handler(args, context):
            progress.append(context)
            context.progress({"phase": "working"})
            entered.set()
            release.wait(10)
            context.check_active()
            return ToolResult(True, "completed")
        def stop():
            stopped.set()
            return ToolResult(True, "cancelled")
        def emit(event):
            events.append(event)
            if event["type"] == "run_settled":
                settled.set()
        session = AgentSession(SingleToolProvider(), ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, handler)]), on_cancel=stop, emit=emit)
        self.addCleanup(session.close)
        worker = Thread(target=lambda: results.append(session.run_turn("test")), daemon=True)
        worker.start()
        self.assertTrue(entered.wait(5))
        self.assertTrue(session.cancel().ok)
        self.assertTrue(stopped.is_set())
        worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results[0].status, "cancelled")
        self.assertEqual(session.snapshot()["status"], "draining")
        self.assertEqual(session.run_turn("new").error_code, "tool_still_running")
        self.assertFalse(settled.is_set())
        self.assertFalse(session.wait_for_idle(0))
        with self.assertRaises(ToolInterrupted):
            progress[0].progress({"phase": "late"})
        release.set()
        self.assertTrue(settled.wait(5))
        self.assertTrue(session.wait_for_idle(1))
        self.assertEqual(session.snapshot()["status"], "idle")
        self.assertEqual(session.snapshot()["history_messages"], 0)
        self.assertEqual([e["type"] for e in events].count("run_settled"), 1)

    def test_tool_deadline_cancels_harness_without_spawning_more_workers(self):
        entered, release, settled = Event(), Event(), Event()
        self.addCleanup(release.set)
        def handler(args, context):
            entered.set()
            release.wait(10)
            context.check_active()
            return ToolResult(True, "completed")
        session = AgentSession(SingleToolProvider(), ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, handler)]), timeout_s=.1,
                               emit=lambda e: settled.set() if e["type"] == "run_settled" else None)
        self.addCleanup(session.close)
        # Expire only after handler entry; scheduler load must not turn this into
        # a timeout before the tool starts. Do not modify global time or limits.
        clock = SimpleNamespace(now=time.monotonic())
        fake_time = SimpleNamespace(monotonic=lambda: clock.now)
        results = []
        with patch("eai_robot.policy.agents.loop.time", fake_time), patch("eai_robot.policy.agents.contracts.time", fake_time):
            worker = Thread(target=lambda: results.append(session.run_turn("test")), daemon=True)
            worker.start()
            self.assertTrue(entered.wait(5))
            clock.now += 1.
            worker.join(2)
            self.assertFalse(worker.is_alive())
            self.assertEqual(results[0].error_code, "tool_timeout")
            self.assertEqual(session.run_turn("new").error_code, "tool_still_running")
            release.set()
            self.assertTrue(settled.wait(5))

    def test_progress_is_scoped_to_tool_and_observer_errors_do_not_change_outcome(self):
        contexts, updates = [], []
        def handler(args, context):
            contexts.append(context)
            context.progress({"phase": "active"})
            return ToolResult(True, "completed")
        session = AgentSession(SingleToolProvider(), ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, handler)]), emit=updates.append)
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("test").status, "completed")
        before = len(updates)
        contexts[0].progress({"phase": "too late"})
        self.assertEqual(len(updates), before)
        def broken(event):
            raise RuntimeError("observer error")
        class TextProvider(DemoProvider):
            def complete(self, req, *, cancel):
                return ChatReply(req.request_id, req.session_id, req.generation, "done")
        other = AgentSession(TextProvider(), ToolRegistry([]), emit=broken)
        self.addCleanup(other.close)
        self.assertEqual(other.run_turn("test").status, "completed")
        self.assertGreater(other.snapshot()["observer_errors"], 0)

    def test_before_hook_can_reject_without_running_handler_and_after_observes(self):
        executed, observed = [], []
        def handler(args, context):
            executed.append(True)
            return ToolResult(True, "completed")
        session = AgentSession(SingleToolProvider(), ToolRegistry([Tool({"type": "function", "function": {"name": "work", "description": "work", "parameters": EMPTY}}, handler)]),
                               before_tool=lambda t, a, c: ToolResult(False, "rejected", error_code="condition_unmet"),
                               after_tool=lambda t, r, c: observed.append(r.error_code))
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("test").status, "completed")
        self.assertEqual(executed, [])
        self.assertEqual(observed, ["condition_unmet"])

    def test_cli_stream_and_stop_preserve_json_lines(self):
        gate, seen = Event(), Event()
        first = packet({"content": "准备中"})
        rest = packet({"content": "结束"}) + packet(reason="stop") + b'data: [DONE]\n\n'
        records, errors = [], []
        with streaming_service(lambda data, index: ([first, rest], gate)) as (url, _), tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "provider.toml"
            config.write_text('[provider]\nadapter="chat_completions"\nexecution="self_hosted"\n'
                              f'endpoint="{url}"\nmodel="fixture"\nstreaming=true\n', encoding="utf-8")
            proc = subprocess.Popen([sys.executable, "-m", "eai_robot.applications.agent.cli",
                                     "--provider-config", str(config), "--stdin"],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, encoding="utf-8", cwd=directory,
                                    env=dict(os.environ, PYTHONPATH=str(ROOT / "src"), PYTHONIOENCODING="utf-8"))
            def read():
                for line in proc.stdout:
                    try:
                        record = json.loads(line)
                        records.append(record)
                        if record.get("event", {}).get("type") == "model_delta":
                            seen.set()
                    except ValueError:
                        errors.append(line)
            reader = Thread(target=read, daemon=True)
            reader.start()
            try:
                proc.stdin.write("请演练\n")
                proc.stdin.flush()
                self.assertTrue(seen.wait(5))
                proc.stdin.write("停止\n")
                proc.stdin.flush()
                proc.stdin.close()
                proc.wait(timeout=5)
                reader.join(2)
                self.assertEqual(proc.returncode, 0, proc.stderr.read())
                self.assertEqual(errors, [])
                self.assertTrue(any(r.get("status") == "cancelled" for r in records))
                self.assertTrue(any(r.get("event", {}).get("type") == "run_settled" for r in records))
            finally:
                gate.set()
                if proc.poll() is None:
                    proc.kill()
                    proc.wait(timeout=5)
                reader.join(2)
                for stream in (proc.stdin, proc.stdout, proc.stderr):
                    stream.close()

    def test_hardware_limits_are_unknown_and_application_gates_capabilities(self):
        profile = RobotCapabilities(allowed_tools=("get_robot_state", "stop_motion"))
        self.assertTrue(all(value is None for value in profile.describe()["physical_limits"].values()))
        self.assertFalse(profile.describe()["physical_execution"])
        with self.assertRaises(ValueError):
            RobotCapabilities(physical_execution=True)
        skills = DryRunSkills({"stand": [.5] * 6})
        class Provider(DemoProvider):
            def complete(self, req, *, cancel):
                self.index += 1
                if self.index == 1:
                    self.allowed = [tool["function"]["name"] for tool in req.tools]
                    return ChatReply(req.request_id, req.session_id, req.generation,
                                     tool_calls=(ToolCall("x", "move_to_pose", {"pose_name": "stand"}),), finish_reason="tool_calls")
                return ChatReply(req.request_id, req.session_id, req.generation, "unavailable")
        provider = Provider()
        session = create_robot_session(provider, skills, capabilities=profile)
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("test").status, "completed")
        self.assertNotIn("move_to_pose", provider.allowed)
        self.assertEqual(skills.tasks, {})


if __name__ == "__main__":
    unittest.main()
