"""Provider/tool integration over real loopback HTTP; no model or robot access."""
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from threading import Event, Thread
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from eai_robot.policy.model_io.contracts import ChatReply, ChatRequest, Message, ProviderError, ToolCall
from eai_robot.policy.agents.contracts import ToolContext
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.applications.agent.cli import load_provider
from eai_robot.applications.agent.demo import DemoProvider
from eai_robot.applications.agent.session import run_stdin
from eai_robot.applications.agent.skills import DryRunSkills


def answer(text="done", calls=(), reason=None):
    return {"id": "fixture", "object": "chat.completion", "created": 0, "model": "fixture", "choices": [{"index": 0, "message": {"role": "assistant", "content": text,
                          "tool_calls": [{"id": c.id, "type": "function", "function": {
                              "name": c.name, "arguments": json.dumps(c.arguments)}} for c in calls]},
                          "finish_reason": reason or ("tool_calls" if calls else "stop")}]}


@contextmanager
def service(responder):
    requests = []
    headers = []
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def do_POST(self):
            data = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            requests.append(data)
            headers.append(dict(self.headers))
            status, body = responder(data)
            raw = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Length", str(len(raw)))
            if status == 302:
                self.send_header("Location", "https://example.invalid/never-follow")
            self.end_headers()
            try:
                self.wfile.write(raw)
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    server.daemon_threads = True
    thread = Thread(target=server.serve_forever, kwargs={"poll_interval": .01}, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}/v1/chat/completions", requests, headers
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)


def tool_context():
    return ToolContext("test", 0, time.monotonic() + 5, Event(), lambda: True, lambda _: None)


def request(timeout=10):
    # Ordinary protocol fixtures include SDK cold import/SSL initialization.
    # Deadline tests set explicit short provider/request budgets independently.
    return ChatRequest("r", "s", 0, ({"role": "user", "content": "test"},), (), timeout)


class AgentTests(unittest.TestCase):
    def skills(self):
        return DryRunSkills({"stand": [.5] * 6, "left": [.4] * 6})

    def test_http_loop_returns_tool_results_to_model(self):
        skills = self.skills()
        def responder(data):
            count = len([m for m in data["messages"] if m["role"] == "tool"])
            calls = [ToolCall("a", "get_robot_state", {}),
                     ToolCall("b", "list_poses", {}),
                     ToolCall("c", "move_to_pose", {"pose_name": "left"})]
            return 200, answer(calls=(calls[count],)) if count < 3 else answer("simulated only")
        with service(responder) as (url, requests, headers), patch.dict(os.environ, {"AGENT_TEST_KEY": "secret-for-test"}):
            provider = ChatCompletionsProvider(ProviderConfig(url, "test-model", "self_hosted", "AGENT_TEST_KEY"))
            events = []
            session = AgentSession(provider, skills.registry(), emit=events.append)
            self.addCleanup(session.close)
            result = session.run_turn("go left")
            self.assertEqual(result.status, "completed")
            self.assertEqual(len(requests), 4)
            self.assertEqual(len(skills.tasks), 1)
            tools = [m for m in requests[-1]["messages"] if m["role"] == "tool"]
            payload = json.loads(tools[-1]["content"])
            self.assertEqual(payload["tool_call_id"], "c")
            self.assertFalse(payload["data"]["hardware_sent"])
            self.assertEqual(payload["data"]["units"], "normalized_0_1")
            self.assertEqual(headers[0]["Authorization"], "Bearer secret-for-test")
            self.assertNotIn("secret-for-test", json.dumps(events) + json.dumps(provider.describe()))

    def test_cli_uses_user_config_from_an_unrelated_directory(self):
        def responder(data):
            count = sum(m["role"] == "tool" for m in data["messages"])
            calls = [ToolCall("cli-state", "get_robot_state", {}),
                     ToolCall("cli-pose", "move_to_pose", {"pose_name": "stand"})]
            return 200, answer(calls=(calls[count],)) if count < 2 else answer("演练完成")
        with service(responder) as (url, requests, _), tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "user-choice.toml"
            config.write_text('[provider]\nadapter="chat_completions"\nexecution="self_hosted"\n'
                              f'endpoint="{url}"\nmodel="fixture-model"\n', encoding="utf-8")
            result = subprocess.run([sys.executable, "-m", "eai_robot.applications.agent.cli",
                                     "--provider-config", str(config), "--text", "演练站立"],
                                    cwd=directory, env=dict(os.environ, PYTHONPATH=str(ROOT / "src")),
                                    capture_output=True, text=True, timeout=15, encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(len(requests), 3)
        self.assertIn('"status": "completed"', result.stdout)
        self.assertIn('"hardware_connected": false', result.stdout)

    def test_malformed_incomplete_and_duplicate_json_are_rejected(self):
        values = [b'{"choices":[],"choices":[]}', {"choices": []},
                  answer(reason="length"), answer(text=["not text"]),
                  b'{"choices":NaN}', b'not json',
                  {"choices": [{"message": {"role": "assistant", "tool_calls": [{
                      "id": "x", "type": "function", "function": {"name": "move_to_pose", "arguments": "[]"}}]},
                      "finish_reason": "tool_calls"}]}]
        current = [None]
        with service(lambda _: (200, current[0])) as (url, _, _):
            for value in values:
                with self.subTest(value=value):
                    current[0] = value
                    provider = ChatCompletionsProvider(ProviderConfig(url, "m", "self_hosted"))
                    self.addCleanup(provider.close)
                    with self.assertRaisesRegex(ProviderError, "protocol_error"):
                        provider.complete(request(), cancel=Event())

    def test_authentication_and_redirects_do_not_expose_response_or_key(self):
        for status, expected in ((401, "authentication"), (429, "rate_limited"), (302, "redirect_refused")):
            with self.subTest(status=status), service(lambda _: (status, b'SECRET RESPONSE')) as (url, _, _):
                provider = ChatCompletionsProvider(ProviderConfig(url, "m", "self_hosted"))
                self.addCleanup(provider.close)
                with self.assertRaises(ProviderError) as caught:
                    provider.complete(request(), cancel=Event())
                self.assertEqual(str(caught.exception), expected)

    def test_config_validation_and_credential_reference(self):
        for url in ("https://user:secret@host/x", "https://host/x?key=secret", "http://remote.example/x", "file:///tmp/x"):
            with self.subTest(url=url), self.assertRaises(ValueError):
                ProviderConfig(url, "m", "self_hosted")
        with self.assertRaises(ValueError):
            ProviderConfig("http://127.0.0.1/x", "m", "managed_api")
        with patch.dict(os.environ, {}, clear=True), self.assertRaisesRegex(ProviderError, "missing_credential"):
            ChatCompletionsProvider(ProviderConfig("https://example.invalid/x", "m", "managed_api", "ABSENT_KEY"))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "provider.toml"
            path.write_text('[provider]\nadapter="chat_completions"\nexecution="managed_api"\nendpoint="https://example.invalid/x"\nmodel="m"\napi_key="NEVER_INLINE"\n')
            with self.assertRaises(ValueError) as caught:
                load_provider(path)
            self.assertNotIn("NEVER_INLINE", str(caught.exception))

    def test_unknown_tool_and_illegal_pose_cannot_plan(self):
        skills = self.skills()
        registry = skills.registry()
        for name, args, code in (("shell", {}, "unknown_tool"),
                                 ("move_to_pose", {"pose_name": "unknown"}, "invalid_arguments"),
                                 ("move_to_pose", {"pose_name": "stand", "enable": "true"}, "invalid_arguments"),
                                 ("move_to_pose", {"pose_name": 12}, "invalid_arguments")):
            result = registry.invoke(ToolCall("x", name, args), tool_context())
            self.assertEqual(result.error_code, code)
        self.assertEqual(skills.tasks, {})
        self.assertIsNone(skills.state({}).data["joint_positions"])

    def test_schema_copy_cannot_expand_allowlist(self):
        registry = self.skills().registry()
        definitions = registry.definitions
        definitions[2]["function"]["parameters"]["properties"]["pose_name"]["enum"].append("evil")
        result = registry.invoke(ToolCall("x", "move_to_pose", {"pose_name": "evil"}), tool_context())
        self.assertEqual(result.error_code, "invalid_arguments")

    def test_duplicate_ids_and_budgets_do_not_repeat_actions(self):
        for mode, expected_count, code in (("duplicate", 1, "duplicate_tool_call"),
                                            ("rounds", 1, "round_budget"),
                                            ("calls", 0, "call_budget")):
            skills = self.skills()
            class Provider(DemoProvider):
                def complete(self, req, *, cancel):
                    self.index += 1
                    calls = (ToolCall("same" if mode == "duplicate" else str(self.index),
                                      "move_to_pose", {"pose_name": "stand"}),)
                    if mode == "calls":
                        calls += (ToolCall("extra", "get_robot_state", {}),)
                    return ChatReply(req.request_id, req.session_id, req.generation, tool_calls=calls, finish_reason="tool_calls")
            session = AgentSession(Provider(), skills.registry(), max_rounds=1 if mode == "rounds" else 3, max_calls=1 if mode == "calls" else 16)
            self.addCleanup(session.close)
            result = session.run_turn("test")
            self.assertEqual(result.error_code, code)
            self.assertEqual(len(skills.tasks), expected_count)

    def test_batch_stop_preempts_an_earlier_move(self):
        skills = self.skills()
        class Provider(DemoProvider):
            def complete(self, req, *, cancel):
                return ChatReply(req.request_id, req.session_id, req.generation, tool_calls=(
                    ToolCall("move", "move_to_pose", {"pose_name": "stand"}),
                    ToolCall("stop", "stop_motion", {})), finish_reason="tool_calls")
        session = AgentSession(Provider(), skills.registry())
        self.addCleanup(session.close)
        self.assertEqual(session.run_turn("stop").status, "cancelled")
        self.assertEqual(skills.tasks, {})

    def test_identity_mismatch_and_late_response_cannot_plan(self):
        for mode in ("identity", "late"):
            skills = self.skills()
            entered, release = Event(), Event()
            class Provider(DemoProvider):
                def complete(self, req, *, cancel):
                    entered.set()
                    release.wait(2)
                    return ChatReply(req.request_id, req.session_id,
                                     req.generation + (mode == "identity"), tool_calls=(
                                         ToolCall("x", "move_to_pose", {"pose_name": "stand"}),), finish_reason="tool_calls")
            session = AgentSession(Provider(), skills.registry())
            self.addCleanup(session.close)
            results = []
            thread = Thread(target=lambda: results.append(session.run_turn("test")), daemon=True)
            thread.start()
            self.assertTrue(entered.wait(2))
            if mode == "late":
                session.cancel()
            release.set()
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(skills.tasks, {})
            self.assertEqual(results[0].status, "cancelled" if mode == "late" else "failed")

    def test_http_cancel_is_fast_and_limits_abandoned_workers(self):
        entered, release = Event(), Event()
        def respond(_):
            entered.set()
            release.wait(10)
            return 200, answer()
        with service(respond) as (url, _, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "m", "self_hosted", timeout_s=10))
            self.addCleanup(provider.close)
            self.addCleanup(release.set)
            cancel = Event()
            errors = []
            def run():
                try:
                    provider.complete(request(timeout=10), cancel=cancel)
                except ProviderError as exc:
                    errors.append(exc.code)
            thread = Thread(target=run, daemon=True)
            thread.start()
            self.assertTrue(entered.wait(2))
            cancel.set()
            thread.join(2)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, ["cancelled"])
            with self.assertRaisesRegex(ProviderError, "request_still_in_flight"):
                provider.complete(request(), cancel=Event())
            release.set()

    def test_timeout_and_response_size(self):
        with service(lambda _: (time.sleep(.2) or 200, answer())) as (url, _, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "m", "self_hosted", timeout_s=.1))
            self.addCleanup(provider.close)
            with self.assertRaisesRegex(ProviderError, "timeout"):
                provider.complete(request(), cancel=Event())
        with service(lambda _: (200, b'x' * (1024 * 1024 + 1))) as (url, _, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "m", "self_hosted"))
            self.addCleanup(provider.close)
            with self.assertRaisesRegex(ProviderError, "response_too_large"):
                provider.complete(request(), cancel=Event())

    def test_stdin_stop_installed_before_next_line_and_no_queue_replay(self):
        skills = self.skills()
        entered, release = Event(), Event()
        class Provider(DemoProvider):
            def complete(self, req, *, cancel):
                entered.set()
                cancel.wait(2)
                return ChatReply(req.request_id, req.session_id, req.generation, tool_calls=(
                    ToolCall("x", "move_to_pose", {"pose_name": "stand"}),), finish_reason="tool_calls")
        session = AgentSession(Provider(), skills.registry())
        self.addCleanup(session.close)
        events = []
        self.assertEqual(run_stdin(io.StringIO("站立\n向左\n停止\n"), session, events.append), 0)
        self.assertEqual(skills.tasks, {})
        self.assertTrue(any(event.get("error_code") == "busy" for event in events))
        self.assertTrue(any(event.get("status") == "cancelled" for event in events))

    def test_stdin_interrupt_cancels_before_joining_inference(self):
        skills = self.skills()
        observed, entered = Event(), Event()
        class Provider(DemoProvider):
            def complete(self, req, *, cancel):
                entered.set()
                if cancel.wait(5):
                    observed.set()
                return ChatReply(req.request_id, req.session_id, req.generation, "cancelled")
        session = AgentSession(Provider(), skills.registry())
        self.addCleanup(session.close)
        def interrupted_input():
            yield "站立"
            self.assertTrue(entered.wait(2))
            raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            run_stdin(interrupted_input(), session, lambda _: None)
        # The provider sees cancellation, not just eventual network completion.
        self.assertTrue(observed.is_set())
        self.assertEqual(skills.tasks, {})

    def test_import_and_offline_cli_do_not_need_models_hardware_or_external_checkout(self):
        code = '''
import sys
from eai_robot.applications.agent.cli import main
assert not any(n.startswith(('torch', 'serial', 'lerobot', 'cv2', 'pynput')) for n in sys.modules)
raise SystemExit(main(['--demo']))
'''
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, PYTHONPATH=str(ROOT / "src"))
            result = subprocess.run([sys.executable, "-c", code], cwd=directory,
                                    env=env, capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"status": "completed"', result.stdout)
        self.assertIn('"hardware_connected": false', result.stdout)

    def test_closed_session_and_invalid_text_do_not_call_provider(self):
        session = AgentSession(DemoProvider(), self.skills().registry())
        self.assertEqual(session.run_turn("").error_code, "invalid_text")
        session.close()
        self.assertEqual(session.run_turn("test").error_code, "closed")


if __name__ == "__main__":
    unittest.main()
