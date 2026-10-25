"""Provider/tool integration over real loopback HTTP; no model or robot access."""
from contextlib import contextmanager, redirect_stdout
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
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from eai_robot.policy.model_io.contracts import ChatReply, ChatRequest, Message, ProviderError, ToolCall
from eai_robot.policy.agents.contracts import ToolContext
from eai_robot.policy.agents.loop import AgentSession
from eai_robot.policy.model_io.providers import ChatCompletionsProvider, ProviderConfig
from eai_robot.applications.agent import cli as agent_cli
from eai_robot.applications.agent.cli import load_provider
from eai_robot.applications.agent.capabilities import HARDWARE_VISUAL_TOOLS, RobotCapabilities
from eai_robot.applications.agent.demo import DemoProvider
from eai_robot.applications.agent.hardware_visual import (
    HARDWARE_ACKNOWLEDGEMENT, ArucoCenterHardwareBackend, HardwareVisualSkills,
)
from eai_robot.applications.agent.session import is_stop, run_stdin
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
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
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
    @staticmethod
    def fake_hardware_backend():
        class Backend:
            def __init__(self):
                self.release_called = False
                self.value = {"mode": "aruco_center_hardware", "connected": False,
                              "tracking": "idle", "hardware_sent": False,
                              "motion_command_count": 0, "motion_confirmed_count": 0,
                              "torque_enabled": False, "requires_operator_release": False,
                              "error_code": None, "hold_reason": None}
            def snapshot(self):
                return dict(self.value)
            def detect(self):
                return {"target_type": "aruco marker", "label": "aruco:0",
                        "center_px": [320.0, 240.0],
                        "bbox_px": [300.0, 220.0, 40.0, 40.0],
                        "confidence": 1.0, "camera_observed": True,
                        "hardware_sent": False}
            def start(self, _):
                self.value.update(connected=True, tracking="running", hardware_sent=True,
                                  motion_command_count=1, motion_confirmed_count=1,
                                  torque_enabled=True, requires_operator_release=True)
                return self.snapshot()
            def stop(self):
                self.value.update(tracking="holding", hold_reason="operator_stop")
                return self.snapshot()
            def release(self):
                self.release_called = True
                self.value.update(connected=False, tracking="released", torque_enabled=False,
                                  requires_operator_release=False)
                return self.snapshot()
        return Backend()

    def test_hardware_visual_tools_wrap_backend_without_motor_values(self):
        backend = self.fake_hardware_backend()
        skills = HardwareVisualSkills(backend)
        registry = skills.registry(HARDWARE_VISUAL_TOOLS)
        context = tool_context()
        detection = registry.invoke(
            ToolCall("detect-hw", "detect_object", {"description": "ArUco marker"}), context)
        self.assertTrue(detection.data["camera_observed"])

        natural = registry.invoke(ToolCall(
            "detect-hw-natural", "detect_object",
            {"description": "the ArUco marker ID 0 in the camera view"}), context)
        self.assertTrue(natural.data["camera_observed"])
        conflicting = registry.invoke(ToolCall(
            "detect-hw-conflict", "detect_object", {"description": "ArUco marker ID 5"}), context)
        self.assertEqual(conflicting.error_code, "unsupported_description")
        started = registry.invoke(ToolCall(
            "start-hw", "start_tracking", {"object_id": detection.data["object_id"]}), context)
        self.assertEqual(started.status, "running")
        self.assertTrue(started.data["hardware_sent"])
        self.assertGreater(started.data["motion_confirmed_count"], 0)
        self.assertNotIn("joint_positions", started.data)
        status = registry.invoke(ToolCall(
            "status-hw", "get_task_status", {"task_id": started.data["task_id"]}), context)
        self.assertEqual(status.data["tracking"], "running")
        stopped = registry.invoke(ToolCall("stop-hw", "stop_tracking", {}), context)
        self.assertEqual(stopped.data["tracking"], "holding")
        self.assertTrue(stopped.data["requires_operator_release"])
        self.assertFalse(skills.release()["torque_enabled"])

        capabilities = RobotCapabilities(
            allowed_tools=HARDWARE_VISUAL_TOOLS, mode="aruco_center_hardware",
            physical_execution=True, evidence="operator-armed centre MVP")
        self.assertTrue(capabilities.describe()["physical_execution"])

    def test_hardware_cli_runs_tool_flow_and_requires_local_torque_release(self):
        backend = self.fake_hardware_backend()
        stdout = io.StringIO()
        argv = [
            "--provider-config", "fake-provider.toml", "--text",
            "检测 ArUco，开始追踪，查询状态，然后停止追踪",
            "--visual-hardware-profile", "fake-profile.toml",
            "--device-port", "COM_TEST", "--camera-index", "0",
            "--enable-visual-motion", "--acknowledge", HARDWARE_ACKNOWLEDGEMENT,
        ]
        with (patch.object(agent_cli, "ArucoCenterHardwareBackend", return_value=backend),
              patch.object(agent_cli, "load_provider", return_value=agent_cli.VisualDemoProvider()),
              patch.object(agent_cli.sys.stdin, "isatty", return_value=True),
              patch("builtins.input", return_value="RELEASE TORQUE"),
              redirect_stdout(stdout)):
            result = agent_cli.main(argv)
        self.assertEqual(result, 0)
        self.assertTrue(backend.release_called)
        self.assertFalse(backend.snapshot()["torque_enabled"])
        transcript = stdout.getvalue()
        self.assertIn('"mode": "aruco_center_hardware"', transcript)
        self.assertIn('"tool": "detect_object"', transcript)
        self.assertIn('"tool": "start_tracking"', transcript)
        self.assertIn('"tool": "stop_tracking"', transcript)
        self.assertIn('"motion_authorized": true', transcript)

        unreleased = self.fake_hardware_backend()
        unreleased.start(1)
        messages = []
        self.assertFalse(agent_cli.release_hardware(
            HardwareVisualSkills(unreleased), input_fn=lambda _: (_ for _ in ()).throw(EOFError()),
            output_fn=lambda message, **_: messages.append(message)))
        self.assertFalse(unreleased.release_called)
        self.assertTrue(unreleased.snapshot()["torque_enabled"])
        self.assertTrue(any("实体急停" in message for message in messages))

    def test_hardware_cli_waits_for_active_worker_without_sending_commands(self):
        class Backend:
            def __init__(self):
                self.states = iter(("running", "running", "holding"))
                self.current = "running"
            def snapshot(self):
                try:
                    self.current = next(self.states)
                except StopIteration:
                    pass
                return {"tracking": self.current, "hardware_sent": True,
                        "motion_confirmed_count": 1, "torque_enabled": True}
        skills = SimpleNamespace(backend=Backend())
        emitted = []
        final = agent_cli.wait_for_hardware_completion(
            skills, emitted.append, sleep_fn=lambda _: None)
        self.assertEqual(final["tracking"], "holding")
        self.assertEqual([item["hardware_monitor"]["tracking"] for item in emitted],
                         ["running", "holding"])

    def test_hardware_cli_rejects_missing_operator_ack_before_devices(self):
        argv = [
            "--provider-config", "fake-provider.toml", "--text", "追踪 ArUco",
            "--visual-hardware-profile", "fake-profile.toml",
            "--device-port", "COM_TEST", "--camera-index", "0",
            "--enable-visual-motion",
        ]
        with (patch.object(agent_cli, "ArucoCenterHardwareBackend") as backend,
              self.assertRaises(SystemExit)):
            agent_cli.main(argv)
        backend.assert_not_called()

    def test_hardware_backend_constructor_opens_no_devices(self):
        profile = ROOT / "configs" / "visual_servo" / "aruco_center_mvp.toml"
        shared = ROOT / "local_artifacts" / "tmp" / "agent-preview" / "latest.frame"
        backend = ArucoCenterHardwareBackend(
            profile, port="COM_DO_NOT_OPEN", camera_index=0, max_runtime_s=5,
            shared_frame=shared)
        self.assertEqual(backend.snapshot()["tracking"], "idle")
        self.assertFalse(backend.snapshot()["connected"])
        self.assertFalse(backend.snapshot()["hardware_sent"])
        self.assertEqual(backend.snapshot()["motion_command_count"], 0)
        self.assertEqual(backend.snapshot()["motion_confirmed_count"], 0)
        self.assertEqual(backend.snapshot()["frame_source"], "shared")
        self.assertEqual(backend._args(False).shared_frame, shared)

        backend._ready = Event()
        backend._motion_command_sent({"hidden_joint": 1})
        moving = backend.snapshot()
        self.assertEqual(moving["tracking"], "moving")
        self.assertTrue(moving["hardware_sent"])
        self.assertEqual(moving["motion_command_count"], 1)
        self.assertEqual(moving["motion_confirmed_count"], 0)
        self.assertFalse(backend._ready.is_set())
        backend._motion_confirmed({"hidden_joint": 1}, {"hidden_joint": 1})
        confirmed = backend.snapshot()
        self.assertEqual(confirmed["tracking"], "running")
        self.assertEqual(confirmed["motion_confirmed_count"], 1)
        self.assertTrue(backend._ready.is_set())

        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "changed.toml"
            changed.write_text(profile.read_text(encoding="utf-8").replace(
                "base_step_counts = 15", "base_step_counts = 14"), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unchanged validated"):
                ArucoCenterHardwareBackend(
                    changed, port="COM_DO_NOT_OPEN", camera_index=0, max_runtime_s=5)

    def test_local_stop_only_intercepts_standalone_stop_commands(self):
        self.assertTrue(is_stop("请停止当前任务。"))
        self.assertTrue(is_stop("急停！"))
        self.assertFalse(is_stop("检测 ArUco，开始追踪，查询状态后停止追踪。"))
        self.assertFalse(is_stop("停止追踪"))

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

    def test_http_loop_runs_visual_tracking_rehearsal_through_tool_results(self):
        skills = self.skills()

        def tool_data(message):
            return json.loads(message["content"])["data"]

        def responder(data):
            results = [message for message in data["messages"] if message["role"] == "tool"]
            if not results:
                call = ToolCall("visual-state", "get_robot_state", {})
            elif len(results) == 1:
                call = ToolCall("visual-detect", "detect_object", {"description": "ArUco marker"})
            elif len(results) == 2:
                call = ToolCall("visual-start", "start_tracking",
                                {"object_id": tool_data(results[-1])["object_id"]})
            elif len(results) == 3:
                call = ToolCall("visual-status", "get_task_status",
                                {"task_id": tool_data(results[-1])["task_id"]})
            elif len(results) == 4:
                call = ToolCall("visual-stop", "stop_tracking", {})
            else:
                return 200, answer("视觉追踪演练已停止，没有驱动硬件。")
            return 200, answer(calls=(call,))

        with service(responder) as (url, requests, _):
            provider = ChatCompletionsProvider(ProviderConfig(url, "fixture", "self_hosted"))
            session = AgentSession(provider, skills.registry())
            self.addCleanup(session.close)
            result = session.run_turn("找到 ArUco，开始追踪，查询状态后停止")

        self.assertEqual(result.status, "completed")
        self.assertEqual(len(requests), 6)
        self.assertEqual(result.text, "视觉追踪演练已停止，没有驱动硬件。")
        tracking = next(task for task in skills.tasks.values() if task["task_type"] == "tracking")
        self.assertEqual(tracking["state"], "cancelled")
        self.assertFalse(tracking["hardware_sent"])

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
            child_env = dict(os.environ, PYTHONPATH=str(ROOT / "src"),
                             PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
            result = subprocess.run([sys.executable, "-m", "eai_robot.applications.agent.cli",
                                     "--provider-config", str(config), "--text", "演练站立"],
                                    cwd=directory, env=child_env,
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

    def test_visual_tracking_dry_run_lifecycle_and_session_bound_object(self):
        skills = self.skills()
        registry = skills.registry()
        context = tool_context()

        unsupported = registry.invoke(
            ToolCall("bad-description", "detect_object", {"description": "red cup"}), context)
        self.assertFalse(unsupported.ok)
        self.assertFalse(unsupported.data["found"])
        self.assertEqual(unsupported.error_code, "unsupported_description")

        detection = registry.invoke(
            ToolCall("detect", "detect_object", {"description": "ArUco 标记"}), context)
        self.assertTrue(detection.data["found"])
        self.assertFalse(detection.data["hardware_sent"])
        object_id = detection.data["object_id"]

        other_context = ToolContext("other-session", 0, time.monotonic() + 5, Event(),
                                    lambda: True, lambda _: None)
        cross_session = registry.invoke(
            ToolCall("cross", "start_tracking", {"object_id": object_id}), other_context)
        self.assertEqual(cross_session.error_code, "unknown_object")

        started = registry.invoke(
            ToolCall("start", "start_tracking", {"object_id": object_id}), context)
        task_id = started.data["task_id"]
        self.assertEqual(started.data["state"], "running")
        self.assertEqual(skills.state({}).data["tracking"], "running")

        busy = registry.invoke(
            ToolCall("busy", "start_tracking", {"object_id": object_id}), context)
        self.assertEqual(busy.error_code, "tracking_busy")
        status = registry.invoke(
            ToolCall("status", "get_task_status", {"task_id": task_id}), context)
        self.assertEqual(status.data["state"], "running")

        stopped = registry.invoke(ToolCall("stop", "stop_tracking", {}), context)
        self.assertEqual(stopped.data["stopped_task_id"], task_id)
        self.assertEqual(skills.task({"task_id": task_id}).data["state"], "cancelled")
        self.assertEqual(skills.state({}).data["tracking"], "idle")
        repeated = registry.invoke(ToolCall("stop-again", "stop_tracking", {}), context)
        self.assertIsNone(repeated.data["stopped_task_id"])

        restarted = registry.invoke(
            ToolCall("restart", "start_tracking", {"object_id": object_id}), context)
        global_stop = skills.stop({}, context)
        self.assertEqual(global_stop.data["stopped_tracking_task_id"], restarted.data["task_id"])
        self.assertEqual(skills.task({"task_id": restarted.data["task_id"]}).data["state"], "cancelled")

    def test_visual_tracking_rejects_stale_object(self):
        now = [100.0]
        skills = DryRunSkills({"stand": [.5] * 6}, clock=lambda: now[0], object_ttl_s=1.0)
        registry = skills.registry()
        context = tool_context()
        detection = registry.invoke(
            ToolCall("detect", "detect_object", {"description": "aruco"}), context)
        now[0] = 101.01
        result = registry.invoke(ToolCall("start", "start_tracking",
                                         {"object_id": detection.data["object_id"]}), context)
        self.assertEqual(result.error_code, "stale_object")
        self.assertEqual(skills.tasks, {})

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

    def test_visual_demo_cli_runs_detect_track_status_and_stop_without_devices(self):
        code = '''
import sys
from eai_robot.applications.agent.cli import main
assert not any(n.startswith(('torch', 'serial', 'lerobot', 'cv2', 'pynput')) for n in sys.modules)
raise SystemExit(main(['--visual-demo']))
'''
        with tempfile.TemporaryDirectory() as directory:
            env = dict(os.environ, PYTHONPATH=str(ROOT / "src"), PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
            result = subprocess.run([sys.executable, "-c", code], cwd=directory,
                                    env=env, capture_output=True, text=True, timeout=15, encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('"tool": "detect_object"', result.stdout)
        self.assertIn('"tool": "start_tracking"', result.stdout)
        self.assertIn('"tool": "stop_tracking"', result.stdout)
        self.assertIn('"status": "completed"', result.stdout)
        self.assertIn('"hardware_connected": false', result.stdout)

    def test_closed_session_and_invalid_text_do_not_call_provider(self):
        session = AgentSession(DemoProvider(), self.skills().registry())
        self.assertEqual(session.run_turn("").error_code, "invalid_text")
        session.close()
        self.assertEqual(session.run_turn("test").error_code, "closed")


if __name__ == "__main__":
    unittest.main()
