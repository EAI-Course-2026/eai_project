"""离线回归：语音白名单解析、动作计划、stop 抢占与干跑边界（不打开硬件）。"""
import io
import json
import sys
import tempfile
import threading
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from eai_robot.arm.calibration import load as load_calibration  # noqa: E402
from eai_robot.course.poses.control_presets import POSES_PATH, load_poses  # noqa: E402
from eai_robot.course.robot import DEFAULT_CALIBRATION  # noqa: E402
from eai_robot.voice import cli as voice_cli  # noqa: E402
from eai_robot.voice import intent as voice_intent  # noqa: E402
from eai_robot.voice.dispatcher import Dispatcher, POSE_SEQUENCES, build_plan  # noqa: E402

POSES = load_poses(POSES_PATH, load_calibration(DEFAULT_CALIBRATION))


class IntentTests(unittest.TestCase):
    def test_whitelist_is_fixed(self):
        self.assertEqual(
            voice_intent.COMMANDS,
            ("stand", "wave", "move_left", "move_right", "open_gripper", "close_gripper", "stop"),
        )
        self.assertEqual(set(POSE_SEQUENCES), set(voice_intent.COMMANDS))

    def test_chinese_aliases_with_punctuation(self):
        for text, command in [
            ("挥手。", "wave"),
            ("向左！", "move_left"),
            ("向右", "move_right"),
            ("站立。", "stand"),
            ("归位", "stand"),
            ("抓取", "close_gripper"),
            ("张开夹爪", "open_gripper"),
            ("停止", "stop"),
        ]:
            with self.subTest(text=text):
                outcome = voice_intent.parse(text)
                self.assertTrue(outcome.accepted, outcome.reason)
                self.assertEqual(outcome.intent.command, command)

    def test_english_keywords_need_exact_match(self):
        self.assertEqual(voice_intent.parse("stop").intent.command, "stop")
        self.assertEqual(voice_intent.parse("Stop!").intent.command, "stop")
        # 整句英文不能命中动作命令，避免 "open the door" 之类误触发夹爪。
        self.assertFalse(voice_intent.parse("open the door").accepted)
        self.assertFalse(voice_intent.parse("let's go left now").accepted)
        self.assertEqual(voice_intent.parse("open").intent.command, "open_gripper")

    def test_stop_preempts_other_keywords(self):
        outcome = voice_intent.parse("停止挥手")
        self.assertTrue(outcome.accepted)
        self.assertEqual(outcome.intent.command, "stop")

    def test_negation_is_refused(self):
        outcome = voice_intent.parse("不要向左")
        self.assertFalse(outcome.accepted)
        self.assertIn("否定词", outcome.reason)

    def test_longest_keyword_wins(self):
        outcome = voice_intent.parse("请转到左边")
        self.assertEqual(outcome.intent.command, "move_left")
        self.assertEqual(outcome.intent.matched, "转到左边")

    def test_empty_and_unknown_are_refused(self):
        for text in ["", "   ", "。", "今天天气不错", "播放音乐"]:
            with self.subTest(text=text):
                outcome = voice_intent.parse(text)
                self.assertFalse(outcome.accepted)
                self.assertTrue(outcome.reason)

    def test_non_string_rejected(self):
        with self.assertRaises(TypeError):
            voice_intent.parse(None)


class PlanningTests(unittest.TestCase):
    def test_wave_sequence_matches_existing_pose_control(self):
        plan = build_plan(voice_intent.parse("挥手").intent, POSES)
        self.assertTrue(plan.complete)
        self.assertEqual([step.pose for step in plan.steps], ["raise", "left", "right", "left", "stand"])

    def test_missing_pose_blocks_whole_plan(self):
        poses = {name: values for name, values in POSES.items() if name != "left"}
        plan = build_plan(voice_intent.parse("挥手").intent, poses)
        self.assertFalse(plan.complete)
        self.assertIn("left", plan.missing)

    def test_gripper_poses_absent_by_default(self):
        # 参考姿态文件尚未包含夹爪姿态：命令被接受但整体拒绝执行，不产生运动。
        plan = build_plan(voice_intent.parse("抓取").intent, POSES)
        self.assertFalse(plan.complete)
        self.assertEqual(plan.missing, ("gripper_close",))


class DispatcherTests(unittest.TestCase):
    def setUp(self):
        self.lines = []
        self.dispatcher = Dispatcher(POSES, dry_run=True, log=self.lines.append, step_pause=0.0)

    def test_dry_run_never_touches_arm(self):
        result = self.dispatcher.submit("挥手")
        self.assertTrue(result.accepted)
        self.assertEqual(result.executed, 5)
        self.assertEqual(self.dispatcher.arm, None)
        self.assertTrue(all(entry["dry_run"] for entry in self.dispatcher.journal))

    def test_unknown_text_produces_no_action(self):
        result = self.dispatcher.submit("打开窗户")
        self.assertFalse(result.accepted)
        self.assertEqual(result.executed, 0)
        self.assertEqual(self.dispatcher.journal[-1]["command"], None)

    def test_missing_pose_is_refused_not_partially_executed(self):
        result = self.dispatcher.submit("抓取")
        self.assertFalse(result.accepted)
        self.assertEqual(result.executed, 0)
        self.assertIn("gripper_close", result.reason)

    def test_stop_preempts_running_sequence(self):
        # step_pause=0.05 让序列持续足够长，便于在中间插入 stop。
        dispatcher = Dispatcher(POSES, dry_run=True, log=self.lines.append, step_pause=0.05)
        results = []

        def run():
            results.append(dispatcher.submit("挥手"))

        worker = threading.Thread(target=run)
        worker.start()
        deadline = time.monotonic() + 2
        while not dispatcher.busy and time.monotonic() < deadline:
            time.sleep(0.005)
        dispatcher.stop("测试抢占")
        worker.join(timeout=5)
        self.assertFalse(worker.is_alive())
        self.assertTrue(results and results[0].accepted)
        self.assertLess(results[0].executed, 5)
        self.assertEqual(results[0].reason, "被停止命令抢占")

    def test_stop_during_sequence_clears_pending_and_allows_next_command(self):
        dispatcher = Dispatcher(POSES, dry_run=True, log=self.lines.append, step_pause=0.05)
        results = []
        worker = threading.Thread(target=lambda: results.append(dispatcher.submit("挥手")))
        worker.start()
        deadline = time.monotonic() + 2
        while not dispatcher.busy and time.monotonic() < deadline:
            time.sleep(0.005)
        dispatcher.stop("测试抢占")
        worker.join(timeout=5)
        # 停止后新的普通命令重新开放调度。
        follow_up = dispatcher.submit("站立")
        self.assertTrue(follow_up.accepted)
        self.assertEqual(follow_up.executed, 1)

    def test_concurrent_command_is_refused_while_busy(self):
        dispatcher = Dispatcher(POSES, dry_run=True, log=self.lines.append, step_pause=0.05)
        results = []
        worker = threading.Thread(target=lambda: results.append(dispatcher.submit("挥手")))
        worker.start()
        deadline = time.monotonic() + 2
        while not dispatcher.busy and time.monotonic() < deadline:
            time.sleep(0.005)
        second = dispatcher.submit("站立")
        self.assertFalse(second.accepted)
        self.assertIn("并发", second.reason)
        worker.join(timeout=5)

    def test_motion_mode_requires_arm(self):
        with self.assertRaises(ValueError):
            Dispatcher(POSES, dry_run=False)


class CliTests(unittest.TestCase):
    def run_cli(self, argv, stdin_text=None):
        stdout = io.StringIO()
        original = sys.stdin
        if stdin_text is not None:
            sys.stdin = io.StringIO(stdin_text)
            sys.stdin.isatty = lambda: False  # type: ignore[attr-defined]
        try:
            with redirect_stdout(stdout):
                code = voice_cli.main(argv)
        finally:
            sys.stdin = original
        return code, stdout.getvalue()

    def test_dry_run_text_command(self):
        code, out = self.run_cli(["--text", "挥手"])
        self.assertEqual(code, 0)
        self.assertIn("raise -> left -> right -> left -> stand", out)
        self.assertIn("未打开串口", out)

    def test_dry_run_unknown_text(self):
        code, out = self.run_cli(["--text", "帮我倒杯水"])
        self.assertEqual(code, 0)
        self.assertIn("不在命令白名单内", out)

    def test_preview_lists_raw_targets_without_serial(self):
        code, out = self.run_cli(["--text", "站立", "--preview"])
        self.assertEqual(code, 0)
        self.assertIn("[preview] stand", out)

    def test_stdin_stream_with_stop(self):
        code, out = self.run_cli(["--stdin"], stdin_text="挥手\n停止\n站立\n")
        self.assertEqual(code, 0)
        self.assertIn("停止命令", out)
        self.assertIn("站立", out)

    def test_text_and_stdin_conflict(self):
        code, _ = self.run_cli(["--text", "站立", "--stdin"], stdin_text="")
        self.assertEqual(code, 2)

    def test_motion_requires_confirmation(self):
        code, err_out = self.run_cli(["--stdin", "--enable", "--port", "/dev/null"], stdin_text="站立\n")
        self.assertEqual(code, 1)

    def test_list_commands(self):
        code, out = self.run_cli(["--list-commands"])
        self.assertEqual(code, 0)
        self.assertEqual(out.split(), list(voice_intent.COMMANDS))

    def test_events_jsonl_is_written(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "voice.jsonl"
            code, _ = self.run_cli(["--text", "挥手", "--events", str(path)])
            self.assertEqual(code, 0)
            entries = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(entries)
            self.assertEqual(entries[0]["command"], "wave")
            self.assertTrue(entries[0]["dry_run"])


class ModuleBoundaryTests(unittest.TestCase):
    def test_intent_and_dispatcher_do_not_import_hardware_modules(self):
        source = "\n".join(
            (ROOT / "src/eai_robot/voice" / name).read_text(encoding="utf-8")
            for name in ("intent.py", "dispatcher.py"))
        for forbidden in ("import serial", "pyserial", "from lerobot", "import lerobot"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
