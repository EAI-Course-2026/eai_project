"""voice 控制台：识别文本 -> 白名单意图 -> 动作调度 -> 既有统一控制器。

默认干跑：不构造串口后端、不打开端口，只打印计划与目标，可用作审查和回归入口。

```sh
# 单条文本（干跑）
uv run --locked eai-course voice --text "挥手"
# 从管道读取识别结果（每条一行），对接 edge_inference_lab 的 mic_asr.py
mic_asr.py --mode live --emit text | uv run --locked eai-course voice --stdin
# 真机运动：--enable 且必须显式确认；同一串口只允许一个进程
uv run --locked eai-course voice --stdin --enable
```

运动前置条件与既有工具一致：GUI 与其他控制进程先断开、操作者支撑机械臂、串口独占。
运动模式在启用扭矩前要求交互确认，或对非交互场景显式传入
``--confirm "START VOICE CONTROL"``；任何缺失都直接拒绝开启，不退化成干跑。
"""
from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
from pathlib import Path

from eai_robot.calibration.servo.results import JOINTS, load as load_calibration
from eai_robot.control.planning.poses import POSES_PATH, load_poses
from eai_robot.hardware.robots.so101 import DEFAULT_CALIBRATION, default_port
from eai_robot.applications.teleoperation import intent as intent_module
from eai_robot.applications.teleoperation.dispatcher import Dispatcher

CONFIRM_PHRASE = "START VOICE CONTROL"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="voice",
        description="语音命令调度入口；默认干跑，不打开串口",
    )
    parser.add_argument("--text", help="单条识别文本，解析后立即结束")
    parser.add_argument("--stdin", action="store_true", help="从标准输入逐行读取识别文本")
    parser.add_argument("--poses", type=Path, default=POSES_PATH, help="姿态文件（默认仓库参考姿态）")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION, help="共享标定文件")
    parser.add_argument("--port", default=None, help="串口；默认读 configs/hardware.local.toml")
    parser.add_argument("--enable", action="store_true", help="开启运动（必须显式确认）")
    parser.add_argument("--confirm", default="", help=f"非交互确认短语：{CONFIRM_PHRASE}")
    parser.add_argument("--events", type=Path, default=None, help="事件 JSONL 输出路径（可选，便于留证据）")
    parser.add_argument("--step-pause", type=float, default=0.1, help="序列步骤间停顿秒数，0～5")
    parser.add_argument("--preview", action="store_true", help="干跑时额外打印姿态到原始目标的换算表")
    parser.add_argument("--list-commands", action="store_true", help="打印命令白名单后退出")
    return parser


def preview(poses: dict[str, list[float]], calibration, log=print) -> None:
    """打印姿态到原始目标的换算结果，用于干跑审查（与上位机预览口径一致）。"""
    for name, values in poses.items():
        targets = {calibration[joint].id: calibration[joint].position(value)
                   for joint, value in zip(JOINTS, values, strict=True)}
        log(f"[preview] {name}: {targets}")


def _motion_confirmed(args, stdin_isatty: bool) -> bool:
    if not stdin_isatty:
        return args.confirm.strip() == CONFIRM_PHRASE
    return input(f'输入 "{CONFIRM_PHRASE}" 开启扭矩：').strip() == CONFIRM_PHRASE


def _read_stdin(dispatcher: Dispatcher, intents: "queue.Queue[str]") -> None:
    """读线程：停止命令立即抢占，其余文本转交执行线程。"""
    for line in sys.stdin:
        text = line.strip()
        if not text:
            continue
        outcome = intent_module.parse(text)
        if outcome.intent is not None and outcome.intent.command == intent_module.STOP:
            dispatcher.stop("停止命令")
            continue
        intents.put(text)
    intents.put("")  # 结束标记


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_commands:
        for command in intent_module.COMMANDS:
            print(command)
        return 0
    if args.text is not None and args.stdin:
        print("--text 与 --stdin 不能同时使用", file=sys.stderr)
        return 2
    stdin_isatty = sys.stdin.isatty()
    streaming = args.stdin or (args.text is None and not stdin_isatty)
    if args.text is None and not streaming:
        print("需要 --text 或 --stdin（或通过管道提供文本）", file=sys.stderr)
        return 2

    calibration = load_calibration(args.calibration)
    poses = load_poses(args.poses, calibration)

    if not args.enable:
        dispatcher = Dispatcher(poses, dry_run=True, step_pause=args.step_pause)
        if args.preview:
            preview(poses, calibration)
        if args.text is not None:
            dispatcher.submit(args.text)
        else:
            intents: "queue.Queue[str]" = queue.Queue()
            thread = threading.Thread(target=_read_stdin, args=(dispatcher, intents), daemon=True)
            thread.start()
            while True:
                text = intents.get()
                if text == "":
                    break
                dispatcher.submit(text)
        _write_events(args.events, dispatcher)
        print("[voice] 干跑结束：未打开串口。加入 --enable 并确认后可真机运动。")
        return 0

    # ---- 运动模式：先确认，再取串口独占，最后才构造后端
    if not _motion_confirmed(args, stdin_isatty):
        print("未确认，拒绝开启运动。", file=sys.stderr)
        return 1
    port = args.port or default_port()
    if not port:
        print("未指定 --port，也没有 configs/hardware.local.toml", file=sys.stderr)
        return 1

    from eai_robot.hardware.drivers.backends import LeRobotBackend
    from eai_robot.control.execution.arm import ArmController
    from eai_robot.config import load_config
    from eai_robot.policy.execution.runner import PortLease

    lease = PortLease(port)
    backend = None
    arm = None
    dispatcher = Dispatcher(poses, dry_run=True, step_pause=args.step_pause)
    try:
        lease.acquire()
        backend = LeRobotBackend(port, load_config()["serial"]["baudrate"])
        arm = ArmController(backend, calibration, motion="smooth", velocity=80, timeout=8)
        arm.inspect()
        dispatcher.arm = arm
        dispatcher.dry_run = False
        arm.motion_guard = lambda: (_ for _ in ()).throw(RuntimeError("语音停止抢占")) \
            if dispatcher.stopped else None
        arm.enable()
        print(f"[voice] 控制已启用；命令白名单：{', '.join(intent_module.COMMANDS)}")
        print("[voice] 停止命令立即抢占并保持当前姿态；退出时关闭扭矩，请先支撑机械臂。")
        if args.text is not None:
            dispatcher.submit(args.text)
        else:
            intents = queue.Queue()
            thread = threading.Thread(target=_read_stdin, args=(dispatcher, intents), daemon=True)
            thread.start()
            while True:
                text = intents.get()
                if text == "":
                    break
                dispatcher.submit(text)
        return 0
    except KeyboardInterrupt:
        print("\n[voice] 操作者中断", file=sys.stderr)
        return 130
    finally:
        try:
            if arm is not None and arm.torque_touched:
                failures = arm.disable()
                if failures:
                    print(f"[voice] 关闭扭矩存在失败：{failures}", file=sys.stderr)
        finally:
            if backend is not None:
                backend.close()
            lease.release()
            _write_events(args.events, dispatcher)


def _write_events(path: Path | None, dispatcher: Dispatcher) -> None:
    for entry in dispatcher.journal:
        print("[event] " + json.dumps(entry, ensure_ascii=False), flush=True)
    if path is not None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            for entry in dispatcher.journal:
                stream.write(json.dumps(entry, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
