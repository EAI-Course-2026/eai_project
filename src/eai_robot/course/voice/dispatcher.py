"""动作调度：白名单意图映射为姿态序列，交给既有统一控制器执行。

设计约束（与 docs/policy/README.md 的 fail-closed 语义一致）：

- 默认干跑：不构造后端、不打开串口，只打印将要写入的目标。
- 只有整条姿态序列都能在姿态文件中解析时才允许运动；缺任何一个姿态都整体拒绝，
  绝不执行“半条”序列。
- ``stop`` 在任意时刻生效：置停止标志、丢弃剩余步骤与挂起队列；为保持当前姿态，
  停止不会关闭扭矩（关闭扭矩需先支撑机械臂，见 README 的释放约定）。
- 序列执行期间不接受新的普通命令（避免意想不到的连续运动），只接受 ``stop``。
- 复用 ``eai_robot.course.poses.control_presets.load_poses`` 做姿态文件校验与
  标定换算：姿态文件必须携带来源标定，换算后的原始目标必须落在当前标定范围内。

本模块不导入串口；运动模式下的 ``arm`` 由调用方注入 ``ArmController``。
"""
from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time

from .intent import Intent, parse

#: 意图到姿态序列的固定映射。姿态名来自姿态文件（默认 poses.reference.json）。
POSE_SEQUENCES: dict[str, tuple[str, ...]] = {
    "stand": ("stand",),
    "wave": ("raise", "left", "right", "left", "stand"),
    "move_left": ("left",),
    "move_right": ("right",),
    "open_gripper": ("gripper_open",),
    "close_gripper": ("gripper_close",),
    "stop": (),
}


@dataclass(frozen=True)
class Step:
    pose: str
    ratios: tuple[float, ...]


@dataclass(frozen=True)
class Plan:
    command: str
    steps: tuple[Step, ...]
    missing: tuple[str, ...]

    @property
    def complete(self) -> bool:
        return not self.missing


@dataclass
class Result:
    accepted: bool
    command: str | None = None
    planned: int = 0
    executed: int = 0
    reason: str = ""


@dataclass
class Dispatcher:
    """串行执行动作计划；``stop`` 可从其他线程调用以抢占。"""

    poses: dict[str, list[float]]
    arm: object | None = None
    dry_run: bool = True
    log: object = print
    journal: list = field(default_factory=list)
    step_pause: float = 0.1
    clock: object = time.perf_counter

    def __post_init__(self) -> None:
        if type(self.dry_run) is not bool:
            raise TypeError("dry_run must be a bool")
        if not self.dry_run and self.arm is None:
            raise ValueError("运动模式需要注入已启用的控制器；否则保持干跑")
        if self.step_pause < 0 or self.step_pause > 5:
            raise ValueError("step_pause 必须在 0～5 秒之间")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._busy = False

    # ---------------------------------------------------------------- 查询
    @property
    def stopped(self) -> bool:
        return self._stop.is_set()

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    # ---------------------------------------------------------------- 停止
    def stop(self, reason: str = "operator stop") -> bool:
        """抢占当前动作：置停止标志并丢弃剩余步骤。返回是否原本有待执行的步骤。"""
        was_busy = self.busy
        self._stop.set()
        self._record(command="stop", text="", matched="", planned=0, executed=0, status=reason)
        self.log(f"[voice] {reason}：已取消剩余步骤（保持当前姿态，未关闭扭矩）")
        return was_busy

    # ---------------------------------------------------------------- 提交
    def submit(self, text: str) -> Result:
        outcome = parse(text)
        if outcome.intent is None:
            self.log(f"[voice] 忽略：{outcome.reason}")
            self._record(command=None, text=text, matched="", planned=0, executed=0, status=outcome.reason)
            return Result(False, reason=outcome.reason)
        return self.dispatch(outcome.intent)

    def dispatch(self, intent: Intent) -> Result:
        if intent.command == "stop":
            self.stop("停止命令")
            return Result(True, command="stop", reason="停止命令")
        with self._lock:
            if self._busy:
                reason = "正在执行上一条命令，拒绝并发动作"
                self.log(f"[voice] 忽略：{reason}")
                self._record(command=intent.command, text=intent.text, matched=intent.matched,
                             planned=0, executed=0, status=reason)
                return Result(False, command=intent.command, reason=reason)
            self._busy = True
            self._stop.clear()  # 新的普通命令重新开放调度（与会话门禁 arm 语义一致）
        try:
            plan = build_plan(intent, self.poses)
            return self._run(intent, plan)
        finally:
            with self._lock:
                self._busy = False

    # ---------------------------------------------------------------- 执行
    def _run(self, intent: Intent, plan: Plan) -> Result:
        if not plan.complete:
            reason = "姿态文件缺少 " + "、".join(plan.missing) + "，整体拒绝执行"
            self.log(f"[voice] 忽略：{reason}")
            self._record(command=intent.command, text=intent.text, matched=intent.matched,
                         planned=len(plan.steps), executed=0, status=reason)
            return Result(False, command=intent.command, planned=len(plan.steps), reason=reason)

        self.log(f"[voice] 执行 {intent.command}：{' -> '.join(step.pose for step in plan.steps)}"
                 + ("（干跑，未打开串口）" if self.dry_run else ""))
        executed = 0
        for index, step in enumerate(plan.steps):
            if self._stop.is_set():
                break
            if index and not self._sleep_between_steps():
                break
            if self.dry_run:
                self.log(f"[dry-run] 步骤 {index + 1}/{len(plan.steps)}：{step.pose} "
                         f"归一化 {[round(value, 4) for value in step.ratios]}")
            else:
                try:
                    self.arm.move(list(step.ratios))
                except RuntimeError as exc:
                    # motion_guard 在停止标志置位时抛出，用于打断一次长动作。
                    if self._stop.is_set():
                        self.log(f"[voice] 运动被停止命令中断：{exc}")
                        break
                    raise
                self.log(f"[voice] 步骤 {index + 1}/{len(plan.steps)}：{step.pose} 完成")
            executed += 1

        preempted = self._stop.is_set() and executed < len(plan.steps)
        status = "被停止命令抢占" if preempted else "完成"
        if preempted:
            self.log(f"[voice] 已抢占：完成 {executed}/{len(plan.steps)} 步，剩余步骤丢弃")
        self._record(command=intent.command, text=intent.text, matched=intent.matched,
                     planned=len(plan.steps), executed=executed, status=status)
        return Result(True, command=intent.command, planned=len(plan.steps),
                      executed=executed, reason=status)

    def _sleep_between_steps(self) -> bool:
        """步骤间停顿；停止标志置位时立即返回 False。"""
        deadline = self.clock() + self.step_pause
        while self.clock() < deadline:
            if self._stop.is_set():
                return False
            time.sleep(min(0.02, max(0.0, deadline - self.clock())))
        return not self._stop.is_set()

    def _record(self, *, command, text, matched, planned, executed, status) -> None:
        self.journal.append({
            "at": round(self.clock(), 4),
            "command": command,
            "text": text,
            "matched": matched,
            "planned": planned,
            "executed": executed,
            "status": status,
            "dry_run": self.dry_run,
        })


def build_plan(intent: Intent, poses: dict[str, list[float]]) -> Plan:
    """把意图展开成姿态序列；缺失姿态记录在 ``missing``，由调用方整体拒绝。"""
    if intent.command not in POSE_SEQUENCES:
        raise ValueError(f"未注册的语音命令：{intent.command}")
    names = POSE_SEQUENCES[intent.command]
    missing = tuple(name for name in names if name not in poses)
    steps = tuple(Step(name, tuple(poses[name])) for name in names if name in poses)
    return Plan(intent.command, steps, missing)
