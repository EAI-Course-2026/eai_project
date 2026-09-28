"""Calibrated, checked synchronous position control for SCS215."""
import math
import time
from .calibration import JOINTS, MODEL_NUMBER, validate


class ArmController:
    def __init__(self, backend, calibration, margin=0, max_step=5, interval=0.05,
                 velocity=None, tolerance=10, timeout=5.0, motion="direct",
                 allow_wide_range=False, feedback_slack=3, progress_timeout=None):
        self.backend = backend
        self.calibration = validate(calibration)
        if not (type(max_step) is int and 1 <= max_step <= 50
                and math.isfinite(interval) and 0.02 <= interval <= 1
                and (velocity is None or type(velocity) is int and 1 <= velocity <= 1000)
                and type(tolerance) is int and 0 <= tolerance <= 50
                and math.isfinite(timeout) and 0.1 <= timeout <= 60
                and motion in ("direct", "smooth")
                and type(feedback_slack) is int and 0 <= feedback_slack <= 5
                and (progress_timeout is None or math.isfinite(progress_timeout)
                     and 0.5 <= progress_timeout <= 10)):
            raise ValueError("运动参数无效")
        self.margin, self.max_step, self.interval = margin, max_step, interval
        self.velocity, self.tolerance, self.timeout = velocity, tolerance, timeout
        self.motion, self.feedback_slack = motion, feedback_slack
        self.progress_timeout = progress_timeout
        self.enabled = False
        self.torque_touched = False
        for cal in calibration.values():
            cal.bounds(margin)
            if cal.range_max - cal.range_min >= 900 and not allow_wide_range:
                raise ValueError("校准行程接近 0/1023 反馈边界；核对连续行程后使用 --allow-wide-range")

    def positions(self):
        result = {sid: self.backend.read(sid, "Present_Position") for sid in range(1, 7)}
        if any(type(p) is not int or not 0 <= p <= 1023 for p in result.values()):
            raise RuntimeError("SCS215 位置反馈无效，停止控制")
        return result

    def _check_pose(self, positions):
        for name, cal in self.calibration.items():
            low, high = cal.bounds(self.margin)
            if not low - self.feedback_slack <= positions[cal.id] <= high + self.feedback_slack:
                raise RuntimeError(f"{name} 当前刻度 {positions[cal.id]} 不在 {low}..{high}；"
                                   "请在扭矩关闭且支撑机械臂时手动移入范围")

    def inspect(self):
        """Read-only model, ID, hardware position-mode and limit validation."""
        limits = inspect_hardware(self.backend)
        for name, cal in self.calibration.items():
            low, high = cal.bounds(self.margin)
            hw_low, hw_high = limits[cal.id]
            if low < hw_low or high > hw_high:
                raise RuntimeError(f"{name} 软件范围 {low}..{high} 超出硬件限位 {hw_low}..{hw_high}")
        return self.positions()

    def enable(self):
        current = self.inspect()
        self._check_pose(current)
        if any(self.backend.read(sid, "Torque_Enable") != 0 for sid in range(1, 7)):
            raise RuntimeError("启用前六个关节必须已关闭扭矩；先支撑机械臂并运行 release")
        # Feedback can drift a few counts beyond an endpoint. Commanded goals
        # always remain inside the effective limits, including the initial hold.
        hold = {cal.id: max(cal.bounds(self.margin)[0],
                           min(cal.bounds(self.margin)[1], current[cal.id]))
                for cal in self.calibration.values()}
        # Preserve the existing speed by default, matching move_positions.py.
        for sid in range(1, 7):
            self.backend.write(sid, "Running_Time", 0)
            if self.velocity is not None:
                self.backend.write(sid, "Goal_Velocity", self.velocity)
        self.backend.sync_positions(hold)
        self._verify_goals(hold)
        # Set before the first torque write: partial enable also needs cleanup.
        self.torque_touched = True
        try:
            for sid in range(1, 7):
                self.backend.write(sid, "Torque_Enable", 1)
            self.enabled = True
        except BaseException:
            self.disable()
            raise

    def recover_startup(self, *, max_overtravel=8, max_step=10, max_recovery=60,
                        clearance=None, on_event=None, waypoint_tolerance=20):
        """Explicit, bounded startup recovery from EEPROM vicinity into software range.

        The temporary hold and waypoints may lie outside the software 0..1 range,
        but every commanded position stays inside the EEPROM limits. Normal move()
        keeps its stricter software bounds and is used only after recovery succeeds.
        """
        if (self.enabled or type(max_overtravel) is not int or not 0 <= max_overtravel <= 8
                or type(max_step) is not int or not 1 <= max_step <= 20
                or type(max_recovery) is not int or not 1 <= max_recovery <= 60
                or type(waypoint_tolerance) is not int or not 1 <= waypoint_tolerance <= 20):
            raise ValueError("启动回收参数无效或扭矩已开启")
        if on_event is not None and not callable(on_event):
            raise ValueError("启动回收事件回调无效")
        clearance = {} if clearance is None else dict(clearance)
        if not set(clearance) <= {4, 6}:
            raise ValueError("启动避碰仅允许腕俯仰 ID4 和夹爪 ID6")
        limits = inspect_hardware(self.backend)
        for name, cal in self.calibration.items():
            low, high = cal.bounds(self.margin)
            hw_low, hw_high = limits[cal.id]
            if low < hw_low or high > hw_high:
                raise RuntimeError(f"{name} 软件范围超出 EEPROM 限位")
        current = self.positions()
        for sid, raw in current.items():
            hw_low, hw_high = limits[sid]
            if raw in (0, 1023) or not hw_low - max_overtravel <= raw <= hw_high + max_overtravel:
                raise RuntimeError(f"ID {sid} 初始反馈 {raw} 距 EEPROM 限位过远，拒绝自动回收")
            if self.backend.read(sid, "Torque_Enable") != 0:
                raise RuntimeError("启动回收前六台扭矩必须关闭")
            if self.backend.read(sid, "Status") != 0:
                raise RuntimeError(f"ID {sid} 启动回收前状态报警")
        hold = {sid: max(limits[sid][0], min(limits[sid][1], raw))
                for sid, raw in current.items()}
        for sid, raw in clearance.items():
            cal = self.calibration[JOINTS[sid - 1]]
            if (type(raw) is not int or not cal.range_min <= raw <= cal.range_max
                    or abs(raw - hold[sid]) > 180):
                raise ValueError(f"ID {sid} 启动避碰目标无效或距离超过 180 刻度")
        # Check the whole recovery distance before any write or torque enable.
        destinations = {}
        for name, cal in self.calibration.items():
            sid = cal.id
            low, high = cal.bounds(self.margin)
            projected = clearance.get(sid, current[sid])
            if projected < low - self.feedback_slack:
                destinations[sid] = min(high, low + self.tolerance)
            elif projected > high + self.feedback_slack:
                destinations[sid] = max(low, high - self.tolerance)
            if sid in destinations and abs(destinations[sid] - hold[sid]) > max_recovery:
                raise RuntimeError(f"ID {sid} 启动回收需移动超过 {max_recovery} 刻度，先手动放到范围附近")
        trace = []
        def record(phase, **data):
            item = {"phase": phase, **data}
            trace.append(item)
            if on_event is not None:
                on_event(item)
        record("preflight", feedback=dict(current), hardware_hold=dict(hold),
               recovery_destinations=dict(destinations))
        for sid in range(1, 7):
            self.backend.write(sid, "Running_Time", 0)
            if self.velocity is not None:
                self.backend.write(sid, "Goal_Velocity", self.velocity)
        self.backend.sync_positions(hold)
        self._verify_goals(hold)
        self.torque_touched = True
        try:
            for sid in range(1, 7):
                self.backend.write(sid, "Torque_Enable", 1)
            self.enabled = True
            deadline = time.monotonic() + min(self.timeout, 2.0)
            while True:
                feedback = self.positions()
                for sid in range(1, 7):
                    if self.backend.read(sid, "Status") != 0:
                        raise RuntimeError(f"ID {sid} 回收时状态报警")
                if all(limits[sid][0] - self.feedback_slack <= feedback[sid]
                       <= limits[sid][1] + self.feedback_slack for sid in range(1, 7)):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"使能后反馈未回到 EEPROM 限位附近：{feedback}")
                time.sleep(self.interval)
            record("torque_on", feedback=dict(feedback))
            goals = dict(hold)
            for sid in (4, 6):
                if sid not in clearance:
                    continue
                origin, destination = goals[sid], clearance[sid]
                steps = max(1, math.ceil(abs(destination - origin) / max_step))
                for index in range(1, steps + 1):
                    target = round(origin + (destination - origin) * index / steps)
                    goals[sid] = target
                    self.backend.sync_positions(goals)
                    self._verify_goals(goals)
                    deadline = time.monotonic() + min(self.timeout, 2.0)
                    while True:
                        feedback = self.positions()
                        if any(self.backend.read(motor, "Status") != 0 for motor in range(1, 7)):
                            raise RuntimeError("启动避碰中状态报警")
                        if any(not limits[motor][0] - self.feedback_slack <= feedback[motor]
                               <= limits[motor][1] + self.feedback_slack for motor in range(1, 7)):
                            raise RuntimeError(f"启动避碰中反馈超出 EEPROM 邻域：{feedback}")
                        clearance_tolerance = self.tolerance if index == steps else waypoint_tolerance
                        if abs(feedback[sid] - target) <= clearance_tolerance:
                            break
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f"ID {sid} 启动避碰未跟上目标 {target}：反馈 {feedback[sid]}")
                        time.sleep(self.interval)
                    record("clearance", id=sid, target=target, feedback=dict(feedback))
            for sid in (2, 3, 4, 1, 5, 6):
                if sid not in destinations:
                    continue
                origin, destination = goals[sid], destinations[sid]
                steps = max(1, math.ceil(abs(destination - origin) / max_step))
                for index in range(1, steps + 1):
                    target = round(origin + (destination - origin) * index / steps)
                    goals[sid] = target
                    self.backend.sync_positions(goals)
                    self._verify_goals(goals)
                    deadline = time.monotonic() + min(self.timeout, 2.0)
                    while True:
                        feedback = self.positions()
                        if any(self.backend.read(motor, "Status") != 0 for motor in range(1, 7)):
                            raise RuntimeError("启动回收中状态报警")
                        if any(not limits[motor][0] - self.feedback_slack <= feedback[motor]
                               <= limits[motor][1] + self.feedback_slack for motor in range(1, 7)):
                            raise RuntimeError(f"启动回收中反馈再次越过 EEPROM 限位：{feedback}")
                        if abs(feedback[sid] - target) <= waypoint_tolerance:
                            break
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f"ID {sid} 启动回收未跟上目标 {target}：反馈 {feedback[sid]}")
                        time.sleep(self.interval)
                    record("recover", id=sid, target=target, feedback=dict(feedback))
                low, high = self.calibration[JOINTS[sid - 1]].bounds(self.margin)
                if not low - self.feedback_slack <= feedback[sid] <= high + self.feedback_slack:
                    raise RuntimeError(f"ID {sid} 回收后仍未进入软件范围：{feedback[sid]}")
            self._check_pose(self.positions())
            return trace
        except BaseException:
            self.disable()
            raise

    def _verify_goals(self, targets):
        for sid, target in targets.items():
            if self.backend.read(sid, "Goal_Position") != target:
                raise RuntimeError(f"ID {sid} 未确认接收目标 {target}")

    def move(self, ratios):
        if not self.enabled:
            raise RuntimeError("尚未启用控制")
        if len(ratios) != 6:
            raise ValueError("必须提供六个关节值")
        # Validate the complete command before sending any write.
        targets = {cal.id: cal.position(value, self.margin)
                   for cal, value in zip((self.calibration[n] for n in JOINTS), ratios, strict=True)}
        try:
            current = self.positions()
            self._check_pose(current)
            if self.motion == "direct":
                # One frame for all six goals, matching the working experiment.
                self.backend.sync_positions(targets)
            else:
                current = {cal.id: max(cal.bounds(self.margin)[0],
                                      min(cal.bounds(self.margin)[1], current[cal.id]))
                           for cal in self.calibration.values()}
                delta = max(abs(targets[sid] - current[sid]) for sid in targets)
                steps = max(1, math.ceil(delta / self.max_step))
                for step in range(1, steps + 1):
                    next_pose = {sid: round(start + (targets[sid] - start) * step / steps)
                                 for sid, start in current.items()}
                    self.backend.sync_positions(next_pose)
                    time.sleep(self.interval)
                    feedback = self.positions()
                    self._check_pose(feedback)
                    if any(abs(feedback[sid] - next_pose[sid]) > 50 for sid in targets):
                        raise RuntimeError("关节跟随误差超过 50 刻度，停止运动")
            self._verify_goals(targets)
            deadline = time.monotonic() + self.timeout
            progress_at = {sid: time.monotonic() for sid in targets}
            best_error = {sid: abs(current[sid] - targets[sid]) for sid in targets}
            while True:
                feedback = self.positions()
                self._check_pose(feedback)
                if all(abs(feedback[sid] - targets[sid]) <= self.tolerance for sid in targets):
                    return targets, feedback
                now = time.monotonic()
                if self.progress_timeout is not None:
                    for sid in targets:
                        error = abs(feedback[sid] - targets[sid])
                        if error < best_error[sid]:
                            best_error[sid] = error
                            progress_at[sid] = now
                        if error > self.tolerance and now - progress_at[sid] >= self.progress_timeout:
                            failure = TimeoutError(f"ID {sid} 连续 {self.progress_timeout}s 无到位进展："
                                                   f"目标 {targets[sid]}，反馈 {feedback[sid]}")
                            failure.feedback = dict(feedback)
                            failure.targets = dict(targets)
                            raise failure
                if time.monotonic() >= deadline:
                    error = TimeoutError(f"未在 {self.timeout}s 内到位：目标 {targets}，反馈 {feedback}")
                    error.feedback = dict(feedback)
                    error.targets = dict(targets)
                    raise error
                time.sleep(self.interval)
        except BaseException:
            self.disable()
            raise

    def disable(self):
        self.enabled = False
        failures = release(self.backend)
        self.torque_touched = bool(failures)
        return failures


def inspect_hardware(backend):
    check_identity(backend)
    limits = {}
    for sid in range(1, 7):
        low = backend.read(sid, "Min_Position_Limit")
        high = backend.read(sid, "Max_Position_Limit")
        if not 0 <= low < high <= 1023:
            raise RuntimeError(f"ID {sid} 硬件限位 {low}..{high} 不是位置模式；不自动修改 EEPROM")
        limits[sid] = low, high
    return limits


def check_identity(backend):
    for sid in range(1, 7):
        model = backend.read(sid, "Model_Number")
        if model != MODEL_NUMBER or backend.read(sid, "ID") != sid:
            raise RuntimeError(f"ID {sid} 型号/编号不匹配：期望 SCS215 型号码 {MODEL_NUMBER}，实际 {model}")


def release(backend):
    """Try every joint even if an earlier torque-disable fails."""
    failures = []
    for sid in range(1, 7):
        try:
            backend.write(sid, "Torque_Enable", 0)
        except Exception as exc:
            failures.append(f"ID {sid}: {exc}")
    return failures
