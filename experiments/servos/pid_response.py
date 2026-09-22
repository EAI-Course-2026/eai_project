"""SCS215: host-side PID -> signed PWM -> measured angle.

Run: python experiments/servos/pid_response.py --compare
One run: python experiments/servos/pid_response.py --kp 3 --ki 0.5 --kd 0.25
Offline example (NOT hardware measurements): python experiments/servos/pid_response.py --simulate --compare

The SC table defines P/D but reserves address 23. We do NOT write a guessed I
register. PWM mode is documented by Feetech's scscl.PWMMode/WritePWM SDK.
Reference: https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/scscl.py
Reference: https://docs.waveshare.net/Memory_Map_Explanation/SC_Servo_Memory_Map_Explanation/
SCS215 nominal scale: 300 degrees over 0..1023 (calibrate for exact angles).
Host software is not hard real-time. In PWM mode unplugging USB does NOT guarantee
motor stop: keep the servo power disconnect accessible. Use an unloaded servo.
"""
import argparse
import csv
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from eai_robot.config import ROOT, load_config
# Name, Kp, Ki, Kd. These are Python PWM/degree gains, NOT register values.
EXPERIMENTS = [("P_low", 1.5, 0, 0), ("P_high", 3, 0, 0),
               ("PD", 3, 0, 0.25), ("PID", 3, 0.5, 0.25)]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def word(value):
    return int(value).to_bytes(2, "big")


def pwm_word(value):
    value = int(round(value))
    if abs(value) > 1000:
        raise ValueError("PWM must be within -1000..1000")
    return word(abs(value) | (1024 if value < 0 else 0))


def packet(sid, instruction, params):
    body = bytes([sid, len(params) + 2, instruction]) + bytes(params)
    return b"\xff\xff" + body + bytes([(~sum(body)) & 255])


def decode(reply, sid, count):
    if (len(reply) != count + 6 or reply[:2] != b"\xff\xff"
            or reply[2] != sid or reply[3] != count + 2
            or sum(reply[2:]) & 255 != 255):
        raise RuntimeError(f"无有效舵机回复: {reply.hex(' ') or '无数据'}")
    if reply[4]:
        raise RuntimeError(f"舵机故障状态 0x{reply[4]:02x}")
    return reply[5:-1]


class Servo:
    def __init__(self, port, sid):
        import serial
        self.sid = sid
        self.s = serial.Serial(port, load_config()['serial']['baudrate'], timeout=0.05, write_timeout=0.05)
        self.ack = True

    def exchange(self, instruction, params, count=0, expect=True):
        self.s.reset_input_buffer()
        self.s.write(packet(self.sid, instruction, params))
        if not expect:
            return b""
        # pyserial read(n) handles fragmented arrival until its timeout.
        return decode(self.s.read(count + 6), self.sid, count)

    def read(self, address, count):
        return self.exchange(2, [address, count], count)

    def write(self, address, data, verify=False):
        self.exchange(3, bytes([address]) + bytes(data), expect=self.ack)
        if verify and self.read(address, len(data)) != bytes(data):
            raise RuntimeError(f"地址 {address} 的写入回读不一致")

    def position(self):
        value = int.from_bytes(self.read(56, 2), "big")
        if not 0 <= value <= 1023:
            raise RuntimeError(f"位置反馈超出 SCS215 范围: {value}")
        return value

    def stop(self):
        # Do not wait for a reply before sending the torque-disable instruction.
        self.s.write(packet(self.sid, 3, [40, 0]))
        time.sleep(0.005)
        self.write(40, b"\x00", verify=True)


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    limit: float
    integral: float = 0
    previous: float | None = None
    velocity: float = 0

    def step(self, target, angle, dt):
        error = target - angle
        raw_v = 0 if self.previous is None else (angle - self.previous) / dt
        # Filter measured velocity; D on measurement avoids setpoint derivative kick.
        self.velocity += dt / (0.05 + dt) * (raw_v - self.velocity)
        self.previous = angle
        p = self.kp * error
        d = -self.kd * self.velocity
        proposed = self.integral + error * dt
        if self.ki:
            proposed = clamp(proposed, -self.limit / self.ki, self.limit / self.ki)
        candidate = p + self.ki * proposed + d
        # Conditional integration prevents accumulating error into saturation.
        if abs(candidate) <= self.limit or candidate * error < 0:
            self.integral = proposed
        i = self.ki * self.integral
        return clamp(p + i + d, -self.limit, self.limit), p, i, d


def move_to_start(servo, start, lower, upper):
    servo.stop()
    servo.write(42, word(start) + word(0) + word(150), verify=True)
    servo.write(40, b"\x01", verify=True)
    deadline, stable = time.monotonic() + 12, 0
    while time.monotonic() < deadline:
        position = servo.position()
        if not lower <= position <= upper:
            raise RuntimeError("回到起点时超过允许位置范围")
        stable = stable + 1 if abs(position - start) <= 4 else 0
        if stable >= 10:
            servo.stop()
            return
        time.sleep(0.05)
    raise RuntimeError("12 秒内未能回到起点；检查负载、供电或位置范围")


def record_hardware(servo, args, gains, writer, rows, lower, upper):
    name, kp, ki, kd = gains
    step_deg = args.span_deg / 1023
    origin = servo.position()
    target_raw = args.start + round(args.delta / step_deg)
    target = (target_raw - origin) * step_deg
    if not lower < origin < upper or not lower < target_raw < upper:
        raise RuntimeError("起点或目标超出允许位置范围")
    pid = PID(kp, ki, kd, args.max_pwm)
    servo.write(44, word(0), verify=True)
    servo.write(9, b"\x00" * 4, verify=True)  # Temporary motor/PWM mode.
    servo.write(40, b"\x01", verify=True)
    t0 = previous = time.monotonic()
    previous_angle = 0.0
    period = 1 / args.hz
    next_sample = t0
    while True:
        time.sleep(max(0, next_sample - time.monotonic()))
        raw = servo.position()
        now = time.monotonic()
        elapsed = now - t0
        dt = max(now - previous, 1e-4)
        if dt > 0.12:
            raise RuntimeError("控制循环超过 120ms，停止本次实验")
        if not lower <= raw <= upper:
            raise RuntimeError("到达软件位置边界，停止本次实验")
        angle = (raw - origin) * step_deg
        if abs(angle - previous_angle) > 15:
            raise RuntimeError("位置反馈突变，停止本次实验")
        if angle * math.copysign(1, args.delta) < -5:
            raise RuntimeError("运动方向与目标相反；核对 PWM 方向后再实验")
        if elapsed > args.duration:
            break
        u, p, i, d = pid.step(target, angle, dt)
        servo.write(44, pwm_word(u * args.pwm_sign))
        row = [name, elapsed, angle, target, u, p, i, d, raw]
        writer.writerow(row)
        rows.append(row)
        previous, previous_angle = now, angle
        next_sample = max(next_sample + period, now)
    servo.stop()
    print(f"{name}: 最后误差 {target - rows[-1][2]:.2f}°")


def simulate(args, experiments, writer, rows):
    # Illustrative plant only, not an identified SCS215 model.
    # theta'' = 6 * PWM - 3 * theta' - constant opposing load.
    dt = 1 / args.hz
    for name, kp, ki, kd in experiments:
        pid = PID(kp, ki, kd, args.max_pwm)
        angle = speed = u = 0.0
        for n in range(round(args.duration / dt) + 1):
            u, p, i, d = pid.step(args.delta, angle, dt)
            row = [name, n * dt, angle, args.delta, u, p, i, d, ""]
            rows.append(row)
            writer.writerow(row)
            for _ in range(10):
                speed += (6 * u - 3 * speed - 20 * math.copysign(1, args.delta)) * dt / 10
                angle += speed * dt / 10


def plot(rows, directory, simulated, status):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for name in dict.fromkeys(r[0] for r in rows):
        group = [r for r in rows if r[0] == name]
        line, = axes[0].plot([r[1] for r in group], [r[2] for r in group], label=name)
        axes[0].plot([r[1] for r in group], [r[3] for r in group], "--",
                     color=line.get_color(), alpha=0.5)
        axes[1].plot([r[1] for r in group], [r[4] for r in group], label=name)
    axes[0].set_ylabel("Relative angle (deg)")
    axes[1].set_ylabel("PWM command (1000 = 100%)")
    axes[1].set_xlabel("Time since step command (s)")
    for ax in axes:
        ax.grid(alpha=0.25)
        ax.legend()
    title = "SIMULATION - illustrative model, NOT measured" if simulated else "SCS215 measured step responses"
    # 图中只用英文，完整中文错误保留在终端和 status.txt。
    plot_status = "complete" if status == "complete" else "ABORTED (see status.txt)"
    axes[0].set_title(f"{title}\nDashed lines: target; run status: {plot_status}")
    fig.tight_layout()
    fig.savefig(directory / "response.png", dpi=160)
    fig.savefig(directory / "response.svg")
    plt.close(fig)


def hardware(args, experiments, writer, rows, directory):
    servo = Servo(args.port, args.id)
    saved = None
    mutated = False
    try:
        servo.exchange(1, [])
        if servo.read(2, 1) != b"\x01":
            raise RuntimeError("设备未报告 SCS 大端格式，停止")
        servo.ack = bool(servo.read(8, 1)[0])
        limits = servo.read(9, 4)
        lo, hi = int.from_bytes(limits[:2], "big"), int.from_bytes(limits[2:], "big")
        if not 0 <= lo < hi <= 1023:
            raise RuntimeError("设备不在有效位置模式；先恢复原有位置上下限")
        step_deg = args.span_deg / 1023
        target = args.start + round(args.delta / step_deg)
        lower, upper = max(lo, 40), min(hi, 983)
        if not lower + 10 <= min(args.start, target) < max(args.start, target) <= upper - 10:
            raise RuntimeError(f"90°路径超出可用范围 {lower}..{upper}，请调整 --start 或 --delta")
        initial_position = servo.position()
        if not lower <= initial_position <= upper:
            raise RuntimeError("当前轴位置太靠近端点，请断开扭矩后将输出轴放到有效范围中部")
        saved = {"limits": limits.hex(), "lock": servo.read(48, 1).hex(),
                 "motion_time_speed": servo.read(44, 4).hex(),
                 "initial_position": initial_position, "port": args.port, "id": args.id}
        (directory / "original_settings.json").write_text(json.dumps(saved, indent=2))
        # Never unlock EEPROM: mode changes remain volatile and are read back.
        mutated = True
        servo.stop()
        servo.write(48, b"\x01", verify=True)
        for gains in experiments:
            servo.write(9, limits, verify=True)
            move_to_start(servo, args.start, lower, upper)
            # Narrow PWM travel to the intended path plus 10-degree overshoot margin.
            margin = round(10 / step_deg)
            run_lo = max(lower, min(args.start, target) - margin)
            run_hi = min(upper, max(args.start, target) + margin)
            print(f"{gains[0]}: Kp={gains[1]}, Ki={gains[2]}, Kd={gains[3]}")
            record_hardware(servo, args, gains, writer, rows, run_lo, run_hi)
            servo.write(44, word(0), verify=True)
            servo.write(9, limits, verify=True)
    finally:
        try:
            if mutated:
                servo.stop()
                servo.write(44, word(0), verify=True)
                servo.write(9, bytes.fromhex(saved["limits"]), verify=True)
                # Set a benign future position goal, leave torque OFF.
                servo.write(42, word(servo.position()), verify=True)
                servo.write(44, bytes.fromhex(saved["motion_time_speed"]), verify=True)
                servo.write(48, bytes.fromhex(saved["lock"]), verify=True)
                print("已确认关闭扭矩并恢复原位置模式及设置。")
        except Exception as exc:
            # Loss of the link can prevent ANY software stop; never claim success.
            print(f"停止/恢复未确认，请立即断开舵机电源：{exc}")
            raise
        finally:
            servo.s.close()


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="SCS215 转过 90°：Python PID、采样、曲线")
    parser.add_argument("--port", default=cfg['serial']['port'])
    parser.add_argument("--id", type=int, default=cfg["single"]["id"])
    parser.add_argument("--start", type=int, default=cfg["pid"]["start_step"], help="每次实验的起点刻度")
    parser.add_argument("--delta", type=float, default=90, help="相对起点的角度变化")
    parser.add_argument("--span-deg", type=float, default=cfg["pid"]["span_deg"], help="0..1023 对应角度；精确角度需校准")
    parser.add_argument("--kp", type=float, default=3)
    parser.add_argument("--ki", type=float, default=0.5)
    parser.add_argument("--kd", type=float, default=0.25)
    parser.add_argument("--duration", type=float, default=6)
    parser.add_argument("--hz", type=float, default=50)
    parser.add_argument("--max-pwm", type=int, default=250, help="输出上限，250=25%%占空比")
    # 本机实测：正 PWM 使位置刻度减小，因此反转输出方向。
    parser.add_argument("--pwm-sign", type=int, choices=(-1, 1), default=cfg["pid"]["pwm_sign"],
                        help="PWM 与位置方向的映射；本机实测默认 -1")
    parser.add_argument("--compare", action="store_true", help="比较文件顶部的四组参数")
    parser.add_argument("--simulate", action="store_true", help="纯软件示例，不连接舵机")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs" / "pid")
    args = parser.parse_args()
    numbers = [args.kp, args.ki, args.kd, args.duration, args.hz, args.span_deg, args.delta]
    if not all(math.isfinite(n) for n in numbers):
        parser.error("参数必须是有限数值")
    if not (0 <= args.id <= 253 and 0 <= args.start <= 1023
            and 0 < abs(args.delta) <= 90 and 1 <= args.duration <= 30
            and 20 <= args.hz <= 100 and 0 < args.span_deg <= 360
            and 1 <= args.max_pwm <= 400 and min(args.kp, args.ki, args.kd) >= 0):
        parser.error("参数超出实验范围；delta 最大 ±90°，PWM 最大 400，hz 20..100")
    try:
        import matplotlib  # Check plotting BEFORE hardware can move.
    except ImportError:
        parser.error("缺少 matplotlib：uv sync --extra experiments")
    experiments = EXPERIMENTS if args.compare else [("custom_PID", args.kp, args.ki, args.kd)]
    directory = args.out / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ("-SIMULATION" if args.simulate else "-hardware"))
    directory.mkdir(parents=True)
    rows, status = [], "complete"
    (directory / "experiment.json").write_text(json.dumps(
        {**vars(args), "out": str(args.out), "experiments": experiments}, indent=2))
    try:
        with (directory / "samples.csv").open("w", newline="", buffering=1) as stream:
            writer = csv.writer(stream)
            writer.writerow(["experiment", "time_s", "angle_deg", "target_deg", "pwm", "P", "I", "D", "position_raw"])
            if args.simulate:
                simulate(args, experiments, writer, rows)
            else:
                hardware(args, experiments, writer, rows, directory)
    except (Exception, KeyboardInterrupt) as exc:
        status = f"ABORTED: {type(exc).__name__}: {exc}"
        print(status)
    finally:
        (directory / "status.txt").write_text(status + "\n")
        if rows:
            plot(rows, directory, args.simulate, status)
        print(f"数据与曲线：{directory}")
    return 0 if status == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
