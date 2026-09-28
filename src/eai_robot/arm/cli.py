"""Command-line workflow shared by the serial and LeRobot entry points."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

from eai_robot.config import ROOT, load_config
from .calibration import JOINTS, JointCalibration, load, save, parse_ratios
from .controller import ArmController, inspect_hardware, check_identity, release

DEFAULT_CALIBRATION = ROOT / "calibration/scs215_so101.json"


def print_pose(backend, calibration=None, margin=0, diagnostics=False):
    for sid, name in enumerate(JOINTS, 1):
        position = backend.read(sid, "Present_Position")
        if not 0 <= position <= 1023:
            raise RuntimeError(f"ID {sid} 位置反馈无效：{position}")
        ratio = f" ratio={calibration[name].ratio(position, margin):.4f}" if calibration else ""
        state = ""
        if diagnostics:
            torque = backend.read(sid, "Torque_Enable")
            voltage = backend.read(sid, "Present_Voltage") / 10
            temperature = backend.read(sid, "Present_Temperature")
            status = backend.read(sid, "Status")
            state = f" torque={torque} voltage={voltage:.1f}V temperature={temperature}C status=0x{status:02x}"
        print(f"ID {sid} {name:14} raw={position}{ratio}{state}")


def report_release(failures):
    if failures:
        print("扭矩关闭未确认：" + "; ".join(failures) + "。请立即切断舵机电源。", file=sys.stderr)
    else:
        print("六个舵机均已回读确认关闭扭矩；继续支撑机械臂。")


def calibrate(backend, path, overwrite=False, input_fn=input, allow_wide_range=False):
    if path.exists() and not overwrite:
        raise ValueError(f"校准文件已存在：{path}；重新采集请加 --overwrite")
    limits = inspect_hardware(backend)
    print("请支撑机械臂。校准将关闭全部扭矩，逐关节手动设置安全端点；不要强推机械止挡。")
    if input_fn('输入 RELEASE 确认已支撑机械臂：').strip() != "RELEASE":
        raise ValueError("取消校准，未改变扭矩")
    failures = release(backend)
    report_release(failures)
    if failures:
        raise RuntimeError("无法开始校准")
    calibration = {}
    for sid, name in enumerate(JOINTS, 1):
        endpoints = []
        for ratio in (0, 1):
            input_fn(f"ID {sid} {name}：手动移至希望输入 {ratio} 对应的安全端点，回车采集：")
            value = backend.read(sid, "Present_Position")
            low, high = limits[sid]
            if not low <= value <= high:
                raise ValueError(f"ID {sid} 端点 {value} 超出位置反馈/硬件限位")
            print(f"采集 {ratio} -> {value}")
            endpoints.append(value)
        zero, one = endpoints
        if abs(one - zero) < 20:
            raise ValueError(f"{name} 行程不足 20 刻度，请重新采集")
        low, high = sorted(endpoints)
        if high - low >= 900 and not allow_wide_range:
            raise ValueError(f"{name} 接近反馈边界，需调整装配零位后重新采集；不支持跨 0/1023")
        calibration[name] = JointCalibration(sid, int(zero > one), 0, low, high)
    save(path, calibration)
    path.with_suffix(".meta.json").write_text(json.dumps({
        "method": "manual_endpoints", "captured": datetime.now().astimezone().isoformat(),
        "model_number": 1315, "manual_endpoints_verified": True,
        "note": "Two user-selected continuous safe endpoints per joint; software calibration only.",
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"校准完成：{path}（仅保存软件范围，未改 EEPROM）。")


def import_hardware_calibration(backend, path, overwrite=False, reverse=()):
    if path.exists() and not overwrite:
        raise ValueError("校准文件已存在；重新导出需 --overwrite")
    limits = inspect_hardware(backend)
    result = {name: JointCalibration(sid, int(name in reverse), 0, *limits[sid])
              for sid, name in enumerate(JOINTS, 1)}
    save(path, result)
    provenance = {
        "method": "existing_hardware_limits", "captured": datetime.now().astimezone().isoformat(),
        "model_number": 1315, "manual_endpoints_verified": False,
        "direction": "0 increases raw position unless listed in reverse_joints",
        "reverse_joints": list(reverse),
        "note": "Read-only import of previously stored EEPROM limits; not a new manual calibration.",
    }
    path.with_suffix(".meta.json").write_text(json.dumps(provenance, ensure_ascii=False, indent=2) + "\n",
                                            encoding="utf-8")
    print(f"已导出现有硬件限位：{path}；未写舵机，未重新验证机械安全端点。")
    print("方向默认原始刻度递增；反向关节可用 --reverse 指定。宽行程运动需 --allow-wide-range。")


def parser_for(backend_name):
    cfg = load_config()
    parser = argparse.ArgumentParser(description=f"SCS215 六关节编号/校准/同步控制（{backend_name}）")
    parser.add_argument("--port", default=cfg["serial"]["port"])
    parser.add_argument("--baudrate", type=int, default=cfg["serial"]["baudrate"])
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("inspect", help="只读型号、编号、硬件限位及位置；不需要校准文件")
    calibration = sub.add_parser("calibrate", help="关闭扭矩并手动采集六关节端点")
    calibration.add_argument("--overwrite", action="store_true")
    calibration.add_argument("--from-hardware", action="store_true", help="只读导出舵机已存的限位作为校准起点")
    calibration.add_argument("--reverse", choices=JOINTS, action="append", default=[], help="导出时指定反向关节，可重复")
    calibration.add_argument("--allow-wide-range", action="store_true", help="手动确认行程连续后允许宽范围")
    sub.add_parser("release", help="已支撑机械臂后关闭六关节扭矩")
    control = sub.add_parser("control", help="校准后输入六个 0..1，默认只读")
    control.add_argument("--enable", action="store_true", help="明确启用扭矩和运动")
    control.add_argument("--values", nargs=6, type=float, metavar="RATIO", help="一次目标；省略时交互输入")
    control.add_argument("--dry-run", action="store_true", help="仅检查校准和映射，不打开串口")
    control.add_argument("--margin-counts", type=int, default=0)
    control.add_argument("--max-step", type=int, default=5)
    control.add_argument("--interval", type=float, default=0.05)
    control.add_argument("--velocity", type=int, default=None, help="1..1000；默认沿用硬件当前速度")
    control.add_argument("--motion", choices=("direct", "smooth"), default="direct", help="默认一帧发送最终目标；smooth 插值")
    control.add_argument("--allow-wide-range", action="store_true", help="确认宽行程连续且未跨反馈边界")
    control.add_argument("--tolerance", type=int, default=10)
    control.add_argument("--timeout", type=float, default=5.0)
    return parser


def main(backend_name="serial", argv=None):
    parser = parser_for(backend_name)
    args = parser.parse_args(argv)
    backend = controller = None
    try:
        if args.baudrate not in (1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200):
            raise ValueError("不支持的 SCS 波特率")
        calibration = None
        if args.command == "control":
            calibration = load(args.calibration)
            # Validate parameters and all input before connecting, including dry run.
            controller = ArmController(None, calibration, args.margin_counts, args.max_step,
                                       args.interval, args.velocity, args.tolerance, args.timeout,
                                       motion=args.motion, allow_wide_range=args.allow_wide_range)
            values = parse_ratios(" ".join(map(str, args.values))) if args.values is not None else None
            if args.dry_run and args.enable:
                raise ValueError("--dry-run 与 --enable 不能同时使用")
            print("输入顺序：" + " ".join(JOINTS))
            for name in JOINTS:
                cal = calibration[name]
                print(f"ID {cal.id} {name}: 0 -> {cal.position(0, args.margin_counts)}, "
                      f"1 -> {cal.position(1, args.margin_counts)}")
            if values is not None:
                print("目标刻度：", [calibration[n].position(v, args.margin_counts)
                                     for n, v in zip(JOINTS, values, strict=True)])
            if args.dry_run:
                return 0
            if values is not None and not args.enable:
                raise ValueError("发送 --values 需要 --enable；离线映射使用 --dry-run")
        if not args.port:
            raise ValueError("请通过 --port 或 hardware.local.toml 指定串口")
        if args.command == "calibrate" and args.calibration.exists() and not args.overwrite:
            raise ValueError("校准文件已存在；重新采集请加 --overwrite")
        from .backends import SerialBackend, LeRobotBackend
        backend = (SerialBackend if backend_name == "serial" else LeRobotBackend)(args.port, args.baudrate)
        if args.command == "release":
            # Avoid releasing mismatched devices on an unverified bus.
            check_identity(backend)
            failures = release(backend)
            report_release(failures)
            return int(bool(failures))
        if args.command == "calibrate":
            if args.from_hardware:
                import_hardware_calibration(backend, args.calibration, args.overwrite, args.reverse)
            else:
                if args.reverse:
                    raise ValueError("--reverse 用于 --from-hardware；手动校准会自动识别方向")
                calibrate(backend, args.calibration, args.overwrite, allow_wide_range=args.allow_wide_range)
            return 0
        if args.command == "inspect":
            limits = inspect_hardware(backend)
            print(f"六个 ID/型号通过；硬件限位：{limits}")
            print_pose(backend, diagnostics=True)
            return 0
        controller.backend = backend
        controller.inspect()
        print_pose(backend, calibration, args.margin_counts)
        if not args.enable:
            print("只读完成。运动需要 control --enable。")
            return 0
        controller.enable()
        if values is not None:
            targets, reached = controller.move(values)
            print(f"已到位：目标 {targets}，反馈 {reached}")
            return 0
        print("输入六个 0..1；show 查询；q 退出。退出会关闭扭矩，请支撑机械臂。")
        while True:
            text = input("0..1 x6 > ").strip()
            if text.lower() in {"q", "quit", "exit"}:
                break
            if text.lower() == "show":
                print_pose(backend, calibration, args.margin_counts)
                continue
            if not text:
                continue
            try:
                values = parse_ratios(text)
            except ValueError as exc:
                print(f"输入无效：{exc}")
                continue
            targets, reached = controller.move(values)
            print(f"已到位：目标 {targets}，反馈 {reached}")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("已中断。", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"未完成：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        shutdown_failed = False
        if controller is not None and controller.torque_touched:
            failures = controller.disable()
            report_release(failures)
            shutdown_failed = bool(failures)
        if backend is not None:
            backend.close()
        if shutdown_failed:
            return 1
