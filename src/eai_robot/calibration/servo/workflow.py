"""Explicit servo calibration workflows using an injected register backend."""
from datetime import datetime
import json
from pathlib import Path

from eai_robot.calibration.servo.results import JOINTS, JointCalibration, save
from eai_robot.hardware.drivers.inspection import inspect_hardware, release

def calibrate(backend, path, overwrite=False, input_fn=input, allow_wide_range=False, report_release=None):
    if path.exists() and not overwrite:
        raise ValueError(f"校准文件已存在：{path}；重新采集请加 --overwrite")
    limits = inspect_hardware(backend)
    print("请支撑机械臂。校准将关闭全部扭矩，逐关节手动设置安全端点；不要强推机械止挡。")
    if input_fn('输入 RELEASE 确认已支撑机械臂：').strip() != "RELEASE":
        raise ValueError("取消校准，未改变扭矩")
    failures = release(backend)
    if report_release is not None:
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


def save_manual_candidate(captures, limits, root):
    """Validate GUI samples and persist locally; never replace shared calibration."""
    result = {}
    for sid, name in enumerate(JOINTS, 1):
        zero, one = captures[name]
        if zero is None or one is None or abs(one - zero) < 20:
            raise ValueError(f'{name} 尚未采集两个相距至少 20 刻度的端点')
        lo, hi = sorted((zero, one))
        hw_lo, hw_hi = limits[sid]
        if lo < hw_lo or hi > hw_hi:
            raise ValueError(f'{name} 端点超出硬件限位')
        result[name] = JointCalibration(sid, int(zero > one), 0, lo, hi)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    path = Path(root) / 'local_artifacts' / 'experiments' / f'{stamp}_servo-calibration' / 'candidate.json'
    if path.exists():
        raise FileExistsError(path)
    save(path, result)
    path.with_suffix('.meta.json').write_text(json.dumps({
        'method': 'manual_endpoints_gui', 'captured': datetime.now().astimezone().isoformat(),
        'model_number': 1315, 'manual_endpoints_verified': True,
        'note': 'User-selected software endpoints; EEPROM unchanged. Candidate pending review.'},
        ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path
