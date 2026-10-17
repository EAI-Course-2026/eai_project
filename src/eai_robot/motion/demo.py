"""Pure saved-home validation and joint-space demo planning."""
from dataclasses import asdict
import json
from pathlib import Path
from eai_robot.config import ROOT
from eai_robot.calibration.results import JOINTS, DEFAULT_FEEDBACK_SLACK

DEFAULT_HOME = ROOT / 'configs/demo_home.json'
# Match ArmController.feedback_slack for torque-off gravity drift at the entry boundary.
START_FEEDBACK_SLACK = DEFAULT_FEEDBACK_SLACK
# Fixed offsets from the saved home, never from the current measured pose.
# Grip excursions total 310 counts on this arm, with 1.2 s photographic dwell.
PROFILES = {
    'home': (),
    'gripper': (
        ('夹爪端 A：停留拍摄', (0, 0, 0, 0, 0, 170)),
        ('夹爪端 B：停留拍摄', (0, 0, 0, 0, 0, -140)),
        ('夹爪端 A：再次展示', (0, 0, 0, 0, 0, 170)),
        ('夹爪端 B：再次展示', (0, 0, 0, 0, 0, -140)),
    ),
    'showcase': (
        ('左侧展示、夹爪端 A', (-120, 0, 0, 0, -120, 170)),
        ('右侧展示、夹爪端 B', (120, 0, 0, 0, 120, -140)),
        ('左侧展示、腕部转向', (-120, 0, 0, 0, 120, 170)),
        ('右侧展示、腕部回转', (120, 0, 0, 0, -120, -140)),
    ),
    'transfer': (
        ('A 侧准备，夹爪端 A', (-100, 0, 0, 0, 0, 170)),
        ('A 侧探出', (-100, -18, 18, -18, 0, 170)),
        ('模拟夹持，夹爪端 B', (-100, -18, 18, -18, 0, -140)),
        ('收回到转运姿态', (-100, 0, 0, 0, 0, -140)),
        ('转到 B 侧', (100, 0, 0, 0, 70, -140)),
        ('B 侧探出', (100, -18, 18, -18, 70, -140)),
        ('模拟释放，夹爪端 A', (100, -18, 18, -18, 70, 170)),
        ('B 侧收回', (100, 0, 0, 0, 70, 170)),
    ),
}


def load_home(path, calibration, margin=20):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    expected = {name: asdict(calibration[name]) for name in JOINTS}
    if data.get('schema_version') != 1 or data.get('calibration') != expected:
        raise ValueError('起点文件与当前六关节校准不匹配；重新示教或核对 --home-file')
    home, radius = data.get('home_raw'), data.get('start_radius_counts')
    for values in (home, radius):
        if not isinstance(values, list) or len(values) != 6 or any(type(v) is not int for v in values):
            raise ValueError('起点和初始误差范围必须是六个整数')
    if any(not 20 <= r <= 300 for r in radius):
        raise ValueError('每关节初始误差半径必须为 20..300 刻度')
    for name, value in zip(JOINTS, home, strict=True):
        low, high = calibration[name].bounds(margin)
        if not low <= value <= high:
            raise ValueError(f'{name} 起点 {value} 超出内缩范围 {low}..{high}')
    return dict(enumerate(home, 1)), dict(enumerate(radius, 1))


def check_start(calibration, start, home, radius):
    if set(start) != set(range(1, 7)) or any(type(p) is not int or not 0 <= p <= 1023 for p in start.values()):
        raise ValueError('起始反馈必须包含 ID 1..6 的有效原始位置')
    envelope = {}
    for name in JOINTS:
        cal, sid = calibration[name], calibration[name].id
        low = max(0, cal.range_min - START_FEEDBACK_SLACK,
                  home[sid] - radius[sid] - START_FEEDBACK_SLACK)
        high = min(1023, cal.range_max + START_FEEDBACK_SLACK,
                   home[sid] + radius[sid] + START_FEEDBACK_SLACK)
        envelope[sid] = [low, high]
        if not low <= start[sid] <= high:
            raise ValueError(f'{name} 起始 {start[sid]} 超出本演示入口 {low}..{high}；'
                             '支撑且关闭扭矩后移入范围，或重新示教；未启用运动')
    return envelope


def stage(calibration, label, targets, margin=0, phase='demo'):
    ratios = []
    for name in JOINTS:
        cal = calibration[name]
        low, high = cal.bounds(margin)
        if not low <= targets[cal.id] <= high:
            raise ValueError(f'{label}: {name} 目标 {targets[cal.id]} 超出 {low}..{high}；'
                             '调整保存的起点，不自动裁剪动作')
        ratios.append(cal.ratio(targets[cal.id]))
    return {'name': label, 'phase': phase, 'targets': dict(targets), 'ratios': ratios}


def build_plan(calibration, start, home, radius, profile='showcase', margin=20):
    if profile not in PROFILES:
        raise ValueError('未知 demo')
    if type(margin) is not int or not 10 <= margin <= 100:
        raise ValueError('演示边界余量必须为 10..100 刻度')
    check_start(calibration, start, home, radius)
    # Maintain measured base/roll/grip during the first proximal homing phase.
    folded = {cal.id: max(cal.range_min, min(cal.range_max, start[cal.id]))
              for cal in calibration.values()}
    for sid in (2, 3, 4):
        folded[sid] = home[sid]
    plan = [stage(calibration, '归位 1：肩、肘、腕进入统一姿态', folded, phase='home'),
            stage(calibration, '归位 2：底座、腕旋转、夹爪进入统一起点', home, margin, 'home')]
    for label, offsets in PROFILES[profile]:
        targets = {sid: home[sid] + offset for sid, offset in enumerate(offsets, 1)}
        plan.append(stage(calibration, label, targets, margin))
    if profile != 'home':
        plan.append(stage(calibration, '返回统一演示起点', home, margin))
    return plan
