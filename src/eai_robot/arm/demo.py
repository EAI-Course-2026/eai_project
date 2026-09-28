"""Feedback-paced homing and repeatable joint-space demos for a calibrated arm."""
import argparse
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

from eai_robot.config import ROOT, load_config
from .calibration import JOINTS, load
from .cli import DEFAULT_CALIBRATION, report_release
from .controller import ArmController

DEFAULT_HOME = ROOT / 'configs/demo_home.json'
# Match ArmController.feedback_slack for torque-off gravity drift at the entry boundary.
START_FEEDBACK_SLACK = 3
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


def check_status(backend):
    for sid in range(1, 7):
        status = backend.read(sid, 'Status')
        if status:
            raise RuntimeError(f'ID {sid} 状态寄存器报警：0x{status:02x}')


def run_stage(arm, item, max_step=20):
    """Bound intermediate lag to 25 counts; enforce final arrival separately."""
    current = arm.positions()
    arm._check_pose(current)
    origin = {cal.id: max(cal.range_min, min(cal.range_max, current[cal.id]))
              for cal in arm.calibration.values()}
    goal = item['targets']
    steps = max(1, math.ceil(max(abs(goal[s] - origin[s]) for s in goal) / max_step))
    item['substeps'] = []
    for index in range(1, steps + 1):
        check_status(arm.backend)
        target = {sid: round(origin[sid] + (goal[sid] - origin[sid]) * index / steps) for sid in goal}
        ratios = [arm.calibration[name].ratio(target[arm.calibration[name].id]) for name in JOINTS]
        entry = {'targets': target, 'reached': False}
        item['substeps'].append(entry)
        started = time.monotonic()
        arrival_tolerance = arm.tolerance
        arm.tolerance = arrival_tolerance if index == steps else max(arrival_tolerance, 25)
        entry['tolerance_counts'] = arm.tolerance
        try:
            _, feedback = arm.move(ratios)
        except Exception as failure:
            # Retain exact feedback and per-joint error for a failed substep.
            try:
                feedback = getattr(failure, 'feedback', None)
                entry['feedback_source'] = 'at_failure_before_release' if feedback is not None else 'after_release'
                if feedback is None:
                    feedback = arm.positions()
                entry['feedback'] = feedback
                entry['errors'] = {sid: feedback[sid] - target[sid] for sid in target}
            except Exception as exc:
                entry['read_error'] = str(exc)
            raise
        finally:
            arm.tolerance = arrival_tolerance
        entry.update(feedback=feedback, reached=True, duration_seconds=round(time.monotonic() - started, 3))
        check_status(arm.backend)
        time.sleep(.08)
    return feedback


def teach_home(path, arm, calibration, margin):
    if Path(path).exists():
        raise ValueError('示教文件已存在，请使用新路径保存')
    pose = arm.inspect()
    if any(arm.backend.read(sid, 'Torque_Enable') for sid in range(1, 7)):
        raise ValueError('请先支撑机械臂并关闭六关节扭矩，再手动摆好起点并示教')
    stage(calibration, '示教起点', pose, margin)
    data = {'schema_version': 1, 'description': '用户手动摆放后只读采集的固定起点；未认证避碰',
            'home_raw': [pose[s] for s in range(1, 7)],
            'start_radius_counts': [200, 110, 110, 100, 220, 200],
            'calibration': {n: asdict(calibration[n]) for n in JOINTS}}
    # Check all profiles before saving a home that cannot execute their offsets.
    home, radius = pose, dict(enumerate(data['start_radius_counts'], 1))
    for profile in PROFILES:
        build_plan(calibration, pose, home, radius, profile, margin)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open('x', encoding='utf-8') as output:
        output.write(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    print(f'已只读采集固定起点：{path}，没有启用扭矩。', flush=True)


def main(argv=None):
    cfg = load_config()
    parser = argparse.ArgumentParser(description='先反馈归位，再执行固定 demo；默认只读预览')
    parser.add_argument('--backend', choices=('serial', 'lerobot'), default='serial')
    parser.add_argument('--port', default=cfg['serial']['port'])
    parser.add_argument('--baudrate', type=int, default=cfg['serial']['baudrate'])
    parser.add_argument('--calibration', type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument('--home-file', type=Path, default=DEFAULT_HOME)
    parser.add_argument('--teach-home', type=Path, help='只读保存手动摆好的起点到新文件')
    parser.add_argument('--demo', choices=tuple(PROFILES), default='showcase')
    parser.add_argument('--margin-counts', type=int, default=20)
    parser.add_argument('--tolerance', type=int, default=15)
    parser.add_argument('--timeout', type=float, default=8, help='每个小段的到位等待上限')
    parser.add_argument('--hold', type=float, default=1.2, help='每个动作停留 0..5 秒')
    parser.add_argument('--enable', action='store_true')
    parser.add_argument('--anchor-raw', type=int, nargs=6, help='离线模拟不同起点，不打开串口')
    parser.add_argument('--log', type=Path, help='默认保存各 demo 独立日志')
    args = parser.parse_args(argv)
    if args.log is None:
        args.log = ROOT / f'outputs/assignment2/demo_{args.demo}_latest.json'
    backend = arm = None
    record = {'started_at': datetime.now().astimezone().isoformat(), 'backend': args.backend,
              'profile': args.demo, 'home_file': str(args.home_file), 'stages': [], 'completed': False,
              'arrival_tolerance_counts': args.tolerance, 'homed': False,
              'trajectory': 'feedback_paced_20_counts', 'intermediate_lag_limit_counts': 25,
              'power': 'existing power bank'}
    exit_code = 1
    try:
        if args.anchor_raw is not None and (args.enable or args.teach_home):
            raise ValueError('--anchor-raw 只用于离线预览')
        if args.teach_home and args.enable:
            raise ValueError('--teach-home 只读，不可同时 --enable')
        if not math.isfinite(args.hold) or not 0 <= args.hold <= 5:
            raise ValueError('--hold 必须为 0..5 秒')
        if not 0 <= args.tolerance <= 15:
            raise ValueError('演示容差必须为 0..15 刻度')
        calibration = load(args.calibration)
        arm = ArmController(None, calibration, interval=.08, tolerance=args.tolerance,
                            timeout=args.timeout, motion='direct', allow_wide_range=True)
        home, radius = (None, None) if args.teach_home else load_home(args.home_file, calibration, args.margin_counts)
        if args.anchor_raw is None:
            from .backends import SerialBackend, LeRobotBackend
            backend = (SerialBackend if args.backend == 'serial' else LeRobotBackend)(args.port, args.baudrate)
            arm.backend = backend
            start = arm.inspect()
        else:
            start = dict(enumerate(args.anchor_raw, 1))
        if args.teach_home:
            teach_home(args.teach_home, arm, calibration, args.margin_counts)
            return 0
        envelope = check_start(calibration, start, home, radius)
        plan = build_plan(calibration, start, home, radius, args.demo, args.margin_counts)
        record.update(initial_raw=start, home_raw=home, accepted_start_raw=envelope, plan=plan)
        print(f'读取当前姿态：{start}\n统一演示起点：{home}\n允许入口范围：{envelope}', flush=True)
        for index, item in enumerate(plan, 1):
            print(f'{index}. {item["name"]}：{list(item["targets"].values())}', flush=True)
        if not args.enable:
            print('只读预览完成；使用 --enable 执行。', flush=True)
            return 0
        check_status(backend)
        arm.enable()
        # Re-read after enable so any manual change between preview and torque-on
        # is checked, and the first homing targets hold the latest measured pose.
        actual_start = arm.positions()
        check_start(calibration, actual_start, home, radius)
        plan = build_plan(calibration, actual_start, home, radius, args.demo, args.margin_counts)
        record['enabled_initial_raw'] = actual_start
        record['plan'] = plan
        for index, item in enumerate(plan, 1):
            print(f'\n执行 {index}/{len(plan)}：{item["name"]}', flush=True)
            progress = {k: v for k, v in item.items()}
            record['stages'].append(progress)
            before = time.monotonic()
            feedback = run_stage(arm, progress)
            progress.update(feedback=feedback, reached=True,
                            duration_seconds=round(time.monotonic() - before, 3),
                            max_error_counts=max(abs(feedback[s] - item['targets'][s]) for s in feedback))
            print(f'到位反馈：{feedback}；最大误差 {progress["max_error_counts"]} 刻度', flush=True)
            if index == 2:
                record['homed'] = True
                print('统一起点已确认。', flush=True)
            time.sleep(args.hold)
        record['completed'] = True
        exit_code = 0
    except (KeyboardInterrupt, EOFError):
        record['error'] = 'interrupted'
        exit_code = 130
        print('演示已中断。', file=sys.stderr, flush=True)
    except Exception as exc:
        record['error'] = f'{type(exc).__name__}: {exc}'
        print(f'演示停止：{record["error"]}', file=sys.stderr, flush=True)
    finally:
        if arm is not None and backend is not None and args.enable and (arm.torque_touched or arm.enabled or record['stages']):
            failures = arm.disable()
            report_release(failures)
            record['torque_off_failures'] = failures
            if failures:
                exit_code = 1
            try:
                record['final_raw'] = arm.positions()
                record['torque_after_exit'] = {sid: backend.read(sid, 'Torque_Enable') for sid in range(1, 7)}
            except Exception as exc:
                record['final_read_error'] = str(exc)
                exit_code = 1
        if backend is not None:
            backend.close()
        if args.enable:
            record['finished_at'] = datetime.now().astimezone().isoformat()
            args.log.parent.mkdir(parents=True, exist_ok=True)
            args.log.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            print(f'演示记录：{args.log}', flush=True)
    return exit_code
