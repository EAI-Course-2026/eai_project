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
from eai_robot.calibration.servo.results import JOINTS, load
from eai_robot.control.planning.demo import DEFAULT_HOME, PROFILES, load_home, check_start, stage, build_plan
from eai_robot.control.execution.demo import check_status, run_stage
from eai_robot.applications.tools.arm import DEFAULT_CALIBRATION, report_release
from eai_robot.control.execution.arm import ArmController


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
    parser.add_argument('--backend', choices=('serial', 'lerobot'), default='lerobot')
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
            from eai_robot.hardware.drivers.backends import SerialBackend, LeRobotBackend
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
