"""Guarded, logged physical acceptance of the SCS215 candidate software range."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from eai_robot.config import ROOT, load_config
from eai_robot.arm.backends import SerialBackend, LeRobotBackend
from eai_robot.arm.calibration import JOINTS, load
from eai_robot.arm.controller import ArmController, inspect_hardware
from eai_robot.arm.demo import check_start, check_status, load_home

CANDIDATE = ROOT / 'calibration/scs215_safe_candidate_20260928.json'
CURRENT = ROOT / 'calibration/scs215_so101.json'
HOME = ROOT / 'configs/demo_home_scs215_safe_candidate_20260928.json'


def emit(handle, event, **data):
    item = {'time': datetime.now().astimezone().isoformat(), 'event': event, **data}
    handle.write(json.dumps(item, ensure_ascii=False) + '\n')
    handle.flush()
    print(event, {k:v for k,v in data.items() if k in ('joint','target','max_error','feedback','error')}, flush=True)


def move_pose(arm, target, label, log, max_step):
    start = arm.positions()
    if set(target) != set(range(1, 7)):
        raise ValueError('target must contain all six joint IDs')
    steps = max(1, math.ceil(max(abs(target[s]-start[s]) for s in target) / max_step))
    emit(log, 'stage_start', stage=label, from_raw=start, target=target, steps=steps)
    for index in range(1, steps+1):
        check_status(arm.backend)
        waypoint = {sid: round(start[sid] + (target[sid]-start[sid]) * index / steps)
                    for sid in range(1, 7)}
        ratios = [arm.calibration[name].ratio(waypoint[arm.calibration[name].id]) for name in JOINTS]
        goals, feedback = arm.move(ratios)
        error = max(abs(feedback[s]-goals[s]) for s in goals)
        emit(log, 'waypoint', stage=label, index=index, target=goals,
             feedback=feedback, max_error=error)
        check_status(arm.backend)
        time.sleep(.08)
    emit(log, 'stage_complete', stage=label, feedback=feedback, max_error=error)
    return feedback


def run(argv=None):
    parser=argparse.ArgumentParser(description='SCS215 候选软件校准逐关节实物验收')
    parser.add_argument('--backend',choices=('serial','lerobot'),default='serial')
    parser.add_argument('--port',default=load_config()['serial']['port'])
    parser.add_argument('--baudrate',type=int,default=load_config()['serial']['baudrate'])
    parser.add_argument('--joint',choices=('all',*JOINTS),default='all')
    parser.add_argument('--rounds',type=int,default=1)
    parser.add_argument('--max-step',type=int,default=20)
    parser.add_argument('--timeout',type=float,default=8.0)
    parser.add_argument('--enable',action='store_true')
    parser.add_argument('--return-home',action='store_true',
                        help='从已校准范围内小步返回固定起点，跳过演示入口检查与端点验收')
    parser.add_argument('--manual-start',action='store_true',
                        help='人工确认姿态离桌后，跳过固定演示入口；仍执行 EEPROM 邻域、回收距离和状态检查')
    parser.add_argument('--clearance-wrist',type=int,
                        help='先将 ID4 调至指定刻度，并保持到肘部回到起点后')
    parser.add_argument('--clearance-gripper',type=int,
                        help='先将 ID6 调至指定刻度，并保持到肘部回到起点后')
    parser.add_argument('--log',type=Path)
    args=parser.parse_args(argv)
    if not 1<=args.rounds<=2 or not 5<=args.max_step<=20 or not 0.5<=args.timeout<=10:
        parser.error('rounds 1..2, max-step 5..20, timeout 0.5..10')
    candidate=load(CANDIDATE)
    existing=load(CURRENT)
    home,radius=load_home(HOME,candidate,20)
    for sid,raw in ((4,args.clearance_wrist),(6,args.clearance_gripper)):
        if raw is not None:
            cal=candidate[JOINTS[sid-1]]
            if not cal.range_min<=raw<=cal.range_max:
                parser.error(f'ID {sid} 避碰刻度必须在 {cal.range_min}..{cal.range_max}')
    if args.log is None:
        args.log=ROOT/'outputs/assignment2'/f'candidate_verify_{datetime.now().strftime("%Y%m%d_%H%M%S")}.jsonl'
    backend=arm=None
    result=1
    log=None
    try:
        backend=(SerialBackend if args.backend=='serial' else LeRobotBackend)(args.port,args.baudrate)
        hardware=inspect_hardware(backend)
        arm=ArmController(backend,candidate,tolerance=15,timeout=args.timeout,
                          interval=.08,motion='direct',allow_wide_range=True,
                          progress_timeout=1.5)
        current=arm.positions()
        torque={sid:backend.read(sid,'Torque_Enable') for sid in range(1,7)}
        status={sid:backend.read(sid,'Status') for sid in range(1,7)}
        print('当前反馈',current,'硬件限位',hardware,'扭矩',torque,'状态',status,flush=True)
        if any(torque.values()) or any(status.values()):
            raise RuntimeError('验收前必须六台扭矩关闭且无状态报警')
        # Hardware-based start gate, distinct from the candidate 0..1 work range.
        if args.return_home or args.manual_start:
            entry={'mode':'return_home' if args.return_home else 'manual_start', 'start':current}
            print('人工确认起点／返回起点模式：跳过固定演示入口，仍检查 EEPROM 邻域和状态',flush=True)
        else:
            entry=check_start(existing,current,home,radius)
            print('硬件限位与固定起点入口通过',entry,flush=True)
        if not args.enable:
            print('只读预检完成；加 --enable 才会回收并运动。',flush=True)
            return 0
        args.log.parent.mkdir(parents=True,exist_ok=True)
        log=args.log.open('x',encoding='utf-8')
        emit(log,'preflight',start=current,hardware=hardware,torque=torque,status=status,
             candidate=str(CANDIDATE),joint=args.joint,rounds=args.rounds)
        clearance={sid:raw for sid,raw in ((4,args.clearance_wrist),
                                             (6,args.clearance_gripper)) if raw is not None}
        arm.recover_startup(max_overtravel=8,max_step=10,max_recovery=60,
                            clearance=clearance,
                            on_event=lambda item: emit(log,'startup_'+item['phase'],
                                                       **{k:v for k,v in item.items() if k!='phase'}))
        current=arm.positions()
        if not (args.return_home or args.manual_start):
            check_start(candidate,current,home,radius)
        emit(log,'recovered',feedback=current)
        # Align shoulder/elbow/wrist before base, wrist rotation and gripper.
        folded=arm.positions()
        for sid in (2,3):folded[sid]=home[sid]
        if args.clearance_wrist is None:folded[4]=home[4]
        move_pose(arm,folded,'home_proximal',log,args.max_step)
        work_home=dict(home)
        if not args.return_home:
            work_home.update(clearance)
        move_pose(arm,work_home,'home_distal',log,args.max_step)
        joints=() if args.return_home else (JOINTS if args.joint=='all' else (args.joint,))
        for name in joints:
            sid=candidate[name].id
            lo,hi=candidate[name].range_min,candidate[name].range_max
            near,far=sorted((lo,hi),key=lambda x:abs(x-work_home[sid]))
            for round_index in range(1,args.rounds+1):
                for edge,target_raw in (('near',near),('far',far)):
                    target=dict(work_home);target[sid]=target_raw
                    feedback=move_pose(arm,target,f'{name}_{edge}_{round_index}',log,args.max_step)
                    voltage={motor:backend.read(motor,'Present_Voltage')/10 for motor in range(1,7)}
                    emit(log,'endpoint',joint=name,edge=edge,round=round_index,
                         target=target_raw,feedback=feedback,voltage=voltage)
                    move_pose(arm,work_home,f'{name}_return_{edge}_{round_index}',log,args.max_step)
        if not args.return_home and work_home != home:
            move_pose(arm,home,'restore_original_home',log,args.max_step)
        result=0
        emit(log,'complete',joints=list(joints),rounds=args.rounds,feedback=arm.positions())
    except (KeyboardInterrupt,EOFError):
        result=130
        if log:emit(log,'interrupted')
    except Exception as exc:
        if log:
            emit(log,'failed',error=f'{type(exc).__name__}: {exc}',
                 feedback=getattr(exc,'feedback',None),
                 target=getattr(exc,'targets',None))
        print(f'验收停止：{type(exc).__name__}: {exc}',file=sys.stderr,flush=True)
    finally:
        if arm is not None and (arm.torque_touched or arm.enabled):
            try:
                failures=arm.disable()
                if log:emit(log,'torque_release',failures=failures)
                if failures:result=1
            except Exception as exc:
                result=1
                if log:emit(log,'torque_release_failed',error=str(exc))
        if backend is not None:
            try:
                final={sid:backend.read(sid,'Present_Position') for sid in range(1,7)}
                torque={sid:backend.read(sid,'Torque_Enable') for sid in range(1,7)}
                if log:emit(log,'final',feedback=final,torque=torque)
                if any(torque.values()):result=1
            except Exception as exc:
                if log:emit(log,'final_read_failed',error=str(exc))
                result=1
            backend.close()
        if log:
            log.close()
            print('验收日志',args.log,flush=True)
    return result


if __name__=='__main__':
    raise SystemExit(run())
