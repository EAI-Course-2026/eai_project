"""Single-owner hardware service for the local six-axis control desk.

All serial operations happen on one worker. HTTP threads only enqueue jobs and
read snapshots; stop is an event observed by the worker between servo reads.
"""
from __future__ import annotations

from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread
import time
import secrets

import numpy as np

from eai_robot.config import ROOT
from .calibration import JOINTS, JointCalibration, load, save
from .controller import ArmController, MotionPaused, inspect_hardware
from .demo import DEFAULT_HOME, PROFILES, build_plan, check_start, load_home, run_stage, teach_home

from .desk_cartesian import DeskCartesian, PERIOD, LEASE_SECONDS, vector, speed

BAUDRATES = (1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200)
MARGIN = 20


class MotionCancelled(Exception):
    pass


class InterruptibleBackend:
    def __init__(self, backend, stop_event):
        self.backend, self.stop_event = backend, stop_event

    def read(self, sid, register):
        if register == 'Present_Position' and self.stop_event.is_set():
            raise MotionCancelled('已请求停止运动')
        return self.backend.read(sid, register)

    def write(self, sid, register, value):
        if self.stop_event.is_set() and not (register == 'Torque_Enable' and value == 0):
            raise MotionCancelled('已请求停止运动')
        return self.backend.write(sid, register, value)

    def sync_positions(self, targets):
        if self.stop_event.is_set():
            raise MotionCancelled('已请求停止运动')
        return self.backend.sync_positions(targets)

    def close(self):
        return self.backend.close()


class ArmDeskService:
    def __init__(self, calibration_path, home_path=DEFAULT_HOME, backend_factory=None,
                 idle_poll_seconds=1.5, watchdog_seconds=8.0):
        self.calibration_path, self.home_path = Path(calibration_path), Path(home_path)
        self.calibration = load(self.calibration_path)
        try:
            self.home, self.radius = load_home(self.home_path, self.calibration, MARGIN)
            home_required = False
        except (FileNotFoundError, ValueError):
            # A newly captured calibration intentionally invalidates an older
            # home. Manual control remains independent of the optional home.
            self.home = self.radius = None
            home_required = True
        self.backend_factory = backend_factory or self._backend_factory
        self.idle_poll_seconds = idle_poll_seconds
        self.watchdog_seconds = watchdog_seconds
        self.lock = Lock()
        self.queue = Queue()
        self.stop_event = Event()
        self.pause_event = Event()
        self.stream = None
        self.cancelled_owners = {}
        self.preview = None
        self.cartesian = DeskCartesian(self.calibration, MARGIN)
        self.shutdown_event = Event()
        self.backend = None
        self.arm = None
        self.homed = False
        self.last_client_seen = time.monotonic()
        self.captures = {name: [None, None] for name in JOINTS}
        self.state = {'connected': False, 'busy': False, 'stopping': False,
                      'homed': False, 'torque_on': False, 'control_enabled': False,
                      'motion_state': 'idle', 'targets': None,
                      'cartesian': {'frame': 'URDF base', 'tcp': 'gripper_frame_link',
                                    'actual': None, 'command_mm': None, 'requested_mm': None,
                                    'tracking_error_mm': None, 'joint_error_deg': None,
                                    'preview': None, 'stream_active': False, 'feedback_at': None,
                                    'notice': '只控制 XYZ；末端朝向可能变化。'},
                      'home_required': home_required,
                      'status': '等待连接',
                      'error': None, 'positions': None, 'joints': [], 'events': [],
                      'captures': self.captures, 'profile': None,
                      'port': '', 'backend': 'serial'}
        self.worker = Thread(target=self._loop, name='arm-desk-hardware', daemon=True)
        self.worker.start()
        self.watchdog = Thread(target=self._watchdog_loop, name='arm-desk-watchdog', daemon=True)
        self.watchdog.start()

    @staticmethod
    def _backend_factory(kind, port, baudrate):
        from .backends import SerialBackend, LeRobotBackend
        return (SerialBackend if kind == 'serial' else LeRobotBackend)(port, baudrate)

    def _event(self, message):
        with self.lock:
            events = self.state['events']
            events.append({'time': datetime.now().strftime('%H:%M:%S'), 'message': str(message)})
            del events[:-80]
            self.state['status'] = str(message)

    def _set(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            self.last_client_seen = time.monotonic()
            result = json.loads(json.dumps(self.state, ensure_ascii=False))
        result['calibration'] = [
            {'id': self.calibration[name].id, 'name': name,
             'allowed_min': min(self.calibration[name].ratio(self._manual_limits(name)[0]), self.calibration[name].ratio(self._manual_limits(name)[1])),
             'allowed_max': max(self.calibration[name].ratio(self._manual_limits(name)[0]), self.calibration[name].ratio(self._manual_limits(name)[1])),
             'min': self.calibration[name].range_min, 'max': self.calibration[name].range_max,
             'reverse': bool(self.calibration[name].drive_mode),
             'safe_min': self.calibration[name].position(0, MARGIN),
             'safe_max': self.calibration[name].position(1, MARGIN)}
            for name in JOINTS]
        result['home'] = [self.home[s] for s in range(1, 7)] if self.home else None
        result['profiles'] = list(PROFILES)
        return result

    def _cart_set(self, **values):
        with self.lock:
            self.state['cartesian'].update(values)

    def _motion_guard(self):
        if self.stop_event.is_set():
            raise MotionCancelled('已请求释放扭矩')
        if self.pause_event.is_set():
            raise MotionPaused('已暂停运动；保持扭矩')

    def _manual_limits(self, name):
        return self.calibration[name].bounds(MARGIN)

    def _targets(self, values, joints=None):
        if not isinstance(values, list) or len(values) != 6 or any(
                type(v) not in (float, int) or not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ValueError('必须输入六个 0..1 的有限数值')
        selected = list(JOINTS) if joints is None else joints
        if (not isinstance(selected, list) or not selected or
                any(not isinstance(name, str) or name not in JOINTS for name in selected) or
                len(set(selected)) != len(selected)):
            raise ValueError('请选择至少一个要调整的关节')
        targets = {}
        for name, value in zip(JOINTS, values, strict=True):
            if name not in selected:
                continue
            cal = self.calibration[name]
            low, high = cal.bounds(MARGIN)
            raw = cal.position(value)
            if not low <= raw <= high:
                raise ValueError(f'{name} 目标 {raw} 超出普通控制范围 {low}..{high}；标定范围 {cal.range_min}..{cal.range_max}')
            targets[cal.id] = raw
        return targets

    def _jog_target(self, name, current, delta):
        cal = self.calibration[name]
        low, high = cal.bounds(MARGIN)
        target = current + delta
        inward = (cal.range_min <= current < low and delta > 0 and target <= high or
                  high < current <= cal.range_max and delta < 0 and target >= low)
        if not (low <= target <= high or inward):
            raise ValueError(f'{name} 当前 {current}，点动目标 {target}；下发范围 {low}..{high}。端点余量区仅允许向区间内部点动')
        return target

    def _check_home_start(self, positions):
        try:
            check_start(self.calibration, positions, self.home, self.radius)
        except ValueError as exc:
            raise ValueError(str(exc).replace('本演示入口', '归位入口（仅用于归位/演示）')) from exc

    def submit(self, action, payload=None):
        payload = {} if payload is None else payload
        if action not in {'connect', 'disconnect', 'enable_control', 'jog', 'home', 'move', 'demo', 'stop',
                          'capture', 'save_calibration', 'teach_home', 'pause', 'cartesian_preview',
                          'cartesian_execute', 'cartesian_step', 'cartesian_start', 'cartesian_intent'}:
            raise ValueError('未知操作')
        if not isinstance(payload, dict):
            raise ValueError('操作参数必须为对象')
        if action == 'cartesian_intent':
            direction = vector(payload.get('direction'), '方向', 1)
            velocity = speed(payload.get('speed'))
            sequence = payload.get('sequence')
            if type(sequence) is not int or sequence < 0:
                raise ValueError('连续控制序号无效')
            with self.lock:
                stream = self.stream
                if (stream is None or stream['owner'] != payload.get('owner') or
                        time.monotonic() >= stream['expires'] or self.pause_event.is_set() or
                        self.stop_event.is_set()):
                    raise RuntimeError('连续控制已停止；请松开并重新按下')
                if sequence <= stream['sequence']:
                    raise RuntimeError('已丢弃过期方向输入')
                stream.update(direction=direction, speed=velocity, sequence=sequence,
                              expires=time.monotonic() + LEASE_SECONDS)
            return
        if action == 'pause':
            with self.lock:
                if self.state['stopping']:
                    return
                if not self.state['connected'] or not self.state['control_enabled']:
                    return
                owner = payload.get('owner') or (self.stream or {}).get('owner')
                if isinstance(owner, str) and 1 <= len(owner) <= 80:
                    self.cancelled_owners[owner] = time.monotonic() + 30
                self.pause_event.set()
                self.stream = None
                if self.state['busy']:
                    return
                self.state['busy'] = True
            self.queue.put((action, payload))
            return
        if action == 'stop':
            with self.lock:
                self.stream = None
                self.preview = None
                self.state['stopping'] = True
                self.state['busy'] = True
                self.state['status'] = '正在停止并关闭扭矩…'
            self.stop_event.set()
            self.pause_event.set()
            self.queue.put((action, payload))
            return
        # Validate untrusted input before it reaches the hardware worker.
        if action == 'connect':
            kind, port, baudrate = payload.get('backend'), payload.get('port'), payload.get('baudrate')
            if kind not in ('serial', 'lerobot') or not isinstance(port, str) or not port.strip() or len(port) > 256:
                raise ValueError('请选择后端并填写串口')
            if type(baudrate) is not int or baudrate not in BAUDRATES:
                raise ValueError('波特率无效')
        elif action in ('cartesian_preview', 'cartesian_step', 'cartesian_start'):
            payload = dict(payload)
            payload['speed'] = speed(payload.get('speed', 15))
            if action == 'cartesian_preview':
                payload['xyz'] = vector(payload.get('xyz'), 'XYZ', 1000).tolist()
            else:
                direction = vector(payload.get('direction'), '方向', 1)
                if not np.linalg.norm(direction):
                    raise ValueError('请选择移动方向')
                payload['direction'] = (direction / np.linalg.norm(direction)).tolist()
                if action == 'cartesian_step':
                    if payload.get('distance') not in (1, 5, 10) or type(payload.get('distance')) not in (int, float):
                        raise ValueError('单步距离仅支持 1、5、10 mm')
                elif (not isinstance(payload.get('owner'), str) or not 1 <= len(payload['owner']) <= 80 or
                      type(payload.get('sequence')) is not int or payload['sequence'] < 0):
                    raise ValueError('连续控制会话无效')
        elif action == 'cartesian_execute':
            if not isinstance(payload.get('preview_id'), str):
                raise ValueError('请先预览整条路径')
        elif action == 'move':
            self._targets(payload.get('values'), payload.get('joints'))
        elif action == 'jog':
            if payload.get('joint') not in JOINTS or type(payload.get('delta')) is not int or payload['delta'] not in (-5, 5):
                raise ValueError('点动仅支持指定关节 ±5 刻度')
        elif action == 'demo' and (payload.get('profile') not in PROFILES or payload.get('profile') == 'home'):
            raise ValueError('请选择演示版本')
        elif action == 'capture':
            if payload.get('joint') not in JOINTS or type(payload.get('endpoint')) is not int or payload['endpoint'] not in (0, 1):
                raise ValueError('请选择关节与 0/1 端点')
        with self.lock:
            if self.state['busy'] or self.state['stopping']:
                raise RuntimeError('当前任务正在执行，请先等待或停止')
            if action == 'connect' and self.state['connected']:
                raise RuntimeError('已经连接，请先断开')
            if action != 'connect' and not self.state['connected']:
                raise RuntimeError('请先连接机械臂')
            if action in ('home', 'demo') and self.home is None:
                raise RuntimeError('当前校准尚无匹配起点，请先示教')
            if action in ('move', 'jog', 'cartesian_start', 'cartesian_step', 'cartesian_execute') and not self.state['control_enabled']:
                raise RuntimeError('请先从当前姿态启用控制；停止后需重新启用')
            if action == 'jog':
                cal = self.calibration[payload['joint']]
                self._jog_target(payload['joint'], self.state['positions'][cal.id], payload['delta'])
            if action == 'enable_control' and self.state['control_enabled']:
                raise RuntimeError('控制已经启用')
            if action in ('home', 'demo') and self.state['positions'] is not None:
                self._check_home_start(self.state['positions'])
            if action in ('capture', 'save_calibration', 'teach_home') and self.state['torque_on']:
                raise RuntimeError('先停止并关闭扭矩，再采集校准')
            self.pause_event.clear()
            if action == 'cartesian_start':
                now = time.monotonic()
                self.cancelled_owners = {o: t for o, t in self.cancelled_owners.items() if t > now}
                if payload['owner'] in self.cancelled_owners:
                    raise RuntimeError('此连续控制会话已取消；请重新按下')
                if len(self.cancelled_owners) > 128:
                    self.cancelled_owners.pop(next(iter(self.cancelled_owners)))
                self.stream = dict(owner=payload['owner'], sequence=payload['sequence'],
                                   direction=np.array(payload['direction']), speed=payload['speed'],
                                   expires=time.monotonic() + LEASE_SECONDS)
            if action not in ('cartesian_preview', 'cartesian_execute'):
                self.preview = None
                self.state['cartesian']['preview'] = None
            self.state['busy'] = True
            self.state['error'] = None
            self.state['status'] = '正在执行…'
            if action in ('move', 'jog', 'home', 'demo', 'cartesian_start', 'cartesian_step', 'cartesian_execute'):
                self.state['motion_state'] = 'moving'
        self.queue.put((action, payload))

    def _loop(self):
        while not self.shutdown_event.is_set():
            try:
                action, payload = self.queue.get(timeout=self.idle_poll_seconds)
            except Empty:
                self._idle()
                continue
            try:
                if action == 'stop':
                    self._stop()
                    self.stop_event.clear()
                    self.pause_event.clear()
                    self._set(motion_state='stopped')
                    self._event('已停止，六关节扭矩已释放')
                else:
                    if self.stop_event.is_set():
                        raise MotionCancelled('已请求停止运动')
                    self._dispatch(action, payload)
            except MotionPaused as exc:
                try:
                    self._hold(str(exc))
                except Exception as hold_error:
                    self._set(error=f'保持失败：{hold_error}', motion_state='failed')
                    try:
                        self._stop()
                    except Exception as release_error:
                        self._event(f'扭矩关闭未确认：{release_error}')
            except Exception as exc:
                cancelled = isinstance(exc, MotionCancelled)
                self._set(error=None if cancelled else f'{type(exc).__name__}: {exc}', homed=False,
                          motion_state='stopped' if cancelled else 'failed')
                self.homed = False
                self._event(f'操作停止：{type(exc).__name__}: {exc}')
                try:
                    self._stop()
                except Exception as release_error:
                    self._event(f'扭矩关闭未确认：{release_error}；请切断舵机电源')
            finally:
                # A stop may have arrived while an operation was finishing.
                if action != 'stop' and self.stop_event.is_set():
                    try:
                        self._stop()
                    except Exception as exc:
                        self._set(error=f'关闭扭矩失败：{exc}')
                if self.pause_event.is_set() and not self.stop_event.is_set():
                    try:
                        if self.arm and self.arm.enabled:
                            self._hold('已暂停运动；保持扭矩')
                        else:
                            self.pause_event.clear()
                    except Exception as exc:
                        self._set(error=f'暂停失败：{exc}')
                        self._stop()
                if action in ('cartesian_start', 'cartesian_step', 'cartesian_execute'):
                    with self.lock:
                        self.stream = None
                    self._cart_set(stream_active=False)
                with self.lock:
                    if action == 'stop':
                        self.state['busy'] = False
                        self.state['stopping'] = False
                    elif self.state['stopping']:
                        self.state['busy'] = True
                    else:
                        self.state['busy'] = False
                self.queue.task_done()

    def _dispatch(self, action, payload):
        if action == 'connect':
            kind, port, baudrate = payload['backend'], payload['port'].strip(), payload['baudrate']
            backend = self.backend_factory(kind, port, baudrate)
            try:
                arm = ArmController(InterruptibleBackend(backend, self.stop_event), self.calibration,
                                    tolerance=15, timeout=8, motion='direct', allow_wide_range=True)
                arm.motion_guard = self._motion_guard
                arm.inspect()
                self.backend, self.arm = backend, arm
                self._set(connected=True, port=port, backend=kind)
                self._refresh()
                if self.state['torque_on']:
                    raise RuntimeError('连接时发现扭矩已开启；先停止并释放')
                self._event('六台 SCS215 已验证连接；当前仅只读')
            except BaseException:
                if self.backend is None:
                    backend.close()
                raise
        elif action == 'disconnect':
            self._stop()
            if self.backend is not None:
                self.backend.close()
            self.backend = self.arm = None
            self._set(connected=False, positions=None, joints=[], torque_on=False, homed=False, control_enabled=False, targets=None)
            self._cart_set(actual=None, command_mm=None, requested_mm=None, feedback_at=None)
            self._event('已断开串口，扭矩关闭')
        elif action == 'pause':
            self._hold('已暂停运动；保持扭矩')
        elif action == 'cartesian_preview':
            self._refresh()
            start = self.arm.positions()
            try:
                plan = self.cartesian.plan(start, payload['xyz'], payload['speed'], self._motion_guard)
            except ValueError as exc:
                self.preview = None
                self._cart_set(preview=None, notice=str(exc))
                self._event(f'预览未通过：{exc}；未启动运动')
                return
            summary = self.cartesian.summary(plan, start, payload['speed'])
            identifier = secrets.token_hex(16)
            self.preview = dict(id=identifier, plan=plan, start=start, speed=payload['speed'],
                                expires=time.monotonic() + 30)
            self._cart_set(preview=dict(summary, id=identifier, expires_at=time.time() + 30))
            self._event(summary['message'])
        elif action == 'cartesian_execute':
            preview = self.preview
            if not preview or preview['id'] != payload['preview_id'] or time.monotonic() >= preview['expires']:
                raise MotionPaused('预览已过期，请重新预览')
            self._refresh()
            start = self.arm.positions()
            if max(abs(start[s] - preview['start'][s]) for s in start) > 2:
                raise MotionPaused('姿态已变化，请重新预览路径')
            if not preview['plan'].reached_target:
                raise MotionPaused('整条路径不可达，未启动运动')
            self.preview = None
            self._cart_set(preview=None)
            self._execute_cartesian(preview['plan'], start, preview['speed'])
        elif action == 'cartesian_step':
            self._refresh()
            start = self.arm.positions()
            target = np.array(self.cartesian.pose(start)['xyz_mm']) + np.array(payload['direction']) * payload['distance']
            try:
                plan = self.cartesian.plan(start, target, payload['speed'], self._motion_guard)
            except ValueError as exc:
                raise MotionPaused(str(exc)) from exc
            if not plan.reached_target:
                raise MotionPaused('此方向路径不可达；请反向移动或调整关节姿态')
            self._execute_cartesian(plan, start, payload['speed'])
        elif action == 'cartesian_start':
            self._continuous_cartesian()
        elif action == 'home':
            self._home()
            self._refresh()
        elif action == 'enable_control':
            self._refresh()
            positions = self.arm.inspect()
            for name, cal in self.calibration.items():
                raw = positions[cal.id]
                if not cal.range_min <= raw <= cal.range_max:
                    raise RuntimeError(f'{name} 当前 {raw} 不在接管范围 {cal.range_min}..{cal.range_max}；保持支撑并在扭矩关闭时手动调整')
            self.arm.enable()
            self._refresh()
            self._set(control_enabled=True, torque_on=True,
                      targets={sid:self.backend.read(sid, 'Goal_Position') for sid in range(1, 7)},
                      motion_state='holding')
            self._cart_set(command_mm=self.cartesian.pose(self.state['targets'])['xyz_mm'],
                           tracking_error_mm=0., joint_error_deg=0., notice='已接管：按住方向按钮，或开启键盘控制；只控制 XYZ')
            self._event('已从当前姿态接管；普通控制无需归位')
        elif action in ('move', 'jog'):
            if not self.arm.enabled:
                raise RuntimeError('请重新启用控制')
            if action == 'move':
                targets = self.arm.positions()
                targets.update(self._targets(payload['values'], payload.get('joints')))
            else:
                # Point jogging starts from measured feedback, never unsent UI targets.
                targets = self.arm.positions()
                cal = self.calibration[payload['joint']]
                targets[cal.id] = self._jog_target(payload['joint'], targets[cal.id], payload['delta'])
            self._set(targets=targets, homed=False)
            self.homed = False
            self._event('正在分段执行关节目标…')
            run_stage(self.arm, {'name': '关节控制', 'targets': targets}, on_progress=self._motion_refresh)
            if action == 'jog':
                self._wait_jog(cal.id, targets[cal.id])
            self._refresh()
            self._set(motion_state='reached')
            self._event('目标已到位；扭矩保持，停止按钮可释放')
        elif action == 'demo':
            profile = payload['profile']
            self._home()
            start = self.arm.positions()
            plan = build_plan(self.calibration, start, self.home, self.radius, profile, MARGIN)
            self._set(profile=profile)
            for index, item in enumerate(plan[2:], 1):
                if self.stop_event.is_set():
                    raise MotionCancelled('已请求停止')
                self._event(f'{profile} · {index}/{len(plan)-2} · {item["name"]}')
                self._set(targets=item['targets'], motion_state='moving')
                run_stage(self.arm, item, on_progress=self._motion_refresh)
                self._refresh()
                self._wait_hold(1.2)
            self._set(motion_state='reached', targets=dict(self.home), homed=True)
            self.homed = True
            self._event(f'{profile} 演示完成，已返回统一起点')
        elif action == 'capture':
            if self.arm.enabled:
                raise RuntimeError('先停止并关闭扭矩')
            sid = JOINTS.index(payload['joint']) + 1
            if self.backend.read(sid, 'Torque_Enable') != 0:
                raise RuntimeError(f'ID {sid} 扭矩仍开启，不采集端点')
            value = self.backend.read(sid, 'Present_Position')
            low = self.backend.read(sid, 'Min_Position_Limit')
            high = self.backend.read(sid, 'Max_Position_Limit')
            if not low <= value <= high:
                raise ValueError(f'ID {sid} 反馈 {value} 超出硬件限位')
            self.captures[payload['joint']][payload['endpoint']] = value
            self._set(captures=self.captures)
            self._event(f'ID {sid} {payload["joint"]} · 端点 {payload["endpoint"]} = {value}')
        elif action == 'save_calibration':
            self._save_calibration()
        elif action == 'teach_home':
            if self.arm.enabled:
                raise RuntimeError('先停止并关闭扭矩')
            path = ROOT / 'configs' / f'demo_home_{datetime.now().strftime("%Y%m%d_%H%M%S")}.local.json'
            teach_home(path, self.arm, self.calibration, MARGIN)
            self.home, self.radius = load_home(path, self.calibration, MARGIN)
            self.home_path = path
            self._set(home_required=False)
            self._refresh()
            self._event(f'已保存新起点：{path}；重新启动时用 --home-file 指定')
        else:
            raise ValueError('未知操作')

    def _motion_refresh(self):
        self._motion_guard()
        self._refresh()

    def _hold(self, message):
        if not self.arm or not self.arm.enabled:
            self.pause_event.clear()
            self._set(motion_state='stopped')
            self._cart_set(notice='运动已取消；尚未接管')
            return
        with self.lock:
            self.stream = None
            self.preview = None
        self._refresh()
        raw = self.arm.positions()
        self.arm._check_pose(raw)
        goals = {c.id: max(c.range_min, min(c.range_max, raw[c.id])) for c in self.calibration.values()}
        self.arm.backend.sync_positions(goals)
        self.arm._verify_goals(goals)
        self._set(targets=goals, motion_state='paused', homed=False)
        self._cart_set(command_mm=self.cartesian.pose(goals)['xyz_mm'], requested_mm=None,
                       preview=None, stream_active=False, notice=message)
        self.pause_event.clear()
        self._refresh()
        self._event(message)

    def _cart_feedback(self, command_q):
        self._motion_guard()
        raw = self.arm.positions()
        self.arm._check_pose(raw)
        q = self.cartesian.angles(raw)
        # Check torque/faults for every tick; a partial enable never counts as control.
        for sid in range(1, 7):
            if self.backend.read(sid, 'Torque_Enable') != 1 or self.backend.read(sid, 'Status'):
                raise RuntimeError(f'ID {sid} 扭矩丢失或状态报警')
        error = float(np.max(np.abs(q - command_q)))
        self._cart_set(actual=self.cartesian.pose(raw), feedback_at=time.monotonic(), joint_error_deg=error)
        goal_counts = {self.calibration[n].id: self.calibration[n].position(
            (self.cartesian.mapper.urdf_degrees_to_normalized(n, float(a), clip=False) + 100) / 200)
            for n, a in zip(JOINTS[:5], command_q, strict=True)}
        lag = max(abs(raw[s] - goal_counts[s]) for s in goal_counts)
        if error > 12 or lag > 25:
            raise MotionPaused('关节跟随误差超过 12° 或 25 刻度；已暂停，目标不再累积')
        return raw

    def _send_cartesian(self, q, previous, lease=False):
        self._motion_guard()
        if lease:
            self._stream_intent()
        goals = self.cartesian.raw_targets(q, previous)
        self.arm.backend.sync_positions(goals)
        self.arm._verify_goals(goals)
        command = (self.cartesian.fk.forward_kinematics(q)[:3, 3] * 1000).tolist()
        self._set(targets=goals, homed=False, motion_state='moving')
        actual = self.state['cartesian']['actual']
        self._cart_set(command_mm=command, tracking_error_mm=float(np.linalg.norm(np.array(actual['xyz_mm']) - command)))
        return goals

    def _stream_intent(self):
        self._motion_guard()
        with self.lock:
            intent = self.stream
            if intent is None or time.monotonic() >= intent['expires']:
                raise MotionPaused('方向输入已结束或超过 300 ms 未续期；保持当前位置')
            return dict(intent)

    def _continuous_cartesian(self):
        self._stream_intent()
        self._refresh()
        raw = self.arm.positions()
        q = self.cartesian.angles(raw)
        command = self.cartesian.fk.forward_kinematics(q)[:3, 3]
        planner = self.cartesian.planner(raw, 30)
        velocity = np.zeros(3)
        previous = dict(raw)
        self._cart_set(stream_active=True, requested_mm=None, notice='按住移动，松开暂停；只控制 XYZ')
        self._event('末端连续控制已开始')
        last_telemetry = time.monotonic()
        while True:
            tick = time.monotonic()
            intent = self._stream_intent()
            self._cart_feedback(q)
            direction = intent['direction']
            norm = float(np.linalg.norm(direction))
            desired = direction / norm * intent['speed'] if norm else np.zeros(3)
            delta = desired - velocity
            velocity += delta * min(1., 60 * PERIOD / max(float(np.linalg.norm(delta)), 1e-9))
            result = planner.plan(q, command + velocity * PERIOD / 1000, on_step=self._motion_guard)
            if not result.reached_target:
                raise MotionPaused('此方向接近关节边界或直线不可达；可反向移动或调整关节姿态')
            if time.monotonic() - tick > LEASE_SECONDS:
                raise MotionPaused('求解或反馈耗时过长；已暂停，请降低速度或调整姿态')
            if result.waypoints:
                q_next = result.waypoints[-1].joint_degrees
                try:
                    previous = self._send_cartesian(q_next, previous, lease=True)
                except ValueError as exc:
                    raise MotionPaused(str(exc)) from exc
                q = q_next
                command = result.waypoints[-1].achieved_position
            if time.monotonic() - last_telemetry >= .5:
                self._refresh()
                last_telemetry = time.monotonic()
            self.pause_event.wait(max(0, PERIOD - (time.monotonic() - tick)))

    def _execute_cartesian(self, plan, start, velocity):
        self._motion_guard()
        q = self.cartesian.angles(start)
        previous = dict(start)
        self._cart_set(requested_mm=(plan.requested_target * 1000).tolist(), notice='正在执行直线；末端朝向可能变化')
        self._event('完整路径预检通过，正在执行末端直线')
        for index, waypoint in enumerate(plan.waypoints):
            tick = time.monotonic()
            self._cart_feedback(q)
            if time.monotonic() - tick > LEASE_SECONDS:
                raise MotionPaused('反馈耗时过长；已暂停运动')
            try:
                previous = self._send_cartesian(waypoint.joint_degrees, previous)
            except ValueError as exc:
                raise MotionPaused(str(exc)) from exc
            q = waypoint.joint_degrees
            ramp = min(1., (index + 1) / 4, (len(plan.waypoints) - index) / 4)
            self.pause_event.wait(max(0, PERIOD / ramp - (time.monotonic() - tick)))
        deadline = time.monotonic() + self.arm.timeout
        while True:
            raw = self._cart_feedback(q)
            actual_mm = np.array(self.cartesian.pose(raw)['xyz_mm'])
            error_mm = float(np.linalg.norm(actual_mm - plan.requested_target * 1000))
            self._cart_set(tracking_error_mm=error_mm)
            if max(abs(raw[s] - previous[s]) for s in range(1, 6)) <= 4 and error_mm <= 3:
                break
            if time.monotonic() >= deadline:
                raise MotionPaused('末端未在时限内到位；已暂停并保持，目标不再累积')
            self.pause_event.wait(PERIOD)
        self._refresh()
        self._set(motion_state='reached')
        self._cart_set(notice='末端已到位；扭矩保持')
        self._event('末端已到位；扭矩保持')

    def _wait_jog(self, sid, target):
        # A 5-count jog needs tighter arrival confirmation for the moved axis;
        # stationary axes keep their ordinary tolerance for gravity/encoder drift.
        deadline = time.monotonic() + self.arm.timeout
        while True:
            if self.stop_event.is_set():
                raise MotionCancelled('已请求停止运动')
            self._motion_refresh()
            with self.lock:
                positions = dict(self.state['positions'])
            self.arm._check_pose(positions)
            if abs(positions[sid] - target) <= 2:
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f'ID {sid} 点动未到位：目标 {target}，反馈 {positions[sid]}，容差 2 刻度')
            self.stop_event.wait(self.arm.interval)

    def _home(self):
        current = self.arm.inspect()
        self._check_home_start(current)
        if not self.arm.enabled:
            self.arm.enable()
            self._set(torque_on=True, control_enabled=True)
        current = self.arm.positions()
        self._check_home_start(current)
        plan = build_plan(self.calibration, current, self.home, self.radius, 'home', MARGIN)
        self.homed = False
        self._set(homed=False)
        for item in plan:
            if self.stop_event.is_set():
                raise MotionCancelled('已请求停止')
            self._event(item['name'])
            self._set(targets=item['targets'])
            run_stage(self.arm, item, on_progress=self._motion_refresh)
            self._refresh()
        self.homed = True
        self._set(homed=True, torque_on=True, control_enabled=True, targets=dict(self.home), motion_state='reached')
        self._event('已到达统一起点；普通控制保持启用')

    def _wait_hold(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self._motion_guard()
            if self.pause_event.wait(min(.1, deadline-time.monotonic())):
                self._motion_guard()
            if self.stop_event.is_set():
                raise MotionCancelled('已请求停止')

    def _save_calibration(self):
        if self.arm.enabled:
            raise RuntimeError('先停止并关闭扭矩')
        result = {}
        limits = inspect_hardware(self.backend)
        for sid, name in enumerate(JOINTS, 1):
            zero, one = self.captures[name]
            if zero is None or one is None or abs(one-zero) < 20:
                raise ValueError(f'{name} 尚未采集两个相距至少 20 刻度的端点')
            lo, hi = sorted((zero, one))
            hw_lo, hw_hi = limits[sid]
            if lo < hw_lo or hi > hw_hi:
                raise ValueError(f'{name} 端点超出硬件限位')
            result[name] = JointCalibration(sid, int(zero > one), 0, lo, hi)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = ROOT / 'calibration' / f'scs215_manual_{stamp}.json'
        if path.exists():
            raise FileExistsError(path)
        save(path, result)
        path.with_suffix('.meta.json').write_text(json.dumps({
            'method': 'manual_endpoints_gui', 'captured': datetime.now().astimezone().isoformat(),
            'model_number': 1315, 'manual_endpoints_verified': True,
            'note': 'User-selected software endpoints; EEPROM unchanged.'}, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        self._event(f'已另存校准：{path}；使用前需示教匹配的新起点')

    def _refresh(self):
        if self.backend is None:
            return
        joints = []
        positions = {}
        for sid, name in enumerate(JOINTS, 1):
            raw = self.backend.read(sid, 'Present_Position')
            if type(raw) is not int or not 0 <= raw <= 1023:
                raise RuntimeError(f'ID {sid} 反馈无效：{raw}')
            positions[sid] = raw
            joints.append({'id': sid, 'name': name, 'raw': raw,
                           'ratio': self.calibration[name].ratio(raw),
                           'voltage': self.backend.read(sid, 'Present_Voltage')/10,
                           'temperature': self.backend.read(sid, 'Present_Temperature'),
                           'status': self.backend.read(sid, 'Status'),
                           'torque': self.backend.read(sid, 'Torque_Enable')})
        self._set(positions=positions, joints=joints,
                  torque_on=any(j['torque'] for j in joints))
        try:
            actual = self.cartesian.pose(positions)
            self._cart_set(actual=actual, feedback_at=time.monotonic())
            targets = self.state['targets']
            if targets and self.state['control_enabled']:
                command = self.cartesian.pose(targets)['xyz_mm']
                self._cart_set(command_mm=command,
                    tracking_error_mm=float(np.linalg.norm(np.array(actual['xyz_mm']) - command)),
                    joint_error_deg=float(np.max(np.abs(self.cartesian.angles(positions) - self.cartesian.angles(targets)))))
        except ValueError as exc:
            self._cart_set(actual=None, notice=f'模型反馈不可用：{exc}')
        faults = [f'ID {j["id"]}: 0x{j["status"]:02x}' for j in joints if j['status']]
        if faults:
            raise RuntimeError('舵机状态报警：'+', '.join(faults))
        if self.arm is not None and self.arm.enabled and any(j['torque'] != 1 for j in joints):
            raise RuntimeError('控制中发现部分关节扭矩关闭，请重新接管')

    def _stop(self):
        with self.lock:
            self.stream = None
            self.preview = None
        self._cart_set(stream_active=False, preview=None, requested_mm=None, command_mm=None,
                       tracking_error_mm=None, joint_error_deg=None, notice='扭矩已释放；开始控制后才能移动')
        failures = []
        if self.arm is not None and self.arm.torque_touched or self.arm is not None and self.arm.enabled:
            failures = self.arm.disable()
        elif self.backend is not None:
            for sid in range(1, 7):
                try:
                    if self.backend.read(sid, 'Torque_Enable'):
                        self.backend.write(sid, 'Torque_Enable', 0)
                except Exception as exc:
                    failures.append(f'ID {sid}: {exc}')
        self.homed = False
        self._set(homed=False, torque_on=bool(failures), control_enabled=False)
        if failures:
            raise RuntimeError('; '.join(failures))
        if self.backend is not None:
            self._refresh()
            if self.state['torque_on']:
                raise RuntimeError('扭矩回读仍开启')

    def _watchdog_loop(self):
        while not self.shutdown_event.wait(.5):
            with self.lock:
                stale = time.monotonic() - self.last_client_seen > self.watchdog_seconds
                must_stop = self.state['torque_on'] and stale and not self.state['stopping']
                if must_stop:
                    self.state['stopping'] = True
                    self.state['status'] = '浏览器失去连接，正在关闭扭矩…'
            if must_stop:
                self.stop_event.set()
                self.queue.put(('stop', {}))

    def _idle(self):
        if self.backend is None:
            return
        with self.lock:
            idle_for = time.monotonic() - self.last_client_seen
        if self.arm and self.arm.enabled and idle_for > self.watchdog_seconds:
            try:
                self._stop()
                self._event('浏览器失去连接，已自动关闭扭矩')
            except Exception as exc:
                self._set(error=f'自动关闭扭矩失败：{exc}')
            return
        try:
            self._refresh()
        except Exception as exc:
            self._set(error=f'反馈读取失败：{exc}', motion_state='failed', control_enabled=False)
            try:
                self._stop()
            except Exception:
                pass

    def close(self):
        self.stop_event.set()
        self.shutdown_event.set()
        self.queue.put(('stop', {}))
        self.worker.join(timeout=10)
        self.watchdog.join(timeout=2)
        # If a backend call is unresponsive, never race a second serial owner.
        if not self.worker.is_alive() and self.backend is not None:
            self.backend.close()
            self.backend = None
