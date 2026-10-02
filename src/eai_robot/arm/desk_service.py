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

from eai_robot.config import ROOT
from .calibration import JOINTS, JointCalibration, load, save
from .controller import ArmController, inspect_hardware
from .demo import DEFAULT_HOME, PROFILES, build_plan, check_start, load_home, run_stage, teach_home

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
            # home. The GUI stays read-only until a matching home is taught.
            self.home = self.radius = None
            home_required = True
        self.backend_factory = backend_factory or self._backend_factory
        self.idle_poll_seconds = idle_poll_seconds
        self.watchdog_seconds = watchdog_seconds
        self.lock = Lock()
        self.queue = Queue()
        self.stop_event = Event()
        self.shutdown_event = Event()
        self.backend = None
        self.arm = None
        self.homed = False
        self.last_client_seen = time.monotonic()
        self.captures = {name: [None, None] for name in JOINTS}
        self.state = {'connected': False, 'busy': False, 'stopping': False,
                      'homed': False, 'torque_on': False,
                      'home_required': home_required,
                      'status': '请示教匹配的统一起点' if home_required else '等待连接',
                      'error': None, 'positions': None, 'joints': [], 'events': [],
                      'captures': self.captures, 'profile': None,
                      'port': '', 'backend': 'lerobot'}
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

    def _manual_limits(self, name):
        cal = self.calibration[name]
        inner_low, inner_high = cal.bounds(MARGIN)
        sid = cal.id
        if self.home is None:
            return inner_low, inner_high
        return max(inner_low, self.home[sid]-self.radius[sid]), min(inner_high, self.home[sid]+self.radius[sid])

    def submit(self, action, payload=None):
        payload = payload or {}
        if action not in {'connect', 'disconnect', 'home', 'move', 'demo', 'stop',
                          'capture', 'save_calibration', 'teach_home'}:
            raise ValueError('未知操作')
        if not isinstance(payload, dict):
            raise ValueError('操作参数必须为对象')
        if action == 'stop':
            with self.lock:
                self.state['stopping'] = True
                self.state['busy'] = True
                self.state['status'] = '正在停止并关闭扭矩…'
            self.stop_event.set()
            self.queue.put((action, payload))
            return
        # Validate untrusted input before it reaches the hardware worker.
        if action == 'connect':
            kind, port, baudrate = payload.get('backend'), payload.get('port'), payload.get('baudrate')
            if kind not in ('serial', 'lerobot') or not isinstance(port, str) or not port.strip() or len(port) > 256:
                raise ValueError('请选择后端并填写串口')
            if type(baudrate) is not int or baudrate not in BAUDRATES:
                raise ValueError('波特率无效')
        elif action == 'move':
            values = payload.get('values')
            if not isinstance(values, list) or len(values) != 6 or any(
                    type(v) not in (float, int) or not math.isfinite(v) or not 0 <= v <= 1 for v in values):
                raise ValueError('必须输入六个 0..1 的有限数值')
            targets = {self.calibration[name].id: self.calibration[name].position(value, MARGIN)
                       for name, value in zip(JOINTS, values, strict=True)}
            if self.home is None:
                raise RuntimeError('当前校准尚无匹配起点，请先示教')
            check_start(self.calibration, targets, self.home, self.radius)
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
            if action in ('home', 'demo', 'move') and self.home is None:
                raise RuntimeError('当前校准尚无匹配起点，请先示教')
            if action == 'move' and not self.state['homed']:
                raise RuntimeError('请先进入统一起点')
            if action in ('capture', 'save_calibration', 'teach_home') and self.state['torque_on']:
                raise RuntimeError('先停止并关闭扭矩，再采集校准')
            self.state['busy'] = True
            self.state['error'] = None
            self.state['status'] = '正在执行…'
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
                    self._event('已停止，六关节扭矩已释放')
                else:
                    self.stop_event.clear()
                    self._dispatch(action, payload)
            except Exception as exc:
                self._set(error=f'{type(exc).__name__}: {exc}', homed=False)
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
            self._set(connected=False, positions=None, joints=[], torque_on=False, homed=False)
            self._event('已断开串口，扭矩关闭')
        elif action == 'home':
            self._home()
            self._refresh()
        elif action == 'move':
            targets = {self.calibration[name].id: self.calibration[name].position(value, MARGIN)
                       for name, value in zip(JOINTS, payload['values'], strict=True)}
            item = {'name': '六关节同步目标', 'targets': targets}
            self._event('正在移动六个关节…')
            run_stage(self.arm, item)
            self._refresh()
            self._event('六关节到位；扭矩保持，停止按钮可释放')
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
                run_stage(self.arm, item)
                self._refresh()
                self._wait_hold(1.2)
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
            self._event(f'已保存新起点：{path}；重新启动时用 --home-file 指定')
        else:
            raise ValueError('未知操作')

    def _home(self):
        current = self.arm.inspect()
        check_start(self.calibration, current, self.home, self.radius)
        if not self.arm.enabled:
            self.arm.enable()
            self._set(torque_on=True)
        current = self.arm.positions()
        check_start(self.calibration, current, self.home, self.radius)
        plan = build_plan(self.calibration, current, self.home, self.radius, 'home', MARGIN)
        self.homed = False
        self._set(homed=False)
        for item in plan:
            if self.stop_event.is_set():
                raise MotionCancelled('已请求停止')
            self._event(item['name'])
            run_stage(self.arm, item)
            self._refresh()
        self.homed = True
        self._set(homed=True, torque_on=True)
        self._event('统一起点已确认，可以发送六关节目标')

    def _wait_hold(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.stop_event.wait(min(.1, deadline-time.monotonic())):
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
        faults = [f'ID {j["id"]}: 0x{j["status"]:02x}' for j in joints if j['status']]
        if faults:
            raise RuntimeError('舵机状态报警：'+', '.join(faults))

    def _stop(self):
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
        self._set(homed=False, torque_on=bool(failures))
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
            self._set(error=f'反馈读取失败：{exc}')
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
