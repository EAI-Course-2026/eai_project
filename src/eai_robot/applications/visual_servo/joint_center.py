"""Bounded image-to-encoder centering; FK is a workspace guard, not a motion model."""
from __future__ import annotations
import json
import time
from datetime import datetime
import numpy as np
from eai_robot.config import ROOT
from eai_robot.control.execution.feedback import (
    read_raw_motors, arm_degrees_from_raw, ensure_execution_start_is_safe,
    raw_outside_calibration,
)
from eai_robot.control.kinematics.joint_mapping import ARM_JOINT_NAMES
from eai_robot.applications.visual_servo.shared_camera import PreviewFrameUnavailable

# Re-export helpers for existing callers; numerical logic lives in control/.
from eai_robot.control.visual_servo.empirical import (
    JOINTS, STEP, ARRIVAL_TOLERANCE, commanded_joints, center_tolerance,
    joint_step_options, choose_step, motion_confirmed, motion_observed,
)


class TargetLostError(RuntimeError):
    pass


def wait_for_target(runtime,detector,preview,stop,required_frames=5):
    """Hold existing goals and refresh preview until consecutive fresh detections."""
    seen=0
    while not stop.stop_requested.is_set():
        try:
            frame=runtime.read_frame()
        except (PreviewFrameUnavailable,TimeoutError):
            seen=0
            if preview:
                preview.show(np.zeros((480,640,3),np.uint8),None,'CAMERA WAIT - TORQUE HOLD')
            time.sleep(.03)
            continue
        detected,_=detector.detect(frame)
        seen=seen+1 if detected is not None else 0
        if preview:
            preview.show(frame,detected,f'TARGET WAIT - HOLD {seen}/{required_frames}')
        if stop.stop_requested.is_set():
            break
        if seen>=required_frames:
            return frame,detected
    raise RuntimeError('User stop while waiting for target')


def verify_existing_hold(robot):
    """Read-only handover gate; partial torque, faults, drift or motion refuse it."""
    first = read_raw_motors(robot, ARM_JOINT_NAMES)
    for _ in range(5):
        actual = read_raw_motors(robot, ARM_JOINT_NAMES)
        for name in ARM_JOINT_NAMES:
            if robot.bus.read('Torque_Enable', name, num_retry=2) != 1:
                raise RuntimeError(f'Cannot adopt hold: torque is not on for {name}')
            if robot.bus.read('Status', name, num_retry=2) != 0:
                raise RuntimeError(f'Cannot adopt hold: status fault for {name}')
            goal = robot.bus.read('Goal_Position', name, normalize=False, num_retry=2)
            # Existing static load error can exceed the moving-axis arrival
            # tolerance. Eight raw counts is under 3 degrees on these axes;
            # require a separate stationary read sequence as well.
            if abs(goal-actual[name]) > 8 or abs(actual[name]-first[name]) > 2:
                raise RuntimeError(f'Cannot adopt hold: arm not stationary at held goal for {name}')
        time.sleep(.05)


def run_joint_center(runtime, detector, args, stop, preview):
    from eai_robot.applications.visual_servo.ball import median_detection, StopRequestedError
    mapper = runtime._require_mapper()
    if not hasattr(runtime, '_joint_center_reference'):
        initial = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
        if getattr(args, 'hold_reference', None):
            initial = json.loads(args.hold_reference.read_text())
            if set(initial) != set(ARM_JOINT_NAMES) or any(type(v) is not int for v in initial.values()) or raw_outside_calibration(mapper,initial):
                raise RuntimeError('Invalid preserved handover reference')
        initial_q = arm_degrees_from_raw(mapper, initial)
        origin = runtime.fk.forward_kinematics(initial_q)[:3, 3]
        runtime._joint_center_reference = (initial, initial_q, origin)
    initial, initial_q, origin = runtime._joint_center_reference
    path = ROOT / 'local_artifacts/experiments' / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_joint-center.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    base_counts=getattr(args,'base_step_counts',10)
    wrist_counts=getattr(args,'wrist_step_counts',12)
    options=joint_step_options(base_counts,wrist_counts)
    print(f'\nImage-to-joint centering; base={options[0]}, wrist={options[1]}, arrival tolerance=3 counts. Log: {path}')
    started = time.monotonic()
    runtime.hold_reason = 'USER STOP'

    def record(event, **data):
        with path.open('a') as stream:
            stream.write(json.dumps(dict(time=time.monotonic(), event=event, **data)) + '\n')

    record('settings',base_steps=options[0],wrist_steps=options[1],max_travel_mm=args.max_travel_mm,
           max_joint_excursion_deg=getattr(args,'max_joint_excursion_deg',10),max_joint_step_deg=args.max_joint_step_deg)

    def valid(raw, reference):
        if raw_outside_calibration(mapper, raw):
            return False
        q = arm_degrees_from_raw(mapper, raw)
        try:
            ensure_execution_start_is_safe(q, margin_deg=args.joint_margin_deg)
        except RuntimeError:
            return False
        if np.max(np.abs(q - initial_q)) > getattr(args,'max_joint_excursion_deg',10.0):
            return False
        if np.max(np.abs(q - arm_degrees_from_raw(mapper, reference))) > args.max_joint_step_deg:
            return False
        xyz = runtime.fk.forward_kinematics(q)[:3, 3]
        return np.linalg.norm(xyz - origin) <= args.max_travel_mm / 1000

    def move(target):
        before = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
        if not valid(target, before):
            raise RuntimeError('Joint/workspace bound reached')
        changed = commanded_joints(before, target)
        if not changed:
            raise RuntimeError('Refusing motion inside arrival tolerance')
        # Require current target and live preview immediately before each command.
        frame = runtime.read_frame()
        detected, _ = detector.detect(frame)
        if detected is None:
            raise TargetLostError('Target lost before command')
        for n in ARM_JOINT_NAMES:
            if runtime.robot.bus.read('Torque_Enable', n, num_retry=2) != 1 or runtime.robot.bus.read('Status', n, num_retry=2) != 0:
                raise RuntimeError(f'Torque/status fault: {n}')
        runtime.robot.bus.sync_write('Goal_Position', changed, normalize=False, num_retry=2)
        for n, value in changed.items():
            if runtime.robot.bus.read('Goal_Position', n, normalize=False, num_retry=2) != value:
                raise RuntimeError(f'Goal readback mismatch: {n}')
        deadline = time.monotonic() + 1.0
        stable_anchor = before
        stable_since = time.monotonic()
        while True:
            actual = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
            if not valid(actual, before):
                raise RuntimeError('Measured pose outside permitted bounds')
            try:
                frame = runtime.read_frame()
            except (PreviewFrameUnavailable,TimeoutError):
                holds={n:actual[n] for n in changed}
                runtime.robot.bus.sync_write('Goal_Position',holds,normalize=False,num_retry=2)
                for n,value in holds.items():
                    if runtime.robot.bus.read('Goal_Position',n,normalize=False,num_retry=2)!=value:
                        raise RuntimeError('Camera-loss hold readback mismatch')
                record('camera_lost_hold',actual=actual,hold_goals=holds)
                raise
            detected, _ = detector.detect(frame)
            if preview:
                preview.show(frame, detected, 'VERIFYING ENCODER MOTION')
            if stop.stop_requested.is_set():
                raise StopRequestedError('stop requested')
            if detected is None:
                holds={n:actual[n] for n in changed}
                runtime.robot.bus.sync_write('Goal_Position',holds,normalize=False,num_retry=2)
                for n,value in holds.items():
                    if runtime.robot.bus.read('Goal_Position',n,normalize=False,num_retry=2)!=value:
                        raise RuntimeError('Target-loss hold readback mismatch')
                record('target_lost_hold',actual=actual,hold_goals=holds)
                raise TargetLostError('Target lost during motion')
            now = time.monotonic()
            if any(abs(actual[n] - stable_anchor[n]) > 1 for n in changed):
                stable_since = now
                stable_anchor = actual
            arrived = motion_confirmed(before, changed, actual)
            settled_response = motion_observed(before, changed, actual) and now - stable_since >= 0.20
            if settled_response:
                # Always wait for a stable measurement, even on first entering
                # the arrival band; continued motion would corrupt the Jacobian.
                holds = {n: actual[n] for n in changed}
                runtime.robot.bus.sync_write('Goal_Position', holds, normalize=False, num_retry=2)
                for n, value in holds.items():
                    if runtime.robot.bus.read('Goal_Position', n, normalize=False, num_retry=2) != value:
                        raise RuntimeError('Measured hold goal readback mismatch')
                if not arrived:
                    record('partial_response', requested=changed, measured_hold=holds)
                runtime.command_q = arm_degrees_from_raw(mapper, actual)
                runtime.command_position = runtime.fk.forward_kinematics(runtime.command_q)[:3, 3]
                record('motion', before=before, goal=changed, actual=actual)
                return actual
            if time.monotonic() >= deadline:
                record('arrival_timeout', before=before, goal=changed, actual=actual)
                raise RuntimeError(f'Arrival not confirmed within 1 second; goal={changed}, actual={actual}')

    try:
        time.sleep(args.torque_settle_s)
        try:
            frame = runtime.read_frame()
        except (PreviewFrameUnavailable,TimeoutError):
            frame,_=wait_for_target(runtime,detector,preview,stop)
        observed, _ = detector.detect(frame)
        if observed is None:
            frame,observed=wait_for_target(runtime,detector,preview,stop)
        h, w = frame.shape[:2]
        if not getattr(args,'resume_response',None) and not (w*.25 <= observed.center_x <= w*.75 and h*.25 <= observed.center_y <= h*.75):
            raise RuntimeError('Place marker in central half of preview before probing')
        if getattr(args, 'resume_response', None):
            jacobian = np.asarray(json.loads(args.resume_response.read_text())['jacobian'],dtype=float)
            if jacobian.shape != (2,2) or not np.all(np.isfinite(jacobian)):
                raise RuntimeError('Invalid saved image response')
            print('Resuming previously measured image response; no new calibration probes.')
        else:
            columns = []
            for joint in JOINTS:
                probe_count=wrist_counts if joint=='wrist_flex' else base_counts
                before = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
                baseline = median_detection(runtime, detector, args.feature_samples)
                candidate = None
                for sign in (1, -1):
                    target = dict(before)
                    target[joint] += sign * probe_count
                    if valid(target, before):
                        candidate = target
                        break
                if candidate is None:
                    raise RuntimeError(f'No safe probe for {joint}')
                actual = move(candidate)
                time.sleep(0.15)
                observed = median_detection(runtime, detector, args.feature_samples)
                column = (observed.center - baseline.center) / (actual[joint] - before[joint])
                if np.linalg.norm(observed.center - baseline.center) < 3:
                    raise RuntimeError(f'Insufficient image response for {joint}')
                columns.append(column)
                # Return only the probed joint; other axes retain their measured holds.
                restore = dict(actual)
                # Do not shrink the reverse request by the arrival residual; retain
                # the full tested request amplitude from the measured position.
                restore[joint] -= int(np.sign(candidate[joint] - before[joint])) * probe_count
                move(restore)
            jacobian = np.column_stack(columns)
        if np.linalg.cond(jacobian) > 30 or np.any(np.linalg.norm(jacobian,axis=0) < .1):
            raise RuntimeError('Image-to-joint response is poorly conditioned')
        # A moving target contributes its own pixel displacement. Updating this
        # model from servo frames would misattribute that displacement to the
        # arm and can reverse the learned direction. Recalibrate only with a
        # stationary target in the explicit probe phase.
        jacobian.setflags(write=False)
        record('calibration', jacobian=jacobian.tolist(), initial=initial)
        print('Measured pixel/count response:', jacobian, flush=True)
        stable = 0
        worsening = 0
        completed=getattr(runtime,'demo_completed_cycles',0)
        moved_since_lock=False
        while not stop.stop_requested.is_set():
            if args.max_runtime_s > 0 and time.monotonic() - started >= args.max_runtime_s:
                runtime.hold_reason = 'TIME LIMIT'
                print('\nRuntime reached; HOLDING.')
                break
            try:
                frame = runtime.read_frame()
            except (PreviewFrameUnavailable,TimeoutError):
                record('camera_wait')
                stable=0
                wait_for_target(runtime,detector,preview,stop)
                record('camera_recovered')
                worsening=0
                continue
            detected, _ = detector.detect(frame)
            if detected is None:
                record('target_lost')
                stable=0
                wait_for_target(runtime,detector,preview,stop)
                record('target_reacquired')
                worsening=0
                continue
            error = np.array([frame.shape[1]/2, frame.shape[0]/2]) - detected.center
            tolerance = center_tolerance(frame.shape, args.deadband_px, getattr(args,'center_region_fraction',0.0))
            locked = bool(np.all(np.abs(error) <= tolerance))
            stable = stable + 1 if locked else 0
            if stable == args.stable_frames:
                if moved_since_lock:
                    completed+=1
                    runtime.demo_completed_cycles=completed
                    moved_since_lock=False
                record('locked', error=error.tolist(), tolerance=tolerance.tolist(),completed_cycles=completed)
            status = f'{"ALIGNED" if stable >= args.stable_frames else "CENTERING"} {completed}/10 ex={error[0]:+.0f} ey={error[1]:+.0f}'
            if preview:
                preview.show(frame, detected, status)
            print('\r' + status + ' '*15, end='', flush=True)
            if locked:
                continue
            before = read_raw_motors(runtime.robot, ARM_JOINT_NAMES)
            def target_for(step):
                target = dict(before)
                for n, v in zip(JOINTS, step):
                    target[n] += int(v)
                return target
            step = choose_step(jacobian, error, lambda s: valid(target_for(s), before),options)
            if step is None:
                raise RuntimeError('No safe per-joint step predicts lower error; HOLDING')
            try:
                actual = move(target_for(step))
            except (TargetLostError,PreviewFrameUnavailable,TimeoutError):
                stable=0
                wait_for_target(runtime,detector,preview,stop)
                record('target_reacquired')
                worsening=0
                continue
            moved_since_lock=True
            time.sleep(0.15)
            _,after = wait_for_target(runtime,detector,preview,stop,required_frames=args.feature_samples)
            after_error = np.array([frame.shape[1]/2, frame.shape[0]/2]) - after.center
            delta = np.array([actual[n] - before[n] for n in JOINTS], dtype=float)
            record('response', error_before=error.tolist(), error_after=after_error.tolist(),
                   predicted_pixels=(jacobian@delta).tolist(), model_updated=False)
            worsening = worsening + 1 if np.linalg.norm(after_error) > np.linalg.norm(error) + 3 else 0
            if worsening >= 2:
                raise RuntimeError('Two consecutive moves increased image error; HOLDING')
    except RuntimeError as exc:
        runtime.hold_reason = str(exc)
        record('hold', reason=str(exc))
        print(f'\n{exc}; stopped sending motion, torque remains on pending support.', flush=True)
    return 0
