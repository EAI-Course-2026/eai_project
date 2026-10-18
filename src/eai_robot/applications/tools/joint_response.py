"""Camera-free, single-axis raw-position response experiment (base yaw only)."""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import time

from eai_robot.calibration.servo.results import JOINTS, load
from eai_robot.config import ROOT, load_config
from eai_robot.hardware.drivers.backends import SerialBackend
from eai_robot.hardware.drivers.inspection import inspect_hardware


def measure_step(backend, sid, start, target, *, duration=1., period=.01,
                 movement_threshold=2, clock=time.perf_counter, sleep=time.sleep,
                 on_sample=None):
    """Observe an explicit target for the whole window, never exit on tolerance alone."""
    if target == start:
        raise ValueError("step must be nonzero")
    sent_at = clock()
    backend.sync_positions({sid: target})
    if backend.read(sid, "Goal_Position") != target:
        raise RuntimeError("target register did not match the requested position")
    rows = []
    deadline = sent_at + duration
    while True:
        position = backend.read(sid, "Present_Position")
        torque = backend.read(sid, "Torque_Enable")
        status = backend.read(sid, "Status")
        elapsed = clock() - sent_at
        if type(position) is not int or not 0 <= position <= 1023:
            raise RuntimeError("invalid encoder feedback")
        if torque != 1 or status != 0:
            raise RuntimeError("torque lost or servo status alarm")
        row = {"elapsed_s": elapsed, "position": position, "target": target}
        rows.append(row)
        if on_sample is not None:
            on_sample(row)
        if not min(start, target) - 5 <= position <= max(start, target) + 5:
            raise RuntimeError("feedback exceeded bounded test excursion")
        if clock() >= deadline:
            break
        sleep(min(period, max(0., deadline - clock())))
    sign = 1 if target > start else -1
    tolerance = min(1, abs(target - start) - 1)
    first_motion = next((r["elapsed_s"] for r in rows
                         if sign * (r["position"] - start) >= movement_threshold), None)
    first_arrival = next((r["elapsed_s"] for r in rows
                          if abs(r["position"] - target) <= tolerance), None)
    at_200ms = [r for r in rows if r["elapsed_s"] <= .2]
    summary = {
        "start": start, "target": target, "delta_counts": target - start,
        "movement_threshold_counts": movement_threshold, "tolerance_counts": tolerance,
        "first_motion_s": first_motion, "first_arrival_s": first_arrival,
        "moved_within_200ms": first_motion is not None and first_motion <= .2,
        "arrived_within_200ms": first_arrival is not None and first_arrival <= .2,
        "position_last_at_or_before_200ms": at_200ms[-1]["position"] if at_200ms else None,
        "final_position": rows[-1]["position"],
        "final_signed_displacement_counts": sign * (rows[-1]["position"] - start),
        "max_signed_displacement_counts": max(sign * (r["position"] - start) for r in rows),
        "final_error_counts": abs(rows[-1]["position"] - target),
        "observed_motion": first_motion is not None,
    }
    return summary, rows


def validate_targets(start, cal, hardware_bounds, counts):
    if not counts or any(type(n) is not int or not 2 <= n <= 30 for n in counts):
        raise ValueError("test amplitudes must be integer counts within 2..30")
    low = max(cal.range_min, hardware_bounds[0]) + 10
    high = min(cal.range_max, hardware_bounds[1]) - 10
    if not low <= start - max(counts) or not start + max(counts) <= high:
        raise ValueError("test excursion lacks 10-count margin inside calibrated hardware limits")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=load_config()["serial"]["port"])
    parser.add_argument("--counts", type=int, nargs="+", default=[30, 20, 10, 5, 3, 2])
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    calibration = load(ROOT / "calibration/scs215_so101.json")
    sid = 1
    cal = calibration["shoulder_pan"]
    output = args.output or ROOT / "local_artifacts/experiments" / (
        datetime.now().strftime("%Y-%m-%d_%H%M%S_%f") + "_base-joint-response")
    output.mkdir(parents=True, exist_ok=False)
    backend = None
    torque_touched = False
    trials = []
    report = {"scope": "real single-joint raw control; no vision or FK execution",
              "joint": "shoulder_pan", "id": sid, "execute": args.execute,
              "amplitudes_counts": args.counts, "trials": trials}
    csv_file = (output / "samples.csv").open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(csv_file, fieldnames=["trial", "kind", "elapsed_s", "position", "target"])
    writer.writeheader()

    def save():
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        csv_file.flush()

    try:
        backend = SerialBackend(args.port, 1_000_000)
        limits = inspect_hardware(backend)
        before = {i: {r: backend.read(i, r) for r in ("Present_Position", "Torque_Enable", "Status")}
                  for i in range(1, 7)}
        if any(m["Torque_Enable"] != 0 or m["Status"] != 0 for m in before.values()):
            raise RuntimeError("all six joints must initially have torque off and no alarms")
        start = before[sid]["Present_Position"]
        validate_targets(start, cal, limits[sid], args.counts)
        baseline = [backend.read(sid, "Present_Position") for _ in range(20)]
        if max(baseline) - min(baseline) > 2:
            raise RuntimeError("baseline moved by more than two counts; stabilize the arm first")
        movement_threshold = max(2, max(baseline) - min(baseline) + 1)
        report.update(before=before, baseline=baseline, original_raw=start,
                      movement_threshold_counts=movement_threshold,
                      goal_velocity=backend.read(sid, "Goal_Velocity"), hardware_limits=limits)
        save()
        print(json.dumps({"event": "preflight", "joint": "shoulder_pan", "start": start,
                          "target_range": [start-max(args.counts), start+max(args.counts)],
                          "other_axes": "torque remains off", "output": str(output)}), flush=True)
        if not args.execute:
            return 0
        if input('Clear base rotation area; keep other joints supported. Type START JOINT TEST: ').strip() != "START JOINT TEST":
            report["result"] = "cancelled"
            return 0
        if any(backend.read(i, "Torque_Enable") != 0 or backend.read(i, "Status") != 0 for i in range(1, 7)):
            raise RuntimeError("device state changed during confirmation")
        # No current-pose jump: re-read and seed only the selected yaw servo.
        current = backend.read(sid, "Present_Position")
        if abs(current - start) > 2:
            raise RuntimeError("start pose changed; rerun preflight")
        start = current
        validate_targets(start, cal, limits[sid], args.counts)
        backend.write(sid, "Running_Time", 0)
        backend.sync_positions({sid: start})
        if backend.read(sid, "Goal_Position") != start:
            raise RuntimeError("initial hold target not confirmed")
        torque_touched = True
        backend.write(sid, "Torque_Enable", 1)
        time.sleep(.2)
        def sample_writer(trial, kind):
            def write(row):
                writer.writerow(dict(trial=trial, kind=kind, **row))
                csv_file.flush()
            return write
        for amplitude in args.counts:
            for sign in (1, -1):
                if any(backend.read(i, "Torque_Enable") != 0 for i in range(2, 7)):
                    raise RuntimeError("another axis unexpectedly enabled")
                origin = backend.read(sid, "Present_Position")
                # Measure each shot relative to actual feedback. Static return
                # offset is evidence, not an assumed new zero or 'arrival'.
                # Keep every target inside the originally validated excursion.
                if abs(origin - start) > 5:
                    raise RuntimeError("baseline drift exceeded five counts")
                target = origin + sign * amplitude
                if not start-max(args.counts) <= target <= start+max(args.counts):
                    raise RuntimeError("target would exceed original validated excursion")
                result, rows = measure_step(backend, sid, origin, target,
                                            movement_threshold=movement_threshold,
                                            on_sample=sample_writer(len(trials)+1, "outward"))
                trials.append(result)
                print(json.dumps({"event": "trial", **result}), flush=True)
                save()
                if result["final_position"] == start:
                    returned = {"final_error_counts": 0, "result": "already_at_baseline"}
                else:
                    returned, _ = measure_step(backend, sid, result["final_position"], start,
                        movement_threshold=movement_threshold,
                        on_sample=sample_writer(len(trials), "return"))
                result["return_to_start"] = returned
                save()
                if returned["final_error_counts"] > 5:
                    raise RuntimeError("return feedback left bounded baseline region; no further test")
                if amplitude == max(args.counts) and not result["observed_motion"]:
                    raise RuntimeError("large step produced no measured motion; smaller-step scan aborted")
        report["result"] = "completed"
        print(json.dumps({"event": "completed", "output": str(output)}), flush=True)
        return 0
    except BaseException as exc:
        report["result"] = "failed"
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if backend is not None:
            if torque_touched:
                try:
                    backend.write(sid, "Torque_Enable", 0)
                    report["selected_torque_off_verified"] = backend.read(sid, "Torque_Enable") == 0
                except BaseException as exc:
                    report["release_error"] = str(exc)
                    print("Torque release unconfirmed: support arm and disconnect servo power.", flush=True)
            try:
                report["torque_after"] = {i: backend.read(i, "Torque_Enable") for i in range(1, 7)}
            except BaseException as exc:
                report["torque_after_read_error"] = str(exc)
            finally:
                backend.close()
        save()
        csv_file.close()


if __name__ == "__main__":
    raise SystemExit(main())
