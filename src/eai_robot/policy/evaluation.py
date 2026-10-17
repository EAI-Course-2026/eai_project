"""Evidence aggregation; task success is an explicit operator label, never inferred from motion."""
from collections import Counter
import json
from pathlib import Path
import numpy as np

from .recording import validate_episode


def distribution(values):
    if not values:
        return None
    array = np.asarray(values, dtype=float)
    if not np.isfinite(array).all():
        raise ValueError("Evaluation evidence must be finite")
    return {"samples": len(array), "p50": float(np.percentile(array, 50)), "p95": float(np.percentile(array, 95)), "max": float(array.max())}


def label_episode(directory, outcome, evidence):
    if outcome not in {"success", "failure", "aborted"} or not isinstance(evidence, str) or not evidence.strip():
        raise ValueError("Explicit success/failure/aborted label and operator evidence required")
    directory = Path(directory)
    path = directory / "episode.json"
    header = json.loads(path.read_text())
    if header.get("mode") not in {"demonstration", "rollout"}:
        raise ValueError("Shadow output cannot be labeled as task execution")
    if outcome == "success" and (not header.get("complete") or not header.get("frames")):
        raise ValueError("Incomplete/empty session cannot be marked successful")
    # Preserve label history; labels are not model-generated assertions.
    previous = header.get("outcome", "unlabeled")
    with (directory / "labels.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"previous": previous, "outcome": outcome, "evidence": evidence}) + "\n")
    header.update(outcome=outcome, outcome_evidence=evidence)
    path.write_text(json.dumps(header, indent=2) + "\n")


def evaluate_episodes(directories, manifest):
    directories = [Path(p).resolve() for p in directories]
    if not directories or len(set(directories)) != len(directories):
        raise ValueError("At least one distinct episode required")
    labels, rejections = Counter(), Counter()
    tracking_errors, acquisition_skews, tick_intervals = [], [], []
    modes = set()
    for directory in directories:
        report = validate_episode(directory, manifest, allow_empty=True)
        header = report["header"]
        modes.add(header["mode"])
        if header["mode"] != "rollout":
            raise ValueError("Task evaluation requires rollout episodes; demonstrations and shadow are separate evidence")
        outcome = header.get("outcome", "unlabeled")
        if outcome not in {"success", "failure", "aborted", "unlabeled"}:
            raise ValueError("Unknown task outcome")
        labels[outcome] += 1
        feedback_times = []
        for row in report["rows"]:
            if row["rejection"]:
                rejections[row["rejection"]] += 1
            oldest = min(row["captured_at"], *row["camera_times"].values())
            acquisition_skews.append(row["completed_at"] - oldest)
            if row["sent"] is not None and row["feedback"] is not None:
                tracking_errors.append(float(np.abs(np.asarray(row["sent"]) - np.asarray(row["feedback"])).max()))
                feedback_times.append(row["feedback_at"])
        if len(feedback_times) > 1:
            gaps = np.diff(feedback_times)
            if not np.isfinite(gaps).all() or np.any(gaps <= 0):
                raise ValueError("Feedback timestamps must be strictly increasing")
            tick_intervals.extend(gaps.tolist())
    total = len(directories)
    return {"manifest_fingerprint": manifest.fingerprint, "trials": total, "outcomes": dict(labels), "success_rate": None if labels["unlabeled"] else labels["success"] / total, "success_rate_pending_labels": labels["unlabeled"], "rejection_counts": dict(rejections), "target_feedback_error_normalized": distribution(tracking_errors), "acquisition_skew_s": distribution(acquisition_skews), "feedback_tick_interval_s": distribution(tick_intervals), "notes": ["Immediate encoder error is not settled-position or Cartesian accuracy", "All supplied trials count, including aborted/failure; do not omit failed trials", "Task success requires operator evidence and a fixed external success criterion"]}
