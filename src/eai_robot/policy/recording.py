"""Append-only local episode journal: request, sent action and encoder feedback differ."""
import json
from pathlib import Path
import numpy as np

from .contracts import KEYS, vector
from .observations import Observation


class Journal:
    def __init__(self, directory, manifest, *, mode, scene_id=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.manifest = manifest
        (self.directory / "manifest.json").write_text(json.dumps(manifest.data, indent=2) + "\n")
        (self.directory / "episode.json").write_text(json.dumps({"schema_version": 1, "mode": mode, "scene_id": scene_id, "manifest_fingerprint": manifest.fingerprint, "action_hz": manifest.data["action_hz"], "complete": False}, indent=2) + "\n")
        self.stream = (self.directory / "frames.jsonl").open("x", encoding="utf-8")
        self.index = 0

    def append(self, observation, *, requested=None, sent=None, feedback=None, feedback_at=None, rejection=None):
        observation.validate(self.manifest)
        requested = None if requested is None else vector(requested, bounded=False).tolist()
        if sent is not None and set(sent) != set(KEYS):
            raise ValueError("Sent action must contain all six canonical joint keys")
        sent = None if sent is None else vector([sent[key] for key in KEYS]).tolist()
        feedback = None if feedback is None else vector(feedback).tolist()
        filename = f"frame-{self.index:06d}.npz"
        np.savez_compressed(self.directory / filename, state=observation.state, **observation.images)
        row = {"index": self.index, "frame": filename, "task": observation.task, "captured_at": observation.captured_at, "completed_at": observation.completed_at, "camera_times": observation.camera_times, "requested": requested, "sent": sent, "feedback": feedback, "feedback_at": feedback_at, "rejection": rejection}
        self.stream.write(json.dumps(row, allow_nan=False) + "\n")
        self.stream.flush()
        self.index += 1

    def close(self, *, complete=False, outcome="unlabeled"):
        self.stream.close()
        path = self.directory / "episode.json"
        header = json.loads(path.read_text())
        header.update(complete=complete, outcome=outcome, frames=self.index)
        path.write_text(json.dumps(header, indent=2) + "\n")


def validate_episode(directory, manifest, *, demonstrations=False, budgets=None):
    from .safety import Budgets
    budgets = budgets or Budgets()
    directory = Path(directory)
    header = json.loads((directory / "episode.json").read_text())
    if header["manifest_fingerprint"] != manifest.fingerprint or header["action_hz"] != manifest.data["action_hz"]:
        raise ValueError("Episode model/frequency mismatch")
    if demonstrations and (not header["complete"] or header["mode"] != "demonstration" or header["outcome"] != "success"):
        raise ValueError("Only complete, explicitly successful demonstrations are trainable")
    rows = [json.loads(line) for line in (directory / "frames.jsonl").read_text().splitlines()]
    if not rows or (header["complete"] and header.get("frames") != len(rows)):
        raise ValueError("Empty or truncated episode")
    previous = None
    for index, row in enumerate(rows):
        if row["index"] != index or row["frame"] != f"frame-{index:06d}.npz":
            raise ValueError("Episode sequence/frame mismatch")
        with np.load(directory / row["frame"], allow_pickle=False) as frame:
            images = {c["source"]: frame[c["source"]] for c in manifest.data["cameras"].values()}
            obs = Observation(frame["state"], images, row["task"], row["captured_at"], row["completed_at"], row["camera_times"])
            obs.validate(manifest, max_skew_s=budgets.max_acquisition_skew_s)
        if previous is not None and (row["captured_at"] < previous or (demonstrations and row["captured_at"] == previous)):
            raise ValueError("Episode timestamps must increase")
        if demonstrations:
            if row["sent"] is None or row["feedback"] is None or row["rejection"] or row["feedback_at"] is None or row["feedback_at"] < row["completed_at"]:
                raise ValueError("Demonstration requires actual sent action and subsequent feedback")
            vector(row["sent"])
            vector(row["feedback"])
            if previous is not None and row["captured_at"] - previous > 2.0 / header["action_hz"]:
                raise ValueError("Demonstration timing gap exceeds two action periods")
        previous = row["captured_at"]
    return {"frames": len(rows), "header": header, "rows": rows}
