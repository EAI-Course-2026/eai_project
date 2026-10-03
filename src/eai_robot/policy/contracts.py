"""Versioned model provenance and robot semantics, separate from model normalization."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np

FORK_SHA = "6a077907c7989635218969ee78f5436f8faec92b"
FORK_URL = "https://github.com/EAI-Course-2026/eai-course-lerobot.git"
JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
KEYS = tuple(f"{name}.pos" for name in JOINTS)
UNITS = "scs215_calibrated_percent"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return float(value)


def vector(values, *, bounded=True):
    raw = np.asarray(values)
    if raw.dtype.kind not in "fiu" or raw.shape != (6,):
        raise ValueError("Expected six numeric joint values, in canonical order")
    result = raw.astype(np.float64)
    if not np.isfinite(result).all():
        raise ValueError("Joint values must be finite")
    if bounded and (np.any(result[:5] < -100) or np.any(result > 100) or result[5] < 0):
        raise ValueError("Joint values outside shared normalized ranges")
    return result


@dataclass(frozen=True)
class Manifest:
    path: Path
    data: dict

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        data = json.loads(path.read_text(encoding="utf-8"))
        manifest = cls(path, data)
        manifest.validate()
        return manifest

    def validate(self):
        d = self.data
        if d.get("schema_version") != 1 or d.get("policy_type") not in {"act", "smolvla", "pi05", "fake"}:
            raise ValueError("Unsupported manifest version or policy type")
        if d.get("framework_sha") != FORK_SHA:
            raise ValueError("Manifest must use the pinned course fork")
        if tuple(d.get("state_order", ())) != JOINTS or tuple(d.get("action_order", ())) != JOINTS:
            raise ValueError("State/action order must explicitly match all six canonical joints")
        if d.get("state_units") not in {UNITS, "unknown"} or d.get("action_units") not in {UNITS, "unknown"}:
            raise ValueError("Unsupported units: degrees/radians/Cartesian require a reviewed adapter")
        if d.get("action_mode") not in {"absolute", "unknown"}:
            raise ValueError("Postprocessed output must be absolute; use official relative-action processor")
        finite_positive(d.get("action_hz"), "action_hz")
        if type(d.get("max_chunk_steps")) is not int or not 1 <= d["max_chunk_steps"] <= 1000:
            raise ValueError("max_chunk_steps must be 1..1000")
        if d.get("motion_verified") not in (True, False) or type(d.get("motion_verified")) is not bool:
            raise ValueError("motion_verified must be a boolean")
        cameras = d.get("cameras")
        if not isinstance(cameras, dict) or not cameras:
            raise ValueError("Explicit camera mapping is required")
        if len({c.get("source") for c in cameras.values()}) != len(cameras):
            raise ValueError("Each model camera requires a distinct real source")
        for key, spec in cameras.items():
            shape = spec.get("shape")
            if not key.startswith("observation.images.") or not isinstance(spec.get("source"), str) or not spec["source"]:
                raise ValueError("Invalid model camera key/source")
            if not isinstance(shape, list) or len(shape) != 3 or shape[0] != 3 or any(type(v) is not int or not 1 <= v <= 4096 for v in shape):
                raise ValueError("Camera shape must be RGB CHW, bounded to 4096 pixels")
        if d["policy_type"] != "fake":
            if not re.fullmatch(r"[a-f0-9]{40}|[a-f0-9]{64}", d.get("revision", "")):
                raise ValueError("Immutable Hub commit or local content digest required")
            if not d.get("checkpoint") or not isinstance(d.get("files"), dict):
                raise ValueError("Local checkpoint and SHA256 file inventory required")
            for name in ("config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"):
                if name not in d["files"]:
                    raise ValueError(f"Missing checkpoint inventory: {name}")
            for name, digest in d["files"].items():
                self.artifact_path(name)
                if not re.fullmatch(r"[a-f0-9]{64}", digest):
                    raise ValueError(f"Invalid SHA256: {name}")

    @property
    def root(self):
        return (self.path.parent / self.data.get("checkpoint", ".")).resolve()

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(self.data, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

    def artifact_path(self, name):
        if Path(name).is_absolute() or ".." in Path(name).parts:
            raise ValueError("Artifact must be relative to the checkpoint")
        path = (self.root / name).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("Artifact escaped checkpoint")
        return path

    def verify_files(self):
        for name, digest in self.data.get("files", {}).items():
            if sha256(self.artifact_path(name)) != digest:
                raise ValueError(f"Artifact checksum mismatch: {name}")
        if self.data["policy_type"] != "fake":
            actual = {str(p.relative_to(self.root)) for p in self.root.rglob("*") if p.is_file() and ".cache" not in p.relative_to(self.root).parts}
            if actual != set(self.data["files"]):
                raise ValueError("Checkpoint inventory must cover every local artifact")

    def require_robot_semantics(self, calibration_path, *, motion=False):
        d = self.data
        if d["state_units"] != UNITS or d["action_units"] != UNITS or d["action_mode"] != "absolute":
            raise ValueError("Unknown robot semantics: only offline inference is allowed")
        if d.get("calibration_sha256") != sha256(Path(calibration_path)):
            raise ValueError("Calibration digest does not match the model/data contract")
        if motion and (not d["motion_verified"] or not d.get("compatibility_evidence") or d["policy_type"] == "fake"):
            raise ValueError("Motion requires a real, matched checkpoint and compatibility evidence")

    def check_model_config(self, cfg):
        if cfg.get("type") != self.data["policy_type"] or cfg.get("n_obs_steps", 1) != 1:
            raise ValueError("Policy type mismatch or unsupported temporal observation history")
        inputs = cfg.get("input_features", {})
        if inputs.get("observation.state", {}).get("shape") != [6] or cfg.get("output_features", {}).get("action", {}).get("shape") != [6]:
            raise ValueError("Checkpoint must declare six state/action dimensions; padding/truncation is not an adapter")
        if set(inputs) != {"observation.state", *self.data["cameras"]}:
            raise ValueError("Checkpoint camera/features mismatch; no implicit empty cameras")
        for key, camera in self.data["cameras"].items():
            if inputs[key].get("shape") != camera["shape"]:
                raise ValueError(f"Checkpoint image shape mismatch: {key}")
        if cfg.get("temporal_ensemble_coeff") is not None or cfg.get("rtc_config"):
            raise ValueError("Chunk runtime requires ordinary inference; temporal ensemble/RTC is a separate scheduler")
        if cfg.get("use_visual_memory") or cfg.get("use_proprioceptive_memory") or cfg.get("adapt_to_pi_aloha"):
            raise ValueError("Memory/ALOHA policies require a separate reviewed observation/action adapter")
        steps = cfg.get("n_action_steps", cfg.get("chunk_size", 1))
        if steps > self.data["max_chunk_steps"]:
            raise ValueError("Checkpoint execution horizon exceeds manifest")
