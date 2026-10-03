"""Checkpoint inventory, metadata inspection and explicit pinned Hub preparation."""
import json
from pathlib import Path
import re

from .contracts import FORK_SHA, JOINTS, Manifest, sha256


def checkpoint_info(root):
    root = Path(root)
    cfg = json.loads((root / "config.json").read_text())
    report = {"policy_type": cfg.get("type"), "input_features": cfg.get("input_features"), "output_features": cfg.get("output_features"), "normalization_mapping": cfg.get("normalization_mapping"), "chunk_size": cfg.get("chunk_size"), "n_action_steps": cfg.get("n_action_steps"), "empty_cameras": cfg.get("empty_cameras", 0), "issues": []}
    for field, key in (("input_features", "observation.state"), ("output_features", "action")):
        if cfg.get(field, {}).get(key, {}).get("shape") != [6]:
            report["issues"].append(f"{key} is not six dimensional; require a matched SO101 checkpoint")
    for filename in ("model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"):
        if not (root / filename).is_file():
            report["issues"].append(f"Missing {filename}")
    for filename in ("policy_preprocessor.json", "policy_postprocessor.json"):
        path = root / filename
        if not path.exists():
            continue
        pipeline = json.loads(path.read_text())
        for step in pipeline.get("steps", []):
            if "normalizer" in step.get("registry_name", ""):
                report.setdefault("normalizers", []).append(step)
                config = step.get("config", {})
                for key, feature_type in (("observation.state", "STATE"), ("action", "ACTION")):
                    if feature_type in config.get("norm_map", {}) and config["norm_map"][feature_type] != "IDENTITY" and key not in config.get("features", {}):
                        # Postprocessors usually normalize only action, so state is optional there.
                        if filename == "policy_preprocessor.json" or key == "action":
                            report["issues"].append(f"{filename}: no {key} normalization feature/statistics")
            state_file = step.get("state_file")
            if state_file and not (root / state_file).is_file():
                report["issues"].append(f"Missing processor statistics: {state_file}")
    report["issues"].append("Joint order, units, calibration, camera views and task compatibility require provenance; shape alone is insufficient")
    return report


def freeze_checkpoint(checkpoint, destination, *, revision, model_id="local", action_hz=30, assets=None):
    root = Path(checkpoint).resolve()
    destination = Path(destination).resolve()
    cfg = json.loads((root / "config.json").read_text())
    cameras = {key: {"source": key.rsplit(".", 1)[-1], "shape": feature["shape"]} for key, feature in cfg["input_features"].items() if feature["type"] == "VISUAL"}
    import os
    files = {str(p.relative_to(root)): sha256(p) for p in sorted(root.rglob("*")) if p.is_file() and ".cache" not in p.relative_to(root).parts}
    data = {"schema_version": 1, "policy_type": cfg["type"], "framework_sha": FORK_SHA, "model_id": model_id, "revision": revision, "checkpoint": os.path.relpath(root, destination.parent), "files": files, "assets": assets or {}, "state_order": list(JOINTS), "action_order": list(JOINTS), "state_units": "unknown", "action_units": "unknown", "action_mode": "unknown", "action_hz": action_hz, "max_chunk_steps": cfg.get("n_action_steps", cfg["chunk_size"]), "cameras": cameras, "calibration_sha256": "", "motion_verified": False, "compatibility_evidence": ""}
    manifest = Manifest(destination, data)
    manifest.validate()
    manifest.check_model_config(cfg)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(data, indent=2) + "\n")
    return manifest


def freeze_bundle(checkpoint, destination, *, revision, model_id, assets):
    """A training base artifact may have another embodiment; it is not a robot contract."""
    import os
    root, destination = Path(checkpoint).resolve(), Path(destination).resolve()
    cfg = json.loads((root / "config.json").read_text())
    data = {"schema_version": 1, "kind": "training_base_bundle", "policy_type": cfg["type"], "framework_sha": FORK_SHA, "model_id": model_id, "revision": revision, "checkpoint": os.path.relpath(root, destination.parent), "files": {str(p.relative_to(root)): sha256(p) for p in sorted(root.rglob("*")) if p.is_file() and ".cache" not in p.relative_to(root).parts}, "assets": assets}
    destination.write_text(json.dumps(data, indent=2) + "\n")
    return destination


def fetch_checkpoint(repo_id, directory, *, revision=None, weights=False, manifest_path=None):
    """Called only by an explicit fetch CLI; runtime never downloads artifacts."""
    from huggingface_hub import HfApi, snapshot_download
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    sha = HfApi().model_info(repo_id, revision=revision).sha
    if not re.fullmatch(r"[a-f0-9]{40}", sha):
        raise ValueError("Expected immutable Hub commit")
    patterns = ["*.json", "*.safetensors"] if weights else ["*.json", "policy_*safetensors"]
    snapshot_download(repo_id, revision=sha, local_dir=directory, allow_patterns=patterns)
    cfg = json.loads((directory / "config.json").read_text())
    result = {"model_id": repo_id, "revision": sha, "weights_downloaded": weights, "inspection": checkpoint_info(directory)}
    if not weights:
        return result
    assets = {}
    asset_models = {}
    if cfg["type"] == "smolvla":
        asset_models["vlm_model_name"] = cfg["vlm_model_name"]
    if cfg["type"] == "pi05":
        asset_models["text_tokenizer_name"] = cfg.get("text_tokenizer_name", "google/paligemma-3b-pt-224")
    for field, repo in asset_models.items():
        asset_sha = HfApi().model_info(repo).sha
        target = f"assets/{field}"
        # Full checkpoint contains backbone weights; external assets contain only config/tokenizer.
        snapshot_download(repo, revision=asset_sha, local_dir=directory / target, allow_patterns=["*.json", "*.model", "*.txt", "*.tiktoken", "*.jinja", "*.vocab", "*.merges"])
        assets[field] = {"model_id": repo, "revision": asset_sha, "path": target, "tokenizer": True}
    bundle = directory.parent / f"{directory.name}.bundle.local.json"
    result["training_base_bundle"] = str(freeze_bundle(directory, bundle, revision=sha, model_id=repo_id, assets=assets))
    state_shape = cfg.get("input_features", {}).get("observation.state", {}).get("shape")
    action_shape = cfg.get("output_features", {}).get("action", {}).get("shape")
    if state_shape != [6] or action_shape != [6]:
        result["deployment_manifest_blocked"] = "Requires a matched six-dimensional SO101 checkpoint; training base bundle is available"
        return result
    manifest_path = manifest_path or directory.parent / f"{directory.name}.manifest.local.json"
    manifest = freeze_checkpoint(directory, manifest_path, revision=sha, model_id=repo_id, assets=assets)
    result["manifest"] = str(manifest.path)
    return result
