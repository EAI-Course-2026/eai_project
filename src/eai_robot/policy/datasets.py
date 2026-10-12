"""Offline conversion of successful operator episodes and checked training recipes."""
import json
from pathlib import Path
import numpy as np

from .contracts import JOINTS, UNITS, sha256
from .recording import validate_episode


def export_dataset(episodes, manifest, *, root, repo_id, split_file):
    """Local only; export actual sent commands as action labels, never model requests."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    import cv2
    episodes = [Path(p).resolve() for p in episodes]
    if len(episodes) < 2 or len(set(episodes)) != len(episodes):
        raise ValueError("At least two distinct episodes required for held-out evaluation")
    split = json.loads(Path(split_file).read_text())
    train, evaluation = split.get("train", []), split.get("evaluation", [])
    if not train or not evaluation or len(set(train + evaluation)) != len(train + evaluation) or set(train + evaluation) != set(range(len(episodes))):
        raise ValueError("Explicit disjoint episode-level train/evaluation split must cover all episodes")
    hz = manifest.data["action_hz"]
    if hz != int(hz):
        raise ValueError("LeRobot export currently requires integer action_hz")
    reports = [validate_episode(p, manifest, demonstrations=True) for p in episodes]
    scene_ids = [r["header"].get("scene_id") for r in reports]
    if any(not isinstance(s, str) or not s.strip() for s in scene_ids):
        raise ValueError("Every demonstration needs an explicit scene_id")
    if {scene_ids[i] for i in train} & {scene_ids[i] for i in evaluation}:
        raise ValueError("Train/evaluation scenes must be disjoint")
    features = {"observation.state": {"dtype": "float32", "shape": (6,), "names": [f"{n}.pos" for n in JOINTS]}, "action": {"dtype": "float32", "shape": (6,), "names": [f"{n}.pos" for n in JOINTS]}}
    for key, spec in manifest.data["cameras"].items():
        features[key] = {"dtype": "image", "shape": (spec["shape"][1], spec["shape"][2], 3), "names": ["height", "width", "channels"]}
    dataset = LeRobotDataset.create(repo_id=repo_id, fps=int(hz), root=Path(root), robot_type="scs215_so101_follower", features=features, use_videos=False)
    try:
        for directory, report in zip(episodes, reports):
            for row in report["rows"]:
                with np.load(directory / row["frame"], allow_pickle=False) as obs:
                    frame = {"observation.state": obs["state"].astype(np.float32), "action": np.asarray(row["sent"], dtype=np.float32), "task": row["task"]}
                    for key, spec in manifest.data["cameras"].items():
                        frame[key] = cv2.resize(obs[spec["source"]], (spec["shape"][2], spec["shape"][1]))
                    dataset.add_frame(frame)
            dataset.save_episode()
    finally:
        dataset.finalize()
    provenance = {"schema_version": 1, "manifest_fingerprint": manifest.fingerprint, "state_units": manifest.data["state_units"], "action_units": manifest.data["action_units"], "calibration_sha256": manifest.data["calibration_sha256"], "split": split, "episodes": [{"index": i, "journal_sha256": sha256(p / "frames.jsonl"), "frames": r["frames"]} for i, (p, r) in enumerate(zip(episodes, reports))]}
    (Path(root) / "meta/eai_provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return {"episodes": len(episodes), "frames": sum(r["frames"] for r in reports), "uploaded": False}


def training_recipe(policy_type, dataset_root, repo_id, *, output_dir, base_bundle=None):
    root = Path(dataset_root)
    info = json.loads((root / "meta/info.json").read_text())
    stats = json.loads((root / "meta/stats.json").read_text())
    provenance = json.loads((root / "meta/eai_provenance.json").read_text())
    if provenance["state_units"] != UNITS or provenance["action_units"] != UNITS:
        raise ValueError("Dataset must have verified SCS215 normalized units")
    if info["features"]["observation.state"]["shape"] != [6] or info["features"]["action"]["shape"] != [6]:
        raise ValueError("Dataset must contain six dimensional state and sent action")
    episodes = provenance["split"]["train"]
    if policy_type not in {"act", "smolvla", "pi05"}:
        raise ValueError("Supported training policies: act, smolvla, pi05")
    args = ["lerobot-train", f"--policy.type={policy_type}", f"--dataset.repo_id={repo_id}", f"--dataset.root={root}", "--dataset.episodes=" + json.dumps(episodes, separators=(",", ":")), f"--output_dir={output_dir}", "--policy.device=cuda", "--policy.push_to_hub=false", "--wandb.enable=false"]
    if policy_type != "act":
        if base_bundle is None:
            raise ValueError("A frozen training base bundle is required for VLA fine-tuning")
        from .contracts import Manifest, FORK_SHA
        bundle_path = Path(base_bundle).resolve()
        bundle = json.loads(bundle_path.read_text())
        if bundle.get("kind") != "training_base_bundle" or bundle.get("framework_sha") != FORK_SHA or bundle.get("policy_type") != policy_type:
            raise ValueError("Training bundle policy/framework mismatch")
        # Artifact methods are shared; a base bundle has no deployment semantics.
        artifacts = Manifest(bundle_path, bundle)
        artifacts.verify_files()
        base_checkpoint = artifacts.root
        if "model.safetensors" not in bundle["files"]:
            raise ValueError("Training bundle is missing weights")
        args += [f"--policy.pretrained_path={base_checkpoint}"]
        field = "vlm_model_name" if policy_type == "smolvla" else "text_tokenizer_name"
        asset = bundle["assets"].get(field)
        if not asset:
            raise ValueError(f"Training bundle missing frozen {field}")
        args += [f"--policy.{field}={artifacts.artifact_path(asset['path'])}"]
        if policy_type == "smolvla":
            args += ["--policy.load_vlm_weights=false"]
    if policy_type == "pi05":
        for key in ("observation.state", "action"):
            if any(name not in stats.get(key, {}) for name in ("q01", "q99")):
                raise ValueError("PI05 QUANTILES requires q01/q99; compute stats before training")
        args += ["--policy.dtype=bfloat16", "--policy.gradient_checkpointing=true", "--policy.compile_model=false"]
    args += ["--batch_size=1", "--steps=20000"]
    return {"argv": args, "evaluation_episodes": provenance["split"]["evaluation"], "notes": ["Start with GPU doctor and batch 1; tune memory and convergence using real evidence", "Base checkpoint config/settings and camera mapping must be reviewed against dataset; pretrained_path loads weights, not deployment settings", "This generates a recipe and does not launch training or upload data"]}
