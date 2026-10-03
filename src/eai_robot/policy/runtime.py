"""Strict local checkpoint loading and independent postprocessed action chunks."""
import importlib.metadata
import json
import os
import time
import numpy as np

from .contracts import FORK_SHA, FORK_URL


def check_fork():
    dist = importlib.metadata.distribution("lerobot")
    source = json.loads(dist.read_text("direct_url.json") or "{}")
    if dist.version != "0.6.2" or source.get("url") != FORK_URL or source.get("vcs_info", {}).get("commit_id") != FORK_SHA:
        raise RuntimeError("Install the exact pinned LeRobot fork using the locked environment")


def load_pi05_strict(policy_class, config, root):
    """The pinned PI05.from_pretrained catches loading errors and returns random weights.

    Reuse its key remapping through the application adapter, but propagate every load
    error. Never treat a randomly initialized PI05 as a deployed checkpoint.
    """
    from safetensors.torch import load_file
    original = load_file(str(root / "model.safetensors"))
    policy = policy_class(config)
    fixed = policy._fix_pytorch_state_dict_keys(original, config)
    remapped = {key if key.startswith("model.") else f"model.{key}": value for key, value in fixed.items()}
    if len(remapped) != len(fixed):
        raise ValueError("PI05 checkpoint has colliding remapped keys")
    policy.load_state_dict(policy._prepare_pretrained_state_dict(remapped), strict=True)
    policy.to(config.device)
    policy.eval()
    return policy


class FakeRuntime:
    def __init__(self, manifest):
        self.manifest = manifest

    def predict(self, observation):
        observation.validate(self.manifest)
        return np.repeat(observation.state[None], self.manifest.data["max_chunk_steps"], axis=0), 0.0


class LeRobotRuntime:
    def __init__(self, manifest, device="cpu"):
        check_fork()
        manifest.verify_files()
        config_data = json.loads(manifest.artifact_path("config.json").read_text())
        manifest.check_model_config(config_data)
        # Nested VLM/tokenizer loaders must not fall back to a moving Hub revision.
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["TRANSFORMERS_OFFLINE"] = "1"
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        from lerobot.processor import bind_relative_anchor
        if device not in {"cpu", "mps", "cuda"}:
            raise ValueError("Supported devices: cpu, mps, cuda")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA unavailable; run GPU doctor on the inference host")
        cfg = PreTrainedConfig.from_pretrained(manifest.root, local_files_only=True)
        cfg.device = device
        if hasattr(cfg, "compile_model"):
            cfg.compile_model = False
        overrides = {"device_processor": {"device": device}}
        assets = manifest.data.get("assets", {})
        for field in ("vlm_model_name", "text_tokenizer_name"):
            if hasattr(cfg, field):
                if field not in assets:
                    raise ValueError(f"Missing frozen external asset: {field}")
                setattr(cfg, field, str(manifest.artifact_path(assets[field]["path"])))
        if hasattr(cfg, "pretrained_backbone_weights"):
            cfg.pretrained_backbone_weights = None  # Full checkpoint includes the backbone.
        # Full SmolVLA state dict already contains the VLM. Instantiate from frozen config.
        if hasattr(cfg, "load_vlm_weights"):
            cfg.load_vlm_weights = False
        tokenizer_assets = [a for a in assets.values() if a.get("tokenizer", False)]
        if tokenizer_assets:
            if len(tokenizer_assets) != 1:
                raise ValueError("Exactly one tokenizer asset supported")
            overrides["tokenizer_processor"] = {"tokenizer_name": str(manifest.artifact_path(tokenizer_assets[0]["path"]))}
        policy_class = get_policy_class(cfg.type)
        self.policy = load_pi05_strict(policy_class, cfg, manifest.root) if cfg.type == "pi05" else policy_class.from_pretrained(manifest.root, config=cfg, local_files_only=True, strict=True)
        self.pre, self.post = make_pre_post_processors(cfg, pretrained_path=manifest.root, preprocessor_overrides=overrides)
        bind_relative_anchor(self.policy, self.pre)
        self.manifest, self.device = manifest, device

    def predict(self, observation):
        import torch
        self.policy.reset()
        self.pre.reset()
        self.post.reset()
        batch = observation.model_batch(self.manifest)
        self._sync()
        started = time.perf_counter()
        with torch.inference_mode():
            batch = self.pre(batch)
            action = self.post(self.policy.predict_action_chunk(batch))
        self._sync()
        elapsed = time.perf_counter() - started
        horizon = self.policy.config.n_action_steps
        array = action.detach().float().cpu().numpy()
        if array.ndim != 3 or array.shape[0] != 1 or array.shape[2] != 6 or array.shape[1] < horizon or not np.isfinite(array).all():
            raise ValueError("Postprocessor must produce finite [1, T, 6] actions")
        return array[0, :horizon].copy(), elapsed

    def _sync(self):
        import torch
        if self.device == "cuda":
            torch.cuda.synchronize()
        elif self.device == "mps":
            torch.mps.synchronize()


def make_runtime(manifest, device="cpu"):
    return FakeRuntime(manifest) if manifest.data["policy_type"] == "fake" else LeRobotRuntime(manifest, device)
