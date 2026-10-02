"""项目路径与硬件配置；导入不打开串口。"""
from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parents[2]


def _merge_config(base, local):
    result = dict(base)
    for key, value in local.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _merge_config(result[key], value)
        else:
            result[key] = value
    return result


def load_config():
    local = ROOT / "configs/hardware.local.toml"
    example = ROOT / "configs/hardware.example.toml"
    with example.open("rb") as stream:
        config = tomllib.load(stream)
    if local.exists():
        with local.open("rb") as stream:
            config = _merge_config(config, tomllib.load(stream))
    return config


def default_camera_index():
    value = load_config().get("camera", {}).get("index_or_path", "")
    if value == "":
        return None
    if type(value) is not int or value < 0:
        raise ValueError("Set [camera].index_or_path to a non-negative integer in hardware.local.toml")
    return value
