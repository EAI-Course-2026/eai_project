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


def default_camera_fps():
    """Frame rate this machine's camera actually reports.

    LeRobot refuses to run when the camera cannot match the requested rate, so
    the value has to come from the machine rather than from a code constant: many
    640x480 modules only offer 30 and will ignore a request for 15.
    """
    value = load_config().get("camera", {}).get("fps", "")
    if value == "":
        return 15
    if type(value) is not int or value <= 0:
        raise ValueError("Set [camera].fps to a positive integer in hardware.local.toml")
    return value
