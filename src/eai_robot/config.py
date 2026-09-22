"""项目路径与硬件配置；导入不打开串口。"""
from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parents[2]


def load_config():
    local = ROOT / "configs/hardware.local.toml"
    example = ROOT / "configs/hardware.example.toml"
    with (local if local.exists() else example).open("rb") as stream:
        return tomllib.load(stream)
