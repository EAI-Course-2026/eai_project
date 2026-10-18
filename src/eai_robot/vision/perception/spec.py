"""Model specification: what to load, from where, on which device.

Two policies are encoded here rather than left to each detector.

**Weights are never fetched implicitly.** The repository contract forbids
committing models, and a detector that silently downloads a 700 MB checkpoint on
first use produces two failures: the lab machine has no network, and two runs of
"the same" experiment load different bytes. A missing weight is therefore a
:class:`TrackerUnavailable` with the path it looked for, unless the caller sets
``allow_download``. Reproducibility is then a property of the record, not of the
operator's memory.

**Device selection is explicit and reported.** ``auto`` resolves once, at load
time, and the resolution is stored on the spec so the log says ``mps`` rather
than "auto". A detector running on CPU instead of the GPU it was designed for is
a latency cliff, not a mystery.

No heavy import happens at module import time: ``torch`` is only touched when a
device actually has to be resolved, so the pure-logic tests stay fast.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

DEVICE_CHOICES = ("auto", "cpu", "mps", "cuda")
DTYPE_CHOICES = ("float32", "float16", "bfloat16")


def available_devices() -> tuple[str, ...]:
    """Devices this interpreter can actually use, cheapest check first."""
    devices = ["cpu"]
    try:
        import torch
    except ImportError:  # pragma: no cover - torch is a core dependency
        return tuple(devices)
    if torch.cuda.is_available():
        devices.append("cuda")
    backends = getattr(torch, "backends", None)
    if backends is not None and getattr(backends, "mps", None) is not None:
        if backends.mps.is_available():
            devices.append("mps")
    return tuple(devices)


def resolve_device(preference: str = "auto") -> str:
    """Turn a preference into one concrete device name or raise."""
    if preference not in DEVICE_CHOICES:
        raise ValueError(f"device must be one of {DEVICE_CHOICES}")
    devices = available_devices()
    if preference == "auto":
        for candidate in ("cuda", "mps", "cpu"):
            if candidate in devices:
                return candidate
        return "cpu"
    if preference not in devices:
        raise RuntimeError(
            f"device {preference!r} is not available here (have {devices})")
    return preference


@dataclass(frozen=True)
class ModelSpec:
    """Everything needed to load one detector, and nothing model-specific."""

    name: str
    weights: Path | None = None
    device: str = "auto"
    dtype: str = "float32"
    input_size: tuple[int, int] | None = None
    allow_download: bool = False
    options: dict[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("model name is required")
        if self.device not in DEVICE_CHOICES:
            raise ValueError(f"device must be one of {DEVICE_CHOICES}")
        if self.dtype not in DTYPE_CHOICES:
            raise ValueError(f"dtype must be one of {DTYPE_CHOICES}")
        if self.input_size is not None:
            width, height = self.input_size
            if width <= 0 or height <= 0:
                raise ValueError("input size must be positive")
        object.__setattr__(self, "options", dict(self.options))

    def validate_weights(self) -> None:
        """Refuse a load that would need an implicit download."""
        if self.weights is None:
            return
        path = Path(self.weights)
        if path.exists():
            return
        if self.allow_download:
            return
        raise _unavailable(
            f"weights for {self.name!r} not found at {path}",
            "set allow_download, or place the weights there yourself")

    def resolved(self) -> "ModelSpec":
        """A copy whose device is concrete; safe to log."""
        if self.device != "auto":
            return self
        return ModelSpec(self.name, self.weights, resolve_device("auto"),
                         self.dtype, self.input_size, self.allow_download,
                         dict(self.options))

    def describe(self) -> dict[str, object]:
        return {
            "name": self.name,
            "weights": None if self.weights is None else str(self.weights),
            "device": self.device,
            "dtype": self.dtype,
            "input_size": list(self.input_size) if self.input_size else None,
            "allow_download": self.allow_download,
            "options": dict(self.options),
        }


def _unavailable(message: str, hint: str = "") -> Exception:
    # Imported here to keep this module free of a cycle with ``base``.
    from eai_robot.vision.tracking.base import TrackerUnavailable
    return TrackerUnavailable(message + (f"; {hint}" if hint else ""))
