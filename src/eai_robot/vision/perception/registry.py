"""Where detectors come from, and what to say when one is missing.

Optional perception dependencies are exactly that -- optional. The control
package must keep importing, testing and driving the arm on a machine with no
MediaPipe, no transformers and no model weights. The registry is therefore a
lookup that fails *informatively*:

    TrackerUnavailable: detector 'mediapipe-hands' is catalogued but not
    implemented: 21 hand landmarks at 30+ Hz on CPU. Install with
    `uv pip install "mediapipe>=0.10,<0.11"`. Weights are bundled with the
    wheel, nothing is downloaded.

A traceback ending in ``ModuleNotFoundError: No module named 'mediapipe'`` tells
nobody what to do. The catalogue below is the difference between a skeleton and
a dead end: every candidate detector states its dependency, its expected latency
on this class of machine, what evidence it returns, and whether it needs weights.
Adding one is a single ``register`` call plus an implementation.

These families are deliberately *not* project extras. Each one carries a
requirement that cannot be resolved together with this project's lock -- SAM 2
is distributed only from git, MediaPipe pins its own NumPy, an open-vocabulary
YOLO pulls a full PyTorch build -- and an unresolvable extra breaks
``uv run --locked`` for every user of the servo loop, including the ones who
will never load a model. The exact command travels with the entry instead, and
``docs/experiments/perception_infrastructure.md`` states the same commands. A
detector that becomes reproducible enough to lock can be promoted to an extra
later; until then the lockfile stays about the arm.

Latency figures are order-of-magnitude budgets for the *declared* hardware
(CPU, or Apple MPS) and are used only to set ``latency_budget_s`` -- the value
``AsyncDetector.health`` compares against. They are not benchmarks.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from eai_robot.vision.perception.spec import ModelSpec
from eai_robot.vision.tracking.base import TrackerUnavailable

DetectorFactory = Callable[[ModelSpec], "ObjectDetector"]


@dataclass(frozen=True)
class DetectorEntry:
    """A candidate detector: what it needs and what it can promise."""

    name: str
    install: str             # exact command that makes the dependency importable
    summary: str
    evidence: str            # mask | keypoints | box
    latency_s: float         # order-of-magnitude budget, not a benchmark
    needs_weights: bool
    tasks: tuple[str, ...] = ()   # hand / generic object / segmentation

    def describe(self) -> dict[str, object]:
        return {
            "name": self.name,
            "implemented": is_registered(self.name),
            "install": self.install,
            "summary": self.summary,
            "evidence": self.evidence,
            "latency_budget_s": self.latency_s,
            "needs_weights": self.needs_weights,
            "tasks": list(self.tasks),
        }


#: Candidates the architecture is designed around. Registration is what makes
#: one usable; the entry is what makes it discoverable and its absence legible.
CATALOGUE: tuple[DetectorEntry, ...] = (
    DetectorEntry(
        name="dark-blob", install="nothing -- NumPy and OpenCV only",
        summary="threshold + contour compactness; the existing black-ball path",
        evidence="mask", latency_s=0.03, needs_weights=False,
        tasks=("generic object",)),
    DetectorEntry(
        name="mediapipe-hands", install='uv pip install "mediapipe>=0.10,<0.11"',
        summary="21 hand landmarks at 30+ Hz on CPU",
        evidence="keypoints", latency_s=0.03, needs_weights=False,
        tasks=("hand",)),
    DetectorEntry(
        name="owlv2",
        install='uv pip install "transformers>=4.44,<5" torch',
        summary="open-vocabulary boxes from a text prompt (OWLv2 / Grounding DINO)",
        evidence="box", latency_s=0.35, needs_weights=True,
        tasks=("generic object",)),
    DetectorEntry(
        name="sam2",
        install='uv pip install "git+https://github.com/facebookresearch/sam2"',
        summary="promptable segmentation (SAM 2) for a stable mask and extent",
        evidence="mask", latency_s=0.25, needs_weights=True,
        tasks=("generic object", "segmentation")),
    DetectorEntry(
        name="yolo-world", install='uv pip install "ultralytics>=8.2"',
        summary="open-vocabulary YOLO; faster than OWLv2, coarser masks",
        evidence="box", latency_s=0.08, needs_weights=True,
        tasks=("generic object",)),
)

_BY_NAME = {entry.name: entry for entry in CATALOGUE}
_FACTORIES: dict[str, DetectorFactory] = {}
_BUILTINS_LOADED = False


def _load_builtins() -> None:
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED:
        return
    _BUILTINS_LOADED = True

    def build_dark_blob(spec: ModelSpec):
        from eai_robot.vision.perception.builtin import DarkBlobDetector
        return DarkBlobDetector(
            black_threshold=int(spec.options.get("black_threshold", 75)),
            min_area_ratio=float(spec.options.get("min_area_ratio", 0.002)),
            max_area_ratio=float(spec.options.get("max_area_ratio", 0.25)),
            min_circularity=float(spec.options.get("min_circularity", 0.55)),
        )

    register("dark-blob", build_dark_blob)


def register(name: str, factory: DetectorFactory) -> None:
    """Make a detector available by name. Later registrations win."""
    if name not in _BY_NAME:
        raise ValueError(f"unknown detector {name!r}; add it to CATALOGUE first")
    _FACTORIES[name] = factory


def is_registered(name: str) -> bool:
    _load_builtins()
    return name in _FACTORIES


def available() -> dict[str, dict[str, object]]:
    _load_builtins()
    return {name: entry.describe() for name, entry in _BY_NAME.items()}


def create(spec: ModelSpec):
    """Instantiate a detector, or explain precisely why that is impossible."""
    _load_builtins()
    entry = _BY_NAME.get(spec.name)
    if entry is None:
        raise TrackerUnavailable(
            f"unknown detector {spec.name!r}; known: {sorted(_BY_NAME)}")
    factory = _FACTORIES.get(spec.name)
    if factory is None:
        weights = ("weights are not bundled; loading them needs an explicit "
                   "--allow-weight-download and the hash goes into the record"
                   if entry.needs_weights else
                   "weights ship with the wheel, nothing is downloaded")
        raise TrackerUnavailable(
            f"detector {spec.name!r} is catalogued but not implemented: "
            f"{entry.summary}. Install with: {entry.install}. {weights}. "
            f"Implement {entry.name!r} against ObjectDetector and call "
            f"register({entry.name!r}, factory) in eai_robot.vision.perception.registry.")
    spec.validate_weights()
    resolved = spec.resolved()
    detector = factory(resolved)
    # The device resolution must be visible in the record; an "auto" that ran on
    # CPU is the difference between a working demo and an unexplained 10x.
    if getattr(detector, "name", None) != spec.name:
        raise TrackerUnavailable(
            f"factory for {spec.name!r} returned {getattr(detector, 'name', None)!r}")
    return detector
