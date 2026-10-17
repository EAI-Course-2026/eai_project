"""Registered SCS215 SO101 Robot; standard CLIs use the same implementation as course tools."""

from dataclasses import dataclass, field, replace, asdict
from functools import cached_property
import math
from numbers import Real
from pathlib import Path

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.configs import ColorMode
from lerobot.robots import Robot, RobotConfig
from .bus import SCS215MotorsBus
from .safety import JOINTS, feedback_in_range


@RobotConfig.register_subclass("scs215_so101_follower")
@dataclass
class SCS215SO101FollowerConfig(RobotConfig):
    port: str
    baudrate: int = 1_000_000
    calibration_path: Path | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
    # Standard CLI connect may enable motion only with an explicit user flag.
    enable_motion: bool = False
    max_relative_target: float | dict[str, float] | None = None
    num_read_retries: int = 2


class SCS215SO101Follower(Robot):
    config_class = SCS215SO101FollowerConfig
    name = "scs215_so101_follower"

    def __init__(self, config):
        for name, camera in config.cameras.items():
            if name in {f"{joint}.pos" for joint in JOINTS}:
                raise ValueError("Camera name conflicts with motor observation")
            if (
                getattr(camera, "use_depth", False)
                or not getattr(camera, "use_rgb", True)
                or getattr(camera, "color_mode", ColorMode.RGB) != ColorMode.RGB
            ):
                raise ValueError("SCS215 plugin currently supports RGB cameras only")
        if config.num_read_retries < 0 or config.num_read_retries > 10:
            raise ValueError("num_read_retries must be 0..10")
        caps = config.max_relative_target
        if caps is not None:
            vals = caps.values() if isinstance(caps, dict) else [caps]
            if isinstance(caps, dict) and not set(caps) <= set(JOINTS):
                raise ValueError("Unknown max_relative_target motor")
            if any(not math.isfinite(x) or x <= 0 for x in vals):
                raise ValueError("max_relative_target must be finite and positive")
        if config.calibration_path is not None:
            path = Path(config.calibration_path).resolve()
        elif config.calibration_dir is not None:
            if not config.id:
                raise ValueError("calibration_dir requires id")
            path = Path(config.calibration_dir).resolve() / f"{config.id}.json"
        else:
            path = Path("calibration/scs215_so101.json").resolve()
        # Robot's file loading is reused, but the shared file is not selected by a host-specific id.
        super().__init__(replace(config, id=path.stem, calibration_dir=path.parent))
        self.id = config.id or path.stem
        self.config = config
        self._validate_calibration(self.calibration, allow_empty=True)
        self.bus = SCS215MotorsBus(
            config.port, calibration=self.calibration, baudrate=config.baudrate
        )
        self.cameras = make_cameras_from_configs(config.cameras)
        self.motion_enabled = False
        self.torque_touched = False
        self._torque_motors = set()

    @staticmethod
    def _validate_calibration(calibration, allow_empty=False):
        if allow_empty and not calibration:
            return
        if set(calibration) != set(JOINTS):
            raise ValueError("Calibration must contain exactly the six SCS215 joints")
        for sid, name in enumerate(JOINTS, 1):
            c = calibration[name]
            if (
                any(type(v) is not int for v in asdict(c).values())
                or c.id != sid
                or c.drive_mode not in (0, 1)
                or c.homing_offset != 0
                or not 0 <= c.range_min < c.range_max <= 1023
            ):
                raise ValueError(f"Invalid SCS215 calibration: {name}")

    @cached_property
    def action_features(self):
        return {f"{name}.pos": float for name in JOINTS}

    @cached_property
    def observation_features(self):
        return {
            **self.action_features,
            **{name: (cam.height, cam.width, 3) for name, cam in self.cameras.items()},
        }

    @property
    def is_connected(self):
        return self.bus.is_connected and all(
            c.is_connected for c in self.cameras.values()
        )

    @property
    def is_calibrated(self):
        return bool(self.calibration) and all(
            name in self.bus.hardware_limits
            and self.bus.hardware_limits[name][0]
            <= c.range_min
            < c.range_max
            <= self.bus.hardware_limits[name][1]
            for name, c in self.calibration.items()
        )

    def connect(self, calibrate=True):
        if self.is_connected:
            raise RuntimeError("Robot is already connected")
        try:
            self.bus.connect(validate_calibration=calibrate)
            if calibrate and not self.is_calibrated:
                raise RuntimeError(
                    "Missing/incompatible calibration; run lerobot-calibrate explicitly"
                )
            for cam in self.cameras.values():
                cam.connect()
            if calibrate and self.config.enable_motion:
                self.enable_motion()
        except BaseException:
            try:
                self.disconnect()
            except Exception as cleanup_error:
                raise RuntimeError(
                    f"Connect failed; cleanup also failed: {cleanup_error}"
                )
            raise

    def configure(self):
        pass

    def enable_motion(self, motors=None):
        if not self.bus.is_connected:
            raise RuntimeError("Robot is not connected")
        self._validate_calibration(self.calibration)
        if not self.is_calibrated:
            raise RuntimeError("Calibration must fit the connected hardware limits")
        names = self.bus._get_motors_list(motors)
        if not names:
            raise ValueError("Select at least one motor to enable")
        goals = {}
        for name in names:
            if self.bus.read("Torque_Enable", name, normalize=False) != 0:
                raise RuntimeError(f"{name}: torque must be off before enabling")
            raw = self.bus.read("Present_Position", name, normalize=False)
            c = self.calibration[name]
            if not feedback_in_range(raw, c.range_min, c.range_max):
                raise RuntimeError(f"{name}: start position is outside calibration")
            if self.bus.read("Status", name, normalize=False):
                raise RuntimeError(f"{name}: motor alarm")
            goals[name] = max(c.range_min, min(c.range_max, raw))
        for name in names:
            self.bus.write_verified("Running_Time", name, 0)
        self.bus.sync_write("Goal_Position", goals, normalize=False)
        self.torque_touched = True
        self._torque_motors.update(names)
        self.bus.enable_torque(names)
        self.motion_enabled = True

    def get_observation(self):
        if not self.is_connected or not self.is_calibrated:
            raise RuntimeError("Robot must be connected and calibrated")
        raw = {
            name: self.bus.read(
                "Present_Position",
                name,
                normalize=False,
                num_retry=self.config.num_read_retries,
            )
            for name in JOINTS
        }
        for name, value in raw.items():
            c = self.calibration[name]
            if not feedback_in_range(value, c.range_min, c.range_max):
                raise RuntimeError(f"{name}: observation outside calibration")
        normalized = self.bus._normalize(
            {self.bus.motors[name].id: value for name, value in raw.items()}
        )
        observation = {
            f"{name}.pos": float(normalized[self.bus.motors[name].id])
            for name in JOINTS
        }
        for name, cam in self.cameras.items():
            observation[name] = cam.async_read()
        return observation

    def send_action(self, action, *, before_write=None):
        if not self.motion_enabled or not self.bus.is_connected:
            raise RuntimeError(
                "Motion requires explicit enable_motion() or --robot.enable_motion=true"
            )
        goals = {}
        for key, value in action.items():
            name = key.removesuffix(".pos")
            low = 0 if name == "gripper" else -100
            if (
                key != f"{name}.pos"
                or name not in JOINTS
                or not isinstance(value, Real)
                or isinstance(value, bool)
                or not math.isfinite(value)
                or not low <= value <= 100
            ):
                raise ValueError(f"Invalid normalized action: {key}={value}")
            c = self.calibration[name]
            raw = self.bus.read(
                "Present_Position",
                name,
                normalize=False,
                num_retry=self.config.num_read_retries,
            )
            if self.bus.read("Torque_Enable", name, normalize=False) != 1:
                raise RuntimeError(f"{name}: torque lost; explicit re-enable required")
            if self.bus.read("Status", name, normalize=False) or not feedback_in_range(
                raw, c.range_min, c.range_max
            ):
                raise RuntimeError(
                    f"{name}: motor alarm or feedback outside calibration"
                )
            cap = self.config.max_relative_target
            cap = cap.get(name) if isinstance(cap, dict) else cap
            if cap is not None:
                current = float(
                    self.bus.read(
                        "Present_Position", name, num_retry=self.config.num_read_retries
                    )
                )
                # Reject rather than silently clip: this fork's record loop stores
                # the requested action, so a successful write must match it.
                if abs(value - current) > cap:
                    raise ValueError(f"{name}: action exceeds max_relative_target")
            goals[name] = float(value)
        if goals:
            if before_write is not None:
                before_write()
            self.bus.sync_write("Goal_Position", goals)
        return {f"{name}.pos": value for name, value in goals.items()}

    def calibrate(self):
        from .calibration import calibrate_robot
        return calibrate_robot(self)

    def disconnect(self):
        errors = []
        if self.bus.is_connected:
            try:
                if self.torque_touched:
                    self.bus.disable_torque(
                        [n for n in JOINTS if n in self._torque_motors]
                    )
                    self.torque_touched = False
                    self._torque_motors.clear()
                self.bus.disconnect(disable_torque=False)
            except Exception as e:
                errors.append(e)
                self.bus.port_handler.closePort()
        self.motion_enabled = False
        for cam in self.cameras.values():
            if cam.is_connected:
                try:
                    cam.disconnect()
                except Exception as e:
                    errors.append(e)
        if errors:
            raise RuntimeError("Disconnect failed: " + "; ".join(map(str, errors)))
