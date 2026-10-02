"""Bind migrated course tools to the shared calibration and guarded SCS215 bus.

The LeRobot framework comes from a fixed fork commit. Application code and
calibration live here, never in per-user Hugging Face cache copies.
"""
from pathlib import Path
import math
import time

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.motors.feetech import FeetechMotorsBus
from eai_robot.config import ROOT, load_config
from eai_robot.arm.calibration import JOINTS, MODEL_NUMBER, load

DEFAULT_CALIBRATION = ROOT / "calibration/scs215_so101.json"


def default_port():
    return load_config()["serial"]["port"]


class CourseMotorsBus(FeetechMotorsBus):
    """Read-only connection; torque changes do not unlock EEPROM."""
    def connect(self, handshake=False):
        try:
            super().connect(handshake=False)
            self.set_baudrate(load_config()["serial"]["baudrate"])
            for name in self.motors:
                if self.ping(name, num_retry=2, raise_on_error=True) != MODEL_NUMBER:
                    raise RuntimeError(f"{name}: expected SCS215 model {MODEL_NUMBER}")
                low = self.read("Min_Position_Limit", name, normalize=False)
                high = self.read("Max_Position_Limit", name, normalize=False)
                if not 0 <= low < high <= 1023:
                    raise RuntimeError(f"{name}: invalid EEPROM limits {low}..{high}")
                if name in self.calibration:
                    cal = self.calibration[name]
                    if cal.range_min < low or cal.range_max > high:
                        raise RuntimeError(f"{name}: calibration exceeds current EEPROM limits")
        except BaseException:
            self.port_handler.closePort()
            raise

    def _split_into_byte_chunks(self, value, length):
        return list(value.to_bytes(length, "big"))

    def write_verified(self, register, motor, value):
        name = motor if isinstance(motor, str) else self._id_to_name(motor)
        address, size = self.model_ctrl_table[self.motors[name].model][register]
        self.port_handler.ser.reset_input_buffer()
        result = self.packet_handler.writeTxOnly(
            self.port_handler, self.motors[name].id, address, size,
            self._serialize_data(value, size),
        )
        if result != self._comm_success:
            raise ConnectionError(self.packet_handler.getTxRxResult(result))
        self.port_handler.ser.flush()
        time.sleep(0.02)
        self.port_handler.ser.reset_input_buffer()
        if self.read(register, name, normalize=False) != value:
            raise RuntimeError(f"{name} {register}: write/readback mismatch")

    def disable_torque(self, motors=None, num_retry=0):
        errors = []
        for name in self._get_motors_list(motors):
            try:
                self.write_verified("Torque_Enable", name, 0)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError("Torque release failed: " + "; ".join(errors))

    def _disable_torque(self, motor, model, num_retry=0):
        self.write_verified("Torque_Enable", motor, 0)

    def enable_torque(self, motors=None, num_retry=0):
        names = self._get_motors_list(motors)
        try:
            for name in names:
                self.write_verified("Torque_Enable", name, 1)
        except BaseException:
            self.disable_torque(names)
            raise


class CourseFollower(SO101Follower):
    """Course Robot interface with explicit enable and no automatic recalibration."""
    def __init__(self, config):
        super().__init__(config)
        if any(m.model != "scs215" for m in self.bus.motors.values()):
            raise RuntimeError("Wrong LeRobot installation; run uv sync --locked to install the pinned SCS215 fork")
        self.bus = CourseMotorsBus(config.port, self.bus.motors, calibration=self.calibration, protocol_version=1)
        self.motion_enabled = False

    def connect(self, calibrate=True):
        try:
            self.bus.connect()
            for camera in self.cameras.values():
                camera.connect()
        except BaseException:
            self.disconnect()
            raise

    def calibrate(self):
        raise RuntimeError("Use explicit project calibration commands; automatic EEPROM calibration is disabled")

    def configure(self):
        # Connecting must not write PID gains, EEPROM limits or torque.
        pass

    def enable_motion(self, motors=None):
        names = self.bus._get_motors_list(motors)
        goals = {}
        for name in names:
            if self.bus.read("Torque_Enable", name, normalize=False) != 0:
                raise RuntimeError(f"{name}: torque must be off before enabling")
            raw = self.bus.read("Present_Position", name, normalize=False)
            cal = self.calibration[name]
            if not cal.range_min <= raw <= cal.range_max:
                raise RuntimeError(f"{name}: start position is outside calibration")
            if self.bus.read("Status", name, normalize=False):
                raise RuntimeError(f"{name}: motor alarm")
            goals[name] = raw
        # Clear stale time and seed goals before enabling. Speed is preserved.
        for name in names:
            self.bus.write_verified("Running_Time", name, 0)
        self.bus.sync_write("Goal_Position", goals, normalize=False)
        self.bus.enable_torque(names)
        self.motion_enabled = True

    def send_action(self, action):
        if not self.motion_enabled:
            raise RuntimeError("Motion requires explicit enable_motion()")
        for key, value in action.items():
            name = key.removesuffix(".pos")
            low = 0 if name == "gripper" else -100
            if key != f"{name}.pos" or name not in JOINTS or not math.isfinite(value) or not low <= value <= 100:
                raise ValueError(f"Invalid normalized action: {key}={value}")
        return super().send_action(action)

    def disconnect(self):
        try:
            if self.bus.is_connected:
                self.bus.disconnect(disable_torque=self.motion_enabled)
        finally:
            self.motion_enabled = False
            for camera in self.cameras.values():
                if camera.is_connected:
                    camera.disconnect()


def make_robot(port, robot_id="scs215_so101", *, calibration_path=DEFAULT_CALIBRATION, cameras=None):
    path = Path(calibration_path)
    load(path)  # Validate all IDs, ranges and direction flags before opening hardware.
    config = SO101FollowerConfig(
        port=port, id=path.stem, calibration_dir=path.parent,
        cameras=cameras or {}, disable_torque_on_disconnect=False,
    )
    return CourseFollower(config)
