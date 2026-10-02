"""One SCS215 transport for direct scripts and the standard LeRobot Robot API."""

from copy import deepcopy
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from .safety import JOINTS, MODEL_NUMBER


class SCS215MotorsBus(FeetechMotorsBus):
    # Per-class copies only: never modify LeRobot's process-wide model tables.
    model_number_table = {
        **FeetechMotorsBus.model_number_table,
        "scs215": MODEL_NUMBER,
        "scs_series": MODEL_NUMBER,
    }
    model_ctrl_table = deepcopy(FeetechMotorsBus.model_ctrl_table)
    model_ctrl_table["scs215"] = {
        k: v
        for k, v in model_ctrl_table["scs_series"].items()
        if k not in {"Return_Delay_Time", "I_Coefficient", "Acceleration"}
    }
    model_resolution_table = {**FeetechMotorsBus.model_resolution_table, "scs215": 1024}
    model_baudrate_table = {
        **FeetechMotorsBus.model_baudrate_table,
        "scs215": deepcopy(FeetechMotorsBus.model_baudrate_table["scs_series"]),
    }
    model_encoding_table = {
        **FeetechMotorsBus.model_encoding_table,
        "scs215": deepcopy(FeetechMotorsBus.model_encoding_table["scs_series"]),
    }

    def __init__(
        self,
        port,
        motors=None,
        calibration=None,
        protocol_version=1,
        *,
        baudrate=1_000_000,
    ):
        if protocol_version != 1:
            raise ValueError("SCS215 requires protocol 1")
        if baudrate not in self.model_baudrate_table["scs215"]:
            raise ValueError("Unsupported SCS215 baudrate")
        if motors is None:
            motors = {
                name: Motor(
                    sid,
                    "scs215",
                    MotorNormMode.RANGE_0_100
                    if name == "gripper"
                    else MotorNormMode.RANGE_M100_100,
                )
                for sid, name in enumerate(JOINTS, 1)
            }
        if any(m.model != "scs215" for m in motors.values()):
            raise ValueError("This bus only supports SCS215")
        self.configured_baudrate = baudrate
        self.hardware_limits = {}
        super().__init__(port, motors, calibration=calibration, protocol_version=1)

    def connect(self, handshake=False, *, validate_calibration=True):
        try:
            super().connect(handshake=False)
            self.set_baudrate(self.configured_baudrate)
            for name in self.motors:
                if self.ping(name, num_retry=2, raise_on_error=True) != MODEL_NUMBER:
                    raise RuntimeError(f"{name}: expected SCS215 model {MODEL_NUMBER}")
                if self.read("ID", name, normalize=False) != self.motors[name].id:
                    raise RuntimeError(f"{name}: ID mismatch")
                low = self.read("Min_Position_Limit", name, normalize=False)
                high = self.read("Max_Position_Limit", name, normalize=False)
                if not 0 <= low < high <= 1023:
                    raise RuntimeError(f"{name}: invalid EEPROM limits {low}..{high}")
                self.hardware_limits[name] = (low, high)
                if validate_calibration and name in self.calibration:
                    c = self.calibration[name]
                    if c.range_min < low or c.range_max > high:
                        raise RuntimeError(
                            f"{name}: calibration exceeds current EEPROM limits"
                        )
        except BaseException:
            self.hardware_limits = {}
            if self.port_handler.ser is not None:
                self.port_handler.closePort()
            raise

    def configure_motors(self, *args, **kwargs):
        # No STS PID, mode, return-delay or acceleration writes on SCS215 connect.
        pass

    def write_calibration(self, *args, **kwargs):
        raise RuntimeError(
            "EEPROM limit updates require explicit maintenance; normal calibration is software-only"
        )

    def _split_into_byte_chunks(self, value, length):
        return list(value.to_bytes(length, "big"))

    def write_verified(self, register, motor, value):
        name = motor if isinstance(motor, str) else self._id_to_name(motor)
        address, size = self.model_ctrl_table[self.motors[name].model][register]
        self.port_handler.ser.reset_input_buffer()
        result = self.packet_handler.writeTxOnly(
            self.port_handler,
            self.motors[name].id,
            address,
            size,
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
            except Exception as e:
                errors.append(f"{name}: {e}")
        if errors:
            raise RuntimeError("Torque release failed: " + "; ".join(errors))

    def _disable_torque(self, motor, model, num_retry=0):
        self.write_verified("Torque_Enable", motor, 0)

    def enable_torque(self, motors=None, num_retry=0):
        names = self._get_motors_list(motors)
        try:
            for name in names:
                self.write_verified("Torque_Enable", name, 1)
        except BaseException as error:
            try:
                self.disable_torque(names)
            except Exception as cleanup_error:
                raise RuntimeError(
                    f"Enable failed: {error}; release failed: {cleanup_error}"
                ) from error
            raise
