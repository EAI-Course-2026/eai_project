"""SCS215 adapter for pinned LeRobot 0.6.1, without editing installed packages.

Use LeRobot's registered scs_series protocol/table and override its model number
locally. SCS215 has no STS Homing_Offset or Operating_Mode register. Calibration
is software-only; normal control must never unlock EEPROM.
"""
import time
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from eai_robot.arm.calibration import JOINTS, MODEL_NUMBER


class SCS215MotorsBus(FeetechMotorsBus):
    model_number_table = {**FeetechMotorsBus.model_number_table, "scs_series": MODEL_NUMBER}

    def __init__(self, port):
        motors = {name: Motor(sid, "scs_series", MotorNormMode.RANGE_0_100)
                  for sid, name in enumerate(JOINTS, 1)}
        super().__init__(port, motors, protocol_version=1)

    def _split_into_byte_chunks(self, value, length):
        # SDK endianness is a process-global variable. Keep sync writes explicit.
        return list(value.to_bytes(length, "big"))

    def write_verified(self, register, motor, value):
        address, size = self.model_ctrl_table["scs_series"][register]
        self.port_handler.ser.reset_input_buffer()
        result = self.packet_handler.writeTxOnly(
            self.port_handler, self.motors[motor].id, address, size,
            self._serialize_data(value, size),
        )
        if result != self._comm_success:
            raise ConnectionError(self.packet_handler.getTxRxResult(result))
        # Acknowledgements may be disabled. Independently read the register.
        self.port_handler.ser.flush()
        time.sleep(0.02)
        # SDK clearPort() only flushes outgoing bytes. It does NOT discard the
        # six-byte write ACK; otherwise read2ByteTxRx mistakes it for a read
        # response and indexes an empty payload. Drop ACKs before the read.
        self.port_handler.ser.reset_input_buffer()
        if self.read(register, motor, normalize=False) != value:
            raise RuntimeError(f"{motor} {register} 写入回读不一致")

    def disable_torque(self, motors=None, num_retry=0):
        for motor in self._get_motors_list(motors):
            self.write_verified("Torque_Enable", motor, 0)

    def enable_torque(self, motors=None, num_retry=0):
        for motor in self._get_motors_list(motors):
            self.write_verified("Torque_Enable", motor, 1)

    def _disable_torque(self, motor, model, num_retry=0):
        self.write_verified("Torque_Enable", self._id_to_name(motor), 0)
