"""Raw SCS215 registers; importing this module does not import LeRobot."""
import serial
from eai_robot.hardware.scs import Bus
from .calibration import JOINTS

REGISTERS = {
    "Model_Number": (3, 2), "ID": (5, 1),
    "Min_Position_Limit": (9, 2), "Max_Position_Limit": (11, 2),
    "Torque_Enable": (40, 1), "Goal_Position": (42, 2),
    "Running_Time": (44, 2), "Goal_Velocity": (46, 2),
    "Present_Position": (56, 2),
    "Present_Voltage": (62, 1), "Present_Temperature": (63, 1), "Status": (65, 1),
}


class SerialBackend:
    """Only pyserial, including the six-servo broadcast packet."""
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate, timeout=0.1, write_timeout=0.2)
        self.bus = Bus(self.ser)

    def read(self, sid, register):
        address, size = REGISTERS[register]
        return self.bus.read(sid, address) if size == 1 else self.bus.read_word(sid, address)

    def write(self, sid, register, value):
        address, size = REGISTERS[register]
        if size == 1:
            self.bus.verified_write(sid, address, value)
        else:
            self.bus.verified_word(sid, address, value)

    def sync_positions(self, targets):
        self.bus.sync_positions(targets)

    def close(self):
        self.ser.close()


class LeRobotBackend:
    """Uses the project-local SCS215 FeetechMotorsBus adapter."""
    def __init__(self, port, baudrate):
        from eai_robot.hardware.lerobot_scs215 import SCS215MotorsBus
        self.bus = SCS215MotorsBus(port, baudrate=baudrate)
        try:
            # The shared bus sets baudrate before read-only identity checks.
            self.bus.connect(handshake=False)
        except BaseException:
            if self.bus.is_connected:
                self.bus.port_handler.closePort()
            raise

    def read(self, sid, register):
        return int(self.bus.read(register, JOINTS[sid - 1], normalize=False))

    def write(self, sid, register, value):
        self.bus.write_verified(register, JOINTS[sid - 1], value)

    def sync_positions(self, targets):
        self.bus.sync_write("Goal_Position", {JOINTS[sid - 1]: value for sid, value in targets.items()},
                            normalize=False)

    def close(self):
        # Read-only inspection must not disable torque or unlock EEPROM on exit.
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)
