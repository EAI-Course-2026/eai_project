"""飞特 SCS 基础协议；只使用 pyserial，不依赖 lerobot。"""
import time
import serial

class ProtocolError(RuntimeError):
    pass


def command(servo_id, instruction, parameters):
    body = bytes([servo_id, len(parameters) + 2, instruction, *parameters])
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def parse_reply(raw, servo_id, count):
    if not raw:
        raise TimeoutError(f"ID {servo_id} 未回复")
    if (len(raw) != count + 6 or raw[:2] != b"\xff\xff"
            or raw[2] != servo_id or raw[3] != count + 2
            or sum(raw[2:]) & 255 != 255):
        raise ProtocolError(f"ID {servo_id} 回复无效：{raw.hex(' ')}")
    if raw[4]:
        raise ProtocolError(f"ID {servo_id} 故障状态：0x{raw[4]:02X}")
    return raw[5:-1]


class Bus:
    def __init__(self, ser):
        self.ser = ser

    def request(self, sid, instruction, params, count):
        self.ser.reset_input_buffer()
        self.ser.write(command(sid, instruction, params))
        return parse_reply(self.ser.read(count + 6), sid, count)

    def read(self, sid, address):
        return self.request(sid, 2, [address, 1], 1)[0]

    def write(self, sid, address, value):
        self.ser.reset_input_buffer()
        self.ser.write(command(sid, 3, [address, value]))
        # 写入应答可能关闭；改号应答也可能使用旧 ID。以独立回读为准。
        self.ser.read(6)
        time.sleep(0.03)

    def verified_write(self, sid, address, value):
        self.write(sid, address, value)
        got = self.read(sid, address)
        if got != value:
            raise ProtocolError(f"ID {sid} 地址 {address} 回读 {got}，预期 {value}")

    def identity(self, sid):
        self.request(sid, 1, [], 0)
        got = self.read(sid, 5)
        if got != sid:
            raise ProtocolError(f"ID {sid} 的编号寄存器实际为 {got}")
        return got
