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

    def read_word(self, sid, address):
        # SCS series uses high byte first (STS3215 uses the opposite order).
        return int.from_bytes(self.request(sid, 2, [address, 2], 2), "big")

    def verified_word(self, sid, address, value):
        self.ser.reset_input_buffer()
        self.ser.write(command(sid, 3, [address, *value.to_bytes(2, "big")]))
        self.ser.read(6)
        if self.read_word(sid, address) != value:
            raise ProtocolError(f"ID {sid} 地址 {address} 写入回读不一致")

    def sync_positions(self, targets):
        parameters = [42, 2]
        for sid, position in targets.items():
            if not 1 <= sid <= 253 or not 0 <= position <= 1023:
                raise ValueError("SCS215 ID/目标位置无效")
            parameters.extend([sid, *position.to_bytes(2, "big")])
        if not targets:
            raise ValueError("同步目标不能为空")
        packet = command(254, 0x83, parameters)
        if self.ser.write(packet) != len(packet):
            raise ProtocolError("同步写入不完整")
        self.ser.flush()
