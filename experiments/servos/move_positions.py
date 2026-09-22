"""依次到达配置中的位置；导入或 --help 不启动硬件。"""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import serial
from eai_robot.config import load_config
from eai_robot.hardware.scs import Bus, command


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="SCS215 多位置实验")
    parser.add_argument('--port', default=cfg['serial']['port'])
    parser.add_argument('--id', type=int, default=cfg['single']['id'])
    args = parser.parse_args()
    positions = cfg['single']['positions']
    if not 0 <= args.id <= 253 or not all(isinstance(x, int) and 0 <= x <= 1023 for x in positions):
        parser.error('ID 或位置配置超出 SCS215 范围')
    with serial.Serial(args.port, cfg['serial']['baudrate'], timeout=.2, write_timeout=.2) as ser:
        bus = Bus(ser)
        bus.identity(args.id)
        try:
            bus.verified_write(args.id, 40, 1)
            for position in positions:
                bus.request(args.id, 3, [42, position >> 8, position & 255], 0)
                print(f"已发送目标位置：{position}")
                time.sleep(cfg['single']['pause_seconds'])
        finally:
            ser.write(command(args.id, 3, [40, 0]))
            print('已发送关闭扭矩指令')


if __name__ == '__main__':
    main()
