"""只读查询舵机位置：python experiments/servos/read_position.py --id 1"""
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import serial
from eai_robot.config import load_config
from eai_robot.hardware.scs import Bus


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="查询 SCS215 位置，不发送运动命令")
    parser.add_argument('--port', default=cfg['serial']['port'])
    parser.add_argument('--id', type=int, default=cfg['single']['id'])
    args = parser.parse_args()
    if not 0 <= args.id <= 253:
        parser.error('ID 必须为 0..253')
    with serial.Serial(args.port, cfg['serial']['baudrate'], timeout=.2) as ser:
        bus = Bus(ser)
        for i in range(3):
            data = bus.request(args.id, 2, [56, 2], 2)
            print(f"第 {i+1} 次，ID {args.id} 当前位置：{int.from_bytes(data, 'big')} 步")
            time.sleep(.3)


if __name__ == '__main__':
    main()
