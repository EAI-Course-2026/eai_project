"""两个 0..1 输入映射到各自位置区间；依次发送，不宣称严格同步。"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
import serial
from eai_robot.config import load_config
from eai_robot.hardware.scs import Bus, command
from eai_robot.control.mapping import ratio_to_position


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="双舵机 0～1 比例控制")
    parser.add_argument('--port', default=cfg['serial']['port'])
    args = parser.parse_args()
    motors = [cfg['pair']['first'], cfg['pair']['second']]
    if len({m['id'] for m in motors}) != 2 or any(not 0 <= m['id'] <= 253 for m in motors):
        parser.error('两台舵机必须使用不同的有效 ID')
    for motor in motors:
        ratio_to_position(0, motor['min_step'], motor['max_step'], motor['direction'])
    with serial.Serial(args.port, cfg['serial']['baudrate'], timeout=.2, write_timeout=.2) as ser:
        bus = Bus(ser)
        for motor in motors:
            bus.identity(motor['id'])
        try:
            # 不在启动时直接开扭矩，避免旧目标位置导致意外运动。
            while True:
                line = input('输入两个 0～1 数字，如 0.5 0.5；q 退出：').strip()
                if line.lower() == 'q':
                    break
                try:
                    first, second = map(float, line.split())
                    positions = [ratio_to_position(r, m['min_step'], m['max_step'], m['direction'])
                                 for r, m in zip((first, second), motors)]
                except ValueError as exc:
                    print(f'输入无效：{exc}')
                    continue
                for motor, position in zip(motors, positions):
                    bus.request(motor['id'], 3, [42, position >> 8, position & 255], 0)
                    bus.verified_write(motor['id'], 40, 1)
                print('目标位置：' + ', '.join(f"ID {m['id']}={p} 步" for m, p in zip(motors, positions)))
        except (KeyboardInterrupt, EOFError):
            print('\n退出')
        finally:
            # 一台停止失败也继续尝试另一台；不把发送指令说成已确认停机。
            for motor in motors:
                try:
                    ser.write(command(motor['id'], 3, [40, 0]))
                    print(f"已发送 ID {motor['id']} 关闭扭矩指令")
                except serial.SerialException as exc:
                    print(f"ID {motor['id']} 停止指令发送失败：{exc}；请断开电源")


if __name__ == '__main__':
    main()
