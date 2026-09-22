"""SCS215 编号与只读 ID 查询，不依赖 lerobot。

python scripts/servo_setup.py --scan             # 快查 1..8
python scripts/servo_setup.py --scan --all       # 扫描全部 0..253
python scripts/servo_setup.py                   # 按 configs/hardware.local.toml 的 numbering 改号
python scripts/servo_setup.py --old-id 1 --new-id 4
改号时总线上只能连接当前这一台。扫描无法保证发现重复 ID。
"""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from eai_robot.config import load_config

import argparse
import time
import serial

from eai_robot.hardware.scs import Bus, ProtocolError


def scan(bus, ids):
    found, invalid = [], []
    for sid in ids:
        try:
            bus.identity(sid)
            found.append(sid)
            print(f"在线 ID：{sid}（PING、编号回读及校验通过）")
        except TimeoutError:
            pass
        except ProtocolError as exc:
            invalid.append(str(exc))
    print(f"确认在线的 ID：{found or '无'}")
    if invalid:
        print(f"另有 {len(invalid)} 个地址出现异常回复，不能认定为在线：")
        for message in invalid[:5]:
            print('  ' + message)
        if len(invalid) > 5:
            print('  ……其余异常省略')
    print("扫描结果表示有效地址，不等于物理台数；重复 ID 需要逐台单独检查。")
    return 0 if found and not invalid else 1


def rename(bus, old, new):
    print("改号前提：总线上仅连接当前要编号的一台舵机。")
    bus.identity(old)
    if old == new:
        print(f"已确认当前 ID 就是 {new}，无需写入。")
        return
    # 目标 ID 无回复才允许继续；异常回复也阻止写入。
    try:
        bus.request(new, 1, [], 0)
    except TimeoutError:
        pass
    else:
        raise RuntimeError(f"目标 ID {new} 已有设备回复，请单独连接待编号舵机")
    bus.verified_write(old, 40, 0)  # 关闭扭矩，回读确认。
    try:
        bus.verified_write(old, 48, 0)  # 解锁 EEPROM，回读确认。
        bus.write(old, 5, new)  # 此时锁关闭，ID 写入掉电保存区。
        bus.identity(new)
    finally:
        # 写入失败/中断时也尝试重新锁定；分别查询新旧地址，不盲写未知设备。
        locked = False
        for sid in (new, old):
            try:
                bus.identity(sid)
                bus.verified_write(sid, 48, 1)
                locked = True
                break
            except (TimeoutError, ProtocolError, serial.SerialException):
                continue
        if not locked:
            raise RuntimeError("无法确认 EEPROM 已重新锁定；停止继续编号，先恢复通信并查询当前 ID")
    bus.identity(new)
    print(f"成功：ID {old} → {new}；新 ID 在线、编号寄存器和 EEPROM 锁均已回读确认。")
    print("断电保存最终验证：重新上电后运行 python scripts/servo_setup.py --scan。")


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="SCS215 编号 / 查询当前在线 ID")
    parser.add_argument('--port', default=cfg['serial']['port'])
    parser.add_argument('--scan', action='store_true', help='只读扫描 ID 1..8，不改号、不运动')
    parser.add_argument('--all', action='store_true', help='与 --scan 一起扫描全部 ID 0..253')
    parser.add_argument('--old-id', type=int, default=cfg['numbering']['old_id'])
    parser.add_argument('--new-id', type=int, default=cfg['numbering']['new_id'])
    args = parser.parse_args()
    if not (0 <= args.old_id <= 253 and 0 <= args.new_id <= 253):
        parser.error('ID 必须是 0..253；254 是广播地址，不能用作设备 ID')
    if args.all and not args.scan:
        parser.error('--all 只能与 --scan 一起使用')
    try:
        with serial.Serial(args.port, cfg['serial']['baudrate'], timeout=0.1, write_timeout=0.2) as ser:
            bus = Bus(ser)
            if args.scan:
                return scan(bus, range(254) if args.all else range(1, 9))
            rename(bus, args.old_id, args.new_id)
    except (Exception, KeyboardInterrupt) as exc:
        print(f"未完成：{type(exc).__name__}: {exc}")
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
