"""One-time, readback-verified SCS215 ID3 EEPROM upper-limit adjustment."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from eai_robot.config import load_config
from eai_robot.arm.backends import SerialBackend
from eai_robot.arm.controller import inspect_hardware


def main(argv=None):
    config=load_config()
    parser=argparse.ArgumentParser(description='仅将本机 ID3 EEPROM 位置上限 769 改为 771')
    parser.add_argument('--port',default=config['serial']['port'])
    parser.add_argument('--baudrate',type=int,default=config['serial']['baudrate'])
    parser.add_argument('--enable',action='store_true',help='实际写入 EEPROM；默认只读')
    args=parser.parse_args(argv)
    backend=SerialBackend(args.port,args.baudrate)
    try:
        limits=inspect_hardware(backend)
        torque={sid:backend.read(sid,'Torque_Enable') for sid in range(1,7)}
        status={sid:backend.read(sid,'Status') for sid in range(1,7)}
        position={sid:backend.read(sid,'Present_Position') for sid in range(1,7)}
        locked=backend.bus.read(3,48)
        print('写入前',{'limits':limits,'torque':torque,'status':status,
                         'position':position,'id3_eeprom_lock':locked},flush=True)
        if any(torque.values()) or any(status.values()) or locked!=1:
            raise RuntimeError('所有扭矩和状态必须为 0，ID3 EEPROM 必须处于锁定状态')
        if limits[3] not in ((196,769),(196,771)):
            raise RuntimeError(f'ID3 EEPROM 当前限位 {limits[3]} 与本机预期不符')
        if not args.enable:
            print('只读预检完成；加 --enable 才会写入。',flush=True)
            return 0
        if not 300<=position[3]<=700:
            raise RuntimeError('先将 ID3 支撑并手动放到 300..700，再修改 EEPROM 限位')
        if limits[3]==(196,769):
            try:
                backend.bus.verified_write(3,48,0)
                backend.bus.verified_word(3,11,771)
            finally:
                backend.bus.verified_write(3,48,1)
        after=inspect_hardware(backend)
        lock_after=backend.bus.read(3,48)
        torque_after={sid:backend.read(sid,'Torque_Enable') for sid in range(1,7)}
        print('写入后回读',{'limits':after,'id3_eeprom_lock':lock_after,
                          'torque':torque_after},flush=True)
        if after[3]!=(196,771) or lock_after!=1 or any(torque_after.values()):
            raise RuntimeError('ID3 上限／EEPROM 锁／扭矩回读未通过')
        if any(after[sid]!=limits[sid] for sid in (1,2,4,5,6)):
            raise RuntimeError('其他舵机 EEPROM 限位意外改变')
        return 0
    finally:
        backend.close()


if __name__=='__main__':
    raise SystemExit(main())
