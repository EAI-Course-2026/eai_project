> 历史实验记录：本文数值对应 2026-09-27/28。当时的腕部 4..1014 和旧候选 25..994 已不是当前基线。2026-10-02 当前 EEPROM 为 47..974；新配置与验收边界见 [共享基线](../course/README.md)。

# 作业 2：SCS215 六关节编号、校准和同步控制

2026-09-27 交付第一版。在现有充电宝约 5V 供电下，pyserial 与 LeRobot 两版均完成了一轮六关节约 20 刻度的实物同步控制验证：六个 0..1 输入映射为目标，同步发送，目标寄存器回读正确，位置反馈均进入 10 刻度容差，退出后六台扭矩为 0。另有两轮到位超时，已保留原始记录，不将一次通过推广为全行程或任意负载验收。

## 入口与依赖

- `scripts/servo_setup.py`：编号 / 只读扫描，默认只扫 ID 1..6。
- `scripts/arm_serial.py`：六关节校准和同步控制，仅依赖 pyserial，不导入 LeRobot 或 servo SDK。
- `scripts/arm_lerobot.py`：同一功能，通信使用项目内的 LeRobot SCS215 适配器。

从项目根目录运行。已有环境使用 `uv sync --locked`。单独运行 pyserial 版只需 Python 3.12 和 `pyserial>=3.5,<4`；LeRobot 版使用固定 `lerobot[core_scripts,feetech]==0.6.1`。

默认串口、波特率来自 `configs/hardware.local.toml`；没有本机配置则读取 example。全局参数 `--port`、`--baudrate`、`--calibration` 放在子命令之前。以下命令中的 `SERIAL_PORT` 是占位符，运行前替换成当前电脑识别的串口名（Windows 如 `COM5`）；也可以在被 Git 忽略的 `configs/hardware.local.toml` 中设置默认串口。

## 1. 编号与读取

| ID | 输入顺序 / 程序约定关节 | 已存硬件限位 |
|---|---|---|
| 1 | shoulder_pan（底座） | 183..897 |
| 2 | shoulder_lift（肩） | 64..737 |
| 3 | elbow_flex（肘） | 196..769 |
| 4 | wrist_flex（腕俯仰） | 70..681 |
| 5 | wrist_roll（腕旋转） | 4..1014 |
| 6 | gripper（夹爪） | 237..644 |

六台型号寄存器均为 1315（SCS215），编号寄存器为 1..6。用户已逐台编号，本轮未重新改号。

```sh
uv run --no-sync python scripts/servo_setup.py --port SERIAL_PORT --scan
uv run --no-sync python scripts/arm_serial.py --port SERIAL_PORT inspect
uv run --no-sync python scripts/arm_lerobot.py --port SERIAL_PORT inspect
```

`inspect` 只读型号、ID、限位、位置、扭矩、电压、温度和状态，退出不写扭矩。位置模式要求硬件限位是 `0 <= min < max <= 1023`，不自动改 EEPROM 切换历史 PWM 实验的模式。

需要重新编号时，总线上只接当前这一台，例如：

```sh
uv run --no-sync python scripts/servo_setup.py --port SERIAL_PORT --old-id 1 --joint gripper
# 等价于 --old-id 1 --new-id 6
```

编号检查型号、目标地址占用；关闭扭矩、解锁地址 48、写 ID 地址 5、回读并重新锁定。改号后应断电重启再次扫描。扫描只能确认地址有效，不能排除同 ID 的多个实物。`servo_setup.py` 无参数运行仍按 numbering 配置执行改号，日常查询务必明确用 `--scan`。

## 2. 校准（calibration）

两版共用 `calibration/scs215_so101.json`，每关节记录 `id`、`drive_mode`、`homing_offset: 0`、`range_min`、`range_max`。旁边的 `.meta.json` 记录校准来源。

### 今天版本：导出已有硬件限位

六台 EEPROM 已存有上表中的范围，与原参考脚本的范围一致。本轮直接读取并导出，不写舵机：

```sh
uv run --no-sync python scripts/arm_serial.py --port SERIAL_PORT calibrate --from-hardware
```

提交包已附本台机械臂的导出结果；已存在时拒绝覆盖，需要重新导出才加 `--overwrite`。默认 0 对应较小原始刻度，1 对应较大刻度；需要反向的关节可在导出时加 `--reverse shoulder_lift`，可重复指定。

**来源边界**：这些是已有硬件校准参数的导出，不是今天重新测量全部机械端点；meta 中 `method=existing_hardware_limits`、`manual_endpoints_verified=false`。本轮验证覆盖附近小幅组合目标，不代表全部端点已验证。

### 完整手动校准

支撑机械臂并保持舵机供电，运行：

```sh
uv run --no-sync python scripts/arm_serial.py --port SERIAL_PORT calibrate --overwrite
```

输入 `RELEASE` 确认已支撑后，程序关闭六台扭矩并回读。按 ID 1..6，手动移至希望输入 0 和 1 对应的安全端点，分别回车采集。自动识别反向行程，保存软件范围，不写 EEPROM 或 STS 的 Homing_Offset。两端至少相差 20 刻度，不要强推机械止挡。

大于等于 900 刻度的行程需人工确认反馈连续、不跨 0/1023 边界，再加 `--allow-wide-range`。本机腕旋转为 4..1014，控制时同样需要该参数；它是明确允许使用宽行程，不是宣布全行程已通过实物验证。采集失败或中断不会覆盖原文件。

## 3. 六个 0..1 的同步控制

固定输入顺序：

```text
shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper
```

正向关节映射为 `round(min + ratio * (max - min))`，反向关节先用 `1 - ratio`。输入必须恰好六个有限数，范围 0..1；支持空格或逗号分隔。`--margin-counts N` 内缩两端后再映射。

离线看映射，不打开串口：

```sh
uv run --no-sync python scripts/arm_serial.py control --allow-wide-range --dry-run --values 0.5 0.5 0.5 0.5 0.5 0.5
```

实际控制前支撑并确认无遮挡；先从当前位置附近的小幅目标开始，不能将六个中点自动视为安全姿态。

```sh
# 只读当前姿态与映射，不开扭矩
uv run --no-sync python scripts/arm_serial.py --port SERIAL_PORT control --allow-wide-range

# 推荐提交演示入口：交互输入六个 0..1
uv run --no-sync python scripts/arm_lerobot.py --port SERIAL_PORT control --allow-wide-range --enable

# 无 LeRobot 版同样可用
uv run --no-sync python scripts/arm_serial.py --port SERIAL_PORT control --allow-wide-range --enable
```

交互命令：六个数发送目标；`show` 查询；`q` / Ctrl-C 退出并关闭扭矩。省略交互输入、发送一次目标用 `--values` 加六个数。一次目标到位后同样关闭扭矩，不保持姿态，需继续支撑机械臂。

默认 `--motion direct`：一条 SCS 广播 Sync Write 将六个最终目标一起发送，与可运行的 `move_positions.py` 一样直接设置位置目标，并沿用舵机当前速度寄存器。没有默认覆盖速度为 100。需要指定速度才加 `--velocity 100`（范围 1..1000，0 的最大速度含义不作为显式参数开放）。

可选 `--motion smooth`：默认单步最多 5 刻度，间隔 0.05 秒，每步仍用一条六关节同步帧；可调 `--max-step`、`--interval`。SCS protocol 1 不支持 Sync Read，所以反馈逐台读取。

启用前检查 ID/型号、位置模式、目标范围、扭矩状态；预装当前姿态的目标后再启用扭矩。允许端点附近最多 3 刻度的实际反馈误差，但初始保持目标会钳制到有效区间，**不会扩大可命令的范围**。这处理了实物 ID 2 / ID 3 在关扭矩、受重力影响时略超过硬限位的情况。

最终到位默认容差 10 刻度（约 2.93° 名义舵机刻度），等待 5 秒，可用 `--tolerance`、`--timeout` 调整。程序回读目标寄存器和位置，超时报告失败并关闭扭矩，不把“发送成功”当作“到位”。逐台尝试关闭，一台异常不阻止其他台。通信中断导致无法确认关闭时，需要切断舵机电源。

## 4. 实测结果与限制

两版正式入口均使用六个归一化数值，目标相对当轮反馈各 20 刻度，目标始终在已有硬件限位内。LeRobot 一轮到位通过（10 刻度容差，5 秒等待）；pyserial 腕俯仰反向的一轮到位通过（相同容差，10 秒等待）。所有回合退出后均独立读取确认六台扭矩为 0。

pyserial 最初一轮肘关节误差 14 刻度、随后一轮腕关节误差 12 刻度，分别触发 5 秒和 10 秒超时。这些失败记录也在提交包中；不能据此宣称双版本在任意姿态下都能稳定到位。通过回合的详细目标、反馈与误差保存在本地作业说明和实验日志中；`assignment2_handin/` 不纳入 Git。

当前供电为另接充电宝小电流模式，反馈约 4.9–5.0V，温度约 18–20°C，状态无报警。它足以完成本次通信与小幅演示，不意味着完整额定负载能力已经验证。飞特官方建议 SCS215 使用 7–8.4V；今后有条件应核对合适电源与负载。本轮不把更换电源作为交付第一版的条件，也未修改 PID、EEPROM 限位或编号。

## 5. LeRobot 适配与调试修复

原 SO101 默认 STS3215，本项目使用 SCS215：protocol 1、大端双字节、1024 刻度、型号 1315、Lock 地址 48；不使用 STS 的 Homing_Offset 和 Operating_Mode。

项目子类 `SCS215MotorsBus` 复用已登记的 `scs_series` 控制表，在子类补充型号，并明确大端序列化；未改全局型号表或安装包。LeRobot 版使用真实 `FeetechMotorsBus.read`、`sync_write`，写位置时 `normalize=False`，两版共用 0..1 映射，避免误用 LeRobot 的 0..100 输入。扭矩只写 Torque_Enable，不自动解锁 EEPROM。

实物发现 SDK 的 `clearPort()` 实际只做 TX flush，写 ACK 仍留在接收缓冲区，会被后续读取误认成短回复。适配器现在在写入前及延迟后的独立回读前清 RX，实测回读正常，并有专门回归测试。

参考：[LeRobot 控制表](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/feetech/tables.py)、[SCS215 规格](https://www.feetechrc.com/Data/feetechrc/upload/file/20200525/6372600116270632797842424.pdf)、[飞特官方教程](https://feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf)。实际接口以锁定的 LeRobot 0.6.1 源码为准。

## 6. 离线检查

```sh
uv run --no-sync python -m unittest discover -s tests -v
```

覆盖大端同步帧与校验和、两后端帧一致、六台默认扫描、有限输入验证、校准方向与来源、只读无写入、ACK 清理、速度沿用、直接/插值控制、反馈端点容差、部分启用失败、跟随异常与到位超时、关闭失败非零退出、无 LeRobot 版不导入 LeRobot/SDK。

## 7. 固定起点的多个演示

`scripts/arm_demo.py` 已改为每次先归位，再执行 `gripper`、`showcase`、`transfer`，还可用 `home` 只归位。两个后端共享同样的流程。动作目标来自保存的固定起点，手动改变初始姿态不会改变后面的演示动作。

```sh
uv run --no-sync python scripts/arm_demo.py --demo gripper --enable
uv run --no-sync python scripts/arm_demo.py --backend lerobot --demo showcase --enable
```

默认只读预览；执行需要 `--enable`。归位失败不会开始演示。详细入口范围、夹爪扩大幅度、示教和实测见 [固定起点演示](scs215_demos.md)。旧版六段相对动作的原始通过记录 `demo_120.json` 保留作历史证据，不代表新版任意初始姿态验证。
