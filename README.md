# 具身智能机械臂小车课程项目

目标：使用 LeRobot 和现有硬件，完成舵机实验、机械臂装配与遥操作、底盘整合，最终完成可复现的具身任务。

当前阶段：舵机基础实验，用户已确认逐台完成 6 台编号。尚未确认机械臂、小车或完整任务验收。LeRobot 以固定版本依赖使用，本仓库不是 LeRobot 源码克隆。参考源码继续保留在各自仓库中，不修改 `.venv/site-packages`。

## 环境

Python 3.12，使用 uv 管理：

```sh
uv sync --locked --extra experiments
.venv/bin/python scripts/check_env.py
```

`experiments` 包含 matplotlib；训练时按需加 `--extra training`。所有命令从项目根目录运行；脚本输出路径不依赖终端所在位置。

## 硬件配置

复制 `configs/hardware.example.toml` 为 `configs/hardware.local.toml`，填写本机实际参数。本机整理时已创建 local 文件并保留旧脚本值。

- `serial`：端口、通信速率。
- `single`：单舵机 ID、运动位置序列。
- `numbering`：编号的旧 ID、新 ID；逐台编号只需改 `new_id`。
- `pid`：起点、角度换算、PWM 方向。
- `pair`：两台舵机的 ID、软件范围、方向。

local 文件不进入 Git。当前保存的编号和范围是操作默认值，不是验收结果，也不是已写入舵机的硬限位。具体实物信息见 [硬件登记](docs/hardware.md)。

## 常用入口

```sh
# 查询电脑的串口
.venv/bin/python -m serial.tools.list_ports -v

# 只读扫描 ID 1～8；未知编号加 --all
.venv/bin/python scripts/servo_setup.py --scan

# 逐台编号：总线上只连接当前一台
.venv/bin/python scripts/servo_setup.py --old-id 1 --new-id 6

# 只读位置查询
.venv/bin/python experiments/servos/read_position.py --id 1

# 实物单舵机、多位置运动
.venv/bin/python experiments/servos/move_positions.py

# 实物双舵机控制，输入两个 0～1 数字
.venv/bin/python experiments/servos/two_servo_control.py

# PID 离线示例（不接硬件）
.venv/bin/python experiments/servos/pid_response.py --simulate --compare

# PID 实物实验（会切换 PWM 模式）
.venv/bin/python experiments/servos/pid_response.py --compare

# 离线测试，不打开串口
.venv/bin/python -m unittest discover -s tests -v
```

编号工具不带参数运行时，按 `numbering` 配置执行改号；它不是扫描默认入口。所有脚本的 `--help` 均不启动硬件。

## 目录

- `experiments/servos/`：原理学习和课堂实验。
- `scripts/`：环境检查、编号等日常操作工具。
- `src/eai_robot/`：目前只抽取配置读取、SCS 协议和归一化映射；机械臂、底盘接口等做到对应阶段再增加。
- `configs/`：硬件配置示例和本机配置。
- `tests/`：协议和控制映射的离线验证。
- `docs/`：硬件登记、实验原理、迁移说明与阶段验收。
- `outputs/`：图表、日志；原有 PID 实验数据保持原位。
- `data/`、`calibration/`、`checkpoints/`：运行时按需创建，默认不提交大文件或本机数据。校准结果需随数据集/实验在别处备份，不能把 Git 忽略当作备份。

## 实验与阶段

- [舵机实验及原理](docs/experiments/servo_basics.md)
- [PID 操作、风险与读图](docs/experiments/pid_response.md)
- [阶段目标与验收](docs/milestones.md)
- [旧脚本名称对照](docs/migration.md)
- [原环境检查记录](docs/environment.md)

运动实验前确认供电、接线、编号、行程及固定情况。PID 的 PWM 模式下断开 USB 不保证停机；需要能切断舵机电源。终端中的“已发送停止”也不等于已确认实物停止。
