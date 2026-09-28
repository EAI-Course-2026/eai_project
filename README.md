# SO101 六关节 SCS215 控制

本仓库为 SO101 结构的六关节 SCS215 机械臂提供编号、软件校准、六个 0–1 位置输入、同步目标发送和本机网页上位机。支持纯 pyserial 通信，也支持通过项目内适配器使用 LeRobot 0.6.1 的 Feetech 电机总线。项目长期目标是将成熟的 SCS215 支持整理为可贡献给 LeRobot 社区的实现。

目前**已验证的是电机通信与位置控制**：六台型号和 ID 回读、两种后端的小幅六关节同步到位，以及网页发命令后的实物运动。完整机械端点、额定负载、任意起点避碰、摄像头、LeRobot 数据采集和训练尚未验收。`src/eai_robot/hardware/lerobot_scs215.py` 是电机总线适配器；它不是已注册的 LeRobot `Robot` 类型。具体边界见[架构与 LeRobot 接入状态](docs/architecture.md)。

## 快速开始

需要 Python 3.12 和 uv。在项目根目录执行：

```sh
uv sync --locked --extra experiments
uv run --no-sync python -m unittest discover -s tests -q
uv run --no-sync python scripts/arm_serial.py control --allow-wide-range --dry-run --values 0.5 0.5 0.5 0.5 0.5 0.5
```

复制 `configs/hardware.example.toml` 为被 Git 忽略的 `configs/hardware.local.toml`，填写本机串口和波特率；也可通过 `--port` 指定。Windows 使用本机识别到的 `COM` 端口名。确认六台设备、机械臂支撑与周围空间后，先只读检查：

```sh
uv run --no-sync python scripts/arm_serial.py inspect
uv run --no-sync python scripts/arm_lerobot.py inspect
```

网页上位机入口：

```sh
uv run --no-sync python scripts/arm_desk.py
```

界面可选纯串口或 LeRobot 电机总线后端；连接时只读，归位成功后才开放六个滑杆目标发送。它也提供固定起点演示、停止释放扭矩和校准采集。实物控制有边界与姿态要求，首次使用请阅读[六关节控制说明](docs/experiments/scs215_arm.md)和[上位机操作](docs/experiments/scs215_desk.md)。

## 代码与数据边界

| 位置 | 内容 |
|---|---|
| `src/eai_robot/arm/` | 六值映射、校准、运动检查、演示与网页服务 |
| `src/eai_robot/hardware/` | 原生 SCS 协议及 LeRobot SCS215 电机总线适配器 |
| `scripts/` | 编号、双后端控制、上位机和候选范围验收入口 |
| `experiments/servos/` | 单舵机、双舵机及 PID 原理实验 |
| `calibration/`、`configs/` | 本台机械臂的软件校准快照、演示起点和配置示例 |
| `tests/` | 不驱动实物的协议、控制和网页服务测试 |

`calibration/scs215_so101.json` 是当前演示使用的 EEPROM 限位导出快照，人工重测全部端点尚未完成；日期命名的安全范围是**未启用候选**。硬件限位、软件 0–1 范围、演示入口和到位容差各有不同作用，详见[硬件登记](docs/hardware.md)和[候选范围记录](docs/experiments/scs215_safe_candidate.md)。

完整课程作业提交目录 `assignment2_handin/`、视频、打包脚本、本机配置和运行日志只保留本地，不进入 Git。仓库使用相对源码路径解析，不包含本机用户目录或写死的串口；Windows 路径写法已审查，尚未在 Windows 实机连接舵机验证。

后续工作依次是：完成独立的 LeRobot `Robot` 接口与安全校准适配、接入摄像头与操作者、验证 episode 数据，再测试训练与评估。当前版本**不能直接用于 `lerobot-record`**。见[阶段目标](docs/milestones.md)；分享上游前还需确定开源许可证并完成相应硬件验证。
