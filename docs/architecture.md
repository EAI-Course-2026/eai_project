# 架构与 LeRobot 接入状态

本项目把六关节运动逻辑与串口实现分开。两种后端共用同一份软件校准和 `ArmController`；切换后端只改变通信层，不改变 0–1 输入含义或运动门限。

```text
Python CLI / 本机网页上位机
            │ 六个 0–1 目标
            ▼
ArmController：校准映射、目标检查、六关节同步写、反馈/停止
            │
            ├── SerialBackend → 项目内 SCS 协议 → 串口
            └── LeRobotBackend → SCS215MotorsBus → LeRobot FeetechMotorsBus → 串口
```

`SCS215MotorsBus` 在项目内为六台 SCS215 指定型号 1315、`scs_series`、协议 1 和大端字节序，并处理写应答残留。LeRobot 后端读写原始刻度时使用 `normalize=False`；本项目的 0–1 映射由 `ArmController` 完成，不能与 LeRobot 的 0–100 归一化数值混用。普通位置控制不解锁或改写 EEPROM。此实现适配锁定的 `lerobot[core_scripts,feetech]==0.6.1`，不修改安装目录中的 LeRobot 代码。

## 已实现与未接入

| 层次 | 当前状态 |
|---|---|
| 原生 SCS215 / LeRobot 电机总线通信 | 两版均有六关节小幅同步到位实测；协议与帧行为有离线测试 |
| 软件校准、归位、GUI、停止 | 项目内实现，使用同一个 `ArmController`；目标范围仍受已记录的安全边界限制 |
| LeRobot `RobotConfig` / `Robot` 与 CLI 类型注册 | 未接通；现有 GUI 和命令行不依赖 LeRobot 的机器人配置系统 |
| 摄像头、遥操作输入、episode 数据集 | 尚未实现端到端同步采集 |
| `lerobot-record`、训练、评估 | 尚未用于这台 SCS215 机械臂 |

这里的“LeRobot 版”指**使用 LeRobot 电机总线的控制版**，不表示该机械臂已能由 `lerobot-record` 创建。仓库不提供示例 YAML 冒充可运行的 CLI 配置。锁定版本的录制入口以 `RobotConfig`、`TeleoperatorConfig` 和数据集配置构建对象；仅有舵机总线适配器不足以开始采集。

## 面向 LeRobot 社区的下一步

1. 基于 SCS215 寄存器表实现可独立测试的 `Robot` 类和配置注册；连接、校准、停止流程不得沿用 STS3215 的 `Operating_Mode`、`Homing_Offset` 或 4095 刻度假设。
2. 让 `get_observation` 与 `send_action` 使用锁定版本的真实接口，保持六关节安全范围、反馈检查和异常释放扭矩；先通过离线测试，再逐步实物验证。
3. 接入摄像头和明确的操作者输入，校验观测与动作时间戳，再录制少量本地 episode；确认格式和回放后才扩大采集。
4. 完成跨系统测试、协议依据、验证记录与许可证选择，再考虑向 LeRobot 上游提交适配器。

本台机械臂的硬件观察和未启用候选范围见[硬件登记](hardware.md)与[候选范围记录](experiments/scs215_safe_candidate.md)。课程作业提交包、视频与本机串口配置只保存在本地，不进入仓库。
