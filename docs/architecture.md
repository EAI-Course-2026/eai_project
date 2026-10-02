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

`SCS215MotorsBus` 在项目内为六台 SCS215 指定型号 1315、`scs_series`、协议 1 和大端字节序，并处理写应答残留。LeRobot 后端读写原始刻度时使用 `normalize=False`；本项目的 0–1 映射由 `ArmController` 完成，不能与 LeRobot 的归一化数值混用。普通位置控制不解锁或改写 EEPROM。当前环境固定到团队 LeRobot 0.6.2 fork 的提交 `6a077907c7989635218969ee78f5436f8faec92b`；旧 0.6.1 基线只用于迁移对照。

2026-10-02 已将协作者课程实现迁入 `src/eai_robot/course/`。课程的 `CourseFollower` 复用 fork 的 `Robot` 接口，以共享项目标定替代个人 HF 缓存，连接只读，运动显式启用。课程臂关节使用 -100..100、夹爪使用 0..100；网页和原生控制继续使用六个 0..1，迁移测试验证两种约定的 raw 刻度一致。FK/IK 默认统一使用 NumPy。详细边界和运行入口见[共享基线](course/README.md)。

## 已实现与未接入

| 层次 | 当前状态 |
|---|---|
| 原生 SCS215 / LeRobot 电机总线通信 | 两版均有六关节小幅同步到位实测；协议与帧行为有离线测试 |
| 软件校准、归位、GUI、停止 | 项目内实现，使用同一个 `ArmController`；目标范围仍受已记录的安全边界限制 |
| LeRobot `RobotConfig` / `Robot` | 课程工具已接入 fork 的 SO follower 接口；独立 SCS215 插件与上游贡献仍待实施 |
| 摄像头、遥操作输入、episode 数据集 | 尚未实现端到端同步采集 |
| `lerobot-record`、训练、评估 | 尚未用于这台 SCS215 机械臂 |

原有 `arm_lerobot.py` 仍指**使用 LeRobot 电机总线的控制版**；新迁入的课程工具使用 `CourseFollower`，不表示录制、训练和评估已经端到端验收。标准 CLI 直接创建 fork 的 SO follower 不会自动使用课程中的连接/启用保护层，正式数据采集前仍需统一入口和停止策略。

## 面向 LeRobot 社区的下一步

1. 基于 SCS215 寄存器表实现可独立测试的 `Robot` 类和配置注册；连接、校准、停止流程不得沿用 STS3215 的 `Operating_Mode`、`Homing_Offset` 或 4095 刻度假设。
2. 让 `get_observation` 与 `send_action` 使用锁定版本的真实接口，保持六关节安全范围、反馈检查和异常释放扭矩；先通过离线测试，再逐步实物验证。
3. 接入摄像头和明确的操作者输入，校验观测与动作时间戳，再录制少量本地 episode；确认格式和回放后才扩大采集。
4. 完成跨系统测试、协议依据、验证记录与许可证选择，再考虑向 LeRobot 上游提交适配器。

本台机械臂的硬件观察和未启用候选范围见[硬件登记](hardware.md)与[候选范围记录](experiments/scs215_safe_candidate.md)。课程作业提交包、视频与本机串口配置只保存在本地，不进入仓库。
