# 架构与 LeRobot 接入状态

下一阶段的 VLA 模型部署、观测/动作契约与推理执行方案见
[VLA 部署规划](vla_deployment.md)和[接续 handoff](handoffs/vla_next_session.md)。
其中 `policy/` 与推理运行器是拟新增模块，不属于已实现架构。

SCS215 适配维护在独立仓库 `EAI-Course-2026/eai_project`，以可安装插件
`lerobot_robot_scs215` 放在 `plugins/lerobot_robot_scs215/`。直接脚本、网页默认后端、
课程机器人和标准 LeRobot CLI 共用这个插件中的电机总线。

```text
arm_lerobot.py / 网页 / demo / 课程姿态
    → ArmController（0..1、归位、插值、反馈停止）
    → LeRobotBackend ──────────────────────┐
                                          │
课程 FK/IK、键盘、视觉 / 标准 LeRobot CLI     │
    → SCS215SO101Follower（Robot 接口）      │
    → get_observation / send_action ───────┤
                                          ▼
                          SCS215MotorsBus（同一个类）
                              → Feetech SDK → 舵机

arm_serial.py / 原生诊断选项 → SerialBackend → 项目内 SCS 协议 → 舵机
```

`CourseFollower`、`CourseMotorsBus` 是兼容别名，没有第二份通信或机器人实现。
原生后端保留作协议诊断与迁移对照；网页和 demo 默认选择插件。`ArmController` 仍负责
本项目的接管、归位与插值，插件 `Robot` 提供标准观测/动作接口。网页关节与末端控制
由单一硬件线程调度；独立课程 CLI 和标准 CLI 各有进程生命周期。
同一串口同时只运行一个控制进程。

## 框架源码边界

仍安装团队 **0.6.2 fork 的 `6a077907c7989635218969ee78f5436f8faec92b`**。
这个固定快照包含协作者此前的源码修改（早于 `6a07790` 的 `fe81d5e`）。
本次集成没有新增 fork 修改，也不编辑 `.venv/site-packages/lerobot`。旧 0.6.1 环境
保留作迁移对照；控制环境与 CUDA 训练环境各有独立锁文件。

框架的 `register_third_party_plugins()` 自动导入插件，`RobotConfig` 注册
`scs215_so101_follower`，通用工厂创建插件 Robot。标准 `lerobot-calibrate`、
`lerobot-teleoperate`、`lerobot-record` 因此能调用同一适配层，无需改命令源码。
必须指定 `--robot.type=scs215_so101_follower`；内置 `so101_follower` 仍选择旧 fork 类。

插件继承 `FeetechMotorsBus` 扩展型号 1315、协议 1、1024 刻度、大端字节序、写应答清理
和回读确认。型号表仅在插件类中复制扩展，不修改框架全局表。连接验证 ID、型号与 EEPROM
范围，不执行 STS3215 的 PID、模式、4095 刻度或 Homing Offset 初始化。日常控制和
扭矩释放不解锁 EEPROM。将来只有扩展接口无法表达必要功能时，才做最小 fork 补丁并更新
锁定提交。切换官方版本需要另行验证，不能仅凭相同版本号替换依赖。

## 标定、动作与验收

共享文件仍为 `calibration/scs215_so101.json`，六关节范围仍为 `169..898`、`60..740`、
`196..772`、`67..681`、`47..974`、`239..640`。默认从仓库根目录解析，亦可传
`--robot.calibration_path`。机器人 `id` 标签不会另选个人 HF 缓存标定；显式使用
`calibration_dir` 时按目录与 `id` 选文件。

网页/直接控制使用六个 `0..1`，LeRobot 手臂关节使用 `-100..100`，夹爪使用 `0..100`。
这些是归一化目标，不是角度。最多 3 刻度的反馈漂移只用于接受自然停放位置；初始保持目标
始终限制在真实范围内，超出容差的观测会报错，不会悄悄裁剪为有效训练数据。

默认连接只读。标准 CLI 需要 `--robot.enable_motion=true`，直接 Python 调用
`enable_motion()`。启用前检查扭矩、报警、反馈并预写当前姿态；部分启用失败尝试释放每个
指定关节。退出只释放本次尝试启用过的扭矩，避免只读会话改变外部已有状态。

标准标定可以回读验证旧文件，或手动采样生成软件范围。新范围与 EEPROM 范围取交集，保留
已有方向；需要 `SAVE` 才替换文件，旧 JSON 与元数据备份。不会改 EEPROM 或硬件零点。
更改硬件限位仍使用明确的维护流程。重新标定会使旧 home 的标定绑定失效，需要重新示教，
并复核 FK/IK 物理对齐。

`max_relative_target` 超出时拒绝动作，不静默裁剪。固定 fork 的录制循环保存请求动作，
因此成功发送的动作必须与请求一致。位置限位与单步限制不能代替避碰验证。

| 层次 | 当前验收范围 |
|---|---|
| 直接通信、网页归位与停止 | 原有实物验证保留，离线回归继续覆盖 |
| 插件发现、配置、Robot 工厂 | 新进程自动加载与标准 CLI 路由有离线测试 |
| 标定、遥操作、录制 | 标准命令在模拟设备上运行；录制生成真实本地 RGB episode 文件 |
| 实物同步采集、策略回放 | 仍需小规模真实机械臂验收 |
| CUDA 训练与评估 | 仍需团队 GPU 与实物验证 |

示例见[插件指南](lerobot_plugin.md)，环境与迁移见[共享基线](course/README.md)。

## 上位机 FK/IK 与调度

启动仅加载配置、标定与起点文件并创建服务，连接请求才创建后端、ArmController
和硬件对象。串口候选来自系统枚举。当前姿态接管独立于归位/演示入口，先确认保持
目标，再启用扭矩。DeskCartesian 共用课程 URDF、关节映射、NumPy FK、DLS
位置 IK 和直线规划。编码器经映射和 FK 得到 XYZ；XYZ 请求经整条路径预检、
IK 与范围检查生成 raw 目标，由同一个后端执行。夹爪保持原位，可单独点动。

连续输入采用会话与递增序号，300 ms 不续期即暂停保持。暂停保留扭矩，释放关闭
扭矩；硬件故障尝试释放。页面失焦/切换清除方向，需重新开启键盘。目标、编码器 FK
与跟随误差分别显示。第一版只约束位置，朝向可能变化，没有碰撞规划；FK 是模型
估算。详见[操作指南](experiments/scs215_desk.md)和[今日报告](reports/2026-10-02.md)。

## 仓库协作边界

应用与插件在本仓库统一维护；main 为默认集成基线，develop 为永久日常集成分支。Mac/Windows/CUDA 的区别由配置和两套依赖锁表达，不复制平台源码分支。分支保护、CODEOWNERS、PR 与合并后检查、验收责任和临时分支清理见[协作规则](../CONTRIBUTING.md)。
