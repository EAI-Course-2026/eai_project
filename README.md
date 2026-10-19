# EAI 项目 · SCS215 / SO101

[![main checks](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml?query=branch%3Amain)
[![develop checks](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml/badge.svg?branch=develop)](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml?query=branch%3Adevelop)

这是团队的机械臂应用仓库：维护六关节 SCS215 通信、共享标定、上位机、正逆运动学（FK/IK）、末端控制、视觉与语音能力，以及标准 LeRobot 硬件插件。课程背景和交付文档放在 `docs/course/`。

**长期保留 `main` 和 `develop`。** main 是可复现的集成基线，也是默认分支；develop 承接日常集成。两者均通过 PR、代码审查和三项托管检查更新。临时功能分支从 develop 创建。macOS、Windows 和 CUDA 共用源码，以独立配置和锁文件区分环境，不需要长期平台分支。完整规则见 [CONTRIBUTING](CONTRIBUTING.md)。

## 接下来的开发方向

研发收敛为两条主线：**模型与规则控制**（先完善视觉伺服、反馈与 FK/IK）和
**学习感知与决策**（学习检测、VLM 模仿、LLM Tools、视觉语音与 VLA）。
两者共用 develop，以短期任务分支交付；Agent 组合已有技能，voice 是交互方式。
见[主线与分支安排](docs/development_workstreams.md)。

仓库已经包含 VLA 本地运行器、模型工件管理和自有服务的跨机推理接口；权重与数据不入 Git。
外部 ASR 目前以逐行文本对接，LLM/VLM 托管 API 与 Omni 会话尚待实现。
框架依赖、模型资产、外部进程、自有服务、第三方 API 分别管理，接口与模块归属见
[推理边界决定](docs/inference_boundaries.md)。

## 当前能做什么

| 能力 | 已有验证记录 | 尚待验收 |
|---|---|---|
| 六关节通信、共享标定、当前姿态接管 | 读取 ID 和型号、人工核对范围并回读 EEPROM、双后端小幅同步动作、上位机普通控制反馈 | 全行程动力、负载、断电后的持久性 |
| 终端键盘末端控制 | 操作者已确认可移动；显示请求 TCP、编码器 FK 与关节误差 | 实际轴向、距离、模型零点的测量 |
| 上位机 FK/IK、单步/连续 XYZ、路径预览 | 离线回归与模拟页面验证 | 新版 GUI 末端实物验收；无碰撞规划，仅约束位置 |
| LeRobot 标定、遥操作、录制 | 插件发现、标准命令行模拟、本地 RGB 数据片段；插件实物只读验证 | 真机械臂与摄像头同步录制、回放 |
| 语音命令调度（白名单 + 演练模式） | 7 条固定命令的解析、姿态计划、`stop` 抢占与 27 项离线回归；默认演练模式不打开串口 | 真机逐条命令验收、夹爪姿态记录、真实识别文本覆盖率 |
| ArUco 中央区域伺服 | 2026-10-18末端相机真机10次纠偏、27步反馈确认，已录制demo | 已有短程前伸日志；完整停靠、精密四角点及任意物体真机跟随待验收 |
| Windows / CUDA 环境 | Mac/Windows 离线 CI；Windows CUDA 软件安装与训练入口检查 | Windows 实物访问、真实 GPU 运算、训练与策略评估 |

持续集成检查通过表示上述软件检查通过；每次合并后，目标分支的检查也必须通过。这些检查不能替代实物或 GPU 验收。目前没有宣称已完成整机验收的发布版本。见[里程碑](docs/milestones.md)和[报告](docs/reports/2026-10-03.md)。

## 首次安装

使用 **Python 3.12、uv 0.11.7**，安装方法见[环境指南](docs/environments.md)。从仓库根目录执行：

```sh
uv sync --locked
uv run --locked python scripts/check_env.py
uv run --locked python -m unittest discover -s tests -q
```

Windows PowerShell 可直接运行 `scripts/setup_windows.cmd`，完成相同的控制环境安装、环境检查与离线测试。现有 Conda 环境可以保留，团队可复现基线以两个 uv 锁文件为准。

| 用途 | 项目 / 环境 | Windows 版 PyTorch | macOS 版 PyTorch |
|---|---|---|---|
| 机械臂、上位机、传统视觉、离线测试 | 根目录 / `.venv` | CPU 2.11.0 | 原生 2.11.0，支持时可用 MPS |
| 模型训练 | `environments/training/` / 独立 `.venv` | 2.11.0+cu128 | 原生 2.11.0，无 NVIDIA CUDA |

Windows NVIDIA 主机执行 `scripts/setup_training_windows.cmd`（PowerShell 使用 `.\scripts\setup_training_windows.cmd`），包含真实 CUDA 运算检查。其他机器可运行：

```sh
uv sync --locked --project environments/training
uv run --locked --project environments/training python scripts/check_training_env.py
# NVIDIA 主机增加 --require-cuda；日常训练使用同一 --project 参数
```

两套环境都锁定 LeRobot **0.6.2 分支版本，提交号为 `6a077907c7989635218969ee78f5436f8faec92b`**，并安装本仓库的 `lerobot_robot_scs215` 插件。此版本号不等于 PyPI 上同名、同版本号的包。应用代码在本仓库维护；安装时会自动获取固定版本的框架依赖，无需另外克隆协作者仓库。替换框架来源需要单独评审；当前不改变其仓库状态。根项目没有 `training` 可选依赖组。

日常命令使用 `uv run --locked`。拉取依赖变更或切换分支后，操作硬件前先运行 `uv sync --locked`。`--no-sync` 仅用于已经同步并核验过的固定环境，详见[命令规则](docs/environments.md#command-policy-on-windows-and-macos)。

## 连接自己的电脑

复制 `configs/hardware.example.toml` 为 `configs/hardware.local.toml`，填写 `[serial].port` 和本机 `[camera].index_or_path`。本机文件被 Git 忽略，不提交串口、摄像头编号、个人绝对路径或输出数据。机械臂标定按实体设备共享，换电脑不需要重新标定。见[标定契约](calibration/README.md)。

```sh
# 只读检查六轴 ID、型号、EEPROM 限位和当前位置
uv run --locked python scripts/arm_serial.py inspect
uv run --locked python scripts/arm_lerobot.py inspect
# 打开本地上位机，默认 http://127.0.0.1:8765/
uv run --locked python scripts/arm_desk.py
```

上位机启动时加载配置、标定和服务；点击连接后才创建串口后端和硬件对象。串口列表实时读取自系统，连接操作默认只读。点击「启用控制 · 保持当前姿态」后接管当前位置；普通关节控制与归位、演示入口相互独立，归位是可选动作。末端工作区显示编码器正运动学结果、单步或连续 XYZ 控制，以及整条路径预览；请求目标和反馈分开显示。暂停时保留扭矩；停止或释放时关闭扭矩。释放前先支撑机械臂。同一串口只运行一个控制进程。详细步骤与控制边界见[上位机操作指南](docs/experiments/scs215_desk.md)。

离线运动学与规划命令不需要打开硬件：

```sh
uv run --locked eai-course mapping
uv run --locked eai-course fk
uv run --locked eai-course plan --delta-mm 0 0 10
```

ArUco 视觉伺服可用 `uv run --locked eai-course visual-servo --image PATH_TO_IMAGE --marker-id 0`
演练简单版；`--mode ibvs` 加内参与标记边长可演练四角点控制。
增加 `--target object --roi X Y W H` 可用光流跟踪选定的有纹理物体。
该入口只输出相机运动建议，尚未接入机械臂执行，见
[视觉伺服分步实现](docs/course/aruco_visual_servo.md)。

2026-10-18另通过 `vision --profile configs/visual_servo/aruco_center_mvp.toml --execute`
完成中央区域真机MVP，日志记录10次纠偏；普通检测不加 `--execute`。
这与只读 `visual-servo` 演练入口不同，精密四角点和自动前进尚未验收。
源码已按六个一级能力目录和第二级功能组整理；旧 Python 导入路径需按[迁移对照](docs/source_layout_migration.md)更新。
参数和证据见[本轮报告](docs/reports/2026-10-18-visual-servo-round1.md)，
下一阶段见[精细伺服计划](docs/experiments/visual_servo_precision.md)。
离线复盘使用 `scripts/summarize_visual_servo.py --log LOG --output-dir OUTPUT`，不访问设备。

终端键盘控制使用 `uv run --locked eai-course keyboard --port PORT --speed-mm-s 15`，明确确认后才会开始运动。图形上位机必须先断开连接。标准命令 `lerobot-calibrate`、`lerobot-teleoperate`、`lerobot-record` 使用 `--robot.type=scs215_so101_follower`；实际运动还需要设置 `--robot.enable_motion=true`。完整示例见[插件指南](docs/lerobot_plugin.md)。

语音命令调度默认处于演练模式，只进行白名单解析、姿态规划和 `stop` 抢占，不打开串口：

```sh
uv run --locked eai-course voice --text "挥手"
uv run --locked eai-course voice --list-commands
```

识别文本通过标准输入逐行传入（每行是一条识别结果）；真机运动需要 `--enable` 并由操作者明确确认，
详见[语音命令调度](docs/course/voice/README.md)。

## 日常协作

```sh
git switch develop
git pull --ff-only origin develop
git switch -c codex/your-task
# 开发、检查、提交后推送此分支，创建目标为 develop 的 PR
```

当前仓库管理员为 `abraxas914`、`youzhaozhao`、`LiHuaZhenRen`，共同担任默认代码审查人；Windows/GPU 与实物验收责任按每个 PR 指定，尚无已确认的固定负责人。作者提供证据，另一位审查人批准，维护者合并。阶段集成用 `develop → main` PR；main 紧急修复合并后必须同步回 develop。不直接推送受保护分支，不强推共享历史。

托管检查包括 `Offline tests (macos-latest)`、`Offline tests (windows-latest)` 和 `Windows CUDA training software (no GPU)` 三项。前两项运行相同的完整测试、环境核验、文档链接检查、前端语法检查和两个软件包构建；CUDA 检查安装独立训练环境并核验训练入口，不实际使用 GPU。功能分支通过拉取请求触发检查；推送至 main 或 develop 后还会再次检查。

仅在合并、无独有提交、任务结束且目标分支 CI 成功后清理临时分支。main、develop 与仍维护的 release/* 保留。平台环境属于主线内容，不能按“CUDA 分支”名称推断是否可以删除。远程维护配置记录在 [repository-policy.json](.github/repository-policy.json)，维护者可执行 `python scripts/check_remote.py` 核对实际保护规则（需要已登录 GitHub CLI）。

## 文件导航

源码各子目录的职责、兼容边界和新增能力的放置规则见
[`src/README.md`](src/README.md)。一级收敛为六个能力目录，第二级按实际功能分组；不同设备执行路径仍保留各自门禁。

ACT、SmolVLA、π0.5 的模型加载、跨机器推理、本地执行门禁与示范数据工具见
[推理基础设施指南](docs/policy/README.md)及[后续工作交接](docs/policy/next_session.md)。
SmolVLA 已完成 Mac MPS 的真实离线加载与动作块输出；GPU、本机模型适配和自主实物任务
仍待验收，不能由基础模型输出或软件 CI 替代。

| 路径 | 用途 |
|---|---|
| `src/eai_robot/applications/` | desktop、teleoperation、visual_servo、tools：用户流程与设备会话 |
| `src/eai_robot/calibration/` | servo、vision：舵机标定与相机/手眼接入 |
| `src/eai_robot/control/` | kinematics、planning、execution、visual_servo：计算目标到在线执行 |
| `src/eai_robot/vision/` | detection、tracking、targets：检测、跟踪与目标图案 |
| `src/eai_robot/policy/` | inference、execution、data：策略推理、执行与数据生命周期 |
| `src/eai_robot/hardware/` | drivers、robots：设备通信与机器人装配 |
| `src/eai_robot/cli.py` | eai-course 命令分派，用户命令与参数不变 |
| `docs/course/` | 课程说明、历史材料、作业迁移记录和验收文档 |
| `plugins/lerobot_robot_scs215/` | 可安装总线与 LeRobot Robot 插件 |
| `environments/training/` | 独立训练项目与 CUDA 锁文件 |
| `calibration/`、`configs/` | 共享标定、历史回执和本机配置模板 |
| `tests/`、`scripts/` | 离线回归、环境核验和操作入口 |
| `docs/reports/` | 按日期记录交付、证据、问题与验收范围 |

- [协作与分支生命周期](CONTRIBUTING.md) · [代理工作约定](AGENTS.md)
- [架构](docs/architecture.md) · [硬件](docs/hardware.md) · [Windows/CUDA/Conda](docs/environments.md)
- [VLA 部署阶段方案](docs/vla_deployment.md) · [后续工作交接](docs/handoffs/vla_next_session.md)
- [多模态 Agent 开发规划](docs/agent_development_plan.md) · [策略推理基建](docs/policy/README.md) · [语音命令调度](docs/course/voice/README.md)
- [课程基线](docs/course/README.md) · [35 文件迁移清单](docs/course/COVERAGE.md)
- [2026-10-02 功能报告](docs/reports/2026-10-02.md) · [2026-10-03 仓库与 CI 修复报告](docs/reports/2026-10-03.md)

历史课程资料保留在 `docs/course/source/`，其中的旧安装命令不再作为当前入口。语音识别端已在本机
`edge_inference_lab` 实验中验证；本仓库只接收识别文本并进行白名单命令调度。判断标准与真机
验收前置见[语音命令调度](docs/course/voice/README.md)。项目整体开源许可证尚待团队选择；第三方资料
已有声明保持原样，上游贡献需另行准备许可和可复现实物证据。
