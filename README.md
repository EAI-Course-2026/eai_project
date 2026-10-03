# EAI Project · SCS215 / SO101

[![main checks](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml?query=branch%3Amain)
[![develop checks](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml/badge.svg?branch=develop)](https://github.com/EAI-Course-2026/eai_project/actions/workflows/tests.yml?query=branch%3Adevelop)

这是团队的应用仓库：维护六关节 SCS215 机械臂通信、共享标定、直角瑞士风格上位机、正逆运动学（FK/IK）、末端控制、课程视觉工具，以及标准 LeRobot 硬件插件。日常开发、文档和报告统一在这里进行。

**长期保留 `main` 和 `develop`。** main 是可复现的集成基线，也是默认分支；develop 承接日常集成。两者均通过 PR、代码审查和三项托管检查更新。临时功能分支从 develop 创建。macOS、Windows 和 CUDA 共用源码，以独立配置和锁文件区分环境，不需要长期平台分支。完整规则见 [CONTRIBUTING](CONTRIBUTING.md)。

## 当前能做什么

| 能力 | 已有证据 | 仍待验收 |
|---|---|---|
| 六关节通信、共享标定、当前姿态接管 | ID/型号读取、人工范围与 EEPROM 回读、双后端小幅同步动作、普通上位机控制反馈 | 全行程动力、负载、断电持久性 |
| 终端键盘末端控制 | 操作者已确认可移动；显示请求 TCP、编码器 FK 与关节误差 | 实际轴向、距离、模型零点的测量 |
| 上位机 FK/IK、单步/连续 XYZ、路径预览 | 离线回归与模拟页面验证 | 新版 GUI 末端实物验收；无碰撞规划，仅约束位置 |
| 标准 LeRobot 标定、遥操作、录制 | 插件发现、模拟标准 CLI 与本地 RGB episode；插件实物只读验证 | 真机械臂与摄像头同步录制、回放 |
| Windows / CUDA 环境 | Mac/Windows 离线 CI；Windows CUDA 软件安装与训练入口检查 | Windows 实物访问、真实 GPU 运算、训练与策略评估 |

CI 绿色表示上述软件检查通过；每次合并后的分支检查也必须通过。它不能替代实物或 GPU 验收。当前没有宣称完成整机验收的发布版本。见[里程碑](docs/milestones.md)和[报告](docs/reports/2026-10-03.md)。

## 首次安装

使用 **Python 3.12、uv 0.11.7**，安装方法见[环境指南](docs/environments.md)。从仓库根目录执行：

```sh
uv sync --locked
uv run --locked python scripts/check_env.py
uv run --locked python -m unittest discover -s tests -q
```

Windows PowerShell 可直接运行 `scripts/setup_windows.cmd`，完成相同的控制环境安装、环境检查与离线测试。现有 Conda 环境可以保留，团队可复现基线以两个 uv 锁文件为准。

| 用途 | 项目 / 环境 | Windows PyTorch | macOS PyTorch |
|---|---|---|---|
| 机械臂、上位机、传统视觉、离线测试 | 根目录 / `.venv` | CPU 2.11.0 | 原生 2.11.0，支持时可用 MPS |
| 模型训练 | `environments/training/` / 独立 `.venv` | 2.11.0+cu128 | 原生 2.11.0，无 NVIDIA CUDA |

Windows NVIDIA 主机执行 `scripts/setup_training_windows.cmd`（PowerShell 使用 `.\scripts\setup_training_windows.cmd`），包含真实 CUDA 运算检查。其他机器可运行：

```sh
uv sync --locked --project environments/training
uv run --locked --project environments/training python scripts/check_training_env.py
# NVIDIA 主机增加 --require-cuda；日常训练使用同一 --project 参数
```

两套环境都锁定 LeRobot **0.6.2 fork 提交 `6a077907c7989635218969ee78f5436f8faec92b`**，安装本仓库的 `lerobot_robot_scs215` 插件。该版本号不等于 PyPI 同号包。应用代码在本仓库维护，安装自动获取固定框架依赖，无需另外克隆协作者仓库。框架来源替换另行评审；当前不改变其仓库状态。根项目没有 `training` extra。

日常命令用 `uv run --locked`。拉取依赖变更、切换分支后，硬件操作前先 `uv sync --locked`。`--no-sync` 仅用于已同步且核验的固定环境，详见[命令规则](docs/environments.md#command-policy-on-windows-and-macos)。

## 连接自己的电脑

复制 `configs/hardware.example.toml` 为 `configs/hardware.local.toml`，填写 `[serial].port` 和本机 `[camera].index_or_path`。本机文件被 Git 忽略，不提交串口、摄像头编号、个人绝对路径或输出数据。机械臂标定按实体设备共享，换电脑不需要重新标定。见[标定契约](calibration/README.md)。

```sh
# 只读检查六轴 ID、型号、EEPROM 限位与当前位置
uv run --locked python scripts/arm_serial.py inspect
uv run --locked python scripts/arm_lerobot.py inspect
# 打开本地上位机，默认 http://127.0.0.1:8765/
uv run --locked python scripts/arm_desk.py
```

上位机启动加载配置、标定与服务；点击连接后才创建串口后端与硬件对象。串口列表实时从系统枚举，连接默认只读。点击「启用控制 · 保持当前姿态」接管当前位置；普通关节控制独立于归位/演示入口，归位为可选动作。末端工作区显示编码器 FK、单步/连续 XYZ 和整条路径预览；请求目标与反馈分列。暂停保留扭矩，停止/释放关闭扭矩，释放前支撑机械臂。同一串口只运行一个控制进程。详细步骤与控制边界见[上位机操作指南](docs/experiments/scs215_desk.md)。

离线课程工具不需要打开硬件：

```sh
uv run --locked eai-course mapping
uv run --locked eai-course fk
uv run --locked eai-course plan --delta-mm 0 0 10
```

终端键盘控制使用 `uv run --locked eai-course keyboard --port PORT --speed-mm-s 15`，显式确认后开启运动。GUI 必须先断开。标准 `lerobot-calibrate`、`lerobot-teleoperate`、`lerobot-record` 选择 `--robot.type=scs215_so101_follower`；实际运动另需 `--robot.enable_motion=true`。完整示例见[插件指南](docs/lerobot_plugin.md)。

## 日常协作

```sh
git switch develop
git pull --ff-only origin develop
git switch -c codex/your-task
# 开发、检查、提交后推送此分支，创建目标为 develop 的 PR
```

当前仓库管理员为 `abraxas914`、`youzhaozhao`、`LiHuaZhenRen`，共同担任默认代码审查人；Windows/GPU 与实物验收责任按每个 PR 指定，尚无已确认的固定负责人。作者提供证据，另一位审查人批准，维护者合并。阶段集成用 `develop → main` PR；main 紧急修复合并后必须同步回 develop。不直接推送受保护分支，不强推共享历史。

托管检查包含 `Offline tests (macos-latest)`、`Offline tests (windows-latest)`、`Windows CUDA training software (no GPU)`。前两项运行同一完整测试、环境与文档链接检查、前端语法检查和两个软件包构建；CUDA 检查安装独立训练环境并检查训练入口，不使用 GPU。功能分支以 PR 触发检查，main/develop 推送后再次检查。

仅在合并、无独有提交、任务结束且目标分支 CI 成功后清理临时分支。main、develop 与仍维护的 release/* 保留。平台环境属于主线内容，不能按“CUDA 分支”名称推断是否可以删除。远程维护配置记录在 [repository-policy.json](.github/repository-policy.json)，维护者可执行 `python scripts/check_remote.py` 核对实际保护规则（需要已登录 GitHub CLI）。

## 文件导航

| 路径 | 用途 |
|---|---|
| `src/eai_robot/arm/`、`hardware/` | 标定、控制、上位机与原生协议 |
| `src/eai_robot/course/` | 课程 FK/IK、规划、键盘和视觉 |
| `plugins/lerobot_robot_scs215/` | 可安装总线与 LeRobot Robot 插件 |
| `environments/training/` | 独立训练项目与 CUDA 锁文件 |
| `calibration/`、`configs/` | 共享标定、历史回执和本机配置模板 |
| `tests/`、`scripts/` | 离线回归、环境核验和操作入口 |
| `docs/reports/` | 按日期记录交付、证据、问题与验收范围 |

- [协作与分支生命周期](CONTRIBUTING.md) · [代理工作约定](AGENTS.md)
- [架构](docs/architecture.md) · [硬件](docs/hardware.md) · [Windows/CUDA/Conda](docs/environments.md)
- [课程基线](docs/course/README.md) · [35 文件迁移清单](docs/course/COVERAGE.md)
- [2026-10-02 功能报告](docs/reports/2026-10-02.md) · [2026-10-03 仓库与 CI 修复报告](docs/reports/2026-10-03.md)

历史课程资料保留在 `docs/course/source/`，其旧安装命令不作为当前入口。语音只有历史设计资料。项目整体开源许可证尚待团队选择；第三方资料已有声明保持原样，上游贡献需另行准备许可和可复现实物证据。
