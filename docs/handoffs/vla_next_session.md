# 下一 session：VLA 部署与适配推理 Handoff

日期：2026-10-03。先读本文件，再读 [阶段方案](../vla_deployment.md)。

## 用户目标与已有决定

下一阶段实现 VLA 模型部署、SCS215/SO101 输入输出适配和推理执行。
用户已经确认终端 Cartesian 键盘控制可移动，并要求演示速度提升，当前建议入口
使用 `--speed-mm-s 15`。这证明人工末端控制能动，不证明厘米级精度、GUI 实物验收或
VLA 自主任务完成。本次只编写文档，不启动模型/硬件，不改变标定和依赖。

继续在独立应用仓库开发；安装明确使用团队 0.6.2 fork 的固定完整 SHA。
保留 0.6.1 作迁移对照；控制环境与 CUDA 环境分开。优先外挂/插件，不改已安装框架。
建议首选 SmolVLA 做离线部署与契约验证，GPU 选型/任务/checkpoint 尚未最终确认。
不要将这个建议当成用户已提供了 GPU 或匹配的微调权重。

## 定位与分支

canonical repo：`EAI-Course-2026/eai_project`，本机从当前应用仓库继续。
本轮检查到 main/develop 同为 `be9ac819fed847ad2a00245c7e569d8a7c417146`。
后续可能已有新提交，以新 session 的 `git status/log` 与远程为准。
旧 `/private/tmp/eai-scs215-lerobot-plugin` 已不存在，不再作为运行入口。

先读取仓库根 `AGENTS.md`、`README.md`、`CONTRIBUTING.md`。新功能从最新 develop
创建 `codex/*`，正常 PR 指向 develop，需要另一位维护者审查与 required CI；
不要直接推 main/develop，不替代人的审查，不删除其他工作树或本地材料。
若此文档任务分支尚未合入，先从该分支读取文档；实现分支仍按 develop 流程建立，
不要默认为文档已在主线。任务 PR/文档提交的最终状态看本次 session 收尾。

```sh
git status --short
git log -3 --oneline
git switch develop
git pull --ff-only origin develop
git switch -c codex/vla-deployment
uv sync --locked
uv run --locked python scripts/check_env.py
uv run --locked python scripts/check_repository.py
```

已有未提交工作时先保存/隔离，不盲目切换覆盖。uv 固定 0.11.7，Python 3.12。
以上只做仓库与软件检查，不打开串口。训练环境入口是
`uv sync --locked --project environments/training`；有真实 NVIDIA 设备时检查脚本增加
`--require-cuda`，不要把 CUDA wheel 安装成功写成 GPU 验收通过。

## 必读文件与现状

- [架构](../architecture.md)：统一 Robot/总线、归一化范围、GUI 调度边界。
- [插件指南](../lerobot_plugin.md)与[验证](../lerobot_plugin_validation.md)：标准 CLI 状态。
- [里程碑](../milestones.md)：哪些是软件/模拟/实物证据。
- [仓库报告](../reports/2026-10-03.md)：main/develop、审查、CI 与已有修正。
- [训练环境](../../environments/training/README.md)：双锁与真实 CUDA 要求。
- `plugins/lerobot_robot_scs215/src/lerobot_robot_scs215/robot.py`：观测、动作、启用与断开。
- `src/eai_robot/arm/desk_service.py`、`desk_cartesian.py`：单硬件线程、输入租期与反馈故障。
- `src/eai_robot/course/kinematics/keyboard_control.py`：目标回读、扭矩验证与 encoder FK。
- `tests/test_lerobot_plugin.py`、`test_desk_cartesian.py`：已有模拟与故障回归。

共享六关节 raw 范围：169–898、60–740、196–772、67–681、47–974、239–640。
五轴归一化动作 -100..100，夹爪 0..100。不是 URDF degrees，也不是 TCP。
换电脑不重标定；机器串口/相机信息只写本机忽略配置，不在文档或共享模板硬编码。
摄像头历史识别成功不表示新 session 已完成相机同步验收。

## 下一 session 第一个可交付任务

1. 问清可用 GPU/显存/系统、首个桌面任务和是否已有微调 checkpoint/示范数据。
   可先做软件契约检查，不因这些答案尚缺就停在规划阶段。
2. 核验固定 fork 的 SmolVLA/pre-post processors 与 `async` extras。
   使用真实 checkpoint 配置检查相机键、state/action dimensions、统计和模型 revision；
   不把“六维”自动视为适配通过。若无法下载，先用假策略完成相同接口与门禁测试。
3. 在训练项目中显式加入所需模型 extra 并锁定，先复用该环境；控制根环境仍保持固定。
   Mac 网络客户端如需 async 依赖，单独声明并检查锁变更。
4. 新增最小 `policy/` 契约、输入适配、动作后处理与离线 dry-run。
   模型加载和运行不拥有串口；执行模块才是本地唯一设备所有者。
5. 用模拟设备和假时钟验证未知单位/错误顺序/缺相机/NaN/首步突跳/过期/断线/扭矩丢失。
   给出下一步真实只读 shadow 的具体运行入口与验收条件。

首个 PR 不要求同时完成微调或自主实物任务；要交付能验证的部署与契约基础。
最终自主 rollout 仍要用户明确授权，不能把本轮文档任务或上轮人工键盘授权视为新模型
自主运动的持续授权。禁止维护任务自动打开串口。

## 必须保留的判断

- 关节范围标定与 URDF 真角度标定是两件事。IK 人工控制已可动，模型几何对齐尚待测量。
- 预训练权重能输出动作，不代表适合本机范围/镜头/任务；优先收集匹配示范或验证已有权重。
- 标准录制的真实设备路径尚待验收；键盘/GUI recorder 不是已经存在的完整数据系统。
- 固定 LeRobot 有 async server/client，但原客户端会自行连接 Robot；应用要核验生命周期与
  超时/接管。不要让 GUI 与另一个 robot_client 同时抢串口。
- 单步限幅不能替代避碰。不要扩大硬件范围、关闭反馈或弱化停止来让模型动作通过。
- ACT 可以作为同数据的非语言行为克隆对照，但不能把它当作已实现 VLA。
- 数据集/权重/视频留在被忽略目录或外部存储；Git 只保存配置、摘要、契约和证据索引。

## 可粘贴的新 session 启动请求

> 继续 eai_project 的 VLA 部署与适配推理阶段。先读 AGENTS.md、README.md、
> CONTRIBUTING.md、docs/handoffs/vla_next_session.md 和 docs/vla_deployment.md。
> 按 develop → codex/* → develop PR 流程工作，保留固定 0.6.2 fork 和控制/训练双锁。
> 优先实现 SmolVLA 的离线加载、明确的 observation/action 契约与 dry-run、假策略执行门禁。
> 核对 GPU、checkpoint 和数据条件后再推进真实只读 shadow、微调与受控 rollout。
> 不自动打开串口或启用扭矩；不编辑 site-packages；分清软件、GPU 与实物验收。
