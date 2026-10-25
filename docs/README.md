# 文档导航与责任入口

`docs/` 维护可共享知识。程序输出放 [outputs](../outputs/README.md)，原始证据放本机 local_artifacts，固定交付放 [deliverables](../deliverables/README.md)；完整边界见[工作区规则](workspace.md)。

当前指南负责运行行为，架构/契约负责边界，规划负责未来目标，日期报告只负责当时的证据。相同主题不等于重复文件；不要用旧 handoff 覆盖当前源码。仓库首页和本页负责导航，避免在多份 README 复制完整命令。

## 项目与架构

- [项目定位、硬件边界与任务质量](project_scope.md)
- [架构与 LeRobot 接入状态](architecture.md)
- [两条开发主线与分支安排](development_workstreams.md)（规划/待验收）
- [推理归属、外部依赖与接口决定](inference_boundaries.md)
- [三层 Agent 架构与 pi 参考](agent_layers.md)
- [Agent 标准契约与复用决策](agent_standard_contract.md)
- [Windows, CUDA and Conda setup](environments.md)
- [硬件登记](hardware.md)
- [阶段目标与验收](milestones.md)（规划/待验收）
- [文档、运行输出与本机材料的边界](workspace.md)
- [源码路径迁移对照](source_layout_migration.md)

## Agent：当前文本与协议

- [三层 Agent 运行指南](agent_runtime.md)
- [多服务 API 配置](agent_providers.md)
- [聊天 API 接入与验收](agent_api_acceptance.md)
- [Agent ArUco 真实机械臂验收](agent_hardware_acceptance.md)
- [多模态 Agent 与动作模仿开发规划](agent_development_plan.md)（规划/待验收）

## 机械臂、感知与实验

- [SCS215 plugin: direct scripts and standard LeRobot CLI](lerobot_plugin.md)
- [SCS215 plugin validation — 2026-10-02](lerobot_plugin_validation.md)（历史入口）
- [单关节原始计数响应测试](experiments/joint_response.md)
- [任意物体伺服：需要你决策的清单](experiments/object_servo_decisions.md)（规划/待验收）
- [感知基础设施：模型怎么接进来](experiments/perception_infrastructure.md)
- [SCS215：90° 阶跃与 PID 实验](experiments/pid_response.md)
- [作业 2：SCS215 六关节编号、校准和同步控制](experiments/scs215_arm.md)
- [六关节固定起点演示 v3](experiments/scs215_demos.md)
- [SO101 六关节本机上位机](experiments/scs215_desk.md)
- [SCS215 安全工作范围候选（尚未启用）](experiments/scs215_safe_candidate.md)
- [舵机基础实验说明](experiments/servo_basics.md)
- [任意物体伺服：从 ArUco 扩展到"任何东西"](experiments/visual_servo_any_object.md)
- [固定运动规律伺服：五区域 + 前向 1cm](experiments/visual_servo_fixed_rule.md)
- [下一轮：精细视觉伺服](experiments/visual_servo_precision.md)（规划/待验收）

## 学习策略与接续

- [ACT、SmolVLA、π0.5 推理基建](policy/README.md)
- [推理基建接续条件](policy/next_session.md)
- [VLA 部署方案入口](vla_deployment.md)（历史入口）
- [VLA 交接入口](handoffs/vla_next_session.md)（历史入口）

## 课程与迁移

- [Shared course development baseline](course/README.md)
- [Migration receipt — 2026-10-02](course/MIGRATION.md)
- [Course migration coverage](course/COVERAGE.md)
- [Manual calibration — 2026-10-02](course/CALIBRATION.md)（历史入口）
- [视觉伺服：当前入口与实验基线](course/aruco_visual_servo.md)
- [ArUco visual servoing scaffold](course/aruco_visual_servo_history.md)（历史入口）
- [视觉伺服作业报告：固定运动规律闭环](course/visual_servo_homework.md)
- [语音命令调度（可替换识别端 + 白名单意图 + 姿态调度）](course/voice/README.md)
- [Voice interaction architecture](course/voice/ARCHITECTURE.md)

## 日期报告与原始来源

以下保留来源和验收血缘，不作为当前安装或新开发待办。报告标题中的日期是事件范围，分支、测试数与设备状态不能跨版本沿用。

### reports

- [2026-10-02 项目收尾报告](reports/2026-10-02.md)
- [2026-10-03 仓库协作与 CI 修复](reports/2026-10-03.md)
- [ArUco 真机首步记录 — 2026-10-17](reports/2026-10-17-aruco-hardware-first-step.md)
- [源码职责重构与视觉标定接入 — 2026-10-17](reports/2026-10-17-source-boundaries.md)
- [视觉伺服先行实现 — 2026-10-17](reports/2026-10-17-visual-servo.md)
- [2026-10-18 视觉伺服 MVP 收尾记录](reports/2026-10-18-visual-servo-round1.md)
- [两条研发主线、推理边界与分支整理](reports/2026-10-19-agent-boundaries.md)
- [三层 Agent 落地与项目边界](reports/2026-10-19-agent-layers.md)
- [文本 Agent 运行模块与可替换提供方边界](reports/2026-10-19-agent-runtime.md)
- [固定规律伺服反馈守卫：阶段提交记录](reports/2026-10-19-servo-feedback-guards.md)
- [多提供方配置与 dotenv 接入](reports/2026-10-22-agent-provider-configuration.md)
- [Agent 标准协议与官方 SDK 修正](reports/2026-10-22-agent-standard-sdk.md)
- [Conda 薄适配与 Quick start](reports/2026-10-22-conda-adapter.md)
- [2026-10-24 文档与运行输出血缘审查](reports/2026-10-24-artifact-lineage.md)
- [2026-10-24 Agent 视觉工具与外部 LLM 验收](reports/2026-10-24-agent-visual-tools.md)
- [2026-10-24 Agent ArUco 真机适配层软件记录](reports/2026-10-24-agent-hardware-adapter.md)
- [2026-10-25 网页真机 Agent 模式](reports/2026-10-25-web-hardware-agent.md)

### archive

- [推理基建交接与待用户输入](archive/2026-10-03-policy-infrastructure.md)
- [VLA 部署与适配推理阶段](archive/2026-10-03-vla-deployment.md)
- [下一 session：VLA 部署与适配推理 Handoff](archive/2026-10-03-vla-handoff.md)

### course/source

- [EAI Course LeRobot 项目交接](course/source/HANDOFF.md)
- [LeRobot-LICENSE.txt](course/source/LeRobot-LICENSE.txt)
- [EAI Course LeRobot](course/source/README.md)
- [团队共享机械臂标定](course/source/calibration/README.md)
- [Vision](course/source/vision/README.md)
- [基于视觉反馈的机械臂闭环控制实验报告](course/source/vision/REPORT.md)
- [Voice](course/source/voice/README.md)
- [Week 4](course/source/week4/README.md)
- [Week 4：预设姿态与动作序列](course/source/week4/task1/README.md)
- [Week4 Task 2](course/source/week4/task2/README.md)

## 审查中的后继能力

PR #25 的页面配置、云端语音和 Agent 交接手册在独立评审中，以[该 PR 的中文说明](https://github.com/EAI-Course-2026/eai_project/pull/25)和实际分支为准。合并后把相应手册加入当前入口；未合并时不把它们列为 develop 可用能力。

本轮快照与逐项处置见[血缘审查](reports/2026-10-24-artifact-lineage.md)。新文档应选择明确职责并从本页或所属能力入口链接；退役时保留后继关系，避免每次交接再复制一套规则。
