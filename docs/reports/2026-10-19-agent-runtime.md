# 文本 Agent 运行模块与可替换提供方边界

2026-10-19。基于 develop `7877a4f`，实现分支 `codex/learning-agent-runtime`。
本轮没有连接串口/相机/麦克风，没有启用扭矩或访问真实模型服务。

## 交付

已建立 `policy/agents/`、`applications/agent/`，提供可配置文本 Chat Completions HTTP、
注入工具、有界循环、停止代次、五个 dry-run 工具、逐行输入与固定离线 demo。
核心 prompt 和参考姿态资源已核验进入 wheel；不依赖个人 checkout 或特定语音服务。
运行入口、配置和 API Key 环境变量见[运行指南](../agent_runtime.md)。

带入用户在原工作区已完成的语音边界文档调整，原工作区未提交文件保持原状。
`edge_inference_lab` 仅为历史案例；聊天配置、外部 ASR 配置和机器人硬件配置保持独立。
结构化识别事件、ASR/TTS、VLM/Omni 与真实运动技能没有在本轮宣称完成。

## 软件验证与边界

新增 Agent 测试覆盖真实 loopback HTTP 请求/工具回传、任意目录用户配置、凭据脱敏、
拒绝重定向、非法参数、重复调用、预算、停止/迟到响应、请求超时与响应体上限。
离线 demo 是固定 fixture，HTTP 测试服务使用预设响应，均不等于真实外部 LLM API 验收。
没有供应商/模型/凭据配置，不能宣称已完成课程 4.2 的真实 API 演示或 4.3 视觉语音验收。

本地检查日志保留在忽略的 `local_artifacts/agent-runtime/`。
本地完整测试、环境、构建和托管三项 CI 的最终回执写入 PR 描述/Checks，须核对准确 head。
本轮 PR 未合并，无本轮 post-merge CI，不代替另一维护者审查。
