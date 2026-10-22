# 三层 Agent 落地与项目边界

2026-10-19。延续 `codex/learning-agent-runtime` / [PR #21](https://github.com/EAI-Course-2026/eai_project/pull/21)。
本报告补充[首轮运行模块报告](2026-10-19-agent-runtime.md)：此前位于核心目录的模型协议与 prompt
现已分别迁到模型接口层与应用层；历史报告保留原阶段事实。

## 交付

先明确[项目定位与硬件边界](../project_scope.md)，再按[三层架构](../agent_layers.md)实现：

- `policy/model_io/`：规范化消息、工具与流事件；JSON/SSE 协议适配；每次请求解析环境凭据。
- `policy/agents/`：跨轮有界记忆、刷新上下文、工具注册/检查/进度、会话事件与取消排空。
- `applications/agent/`：机器人 prompt、能力配置、技能绑定、逐行输入、配置与 CLI。

参考 pi 快照 `d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03` 的职责划分与生命周期；
没有复制代码或新增 pi/Node 运行依赖。Python 接口迁移见[迁移对照](../source_layout_migration.md)。
保留独立的 VLA 推理与既有规则语音命令入口。

## 验证与限制

Agent 专项覆盖规范化协议、实时 SSE/参数分片、截断拒绝、逐次鉴权、对话记忆、上下文年龄、
hook、停止并发、迟到进度隔离、超时排空、能力隐藏与 JSONL 输出。
本地完整 529 项测试通过，其中 Agent 专项 30 项通过。三项托管 CI 的准确提交与运行链接记录在 PR 描述/Checks。
环境、仓库、JavaScript、wheel 构建分别核验；解包 wheel 后，在独立工作目录运行离线示例，
核对新模型模块和应用 prompt 与源码字节一致，旧路径未残留。

没有调用真实模型 API、采集相机/麦克风或打开硬件串口。技能仍为 dry-run；
本轮不等于课程 4.2 的外部 API 调用验收或 4.3 的视觉语音/实物验收。
核心 run_settled 仅证明本地编排和核心工具 worker 排空，不证明远端推理结束、应用背景任务结束或机械臂物理停止。
PR 尚待另一维护者审查，未合并，无本轮合并后 CI。
