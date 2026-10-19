# 三层 Agent 架构与 pi 参考

2026-10-19。参考用户提供的本地 pi 快照 `d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03`。
只借鉴接口与职责划分，不把本机 checkout、Node 或 pi 包作为运行依赖，也不修改参考仓库。
产品目标和评测以[项目定位](project_scope.md)为准。

## 三层职责

| pi 对照 | 本项目目录 | 负责 | 不负责 |
|---|---|---|---|
| pi-ai | `policy/model_io/` | 供应商中立消息/工具定义、请求/回复、文本流事件、协议编码、环境凭据 | 工具执行、会话业务、机械臂 |
| pi-agent-core | `policy/agents/` | 对话上下文、工具循环、参数检查、调用预算、取消/代次、进度与终态事件 | HTTP 字段、Key、机械臂 prompt/硬件能力 |
| pi-coding-agent 的应用角色 | `applications/agent/` | 机械臂 prompt、能力描述、技能绑定、输入与 CLI、配置选择 | 自建模型协议、绕过控制器门禁 |

继续保持六个一级能力目录。`policy/inference/` 仍专用于既有 VLA 模型与动作块，
不与聊天消息/事件合并；`model_io` 是决策模型 I/O，不声称已统一所有模态或 VLA。

依赖方向：application → agents → model_io；application 也可以装配 model_io 适配器。
model_io 不反向导入 agents；agents 不导入应用、HTTP 适配器、硬件或机器人控制模块。
供应商格式只在 model_io 的适配器内出现。prompt 在应用包内，随 wheel 交付。

## 本轮实现目标

- 模型接口：规范化 Message/ToolDefinition，完整回复与 SSE 流共用最终回复契约；
  工具参数流片段只用于组装，完整结束前不可派发。环境凭据由专用 resolver 读取，不复制秘密到配置。
- 核心：有界的跨轮对话记忆、每次请求刷新应用上下文、统一事件、工具上下文与可取消进度；
  工具执行移出会话状态锁，停止能立即使代次失效。拒绝旧结果，先前任务不自动重放。
- 应用：注入机器人能力与 prompt，保持文本/ASR 与提供方解耦。当前技能仍为 dry-run；
  长时控制由应用技能服务持有任务，启动返回 task_id，完成以实际反馈为准。

这些目标随本轮源码与测试落实；实际命令、支持范围见[运行指南](agent_runtime.md)。
不会因采用三层而新增无实物证据的运动能力，也不自动安装/发现本机插件或服务。

## 对话、观测、任务分开

对话记忆用于语义延续；当前机器人观测必须重新获取并检查年龄。
跨轮只保留已完成回合的对话，不保存可重新执行的工具队列；取消/失败回合不作为可续跑计划。
应用上下文是带本机采集时间与有效期的快照，每次模型请求重新获取；过期或未来时间拒绝。
静态能力声明不是实测关节反馈。历史模型文本中的位置也不能充当新观测。

工具调用携带 session、generation、deadline、cancel 与进度回调；最终写入门禁仍由执行器维护。
核心的终止、工具任务完成和机械臂物理停止分别有各自证据，不互相代替。

## 与 pi 相同的设计思想及领域差异

参考[模型类型](https://github.com/earendil-works/pi/blob/d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03/packages/ai/src/types.ts)、
[Agent 类型](https://github.com/earendil-works/pi/blob/d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03/packages/agent/src/types.ts)、
[工具执行循环](https://github.com/earendil-works/pi/blob/d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03/packages/agent/src/agent-loop.ts)与
[应用 SDK](https://github.com/earendil-works/pi/blob/d2e7d8b7c1d31d3661f769d5caef7ba8d50c3e03/packages/coding-agent/src/core/sdk.ts)。

借鉴规范化消息、工具生命周期、取消信号和应用装配。当前不迁入 TUI、文件编辑、shell 工具、
自动恢复/重试、树状会话和插件发现。普通运动保持互斥；停止走本地旁路，不等待 steering 队列。
会话恢复可以恢复对话，但不能重放曾经执行的机械臂动作。路由能力与本机门禁分别维护。
