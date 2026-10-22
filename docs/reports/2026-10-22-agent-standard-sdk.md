# Agent 标准协议与官方 SDK 修正

2026-10-22。基于 develop `2eb8470`，分支 `codex/integration-standard-agent`。
本报告修正首版 Agent 的工程边界：[三层报告](2026-10-19-agent-layers.md)里的自定义消息/工具类型、
手写 HTTP/SSE 与字符串参数子集已由成熟库替代。旧报告保留原阶段事实，不作为当前接口说明。

## 改动

工具声明直接采用 OpenAI 标准 function 字典；Message/ToolDefinition 是官方 SDK 类型别名。
注册层仅绑定 handler 与本地执行元数据，定义不经自定义 schema 转译。
支持直接注册官方 pydantic_function_tool 的输出，由 jsonschema 做实际参数检查。

Chat Completions 与新增 Responses 路径均复用官方 OpenAI Python SDK 的 HTTP、SSE、
文本增量和工具参数累积。Responses 当前回合保留完整 output/reasoning 与 call_id 工具输出，
采用 store=false；不将这些状态作为可重放的跨轮计划保存。
两种官方 API 的必要格式转换仅在适配器中完成，标准工具声明只有一份。

增加 SDK base_url 配置、模型选项、token 字段选择、usage 事件及 --list-tools。
图像标准内容块可由调用者显式传入，Chat 还可传 input_audio；采集与实时语音不在其中。
SDK retries=0，保留请求/响应上限、预算、取消与排空、反馈/授权的本地边界。
不把 pi 的完整交互和持久恢复声称为已实现，也不引入无法对账的动作恢复。

后续多提供方配置和最终验证见[配置报告](2026-10-22-agent-provider-configuration.md)。

## 证据与限制

- 首个 SDK 修正提交 `4f664b3` 本地完整 543 项测试通过，Agent 专项 44 项；新增 14 项标准兼容性回归。
- 覆盖官方 Pydantic schema 原样发送、数字/布尔/嵌套/数组/nullable、本地引用、外部引用拒绝、
  多工具交错流与 usage、两条 API 的工具闭环、Responses reasoning 回传与流终态、模态限制和取消隔离。
- 锁定同步、环境检查、文档/仓库检查、JavaScript、两个 wheel 构建通过。
- 解包 wheel 核对源码/prompt 字节，独立目录运行 demo 和标准工具导出通过。
- 仅根控制环境添加 OpenAI/jsonschema/Pydantic/httpx 依赖；训练锁文件未改，锁定检查通过。

兼容回归使用真实 loopback HTTP 与预设响应，不是实际模型推理。真实供应商/具体模型仍需
操作者配置与单独验收；未调用真实模型、摄像头/麦克风或硬件串口。现有五个技能仍为 dry-run，
没有通过空函数冒充视觉追踪/IK 接入或课程 4.2/4.3 验收。

托管 CI 的准确 head、三项检查与运行链接以本轮 PR 为准。尚未合并，无本轮合并后 CI。
当前标准与迁移见[标准契约](../agent_standard_contract.md)、[运行指南](../agent_runtime.md)。
