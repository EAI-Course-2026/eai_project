# Agent 标准契约与复用决策

2026-10-22。取代首版的自定义 Message/ToolDefinition 和手写 HTTP/SSE 实现。
本项目使用 OpenAI 标准 API 与官方 Python SDK，不实现另一套工具协议，也不依赖 pi checkout。

## 哪些直接复用

| 能力 | 实际实现 |
|---|---|
| 消息与函数定义 | 官方 SDK `ChatCompletionMessageParam` / `ChatCompletionFunctionToolParam`；标准字典直接贯穿工具注册、会话和 Chat Completions 请求 |
| Python 类型生成 schema | 可直接注册 `openai.pydantic_function_tool(...)` 的结果，保留 `$defs`、`$ref` 和 `strict` |
| 参数校验 | `jsonschema.Draft202012Validator`，支持数字、整数、布尔、数组、嵌套、nullable、组合和本地引用；不自行维护类型校验器 |
| HTTP、SSE、流式参数累积 | 官方 `OpenAI.chat.completions.create/stream` 和 `OpenAI.responses.create/stream`；httpx 只用于字节/期限限制及拒绝重定向 |
| 流最终结果 | SDK `get_final_completion` / `get_final_response`；本地额外要求正常完成、有效参数、唯一调用和预算，片段不能执行 |
| 锁定依赖 | 根控制环境的 `pyproject.toml` / `uv.lock`；训练环境独立，不为 Agent 添加依赖 |

`Message` / `ToolDefinition` 现在只是 SDK 类型别名。`ChatRequest` / `ChatReply` / `ModelEvent`
是本地身份、预算、事件和提供方 continuation 的信封，不是新网络格式。
`Tool(definition, handler, interrupt=False)` 中只有 definition 发给模型；handler、取消和权限都在本地。

标准工具示例（可原样交给 OpenAI Chat Completions）：

```python
spec = {
    "type": "function",
    "function": {
        "name": "start_tracking",
        "description": "启动已验证的目标追踪技能。",
        "parameters": {
            "type": "object",
            "properties": {"object_id": {"type": "integer", "minimum": 0}},
            "required": ["object_id"],
            "additionalProperties": False,
        },
        "strict": True,
    },
}
# Tool(spec, actual_tracking_handler) — 示例，不表示当前已实现该技能。
```

`strict` 的服务端约束使用 OpenAI 支持的 JSON Schema 子集；本地 Draft 2020-12 校验能力更广，
不代表任意 schema 都会被任意模型接受。应用默认省略服务端 strict，可显式选择；注册层不重写传入的 strict、不把数字转成字符串，也不为兼容放宽限位。
外部 schema 引用不会联网读取；需要的定义放在本地 `$defs`。

## 两种官方 API

Chat Completions 保持标准嵌套 function 定义与 tool_call_id 回传。
Responses 使用官方扁平 function 定义、input_image、function_call_output/call_id。
这些差异只在 `model_io/responses.py` 转换；工具的唯一声明仍是标准 Chat Completions 字典。
Responses 采用 `store=false`，在当前工具回合保留完整 output，包括 encrypted reasoning，
并按顺序与工具输出一起回传。不会只保留模型文本而丢失推理模型续轮所需的内容。

| 路径 | 已实现输入 | 输出与流 | 验证证据 |
|---|---|---|---|
| Chat Completions | 文本、image_url、input_audio 标准内容块 | 文本、函数调用、SSE、usage | 真实 loopback HTTP/SSE + SDK 测试 |
| Responses | 文本、图像；音频明确拒绝 | 文本、函数调用、reasoning 续轮、SSE、usage | 真实 loopback HTTP/SSE + SDK 测试 |
| 麦克风/相机采集、实时 Omni | 尚未接入 | 尚未接入 | 不计为完成 |
| 真实供应商/具体模型/实物 | 需操作者配置与单独验收 | 不做通用兼容声明 | 本轮未访问 |

模型 ID 不硬编码；操作者显式选择 API、base_url、model、凭据和模型参数。
`OpenAI-compatible` 不等于所有服务或模型能力相同；Responses-only 模型不能放到 Chat Completions。
不自动降级、重试工具轮或切换模型。模型扩展可通过 SDK 原生 extra_body 显式配置；不覆盖路由/工具/预算。
完整 assistant（含 reasoning_content）保留当前工具续轮；字段含义依所选服务文档。
2026-10-24：完成后的最终 assistant 元数据也保留在有界对话历史中，修复多轮思考模型的
字段丢失；旧工具调用仍不跨轮重放。实际配置/模型可用 `agent-config probe` 验证，见
[API 验收](agent_api_acceptance.md)。
各服务预设、dotenv 与配置命令见[多服务 API 配置](agent_providers.md)。

## 核心保留什么

核心负责机械臂应用需要的序列化执行、时间/调用预算、取消代次、排空、上下文新鲜度、
执行前策略和事件。执行授权、反馈、运动租期最终由本地控制器维护。
工具循环不把本地 SDK 事件或部分 JSON 当成实际动作；停止优先，旧工具退出前禁止重叠。

本轮没有实现 pi 的 TUI、树状持久会话、压缩、steering 或崩溃恢复，也不以手写替代品凑齐功能。
恢复动作需要 durable intent、幂等语义和物理状态对账，不能把通用聊天历史恢复直接用于机械臂重放。
当前只保留有界的已完成对话；opaque reasoning/工具队列限于本轮，取消/失败后不会恢复执行。
后续评估通用框架时，以取消/重放/执行器门禁测试为准，不只看接口名称是否相似。

官方依据：[Function calling](https://developers.openai.com/api/docs/guides/function-calling)、
[Chat Completions](https://developers.openai.com/api/reference/resources/chat/subresources/completions/methods/create)。
安装版本与最终兼容回归回执见锁文件和本轮 PR。
