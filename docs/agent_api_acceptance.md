# 聊天 API 接入与验收

2026-10-24。先验证聊天与工具循环，再接云端 ASR/TTS。配置能加载、离线协议测试通过、
真实服务通过和实物任务完成是四种证据。预设名称不能替代真实模型验收。

## 操作入口

先按[配置指南](agent_providers.md)生成一个用户本地 TOML 和 .env，填写模型 ID 与 Key。
`check` 仅离线检查；`probe` 才会向配置中的地址发送真实请求，可能产生费用。
不发现其他项目的凭据，不自动选择模型、切换区域、重试或降级。

```sh
# 最小连通验收：只发一个 JSON 文本请求
uv run --locked eai-course agent-config probe \
  --provider-config configs/agent/qwen.local.toml \
  --env-file configs/agent/.env.qwen.local --transport json --case text

# 同一配置验证 JSON/SSE、文本、多轮、工具结果回传及工具后的新回合
uv run --locked eai-course agent-config probe \
  --provider-config configs/agent/qwen.local.toml \
  --env-file configs/agent/.env.qwen.local --transport both

# SSE 首个实际文本片段到达后取消，检查本地返回；不代表停止远端计算/计费
uv run --locked eai-course agent-config probe \
  --provider-config configs/agent/qwen.local.toml \
  --env-file configs/agent/.env.qwen.local --transport sse --case cancel
```

默认运行 `text`、`conversation`、`tools`，可重复指定 `--case`。
每个案例独立建立会话、关闭 provider，任何失败仍保留，并继续其他案例。
默认两种传输合计最多 18 次模型请求/配置；通常成功路径为 12 次。
每次请求遵守配置中的 token/timeout 预算，每个回合最多 3 轮、1 个工具调用；不自动重试。
建议先用单个 text 案例确认账户/区域，再完整验收。普通 `agent` 不会自动运行探针。

可重复指定 `--provider-config` 批量验证服务。同一个显式 .env 可存放不同变量名的 Key，
每份配置只解析自己选中的变量，进程环境优先。例如：

```sh
uv run --locked eai-course agent-config probe \
  --provider-config configs/agent/qwen.local.toml \
  --provider-config configs/agent/deepseek.local.toml \
  --env-file configs/agent/.env.providers.local --transport both
```

`--transport configured` 保持原始流设置；`json/sse/both` 显式覆盖传输。
JSON 测试去掉只适用于流的 `stream_options`，其余供应商参数保持原样，不修改配置文件。
`--strict-tools` / `--no-strict-tools` 分别发送服务端 strict=true/false；默认省略。
本地 schema 校验始终启用。cancel 在 JSON 模式标为 skipped；只有 skipped 的运行不能通过。

## 判据与报告

| 案例 | 通过判据 |
|---|---|
| text | 模型返回本次生成的指定文本 |
| conversation | 第二回合准确复述上回合答案，使用生产会话历史 |
| tools | 模型用标准函数定义发起一次只读查询；答案必须来自仅存在于工具结果的随机值；下一回合仍能对话且不再次调用工具 |
| cancel | 实际 SSE 文本到达后触发取消，会话返回 cancelled；记录本地取消等待时间 |

工具仅为 `lookup_probe`，不会加载机器人技能、打开串口、相机或麦克风。
工具案例验证实际模型选择、参数解析、handler、结果回传和后续推理，不通过搜索答案中的
“已调用工具”判断成功。它不替代作业 4.2 的三个机器人技能演示。
失败案例不会自动改参数再试；用户修改配置后是一次新的验收。

stdout 为一个 JSON 报告；退出码 0 表示所选案例无失败且至少一个通过，失败为 1。
报告包含 UTC 时间、配置序号/SHA256、模型/协议/SDK 版本、各项判据、耗时、首文本延迟、
取消等待时间、模型/工具调用数以及服务返回的扁平 usage。没有 usage 时保留空对象，不能推导为零成本。
报告不包含 Key、完整 URL、提示词、转写、模型正文、推理内容或原始错误回包。
配置 SHA 不包含 .env 内容；配置变化后应重新验收。完整模型版本由服务控制，未知时保持 unknown。

首文本延迟含本机 SDK/连接开销；cancel_wait_ms 是本地等待时间，不是机械臂停止时间。
单次探针不是 p50/p95 基准。流式片段仍必须等待正常终态才能执行工具；收到文本不代表工具已完成。
`provider.capabilities_verified=false` 不会因一次探针改写为全局 true；报告仅证明选中模型、参数、案例和时刻。

| 错误码 | 排查方向 |
|---|---|
| missing_credential | 所选变量是否在进程环境或显式 .env 中有值 |
| authentication | Key、账户权限、平台和区域是否匹配 |
| endpoint_or_model_not_found | base_url、API 类型和模型 ID；不据 404 猜测具体原因 |
| invalid_request | 模型是否支持所选参数、工具、strict 或模态 |
| rate_limited / service_unavailable | 配额、限流、服务可用性；不会自动重试 |
| transport_error / timeout | 网络、连接和预算 |
| protocol_error / incomplete_response | 不完整流、截断、非法工具参数或协议差异 |
| tool_not_used / unexpected_answer / conversation_mismatch | 模型行为不满足案例判据；不能写成 HTTP 不通 |

## 服务与场景的适配边界

千问/百炼、DeepSeek、Kimi、MiniMax、OpenAI 与小众兼容服务沿用同一 SDK 适配器，
通过独立 profile 区分平台/区域/模型/思考模式，避免按品牌复制循环。
Chat Completions 与 Responses 是不同协议，需各自验收；原生私有协议尚未支持。
第三方网关、团队自建服务和本机服务可用 custom 配置，但报告必须分别记录来源，不能相互冒充。

思考模式参数依模型与平台而异，通过 `model_options` 或 `extra_body` 显式填写。
已完成回合保留最终 assistant 元数据（含服务返回的 reasoning_content），并计入历史预算；
当前回合的工具 assistant/结果正常回传，已完成回合的工具队列不会带入新回合。
取消或失败回合不进入历史。独立 reasoning_content 不作为文本流输出；
供应商若将思考内容混在 content 中，本层不会猜测并删除标签，需要按该模型配置分离输出
（如支持的 MiniMax 模型使用 reasoning_split），再运行探针；不能直接拿它作为未来 TTS 输入。
Responses 的 opaque 输出仍仅用于当前工具回合，不持久化或重放为新动作。

官方依据：[OpenAI 工具结果回传](https://developers.openai.com/api/docs/guides/function-calling)、
[DeepSeek 思考与多轮工具要求](https://api-docs.deepseek.com/guides/thinking_mode/)、
[百炼工具调用及模型参数](https://www.alibabacloud.com/help/en/model-studio/qwen-function-calling)、
[MiniMax OpenAI 兼容接口](https://platform.minimax.io/docs/api-reference/text-openai-api)。
端点预设和账户配置见[多服务配置](agent_providers.md)；本轮离线测试不代表这些供应商已真实验收。
