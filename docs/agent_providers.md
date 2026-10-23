# 多服务 API 配置

2026-10-22。统一采用官方 OpenAI SDK，按协议选择 Chat Completions 或 Responses。
提供方预设只是可修改的连接配置，不是模型能力白名单或真实服务验收结论。

## 一次生成配置与本机密钥文件

```sh
uv run --locked eai-course agent-config list
uv run --locked eai-course agent-config init \
  --preset qwen --model YOUR_MODEL_ID \
  --output configs/agent/qwen.local.toml
```

同一命令也可写成 `uv run --locked python scripts/configure_agent.py init ...`。
生成 `configs/agent/qwen.local.toml` 和 `configs/agent/.env.qwen.local`。
配置只存环境变量名；在生成的 .env 文件中填写实际 Key。文件只在本机保存、默认被 Git 忽略，
POSIX 下创建权限为 0600。命令拒绝覆盖已有文件；不自动寻找、复制其他项目凭据或询问远端模型。

```sh
# 只检查配置与所选 Key 是否可在本机解析；不请求 API、不计费，也不证明 Key/模型有效。
uv run --locked eai-course agent-config check \
  --provider-config configs/agent/qwen.local.toml \
  --env-file configs/agent/.env.qwen.local

# 这条才会请求操作者选择的真实模型。当前机器人技能仍为 dry-run。
uv run --locked eai-course agent \
  --provider-config configs/agent/qwen.local.toml \
  --env-file configs/agent/.env.qwen.local --text '先查看状态，再列出演练姿态'
```

`--env-file` 必须显式提供；没有自动加载 `.env` 或目录搜索。进程环境变量优先于文件，
文件中的 `${...}` 不插值、不执行 shell，也不导出到进程全局。每次请求重新读取所选进程环境变量；
修改 .env 文件后需重启会话。可参考 `configs/agent/providers.env.example` 管理多个服务的独立 Key。

## 预设与模型/平台区别

| 预设 | 接入平台/区域 | Key 环境变量 | 说明 |
|---|---|---|---|
| qwen、bailian、bailian-beijing | 百炼北京 | DASHSCOPE_API_KEY | 前两者是别名；千问是模型系列，百炼是平台 |
| bailian-singapore | 百炼新加坡 | DASHSCOPE_API_KEY | Key 必须与区域匹配 |
| bailian-virginia、bailian-hongkong | 百炼对应区域 | DASHSCOPE_API_KEY | 可用独立变量名保存各区域 Key |
| deepseek | DeepSeek 官方 API | DEEPSEEK_API_KEY | 使用操作者选择的模型 ID |
| kimi、moonshot | Kimi 官方 API | MOONSHOT_API_KEY | 可按模型选择 Chat 或 Responses |
| minimax | MiniMax 国际 API | MINIMAX_API_KEY | 其他区域使用控制台给出的 base_url 覆盖 |
| openai | OpenAI API | OPENAI_API_KEY | Responses-only 模型需 `--adapter responses` |
| custom | 自选兼容服务/网关/本地服务 | ROBOT_LLM_API_KEY | 必须显式提供 base_url |

预设 URL 与官方文档来源保存在随 wheel 交付的 `applications/agent/provider_presets.json`。
不自动选择“最新模型”。同一个模型系列可以由不同平台托管：例如通过百炼调用 DeepSeek 时，
选择百炼预设、百炼 Key 和其模型 ID，不能换用 DeepSeek 官方 Key。
百炼支持工作空间专用域名，建议用控制台给出的 URL 覆盖共享域名预设。

```sh
# DeepSeek / Kimi / MiniMax 只需替换 preset 与实际 model；每个配置独立。
uv run --locked eai-course agent-config init --preset deepseek --model YOUR_MODEL_ID \
  --output configs/agent/deepseek.local.toml

# 区域/工作空间/代理可覆盖，不需要新增 Python 提供方类。
uv run --locked eai-course agent-config init --preset bailian-singapore --model YOUR_MODEL_ID \
  --base-url https://YOUR_WORKSPACE.ap-southeast-1.maas.aliyuncs.com/compatible-mode/v1 \
  --credential-env DASHSCOPE_SINGAPORE_API_KEY --output configs/agent/bailian-sg.local.toml

# 小众服务，只要实现标准兼容协议即可使用。
uv run --locked eai-course agent-config init --preset custom --model YOUR_MODEL_ID \
  --base-url https://YOUR_SERVICE/v1 --credential-env MY_SERVICE_API_KEY \
  --output configs/agent/custom.local.toml
```

本地无认证服务可加 `--execution local_process --no-auth` 并使用 loopback HTTP；
不创建 .env 文件。网络服务要求 HTTPS。原生 Anthropic、DashScope 或非兼容 API 需要相应适配器，
不能只改 URL 冒充已支持。本轮不自动降级、切换端点/区域、重试或安装推理服务。

## 模型差异显式配置

`init` 默认输出 Chat Completions 配置；`--adapter responses` 仅在服务/模型支持该协议时选择。
`--streaming/--no-streaming`、`--max-tokens` 与 `--token-limit-parameter` 可在生成时指定。
旧 token 参数与新参数都通过官方 SDK 发送；Responses 统一使用 max_output_tokens。
思考模型需要足够输出预算与请求时间，过小会截断；截断不会被当作成功或执行工具。

标准参数放在 model_options；供应商扩展通过 SDK 原生 extra_body，保持本地控制字段不可覆盖：

```toml
[provider.model_options]
reasoning_effort = "low"  # 仅在所选模型支持时启用

[provider.extra_body.thinking]
type = "enabled"        # 示例：不是所有服务/模型都支持或需要
```

千问的 `enable_thinking`、其他服务的 `thinking` / `reasoning_split` 等也可用 extra_body，
但具体字段和值以所选模型官方文档为准。不能覆盖 messages/input、tools、stream、模型、
路由、token 预算、store/previous_response_id；标准 model_options 字段也不能在 extra_body 重复定义。
完整 assistant（包括 reasoning_content / think 内容）在当前工具回合保留并回传，不只保留最终文字。

兼容服务对服务端 strict 支持不同。应用默认省略 strict，显式 `--strict-tools` 或
`--no-strict-tools` 可选择发送 true/false；本地 JSON Schema 校验与执行门禁始终生效。
注册层不会改写外部传入的标准定义，也不会遇到错误后悄悄改 schema 重试。

## 核验来源与证据

端点和接入形态核对于 2026-10-22，来源为[百炼区域端点](https://www.alibabacloud.com/help/en/model-studio/base-url)、
[百炼兼容接口](https://www.alibabacloud.com/help/en/model-studio/compatibility-of-openai-with-dashscope)、
[DeepSeek](https://api-docs.deepseek.com/)、[Kimi](https://platform.kimi.com/docs/get-api-key)、
[MiniMax OpenAI SDK](https://platform.minimax.io/docs/api-reference/text-openai-api)。
预设不是持续更新的模型目录。使用前以自己控制台的账户、区域、计费方案和可用模型为准。
离线检查/loopback 回归与真实供应商的工具能力、延迟、配额和实物任务验收分别记录。
