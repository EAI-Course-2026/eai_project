# 三层 Agent 运行指南

2026-10-22。产品目标、硬件边界、评测见[项目定位](project_scope.md)，分层与 pi 参考见
[三层架构](agent_layers.md)。模型接入复用官方 OpenAI SDK，支持 Chat Completions / Responses 的完整回复、SSE 与工具循环。
标准图像内容块可通过 Python 接口传入，Chat Completions 也可传 input_audio。采集设备、
ASR/TTS、实时 Omni、IK/跟踪工具和实物执行仍待接入。标准范围见[复用决策](agent_standard_contract.md)。

## 三层代码

| 层 | 实现 | 边界 |
|---|---|---|
| 模型 I/O | `policy/model_io/contracts.py`、`providers.py`、`auth.py` | SDK 消息/工具类型、Chat/Responses SDK 接入、逐请求环境凭据 |
| Agent 核心 | `policy/agents/context.py`、`contracts.py`、`tools.py`、`loop.py` | 有界对话、短时应用上下文、工具策略/进度、预算、取消/代次、结构化事件 |
| 机械臂应用 | `applications/agent/cli.py`、`session.py`、`skills.py`、`capabilities.py` | 输入/配置装配、应用 prompt、能力限制与演练技能 |

prompt 位于 `applications/agent/prompts/system.txt`，随 wheel 安装。
模型层不导入 Agent；核心不导入 HTTP、凭据、机器人或应用；应用注入具体实现。
`Message` 和 `ToolDefinition` 是官方 SDK 类型别名，不存在第二套消息/工具字段。
`policy/inference/` 继续承担既有 VLA 动作块推理，保持独立输出契约。
本项目没有 pi 的运行依赖或本机 checkout 依赖。

## 离线演示与模型配置

```sh
uv run --locked eai-course agent --list-tools  # 输出实际标准工具定义，不调用模型
uv run --locked eai-course agent --demo
```

固定 fixture 调用 get_robot_state、list_poses、move_to_pose(stand)、get_task_status，明确不调用模型。
当前所有姿态动作均是 dry-run，真实反馈为 null；默认参考姿态使用自身标定绑定校验。
`--poses FILE` 替换姿态，`--calibration FILE` 选择目标标定作离线换算，不读取个人串口配置。

复制 `configs/agent/provider.example.toml` 为 `configs/agent/provider.local.toml`，
或在任意用户目录建立配置，填写 API base_url/model：

```toml
[provider]
adapter = "chat_completions"
execution = "managed_api"
base_url = "https://YOUR_SERVICE_HOST/v1"
model = "YOUR_MODEL_ID"
credential_env = "ROBOT_LLM_API_KEY"
timeout_s = 30.0
max_tokens = 512
streaming = true
token_limit_parameter = "max_completion_tokens"
```

**Key 的值放在 `ROBOT_LLM_API_KEY` 环境变量中，不放 TOML。**
`credential_env` 可换为其他合法环境变量名；resolver 每次请求读取它，便于轮换。
程序不自动读取 `.env`，不扫描其他项目/用户目录凭据。配置示例的 `example.invalid` 不是默认服务。
以 zsh 为例，通过隐藏输入注入（不要把真实 Key 写入命令或聊天）：

```sh
read -s 'ROBOT_LLM_API_KEY?API Key: '
export ROBOT_LLM_API_KEY
uv run --locked eai-course agent \
  --provider-config configs/agent/provider.local.toml --text '先查看状态，再演练站立姿态'
unset ROBOT_LLM_API_KEY
```

其他 shell 使用自己的环境注入方式；CLI/TOML 不变。`configs/agent/*.local.toml/json` 已忽略。
本机服务参考 `local_service.example.toml`，无认证可省略 credential_env。
自有服务用 execution=self_hosted；网络只允许 HTTPS 或 SSH 转发/本机的 loopback HTTP。
不跟随重定向，不自动重试动作或切换服务。供应商必须实际支持文本工具协议。

选择 Responses API 时使用 `configs/agent/responses.example.toml`，设置 `adapter="responses"`。
Responses-only 模型必须选此路径。`max_tokens` 是本地预算字段：Chat Completions 按
`token_limit_parameter` 发为 max_completion_tokens（默认）或兼容服务要求的 max_tokens；
Responses 发为 max_output_tokens。旧完整 endpoint 配置仍可迁移使用，不与 base_url 同时填写。
模型参数可放到 `[provider.model_options]`，例如 Chat 的 reasoning_effort 或 Responses 的 reasoning；
程序校验所选 API 的选项，不自动猜测或切换。

SSE 显式设 `streaming=true`。SDK 负责 HTTP/SSE 和分片累积，文本增量作为 model_delta 输出；
只有最终正常完成、参数 JSON 与本地 schema 合法的调用才交给工具。没有完整 finish/terminal
结果的流拒绝执行。Responses 保留当前工具回合的完整 output/reasoning，再与 call_id 输出回传；
采用 store=false。model_end 事件包含 SDK usage，缺失计数保持缺失，不估算成本。

SDK retries=0，拒绝重定向；保留请求/响应大小和期限限制、单 I/O worker 与凭据脱敏。
程序复用官方 SDK，不保证任意兼容服务支持任意模型参数或模态。真实服务与模型必须独立验收。
详见[标准契约及兼容矩阵](agent_standard_contract.md)。

Python 接口直接接受标准内容块，例如：

```python
session.run_turn([
    {"type": "text", "text": "描述看到的目标，不要运动"},
    {"type": "image_url", "image_url": {"url": supplied_image_data_url, "detail": "low"}},
])
```

图像须由调用者显式提供；模块不读取本机摄像头或扫描文件。内容块也受预算限制。
Responses 路径不支持音频输入，会明确返回 unsupported_modality，不静默转文字。

## 可替换语音输入与应用边界

任意识别客户端输出 UTF-8 的一行**最终转写**，接入：

```sh
uv run --locked eai-course agent --provider-config configs/agent/provider.local.toml --stdin
```

上游可为本机模型、云服务客户端或其他设备桥接；`edge_inference_lab` 只作历史案例。
本仓库不导入/启动它，也不依赖特定路径。聊天配置只选择聊天服务，ASR 凭据与机器人硬件配置独立。
`voice --stdin` 保留确定性白名单命令；`agent --stdin` 让模型选择应用注册的工具。
停止词复用原解析器，直接触发本地 cancel；忙时的新普通输入返回 busy，不排队重放。
EOF 等待本轮结束，Ctrl-C 先取消再等待退出。提供方失败后不接受新的行来恢复旧计划。

纯文本通道没有置信度、时间戳或 partial/final 标记，上游必须只发送最终文本。
结构化识别事件、用户本地 ASR 选择、麦克风/TTS 管理仍是[语音架构](course/voice/ARCHITECTURE.md)的后续目标。

## 对话、工具与能力

默认保留最近 8 个已完成对话回合，最多 32768 字符，按完整 user/final-assistant 对裁剪。
失败/取消回合不提交；工具结果与工具调用不进入下一用户回合。不会持久化或恢复动作队列。
`clear_history()` 只允许在空闲时清理对话。历史回答用于指代，不能代替当前观测。
应用能力快照每次模型请求刷新并检查本机时间/有效期；实际设备反馈仍需重新获取。

五个应用工具保持 dry-run：get_robot_state、list_poses、move_to_pose、get_task_status、stop_motion。
RobotCapabilities 选择可用工具，并在正常执行前再次检查；不支持的工具不向模型声明。
能力声明与实测反馈分开：负载、真实工作空间、精度未知，不从 LLM 文本推导，也不能开启 physical_execution。
姿态计划 targets 是 0..1 归一化值，不是角度，hardware_sent 始终 false；保留最近 128 个模拟任务供查询。

核心只接受注入的工具。handler 形如 `handler(arguments, ToolContext) -> ToolResult`，
context 提供 session、generation、deadline、cancel、check_active() 与 progress(data)。
参数校验后执行 before_tool（可拒绝），执行后 after_tool 只作观测。
工具声明直接使用 OpenAI 标准 function 字典，也接受 `openai.pydantic_function_tool` 的输出。
本地由 jsonschema 校验整数/数值、布尔、数组、嵌套、nullable、组合与本地引用；不获取外部引用。
strict 的服务端支持范围以所选模型/API 为准，本地不自动重写 schema 或放宽约束。
长时控制技能应由应用服务拥有任务，启动后返回 accepted/task_id，由状态查询确认完成。
有效 interrupt 工具直接调用应用注入的取消回调，优先于同一批普通动作；核心不硬编码 stop_motion。

## 生命周期与取消

事件顺序为 run_start → model_start/delta/end → tool_start/progress/end → 更多模型轮次 → run_end → run_settled。
每个事件含单调 sequence、session_id、generation、run_id、本机 monotonic 秒 at、data；CLI stdout 使用完整 JSONL，诊断在 stderr。
默认不输出原始用户转写、endpoint、Key 或供应商错误体；model_delta 是明确的模型文本输出。
事件监听器用于观测，应快速返回；抛出的普通异常计入 observer_errors，不得改变工具策略。
策略检查在 before_tool，不放到输出监听器中。

每轮默认最多 8 次模型请求、16 次工具调用、60 秒；模型请求默认 30 秒。这些不替代控制线程租期。
新回合/取消更新 generation，旧响应不能执行工具或提交对话。
模型请求和工具分别最多一个工作线程；工具移出状态锁，取消回调可直接请求应用停止。
核心不会强杀 Python handler：取消/超时先返回并使上下文失效，未退出的旧工具进入 draining，
新回合返回 tool_still_running，直到旧工具退出。进度回调只在本次工具执行期间有效。

run_end 表示本轮编排结束；run_settled 表示本地编排与核心工具 worker 已排空，不会自动续跑。
`wait_for_idle(timeout_s)` 可等待本地核心排空，监听器处理与远端计算不在该条件内。
它不代表远端模型停止计费，也不代表应用后台任务/机械臂物理停止或原任务成功。
实际执行器仍须在写前检查当次授权、期限、反馈与取消；所有物理能力仍待实物验收。
用户手动恢复对话不能重放先前动作。

## 验证范围与迁移

离线测试使用真实 loopback HTTP 的 JSON/SSE 预设响应，验证分片、取消、上下文、能力和依赖边界。
它与 demo 都不能代替真实供应商 LLM、视觉语音或实物验收。
没有实现 pi 的完整交互/持久化恢复；取消后不恢复动作，待反馈对账的动作也不能自动重放。
Python 内部迁移对照见[源码迁移](source_layout_migration.md)，CLI 入口保持兼容。
