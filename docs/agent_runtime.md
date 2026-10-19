# Agent 运行入口（文本与 dry-run 工具）

2026-10-19。现已建立可运行的 `policy/agents/` 与 `applications/agent/`。
本版支持真实 HTTP Chat Completions 调用、工具结果回传和离线固定演示；
工具只生成演练计划，不提供连接串口、启用扭矩或实物执行选项。
视觉/音频输入、ASR/TTS、Omni、物体检测/跟踪和 IK 工具仍待后续接入。

## 目录与职责

| 位置 | 已实现 |
|---|---|
| `policy/agents/contracts.py` | ChatRequest/Reply、ToolCall/Result、ChatProvider 协议与脱敏错误 |
| `policy/agents/providers.py` | 非流式、文本/工具 Chat Completions HTTP；不加载模型或厂商 SDK |
| `policy/agents/tools.py` | 工具注册与参数校验；首版仅支持封闭对象中的字符串参数，不声称实现完整 JSON Schema |
| `policy/agents/loop.py` | 调用循环、结果回传、预算、重复 ID 拒绝、停止代次与迟到响应作废 |
| `policy/agents/prompts/system.txt` | 随 wheel 安装的核心 prompt |
| `applications/agent/skills.py` | 注入的姿态 dry-run 技能，不拥有设备 |
| `applications/agent/session.py` | stdin 读线程与推理分开，识别到停止词直接本地抢占 |
| `applications/agent/cli.py`、`demo.py` | 配置装配、命令入口与明确标记的离线 fixture |

这些模块不导入 `edge_inference_lab` 或特定机器路径。后续识别端通过最终转写文本接入，
不能把外部识别器自带的命令映射作为机器人的权威动作策略。

## 无模型、无设备演示

在仓库根目录：

```sh
uv run --locked eai-course agent --demo
```

固定演示依次调用 `get_robot_state`、`list_poses`、`move_to_pose(stand)`、`get_task_status`。
它不是 LLM 推理，也不能作为课程要求的外部 API 验收。状态输出明确是 dry-run，真实反馈为 null。
默认姿态是随包提供的参考姿态，使用姿态文件自身的标定绑定校验；不读取个人硬件配置。
`--poses FILE` 可替换姿态文件；`--calibration FILE` 可选择目标标定进行离线换算检查。

## 配置自选模型 API

复制 `configs/agent/provider.example.toml` 为 `configs/agent/provider.local.toml`，
填写完整 endpoint 和 model。配置示例使用保留的 `example.invalid`，不会默认调用任何服务。

```toml
[provider]
adapter = "chat_completions"
execution = "managed_api"
endpoint = "https://YOUR_SERVICE_HOST/v1/chat/completions"
model = "YOUR_MODEL_ID"
credential_env = "ROBOT_LLM_API_KEY"
timeout_s = 30.0
max_tokens = 512
```

**API Key 的值放在 `ROBOT_LLM_API_KEY` 环境变量中，不放 TOML。**
通过终端的隐藏输入或现有 secret manager 注入当前进程环境；程序不会自动读取 `.env`，
也不会扫描其他项目的凭据或配置。`credential_env` 可以改为任意合法的环境变量名。
以 zsh 的隐藏输入为例（不要把真实 Key 写入命令或聊天）：

```sh
read -s 'ROBOT_LLM_API_KEY?API Key: '
export ROBOT_LLM_API_KEY
uv run --locked eai-course agent \
  --provider-config configs/agent/provider.local.toml --text '先查看状态，再演练站立姿态'
unset ROBOT_LLM_API_KEY
```

PowerShell 或其他 shell 使用各自的安全环境注入方式；TOML 和 CLI 参数相同。
配置文件可以放在用户自己选择的位置，通过 `--provider-config FILE` 指定；没有固定的个人目录。
`configs/agent/*.local.toml`、`*.local.json` 已被忽略。

本机模型服务参考 `configs/agent/local_service.example.toml`。无认证时省略 credential_env；
自有远端服务用 `execution="self_hosted"`，选择 HTTPS 或经 SSH 转发的 loopback HTTP。
适配器仅接受 HTTPS 或 loopback HTTP，不跟随重定向，不自动重试或切换供应商。
`execution` 描述运行归属，`adapter` 描述协议；两者独立于模型来源。

兼容协议参考[供应商工具调用文档](https://docs.modelstudio.console.alibabacloud.com/en/model-studio/qwen-function-calling)：
messages/tools、assistant.tool_calls、tool_call_id、工具结果追加后继续请求。
这里使用 Python 标准库 HTTP，不依赖该供应商的 SDK、端点、模型或 Key 名称。
模型必须实际支持文本工具调用；当前没有自动能力探针、流式输入或厂商专属参数，
不能据“兼容”二字保证任意服务可用。实际服务需单独验证。

## 与语音提供方解耦

任意识别客户端只需把每条**最终转写**写成 UTF-8 的一行文本，再接到：

```sh
uv run --locked eai-course agent --provider-config configs/agent/provider.local.toml --stdin
```

上游可以是本机模型、云服务客户端或其他设备桥接程序。`edge_inference_lab` 仅是历史已验证案例，
没有默认目录、导入、子进程启动或模型依赖。音频采集及识别服务凭据仍由上游管理；
Agent 配置只描述聊天推理服务，机器人硬件配置继续独立。

`voice --stdin` 仍是原有确定性白名单命令入口；`agent --stdin` 将非停止文本交给 LLM 选择
本次白名单工具。两者不把转写文本作为代码执行。停止词复用已有解析器，直接绕过网络请求。
stdin 首版一次仅接受一个普通回合，忙时的新文本返回 busy，不排队重放。
提供方失败后结束当前 Agent 输入会话；新识别行不会恢复旧计划。EOF 等待当前有界回合结束，Ctrl-C 取消。

纯文本通道不携带提供方、时间戳、置信度或 partial/final 标记：上游必须只发送最终文本。
结构化识别事件和用户本地 ASR 提供方配置仍是[语音架构](course/voice/ARCHITECTURE.md)中的后续目标。
本次没有把这些目标偷换为已实现的 ASR 管理功能。

## 工具、停止与证据边界

首版工具：`get_robot_state`、`list_poses`、`move_to_pose`、`get_task_status`、`stop_motion`。
姿态名来自已校验文件的枚举，不提供任意关节数值/系统命令工具。
任务结果为 `simulated`，targets 使用 0..1 归一化数值，不是真实关节角；hardware_sent 始终为 false。
最近 128 个演练任务可以查询，过期/未知任务返回 unknown_task。

每轮独立历史，默认最多 8 次请求、16 次工具调用、60 秒。供应商超时默认 30 秒，
与机械臂执行租期没有关系。参数或未知工具返回 rejected；重复调用 ID、协议错误或预算耗尽中止本轮。
同一批工具中出现合法 stop_motion 时优先停止，不先运行排在它前面的运动请求。

每轮新输入使用新的 generation；本地 stop/Ctrl-C 使当前 cancel 和 generation 失效，迟到响应不能执行工具。
取消只保证本地不再等待或执行，无法撤销远端计算/计费。
HTTP 无重试，最多保留一个 I/O worker；旧请求清理前，新请求返回 request_still_in_flight。
关闭会话取消本地等待；网络 worker 受 socket 超时、读循环 deadline 和体积上限约束后退出，
不要求推理服务支持远端取消。输出元数据事件不包含输入转写、完整消息、endpoint 或凭据。
终端最终回复是模型文本，不能作为实物完成证据。

离线集成测试通过真实 loopback HTTP 检查请求、工具回传、非法响应、超时/停止、重定向拒绝和凭据脱敏；
它使用测试响应，不是外部 LLM 验收。实物、真实语音、实际供应商和课程 4.3 仍需独立验收。
