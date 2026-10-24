# Agent 开发交接与集成手册

更新：2026-10-24。面向接入服务、优化交互、开发视觉/控制技能及负责评测的协作者。
日常启动与 API 填写见[使用手册](desktop_agent.md)；本页说明实际源码、契约和后续工作。
这是实现现状与接入约束，标为「设计目标」的部分不能作为已交付能力。

## 1. 接手时先核对版本和验收范围

| 内容 | 当前状态 | 接手入口 |
|---|---|---|
| 三层模型/核心/机器人应用 | 已实现，标准 OpenAI SDK 消息和工具 | [三层架构](agent_layers.md)、[标准契约](agent_standard_contract.md) |
| 网页 Agent 与会话 API 配置 | PR #25 已合并至 develop，不需要网页用户编辑文件 | [PR #25](https://github.com/EAI-Course-2026/eai_project/pull/25) |
| 文字、按回合云 ASR/TTS | 北京百炼真实请求已验证 | [真实服务报告](reports/2026-10-24-cloud-speech.md) |
| 更多聊天模型的真实验收 CLI | PR #24 已合并至 develop，提供 agent-config probe | [API 验收手册](agent_api_acceptance.md) |
| 页面内 Chat Completions / Responses | 复用既有适配器；各模型能力要单独验收 | `policy/model_io/` |
| 持续识别、自然打断、语音双工 | 未实现，页面入口禁用并标注待接入 | 本页第 9 节 |
| 腕部相机进入 Agent | 未接入；标准图像消息能力不等于已接上相机 | `policy/model_io/` 与视觉应用 |
| 物体检测/跟踪、IK 作为 Agent tools | 未注册到当前 Agent | 第 8 节；底层能力可分别存在 |
| Agent 实物执行 | 不开放，所有运动工具 dry-run | `RobotCapabilities` 与 `DryRunSkills` |
| 公网服务与租户鉴权 | 未部署，服务仅绑定 loopback | 第 10 节 |

在 PR 合并前复现要使用对应任务分支；不要把 main、develop、PR head、正在运行的旧进程混为一个版本。
本轮没有安装本地 Conda，没有打开串口/相机，也没有电机动作或真实麦克风验收。
详细分支与 worktree 清理回执见[整理记录](reports/2026-10-24-worktree-cleanup.md)。

## 2. 产品语义和工程边界

产品名称统一为 **Agent**。文字、按回合语音以及未来的双工是交互模式，不能各自复制一个工具循环。
目标是自然语言提出控制任务，观测、选择技能、执行并验证效果；任务质量与硬件固有边界共同决定上限。
优先优化模型选择、prompt、harness、感知和已有控制技能的组合，不预设必须进行大规模模型训练。

已知关节/反馈限制由控制模块拥有；未知负载、工作空间精度和位置误差保持未知，不能让模型补齐。
模型选择已验证技能，不能产生串口代码、猜电机控制量、扩大标定范围或把 accepted 当作完成。
端侧路线保留独立文本入口，云语音不会 import 外部实验 checkout，也没有本机目录依赖。

| 层 | 目录 | 可修改的职责 | 不应加入 |
|---|---|---|---|
| 模型 I/O，借鉴 pi-ai | `src/eai_robot/policy/model_io/` | Chat/Responses 协议、SDK 流、消息、凭据、脱敏错误 | 工具 handler、串口、应用 prompt |
| harness，借鉴 pi-agent-core | `src/eai_robot/policy/agents/` | 工具循环、上下文、预算、事件、取消与排空 | 厂商 HTTP 参数、硬件和 GUI |
| 机器人应用，借鉴 pi-coding-agent 的装配角色 | `src/eai_robot/applications/agent/` | prompt、能力白名单、技能、文本入口、云语音装配 | 第二套模型协议、执行器旁路 |
| 上位机集成 | `src/eai_robot/applications/desktop/` | 浏览器会话、内存配置、录音/播放与任务展示 | 页面直接持有串口、复制工具循环 |
| 控制与硬件 | `control/`、`hardware/` | 租期、反馈、门禁、执行、停止与通信 | 依赖云端是否在线才能停止 |

依赖方向是 application → agents → model_io，application 可以装配 provider。
核心不能反向 import 机器人应用；VLA 的 `policy/inference/` 不与聊天消息或 action chunk 混为同一个协议。
不引入 pi Node 包、pi 本地 checkout 或编码 Agent 的 shell/file 工具作为机器人运行依赖。

## 3. 源码阅读顺序与运行链路

建议先跑使用手册的无硬件查询，再按以下顺序看代码：

1. `desktop/server.py`：HTTP 路由、默认启动和生命周期。
2. `desktop/agent_config.py`：`AgentHub`、`AgentRecord`，服务配置与 runtime 的原子替换。
3. `desktop/agent.py`：`DeskAgent`，文字/语音共用的回合协调。
4. `agent/session.py`、`agent/capabilities.py`、`agent/skills.py`：应用装配、能力、五个工具。
5. `policy/agents/loop.py`、`tools.py`、`context.py`：执行、校验、记忆和取消。
6. `policy/model_io/providers.py`、`responses.py`、`auth.py`：官方 SDK 适配与凭据。
7. `agent/speech.py`、`agent/_dashscope_speech.py`：受控语音调用与 SDK 子进程。
8. `desktop/assets/agent.js`、`agent-settings.js`、`agent.css`：输入/播放、配置表单、样式分别维护。

```text
浏览器文字 ──────────────────────────────┐
浏览器录音 → 16k mono WAV → 云 ASR 最终文本 ├→ DeskAgent → 同一 AgentSession
外部端侧/其他识别端 → stdin 最终文本 ──────┘                  │
                                         OpenAI SDK provider → 模型
                                                           │
                                              标准 tools → 本地白名单 handler
                                                           │
                                           工具结果回传 → 最终文字 → 可选云 TTS
                                                                              │
                                                                    WAV → 浏览器播放器
```

上图的 stdin 是独立 CLI 装配，不经过网页 DeskAgent，复用的是相同的 session/工具契约。
TTS 在工具循环完成后运行；生成失败不会再调用模型或执行工具。
模型 SSE 由 SDK 消费，前端每约 400 ms 轮询状态；当前不是浏览器与模型的直接实时连接。

## 4. 配置、凭据和会话契约

### 4.1 页面配置是默认入口

`AgentHub` 使用端口区分的 HttpOnly/SameSite=Strict session cookie 定位记录。
默认无配置，最多 16 个记录；一小时无 Agent API 访问且未忙的记录在后续访问时回收。
页面会轮询，打开页面不等于闲置；立即移除 Key 应调用清除或退出服务。
同一浏览器会话的标签页共享记录，独立会话不共享。它不是登录用户系统或公网多租户鉴权。

`AgentRecord.configure()` 在锁内执行：

1. 确认 UI 回合不忙，核心已排空；普通保存不能切走仍在执行的回合。
2. 分离公开设置与 Key；校验每一阶段、URL、预算及标准 provider 配置。
3. 只有提供方和 URL 都没变时，空 Key 才能沿用旧 Key。
4. 先构造完整候选 runtime，失败不改变旧配置；不发真实 API 探测请求。
5. 原子替换 runtime、生成新 runtime_id、清空对话，再关闭旧实例。

网页 Key 只在服务端内存；返回配置使用 `key_configured`，绝不回传 `api_key`。
内部随机凭据名称避免网页 Key 被同名进程环境变量覆盖。前端不用 localStorage/sessionStorage。
`clear()` 丢弃当前记录的设置、Key 和 runtime；不清除其他记录，不会重新应用启动 seed。

可选文件 seed 仅用于原有 CLI/操作者兼容，显式 `--agent-config` / `--speech-config` 才读取。
`.env` 不自动发现、不插值，CLI 进程环境优先。seed 会保留在 Hub 内存并供新会话继承，
因此不适合未来公网多租户模式；不要误以为清除一个记录会抹掉 Hub 的操作者初始配置。

### 4.2 HTTP 接口

以下均为现有接口，不是 OpenAI 协议的替代品。调用前 GET state 取得 cookie 和当前身份。
POST 需要同源及 `X-Arm-Desk: 1`，Agent 路由验证 loopback Host/端口。

| 方法与路径 | 输入 / 输出 | 语义 |
|---|---|---|
| GET `/api/agent/state` | runtime_id、generation、enabled、ASR/TTS、busy、状态、消息、工具、公开设置 | 仅当前 cookie 会话；no-store |
| GET `/api/agent/catalog` | 既有聊天服务预设和别名 | 地址默认值，不是模型可用性证明 |
| POST `/api/agent/config` | 可选 chat/asr/tts 对象；200 返回公开设置 | 完整替换，省略的阶段被禁用 |
| POST `/api/agent/clear` | `{}`；202 | 清当前配置/对话，取消旧 runtime |
| POST `/api/agent/turn` | text 或 audio 二选一、speak、runtime_id、generation；202 | 仅表示接受，不代表模型或任务成功 |
| POST `/api/agent/stop` | `{}`；202 | 取消当前会话，已连接设备时走原停止回调 |
| GET `/api/agent/audio/{runtime_id}/{generation}` | 200 `audio/wav` 或 404 | 只能取当前会话当前有效代次的最后音频 |
| POST `/api/action`，action=stop | 原控制接口 | 控制器停止一次，并取消所有 Agent 会话 |

配置 JSON 形态如下，仅作字段示例，示例 Key 不能用于真实请求：

```json
{
  "chat": {
    "provider": "custom",
    "adapter": "chat_completions",
    "execution": "managed_api",
    "base_url": "https://example.invalid/v1",
    "model": "OPERATOR_SELECTED_MODEL",
    "api_key": "ENTER_IN_PAGE",
    "streaming": true,
    "timeout_s": 30,
    "max_tokens": 1024,
    "token_limit_parameter": "max_tokens",
    "model_options": {},
    "extra_body": {}
  },
  "asr": {
    "provider": "dashscope",
    "websocket_url": "wss://YOUR_WORKSPACE_ID.cn-beijing.maas.aliyuncs.com/api-ws/v1/inference",
    "model": "qwen-audio-3.1-asr-flash-message",
    "api_key": "ENTER_IN_PAGE",
    "timeout_s": 30
  },
  "tts": {
    "provider": "dashscope",
    "websocket_url": "wss://YOUR_WORKSPACE_ID.cn-beijing.maas.aliyuncs.com/api-ws/v1/inference",
    "model": "qwen-audio-3.1-tts-flash",
    "voice": "longanhuan_v3.1",
    "api_key": "ENTER_IN_PAGE",
    "timeout_s": 30
  }
}
```

`provider` 是聊天展示标签，路由由显式 URL/adapter 决定。页面当前装配 managed_api，要求 HTTPS；
自有服务/端侧 CLI 的 execution 选择见[提供方指南](agent_providers.md)，网页没有完整本地服务编辑器。
下一轮文字请求示例：

```json
{"text":"先查看状态，再列出姿态。","speak":true,"runtime_id":"FROM_STATE","generation":0}
```

audio 字段是完整 WAV 的 base64，不能同时带 text。generation 必须为整数并匹配 state，
runtime_id 要匹配当前实例；切换配置后即使 generation 相同也不能接受旧实例请求。
不要把 Key 或 cookie 放在 URL、日志或示例命令行参数里。

当前 HTTP 错误为：输入/配置/过期/忙等 400，Host/Origin 等 403，无效音频链接 404，
GET 会话上限 429，未预期服务异常使用脱敏 500；它还不是稳定版本化公网 API。
配置体上限 64 KiB，turn 3 MiB，其他 POST 16 KiB。不要仅靠浏览器检查大小。

## 5. 状态、取消和结果可信度

DeskAgent 的状态为 idle、recognizing、thinking、tool、synthesizing、stopping；
`busy=false` 还需结合核心 `wait_for_idle()`，不能据此强行与 draining 的工具并行。
核心单会话一次普通回合，工具串行；每轮默认最多 8 次模型请求、16 次工具、60 秒。
聊天请求默认 30 秒，语音请求默认 30 秒，语音两阶段各自计时；总交互时长不是单个 30 秒。

三个不同层的身份不能混用：

| 身份 | 所有者 | 失效条件 |
|---|---|---|
| 浏览器 cookie token | AgentHub | 记录被回收/服务退出；清配置保留记录与 cookie，仅撤销 runtime |
| runtime_id + DeskAgent generation | 上位机协调器 | 配置替换换实例；每轮/停止换代次 |
| session_id + generation + run_id | harness | 核心回合/取消身份；工具上下文据此判断有效性 |

外部取消 Event 在 `run_turn()` 开始之前就安装，避免停止先于 worker 启动时丢失取消。
停止先使代次与音频失效，再请求核心取消、回收语音 worker，已连接设备走控制器停止。
空闲页面关闭不触发机械臂停止；活跃回合关闭时发 keepalive 取消，但浏览器投递仍不能当作物理停止证明。
跨标签页更换 runtime 时，前端也撤销旧录音与播放器。

事件顺序大致为 run_start → model_start/delta/end → tool_start/progress/end → 后续模型轮 → run_end → run_settled。
事件带单调序号、身份、本机 monotonic at 和 data。observer 不应阻塞；异常只计数，不能授予工具权限。
`run_end` 是编排结束，`run_settled` / wait_for_idle 是本地核心排空；都不等于远端计算结束或物理设备停止。
Python handler 不被强杀；旧工具未退出时禁止重叠执行，写入前仍须检查控制器门禁。

历史最多保存 8 个完成回合，并受字符预算约束；失败/取消不变成可恢复工具队列。
最终 assistant 的 reasoning_content 等元数据跨完成回合保留，以支持厂商协议；不要把 opaque reasoning 当控制指令。
Responses 在工具回合保留所需 continuation，不能随意丢弃 reasoning/output 项。
UI 最多保留 20 条消息、12 条工具事件；只有当前最后一份音频可获取，不是录音归档系统。

## 6. 当前工具与 prompt

标准定义位于 `agent/skills.py`，运行 `eai-course agent --list-tools` 可查看真实 payload。
`Tool.definition` 原样使用 OpenAI Chat Completions 的嵌套 function schema；没有 pi/Codex 专用工具格式。
Responses 扁平 schema 的转换只发生在其模型适配器。

| 工具 | 参数 | 当前返回 | 与作业关系 |
|---|---|---|---|
| get_robot_state | 空对象 | dry_run、未连接、无关节反馈、tracking unavailable | 有该接口，但不能宣称读到真实关节角 |
| list_poses | 空对象 | 校验过标定绑定的命名姿态列表 | 为模型提供受控选项 |
| move_to_pose | pose_name，枚举 | task_id、归一化目标、hardware_sent=false | 仅计划，不移动机器人 |
| get_task_status | task_id | 本进程模拟任务；未知 ID 拒绝 | accepted 与完成状态的查询路径 |
| stop_motion | 空对象，interrupt 工具 | 取消演练；工具本身不卸扭矩 | 本地停止优先，GUI另有控制器停止回调 |

`detect_object`、`start_tracking`、`stop_tracking` 和 IK 还没有注册到该白名单。
不要把已有视觉伺服 CLI 直接描述为 Agent 可调用接口，更不能造一个总返回 found=true 的检测工具过验收。
当前 `RobotCapabilities` 明确禁止 physical_execution，模型无法用 prompt 修改它。

prompt 唯一源码为 `applications/agent/prompts/system.txt`，随 wheel 发布；当前核心规则是：

> 先读取状态，需要姿态时查询可用姿态。不要编造工具、姿态或机器人反馈。
> 工具 accepted 不代表运动完成；按工具结果或任务查询报告状态。
> 当前应用只提供 dry-run：所有姿态动作都是模拟计划，绝不能说真实机械臂已移动。
> 用户要求停止时调用 stop_motion，不再执行本轮其他动作。

回答默认短自然语言以便播报；历史文本不能代替新观测，工具数据不能改写上层规则。
改 prompt 需评测任务成功率、工具选择、拒绝和停止行为，不能只看回复听起来顺畅。

## 7. 增加或切换模型服务

### 7.0 团队模型基线与额度约定

日常联调优先使用百炼北京免费额度，推荐组合、前置检查和分阶段测试安排见
[模型与免费额度建议](agent_models.md)。当前已验证 qwen-plus + ASR Message + TTS Flash；
更便宜或更新的模型只有通过固定工具/取消任务集后才能替换基线。额度不足则停止真实调用，
不自动切付费模型；离线 CI 不使用 API Key，不消耗免费额度。
平台额度开关、客户端调用预算和执行器运动门禁是三件事，不能相互替代。

### 7.1 同协议聊天服务

如果服务支持现有 Chat Completions 或 Responses，先用自定义 URL/模型/Key 和显式 extra_body，
不要为每个厂商复制 provider 类。默认禁用 SDK 自动重试和重定向，避免隐式重复整个工具轮。
预设仅维护在 `applications/agent/provider_presets.json`；网页名称在 `assets/agent-settings.js` 映射。
改预设后同时检查 CLI 和页面，补地区/参数证据，不把“可保存”当“真实通过”。

兼容验收至少覆盖：普通文本、一次工具及回传、多轮工具、JSON/SSE、跨用户轮上下文、取消、
认证/限流、无效参数、usage 与超大/残缺响应。思考模型特别检查工具后的完整 assistant 和 reasoning 字段。
模型不支持某项能力时明确拒绝，不自动删除 tools 或换模型蒙混过关。

### 7.2 真正不同的协议

先定义该服务能满足的能力，再在 model_io 实现 ChatProvider 的 describe、stream、complete、close。
ChatRequest/ChatReply/ModelEvent 仅装身份与预算；供应商对象、SDK 异常不能渗入 handler 或 GUI。
使用官方 SDK 的传输/流累积，保留标准消息/工具作为边界，不写第二套通用 SSE/JSON 解析器。
新增依赖放根环境可选或必要依赖，不能随手加进训练锁；必须有网络/设备均关闭时的导入测试。

### 7.3 ASR/TTS 扩展

当前 `CloudSpeech` 可提供部分阶段；`transcribe(bytes, cancel=...)` 返回最终 text、usage、首包/总耗时，
`synthesize(text, cancel=...)` 返回 audio WAV 及同类指标；关闭后拒绝调用。
当前 SpeechEndpoint 只接受 DashScope，不能仅把 provider 改个名字就接入另一家的音频 API。

要增加供应商，先抽出真正被使用的媒体客户端协议，再增加独立薄适配与页面阶段选择。
保持 mono/16bit/16kHz WAV、最终转写、取消、时长/尺寸和脱敏错误契约；
不要把 Key 写全局 SDK 状态，不在 URL 或子进程命令行传递它。
端侧识别可继续使用 stdin 最终文本，不需要适配云 SDK 或复制聊天核心。

## 8. 新工具、视觉技能和实物接入

工具边界为 `handler(arguments: dict, context: ToolContext) -> ToolResult`。
ToolResult 字段为 ok、status、data、error_code；data 必须有界且 JSON 可序列化。
ToolContext 提供身份、deadline、cancel、check_active()、progress(data)。
标准 schema 经 Draft 2020-12 本地验证，不取远程 $ref；strict 仅在目标模型支持时显式打开。

新增只读模拟工具的最小示例：

```python
from eai_robot.policy.agents.contracts import ToolResult
from eai_robot.policy.agents.tools import Tool, ToolRegistry

def read_fixture(arguments, context):
    context.check_active()
    return ToolResult(True, 'simulated', {'observation': 'fixture', 'hardware_sent': False})

spec = {'type': 'function', 'function': {
    'name': 'read_fixture', 'description': '读取离线测试观测。',
    'parameters': {'type': 'object', 'properties': {}, 'required': [], 'additionalProperties': False}
}}
registry = ToolRegistry([Tool(spec, read_fixture)])
```

示例仅说明 schema/handler 绑定，未加入默认 RobotCapabilities 白名单。
接入实际应用还需更新能力声明、默认工具装配、prompt/说明和测试；不能只在 UI 添加按钮。
不在模型工具中开放任意 Python、shell、串口路径、寄存器写入或模型可自由选择的网络地址。

接视觉伺服时按以下契约设计并单独实现（**设计目标，当前不存在**）：

| 工具目标 | 调用前检查 | 返回与任务语义 |
|---|---|---|
| detect_object(description) | 相机新鲜度、检测器能力、目标可见性 | found/object_id，含坐标系、观测时间、置信度/有效期；不伪造找到 |
| start_tracking(object_id) | ID 属于本会话且未过期，执行权、反馈、限位、目标有效 | accepted/task_id；由本地技能循环闭环，LLM 不逐电机控制 |
| stop_tracking() | 任意活动阶段均可进入停止路径 | 任务终态/停止请求，完成需真实控制器反馈 |
| IK 末端移动 | 坐标系/单位、可达性、预算、当前观测、执行权限 | 明确目标与任务 ID；求解成功不等于已到位 |

长时技能服务持有任务，handler 启动后返回 task_id，状态查询区分 accepted/running/reached/failed/cancelled。
任务要带 session/generation、期限、观测年龄、停止条件与执行器租期；写硬件前再次检查。
默认拒绝无观测、未知坐标系、反馈过期、目标丢失和权限不足。不能靠扩大限位或租期掩盖失败。

实物接入需新增经审查的能力装配和明确授权流程，复用既有控制线程、反馈守卫和单串口所有权。
不能只把 DryRunSkills 的 hardware_sent 改成 true 或翻转 physical_execution 开关。
回归需覆盖：取消先到、迟到模型/目标、停止与写入竞争、反馈异常、扭矩故障、租期过期、重启不重放。
硬件验收记录日期、设备资产、操作者、幅度、边界、任务结果和失败；软件 CI 不能代替这些证据。

## 9. 双工、多模态与优化顺序

当前按段录音→ASR→文本工具循环→整段 TTS，串联延迟包含各阶段网络和 spawn。
浏览器开始录音前停止播放；用户结束录音后才识别。不要把这个流程标为双工。

下一步按可验收切片推进：

| 优先级 | 工作 | 完成条件 |
|---|---|---|
| P0 | 真实麦克风输入、回声/噪声、权限失败、取消测试 | 固定口令集含否定/停止，在实际设备留存脱敏结果 |
| P0 | 服务兼容评测与回归基线 | 选定模型文本/工具/流/取消均有真实证据；脚本与离线 fixture 分开 |
| P0（公开部署前） | 登录/租户/权限、TLS、限流、endpoint 出站策略 | 不能复用一个操作者 seed 给所有网络用户；密钥与硬件执行权独立 |
| P1 | 持续 ASR、VAD/端点检测、部分文本事件 | partial 仅展示，final 才创建动作回合；身份与超时覆盖整条媒体流 |
| P1 | 可打断 TTS 与双工状态机 | 同时收听/播放，回声处理，取消旧播报后旧结果不可回灌；按钮停止独立可靠 |
| P1 | 腕部视觉与观测绑定 | 帧时间/尺寸/传输预算、坐标系、失效与目标 ID 契约齐全 |
| P1 | 视觉/IK 工具与控制执行权 | 先仿真，再经授权实物；共享控制门禁而非新增串口线程 |
| P2 | 路由、prompt 与 harness co-design | 相同固定任务集比较成功率、耗时、成本和停止失败，依据证据选改动 |

双工可以采用原生实时模型或分离 ASR/LLM/TTS，但都必须复用同一工具授权/取消域。
有界队列、背压、会话重连、断网后的终态、半句不执行、否定词、回声自激和重复转写都要测试。
相机帧按实际模态能力进入模型；支持 image_url 的类型并不证明支持当前相机、场景理解或任务完成。

### 已知工程取舍

DashScope 1.27.8 官方 SDK 使用全局 Key/URL 和阻塞方法，当前每次媒体调用独立 spawn 进程，
同一 CloudSpeech 同时只运行一次，取消/超时回收本地进程后才释放 slot。
这隔离全局状态且能硬取消本地 I/O，但有启动延迟，不能据此承诺远端立即停止计费。

ASR 为 SDK 文件接口写受限系统临时目录，正常调用结束删除；子进程被强制终止时可能留下临时 WAV。
后续应把临时文件生命周期移到父进程管理，并对异常/取消回收做测试，不能宣称当前音频全程只在内存。
网页 API 设置与 Key 不写文件这一结论不包含上传的音频临时文件。
浏览器录音使用 ScriptProcessor 的简易分段采集；持续低延迟流需要评估 AudioWorklet/VAD，不能只改按钮文案。

评测建议记录：任务成功率、错误动作/误触发、否定理解、停止延迟、过期输入拒绝、ASR准确率、
LLM 首字/总时长、工具耗时、TTS首包/总时长、端到端延迟及 usage。固定环境后再比较 p50/p95。
本轮三句合成音频只验证基础连通，不能外推真实语音质量或尾延迟。

## 10. 部署与数据生命周期

当前 `ThreadingHTTPServer` 绑定 127.0.0.1，仅面向本机上位机。页面可配置 API，但没有已部署公网 URL。
如需上线，先分离 Web 会话/API 服务与硬件执行代理；后者保留本地门禁与独立停止路径。
公网版需有认证、租户隔离、TLS、请求/配额限制、出站目的地策略和浏览器会话到执行权的绑定。
不要直接把 Host 白名单删掉或把 bind 改成 0.0.0.0 充当上线方案。

当前数据保存边界：

| 数据 | 当前所在位置 / 清理 |
|---|---|
| 页面 Key/配置 | 服务端内存；显式清除、重启或惰性过期回收，无持久化恢复 |
| 浏览器 token | HttpOnly 会话 cookie；不是 Key，也不是用户鉴权 |
| 对话/工具/音频 | 每记录内存有界，配置切换或重启丢失；音频仅当前代次 |
| ASR 输入 WAV | SDK 受限临时目录；异常终止残留取舍见上文 |
| CLI probe 与本机验收输出 | 操作者选定的本地目录；报告/WAV不会自动上传代码仓库 |
| 机器人配置/标定 | 原有独立硬件配置，不能和 provider Key 合并 |

当前没有持久记忆、崩溃恢复或动作恢复。将来即使恢复对话，也不能自动恢复工具执行队列。
任何重启后的动作都需要新观测、授权、幂等/去重语义和物理状态对账。

## 11. 验证、调试和协作交付

### 软件检查

```sh
uv sync --locked --extra speech
uv run --locked --extra speech python scripts/check_env.py
uv run --locked --extra speech python -m unittest discover -s tests -q
uv run --locked --extra speech python scripts/check_repository.py
node --check src/eai_robot/applications/desktop/assets/app.js
node --check src/eai_robot/applications/desktop/assets/agent.js
node --check src/eai_robot/applications/desktop/assets/agent-settings.js
uv build --wheel --all-packages
```

针对性入口：`test_cloud_speech.py` 验证输入、进程取消/期限、凭据与部分配置；
`test_desk_agent.py` 验证 HTTP、回合互斥、迟到输入/识别、TTS 失败不重复工具；
`test_desk_agent_config.py` 验证两浏览器隔离、Key 不回显、原子更新、旧实例和全局停止。
`test_agent_configuration.py` 覆盖预设、厂商扩展和跨轮 reasoning；其他 agent/layers 测试验证 JSON/SSE 与核心。
`test_source_architecture.py` 约束层间依赖和命令入口。不要为了过测删掉运动租期、反馈或取消回归。

本轮代码基线本地 576 项离线测试通过，环境/结构/三个 JS 语法与 wheel 构建通过。
[PR #25](https://github.com/EAI-Course-2026/eai_project/pull/25) 的检查以**最终 head SHA**为准；
文档或代码新提交后必须重新看当前检查，不能沿用旧绿色。三项必需 CI 为 macOS/Windows offline 和 Windows CUDA 软件检查。
真实服务报告、离线 fixture、浏览器麦克风和实物记录必须分别声明。

### 真实 API 验收

用页面的五分钟清单完成文字/工具/媒体流程；文件批处理则使用使用手册的 `speech probe`。
只发送操作者明确选择的文本/文件，不自动录音、扫描其他目录的 Key 或调用所有预设。
记录服务、模型、协议、SDK/源码版本、阶段耗时、usage、错误和脱敏结果；不要提交账户信息或音频。
测试中不要“自动重试到成功”而隐藏原始失败；重新一轮要有新意图与新证据目录。

本轮真实证据：TTS→ASR 三句 3/3，真实模型选择工具并回传，合成音频→ASR→同一工具循环→TTS，
停止语音经最终识别走本地旁路，页面保存配置后真实调用和播放器均已检查。
实际麦克风、自然打断、视觉和实物尚未验收，不能提交为作业 4.3 全通过。

### 协作和审查

维护方式见 [CONTRIBUTING](../CONTRIBUTING.md)。永久 main/develop 保留，功能从最新 develop 新建短期分支。
先明确本次模块与验收切片，再修改；避免重复通用框架、写第二份 schema 或复制整个应用。
改标准协议附 fixture，改取消/执行附竞争回归，改供应商附实际服务证据，改 UI 附浏览器核对结果。
需要另一位 CODEOWNER 批准且三项当前 CI 通过才能合并；代理不代替维护者批准。
合并后核对目标分支 push SHA 的 CI，完成后才删除无独有提交、无人使用的任务分支/worktree。
清理前检查未提交/未跟踪/忽略文件和运行进程，先保存本机资料，不靠 `git clean -fdx` 或强制删除收尾。

建议接手顺序：先复现只读任务 → 选定模型基线 → 完成麦克风与停止评测 → 补媒体生命周期 →
再做双工或视觉/控制技能之一。每次输出可审查的小 PR 和任务质量对比，保持真实硬件边界不变。
