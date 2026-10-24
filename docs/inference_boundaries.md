# 推理归属、外部依赖与接口决定

2026-10-19。此文确定模块边界；文本 ChatProvider、工具循环和演练 CLI 已实现，见[运行指南](agent_runtime.md)。
2026-10-24：聊天 API 接入/验收优先的基线已集成，新增[上位机 Agent](desktop_agent.md)，
页面内分别配置聊天、ASR/TTS；原生百炼语音已有薄适配。端侧作为解耦平行路径保留，
不新增端侧优化或自动安装；各阶段服务和凭据不能互相代替。
下文通用媒体/Omni 与实物技能协议仍为演进目标，不能据目标表格推断全部已实现。
研发方向见[两条开发主线](development_workstreams.md)，业务切片见[Agent 规划](agent_development_plan.md)。

## 已有推理基础在本仓库

| 能力 | 当前实现 | 在哪里运行 / 哪些内容在外部 |
|---|---|---|
| ACT / SmolVLA / π0.5 加载 | `policy/inference/runtime.py` 的 `LeRobotRuntime`、`make_runtime` | 本机或用同一源码启动的自有服务；模型类/处理器来自锁定的 LeRobot fork |
| 模型工件与契约 | `policy/contracts.py`、`inference/artifacts.py` | 应用在仓库；权重、tokenizer、数据及机器配置不入 Git |
| VLA 网络客户端/服务器 | `policy/inference/transport.py` 的 `RemoteRuntime`、`policy_server` | 已有认证、有界 JSON/RGB、会话/序号/manifest 关联；loopback HTTP + SSH 隧道 |
| 本地设备与动作门禁 | `policy/execution/runner.py`、`safety.py` | 保持在机械臂所在主机；推理服务不持有 Robot 或运动权限 |
| 图像检测器 | `vision/perception/detector.py`、`registry.py`、`spec.py` | 已有 detector 协议、模型设备规格、工厂与异步检测；具体模型按注册能力提供 |
| 语音识别 | 云端百炼 ASR；外部逐行文本提供方 | 上位机可录音后上传并接收最终转写；原 stdin 仍可使用，不依赖本机实验目录 |
| 文本 LLM Tools | `policy/model_io/`、`policy/agents/`、`applications/agent/` | 已实现可配置 HTTP、工具循环、逐行文本与演练技能；百炼 qwen-plus 已有真实工具回传证据；其他模型单独验收 |
| TTS | 云端百炼薄适配 | 上位机生成并播放 WAV；本地取消丢弃迟到音频 |
| VLM / Omni | 尚未实现完整接入 | 后续媒体适配器；不能把 VLA HTTP 当作聊天 API |

结论：保留仓库内的应用推理适配、数据契约与部署入口。计算可以外置；
模型框架、权重仓库、外部进程、远端算力与第三方 API 分开管理。
目前 SmolVLA 有 Mac MPS 离线推理证据；这不等于匹配本机任务的自主控制验收。
详见[已有推理指南](policy/README.md)。

## “外部”分为哪些类型

外部类型分两组记录：依赖来源与运行位置。它们不是一组互斥枚举；
例如自有 GPU 服务仍会加载来自模型仓库的权重，并使用外部框架包。

| 类型 | 边界 | 接口与维护责任 |
|---|---|---|
| 外部 Python 框架/SDK | LeRobot fork、PyTorch、未来 provider SDK | 本仓库写薄适配；框架来源/提交/依赖锁固定，不编辑 site-packages |
| 外部模型/数据资产 | Hub、下载工件、本地 checkpoint、训练数据 | 离线获取并冻结 revision/hash；复用已有 manifest/bundle，不在推理时隐式下载 |
| 本机外部进程 | 独立 ASR 实验、未来本地模型服务 | 优先协议接入；旧 ASR 保留 UTF-8 逐行 stdin，新增结构化模式另行版本化 |
| 自有远端推理服务 | 工作站、团队服务器、租用 GPU 上运行自己的模型 | 本仓库维护服务端与客户端；VLA 继续用现有 HTTP 协议和隧道，实测跨机时延 |
| 第三方托管模型 API | LLM/VLM/ASR/TTS/Omni 供应商 | 本仓库维护供应商适配与能力探针；服务端模型/计费由供应商控制，Key 和媒体传输另配置 |

租来的 GPU 上运行自己的服务属于“自有服务”，供应商代管模型接口属于“托管 API”。
两者都可能物理位于云端。不能用 `remote=true` 代替部署归属、传输协议和模型身份。
权重托管站不等于在线推理服务；固定 SDK 版本也不能保证托管 API 的模型权重不变。

## 模块归属与依赖方向

本轮实现进一步区分模型 I/O、通用 Agent 核心、机械臂应用，见[三层架构](agent_layers.md)。

```text
applications/agent         已有文本/dry-run 装配；媒体采集/播放与硬件会话待接入
        ├→ policy/agents   已有工具契约、上下文与决策循环；模型消息/provider 在 model_io
        ├→ vision          检测器、跟踪器、图像观测（已有）
        ├→ control         姿态/IK/视觉控制律、既有执行能力（已有）
        └→ policy          VLA runtime、执行门禁与数据（已有）

policy/model_io/providers.py → 供应商协议 / 本机模型 API / 自有聊天服务
policy/inference/runtime.py → 锁定 LeRobot 模型与处理器
policy/inference/transport.py → 同一 VLA runtime 的远程代理
applications/agent/speech.py → 独立云端 ASR/TTS 配置、校验与 SDK 工作进程
```

供应商适配不能反向调用硬件。`policy/agents` 通过注入的技能协议获取工具结果，模型接口位于 `policy/model_io`，
不导入 `applications`；`vision`、`control` 也不能反向导入应用。
设备生命周期与相机单一所有权在应用层，纯推理只接收数据。
ASR/TTS 首版在应用层适配，出现多应用复用需求后再评审抽取，不提前新建一级 speech 包。
现有 `policy/inference` 保持 VLA 语义，不为名字统一而把 detector、聊天与语音搬进去。

## 已实现接口：保持原有契约

以下是源码现有调用形式的摘要，不能把它们误写成同一个 `infer(dict)`。

```python
# VLA：Observation 包含 state、RGB 图像、task 与本机采集时间。
# Manifest 声明相机映射、六轴顺序/单位、动作频率、资源与标定摘要。
local_runtime.predict(observation)             # -> (actions[T, 6], inference_seconds)
remote_runtime.predict(observation, ticket)    # -> (actions[T, 6], round_trip_seconds)

# vision.perception.detector.ObjectDetector（已有 Protocol）
detector.detect(frame_rgb)                     # -> list[Detection]
detector.describe()                           # -> dict[str, object]
detector.close()                              # 释放模型资源，不操作机械臂
```

`actions` 是官方配对处理器输出的绝对标定百分比，不是角度或原始 tick。
本地 `predict` 第二个值是模型计算时间，远程是客户端往返时间，日志不得混用。
现有 VLA 线上请求为 `session/sequence/manifest/observation`，响应回显前三项并带
`actions/inference_s`。客户端使用原始 ticket、本机采集时间和本机 deadline 验证执行资格；
不比较两台机器的 monotonic 数值，也不信任服务端重新生成的采集时间作为新鲜度证据。
原始图像的获取时间仍保留在客户端。现有服务为单线程同步服务，不承诺多客户端调度。

## 拟新增接口：按输出类型定义

ChatProvider 通过官方 SDK 接入 Chat Completions 与 Responses，支持标准图像内容块（Chat 另含 input_audio）。
完整兼容范围见[标准契约](agent_standard_contract.md)；采集、实时音频和下列 ASR/TTS 签名仍是后续约定。
实际类和字段以源码及[运行指南](agent_runtime.md)为准；不创建只有签名的空 ASR/TTS provider。

```python
class ChatProvider(Protocol):
    def describe(self) -> ProviderInfo: ...
    def stream(self, request: ChatRequest, *, cancel: Event) -> Iterator[ModelEvent]: ...
    def complete(self, request: ChatRequest, *, cancel: Event) -> ChatReply: ...
    def close(self) -> None: ...

class SpeechRecognizer(Protocol):
    def describe(self) -> ProviderInfo: ...
    def transcribe(self, request: AudioRequest, *, cancel: Event) -> Transcript: ...
    def close(self) -> None: ...

class SpeechSynthesizer(Protocol):
    def describe(self) -> ProviderInfo: ...
    def synthesize(self, request: SpeechRequest, *, cancel: Event) -> AudioReply: ...
    def close(self) -> None: ...

class SkillService(Protocol):
    def invoke(self, request: ToolInvocation) -> ToolResult: ...
    def get_task_status(self, task_id: str) -> TaskStatus: ...
    def stop_motion(self, session_id: str) -> ToolResult: ...
```

ChatProvider 支持完整回复与 SSE 流，传输位于模型层；核心消费统一流事件。`cancel` 用于取消等待并丢弃迟到响应；
供应商支持时终止请求，不能保证撤销服务端计算/计费。它不代替本地 `stop_motion`。
流式 Omni 后续采用单独 `OmniSession.send/receive/cancel_turn/close` 事件协议；
不把流式音频和工具片段直接套进一次性 `complete` 返回值。

| 数据类型 | 必需语义 |
|---|---|
| ProviderInfo | provider_id、接口版本、adapter 版本、model_id、model_revision（可未知）、模型来源、运行部署、支持的输入/输出模态、tools/streaming/cancel 能力 |
| ChatRequest | protocol_version、request_id、session_id、generation、回合消息、工具 schema、客户端超时预算、输出/token/调用轮次预算 |
| 消息内容块 | text、image、audio、tool_result 分型；图像/音频附 media_id、格式、尺寸/采样率、采集信息；不接受供应商私有对象穿透业务层 |
| ChatReply | 关联请求身份、完整 assistant 内容、tool_calls、finish_reason、可用 usage/服务请求 ID；支持文本和工具同轮返回 |
| ToolInvocation | tool_call_id、白名单工具名、已解析 JSON 参数、session_id、generation；不携带 Robot、串口或 enable 权限 |
| ToolResult / TaskStatus | 工具调用 ID、会话/代次、ok、status、error_code、message、data、task_id；区分 accepted/running/holding/completed/cancelled/failed/simulated/rejected |
| AudioRequest / Transcript | utterance_id、编码/采样率/声道、采集区间；文本、语言、final 标志；confidence 不可得时为 null，不编造分数 |
| SpeechRequest / AudioReply | request_id、文本、语音配置、编码；生成音频与关联身份；播放完成由本地播放器反馈 |

工具调用 ID 在整个回合历史内唯一。供应商返回的 JSON 参数必须再次验证 schema 和有限数值。
异步动作一旦接单，后续用 task_id 查询，不因模型超时重复执行。
同一 `generation` 内去重工具 ID；停止递增 generation，废弃旧请求、媒体与计划绑定。
流式 provider 只有在工具名、完整参数 JSON 与结束标志齐全后才产生 ToolInvocation。

## 配置决定：分别表达协议、位置与来源

当前文本配置示例位于 `configs/agent/`；下列跨模态字段是完整演进目标，
当前只支持其中运行指南列出的子集，不接受未知配置字段：

```text
stage                  asr | chat | tts | detector | action_policy
adapter                具体实现标识；决定调用哪种协议，不等于供应商品牌
execution              in_process | local_process | self_hosted | managed_api
model_id/revision      模型身份；未知 revision 明示 unknown
artifact_manifest      本地/自有 VLA 必需；托管 API 不伪造本地权重摘要
endpoint               网络适配器使用；只允许操作者配置，模型不能指定
credential_env         环境变量名称，配置/日志不保存值
requested_device       本地或自有服务的设备请求；managed_api 不假设服务器设备
capabilities_required  tools、image_input、audio_input/output、streaming 等
request_timeout_s      模型请求预算；与本地控制租期分开
media_policy           允许传输的模态、目标服务与尺寸/时长上限
```

describe/日志记录解析后的实际部署、设备（可未知）、模型和能力，避免把 `auto` 当作设备证据。
供应商兼容某一 JSON 格式不代表支持全部工具、音频与流式协议；启动探针检查 required capabilities，
未来能力探针缺失时应拒绝该会话配置；当前有标准 Chat/Responses 工具协议、内容块与显式 SSE，不作模型能力保证，不静默切换为另一服务或改变媒体传输范围。
现有 VLA CLI/manifest 与 detector ModelSpec 暂不改名；未来会话配置仅装配它们，不能复制并覆盖两份限额。

## 错误、生命周期与可替换性验收

Provider 错误统一映射到配置/能力不支持、认证、限流、超时、取消、协议错误、服务不可用；
保留脱敏的 provider 错误码与 request_id。供应商对象不进入工具分发器。
网络重试仅针对确定无副作用的推理，并受回合预算限制；不重试技能动作，不自动重连续跑。
所有 provider 显式创建/关闭，模块导入不下载、不联网、不采集；退出关闭工作线程和 owned 资源。

每个适配器至少验证：同一输入/输出契约、能力不足、超时/取消、畸形输出、超大消息、
停止后的旧响应、密钥脱敏、资源关闭。fake 用于离线 CI，真实 API 验证另记实际服务与模型。
新增依赖时明确放根控制锁还是训练锁：机械臂主机作为纯远程客户端无需加载本地 VLM 权重。
不为尚未使用的供应商安装一组 SDK，也不从外部实验复制整套环境。
