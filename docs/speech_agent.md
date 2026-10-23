# 上位机 Agent

2026-10-24。Agent 嵌入机械臂上位机；文字和语音是同一 Agent 的交互模式，
共用 AgentSession、prompt、OpenAI 标准工具与停止机制。当前技能全部 dry-run。
无需连接机械臂即可对话。腕部相机、跟踪/IK 工具与 Agent 实物执行尚未接入。

## 一键启动与页面配置

使用项目 Python 3.12 与 uv；云语音 SDK 为可选依赖，不需要端侧模型或 Conda：

```sh
uv run --locked --extra speech python scripts/arm_desk.py --http-port 8765
```

打开终端打印的 `http://127.0.0.1:8765/#agent`，选择「Agent」，点击右上角「API 接入」。
不需要创建 TOML 或 .env 文件。端口被占用时换一个端口，或用 `--http-port 0` 自动分配。

| 页面配置 | 内容 |
|---|---|
| 聊天与工具 | 服务预设、Chat Completions / Responses 协议、Base URL、模型 ID、API Key |
| 请求参数（展开） | token 限额字段、超时、model_options 和 extra_body JSON |
| 语音识别 ASR（可选） | DashScope 原生 WebSocket URL、识别模型 ID、Key、超时 |
| 语音合成 TTS（可选） | DashScope 原生 WebSocket URL、合成模型 ID、音色 ID、Key、超时 |

聊天预设包括千问/百炼、DeepSeek、Kimi、MiniMax、OpenAI，也可填写其他兼容服务。
预设是地址与协议默认值，不代表每个供应商的每个模型都已真实验收；能力取决于选定模型。
ASR/TTS 当前只有官方 DashScope SDK 适配，可分别使用不同账号/Key。
本轮已验证 `qwen-plus`、`qwen-audio-3.1-asr-flash-message`、
`qwen-audio-3.1-tts-flash` 和音色 `longanhuan_v3.1`。
语音 URL 与聊天 compatible-mode URL 不同；从服务平台取得所属区域/业务空间地址，不能混用。

点击「保存并使用」会验证配置并开始新对话，第一次发送才请求模型。
已有 Key 留空可保留；改变服务或地址时必须重新填写 Key，避免把旧凭据带给其他服务。
失败配置不会替换当前可用配置；处理中的回合需先停止并等待排空。

网页配置只放在该浏览器会话对应的服务端内存，Key 不回显、不写文件、URL 或浏览器存储。
同一会话的标签页共享配置；独立浏览器会话各自隔离。刷新页面仍可使用当前会话；
「清除本次配置」、服务重启、或一小时空闲后会清除服务端凭据。
会话最多 16 个。浏览器会话 cookie 为 HttpOnly / SameSite=Strict；关闭浏览器不会通知服务端立刻回收。
若启动时显式使用下述文件预配置选项，新浏览器会话会继承操作者提供的初始配置。

当前上位机只绑定本机 loopback，这是本地使用链接，尚未部署公网服务。
将来托管部署应复用同一页面配置契约，另补部署鉴权、TLS 与租户权限；不能直接把该硬件上位机暴露公网。

## 怎么交流

1. 输入文字并发送；Enter 发送、Shift+Enter 换行。例如「先查看状态，再列出可用姿态」。
2. 选择「语音输入」或点击「开始说话」，允许本站麦克风权限，再点击「结束并发送」。
   切换工作台页面、隐藏标签页或停止时取消录音；每段最长约 29 秒。
3. 「语音回复」开启时播放 TTS；浏览器阻止自动播放时点击播放器。没有 TTS 也能文字对话。
4. 输入区「停止本轮」取消录音、识别、推理与播报；已连接设备时同时请求原控制器停止。
   原有「释放扭矩 / 急停」也会取消所有 Agent 会话，真实设备停止仍以控制器反馈为准。

录音期间先停止播放；处理期间不接受新的普通输入，没有待重放队列。
切换配置或停止后，旧输入、迟到 ASR、模型结果和音频不能重新生效。
「语音双工」明确标为待接入，当前是按回合录音上传，不是持续监听、实时转写或全双工 Omni。
语音停止仍要经过云识别，有网络延迟；本地停止按钮独立于云端。

浏览器将音频转换为 mono / 16-bit / 16kHz PCM WAV，服务端再次检查格式和时长。
只有最终转写进入工具循环。录音和模型输入会传给你选择的云服务。
音频最长 30 秒；TTS 每次最多 1000 字符，超限保留文字并报告合成错误。
TTS 失败不重跑模型或重复工具。页面与 API 只返回配置元数据和 Key 是否设置，不返回 Key。

## 模块边界

| 模块 | 职责 |
|---|---|
| `policy/model_io` | 标准模型协议、消息、流式与凭据解析 |
| `policy/agents` | 工具循环、状态、事件、取消与有界上下文 |
| `applications/agent/speech.py` | 云语音契约、WAV 校验、期限和本地取消 |
| `applications/agent/_dashscope_speech.py` | 官方 DashScope SDK 薄适配 |
| `applications/desktop/agent.py` | 同一 Agent 的文字/语音回合、消息与音频代次 |
| `applications/desktop/agent_config.py` | 内存配置、原子替换、浏览器会话与凭据隔离 |
| `applications/desktop/assets/agent-settings.js` | 独立配置表单，不负责机器人控制或密钥持久化 |
| `applications/desktop/assets/agent.js` | 输入、主动录音、采样转换、播放与停止 |

原生 SDK 存在全局 Key/URL 和阻塞方法，每次语音调用在独立 spawn 进程运行。
凭据只在内存传递，不进入命令行或临时配置；取消/超时后终止并回收本地工作进程。
本地取消不保证远端立刻停止计算或计费。SDK 原始错误不透传，不自动重试或切换服务。
端侧识别仍可通过 stdin 提供文本，与云语音解耦，没有其他本机项目或目录依赖。

## 可选 CLI 与文件验收

批处理或操作者预配置仍兼容显式文件，网页使用不依赖这些文件。
聊天文件生成见[多服务配置指南](agent_providers.md)；语音复制
`configs/agent/speech.example.toml` 并填写自己的业务空间、模型与凭据环境变量名。
Key 放在所选环境变量或被 Git 忽略的本机 .env；不提交凭据。显式 .env 不插值、不搜索其他目录。

```sh
# 可选：用操作者文件预配置新浏览器会话
uv run --locked --extra speech python scripts/arm_desk.py --http-port 8765 \
  --agent-config configs/agent/provider.local.toml --agent-env-file configs/agent/.env.local \
  --speech-config configs/agent/speech.local.toml --speech-env-file configs/agent/.env.local

# 只检查文件配置与凭据，无网络
uv run --locked --extra speech eai-course speech --config configs/agent/speech.local.toml \
  --env-file configs/agent/.env.local check

# 固定短句真实 TTS → ASR，最多三轮，新输出目录，拒绝覆盖
uv run --locked --extra speech eai-course speech --config configs/agent/speech.local.toml \
  --env-file configs/agent/.env.local probe --rounds 3 --output-dir outputs/speech-check-001
```

另有 `tts --text TEXT --output FILE.wav` 与 `asr --audio FILE.wav`，显式选择文件才上传。
已有 Conda 环境的合作者用该环境的 uv 按相同锁安装 `speech` extra 后，可直接运行 Python 入口；
环境准备见[Conda 指南](environments.md#conda-quick-start)，不维护第二套依赖清单，也不要求安装 Conda。

真实测试记录见[云语音报告](reports/2026-10-24-cloud-speech.md)。合成语音通过不代表麦克风噪声、
相机理解、真实双工或实物任务验收。音频和本机完整日志不提交。
