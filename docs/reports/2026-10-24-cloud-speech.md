# 云语音与上位机 Agent 首轮验收

日期：2026-10-24（Asia/Shanghai）。复用用户本机配置，不提交 Key、工作空间地址、账户额度或原始音频。
测试经用户授权，只用固定文本与合成音频；未采集麦克风、打开相机或连接机械臂。

## 真实服务与结果

- ASR：`qwen-audio-3.1-asr-flash-message`，百炼北京原生 WebSocket。
- TTS：`qwen-audio-3.1-tts-flash`，系统音色 `longanhuan_v3.1`，输出 16kHz mono PCM 后包装 WAV。
- 聊天：`qwen-plus`，Chat Completions，显式关闭思考；OpenAI SDK 2.54.0。
- 语音 SDK：锁定 DashScope 1.27.8，作为 `speech` extra，不改变训练环境或安装端侧模型。

固定短句 TTS → ASR 三轮，标点归一化后逐句相同，3/3 通过：

| 输入 | TTS 首包 ms | TTS 完整请求 ms | ASR 完整请求 ms |
|---|---:|---:|---:|
| 请查看机械臂的当前状态。 | 535 | 1060 | 925 |
| 不要移动，先告诉我有哪些姿态。 | 751 | 1649 | 807 |
| 停止当前任务。 | 473 | 1014 | 802 |

请求耗时包含本机 spawn、SDK 和网络开销，非纯模型计算时间；ASR 采用合成文件一次上传，
不包含人说话/录音时间。三次样本不能推导稳定 p95、噪声表现或服务 SLA。
SDK 返回实际 usage，已存本机报告；它不是零费用声明。

上位机真实 HTTP 入口另完成：

1. 文字「先查看状态，再列出可用姿态，用一句话回答」→ 模型实际调用
   `get_robot_state`、`list_poses` → 明确回复 dry_run → TTS 音频可取，约 5.60 秒。
2. 已合成的状态查询音频 → ASR 最终文本 → 模型实际调用 `get_robot_state` → TTS，约 4.50 秒。
3. 已合成的停止句 → ASR → 本地停止旁路，约 1.02 秒；无新增工具调用，不经过 LLM。

这验证的是原生语音适配、真实聊天工具选择与 UI 同一后端，不只是 Codex 手动调用工具。
当前工具结果均 simulated，没有电机控制或实物反馈。浏览器已显示文字回复、工具状态、
麦克风按钮、停止与音频播放器；浏览器阻止自动播放时提供手动播放入口。

## 尚未覆盖

真实麦克风采集/噪声/回声与现场听感、腕部相机观测、自然打断、连续识别、全双工 Omni、
目标跟踪/IK 和实物自主任务均未验收。本轮不宣称作业 4.3 已全部完成。
本机完整音频、usage 与请求级报告保存在被忽略的 `local_artifacts/speech/`。
离线测试和 PR/合并后 CI 证据分别记录在关联 PR，不能与真实服务结果相互替代。

接入依据：[ASR Python SDK](https://help.aliyun.com/zh/model-studio/qwen-asr-message-python-sdk)、
[TTS Python SDK](https://help.aliyun.com/zh/model-studio/qwen-audio-tts-python-sdk)。
页面 API 配置补验：浏览器中打开「API 接入」，保留已有密钥并保存，创建新对话，
再从聊天框请求状态查询，验证页面配置与同一真实工具/语音链路。配置 API 离线回归另验证
独立浏览器会话隔离、密钥不回显、无效更新原子回退、旧运行实例不能提交与全局停止旁路。
默认启动不需要配置文件；文中的文件仅为本次操作者已有凭据来源。

启动、配置与边界见[上位机 Agent](../desktop_agent.md)。
