# Windows 上位机 Agent 麦克风与停止人工验收

日期：2026-10-24（Asia/Shanghai）。本记录是源码基线 `1d5e742` 上的一次 Windows 人工烟雾验收，
由仓库协作者在本地浏览器中执行。记录只保存脱敏结论，不保存 API Key、业务空间 ID/API Host、
代理地址、原始录音、账户额度或完整服务响应。

## 环境与软件基线

- Windows AMD64，uv 0.11.7，uv 管理的 Python 3.12.13。
- LeRobot 0.6.2，源码提交 `6a077907c7989635218969ee78f5436f8faec92b`。
- 根控制环境使用 PyTorch 2.11.0+cpu；CUDA、相机和 FFmpeg 不属于本次验收。
- 接入前基线 `python -m unittest discover -s tests -q`：587 项，133.946 秒，全部通过。
- Agent 工具清单为 `get_robot_state`、`list_poses`、`move_to_pose`、
  `get_task_status`、`stop_motion`；运行模式为 `dry_run`，`hardware_connected=false`。

## 真实服务与人工步骤

| 项目 | 配置与操作 | 观察结果 |
|---|---|---|
| 文字与工具 | `qwen-plus`，Chat Completions，关闭思考；发送“先查看状态，再列出可用姿态” | 模型依次调用 `get_robot_state`、`list_poses`；页面显示 running → simulated，并明确回复当前为模拟模式及四个命名姿态 |
| 浏览器麦克风与 ASR | 操作者允许本机页面麦克风权限，录制一段状态查询并结束发送；`qwen-audio-3.1-asr-flash-message` | 操作者确认最终转写成功进入同一 Agent 回合；本次没有保存原始音频、逐字转写或请求级指标 |
| TTS | `qwen-audio-3.1-tts-flash`，音色 `longanhuan_v3.1`；开启语音回复并发送简短状态查询 | 页面保留文字，操作者确认音频生成和播放成功；未观察到模型或工具重复调用 |
| 停止本轮 | 在较长状态/姿态请求处于活动阶段时点击“停止本轮”，随后发送新回合 | 操作者确认旧文字和旧音频没有迟到回灌，新回合仍可正常执行 |

所有模型结果只使用 dry-run 工具数据。本次没有连接串口、启用扭矩、打开相机或发送运动命令；
因此它不是机械臂实物验收，也不证明任何姿态已经在物理设备上完成。

## Windows 取消测试日志修正

同一 Windows 环境运行完整测试时，取消/期限回归会让客户端主动关闭 loopback HTTP 连接。
测试服务器随后写响应，在 Windows 上可能得到 `ConnectionAbortedError: [WinError 10053]`；
测试本身仍以 587 项 `OK` 结束。测试 helper 原来只忽略 BrokenPipe 和 ConnectionReset，
本任务把 ConnectionAborted 也视为预期的客户端断开，避免成功测试打印误导性的后台 traceback。
修正后精准回归 1 项通过且没有后台 traceback；完整套件 587 项、116.590 秒，全部通过。

## 仍未覆盖

- 麦克风权限拒绝、无设备、占用和切换设备路径。
- 固定口令集的多轮准确率、否定词、背景噪声、远场、回声及 TTS 自激。
- ASR/TTS p50/p95、请求 usage、限流、断网、超时及额度耗尽行为。
- 持续 ASR、VAD、部分转写、自然打断、真正的语音双工。
- 腕部视觉、检测/跟踪、IK、真实反馈和任何 Agent 实物执行。

本记录把“真实麦克风单次链路已通”与上述尚未完成的 P0/P1 验收分开。
首轮合成音频、真实工具调用和云语音适配证据见
[云语音与上位机 Agent 首轮验收](2026-10-24-cloud-speech.md)，操作步骤与边界见
[上位机 Agent 使用手册](../desktop_agent.md)。
