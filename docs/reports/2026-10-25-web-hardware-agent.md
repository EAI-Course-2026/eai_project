# 网页真机 Agent 模式 — 2026-10-25

## 结果

上位机 Agent 页面新增显式启用的 ArUco 真机模式。外部 LLM 继续通过标准 function calling
选择 `get_robot_state`、`detect_object`、`start_tracking`、`get_task_status`、
`stop_tracking` 和 `stop_motion`；浏览器本身不生成电机控制量。

网页与工具循环共用一个 `ArucoCenterHardwareBackend`。推荐由独立前台进程持续占有相机、
显示 Tk/Pillow 预览并原子发布带单调时间戳的 JPEG；网页和追踪后端只读取新鲜共享帧。
Windows 读写短暂锁冲突会在既有新鲜度租期内重试，不返回旧帧。后端把带状态标注的画面
通过本机 `/api/agent/frame.jpg` 提供给网页；共享帧和原始日志均留在被忽略的本机目录。

运动语义也从“启用扭矩”细分为命令写入和编码器确认。页面显示共享/直连相机来源、追踪阶段、
运动命令数、编码器确认数和扭矩；只有 `tracking=running`、`hardware_sent=true` 且
`motion_confirmed_count>0` 才允许 Agent 声称真实追踪已经启动。腕部相机运动造成的短暂模糊
不再在同一步编码器到位期间触发目标丢失；到位保持后必须重新获取目标，才允许下一步。

停止按钮取消模型回合并请求停止追踪，但保留扭矩。页面仅在后端报告扭矩启用时开放释放按钮，
并要求现场操作者托住机械臂后输入 `RELEASE TORQUE`。服务退出也调用停止和释放流程。

## 启用边界

普通 `scripts/arm_desk.py` 启动仍为 dry-run。真机模式必须同时提供经过校验的 profile、串口、
摄像头索引、`--enable-visual-motion` 和确认短语。具体 PowerShell 命令与现场顺序见
[上位机 Agent 使用手册](../desktop_agent.md#网页真机-agentaruco-0)。

运行上限覆盖正常循环和目标丢失后的等待阶段；目标没有重新出现时，到期进入扭矩保持，
不会无限等待并绕过 `--hardware-max-runtime`。

## 现场验收

2026-10-25，现场操作者在 Windows 上位机完成端到端验收：

1. 独立窗口持续显示腕部相机，并向 Agent 后端发布共享帧；
2. 页面语音输入经 ASR 形成指令，外部 LLM 自主选择视觉工具；
3. `start_tracking` 发送受限关节命令，编码器记录实际响应，ArUco 0 回到容差区域；
4. Agent 的最终答复经 TTS 在页面播放。

被忽略的本地 `20261025_230723_joint-center.jsonl` 与
`20261025_231349_joint-center.jsonl` 包含 `command`、`motion`、`response` 和 `locked`
事件，支持“实际发送命令并得到编码器确认”的结论。日志、串口、相机编号、API Key 和音频
不进入 Git。该验收不扩展到任意物体、自动前进、精密四角点、IK 或持续语音双工。

## 软件验证

- Agent、桌面、共享相机、Windows 按键和视觉伺服专项回归通过；
- 完整离线测试、环境/仓库检查、前端 JavaScript 语法检查和两个 wheel 构建在本 PR 提交前执行；
- 自动测试使用 fake/fixture，不连接串口、相机或发送真实运动；上述现场记录与软件测试分别声明。

## 未完成范围

尚未验收任意物体跟随、自动前进、精密四角点、IK 工具、跨设备复现、负载/全行程和自然打断式
持续语音双工。更换设备、相机、profile 或控制逻辑后必须重新执行现场门禁，不能沿用本次结论。
