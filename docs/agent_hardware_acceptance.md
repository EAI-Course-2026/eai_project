# Agent 真实机械臂验收

本页只验收已经有真机证据的 **ArUco ID 0 中央区域回中 MVP**。它不授权任意物体、
自动前进、精密四角点、姿态运动或 IK。默认 Agent 仍是 dry-run；只有本页的完整命令会进入真机模式。

## 现在处于哪一步

| 项目 | 状态 |
|---|---|
| 外部 LLM 选择视觉工具并接收结果 | 已在 dry-run 验收 |
| 真机工具适配层与停止/释放流程 | 已完成软件和假硬件测试 |
| 持续可见的腕部相机与共享帧 | 2026-10-25 Windows 现场通过 |
| 外部 LLM 驱动真实 ArUco 回中 | 2026-10-25 上位机语音端到端通过 |

本次通过范围是：语音输入 → ASR → 外部 LLM 选择白名单工具 → ArUco 0 检测 →
受限视觉伺服 → 编码器确认运动与回中 → Agent 语音回复。原始关节日志保存在被忽略的
`local_artifacts/experiments/`；共享报告只记录脱敏结论。任意物体、自动前进、精密四角点、
持续语音双工和 IK 仍未因此获得授权。

真机流程为：

```text
外部 LLM → get_robot_state → detect_object（只开相机）
         → start_tracking（才会开串口、启用扭矩）
         → get_task_status → stop_tracking（停止运动、保持扭矩）
         → 现场人员托住机械臂 → 本地确认 → 释放扭矩
```

LLM 只能选择工具，不能填写关节角、电机计数、串口寄存器或扩大运动预算。真机后端只接受仓库中
未修改的 `configs/visual_servo/aruco_center_mvp.toml`；运行时长另由 5–120 秒的本地参数限制。

## 第一阶段：不连接机械臂的软件检查

在仓库根目录运行：

```powershell
uv run --locked eai-course agent --visual-demo
uv run --locked python -m unittest discover -s tests -q
```

第一条必须出现 `detect_object`、`start_tracking`、`get_task_status`、`stop_tracking`，并明确
`hardware_connected=false`。这一步不证明真机可用。

## 第二阶段：现场准备

开始前必须全部满足：

1. 机械臂固定在稳定桌面，运动范围内没有人、线缆和易碎物；ArUco ID 0 标记在相机视野内。
2. 一人操作终端并能立即切断舵机电源；释放扭矩时有人托住机械臂。
3. 关闭上位机、键盘控制、标定和其他占用同一串口或相机的进程。
4. API 配置和 Key 已按[多服务配置](agent_providers.md)保存在被忽略的本机文件或环境变量中。
5. 不使用猜测的串口和相机编号。Windows 可先查看串口：

```powershell
Get-CimInstance Win32_SerialPort | Select-Object DeviceID,Name
uv run --locked lerobot-find-cameras opencv
```

把实际结果记为下文的 `COMX` 和 `N`；不要把本机编号提交到 Git。

## 第三阶段：启动持续可见的共享相机

在第一个 PowerShell 中启动相机唯一所有者。它会打开一个 Tk/Pillow 预览窗口，同时把新鲜帧
发布到被忽略的本机文件；它不连接机械臂串口：

```powershell
$sharedFrame = "local_artifacts/tmp/agent-visual/latest.frame"
uv run --locked eai-course aruco-preview `
  --camera-index N --marker-id 0 --stream-file $sharedFrame
```

确认画面能稳定识别 `DICT_4X4_50` 的 ID 0，并把标记完整放在画面内、与边缘保留约一成画面短边的探测余量。标记不必位于中央一半区域。按 `Q` 或 `Esc`
可结束预览，但真机 Agent 验收期间应让本窗口和发布进程持续运行。Agent 只读取共享帧，不能再直接
打开相机。如果相机打不开、画面卡住、标记跳变或目标不是 ID 0，到此停止，不进入下一阶段。

## 第四阶段：外部 LLM 真机工具调用

只在前三阶段通过且现场人员就位时，在第二个 PowerShell 中运行。`provider.local.toml`、`.env.local`、`COMX` 和 `N`
替换成本机真实值；不要把 Key 直接写进命令：

```powershell
uv run --locked eai-course agent `
  --provider-config configs/agent/provider.local.toml `
  --env-file configs/agent/.env.local `
  --text "先查看状态，检测 ArUco marker，启动中央区域追踪并查询状态。启动后保持追踪，不要调用停止工具。" `
  --visual-hardware-profile configs/visual_servo/aruco_center_mvp.toml `
  --device-port COMX --camera-index N `
  --shared-frame $sharedFrame `
  --hardware-max-runtime 30 `
  --enable-visual-motion `
  --acknowledge ARUCO_CENTER_MVP_WITH_OPERATOR_STOP
```

缺少动作开关、完整确认口令、串口、相机编号或交互式终端时，程序会在打开设备前拒绝运行。
桌面窗口和相机采集始终留在第一个前台进程；后台 Agent 不创建 Tk 窗口，也不直接占用相机。
工具只有返回 `tracking=running`、`hardware_sent=true` 且 `motion_confirmed_count>0`，才证明至少一条
受限关节命令已得到编码器运动确认。`calibrating`、`moving` 或只有扭矩开启都不等于追踪已启动。
如果模型没有调用停止工具，单回合终端会继续等待本地追踪线程，直到运行时长上限、控制器进入保持
或操作者按 `Ctrl+C`；不会因为模型先完成文本回答而立即关闭刚启动的追踪。

需要停止时按 `Ctrl+C` 或发出独立的“急停”。软件停止后会保持扭矩；先托住机械臂，再按终端提示
输入完全一致的 `RELEASE TORQUE`。若停止超时、终端无法读取确认或释放失败，继续托住机械臂，
立即按实体急停或切断舵机电源，不要自动重试。

## 通过标准与记录

一次合格验收需同时记录：日期、操作者、代码提交、模型/API、实际串口和相机（仅本机记录）、
目标初始位置、工具调用顺序、运动方向、停止方式和异常。以下条件全部满足才可标为真机通过：

- 相机检测返回 `found=true`、正确 marker ID，且检测阶段 `hardware_sent=false`；
- 模型使用刚返回的 `object_id` 调用追踪，没有生成控制量；
- 机械臂只做中央区域回中，方向正确且未越出既有运动预算；
- 状态查询反映真实运行状态；
- 停止后不再发送运动，托稳后释放扭矩，最终 `torque_enabled=false`；
- LLM 的最终回答与工具结果一致，不把 `starting`、`holding` 或失败说成完成。

每次换设备、配置或控制逻辑后仍须重新执行本页验收，不能沿用 2026-10-25 的结论。
把脱敏结论写入日期报告；原始日志和视频放在被忽略的 `local_artifacts/validation/`，
不要提交凭据、本机端口、视频或个人绝对路径。
