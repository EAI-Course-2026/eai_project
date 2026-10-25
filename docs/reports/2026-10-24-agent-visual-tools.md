# Agent 视觉工具与外部 LLM 验收

日期：2026-10-24。基线：`origin/develop` 的 `1d5e742`；开发分支：
`codex/agent-visual-tools`。本报告只证明固定 ArUco 场景的 dry-run 工具闭环，
不证明相机、机械臂或真实视觉伺服已经接入 Agent。

## 本轮实现

在既有标准 function calling 循环中新增三个工具：

- `detect_object(description)`：只识别 ArUco fixture，返回会话绑定且五秒有效的 `object_id`；
- `start_tracking(object_id)`：校验会话、期限和并发状态，创建模拟追踪任务；
- `stop_tracking()`：停止当前模拟追踪任务，可重复调用。

`get_robot_state` 同时报告追踪状态，既有 `get_task_status` 可查询任务。
所有视觉结果都带 `mode=dry_run` 或 `hardware_sent=false`。不支持的物体描述、跨会话 ID、
过期 ID 和并发追踪会明确拒绝。LLM 只能选择这些工具，不能直接生成电机控制量。

应用 prompt 要求先读取状态、先检测再追踪、只使用最新返回的 ID，并区分
`stop_tracking` 与全局 `stop_motion`。完整 prompt 见
[`system.txt`](../../src/eai_robot/applications/agent/prompts/system.txt)。选做的最简操作规程见
[`aruco-visual-tracking/SKILL.md`](../../.cursor/skills/aruco-visual-tracking/SKILL.md)。

## 可重复的离线演练

```sh
uv run --locked eai-course agent --visual-demo
```

固定演练顺序为：

```text
get_robot_state → detect_object → start_tracking → get_task_status → stop_tracking
```

此命令不调用模型、OpenCV、相机或串口，适合课堂展示前先检查工具状态机。

## 外部 LLM API 验收

使用 OpenAI Python SDK 2.54.0、Responses API 和外部模型 `gpt-6-luna`。
Key 只从已有的 `OPENAI_API_KEY` 进程环境读取，没有写入仓库、配置或日志；
SDK 自动重试保持为 0。实际用户 prompt 为：

> 先查看机器人状态，检测 ArUco marker，启动追踪，查询任务状态后停止追踪。
> 最后明确说明是否操作了真实相机或机械臂。

模型自主完成了以下调用，而不是由 Codex 逐个执行：

| 序号 | 模型选择的工具 | 本地结果 |
|---:|---|---|
| 1 | `get_robot_state` | `simulated / ok` |
| 2 | `detect_object` | `simulated / ok` |
| 3 | `start_tracking` | `simulated / ok` |
| 4 | `get_task_status` | `simulated / ok` |
| 5 | `stop_tracking` | `simulated / ok` |

本轮共 6 次模型请求、5 次工具调用，端到端 17.094 秒；API 报告 input 7,722、
output 184、合计 7,906 tokens。最终状态为 `completed`，模型明确回答：全程没有操作真实相机或机械臂。

第一次命令在发出 API 请求前被本机 SOCKS 代理依赖缺失拦截，表现为脱敏的
`protocol_error`；没有自动重试。显式清除该进程的代理环境后重新发起一轮，得到上述成功证据。
过程中还发现复合 prompt 中的“停止追踪”会被旧语音停止词规则抢占，已改为只有独立的
本地停止口令才绕过模型，并增加回归测试；独立“急停/停止当前任务”仍走本地取消。

## 边界与下一步

- 当前 `detect_object` 是确定性 fixture，不读取相机，也不支持任意物体描述。
- `start_tracking` 只创建内存任务；真实视觉伺服仍由既有独立应用负责。
- 本轮没有串口、相机、扭矩或电机动作，也没有新增真机执行权限。
- 下一阶段若接真机，应新建受控技能适配层，复用控制器租期、反馈、限位和停止路径，
  先完成仿真/回放，再由操作者按硬件手册验收。

## 软件检查

- Agent 专项：21 项通过；视觉生命周期、过期/跨会话 ID、真实 loopback 工具回传和本地停止分流均覆盖。
- 完整离线测试：592 项通过。
- `check_env.py`、`check_repository.py`、三个前端 JavaScript 语法检查和 `git diff --check` 通过。
- 根包与 SCS215 plugin 的 wheel 构建通过。

完整测试中的设备、相机和运动输出来自 fake/fixture 测试对象；本轮没有打开真实串口或相机。
