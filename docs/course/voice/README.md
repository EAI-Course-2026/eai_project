# 语音命令调度（白名单意图 + 姿态调度）

2026-10-12。课程命令层实现位于 `src/eai_robot/voice/`，入口为
`uv run --locked eai-course voice`。默认干跑：导入、解析、计划与预览都不打开
串口；也可用 `uv run --locked python scripts/course.py voice` 从源码启动。运动只在 `--enable` 且显式确认后发生。

本层衔接课程的语音边界（[交接文档第 9 节](../source/HANDOFF.md)）：

```text
音频输入 -> 语音识别 -> 白名单意图 -> 动作调度 -> 统一控制器
```

识别（音频输入与转写）不属于本仓库：本地识别链路的搭建与实测记录在
`edge_inference_lab` 的 Confucius4-R2T2 项目，本仓库只接收「每条一行」的识别文本。
识别结果绝不会被当作代码执行，只能映射到下表白名单。

## 当前可用范围

| 能力 | 实现与证据 | 下一道真实门槛 |
|---|---|---|
| 白名单解析 | 7 条固定命令、中英文关键词、标点归一化、否定词拒绝、`stop` 优先；27 项离线回归 | 真实口音、噪声下的识别文本覆盖率实测 |
| 动作调度 | 意图 → 姿态序列（`wave` 复用既有 `raise/left/right/left/stand`），缺姿态整体拒绝 | 真机逐命令验收与动作幅度确认 |
| 停止抢占 | `stop` 置标志、丢剩余步骤与挂起队列；读线程可即时抢占；[回归用例](../../../tests/test_voice.py)覆盖 | 电机运动中的实际中断时延测量 |
| 干跑/预览 | 不打开串口打印计划与逐姿态原始目标换算 | 无（软件检查） |
| 夹爪命令 | 命令已定义，参考姿态文件尚无 `gripper_open/gripper_close`，当前整体拒绝并反馈 | 用既有 `record-pose` 记录夹爪姿态后启用 |

尚未验证：真机运动、夹爪姿态、连续多命令节流、真实识别置信度（当前 ASR 不提供
置信度，低置信度近似为“无关键词命中 → 只反馈”）。

软件端到端干跑证据（2026-10-12，合成语音，非实物验收）：识别端
`mic_asr.py --emit text` 的输出经管道进入本层，依次得到 `挥手`（执行中）、
`停止`（抢占 2/5 步后丢弃剩余）、`站立`、`向左`，以及白名单外文本
`打开冰箱门。` 被忽略且无动作；全部事件写入 JSONL。合成语音只证明软件链路，
不代表真实口音与噪声下的识别覆盖率。

## 命令白名单

| 命令 | 中文关键词（节选） | 姿态序列 | 参考姿态文件是否具备 |
|---|---|---|---|
| `stand` | 站立、站好、起立、归位、复位 | `stand` | 是 |
| `wave` | 挥手、招手、打招呼 | `raise -> left -> right -> left -> stand` | 是 |
| `move_left` | 向左、往左、左移、转到左边 | `left` | 是 |
| `move_right` | 向右、往右、右移、转到右边 | `right` | 是 |
| `open_gripper` | 张开夹爪、打开夹爪、松开夹爪 | `gripper_open` | 否，需先记录 |
| `close_gripper` | 闭合夹爪、抓住、抓取、夹住 | `gripper_close` | 否，需先记录 |
| `stop` | 停止、停下、停住、别动、急停 | 无（抢占并保持当前姿态） | 不适用 |

解析规则（`src/eai_robot/voice/intent.py`）：

1. `stop` 最先判定；一句话同时含停止词与其他命令词时按 `stop` 处理。
2. 中文字关键词允许作为子串命中；纯 ASCII 关键词必须与整句完全一致，
   避免 `open the door` 之类整句误触发夹爪。
3. 含否定词（不要/不用/不能/别/勿）的文本一律拒绝，只反馈不驱动。
4. 空文本、白名单外文本只打印反馈，不产生动作计划。

扩大白名单属于安全边界变更：必须同时改 `COMMANDS`、`POSE_SEQUENCES` 和回归用例，
并经另一位维护者审查。

## 停止语义

- `stop` 立即置停止标志，丢弃剩余步骤与挂起命令；运动模式下通过控制器的
  `motion_guard` 在写寄存器前抛出中断，粒度是单个姿态动作。
- 停止不关闭扭矩：机械臂保持当前姿态。关闭扭矩需先支撑机械臂，见
  [上位机操作指南](../../experiments/scs215_desk.md)的释放约定。
- 停止后新的普通命令重新开放调度；执行序列期间到达的普通命令被拒绝并反馈，
  避免连续意外运动。
- 与会话门禁一致：停止后不恢复旧计划，不存在“断开/超时后继续跑”的路径。

## 使用

```sh
# 单条文本，干跑
uv run --locked eai-course voice --text "挥手"
# 打印姿态到原始目标的换算表（离线预览，不打开串口）
uv run --locked eai-course voice --text "站立" --preview
# 从管道读取识别结果，每条一行
edge_inference_lab ... mic_asr.py --mode live --emit text | uv run --locked eai-course voice --stdin
# 事件留证（JSONL，每行一条命令与会话结果）
uv run --locked eai-course voice --stdin --events outputs/voice/session.jsonl
# 真机运动：先断开 GUI 与其他控制进程，支撑机械臂，再显式确认
uv run --locked eai-course voice --stdin --enable
```

运动模式说明：

- 交互终端会要求输入 `START VOICE CONTROL`；非交互（管道）场景必须传入
  `--confirm "START VOICE CONTROL"`。任何缺失都直接拒绝开启，不退化成干跑。
- 复用 `eai_robot.policy.execution.PortLease` 与本机串口检查，保证同一串口只有一个进程；
  退出时关闭扭矩，失败会打印原因。
- 退出码：0 正常、1 未确认或配置缺失、2 参数错误、130 操作者中断。

## 测试

```sh
uv run --locked python -m unittest discover -s tests -p "test_voice.py" -v
```

覆盖：白名单固定性、中文/英文关键词与标点、否定词拒绝、`stop` 优先、
最长关键词命中、姿态缺失整体拒绝、干跑不触碰控制器、执行中并发命令拒绝、
`stop` 抢占与后续恢复、CLI 干跑/预览/事件文件、voice 模块不导入串口或框架。

## 与识别端对接

| 项 | 约定 |
|---|---|
| 文本通道 | 标准输入，每次识别结果一行，UTF-8 |
| 生产端 | `edge_inference_lab` 的 `scripts/mic_asr.py --emit text`（仅输出转写文本，不含识别端自带的演示命令匹配） |
| 本仓库职责 | 唯一权威的白名单解析、姿态计划与停止抢占 |
| 不传递的内容 | 音频、模型权重、识别端路径；本仓库不依赖任何机器本地路径 |

## 真机验收前置清单

1. 复制 `configs/hardware.example.toml` 为 `configs/hardware.local.toml`，填写串口；
   本机文件不入库。
2. 断开上位机 GUI 与其他控制进程；确认同一串口只有一个进程。
3. 支撑机械臂后释放扭矩，记录 `stand/raise/left/right` 之外实际需要的姿态；
   夹爪命令需要先用 `eai-course record-pose gripper_open`、`gripper_close` 记录姿态
   （默认写入被忽略的 `configs/poses.local.json`），并用 `--poses` 指向该文件。
4. 先跑干跑核对计划与目标，再用 `--enable` 单命令逐步验收；一次只验收一个命令。
5. 记录每条命令的实际动作、幅度、停止中断时延与异常；区分软件验证与实物验收，
   未测项不得写成通过。
