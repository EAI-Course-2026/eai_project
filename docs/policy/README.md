# ACT、SmolVLA、π0.5 推理基建

2026-10-03。应用层实现位于 `src/eai_robot/policy/`，所有入口使用
`python scripts/policy.py`。导入、诊断、下载、离线推理和服务器都不连接硬件。
SO101 连接不等于模型、相机或任务已验收；验收及阻塞见
[本次交接](next_session.md)。前置方案仍在未合并 PR #10 的 `codex/vla-handoff`。

## 当前可用范围

| 路径 | 实现和证据 | 下一道真实门槛 |
|---|---|---|
| ACT | 真实框架构造、保存、严格重载、官方处理器及有限动作块的回归 | 本机任务示范及训练 checkpoint；随机权重测试不代表任务能力 |
| SmolVLA | 固定基础 checkpoint、冻结 VLM/tokenizer、Mac MPS 离线推理 | 本机视角、单位/顺序/标定证据及任务适配 |
| π0.5 | 固定 fork 模型导入、元数据审计、严格加载适配器 | NVIDIA 主机、模型访问许可、本机数据和匹配 checkpoint |
| 跨机 | 回环 HTTP、身份校验、限长 JSON、客户端；实际 SmolVLA 服务检查 | GPU 主机和 SSH 隧道实测；网络指标不能由本机替代 |
| 执行 | 有限 headless runner、会话/时序/步长/健康/反馈门禁、写前复查 | 真实只读 shadow、操作者和明确运动授权 |
| 数据 | 键盘人工录制、请求/实际发送/反馈分离、标准本地 LeRobot 导出 | 相机、具体任务、操作者成功标注和独立评估场景 |
| 训练 | 隔离依赖锁、GPU 诊断、经数据检查生成三种模型的训练参数 | GPU/显存和有效数据；本轮不宣称微调成功 |

## 环境和离线入口

控制根项目的依赖和锁保持原样。训练项目在固定 fork 上增加 `smolvla,pi` extras；
ACT 无单独 extra。首次同步及纯软件诊断：

```sh
uv sync --locked
uv sync --locked --project environments/training
uv run --locked --project environments/training python scripts/policy.py doctor
# NVIDIA 主机必须执行；没有 CUDA 时返回失败，不退化成 GPU 验收通过
uv run --locked --project environments/training python scripts/policy.py doctor --require-cuda
uv run --locked python scripts/policy.py dry-run --manifest configs/policy/fake.json
```

训练环境不安装第二份应用；脚本从该 checkout 的 `src/` 加载应用模块。
框架工厂、pre/post processors 和 Robot 仍是固定 fork 的代码。实际权重加载前检查
安装来源 SHA、checkpoint 配置、全文件 SHA256 和外部资产。

```sh
# 元数据即可发现单位、维度、视角或统计缺口
uv run --locked --project environments/training python scripts/policy.py fetch \
  --model lerobot/smolvla_base --revision d9f33c94a60fb382c90dea2164c96845bd955e28 \
  --directory checkpoints/smolvla_base --weights
uv run --locked --project environments/training python scripts/policy.py dry-run \
  --manifest checkpoints/smolvla_base.manifest.local.json --device mps \
  --warmup 1 --repeats 5 --output outputs/policy/smolvla-mps.local.json
```

基础模型生成的 deployment manifest 默认 `state_units/action_units/action_mode=unknown`、
`motion_verified=false`。它只允许合成/明确准备的离线样例。合成三视角只是软件输入，
不代表拥有三台相机。NPZ 实例需 `state` 六维以及各 `source` 对应的 uint8 HWC RGB 图像；
通过 `--observation data/sample.npz --task '...'` 使用真实离线样例。

`fetch` 把所有远程 revision 解析为完整 commit。独立 `training_base_bundle` 记录基座
与外部资产的文件清单，允许基座使用其他 embodiment。32 维 π0.5 基座不能生成本机
deployment manifest；需在六维本机数据上适配，再导出匹配的推理 checkpoint。
不要把 32 维截取或补零当作机器人适配。

现有本地训练 checkpoint 用 `freeze --checkpoint ... --revision CONTENT_OR_COMMIT_SHA
--action-hz ... --output configs/policy/model.local.json` 冻结。VLA 另用 `--assets assets.json`
指定本地冻结资产，格式参考 `fetch` 生成的清单；目录必须在 checkpoint 内。
清单包括权重、config、处理器配置、统计与 tokenizer/VLM 文件。运行时不下载资源。
文件或设置变化后重新冻结、重新验收，不能只修改摘要掩盖变化。

## 模型处理与动作契约

六轴顺序固定 `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`。
部署 state 与 postprocessed action 使用 `scs215_calibrated_percent`：前五轴 -100..100，
夹爪 0..100，绝对目标。校验 `calibration_sha256`，并在 `compatibility_evidence`
引用匹配示范、动作定义、夹爪方向和视角的验收记录。`motion_verified=true` 只能在这些
证据具备后设置；这是本地审查门槛，不是模型自动给出的可信结论。

应用将 HWC RGB 转成模型键对应的 CHW 0..1 图像，按声明 shape resize；官方模型
自己的 padding/resize 仍按 checkpoint 执行。官方处理器唯一负责 batching、tokenizer、
统计标准化与反标准化，应用不重复应用均值/方差。内部 relative action 由官方配对处理器
恢复成绝对目标；应用不另外加 state。每次请求是独立完整 chunk，清空旧缓存/anchors。
当前只支持单帧 observation、普通 chunk；RTC、ACT temporal ensemble、MEM 和
ALOHA 单位变换需另行实现调度/适配。

π0.5 固定 fork 的 `from_pretrained` 会捕获权重错误后返回模型。应用的严格加载适配器
复用官方 key remapping，但任何文件/键错误都会失败，防止随机或部分权重进入部署。
不修改 site-packages。π0.5 的量化统计要求以 checkpoint 的实际处理器为准；本项目
训练配方默认检查 `q01/q99`，缺失时阻断，不偷偷改 normalization mode。

## 跨机部署

推理服务器只绑定 GPU 主机 `127.0.0.1`。双方通过私下配置的环境变量
`EAI_POLICY_TOKEN` 使用至少 32 字符随机共享秘密；不要把它写进 Git、命令历史或报告。
用已有 SSH 登录建立隧道，不把推理服务公开到公网。

```sh
# GPU 主机：先通过 doctor --require-cuda 和该模型的 CUDA dry-run
uv run --locked --project environments/training python scripts/policy.py serve \
  --manifest configs/policy/model.local.json --device cuda --port 8081
# Mac：GPU_HOST 必须替换为实际已授权 SSH 主机
ssh -N -L 8081:127.0.0.1:8081 GPU_HOST
uv run --locked python scripts/policy.py dry-run \
  --manifest configs/policy/model.local.json --remote http://127.0.0.1:8081 \
  --observation data/sample.npz --task 'TASK' --output outputs/policy/remote.local.json
```

两端的 manifest 内容及 fingerprint 必须一致。使用相同相对目录布局复制 bundle/manifest，
Mac 无须为纯客户端安装 VLM，也无须读取模型文件。复制内容不要更改 manifest 中的路径；
machine hardware config 是另一份本机文件。图像以有界 RGB bytes 的 JSON/base64 传输，
请求最大 32 MiB；没有 pickle，也没有远程 Robot 对象。当前为同步重规划，先验证
单请求时延与本地执行；多客户端吞吐、异步队列和 RTC 尚未实现。

服务端不发放运动期限。客户端的 session、sequence、manifest 与原请求关联；观测时间、
往返计时和执行 deadline 全部使用 Mac `perf_counter`。停止后旧响应不能恢复会话，
网络断开、超时、乱序或未知身份都清空本地执行队列。进程不存在自动重连后续跑。

## 真实观测与执行

本轮没有打开串口/相机、启用扭矩或执行模型运动。先补本机配置与经过审查的契约：

1. 复制 `configs/policy/hardware.example.json` 为同目录 `hardware.local.json`，填写
   serial、共享 calibration 路径及 camera source → 实际设备映射。
2. 逐一确认真实视角、RGB、分辨率、帧率及夹爪方向；不能把同一物理相机映射成两路。
3. 核对模型/data 的归一化单位与标定摘要。基础模型未知契约不能开启真实 shadow。
4. 复制 `budgets.example.json` 为本地文件，用实际采集/网络/GPU/控制证据确定预算。
   示例数值只用于软件防护演示，未做实物验收，不能为消除错误任意放宽。
5. GUI 与其他进程先断开。只有明确运行 `shadow/run/record-demo` 才连接设备。

```sh
uv run --locked python scripts/policy.py shadow \
  --manifest configs/policy/model.local.json \
  --hardware-config configs/policy/hardware.local.json \
  --budgets configs/policy/budgets.local.json --task 'TASK' --duration-s 10 \
  --remote http://127.0.0.1:8081 --output outputs/policy/shadow-001
```

shadow 仅反馈/图像读取，connect 使用 `enable_motion=false`，退出不释放其他程序留下的
扭矩。相机时间来自固定 OpenCV 捕获线程的 `latest_timestamp`，在 frame lock 内一起
复制图像；它是软件获取时间，不能冒充曝光硬件时间。关节读取保留起止时间区间，检查
最老图像/关节年龄和采集跨度。实物同步与传感器时延需另外测量。

有限 rollout 需操作者在场、已验证紧急停止、明确当次授权、匹配 checkpoint，且支持
机械臂退出/故障释放。命令最多 60 秒，结束和 Ctrl-C 都清队列并关闭本地 Robot。
断开会释放本进程启用的扭矩；这可能让机械臂下落，操作者必须能支撑它。

```sh
uv run --locked python scripts/policy.py run \
  --manifest configs/policy/model.local.json \
  --hardware-config configs/policy/hardware.local.json \
  --budgets configs/policy/budgets.local.json --task 'TASK' --duration-s 5 \
  --remote http://127.0.0.1:8081 --output outputs/policy/rollout-001 \
  --enable-motion --acknowledge MATCHED_CHECKPOINT_WITH_OPERATOR_STOP
```

同一硬件线程拥有 Robot，先检查进程占用，策略进程之间用进程锁排他；Unix 还设置
TIOCEXCL 与 serial exclusive，Windows 使用串口原生独占。GUI 没有被迁入新 runner，
部署时仍必须先断开 GUI。执行前检查首步/块内变化、范围、扭矩、报警、跟踪误差与
期限；串口逐轴核验后、真正同步写目标前再检查期限。目标寄存器回读与编码器反馈
分别验证，超时不会 burst-send 追赶。错误停止本次运行，不自动启用或恢复旧任务。
步长约束不提供避碰，也不验证几何模型精度。

完整 50 步/30 Hz SmolVLA 动作块跨度约 1.67 秒，不能通过 0.5 秒的示例有效期。
本机 MPS 离线 p95 约 440 ms，也超过 250 ms 示例 response timeout。当前 runner
会如实拒绝它；后续用匹配 checkpoint 的短 `n_action_steps`、GPU 测量和任务评审选择
重规划范围。不得通过盲目延长 deadline 消除陈旧观测问题。

## 人工示范、导出与训练

只有 follower 时，可用 `record-demo` 进行关节键盘采集。不是现有 Cartesian 键盘或 GUI
的旁路 recorder，也不要求购买 leader。它复用唯一 Robot 与动作门禁；键盘线程仅更新
意图，实际 I/O 在硬件线程。按键租期 300 ms，不续期即清除方向；可能需要重复按键。
Q/A、W/S、E/D、R/F、T/G、Y/H 依次增/减六轴，ESC/Ctrl-C 停止。步长为预算的半值。

采集契约可复制 `fake.json` 成 `collection.local.json`，改为实际相机键/shape、采样 Hz
和共享 calibration SHA256，保留 `motion_verified=false`。`fake` 表示没有策略模型；
只有人工采集的独立确认入口允许启用它，自主 `run` 始终拒绝假策略。

```sh
uv run --locked python scripts/policy.py record-demo \
  --manifest configs/policy/collection.local.json \
  --hardware-config configs/policy/hardware.local.json \
  --budgets configs/policy/budgets.local.json --task 'TASK' \
  --scene-id scene-train-a --duration-s 30 --output data/episodes/demo-001 \
  --enable-motion --acknowledge OPERATOR_DEMONSTRATION_WITH_STOP
```

退出释放后操作者输入 `SUCCESS` 才标注成功。没有成功标注、缺发送/后续反馈、相机
跨度过大、时序断裂或未完成的 episode 不能导出为示范；shadow/rollout 也不能自动
充当人工示范。每帧独立保存无 pickle NPZ，JSONL 记录 capture、request、sent、feedback、
feedback_at、rejection；权重、数据、视频和个人配置都被忽略。

```sh
uv run --locked python scripts/policy.py validate-episode \
  --manifest configs/policy/collection.local.json --episode data/episodes/demo-001 --demonstrations
# split.local.json 示例：{"train":[0],"evaluation":[1]}；必须按 episode 且 scene_id 不交叉
uv run --locked --project environments/training python scripts/policy.py export-dataset \
  --manifest configs/policy/collection.local.json \
  --episodes data/episodes/demo-001 data/episodes/demo-eval-001 \
  --split data/split.local.json --root data/lerobot/task --repo-id local/task
uv run --locked --project environments/training python scripts/policy.py training-recipe \
  --policy-type act --dataset-root data/lerobot/task --repo-id local/task \
  --output-dir outputs/train/act --output outputs/policy/train-act.local.json
uv run --locked --project environments/training python scripts/policy.py training-recipe \
  --policy-type smolvla --dataset-root data/lerobot/task --repo-id local/task \
  --base-bundle checkpoints/smolvla_base.bundle.local.json \
  --output-dir outputs/train/smolvla --output outputs/policy/train-smolvla.local.json
```

π0.5 使用相同 `training-recipe --policy-type pi05 --base-bundle ...`，需完整冻结的 π0.5
基座、tokenizer 许可和 q01/q99。配方只输出 argv，不启动训练，不上传数据。起始 batch=1、
20k steps 是待调整的实验参数，不是保证效果；按显存和 held-out 结果调整。基座配置、
相机键、`n_action_steps` 和归一化必须评审；`pretrained_path` 加载权重，与部署 `path`
加载完整 checkpoint 设置不同。记录 seed、数据 fingerprint、训练/评估场景与模型 SHA。

## 复验和限制

```sh
uv run --locked python -m unittest discover -s tests -q
uv run --locked python scripts/check_env.py
uv run --locked python scripts/check_repository.py
uv run --locked --project environments/training python scripts/check_training_env.py
```

离线测试包含真实 ACT checkpoint、官方标准化一次、真实回环 HTTP、门禁故障、
模拟人工录制、标准 LeRobot 本地数据导出和 π0.5 加载错误。托管 Windows 训练软件
检查也导入三种模型；它没有 GPU。本项目没有 CUDA 性能、真实同步数据、模型任务
成功率或运动验收证据。Mac OpenCV/PyAV 会输出已有 AVFoundation 类重复告警，本轮
无硬件捕获；保留日志，在真实相机验收时核验，未通过改安装文件掩盖它。

设计依据按固定源码优先，动态官方文档作参考：
[ACT](https://huggingface.co/docs/lerobot/main/act)、
[SmolVLA](https://huggingface.co/docs/lerobot/main/smolvla)、
[π0.5](https://huggingface.co/docs/lerobot/main/pi05)。用户提供的
[飞书 π₀ 示例](https://zihao-ai.feishu.cn/wiki/XufXwJF3xiqcK4kg2wNciVgEnjg)
是另一设备/任务的部署经验，未执行其中删除缓存、修改设备权限或自动实物推理命令。
