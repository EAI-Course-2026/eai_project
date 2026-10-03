# 推理基建交接与待用户输入

2026-10-03。先读[完整操作指南](README.md)，再检查当前分支、PR head 与 CI。
实现分支 `codex/policy-infrastructure` 从 develop `be9ac819fed847ad2a00245c7e569d8a7c417146`
创建；没有合并 PR #10，没有推 main/develop，也没有代替另一位维护者审查。
实现已提交到[草稿 PR #11](https://github.com/EAI-Course-2026/eai_project/pull/11)，托管检查
需核对该 PR 最新 head；未合并，最终托管回执与本地结果分别报告。
历史方案与旧 handoff 已从 `codex/vla-handoff` 读完，保留在 PR #10。

## 已完成

- 冻结框架 SHA，保持根控制环境，训练环境增加锁定 `smolvla,pi`。
- `contracts.py`：manifest、完整文件摘要、单位/顺序/标定/视角边界。
- `artifacts.py`：固定 Hub revision 的下载、训练基座 bundle、部署清单、元数据报告。
- `observations.py/runtime.py`：RGB/state/task → 官方 preprocessor → 模型 chunk → 官方
  postprocessor；ACT 与 SmolVLA 已实际运行。π0.5 采用严格加载适配，不吞掉权重错误。
- `transport.py`：认证、限长 JSON、只绑定 loopback 的服务器与轻量客户端，SSH 隧道拓扑。
  真实 SmolVLA 在训练进程推理，根控制进程作为网络客户端已跑通。
- `safety.py/execution.py`：本地 session/sequence/期限、连续性、范围、反馈、扭矩和报警门禁，
  本地单硬件线程、串口排他、有限时长 headless runner，目标写前再检查租期。
- 插件动作路径补扭矩丢失检查，以及可选 `before_write` 回调；旧 API 调用兼容。
- `teleoperation.py/recording.py/datasets.py`：follower-only 人工键盘示范、实际发送标签与
  后续反馈、标准 LeRobot 本地导出、episode/scene 训练评估隔离，三模型训练 argv 生成。
- `scripts/policy.py`：doctor/fetch/checkpoint-info/freeze/dry-run/serve/shadow/run/
  record-demo/validate-episode/export-dataset/training-recipe/label-episode/evaluate-episodes。
- `evaluation.py`：操作者标签及历史、全部试验分母、未标注成功率待定、拒绝原因与
  归一化目标反馈误差/采集跨度/tick 间隔汇总；不从可动推断任务成功。
- 真实 ACT 保存重载、官方标准化一次、回环 HTTP、门禁故障、人工录制、标准数据导出、
  π0.5 加载错误都有离线回归。Windows 训练软件 CI 增加三模型导入诊断。

## 实际证据

| 检查 | 结果 | 边界 |
|---|---|---|
| ACT | 小型真实框架 checkpoint 保存、严格重载、3×6 推理通过 | 随机初始化测试；无任务能力/性能结论 |
| SmolVLA MPS | 1 warmup + 5 次；50×6 finite；p50 0.434945 s、p95 0.439557 s | 三路合成图像与六维合成 state，无实物 |
| SmolVLA CPU | 1 warmup + 3 次；50×6 finite；p50 2.105317 s、p95 2.127410 s | 同一固定权重及合成输入；不是实时控制 Hz |
| SmolVLA 本机 HTTP | 1 warmup + 5 次；50×6 finite；客户端 p50 0.482806 s、p95 0.497973 s | 回环服务，不能当作跨机网络/GPU结果 |
| 三模型环境 | ACTPolicy、SmolVLAPolicy、PI05Policy 均能导入 | 没有 CUDA，软件检查不能当 GPU 验收 |
| CUDA 必需诊断 | 当前 Mac 按预期失败 | 需真实 NVIDIA 主机与实际算子/模型检测 |
| 硬件/相机 | 未打开，未启用扭矩，未运动 | 本机 camera 配置为空；串口存在只作枚举证据 |

本地完整测试 182 项通过（其中 policy 28 项、插件新增 2 项故障回归）；根环境检查、
训练环境检查、训练入口、文档链接/仓库策略检查、前端语法与两个 wheel 构建通过。
本地结果与 PR 托管结果分开；尚未合并，故没有 post-merge CI 结果。

SmolVLA 官方 revision：`d9f33c94a60fb382c90dea2164c96845bd955e28`，基础配置为六维 state/
action 与 camera1/2/3 三视角。外部 VLM/tokenizer 的完整 revision 和文件 SHA256 已保存在
忽略的本地 manifest/bundle。权重、资产和测量报告留在 `checkpoints/`、`outputs/policy/`。

π0.5 官方 revision：`b211f3d44c36b6acfcf7ae94a64e8e96f75a64ba`。元数据为 32 维 state/action，
三相机；基础处理器没有本机六维 normalization features/stats。仅下载元数据，未加载
大型 π0.5 权重。不能截取前六维，也不能据基础模型配置声称 SO101 已兼容。

## 现在必须由用户补齐的条件

| 方向 | 下一步已经准备好 | 必须补充的具体输入 |
|---|---|---|
| GPU | locked sync、doctor --require-cuda、CUDA dry-run、服务端 | 现成主机地址/授权连接方式、OS、GPU/显存/驱动；若要租赁，选择和费用授权 |
| 任务 | 人工采集、成功标注、场景隔离、ACT/Smol/π训练配方 | 首个明确桌面任务、成功判据、物体/布局、允许的运动范围和操作者 |
| 相机 | 显式源映射、RGB/layout、时间戳/跨度/年龄检查 | 实际摄像头设备及视角；是否只有单路，能否增加腕部视角；硬件现场可验收时间 |
| 权重/数据 | fetch/freeze、标准导出和训练配方、检查清单 | 已有 checkpoint/数据路径和 provenance；没有则安排匹配示范采集 |
| 契约 | 未知单位/顺序/标定或错误统计均阻断 | 标注数据动作顺序、归一化、夹爪方向、camera keys、采样 Hz 与标定关系 |
| 运动 | shadow → 有限 run，操作停止/反馈/期限回归 | 相机/模型匹配后，当次受控实物执行明确授权与在场操作者 |
| 合并 | develop PR、完整托管检查 | 另一位维护者的实际代码审查；代理不替代批准 |

用户只需回复可用 GPU/访问、首个任务、相机、已有数据/权重这四组条件；如均未定，先
决定一个低复杂度桌面任务与相机配置，再进行人工示范。本轮未购买设备、创建付费
GPU、接受模型许可证、上传数据，或在不明运动条件下开启自主 rollout。

## 推荐接续步骤

1. GPU 条件到位：同一 checkout 同步训练锁，doctor --require-cuda、模型 CUDA dry-run，
   记录显存、warmup/p50/p95，然后用 SSH 隧道做端到端延迟测试。
2. 相机与任务到位：建立本机 collection manifest/hardware config，先验证只读采集跨度、
   RGB/帧率，实物 operator record-demo。无需 leader，但键盘示范质量需真实任务检查。
3. 建立独立场景评估集；导出标准 LeRobot 数据，检查 q01/q99，先 ACT 基线，再用相同
   数据适配 SmolVLA、π0.5。冻结完成后的六维 checkpoint 与完整处理器。
4. 确认数据单位/夹爪/视角/标定证据后才能改 deployment manifest 的未知字段。
   先真实只读 shadow，再由操作者授权小范围有限 rollout。
5. 单独记录任务成功次数/总数、失败分类、发送与反馈误差、控制 Hz、相机 FPS、推理 Hz。
   当前报告不能提供任务成功率、GPU 性能或同步传感器精度。

示例 250 ms 响应预算与 500 ms 动作 TTL 会拒绝当前 50-step MPS 流程。这是门禁应有行为；
用短执行 horizon、实测 GPU 与任务评审确定预算。不要扩大标定、屏蔽反馈、伪造相机或
盲目增大期限来宣称“已打通实物”。
