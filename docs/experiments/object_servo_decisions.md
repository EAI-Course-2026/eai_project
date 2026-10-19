# 任意物体伺服：需要你决策的清单

状态：**骨架已落地，尚未真机执行**。以下每一项都已有默认值，机器可以跑；列出来是因为
它们决定后面真机验收的口径，改起来越晚越贵。每项给出「现状/默认」「为什么需要你定」
「改了会怎样」。

## A. 目标选择：第一批要支持哪种目标

| 选项 | 现状 | 影响 |
|---|---|---|
| `support`（区域纹理，无模型） | 已实现，随时可跑 | 零依赖，但需要你先给 `--roi`；且必须能重新框选 |
| `dark-blob`（阈值+轮廓） | 已实现，`--tracker fused --selection score` 可直接跑 | 只能追深色紧凑物体；用来验证"管路"而不是验证"能力" |
| `mediapipe-hands`（21 关键点） | 目录已登记，**未实现** | 手是形变体，光流在手指张合时会漂；这是最需要"检测器仲裁"的场景 |
| `owlv2` / `yolo-world`（开放词汇框） | 目录已登记，**未实现** | 通用性最好，但只有框，`sqrt_area` 会随物体旋转变化 |
| `sam2`（可提示分割） | 目录已登记，**未实现** | 掩膜最稳，`sqrt_area` 最诚实，但最慢 |

**需要你定**：第一批做哪一个（建议 `mediapipe-hands`，因为它同时检验 coasting 与
仲裁两条路径，而且权重随 wheel 附带、不需要下载）。其余可以先不实现，目录报文已经能
让任何人看懂缺什么。

## B. 权重策略

现状：默认 `allow_download = False`。权重缺失时报
`TrackerUnavailable: weights for 'sam2' not found at <path>`，不会联网。

**需要你定**：

1. 实验室机器能否联网？不能联网就必须**提前把权重复制到**某个位置。
2. 权重放在哪里？建议 `checkpoints/perception/`（仓库已忽略该目录，不进源码）。
   缓存目录（`~/.cache/huggingface`）不建议，因为它不可移植也不可审计。
3. 是否允许 `--allow-weight-download`？允许之后"同一次实验"可能加载不同的字节，
   记录里必须把权重哈希写进去。

改法：实现检测器时读 `ModelSpec.weights`；策略层不用改。

## C. 设备

现状：`--model-device auto` 在本机解析为 `mps`（已实测可用设备为 `cpu, mps`）。
解析结果会写进日志，日志里不会出现"auto"。

**需要你定**：真机验收时允许用 `mps` 吗？`mps` 上有的算子会回退到 CPU 且不报错，
延迟可能比预期高一倍。若要求"验收必须在 CPU 上可复现"，就固定 `--model-device cpu`。

## D. 延迟预算与 coasting

现状：`--tracker-max-coast-s 0.2`。相机 15fps 即每帧 66.7ms，0.2s 约 3 帧。
超过窗口没有新检测就报 `lost` 并保持。

**需要你定**：

1. 能不能接受"用最多约 3 帧之前的检测定位、期间由光流给出每帧位姿"？
   这就是 `fused` 模式的正常工作方式。若不接受，只能用 `--tracker detector`，
   但那时每一步的位置都来自模型，延迟直接变成运动延迟。
2. 0.2s 够不够？对 250ms 的 SAM2 意味着会有约 1 帧的空窗（250/66.7≈3.75 > 3），
   会出现 `lost`→`reacquire` 的抖动。要么放宽到 0.4s（代价是允许更旧的位姿），
   要么选快模型。

## E. extent 约定：真机标定时按哪个量测参考距离

现状：约定由跟踪器决定，并随几何一起记录（`side` / `diameter` / `sqrt_area` / `span`）。
`ExtentScale` 把约定写进自身，约定不一致时决策层直接 hold，不会算出一个"看起来对"的深度。

**需要你定**：做 `--reference-depth-m` 标定时，以**哪一个**量读数？
建议直接用日志里 `acquired` 事件的 `extent_px`，不要另算——那正是运行时会用的量。
若你换成 `--target-extent-m`，就必须说清是边长、面积平方根还是直径。

## F. 距离尺度误差的容忍度

现状：深度只由一次测量决定。`参考距离 300mm ±10mm` 就是 `±3.3%` 深度误差，
在 300mm 处约 ±10mm。停靠判据仍然用像素（120px），不依赖这个数。

**需要你定**：能不能接受"深度只用于报告与门限，真停靠靠像素"？
若能，现在的实现就够；若不能（例如要按毫米停靠），需要加测距或标定流程，
那是另一轮工作。

## G. 验收阶梯：到哪一步算通过

建议沿用固定规律的三级，不跳级：

| 级 | 内容 | 判据 |
|---|---|---|
| L1 离线 | 合成纹理跟踪、融合状态机、约定配对 | 已在 `tests/` 覆盖 |
| L2 只读 | `--execute` 不开，跑 `--tracker fused --detector dark-blob`，看状态机与预览 | 不出现无法解释的 `lost`；`reanchors` 有界 |
| L3 真机单轴 | 只允许一个方向，`--rule-max-steps 3`，`--max-travel-mm 30` | 每步实际关节响应与 `achieved` 一致 |
| L4 真机完整 | 五区域完整跑，任意物体 | 停靠像素达标，全程无守卫拒绝 |

**需要你定**：L3 先做哪个方向？建议先做**左右**（1cm≈5计数，落在死区边界，
最能暴露"请求不可执行"），再做**前向**（会走抬升+手腕配对）。

## H. 是否允许对象在伺服过程中动

现状：规则假设目标是静止的（采样阶段取多帧中位数）。手会动。

**需要你定**：第一批验收用静止物体还是手？
用静止物体可以先把"任意形状"跑通；用手则必须先回答一个当前实现没有回答的问题——
"追一个正在动的手"是否算成功，以及允许多大的滞后。

## I. 记录与回放

现状：日志在 `local_artifacts/experiments/*_object-rule.jsonl`，每个事件都带
`tracker` 快照（状态、重捕获次数、重锚次数、跳过次数、检测器 p50/p95/丢弃/失败）。

**需要你定**：这些日志要不要进 `docs/reports/`？现在的约定是本机产物不入库，
只有报告入库。若要入库，需要先定"哪些字段可以公开"。

## 不改就能跑的最小命令

```bash
# 只读，不扭矩；先在预览里确认跟踪是否稳
eai-course vision --camera-index 0 --object-rule \
  --tracker fused --detector dark-blob --selection score \
  --reference-depth-m 0.30 --preview

# 参数化到配置文件里跑
eai-course vision --profile configs/visual_servo/object_rule_example.toml \
  --camera-index 0 --preview

# 看有哪些检测器、缺什么依赖、延迟量级
eai-course vision --list-detectors

# 不装任何模型，只按区域跟纹理
eai-course vision --camera-index 0 --object-rule \
  --tracker support --roi 280,200,80,80 --reference-depth-m 0.30 --preview
```
