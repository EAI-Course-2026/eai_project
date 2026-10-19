# 感知基础设施：模型怎么接进来

状态：**已落地并可离线验证；除 `dark-blob` 外没有实现任何模型检测器**。
本文描述的是"接入一个模型需要什么、边界在哪、缺什么会怎么报错"，不是模型精度评测。

## 1. 一条硬约束

控制包必须在一台**没有 mediapipe、没有 transformers、没有权重**的机器上照常导入、
照常跑全部控制测试、照常驱动机械臂。

这不是洁癖。上一轮的真机验收依赖的是一条能跑通的路径；如果某次 `uv sync` 因为一个
视觉可选包失败而让整条路径装不上，问题就从"模型不好用"升级成"机械臂不能用"。

**因此这些模型家族不进 `[project.optional-dependencies]`。** 这一条最初写反过：
先按 extra 做了一版（`perception-hands = ["mediapipe..."]` 等），结果
`uv run --locked` 直接解析失败——`segment-anything-2` 在 PyPI 上不存在（SAM 2 只能从
git 装），而 `uv` 会把所有 extra 一起锁进 `uv.lock`，于是包括伺服回路在内的所有人
都装不上。这与 `docs/environments.md` 里"根 `training` extra 已被移除"是同一条教训：
extra 必须是真能解析的，否则它不是一个可选组，而是一个全局故障点。

现在的做法：依赖**不提**在 `pyproject.toml`，而是随注册表条目携带确切安装命令，
并且只在"真要用这个检测器的人"这一侧执行：

| 名称 | 安装命令 | 上游约束 |
|---|---|---|
| `mediapipe-hands` | `uv pip install "mediapipe>=0.10,<0.11"` | 自带 NumPy 约束，装前确认本机 numpy 2.2.6 不被降级 |
| `owlv2` | `uv pip install "transformers>=4.44,<5" torch` | 需另一个 PyTorch 栈 |
| `sam2` | `uv pip install "git+https://github.com/facebookresearch/sam2"` | 仅 git 分发，PyPI 无此包 |
| `yolo-world` | `uv pip install "ultralytics>=8.2"` | ultralytics 会拉整套 torch |

`uv sync` 不带任何额外步骤时，装进来的模型依赖数量是零。将来某个检测器稳定到可以锁，
再升级成 extra 也不迟——在那之前，lockfile 只描述机械臂。

## 2. 缺依赖时要报什么

`vision/perception/registry.py` 的目录是"缺什么"的唯一真相来源。目录里登记五条候选，
每条声明它的安装命令、返回什么证据、延迟量级、是否需要权重：

| 名称 | 已实现 | 安装 | 证据 | 延迟量级 | 权重 |
|---|---|---|---|---|---|
| `dark-blob` | 是 | 不需安装（NumPy + OpenCV） | mask | 30ms | 无 |
| `mediapipe-hands` | 否 | `mediapipe>=0.10,<0.11` | keypoints | 30ms | 随 wheel |
| `owlv2` | 否 | `transformers` + `torch` | box | 350ms | 需下载 |
| `sam2` | 否 | `git+https://github.com/facebookresearch/sam2` | mask | 250ms | 需下载 |
| `yolo-world` | 否 | `ultralytics>=8.2` | box | 80ms | 需下载 |

延迟是**量级预算**，用来给 `AsyncDetector.health` 提供一个比较基准，不是本机实测。

请求一个"已登记但未实现"的检测器，得到的是可执行的句子而不是 traceback：

```
TrackerUnavailable: detector 'sam2' is catalogued but not implemented:
promptable segmentation (SAM 2) for a stable mask and extent.
Install with: uv pip install "git+https://github.com/facebookresearch/sam2".
weights are not bundled; loading them needs an explicit
--allow-weight-download and the hash goes into the record.
Implement 'sam2' against ObjectDetector and call
register('sam2', factory) in eai_robot.vision.perception.registry.
```

命令行在参数校验阶段就把这一条拦下来，不会等到连上机械臂才失败：

```
$ eai-course vision --object-rule --tracker fused --prompt cup \
    --reference-depth-m 0.3 --detector sam2
ball.py: error: detector 'sam2' is catalogued but not implemented yet
(install: uv pip install "git+https://github.com/facebookresearch/sam2").
Use --tracker support with --roi, or --detector dark-blob.
```

`eai-course vision --list-detectors` 打印同一张表，并附上本机可用设备。

## 3. 权重策略

三条规则，都在 `vision/perception/spec.py` 里：

1. **不隐式下载。** `ModelSpec.weights` 指向一个路径；文件不存在且
   `allow_download=False` 时直接报 `TrackerUnavailable` 并给出该路径。
2. **允许下载是显式的。** `--allow-weight-download` 才会联网。开着它时，"同一次实验"
   可能加载不同字节，所以记录里必须写权重哈希——这是实现检测器时的责任。
3. **权重不进源码。** 仓库只留 `checkpoints/` 目录（已忽略）。缓存目录
   （`~/.cache/huggingface`）不可移植、不可审计，因此不作为默认位置。

## 4. 设备策略

`--model-device auto|cpu|mps|cuda`。`auto` 只在**加载时解析一次**，把结果写回规格对象，
所以日志里出现的是 `mps`，不会是 `auto`。在本机实测可用的设备是 `cpu, mps`。

要求一个不可用的设备会抛错，而不是静默回退：

```
RuntimeError: device 'cuda' is not available here (have ('cpu', 'mps'))
```

静默回退是延迟悬崖的常见来源：一次运行慢十倍，日志里却写着"和上次一样"。

## 5. 异步推理：单槽，不排队

`vision/perception/async_detector.py`。相机 15fps 即每帧 66.7ms；一个 250ms 的分割模型
比相机慢 3.75 倍。这里只有两个选项：让控制回路等它，或者不等。

不等。规则如下：

| 决定 | 理由 |
|---|---|
| **最新帧覆盖，不排队** | 队列在模型比相机慢时会无界增长，回路拿到的永远是十帧前的位姿；覆盖让检测延迟等于模型延迟 |
| **丢弃是数字** | 被覆盖的帧计入 `dropped`，日志里能看到，而不是"感觉有点卡" |
| **失败是数据** | 模型抛异常不进入运动回路：批量丢弃、`failures` 加一、`latest` 保留上一次成功结果、`health.degraded` 变真 |
| **帧是借用，不复制** | 每帧复制 640×480 RGB 的开销不比部分检测器小；契约要求调用方交出一个不会改写的帧，仓内每个相机适配器每次读取都新分配数组 |
| **等待只在两处** | `wait_for_first`（捕获一次，有界）与 `wait_for_frame`（重新锚定，有界）。运动回路用 `latest()`，永不阻塞 |

`health()` 给出 `p50 / p95 / budget / degraded`。`degraded` 的判据是连续失败达到
`failure_limit`，或 p95 超过预算的 2 倍——这两个数让调用方可以缩短 coasting 窗口，
而不是无限相信一个旧框。

## 6. 新增一个检测器要做什么

四步，没有第五步：

1. 写一个类满足 `ObjectDetector`：

```python
class MyDetector:
    name = "my-detector"
    latency_budget_s = 0.05          # 量级预算，用于 health 比较
    def detect(self, frame_rgb) -> list[Detection]: ...
    def describe(self) -> dict: ...  # name / weights / device / input_size
    def close(self) -> None: ...     # 幂等
```

2. 在 `CATALOGUE` 里加一条 `DetectorEntry`（名称、安装命令、证据、延迟、是否需权重、任务）。
3. 在 `registry._load_builtins()` 里 `register("my-detector", build)`；工厂接收
   `ModelSpec`，返回实例。**惰性导入**：`import mediapipe` 只写在工厂函数体内，
   不写在模块顶层。
4. 加测试：`registry.create` 能拿到实例、`describe()` 的 `name` 与目录一致、
   缺权重时抛 `TrackerUnavailable`。

`Detection` 只需要一个框；有掩膜或关键点就给，`to_geometry()` 会优先用更诚实的证据：

| 证据 | 表观尺寸 | 什么时候对 |
|---|---|---|
| `mask` | `sqrt(像素面积)` | 最诚实：分割唯一真正观测到的量，长条旋转不变 |
| `keypoints` | 关键点凸包四边形的平均边长 | 手：关键点本身就定义轮廓 |
| `box` | `sqrt(宽×高)` | 兜底：给旋转的长条定框，面积会随姿态变 |

三者都归一到"等面积正方形边长"，因此一个目标从 box 切到 mask 时深度尺度连续。

## 7. 这个基础设施不宣称什么

- 不宣称任何模型的精度或延迟。表里的延迟是量级预算，本机实测值要另外测。
- 不宣称 `dark-blob` 能替代模型。它是**管路的参考检测器**：先把异步、几何转换、
  融合、固定规律整条链路跑通，之后任何回归都能归因到模型，而不是归因到管路。
- 不宣称多目标选择是"智能"的。`select_detection` 只有三条明确策略
  （`score` / `area` / `central`），并且在无提示、无区域时必须由操作者显式指定，
  不允许默认"取分最高的"——那正是走向错误物体的方式。
