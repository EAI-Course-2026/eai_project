# 视觉伺服应用

本轮真机基线是ArUco中央区域MVP。职责按相机、数值控制、硬件流程和结果分析拆分：

| 入口/模块 | 职责 |
|---|---|
| `aruco-preview` / `aruco_preview.py` | 相机唯一所有者；`--headless --stream-file`后台发布，不打开串口 |
| `shared_camera.py` | 时间戳帧适配、250ms新鲜度检查、可恢复的断帧异常 |
| `vision` / `ball.py` | 保留历史入口；检测、参数配置、相机/串口生命周期与显式执行门禁 |
| `joint_center.py` | 实测像素/编码器响应、单步反馈、真实/部分运动、输入丢失恢复、循环记录 |
| `fixed_rule_servo.py` | 固定规律（五区域/前向1cm）真机执行：四角点观测、有界下发、实测反馈折叠 |
| `object_servo.py` | 同一规律作用于任意跟踪目标：跟踪器装配、守卫适配、参考距离尺度、重捕获；**尚未真机执行** |
| `hold_preview.py` | 正常暂停时持续预览；R重试，已支撑后回车释放；EOF不卸扭矩 |
| `profiles.py` | 加载可复用参数；不允许配置执行授权、接管或本机设备字段 |
| `reporting.py` | 只读JSONL汇总，无相机或设备依赖 |
| `cli.py` | `visual-servo`演练：简单模式/四角点IBVS/选定物体光流；不执行真机动作 |
| `experiment.py` | `aruco-experiment`的标定链路与受限实验；尚未完成精密真机验收 |

纯数值逻辑位于 `control/visual_servo/empirical.py`、`camera_step.py` 与 `fixed_rule.py`；
相机/机器人雅可比和IBVS已有实现位于
`control/visual_servo/joint_adapter.py`、`aruco_ibvs.py`。不要把MVP的固定计数模式改名为精密IBVS。

当前参数：`configs/visual_servo/aruco_center_mvp.toml`（回中）、
`configs/visual_servo/aruco_fixed_rule.toml`（固定规律）、
`configs/visual_servo/object_rule_example.toml`（任意物体，示例深度值需重测）。
运行与停机步骤：`docs/course/aruco_visual_servo.md`。
本轮证据：`docs/reports/2026-10-18-visual-servo-round1.md`。
固定规律的设计与离线证据：`docs/experiments/visual_servo_fixed_rule.md`（真机未执行）。
任意物体扩展：`docs/experiments/visual_servo_any_object.md`；
模型接入：`docs/experiments/perception_infrastructure.md`；
待决策项：`docs/experiments/object_servo_decisions.md`。
精细实验计划：`docs/experiments/visual_servo_precision.md`。
