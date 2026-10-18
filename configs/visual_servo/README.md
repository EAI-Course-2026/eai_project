# 视觉伺服参数配置

`aruco_center_mvp.toml`固化2026-10-18真机演示配置，仅用于中央区域跟随。
`aruco_fixed_rule.toml`是固定规律（五区域+前向1cm）参数，**仅离线验证，尚未真机执行**；
它的内参/手眼路径不在配置里，由命令行默认指向 `calibration/vision/` 的实测文件。
`object_rule_example.toml`是同一规律作用于**任意跟踪目标**的示例，同样**仅离线验证**；
它的 `reference_depth_m = 0.30` 是占位示例值，真机前必须自己量。
通过`eai-course vision --profile PATH`加载，显式CLI参数覆盖配置。

配置只接受经过类型检查的servo参数；不能设置`execute`、`adopt_hold`、串口、相机索引或
本机临时文件路径。`[perception]` 段同样是白名单，而且**不接受权重路径与标定路径**：
权重是本机字节，让配置指向它就等于让"同一份配置"在另一台机器上意味着另一个模型。
不开`--execute`时仍是只读相机预览。配置100mm/40°描述本轮模型预算，
不代表未知场景的物理安全空间；新一轮重新核验姿态和现场。

固定规律的设计、离线数值证据与真机分级验收见
[固定规律伺服](../../docs/experiments/visual_servo_fixed_rule.md)。
任意物体扩展的架构、状态机与验收阶梯见
[任意物体伺服](../../docs/experiments/visual_servo_any_object.md)，
模型接入与依赖隔离见[感知基础设施](../../docs/experiments/perception_infrastructure.md)，
需要操作者拍板的项见[决策清单](../../docs/experiments/object_servo_decisions.md)。
精密IBVS尚未验收，暂不提供名称看似可直接真机运行的precision配置。
结果见[本轮报告](../../docs/reports/2026-10-18-visual-servo-round1.md)，
下一步见[精细伺服计划](../../docs/experiments/visual_servo_precision.md)。
配置读取与字段白名单见[profiles.py](../../src/eai_robot/applications/visual_servo/profiles.py)。
