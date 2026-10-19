# 图像观测

| 功能组 | 分工 |
|---|---|
| `detection/` | aruco.py 标记检测/位姿估计，ball.py 黑球检测。 |
| `geometry/` | targets.py 把四角点/圆/包围盒/掩膜/关键点统一成一个目标几何；scale.py 把表观尺寸换成距离。纯 numpy，无相机无模型。 |
| `perception/` | spec.py 模型规格与设备解析；registry.py 检测器目录与缺依赖报文；async_detector.py 单槽异步推理；builtin.py 零依赖参考检测器。 |
| `tracking/` | optical_flow.py 对显式选定区域做光流跟踪，不静默更换目标；support_points.py 四点加单一相似变换的任意物体跟踪；fused.py 用检测器监督光流，带时长上界的 coasting。 |
| `targets/` | black_ball.html 是展示/打印目标图案。 |
| `features.py` | 检测/跟踪共享观测契约，保留在能力父目录。 |

`vision/` 根目录只放 `__init__.py` 与 `features.py`，其余实现进第二级分组。
`geometry/` 与 `tracking/` 可以被 `control/visual_servo/` 导入（`geometry/` 只依赖 numpy）；
`perception/` 依赖可选模型包，只有应用层实例化它。

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
任意物体伺服的设计与验收阶梯见
[任意物体伺服](../../../docs/experiments/visual_servo_any_object.md)，
模型接入的基础设施与决策项见
[感知基础设施](../../../docs/experiments/perception_infrastructure.md)。
