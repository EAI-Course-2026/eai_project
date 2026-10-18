# 图像观测

| 功能组 | 分工 |
|---|---|
| `detection/` | aruco.py 标记检测/位姿估计，ball.py 黑球检测。 |
| `tracking/` | optical_flow.py 对显式选定区域做光流跟踪，不静默更换目标。 |
| `targets/` | black_ball.html 是展示/打印目标图案。 |
| `features.py` | 检测/跟踪共享观测契约，保留在能力父目录。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
