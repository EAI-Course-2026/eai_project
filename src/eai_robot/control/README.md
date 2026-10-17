# 在线执行控制与控制律

`arm_controller.py` 实现 SCS215 接管、插值、回读、超时和停止；
`demo.py` 执行动作阶段，`robot_feedback.py` 供 LeRobot 路径复用反馈检查。
`aruco_ibvs.py` 和 `visual_math.py` 只做图像控制律计算，不连接设备。
这里不组织网页或课程命令。不同 Robot 执行路径尚未合并为一个控制器，既有门禁仍须保留。

完整分工与兼容规则见 [`src/README.md`](../../README.md)。
