# 目标计算与执行控制

| 功能组 | 分工 |
|---|---|
| `kinematics/` | 关节映射、URDF、FK/IK、几何路径；只做数值计算。 |
| `planning/` | poses.py 命名关节目标；demo.py 固定起点/动作计划；cartesian.py 笛卡尔目标适配。 |
| `execution/` | arm.py 接管与在线运动，feedback.py 回读检查，demo.py 阶段执行。 |
| `visual_servo/` | aruco_ibvs.py、math.py 为图像控制数学；joint_adapter.py 为相机/关节雅可比适配；empirical.py 为MVP纯数值响应选择和反馈判据；controller.py 检查观测时效与目标身份。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
