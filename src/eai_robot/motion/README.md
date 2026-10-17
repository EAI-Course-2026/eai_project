# 动作目标与命名姿态

`poses.py` 加载标定绑定的命名关节姿态，`poses.reference.json` 是默认参考数据。
`demo.py` 校验固定起点并生成演示阶段，`cartesian.py` 将编码器反馈和几何目标接合。
本目录不创建设备、不启用扭矩、不管理 HTTP/键盘事件；实际执行由应用调用 `control/`。
几何 FK/IK 和逐点可达性仍在 `kinematics/`。

完整分工与兼容规则见 [`src/README.md`](../../README.md)。
