# 图像观测与标记位姿

`ball_detector.py` 和 `aruco.py` 检测图像特征；给定内参后可估计标记位姿。
相机预览和黑球闭环流程在 `applications/`，纯视觉伺服控制律在 `control/`。
复杂内参、手眼采集与求解新增在同级 `calibration/`，原 `vision/calibration/` 只保留说明。
检测器不创建设备执行会话，也不自动重新标定。

完整分工与兼容规则见 [`src/README.md`](../../README.md)。
