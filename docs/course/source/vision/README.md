> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# Vision

后续 OpenCV 代码放在本目录。建议保持三层边界：摄像头采集、视觉检测、机器人
目标生成。视觉层只输出经过验证的目标或有限命令，不直接操作串口；机器人运动
继续复用 `week4/task2` 的 IK、直线规划和限位检查。

完整接入顺序和安全约束见 [项目交接文档](../HANDOFF.md#8-视觉功能的建议边界)。

LeRobot 已包含 `opencv-python-headless`，适合无窗口的数据处理。需要使用
`cv2.imshow` 时，应先统一团队依赖方案，再改用带 GUI 的 `opencv-python`，避免
两个 OpenCV wheel 同时安装。
