# 视觉标定旧占位目录

此目录目前只有占位说明与 `__init__.py`，没有相机内参或手眼标定算法。
复杂标定流程统一放在同级 `eai_robot/calibration/`，详见
[独立标定设计](../../calibration/README.md)与[源码分工](../../../README.md)。
新增内参、手眼和采样代码请直接放在新目录，不在这里创建第二套流程。
当前保留此占位包；新目录已承接舵机流程，视觉求解器仍待协作者实现。

审核后的共享视觉结果仍存于仓库根
[`calibration/vision/`](../../../../calibration/vision/README.md)。
原始图片、配对位姿、求解中间文件及候选结果留在
`local_artifacts/experiments/<timestamp>_vision-calibration/` 或受控数据存储。
相机本机编号放 `configs/hardware.local.toml` 或显式命令参数；
稳定资产编号、分辨率、坐标系、变换方向、误差及来源随审核结果记录。

现阶段使用 `eai-course aruco-preview` 做相机检测预览。
手眼配对采集可读取设备，不隐式命令运动；运行时读取标定不得触发重新求解。
