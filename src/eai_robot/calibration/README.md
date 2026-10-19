# 标定能力

标定集中在两个功能组，避免散落到检测器、在线控制器或界面中：

- `servo/`：已实现的六关节结果契约、端点采集和候选保存。
- `vision/`：相机内参、手眼及其采样/求解的接入位置，目前只有协作说明，求解器待加入。

`servo/results.py` 只依赖标准库，提供 `JointCalibration`、`JOINTS`、`load` / `save` 与校验。
运行时可以导入结果模块，不导入 `servo/workflow.py` 的交互采集。
`workflow.py` 通过参数接收后端，复用 `hardware/drivers/inspection.py`，不依赖在线控制器。
CLI 和桌面分别位于 `applications/tools/arm.py`、`applications/desktop/service.py`。

插件侧流程仍在
[`plugins/.../calibration.py`](../../../plugins/lerobot_robot_scs215/src/lerobot_robot_scs215/calibration.py)，
保持独立安装，不反向导入本应用。应用/插件共享已有结果格式和契约检查。

共享结果放根 [`calibration/`](../../../calibration/)，原始样本和未审核候选放
`local_artifacts/experiments/<timestamp>_<kind>-calibration/` 或受控存储。
GUI 候选不自动覆盖共享结果；CLI/插件保留显式输出和确认语义。

视觉接入见 [`vision/README.md`](vision/README.md)，总目录说明见 [`src/README.md`](../../README.md)。
