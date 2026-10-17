# 独立标定能力与视觉接入

这里是应用侧标定的正式代码位置。整体分工见 [`src/README.md`](../../README.md)。
标定流程拥有“采集 → 求解 → 验证 → 显式选用/审核发布”的生命周期；
运行时只读取选定结果，不重新采集或求解。

## 已实现

| 模块 | 分工 |
|---|---|
| `results.py` | `JointCalibration`、`JOINTS`、六关节校验、`load(path)` / `save(path, calibration)`、反馈容差与刻度换算；只有标准库依赖 |
| `servo.py` | `calibrate(backend, path, ...)` 人工端点流程，`import_hardware_calibration(...)` 已有 EEPROM 范围导入，`save_manual_candidate(captures, limits, root)` 校验并保存 GUI 候选 |
| `__init__.py` | 无副作用包入口；不自动导入采样、相机、串口或求解器 |

CLI 位于 `applications/arm_cli.py`，网页设备线程位于 `applications/desk_service.py`；
它们提供交互与设备会话，调用本目录中的流程。硬件型号/限位检查与显式扭矩释放来自
`hardware/inspection.py`，标定流程不依赖 `ArmController` 或网页服务。
`arm/calibration.py` 只是旧接口兼容层；新代码直接使用本目录。

LeRobot 插件独立安装，其流程位于
[`plugins/.../calibration.py`](../../../plugins/lerobot_robot_scs215/src/lerobot_robot_scs215/calibration.py)，
`Robot.calibrate()` 委派到该模块。插件不能导入本应用；使用共同格式及离线契约样例
约束两侧一致性。确需跨发行包共用更多逻辑时，单独评审轻量公共包，避免反向依赖。

## 协作者：视觉标定代码放哪里

**新增实现直接放在 `eai_robot/calibration/`，不再放 `vision/calibration/`。**
后者保留为旧占位目录。先同步本次重构提交，再在自己的功能分支开发。
如果旧分支已经新增视觉文件，合并时将新文件移入下列对应位置并修正导入；
原有 `vision.aruco` 检测 API 保持不变。

建议从三个模块起步（尚未实现，不是现成 API）：

```text
calibration/
  capture.py       图片/配对位姿采集、样本清单；通过参数注入相机/机器人读接口
  intrinsics.py    内参、畸变求解，逐视图与整体重投影误差
  hand_eye.py      配对位姿求解，坐标方向、残差、退化/可观测性检查
```

复杂后可各自扩成同名子包。将可离线调用的求解函数与采集、交互分开；
运行时加载器和结果契约应是轻量模块，不应为读取矩阵而导入求解/采样依赖。
`results.py` 当前只处理舵机，**不要把相机文件交给舵机 `load()`**。
视觉结果格式、资产身份、版本和验收阈值由视觉实现明确后再增加独立契约/加载器，
不能假定当前已有统一相机 JSON schema。

命令入口放 `applications/`，在 `eai_robot/cli.py` 薄分派；
可在视觉流程实现并有测试后注册统一 `calibrate` 命令。目前没有该命令。
离线测试放 `tests/test_calibration_*.py`，求解样例应小而可复现，不依赖摄像头或机械臂。
新代码不从 `arm/`、`poses/` 或课程步骤入口取公共函数。

## 数据与审核边界

- 原始图像、配对位姿、样本清单、求解中间文件与候选结果放
  `local_artifacts/experiments/<timestamp>_<kind>-calibration/` 或受控数据存储。
- 已审核、可共享的运行时结果在仓库根 [`calibration/`](../../../calibration/)。
  视觉目录规范见 [`calibration/vision/README.md`](../../../calibration/vision/README.md)。
- 历史结果与发布回执在根 `calibration/history/`；保留已有历史。
- 稳定资产编号随结果记录；相机索引、串口、个人路径只进本机配置。

GUI 舵机候选已写入本机实验目录，结果保存本身不代表审核通过。
现有 CLI 和插件保留显式输出路径、覆盖确认与备份语义；完整的通用审核发布工具尚未实现。
运行时不自动替换选定结果；更换舵机范围后，姿态与起点必须重新核验绑定。

视觉结果需记录设备资产、镜头/分辨率、安装配置、采集时间、源码提交、求解器版本、
样本清单与哈希、误差指标、审核信息及失效条件。手眼明确两个坐标系、变换方向与单位。
不同类型分别制定阈值，不能用同一个重投影误差代表全部质量。
现有舵机 JSON 继续兼容 LeRobot，扩展来源信息放伴随元数据文件。

## 设备与安全

相机采集只访问相机；配对采集可读取机器人与相机，不自行命令运动。
舵机人工采集保留支撑确认、扭矩检查与现有限位，绝不为通过验证扩大范围。
自动运动采样必须是显式启用的独立应用流程，通过受保护执行接口执行；
不能隐藏在读取标定、创建对象或连接设备中。

验收应覆盖：旧命令/格式兼容、取消和失败时文件保护、越界拒绝、设备及坐标匹配、
候选与共享结果隔离、各类求解离线样例。相机与机械臂实物验收另行记录。
