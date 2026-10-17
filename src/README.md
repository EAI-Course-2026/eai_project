# 源码分工与扩展约定

应用包为 `eai_robot`；独立安装的 LeRobot 硬件插件在
[`plugins/lerobot_robot_scs215/`](../plugins/lerobot_robot_scs215/)。
使用命令见[根 README](../README.md)，共享参数见[标定契约](../calibration/README.md)。

## 如何理解控制、动作与位姿

| 要解决的问题 | 唯一主要归属 | 示例 |
|---|---|---|
| 用户按了什么、需要组织哪个流程？ | `applications/` | 网页、键盘、相机预览、CLI、设备维护 |
| 要做哪个动作、依次经过哪些目标？ | `motion/` | `stand` / `raise` 命名关节姿态、演示计划、笛卡尔目标适配 |
| 给定关节角，末端在哪里；给定 XYZ，关节如何解？ | `kinematics/` | FK、IK、URDF、几何路径 |
| 何时下发目标，如何接管、读取反馈、停止？ | `control/` | `ArmController`、反馈检查、分段执行、IBVS 控制律 |
| 怎样读写设备？ | `hardware/` | 串口协议、后端、型号和硬件限位检查 |
| 如何识别设备参数与坐标变换？ | `calibration/` | 舵机端点采集、结果校验；相机内参/手眼的新增位置 |

“命名关节姿态”是保存的一组关节目标，不等于相机位姿估计或 TCP 位姿求解。
`motion` 表达动作目标，`kinematics` 计算几何解，`control` 管在线执行。
例如笛卡尔线段插值与逐点 IK 属于 `kinematics`，动作顺序属于 `motion`，
实时下发、到位检查和取消属于执行层或持有会话的应用服务。

## 当前目录

```text
eai_robot/
  applications/  用户流程、界面适配、设备会话和显式启用
  calibration/   标定结果模型与舵机采集/保存流程；视觉标定接入点
  motion/        命名关节姿态、固定起点与演示计划、笛卡尔目标模型
  control/       在线控制器、反馈检查、动作阶段执行、视觉伺服控制律
  kinematics/    关节映射、FK/IK、URDF、几何路径
  vision/        黑球/ArUco 图像检测、标记位姿估计、视觉目标图案
  policy/        学习策略、观测动作契约、推理、示范数据与评估
  voice/         白名单文本意图与动作调度
  robots/        应用侧机器人配置装配与关节范围读取
  hardware/      应用侧设备后端、原生协议与硬件检查
  arm/           旧导入路径兼容层，不新增实现
  poses/         旧导入路径兼容层，不新增实现
  cli.py         eai-course 命令分派
  config.py      仓库配置定位与读取
  input.py       键盘输入与命令数值解析
```

| 目录 | 关键实现与边界 |
|---|---|
| `applications/` | `arm_cli.py`、`desk_server.py` / `desk_service.py`、`demo.py`、`pose_control.py`、`record_pose.py`、`keyboard_control.py`、`cartesian_plan.py`、`visual_closed_loop.py`、`aruco_preview.py`、`voice_cli.py`、映射/FK 检查与 ID5 维护；拥有交互、线程、设备会话，调用能力模块 |
| `calibration/` | `results.py` 保存六关节数据契约、读取/校验和显式写文件函数；`servo.py` 实现人工端点、EEPROM 范围导入、GUI 候选保存。读取不会触发采样；[视觉接入说明](eai_robot/calibration/README.md) |
| `motion/` | `poses.py` 校验姿态文件并保持原始编码器目标；`demo.py` 验证起点并生成动作序列；`cartesian.py` 将原始反馈与几何目标接合，内部只有计算，不创建设备；`poses.reference.json` 保存默认参考姿态 |
| `control/` | `arm_controller.py` 接管/插值/回读/超时；`robot_feedback.py` 供多个入口复用反馈检查；`demo.py` 执行动作阶段；`aruco_ibvs.py` / `visual_math.py` 提供纯图像控制律。旧 `mapping.py` 转调统一的 `JointCalibration.position()` |
| `kinematics/` | `joint_mapping.py`、`urdf_fk.py`、`position_ik.py`、`cartesian_planner.py`、`kinematics_backend.py` 和 URDF；旧步骤/键盘/维护文件只有兼容转发 |
| `vision/` | `ball_detector.py`、`aruco.py` 产生图像观测；检测可用已提供内参估计标记位姿，不能因此称为“已实现相机标定”。旧预览、闭环与控制律文件只有兼容转发 |
| `robots/` | `so101.py` 装配插件 Robot，`joint_ranges.py` 读取范围；不实现另一份通信或运动学 |
| `hardware/` | `scs.py` 原生协议，`backends.py` 双后端，`inspection.py` 型号/限位检查与显式释放，`lerobot_scs215.py` 兼容导出插件总线 |
| `policy/` | 保留按策略生命周期内聚的模型、数据、CLI 与执行门禁；暂未与其他执行路径合并 |
| `voice/` | `intent.py` 解析识别文本，`dispatcher.py` 将白名单意图展开为动作并管理抢占；识别模型不在本仓库，`cli.py` 为兼容入口 |

## 依赖和扩展规则

- `cli.py` 只分派。新用户流程进入 `applications/`，不把硬件循环塞入数学模块。
- `motion/`、`kinematics/`、检测器和控制律只接收数据；设备由应用显式创建、持有和关闭。
- 运行时从 `calibration.results` 读取选定结果，不导入采样/求解流程。`robots/` 已不再依赖 `arm/`。
- 标定流程通过参数接收设备后端；不导入网页服务。设备检查放 `hardware/inspection.py`，标定不反向依赖在线控制器。
- 新代码不得依赖兼容层 `arm/`、`poses/` 或旧步骤脚本。兼容层指向同一实现，不复制状态或安全逻辑。
- 插件保持独立发行：应用依赖插件，插件不得反向导入 `eai_robot`。插件的舵机流程已从 `robot.py` 抽到插件自己的 `calibration.py`，标准 Robot 方法只委派。
- 新内参/手眼流程进入 `calibration/`，新检测器进入 `vision/`，新控制律进入 `control/`，新命名动作进入 `motion/`，新设备在 `robots/` 装配并通过 `hardware/` 适配。

## 兼容性与仍需区分的边界

`eai-course`、`scripts/arm_*.py` 的命令参数不变；旧 Python 模块可继续导入，
旧模块执行入口也可调用。默认参考姿态和网页资源已迁入各自的拥有目录，
使用显式旧资源路径的本机脚本应更新；默认命令会自动解析新位置。

上位机新采集的舵机候选改存于
`local_artifacts/experiments/<timestamp>_servo-calibration/candidate.json`。
审核后显式选择该文件并示教匹配起点；不会自动替换共享结果。
原有 CLI/LeRobot 的显式标定保存方式和已有共享数据保持兼容。

这是模块职责重构，**不是统一安全执行器改造**：桌面/姿态经 `ArmController`，
键盘/视觉/策略仍有插件 Robot 执行路径，各自安全门禁继续保留。
ArUco 当前只生成相机坐标系速度，未接入机械臂执行；相机内参和手眼算法待协作者加入。
`config.py` 仍依赖源码仓库布局，wheel 构建成功不等于脱离仓库可独立运行。

2026-10-17 的重构保持关节方向、单位、限位、控制周期和扭矩策略，
用离线回归核对兼容与故障停止；不代表新版实物验收。
