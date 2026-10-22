# 源码路径迁移对照

本轮收敛为六个一级能力目录，并将实现归入第二级功能组。用户 CLI 命令与参数保持；外部 Python 导入需更新。

ArUco 接续开发请使用：

| 原模块 | 新模块 |
|---|---|
| `eai_robot.applications.visual_servo` | `eai_robot.applications.visual_servo.cli` |
| `eai_robot.control.visual_servo` | `eai_robot.control.visual_servo.controller` |
| `eai_robot.vision.aruco` | `eai_robot.vision.detection.aruco` |
| `eai_robot.vision.object_tracker` | `eai_robot.vision.tracking.optical_flow` |
| `eai_robot.calibration.results` | `eai_robot.calibration.servo.results` |
| `eai_robot.calibration.servo` | `eai_robot.calibration.servo.workflow` |

`eai_robot.vision.features` 保持原路径。相机内参/手眼新实现进入 `eai_robot.calibration.vision`，当前没有求解器。

完整文件对照（旧兼容入口直接映射到真实实现，不保留新的转发层）：

| 原路径 | 新路径 |
|---|---|
| `src/eai_robot/applications/arm_cli.py` | [src/eai_robot/applications/tools/arm.py](../src/eai_robot/applications/tools/arm.py) |
| `src/eai_robot/applications/aruco_preview.py` | [src/eai_robot/applications/visual_servo/aruco_preview.py](../src/eai_robot/applications/visual_servo/aruco_preview.py) |
| `src/eai_robot/applications/cartesian_plan.py` | [src/eai_robot/applications/tools/cartesian_plan.py](../src/eai_robot/applications/tools/cartesian_plan.py) |
| `src/eai_robot/applications/check_fk.py` | [src/eai_robot/applications/tools/check_fk.py](../src/eai_robot/applications/tools/check_fk.py) |
| `src/eai_robot/applications/check_mapping.py` | [src/eai_robot/applications/tools/check_mapping.py](../src/eai_robot/applications/tools/check_mapping.py) |
| `src/eai_robot/applications/demo.py` | [src/eai_robot/applications/tools/demo.py](../src/eai_robot/applications/tools/demo.py) |
| `src/eai_robot/applications/desk_assets` | [src/eai_robot/applications/desktop/assets](../src/eai_robot/applications/desktop/assets) |
| `src/eai_robot/applications/desk_server.py` | [src/eai_robot/applications/desktop/server.py](../src/eai_robot/applications/desktop/server.py) |
| `src/eai_robot/applications/desk_service.py` | [src/eai_robot/applications/desktop/service.py](../src/eai_robot/applications/desktop/service.py) |
| `src/eai_robot/applications/keyboard_control.py` | [src/eai_robot/applications/teleoperation/keyboard.py](../src/eai_robot/applications/teleoperation/keyboard.py) |
| `src/eai_robot/applications/pose_control.py` | [src/eai_robot/applications/teleoperation/poses.py](../src/eai_robot/applications/teleoperation/poses.py) |
| `src/eai_robot/applications/record_pose.py` | [src/eai_robot/applications/teleoperation/record_pose.py](../src/eai_robot/applications/teleoperation/record_pose.py) |
| `src/eai_robot/applications/reindex_id5.py` | [src/eai_robot/applications/tools/reindex.py](../src/eai_robot/applications/tools/reindex.py) |
| `src/eai_robot/applications/visual_closed_loop.py` | [src/eai_robot/applications/visual_servo/ball.py](../src/eai_robot/applications/visual_servo/ball.py) |
| `src/eai_robot/applications/visual_servo.py` | [src/eai_robot/applications/visual_servo/cli.py](../src/eai_robot/applications/visual_servo/cli.py) |
| `src/eai_robot/applications/voice_cli.py` | [src/eai_robot/applications/teleoperation/voice.py](../src/eai_robot/applications/teleoperation/voice.py) |
| `src/eai_robot/arm/backends.py` | [src/eai_robot/hardware/drivers/backends.py](../src/eai_robot/hardware/drivers/backends.py) |
| `src/eai_robot/arm/calibration.py` | [src/eai_robot/calibration/servo/results.py](../src/eai_robot/calibration/servo/results.py) |
| `src/eai_robot/arm/cli.py` | [src/eai_robot/applications/tools/arm.py](../src/eai_robot/applications/tools/arm.py) |
| `src/eai_robot/arm/controller.py` | [src/eai_robot/control/execution/arm.py](../src/eai_robot/control/execution/arm.py) |
| `src/eai_robot/arm/demo.py` | [src/eai_robot/applications/tools/demo.py](../src/eai_robot/applications/tools/demo.py) |
| `src/eai_robot/arm/desk_cartesian.py` | [src/eai_robot/control/planning/cartesian.py](../src/eai_robot/control/planning/cartesian.py) |
| `src/eai_robot/arm/desk_server.py` | [src/eai_robot/applications/desktop/server.py](../src/eai_robot/applications/desktop/server.py) |
| `src/eai_robot/arm/desk_service.py` | [src/eai_robot/applications/desktop/service.py](../src/eai_robot/applications/desktop/service.py) |
| `src/eai_robot/calibration/results.py` | [src/eai_robot/calibration/servo/results.py](../src/eai_robot/calibration/servo/results.py) |
| `src/eai_robot/calibration/servo.py` | [src/eai_robot/calibration/servo/workflow.py](../src/eai_robot/calibration/servo/workflow.py) |
| `src/eai_robot/control/arm_controller.py` | [src/eai_robot/control/execution/arm.py](../src/eai_robot/control/execution/arm.py) |
| `src/eai_robot/control/aruco_ibvs.py` | [src/eai_robot/control/visual_servo/aruco_ibvs.py](../src/eai_robot/control/visual_servo/aruco_ibvs.py) |
| `src/eai_robot/control/demo.py` | [src/eai_robot/control/execution/demo.py](../src/eai_robot/control/execution/demo.py) |
| `src/eai_robot/control/mapping.py` | [src/eai_robot/control/planning/mapping.py](../src/eai_robot/control/planning/mapping.py) |
| `src/eai_robot/control/robot_feedback.py` | [src/eai_robot/control/execution/feedback.py](../src/eai_robot/control/execution/feedback.py) |
| `src/eai_robot/control/visual_math.py` | [src/eai_robot/control/visual_servo/math.py](../src/eai_robot/control/visual_servo/math.py) |
| `src/eai_robot/control/visual_servo.py` | [src/eai_robot/control/visual_servo/controller.py](../src/eai_robot/control/visual_servo/controller.py) |
| `src/eai_robot/hardware/backends.py` | [src/eai_robot/hardware/drivers/backends.py](../src/eai_robot/hardware/drivers/backends.py) |
| `src/eai_robot/hardware/inspection.py` | [src/eai_robot/hardware/drivers/inspection.py](../src/eai_robot/hardware/drivers/inspection.py) |
| `src/eai_robot/hardware/lerobot_scs215.py` | [src/eai_robot/hardware/drivers/lerobot_scs215.py](../src/eai_robot/hardware/drivers/lerobot_scs215.py) |
| `src/eai_robot/hardware/scs.py` | [src/eai_robot/hardware/drivers/scs.py](../src/eai_robot/hardware/drivers/scs.py) |
| `src/eai_robot/kinematics/cartesian_planner.py` | [src/eai_robot/control/kinematics/cartesian_planner.py](../src/eai_robot/control/kinematics/cartesian_planner.py) |
| `src/eai_robot/kinematics/check_step1.py` | [src/eai_robot/applications/tools/check_mapping.py](../src/eai_robot/applications/tools/check_mapping.py) |
| `src/eai_robot/kinematics/check_step2_fk.py` | [src/eai_robot/applications/tools/check_fk.py](../src/eai_robot/applications/tools/check_fk.py) |
| `src/eai_robot/kinematics/joint_mapping.py` | [src/eai_robot/control/kinematics/joint_mapping.py](../src/eai_robot/control/kinematics/joint_mapping.py) |
| `src/eai_robot/kinematics/keyboard_control.py` | [src/eai_robot/applications/teleoperation/keyboard.py](../src/eai_robot/applications/teleoperation/keyboard.py) |
| `src/eai_robot/kinematics/kinematics_backend.py` | [src/eai_robot/control/kinematics/kinematics_backend.py](../src/eai_robot/control/kinematics/kinematics_backend.py) |
| `src/eai_robot/kinematics/position_ik.py` | [src/eai_robot/control/kinematics/position_ik.py](../src/eai_robot/control/kinematics/position_ik.py) |
| `src/eai_robot/kinematics/reindex_id5.py` | [src/eai_robot/applications/tools/reindex.py](../src/eai_robot/applications/tools/reindex.py) |
| `src/eai_robot/kinematics/run_steps3_to5.py` | [src/eai_robot/applications/tools/cartesian_plan.py](../src/eai_robot/applications/tools/cartesian_plan.py) |
| `src/eai_robot/kinematics/so101_new_calib.urdf` | [src/eai_robot/control/kinematics/so101_new_calib.urdf](../src/eai_robot/control/kinematics/so101_new_calib.urdf) |
| `src/eai_robot/kinematics/urdf_fk.py` | [src/eai_robot/control/kinematics/urdf_fk.py](../src/eai_robot/control/kinematics/urdf_fk.py) |
| `src/eai_robot/motion/cartesian.py` | [src/eai_robot/control/planning/cartesian.py](../src/eai_robot/control/planning/cartesian.py) |
| `src/eai_robot/motion/demo.py` | [src/eai_robot/control/planning/demo.py](../src/eai_robot/control/planning/demo.py) |
| `src/eai_robot/motion/poses.py` | [src/eai_robot/control/planning/poses.py](../src/eai_robot/control/planning/poses.py) |
| `src/eai_robot/motion/poses.reference.json` | [src/eai_robot/control/planning/poses.reference.json](../src/eai_robot/control/planning/poses.reference.json) |
| `src/eai_robot/policy/artifacts.py` | [src/eai_robot/policy/inference/artifacts.py](../src/eai_robot/policy/inference/artifacts.py) |
| `src/eai_robot/policy/cli.py` | [src/eai_robot/applications/tools/policy.py](../src/eai_robot/applications/tools/policy.py) |
| `src/eai_robot/policy/datasets.py` | [src/eai_robot/policy/data/datasets.py](../src/eai_robot/policy/data/datasets.py) |
| `src/eai_robot/policy/evaluation.py` | [src/eai_robot/policy/data/evaluation.py](../src/eai_robot/policy/data/evaluation.py) |
| `src/eai_robot/policy/execution.py` | [src/eai_robot/policy/execution/runner.py](../src/eai_robot/policy/execution/runner.py) |
| `src/eai_robot/policy/recording.py` | [src/eai_robot/policy/data/recording.py](../src/eai_robot/policy/data/recording.py) |
| `src/eai_robot/policy/runtime.py` | [src/eai_robot/policy/inference/runtime.py](../src/eai_robot/policy/inference/runtime.py) |
| `src/eai_robot/policy/safety.py` | [src/eai_robot/policy/execution/safety.py](../src/eai_robot/policy/execution/safety.py) |
| `src/eai_robot/policy/teleoperation.py` | [src/eai_robot/policy/execution/teleoperation.py](../src/eai_robot/policy/execution/teleoperation.py) |
| `src/eai_robot/policy/transport.py` | [src/eai_robot/policy/inference/transport.py](../src/eai_robot/policy/inference/transport.py) |
| `src/eai_robot/poses/arm_config.py` | [src/eai_robot/hardware/robots/joint_ranges.py](../src/eai_robot/hardware/robots/joint_ranges.py) |
| `src/eai_robot/poses/control_presets.py` | [src/eai_robot/applications/teleoperation/poses.py](../src/eai_robot/applications/teleoperation/poses.py) |
| `src/eai_robot/poses/record_pose.py` | [src/eai_robot/applications/teleoperation/record_pose.py](../src/eai_robot/applications/teleoperation/record_pose.py) |
| `src/eai_robot/robots/joint_ranges.py` | [src/eai_robot/hardware/robots/joint_ranges.py](../src/eai_robot/hardware/robots/joint_ranges.py) |
| `src/eai_robot/robots/so101.py` | [src/eai_robot/hardware/robots/so101.py](../src/eai_robot/hardware/robots/so101.py) |
| `src/eai_robot/vision/aruco.py` | [src/eai_robot/vision/detection/aruco.py](../src/eai_robot/vision/detection/aruco.py) |
| `src/eai_robot/vision/aruco_ibvs.py` | [src/eai_robot/control/visual_servo/aruco_ibvs.py](../src/eai_robot/control/visual_servo/aruco_ibvs.py) |
| `src/eai_robot/vision/aruco_preview.py` | [src/eai_robot/applications/visual_servo/aruco_preview.py](../src/eai_robot/applications/visual_servo/aruco_preview.py) |
| `src/eai_robot/vision/ball_detector.py` | [src/eai_robot/vision/detection/ball.py](../src/eai_robot/vision/detection/ball.py) |
| `src/eai_robot/vision/black_ball_target.html` | [src/eai_robot/vision/targets/black_ball.html](../src/eai_robot/vision/targets/black_ball.html) |
| `src/eai_robot/vision/object_tracker.py` | [src/eai_robot/vision/tracking/optical_flow.py](../src/eai_robot/vision/tracking/optical_flow.py) |
| `src/eai_robot/vision/visual_closed_loop.py` | [src/eai_robot/applications/visual_servo/ball.py](../src/eai_robot/applications/visual_servo/ball.py) |
| `src/eai_robot/vision/visual_math.py` | [src/eai_robot/control/visual_servo/math.py](../src/eai_robot/control/visual_servo/math.py) |
| `src/eai_robot/voice/cli.py` | [src/eai_robot/applications/teleoperation/voice.py](../src/eai_robot/applications/teleoperation/voice.py) |
| `src/eai_robot/voice/dispatcher.py` | [src/eai_robot/applications/teleoperation/dispatcher.py](../src/eai_robot/applications/teleoperation/dispatcher.py) |
| `src/eai_robot/voice/intent.py` | [src/eai_robot/applications/teleoperation/intent.py](../src/eai_robot/applications/teleoperation/intent.py) |

共享标定、锁文件与相机结果约定不因目录调整而变化。默认姿态 JSON、URDF 和网页资源已随实现归位。
原始本机实验材料没有改动；退出源码的旧字节码目录已保存在被忽略的本机材料区。

## Agent 三层内部 API（2026-10-19）

CLI `eai-course agent`、`voice` 保持不变，Python 内部接口调整如下；不保留永久转发模块。

| 原接口/位置 | 当前接口/位置 |
|---|---|
| `policy.agents.providers` | `policy.model_io.providers`（JSON/SSE）与 `model_io.auth`（环境凭据） |
| `agents.contracts.ChatRequest/ChatReply/ToolCall/ProviderError` | `model_io.contracts`；新增 Message、ToolDefinition、ModelEvent、ChatProvider |
| 供应商格式 message/tool dict | 核心使用 Message/ToolDefinition，供应商字段编码在 provider 内 |
| `ToolRegistry.schemas` | `ToolRegistry.definitions`，返回规范化定义 |
| `handler(arguments)` | `handler(arguments, ToolContext)`；须遵守期限/取消，进度通过 context.progress |
| `AgentSession.stop()` | 通用 `cancel()`；应用通过 on_cancel 注入本地停止 |
| `policy/agents/prompts/system.txt` | `applications/agent/prompts/system.txt`；通过应用工厂注入核心 |

机械臂会话建议用 `applications.agent.session.create_robot_session` 装配 prompt、能力和停止回调。
核心可单独用于非机械臂工具，不依赖应用包或模型具体适配器。参见[运行指南](agent_runtime.md)。
