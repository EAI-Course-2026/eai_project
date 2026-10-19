> 历史记录，保留早期演练及现场参数演变；当前操作请使用 [现行指南](aruco_visual_servo.md)，不按历史参数直接启动真机。

# ArUco visual servoing scaffold

## Step 1: unified rehearsal (2026-10-17)

`eai-course visual-servo` now runs detection and control proposals from a local
image or camera. It imports no robot or serial implementation and has no motion
enable option. Existing `aruco-preview` and black-ball `vision` commands retain
their behavior.

```sh
# One local image: no camera or robot connection
uv run --locked eai-course visual-servo --image PATH_TO_IMAGE --marker-id 0
# Camera rehearsal: camera access only; Q/Esc closes the preview
uv run --locked eai-course visual-servo --camera-index N --marker-id 0 --preview
# Advanced rehearsal with calibrated corner depths
uv run --locked eai-course visual-servo --image PATH_TO_IMAGE --marker-id 0 \
  --mode ibvs --intrinsics PATH_TO_INTRINSICS --marker-length-m 0.05
```

The marker length above is an example measured ArUco side length in meters;
it is independent of the calibration checkerboard's 25 mm square size.

- `vision/features.py` defines ordered points, image dimensions, monotonic
  capture time and target identity, for future marker/object adapters.
- `control/visual_servo/controller.py` rejects missing, stale, future-dated and changed
  targets. Rejected observations produce no displacement or velocity.
- Simple mode requests a 10 mm step on one optical camera axis: x right,
  y down, z forward. Outside the middle 20% box it chooses the largest
  normalized center error; inside it proposes forward motion. This does not
  estimate distance or authorize repeated forward execution.
- IBVS mode uses PnP depths and undistorts measured corners before computing
  image error and the interaction matrix. It emits a bounded camera twist;
  reaching the per-coordinate pixel tolerance emits zero velocity.
- JSON output distinguishes `camera_delta_m` from
  `camera_twist_m_s_rad_s`. These must not be interpreted as robot commands.

The following is a **rehearsal interchange format**, pending agreement with
the calibration collaborator. It does not set the shared calibration schema:

```json
{
  "distortion_model": "opencv_pinhole",
  "image_size": [640, 480],
  "camera_matrix": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
  "distortion": [0, 0, 0, 0, 0]
}
```

These values illustrate the format and are not a real calibration. The loader
also accepts the collaborator's version-1 output: `model: "opencv_pinhole"`,
`image_size: {"width": 640, "height": 480}` and `distortion_coefficients`.
Pass it directly using `--intrinsics calibration/vision/camera_intrinsics.json`;
no file conversion or checkerboard-size changes are performed. Conflicting alias
fields and unsupported schema versions are rejected. The marker's measured side
length must still be supplied independently via `--marker-length-m`.
The loader
rejects unsupported models and frame resolution mismatches instead of silently
scaling intrinsics. Live acquisition is timestamped before reading; this bounds
processing latency but cannot prove the age of frames buffered by the camera
backend. An execution adapter must establish capture freshness independently.

Offline regression covers camera-motion projection signs, synthetic 3-D
closed-loop error reduction, real generated marker detection/PnP/undistortion,
loss/staleness/identity rejection and still-image CLI isolation. No real camera,
arm or hardware convergence has been validated in this step.

## Step 2: selected-object flow tracking

The same rehearsal entry point accepts a selected textured object:

```sh
uv run --locked eai-course visual-servo --target object --camera-index N \
  --roi 100 100 160 120 --preview
# Omit --roi to select a region in the preview with the mouse and confirm Enter.
uv run --locked eai-course visual-servo --target object --camera-index N --preview
```

Keep the object in the selected region while confirming: live acquisition
restarts with a fresh image after selection. `--image PATH_TO_IMAGE` can also
initialize an ROI offline, but a still image cannot demonstrate temporal tracking.

`vision/tracking/optical_flow.py` uses Shi-Tomasi texture points, forward/backward
Lucas-Kanade flow and a robust partial affine fit to emit a center observation
through the same `VisualFeatures` contract. It supports the simple controller;
it does not infer metric depth or provide corresponding object corners for IBVS.
The tracker requires at least six consistent points. Too few points, an invalid
fit, a frame gap above 250 ms, non-increasing timestamps or changed resolution
discard the tracking state. It never silently selects another object; restart
the command to explicitly select again after loss.

This is an initial object-tracking extension, with limits under occlusion,
background drift, fast motion, weak texture and deformation. It is not semantic
recognition or validated tracking of every object. Tests cover translated texture,
occlusion/loss latching, explicit reselection, frame gaps and invalid initialization.

## Step 3: calibrated robot adapter

### Bounded first-step experiment

`control/visual_servo/joint_adapter.py` now differentiates `base_T_gripper @
gripper_T_camera` to obtain the camera Jacobian, including the mounting lever arm.
It solves the combined image/joint Jacobian for the five motion joints, retaining
camera angular effects. Offline projection and reachable-goal convergence tests
pass; this does not guarantee every four-corner goal is reachable.

The separate `aruco-experiment` command defaults to read-only real-device
preflight. `visual-servo` remains a camera-only rehearsal command.

```sh
uv run --locked eai-course aruco-experiment \
  --intrinsics calibration/vision/camera_intrinsics.json \
  --hand-eye calibration/vision/hand_eye_result.json \
  --hand-eye-gripper-frame gripper_frame_link --marker-length-m 0.025
```

Only assert `--hand-eye-gripper-frame` after confirming the calibration's exact
end frame. The shown file paths refer to local candidate files, not approved
shared assets. Host serial/camera settings come from ignored hardware config.
Close other camera/serial owners before running. Add `--execute` only for an
operator-attended trial: it prompts before enabling torque, seeds measured goals,
and defaults to a single step. Ctrl+C stops and releases torque; keep the arm
supported. Normal completion holds while waiting for support confirmation before
release. The initial corner-to-square correspondence is selected once to avoid
an unnecessary 180-degree turn when the printed marker is upside down.

Solver steps are capped at 0.4 degrees. Encoder quantization and single-count
neighbor candidates are rechecked against 0.75 degrees and 2 mm per step,
3 degrees and 10 mm from the initial pose, joint margins and a 20-second budget.
At most 20 steps may be requested. Reached-waypoint tolerance is 2 encoder counts;
this initial experiment is not precision actuator acceptance. The existing
reviewed wrist range 47..974 is used as on the desktop path, without expanding
limits. Lost targets, processing age above 250 ms, status alarms, unexpected
torque states or increasing image error stop execution. Buffered frames are
flushed between actions; the backend does not expose hardware capture timestamps.
Logs, image errors, raw goals/feedback and frames are saved under ignored local
experiment directories. The first real-device step did not demonstrate error
reduction; see the [hardware record](../reports/2026-10-17-aruco-hardware-first-step.md).

Broader hand-eye/joint execution acceptance remains pending. The adapter must agree the
collaborator's transform direction, units and exact FK end frame. The arm has
five motion joints plus a gripper, while the camera controller returns six twist
components. A position-only IK adapter that discards camera angular velocity
cannot establish four-corner convergence. Further trials must validate measured
joint response and image error reduction, retaining runtime feedback, target
freshness, joint/travel limits and explicit motion enable.

## Original scaffold and remaining execution work

The original camera-only foundation is split into camera observations and pure
image-space math. Its preview commands open only the camera. The separately
enabled `aruco-experiment` adapter above owns the real-device trial.

## Current modules

- `src/eai_robot/vision/detection/aruco.py` detects a configured marker and can
  estimate its pose from supplied intrinsics and physical marker size.
- Camera and hand-eye calibration belong to the independent
  [calibration capability](../../src/eai_robot/calibration/README.md).
  `vision/calibration/` is retained as the earlier placeholder; neither location
  currently contains camera or hand-eye calibration algorithms.
- `eai-course aruco-preview --camera-index N --marker-id ID` shows a
  camera-only detection preview. It does not connect to the robot.
- `src/eai_robot/control/visual_servo/aruco_ibvs.py` builds the four-corner image
  error, point-feature interaction matrix, and a bounded damped least-squares
  camera-frame velocity command.
- Offline math and detector regression coverage is present in
  `tests/test_visual_servo.py`; joint-adapter and simulated-device regressions
  are in `test_visual_servo_joints.py` and `test_visual_servo_experiment.py`.

The camera velocity order is `[vx, vy, vz, wx, wy, wz]` in the camera frame.
Marker corner identity follows the marker's canonical TL, TR, BR, BL ordering,
which rotates with the printed code and is not necessarily the image's current
top-left point. The pure camera solver returns a camera-frame velocity only;
the joint experiment uses the combined image/joint Jacobian described above.

## Intended pipeline

1. Capture an RGB image and identify the configured ArUco dictionary and ID.
2. Use the collaborator-provided camera intrinsics and the printed marker size
   to estimate the marker pose and corner depths.
3. Compare the four detected corners with four configured desired image points.
4. Form the interaction matrix and compute a damped, norm-limited camera twist.
5. Preview the proposed twist and image error before adding any robot adapter.
6. Add hand-eye conversion and bounded Cartesian/IK execution only after the
   calibration files, frame convention, limits and on-hardware validation are
   agreed and reviewed.

Marker ID selection is explicit: if no ID is configured, a frame containing
multiple markers is rejected as ambiguous. Missing markers return no
observation; the eventual runtime must stop and hold rather than reuse stale
error or motion commands.

## Pending integration decisions

- The camera calibration JSON schema and camera asset identifier are being
  prepared separately. The math API accepts `camera_matrix` and `distortion`
  directly so it does not assume that schema.
- Marker dictionary, ID, printed side length and desired corner layout must be
  recorded in a portable configuration, without committing local camera index.
- Hand-eye output must state the transform direction and coordinate frames.
  The robot adapter must convert the camera-frame command using the reviewed
  transform and then use the existing Cartesian planner / IK safety path.
- Raw images, local camera settings and experiment outputs stay in ignored
  local-artifact areas. Only a reviewed, reproducible calibration result should
  be promoted into shared `calibration/` data.

The camera-only scaffold provides no hardware execution; use the separate,
explicitly enabled experimental adapter only within its documented limits.

### 实物居中修正（待本轮实物验收）

`vision --target aruco --joint-center --execute --preview --local-keys`
提供直接测量像素对编码器变化的居中路径，用于排查末端 XYZ 控制中相机转动与
指令位置累积的影响。此模式只做中心点居中，不执行前进或四角点姿态伺服。

启动后对底座和手腕俯仰分别做可恢复的 10 计数探测，再从满足边界的
±10/0 计数组合中选择预测能降低误差的动作。每次使用实际编码器位置作起点，
要求运动方向正确、至少变化 2 计数。距目标不超过 3 计数才记录到位；若未到位但反馈已稳定至少200ms，则把目标收回实测位置，单独记录部分响应并使用实际位移。1秒内无有效响应即停止发送。
因此“未动但在容差内”不计为到位。实际响应被记录；现行运行阶段固定矩阵，只在静止目标的显式探测阶段标定，避免目标运动污染模型。
连续两步误差增加、目标丢失、响应病态或没有安全改善动作时保持并报告原因。
保留软件标定限位、2°端点余量、单步关节角限制与原有总行程限制；另限制
相对接管姿态的任一关节变化不超过10°。10计数在底座开环测试有效，不代表其他轴已验收；本轮5计数反向恢复曾无响应，保留失败日志。

相机预览应单独常开：

```sh
uv run --locked eai-course aruco-preview --camera-index CAMERA_INDEX --marker-id 0 --stream-file local_artifacts/tmp/visual-preview/latest.frame
# 在另一终端启动控制；不重复占用 USB 相机
uv run --locked eai-course vision --target aruco --joint-center --marker-id 0 --camera-index CAMERA_INDEX --shared-frame local_artifacts/tmp/visual-preview/latest.frame --execute --preview --local-keys
```

相机编号依本机确认填写。控制退出后预览仍保持打开，退出控制不代表收敛。
JSONL 中保存实际目标计数、反馈、像素误差和停止原因，证据写入被忽略的
`local_artifacts/experiments/`。本模式的离线测试不代表已完成真实机械臂验收。

`--center-region-fraction 0.2` 明确采用中央20%的矩形区域：640×480图像对应
中心误差横向±64、纵向±48像素。`LOCKED` 只表示连续帧满足这个宽松居中条件，
不表示精确中心、四角点收敛或前进距离已验收；默认0仍使用像素死区。

普通暂停和运行时间结束会继续保持扭矩。控制终端输入 `R` 可原地重试，
原始行程和关节参考不会重置；只有操作者已支撑并按回车才正常释放。
空格/Ctrl+C仍为紧急释放。独立相机窗口不受普通控制暂停影响。

显式的 `--adopt-hold --hold-reference PATH` 用于已知暂停旧控制进程释放串口后的
单所有者交接，不能同时启动两个串口控制器。它不写扭矩开关或接管位置目标；
只读核验五轴扭矩全开、状态正常、至少5次采样稳定（偏差≤2计数），且现有保持目标
与反馈误差≤8计数，才接管。交接保留最初的参考姿态和行程边界。
`--resume-response PATH` 可复用本轮实际动作日志中的局部响应，避免重复探测。
这些入口不能用于无来源的姿态/矩阵，也不能以重置参考绕过范围限制。
2026-10-18实物已记录连续纠偏及在上述中央20%区域的 `LOCKED`；
持续移动目标跟随、精确四角点和自动前进仍需分别验收。

暂停预览修正：控制程序不能在暂停后阻塞于 `input()` 而停止 OpenCV 事件处理。
现在等待重试期间持续读取新帧，显示 `PAUSED`、`LIVE` 帧号和实时标记坐标；
在控制窗口按 R 或终端输入 R 可重试。相机断流显示黑色故障画面，不能拿旧帧冒充实时。
终端 EOF 不等于允许卸扭矩。紧急空格依然立即触发释放。
`--adopt-hold --start-paused` 用于只读接管并恢复实时预览，不执行位置动作。
恢复画面不等于恢复运动：边界、目标丢失、时间限制等暂停原因仍需明确处理。

### 2026-10-18 MVP实物修正与记录

操作者明确要求扩大预算并区分关节起动步长后，当前实验使用：

```sh
uv run --locked eai-course vision --target aruco --joint-center --marker-id 0 --execute --preview --local-keys --shared-frame local_artifacts/tmp/visual-preview/latest.frame --center-region-fraction 0.2 --max-travel-mm 100 --max-joint-excursion-deg 40 --base-step-counts 15 --wrist-step-counts 12
```

相机索引取本机已核实配置。底座候选为±10/±15计数，手腕固定±12计数，
不再用手腕5计数无效动作替代。仍检查单步5°、标定绝对限位与2°余量。
100mm是已授权的本轮模型位移预算，并非已测量的无碰撞物理工作空间。
正常暂停持续实时预览且保持扭矩，R可重试。

**修正在线模型污染：** 纸张本身运动会被逐步拟合错误归因于机械臂，可能使
雅可比方向翻转。运行期间现在固定显式静止探测得到的响应矩阵；实际响应只记录，
不在线修改矩阵。回归包含“目标自身运动后仍保持正确关节方向”的场景。

本轮实物日志 `local_artifacts/experiments/20261018_124827_joint-center.jsonl`
记录两次连续纠偏并进入中央20%区域，约(140,124)→(-4,-9)像素，
随后约(138,-127)→(14,-40)像素；对应每步有实际编码器反馈。
前一轮手腕12计数请求得到约6计数实际运动，纵向误差92→54→22像素。
这些是宽松中心区域MVP的实物记录，不代表四角点姿态、自动前进或任意物体跟随验收。
历史10°/40mm配置及失败日志保留；共享默认10°/40mm不被本轮参数覆盖。
