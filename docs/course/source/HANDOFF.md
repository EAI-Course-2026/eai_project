> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# EAI Course LeRobot 项目交接

更新日期：2026-10-01

## 1. 项目目标与当前状态

本仓库是 `huggingface/lerobot` 的组织 fork，用于控制课程中的 SO-ARM101 follower。
机械结构参考 SO-101，但六个舵机是 Feetech SCS215，不是 LeRobot 原生配置使用的
STS3215。

当前已经完成：

- SCS215 在 LeRobot 电机层和 SO follower 层的适配。
- 1 至 6 号舵机的通信、标定读取和归一化位置控制。
- 预设姿态记录、键盘切换姿态和动作序列。
- 基于官方 SO-101 URDF 的 FK。
- 位置 IK、笛卡尔直线插值和无解位置回退。
- 使用键盘控制末端沿 X/Y/Z 方向移动。
- 15 项不连接机械臂的自动化测试。
- 同一台课程机械臂可复用的参考标定 JSON 和安装说明。

尚未完成：

- OpenCV 目标检测与目标坐标到机器人坐标的标定。
- 语音识别、命令解析和动作调度。
- 供视觉和语音共同调用的稳定高层控制 API。
- 末端姿态约束；当前 IK 只约束位置。
- Task 2 中夹爪的交互控制；键盘笛卡尔控制只驱动 ID 1 至 5。

## 2. 仓库与分支结构

仓库地址：

```text
https://github.com/EAI-Course-2026/eai-course-lerobot
```

远端 `main` 是当前可运行的课程主分支。关键历史锚点如下：

```text
e0d50211  LeRobot 上游基线
    |
fe81d5e5  SCS215 硬件适配
    |
e5a2b958  Week 4 课程控制代码
```

这里没有把 SCS215 修改不可逆地揉进一个大提交。硬件适配和课程代码仍是两个
独立提交，只是都已经包含在 `main`，让队友克隆后可以直接使用。

建议后续分支：

```text
main                         已验证、可运行的集成分支
feature/vision-*             OpenCV 和视觉定位
feature/voice-*              语音识别与意图解析
feature/control-*            可复用控制层重构
fix/*                        缺陷修复
chore/sync-upstream-*        同步 LeRobot 上游
```

不要在多人已经拉取后 rebase 或 force-push `main`。需要同步上游时建立独立分支并
通过 PR 合并：

```cmd
git remote add upstream https://github.com/huggingface/lerobot.git
git fetch upstream
git switch main
git pull --ff-only
git switch -c chore/sync-upstream-YYYYMMDD
git merge upstream/main
```

解决冲突并运行测试后，再推送该分支创建 PR。

注意远端命名：队友新克隆时 `origin` 是组织仓库；最初维护者的旧工作目录中，
`origin` 可能仍指向 Hugging Face，而组织仓库名为 `course`。执行 push 前先检查：

```cmd
git remote -v
```

## 3. SCS215 适配改了什么

适配提交 `fe81d5e5` 只修改以下五个 LeRobot 文件：

```text
src/lerobot/motors/feetech/feetech.py
src/lerobot/motors/feetech/tables.py
src/lerobot/motors/motors_bus.py
src/lerobot/robots/so_follower/config_so_follower.py
src/lerobot/robots/so_follower/so_follower.py
```

关键差异：

- 注册 `scs215`，型号编号 1315、1024 编码分辨率、协议版本 1。
- 使用 SCS215 的控制表和波特率表，跳过手册未定义的寄存器。
- SCS215 不支持 `GroupSyncRead`，读取位置时改为逐舵机读取。
- 不使用 STS3215 的 homing offset 和 operating mode 流程。
- SO follower 的 ID 1 至 6 全部配置为 SCS215。
- 标定记录每个关节的实际安全范围，不把腕部强制设成完整一圈。

这五个文件是以后同步 LeRobot 上游时最可能发生冲突的地方。不要用上游版本直接
覆盖它们；先比较 `fe81d5e5` 的差异，并重新运行硬件检查。

## 4. 已验证环境

当前已验证的本机环境：

| 项目 | 版本或状态 |
| --- | --- |
| 操作系统 | Windows 11 |
| Python | 3.12.14，Miniconda 环境名 `lerobot` |
| LeRobot | 当前 fork，版本 0.6.2 |
| PyTorch | 2.11.0+cu128 |
| CUDA runtime | 12.8，`torch.cuda.is_available()` 为 `True` |
| GPU | NVIDIA GeForce RTX 5060 Laptop GPU |
| NumPy | 2.2.6 |
| FFmpeg | 7.1.1 |
| PyAV | 15.1.0 |
| TorchCodec | 0.11.1+cpu |
| Datasets | 4.8.5 |

机械臂控制本身不依赖 TorchCodec、FFmpeg 或 GPU。它们主要用于数据集视频解码和
后续训练。Windows 下没有使用 Placo，Task 2 会自动采用仓库中的 NumPy FK 和
damped least squares IK 实现。

快速验证：

```cmd
conda activate lerobot
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
lerobot-info
python -m unittest discover -v -s examples\eai_course\week4\task2 -p "test_*.py"
```

## 5. 硬件与标定记录

当前参考机械臂参数：

```text
串口：COM5
波特率：1,000,000
协议：Feetech protocol 1
舵机 ID：1, 2, 3, 4, 5, 6
标定 ID：scs215_com5
```

当前参考标定范围仅用于排查，不可直接复制到另一台机械臂：

| 关节 | ID | 参考 raw 范围 |
| --- | ---: | ---: |
| shoulder_pan | 1 | 183..897 |
| shoulder_lift | 2 | 64..737 |
| elbow_flex | 3 | 196..769 |
| wrist_flex | 4 | 70..681 |
| wrist_roll | 5 | 47..974 |
| gripper | 6 | 237..644 |

标定文件默认位于：

```text
%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\<robot-id>.json
```

仓库同时保存了：

```text
examples/eai_course/calibration/scs215_com5.reference.json
```

团队目前使用同一台实体课程机械臂，因此它是队友环境中的统一标定基线，安装方法
见同目录 `README.md`。换用另一台机械臂、重新安装舵盘或维修连杆时必须重新标定。
`poses.json` 是这台共享机械臂记录的姿态，首次在新电脑运行时仍应低速验证。

## 6. 现有控制代码

### Task 1：预设姿态

目录：`examples/eai_course/week4/task1`

- `arm_config.py`：从当前用户的 LeRobot 标定文件读取六个 raw 范围。
- `record_pose.py`：关闭扭矩，手动摆放后记录六个 `0..1` 关节值。
- `control_presets.py`：同步写入六个舵机的目标、时间和速度。
- `poses.json`：`stand`、`raise`、`left`、`right` 参考姿态。

Task 1 是关节空间动作，不保证末端走直线。

### Task 2：末端笛卡尔控制

目录：`examples/eai_course/week4/task2`

- `joint_mapping.py`：raw、LeRobot 归一化值和 URDF 角度互转。
- `urdf_fk.py`：不依赖 Placo 的 NumPy FK。
- `kinematics_backend.py`：按环境选择 LeRobot/Placo 或 NumPy FK。
- `position_ik.py`：位置 IK，Windows 使用 damped least squares。
- `cartesian_planner.py`：直线采样、逐点 IK 和不可达位置回退。
- `run_steps3_to5.py`：离线规划、读取硬件以及确认后执行轨迹。
- `keyboard_control.py`：20 Hz 连续末端控制和急停。
- `reindex_id5.py`：处理 ID 5 在编码器 `1023 -> 0` 附近跳变的问题。
- `so101_new_calib.urdf`：SO-101 尺寸和关节限制。

键盘控制：

```text
W/S      +X/-X
A/D      +Y/-Y
R/F      +Z/-Z
Q/Esc    正常停止
Space    立即关闭 ID 1 至 5 扭矩
```

默认速度为 20 mm/s、控制频率为 20 Hz、关节边界余量为 2 度。第一次硬件测试建议
使用 10 mm/s。

## 7. 已知限制和常见问题

### 所有方向都显示 `IK 无解`

先检查当前关节是否位于标定端点附近。参考机械臂曾出现 shoulder lift 接近下限、
elbow flex 接近上限，此时即使目标只移动 1 mm，也可能没有满足边界余量的解。
将机械臂放到标定范围内部的安全姿态后再启动。

### wrist roll 在 0 和 1023 附近跳变

这是编码器边界跨越，不是普通的 min/max 标定问题。使用
`reindex_id5.py inspect/center/calibrate`，并严格按照 Task 2 README 操作。

### IK 能控制什么

当前 IK 只约束 `gripper_frame_link` 的 X/Y/Z 位置，不约束末端朝向。第六个夹爪
舵机不属于 IK 链；执行 Task 2 轨迹时保持夹爪当前值，键盘控制也不会开合夹爪。

### 串口占用

同一时间只能有一个进程打开 COM5。不要同时运行键盘控制、姿态动作、标定程序、
视觉控制或语音控制。`lerobot-find-port` 会要求拔插 USB，用于识别端口，不是常驻
控制程序。

## 8. 视觉功能的建议边界

不要让视觉线程直接写舵机。建议拆成：

```text
摄像头采集 -> OpenCV 检测 -> 坐标变换 -> 有限目标命令 -> 统一控制器
```

建议首先完成：

1. 选择摄像头并稳定读取帧。
2. 定义检测输出，例如像素中心、置信度和深度。
3. 完成相机坐标到机器人 base 坐标的标定。
4. 将目标转换为小步笛卡尔位移，复用 Task 2 的规划与限位检查。
5. 目标丢失、置信度过低或 IK 无解时停止，不发送猜测动作。

LeRobot 当前安装的是 `opencv-python-headless`。需要 `cv2.imshow` 时，团队应统一
改为 `opencv-python`，不要同时安装两个 OpenCV wheel。

## 9. 语音功能的建议边界

建议流程：

```text
音频输入 -> 语音识别 -> 白名单意图 -> 动作调度 -> 统一控制器
```

只允许识别结果映射到明确命令，例如：

```text
stand, wave, move_left, move_right, open_gripper, close_gripper, stop
```

不要执行任意文本或动态 Python。`stop` 必须能抢占普通动作；低置信度或无法解析的
文本只打印反馈，不驱动机械臂。

## 10. 下一步推荐重构

视觉和语音接入前，建议先从现有脚本提取一个唯一持有串口的控制层：

```text
ControlRuntime
|-- connect/disconnect
|-- emergency_stop
|-- move_to_pose
|-- move_cartesian_delta
|-- set_gripper
`-- get_state
```

视觉和语音模块只向该控制层提交结构化命令，不自行创建 `SO101Follower`。这样才能
保证串口单一所有者、统一限位、统一急停和动作互斥。

## 11. 安全与提交清单

每次真实运动前：

- 支撑机械臂，清空运动范围，确认急停按键。
- 确认端口、robot ID 和标定文件属于当前机械臂。
- 先运行离线测试，再运行只读检查，再低速执行。
- 机械臂位于关节端点附近时不要启动笛卡尔键盘控制。
- 关闭其他可能占用串口的程序。

每个 PR 至少说明：

- 改动的模块和用户可见行为。
- 是否连接过真实机械臂，使用了什么速度和测试姿态。
- 离线测试结果。
- 是否改变控制频率、关节限制、标定或急停逻辑。

禁止提交：未明确审核的个人标定 JSON、Conda 环境、缓存、原始视频、密钥和
大模型权重。仓库中的 `calibration/scs215_com5.reference.json` 是唯一受维护的
共享参考标定。
