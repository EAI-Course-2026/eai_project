> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# EAI Course LeRobot

本目录保存 EAI Course 2026 的 SO-ARM101 控制代码。课程机械臂使用
Feetech SCS215 舵机，与 LeRobot 原生 SO follower 的 STS3215 不同，因此本 fork
已经在 LeRobot 硬件层加入协议 1、1024 分辨率和 SCS215 寄存器适配。

新成员请先阅读 [HANDOFF.md](./HANDOFF.md)，其中记录了项目现状、分支结构、
已验证环境、硬件参数、已知限制以及视觉和语音功能的接入建议。

## 目录结构

```text
examples/eai_course/
|-- HANDOFF.md        # 面向队友的完整交接文档
|-- setup_windows.cmd # Windows 环境安装与离线测试
|-- calibration/      # 同一台课程机械臂的参考标定 JSON
|-- week4/
|   |-- task1/        # 预设姿态、键盘切换和动作序列
|   `-- task2/        # FK、IK、直线轨迹和末端键盘控制
|-- vision/           # 后续 OpenCV 感知模块
`-- voice/            # 后续语音识别与命令解析模块
```

## 当前分支事实

组织仓库的 `main` 是队友应使用的稳定课程分支，它已经包含以下两层提交：

```text
fe81d5e5  feat(hardware): add Feetech SCS215 support
e5a2b958  feat(course): add SCS215 arm control coursework
```

`fe81d5e5` 是 rebase 后的 SCS215 适配提交。旧哈希 `62754eb8` 不再是组织仓库
当前历史中的提交号。现在不要求存在单独的远端 `scs215-adapter` 分支；适配层仍可
通过独立提交检查和追踪。

## 快速安装

在 Miniconda Prompt 中执行：

```cmd
git clone https://github.com/EAI-Course-2026/eai-course-lerobot.git
cd /d eai-course-lerobot
conda create -n lerobot python=3.12 -y
conda activate lerobot
python -m pip install --upgrade pip
```

使用 NVIDIA GPU 时，先安装与驱动匹配的 PyTorch。下面是本项目已验证的 CUDA
12.8 组合：

```cmd
python -m pip install torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 --index-url https://download.pytorch.org/whl/cu128
```

再从仓库根目录安装当前 fork 和硬件依赖：

```cmd
examples\eai_course\setup_windows.cmd
```

安装依赖需要访问 GitHub、PyPI 和 PyTorch wheel 源。完成安装后，普通串口控制
不要求持续联网。

## 每台机械臂必须单独标定

默认示例使用 `COM5` 和标定 ID `scs215_com5`。团队目前共享同一台实体机械臂，
因此队友可以直接使用仓库中的统一标定基线
[`scs215_com5.reference.json`](./calibration/scs215_com5.reference.json)；安装方法和
限制见 [calibration/README.md](./calibration/README.md)。使用不同机械臂时必须
替换端口、标定 ID，并重新标定：

```cmd
lerobot-find-port
lerobot-calibrate --robot.type=so_follower --robot.port=COM5 --robot.id=scs215_com5
```

运行时标定文件保存在用户的 Hugging Face 缓存目录。除仓库中明确维护的参考文件
外，不应提交个人机械臂的标定 JSON。

## 运行入口

预设姿态和动作序列：

```cmd
cd /d examples\eai_course\week4\task1
python record_pose.py stand --port COM5 --robot-id scs215_com5
python control_presets.py --port COM5 --robot-id scs215_com5
```

末端映射、FK、IK 和直线规划：

```cmd
cd /d ..\task2
python check_step1.py --port COM5 --robot-id scs215_com5
python check_step2_fk.py --hardware --port COM5 --robot-id scs215_com5
python run_steps3_to5.py --hardware --delta-mm 0 0 10 --port COM5 --robot-id scs215_com5
```

确认离线路径和当前姿态安全后，才增加 `--execute`。连续键盘控制第一次应使用
低速：

```cmd
python keyboard_control.py --port COM5 --robot-id scs215_com5 --speed-mm-s 10 --control-hz 20
```

详细按键、确认短语和急停方式见 [Task 2 README](./week4/task2/README.md)。

## 协作约定

队友从组织仓库克隆后，`origin` 指向课程仓库。每项功能从最新 `main` 建立分支：

```cmd
git switch main
git pull --ff-only
git switch -c feature/vision-marker-tracking

git add examples/eai_course/vision
git commit -m "feat(vision): add marker tracking"
git push -u origin feature/vision-marker-tracking
```

语音功能使用 `feature/voice-*`，修复使用 `fix/*`。提交 PR 前至少运行：

```cmd
python -m unittest discover -v -s examples\eai_course\week4\task2 -p "test_*.py"
```

不要提交个人标定 JSON、Conda 环境、`__pycache__`、摄像头原始录制或模型权重。
