> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# Week 4：预设姿态与动作序列

## 文件说明

- `record_pose.py`：手动摆放机械臂并保存一个预设姿态。
- `control_presets.py`：使用键盘切换姿态或执行动作序列。
- `poses.json`：保存各姿态的六个归一化关节值。

仓库中的 `poses.json` 是参考机械臂记录的示例。不同机械装配、标定范围和负载会
产生差异；队友第一次使用时应支撑机械臂并低速验证，最好重新记录全部姿态。

## 记录姿态

先激活环境并进入目录：

```bat
cd /d "<仓库目录>\examples\eai_course\week4\task1"
conda activate lerobot
```

分别记录程序默认使用的四个姿态：

```bat
python record_pose.py stand --port COM5 --robot-id scs215_com5
python record_pose.py raise --port COM5 --robot-id scs215_com5
python record_pose.py left --port COM5 --robot-id scs215_com5
python record_pose.py right --port COM5 --robot-id scs215_com5
```

每条命令都会关闭舵机扭矩。手动摆好姿态后按 Enter，程序会从本机 LeRobot
标定文件读取六个舵机范围，把当前位置转换为六个 `0～1` 数值并写入
`poses.json`。不同机械臂必须使用各自的标定文件。

## 键盘控制

```bat
python control_presets.py --port COM5 --robot-id scs215_com5
```

按键定义：

```text
1  立正姿态 stand
2  抬起姿态 raise
3  向左姿态 left
4  向右姿态 right
w  执行挥手动作
q  关闭扭矩并退出
```

## 设置动作时间或速度

动作序列定义在 `control_presets.py` 的 `ACTION_KEYS` 中。每一步包含目标姿态、运行时间、运行速度和进入下一步前的等待时间。

按时间控制时：

```python
{"pose": "left", "time_ms": 1000, "speed": 0, "wait_s": 1.0}
```

按速度控制时，将 `time_ms` 设为 `0`，填写 `speed`，并根据动作所需时间设置 `wait_s`：

```python
{"pose": "left", "time_ms": 0, "speed": 100, "wait_s": 1.5}
```

程序从地址 `42` 开始，用一次同步写指令发送每个舵机的目标位置、运行时间和运行速度，因此六个关节会同时开始执行当前姿态。
