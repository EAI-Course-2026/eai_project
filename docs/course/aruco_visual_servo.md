# 视觉伺服：当前入口与实验基线

2026-10-18已完成并录制ArUco中央区域真机MVP，最终日志验证10次纠偏、27个实际动作步骤。
这是宽松的中心区域跟随；四角点精密IBVS、自动前进和任意物体真机跟随尚未验收。

- [本轮参数、统计和失败原因](../reports/2026-10-18-visual-servo-round1.md)
- [下一轮精细伺服计划](../experiments/visual_servo_precision.md)
- [源码职责](../../src/eai_robot/applications/visual_servo/README.md)
- [早期演练、标定适配和现场迭代记录](aruco_visual_servo_history.md)

## 一、命令边界

| 命令 | 功能 | 硬件动作 |
|---|---|---|
| `aruco-preview` | 相机采集；可后台发布帧 | 不打开机械臂串口 |
| `vision --profile …` | 检测/预览；不加`--execute`时仅相机 | 无 |
| `vision --profile … --execute` | 本轮实测关节响应的中央区域控制 | 预检及显式启动确认后执行 |
| `visual-servo` | 图片/相机演练，简单控制建议、四角点IBVS、选定物体光流 | 不执行机器人动作 |
| `aruco-experiment` | 内参/手眼/FK的受限实验入口 | 默认只读；精密真机未验收 |

ArUco为DICT_4X4_50、ID0，外圈黑色正方形边长25mm。该标记尺寸与棋盘格格长无关。
MVP只用二维图像和实测关节响应，不依赖内参/手眼完成度来宣称毫米精度。

## 二、日常启动：一个可见窗口

在`configs/hardware.local.toml`填写并核实本机串口、末端相机编号。
配置文件不含机器端口/索引，不自动授予执行权限。

第一终端开启相机唯一所有者，不显示第二个窗口：

```sh
uv run --locked eai-course aruco-preview --marker-id 0 --headless \
  --stream-file local_artifacts/tmp/visual-preview/latest.frame
```

第二终端先检测预览（不开串口）：

```sh
uv run --locked eai-course vision \
  --profile configs/visual_servo/aruco_center_mvp.toml \
  --shared-frame local_artifacts/tmp/visual-preview/latest.frame \
  --preview --local-keys
```

开始新一轮真机实验时，先结束检测进程，再在已知安全、离开端点的姿态启动：

```sh
uv run --locked eai-course vision \
  --profile configs/visual_servo/aruco_center_mvp.toml \
  --shared-frame local_artifacts/tmp/visual-preview/latest.frame \
  --preview --local-keys --execute
```

配置默认为本轮已授权实测的100mm预算、累计40°、单步5°，底座10/15计数、手腕12计数。
这些是模型预算，不是已测量的无碰撞空间。CLI显式参数覆盖配置；默认不加profile的历史
入口仍保留原有10°/40mm设置。新实验启用前重新核验现场与姿态，不把本轮临时接管文件
或旧参考点当作新的物理许可。相机与板保持稳定完成初始响应探测后再移动标记。

## 三、画面、停止和恢复

- `ALIGNED`：目标在中央20%区域内（640×480时横向±64、纵向±48px）；暂不纠偏。
- `CENTERING`：正在纠偏；计数只在实际动作后重新对中时增加，初始静止对中不计数。
- `TARGET WAIT`：丢失标记，保持现有姿态；连续5帧重新识别后继续。
- `CAMERA WAIT`：无新鲜图像，禁止纠偏并保持；连续恢复后继续，不使用旧帧冒充实时。
- `PAUSED`：普通暂停/边界/反馈问题，画面继续更新，扭矩保持；检查原因后R重试。

本轮配置不设运行超时，也不会在计数到10后自行关闭。
在**控制窗口有焦点时**按Q/Esc正常暂停，R重试；回到控制终端、操作者已支撑后按回车
才正常卸扭矩。空格/Ctrl+C为紧急释放。普通暂停不要直接关掉相机采集进程。

`--adopt-hold --hold-reference … --resume-response …`是现场故障排查时的显式持扭矩交接，
只允许已知旧控制器暂停并关闭串口、五轴稳定保持时使用，保留原始预算参考。
它不写扭矩开关，不能作为已经卸扭矩后的新实验启动路径；通常启动不需要这些参数。

## 四、日志与离线分析

日志位于被忽略的`local_artifacts/experiments/`，包括请求/实际计数、到位或部分响应、
图像误差、固定响应矩阵、重新对中计数、输入丢失和停止原因。不会提交个人视频或机器端口。

```sh
uv run --locked python scripts/summarize_visual_servo.py \
  --log local_artifacts/experiments/20261018_125635_joint-center.jsonl \
  --output-dir local_artifacts/experiments/round1-summary
```

该脚本只读文件，输出`summary.json`、`cycles.csv`和`joint_motion.csv`，不导入相机或串口。
“首个确认动作至对中”的时间不含第一步执行延迟，不能当完整收敛时间。
最终本机归档为`local_artifacts/experiments/2026-10-18_visual-servo-round1/`，
含原始证据快照及SHA256清单，历史失败没有被覆盖。
