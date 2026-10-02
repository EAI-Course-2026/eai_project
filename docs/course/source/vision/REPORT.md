> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# 基于视觉反馈的机械臂闭环控制实验报告

## 1. 任务目标

本实验使用 SCS215 机械臂末端摄像头实时观察电脑屏幕上的黑色圆形目标。程序自动检测黑球在图像中的位置，根据目标中心与画面中心之间的偏差生成机械臂末端的小步笛卡尔位移，并不断重复“采集图像—计算误差—执行动作—重新采集”的过程，使机械臂运动方向明显朝向黑球。

在图像平面居中控制之外，程序还可根据黑球半径随运动产生的变化，在线判断机械臂靠近目标的笛卡尔方向，并逐步向前移动。整个运行过程不需要人工遥操作。

## 2. 实验环境

- 机械臂：SCS215 六舵机机械臂（闭环控制使用关节 1–5）
- 电机串口：`COM7`（USB-Enhanced-SERIAL CH343）
- LeRobot 标定 ID：`scs215_com5`
- 视觉传感器：机械臂腕部 USB 摄像头，OpenCV 相机编号 `1`
- 软件：LeRobot、Python、OpenCV、NumPy
- 运动学：复用 Week 4 Task 2 中经过验证的标定映射、正运动学、位置逆运动学和笛卡尔直线规划


## 3. 程序结构

- `visual_closed_loop.py`：主程序，负责相机读取、硬件连接、在线探测、闭环控制和安全停止。
- `ball_detector.py`：黑色圆形检测、轮廓筛选和结果标注。
- `visual_math.py`：图像雅可比检查、阻尼最小二乘和坐标轴映射。
- `test_vision.py`：不连接机械臂即可执行的视觉与控制数学单元测试。
- `black_ball_target.html`：在电脑屏幕上显示可调节位置和大小的黑球。

## 4. 实现原理

### 4.1 黑球检测

摄像头返回 RGB 图像后，程序依次执行：

1. 转换为灰度图并进行高斯滤波；
2. 使用灰度阈值提取黑色区域；
3. 使用椭圆形结构元素进行开运算和闭运算，去除噪点并填补边缘；
4. 提取全部层级的轮廓；
5. 根据面积比例、圆度、最小外接圆填充率和长宽比筛选圆形候选；
6. 选择置信度较高的黑色圆形，输出圆心、半径和置信度。


### 4.2 在线估计图像雅可比

不同相机安装角度下，机械臂坐标轴与图像水平、竖直方向之间的对应关系可能不同。程序没有硬编码方向，而是在正式控制前沿两个笛卡尔轴分别执行小幅、可恢复的探测动作。

设图像特征为黑球圆心

\[
\mathbf{s}=[u,v]^T,
\]

末端在两个控制轴上的位移为

\[
\Delta\mathbf{x}=[\Delta x_1,\Delta x_2]^T.
\]

通过探测动作测量每个方向上的图像位移，可得到局部图像雅可比矩阵

\[
\mathbf{J}\approx\frac{\Delta\mathbf{s}}{\Delta\mathbf{x}}.
\]

实验中使用 `X、Z` 作为图像平面控制轴。程序会检查雅可比矩阵各列的有效运动量和条件数，避免在不可观测或近似奇异的方向上继续控制。

### 4.3 图像闭环控制

设画面中心为 \(\mathbf{s}^{*}\)，当前黑球圆心为 \(\mathbf{s}\)，像素误差为

\[
\mathbf{e}=\mathbf{s}^{*}-\mathbf{s}.
\]

程序使用带阻尼的最小二乘方法将像素误差转换成末端位移：

\[
\Delta\mathbf{x}
=k\mathbf{J}^{T}\left(\mathbf{J}\mathbf{J}^{T}+\lambda^{2}\mathbf{I}\right)^{-1}\mathbf{e},
\]

其中 \(k\) 为闭环增益，\(\lambda\) 为阻尼系数。每一步位移都会被限制在给定上限以内，再交给已有的位置 IK 和笛卡尔直线规划器生成关节目标。

动作执行后，程序重新读取摄像头，而不是一次性计算完整轨迹。因此目标位置变化或机械臂实际运动存在误差时，下一帧仍会重新修正，这构成了视觉反馈闭环。

### 4.4 向目标靠近

仅让黑球位于画面中央只能完成二维对准，不能产生明显的前伸运动。启用 `--approach` 后，程序会沿剩余的深度轴执行可恢复探测，并观察黑球半径变化：

- 半径变大：相机正在靠近目标；
- 半径变小：相机正在远离目标。

程序据此自动确定深度轴的前进符号。正式运行时，在保持黑球接近画面中心的同时逐步沿该方向运动；达到目标半径、最大靠近距离、总位移限制或运动学限制时停止。

## 5. 运行方法

先验证摄像头与检测，不会驱动机械臂：

```powershell
python visual_closed_loop.py --camera-index 1 --preview
```

建议先用较短距离完成验收：

```powershell
python visual_closed_loop.py --execute --approach --control-axes x z --depth-axis y --probe-mm 8 --depth-probe-mm 16 --probe-settle-s 2 --max-step-mm 2 --approach-step-mm 1.5 --max-approach-mm 75 --max-travel-mm 80 --target-radius-px 150 --max-runtime-s 120
```

如果已实测末端到屏幕沿运动方向具有至少 250 mm 净空，可把最大靠近距离配置为 200 mm：

```powershell
python visual_closed_loop.py --execute --approach --control-axes x z --depth-axis y --probe-mm 8 --depth-probe-mm 16 --probe-settle-s 2 --max-step-mm 2 --approach-step-mm 1.5 --max-approach-mm 200 --max-travel-mm 220 --target-radius-px 150 --max-runtime-s 180
```

长距离模式需要输入：

```text
START VISUAL SERVO 200MM
```

## 6. 实验现象与结果

实际连接 SCS215、COM7 和腕部相机后，程序能够：

1. 自动检测屏幕中的黑球并输出圆心、半径和置信度；
2. 自动执行 X、Z 小幅探测并计算图像雅可比；
3. 根据实时像素误差自动控制机械臂朝黑球方向运动；
4. 黑球移动后继续根据新图像进行修正；
5. 启用靠近模式后，根据黑球半径变化判断前进方向；
6. 在目标丢失、运动学不可达或达到位移限制时安全停止。

一次实际探测得到的局部图像雅可比为：

```text
[[-6715.3     5.8]
 [ -170.7 -3987.4]] pixel/m
```

两列运动方向明显独立，能够用于二维图像误差控制。
