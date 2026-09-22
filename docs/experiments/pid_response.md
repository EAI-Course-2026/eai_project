# SCS215：90° 阶跃与 PID 实验

## 运行

在项目目录、已有虚拟环境中：

```sh
.venv/bin/python experiments/servos/pid_response.py --compare
```

每组先用舵机内置位置控制回到刻度 300，再临时切换到 PWM 电机模式，由 Python PID 控制输出轴转过约 90°，采样 6 秒。默认四组为：

| 组别 | Kp | Ki | Kd |
|---|---:|---:|---:|
| P_low | 1.5 | 0 | 0 |
| P_high | 3 | 0 | 0 |
| PD | 3 | 0 | 0.25 |
| PID | 3 | 0.5 | 0.25 |

这些是实验起始值，不是经过这台舵机实测调好的最佳值。改脚本顶部 `EXPERIMENTS` 可改整组比较；单次实验可以运行：

```sh
.venv/bin/python experiments/servos/pid_response.py --kp 3 --ki 0.5 --kd 0.25
```

本机实测正 PWM 使位置刻度减小，因此 `configs/hardware.local.toml` 中 `pid.pwm_sign` 已设为 `-1`。这是执行器方向映射，不是把 PID 系数设为负数；更换设备后需重新核对方向。

默认 50 Hz，PWM 上限 250（25% 占空比）。每次比较使用同一起点，曲线以本次实际起点为 0°；目标按位置分辨率取整后大约为 90°。若运动接近目标但误差仍很大，查看输出是否长期达到 PWM 限幅，再考虑负载与参数，不能把限幅效果误判成 PID 效果。

## 与课程公式的关系

`u = Kp * error + Ki * integral(error) - Kd * measured_velocity`

- P 根据目前离目标的距离产生驱动力。
- I 累计误差，补偿持续负载或摩擦；有抗积分饱和。
- D 使用经过滤波的测量速度，提供阻尼，避免目标阶跃引起微分冲击。
- 输出 u 是有符号 PWM，不是电流或直接力矩。系数单位分别是 PWM/°、PWM/(°·s)、PWM·s/°，不能当作舵机寄存器参数写入。
- 到达目标时，误差可以为零，但积分累积值不一定为零；它可维持抵抗负载的输出。课程中的“到 90° 积分项变为 0”表述不准确。

SC 公开内存表在 21、22 地址定义 P、D，23 为未定义；本程序不把 STS 的 I 地址套给 SCS215，也不在位置命令外再套一个伪装成内环的 PID。实现使用飞特 SCSCL SDK 所描述的 PWM 模式：临时把角度上下限设为 0，向地址 44 写 PWM（bit 10 为方向），从地址 56 读位置。

角度默认按 SCS215 规格中 0..1023 对应 300°换算，90°约 307 个刻度。精确机械角度需要实际标定，`--span-deg` 可以修改换算比例。

## 输出

`outputs/pid/<时间>-hardware/` 下：

- `response.png`、`response.svg`：角度—时间、PWM—时间对比图；虚线为目标。
- `samples.csv`：每组真实时间戳、相对角度、目标、PWM、P/I/D 分量、原始位置。
- `experiment.json`：本次参数。
- `original_settings.json`：修改前的模式和设置备份。
- `status.txt`：完成或中止原因。中止时已有样本仍保存，图标题也会标注中止。

先看 P_low / P_high 的速度与超调，再比较 P_high / PD 的振荡，最后比较 PD / PID 的末端误差。空载时积分效果可能很小，不能保证四条曲线一定明显不同。50 Hz 仅能观察较慢动态，不能代替舵机内部高速控制环或示波器测量。

## 实物运行前

固定好舵机外壳，输出轴空载且有转动空间。程序有软件范围、PWM限幅、反馈超时与异常退出处理；正常退出和 Ctrl+C 会尝试关闭扭矩并恢复原设置，写入锁保持开启使模式修改不持久化。

**PWM 模式下，USB 断开、电脑卡住或进程被强杀时，软件不保证能发出停止指令。保持舵机电源可随时断开，不用于承重机械臂。** 若提示“停止/恢复未确认”，立即断开舵机电源；重新上电后重新检查位置模式。不要依赖软件边界作为机械限位。

之前的 `80 80 fe 18` 异常尚未确认修复。本程序遇到这种回复会在预检阶段停止，不启动实验。

## 离线验证

```sh
.venv/bin/python experiments/servos/pid_response.py --simulate --compare
```

这使用一个简化、有固定负载的二阶模型测试 PID 与绘图流程。输出目录和图标题均标记 **SIMULATION**，不是实测 SCS215 曲线，也不能证明实物调参已成功。

绘图依赖已安装进现有虚拟环境。如果重新 `uv sync` 后缺失：

```sh
uv sync --locked --extra experiments
```

## 协议依据

- [飞特 SCSCL Python SDK：PWMMode / WritePWM](https://gitee.com/ftservo/FTServo_Python/blob/main/scservo_sdk/scscl.py)
- [微雪 SC 内存表](https://docs.waveshare.net/Memory_Map_Explanation/SC_Servo_Memory_Map_Explanation/)
- [飞特 SCS215 产品规格：300°、1024 级](https://www.feetechrc.com/Data/feetechrc/upload/file/20200525/6372600116270632797842424.pdf)
