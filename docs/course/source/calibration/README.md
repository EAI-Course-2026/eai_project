> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# 团队共享机械臂标定

`scs215_com5.reference.json` 是 2026-09-28 从当前课程机械臂导出的 LeRobot
SO follower 标定。团队成员目前使用同一台实体机械臂，因此这份文件是当前团队
统一的标定基线。它包含舵机 ID、方向和六个关节的 raw 安全范围。

## 可以直接使用的情况

队友连接这台共享机械臂，并且舵机、舵盘和连杆没有重新安装时，可以直接把它
复制到自己的 LeRobot 缓存目录，不需要重复标定。Windows 分配的 COM 端口可以
不同；运行程序时通过 `--port` 指定实际端口。

在仓库根目录执行：

```cmd
mkdir "%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower" 2>nul
copy "examples\eai_course\calibration\scs215_com5.reference.json" "%USERPROFILE%\.cache\huggingface\lerobot\calibration\robots\so_follower\scs215_com5.json"
python examples\eai_course\week4\task2\check_step1.py --port COM5 --robot-id scs215_com5
```

如果目标位置已经存在 `scs215_com5.json`，先备份，不要直接覆盖。

## 不能直接使用的情况

以下任一条件成立时必须重新运行 `lerobot-calibrate`：

- 使用另一台机械臂或另一组舵机。
- 舵机 ID、舵盘安装齿位或连杆装配发生变化。
- 维修后关节物理范围发生变化。
- 检查结果显示当前位置越界，或 wrist roll 接近 `0/1023` 跳变边界。

参考 JSON 不会自动移动机械臂，但后续控制程序会依据其中的范围计算目标位置。
错误标定可能导致机械碰撞，因此第一次运行必须支撑机械臂并使用低速。
