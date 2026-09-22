# 2026-09-22 目录整理

| 原入口 | 新入口 |
|---|---|
| trial1.py | experiments/servos/read_position.py |
| trial2.py | experiments/servos/move_positions.py |
| trial3.py | experiments/servos/pid_response.py |
| trial3_README.md | docs/experiments/pid_response.md |
| trial4.py | scripts/servo_setup.py |
| trial5.py | experiments/servos/two_servo_control.py |

根目录不保留同名兼容脚本。终端历史或编辑器运行配置需要使用新路径。

原硬编码值迁入 `configs/hardware.local.toml`。保留用户最新编号目标 `new_id=6`，单舵机 ID 1、双舵机 ID 2/3 与原软件范围；这不表示硬件已按这些值配置。

PID 输出仍固定在项目根 `outputs/pid/`，原始 CSV、参数快照、状态文件及曲线不移动不改写。历史文件内旧路径属于当时记录，不批量替换。

`requirements-pid.txt` 合并到 `pyproject.toml` 的 experiments 可选依赖，以 `uv.lock` 固定版本。原环境检查记录保留为历史事实。

本次没有移动外部参考仓库，没有修改 LeRobot 源码，没有扫描或启动实物。
