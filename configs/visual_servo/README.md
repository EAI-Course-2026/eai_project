# 视觉伺服参数配置

`aruco_center_mvp.toml`固化2026-10-18真机演示配置，仅用于中央区域跟随。
通过`eai-course vision --profile PATH`加载，显式CLI参数覆盖配置。

配置只接受经过类型检查的servo参数；不能设置`execute`、`adopt_hold`、串口、相机索引或
本机临时文件路径。不开`--execute`时仍是只读相机预览。配置100mm/40°描述本轮模型预算，
不代表未知场景的物理安全空间；新一轮重新核验姿态和现场。

精密IBVS尚未验收，暂不提供名称看似可直接真机运行的precision配置。
结果见[本轮报告](../../docs/reports/2026-10-18-visual-servo-round1.md)，
下一步见[精细伺服计划](../../docs/experiments/visual_servo_precision.md)。
配置读取与字段白名单见[profiles.py](../../src/eai_robot/applications/visual_servo/profiles.py)。
