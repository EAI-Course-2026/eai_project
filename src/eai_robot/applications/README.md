# 用户流程与界面适配

这里组织 CLI、网页、键盘、相机预览和闭环任务，拥有设备会话与交互确认。
数学计算和安全检查调用能力模块，标定流程调用 `calibration/servo.py`。
`desk_service.py` 保持单硬件线程；不要在 HTTP 处理线程或检测器里创建第二个串口所有者。
`desk_assets/` 是上位机静态资源，CI 在这里检查前端语法。

完整分工与兼容规则见 [`src/README.md`](../../README.md)。
