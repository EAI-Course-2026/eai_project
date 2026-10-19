# 用户流程与设备会话

| 功能组 | 分工 |
|---|---|
| `agent/` | cli.py 与 session.py 装配可替换文本输入、聊天 API 和本地演练技能；不连接硬件。 |
| `desktop/` | server.py 提供 HTTP/静态资源，service.py 持有单硬件线程，assets/ 保存页面。 |
| `teleoperation/` | keyboard.py、poses.py、record_pose.py 组织人工输入；voice.py、intent.py、dispatcher.py 组织识别文本、白名单与动作调度。 |
| `visual_servo/` | aruco_preview.py 相机预览，ball.py 既有黑球闭环，cli.py 统一标记/物体演练。 |
| `tools/` | arm.py 诊断/控制/标定 CLI；check_mapping.py、check_fk.py、cartesian_plan.py、reindex.py、demo.py 和 policy.py 提供显式工具。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
