# 学习策略生命周期

| 功能组 | 分工 |
|---|---|
| `inference/` | runtime.py 模型运行器，artifacts.py 工件，transport.py 跨机器推理传输。 |
| `execution/` | runner.py 设备所有权/动作执行，safety.py 门禁，teleoperation.py 示范采集。 |
| `data/` | recording.py 记录与片段验证，datasets.py 导出/配方，evaluation.py 评估。 |
| `contracts.py、observations.py` | 被多个功能组共用的契约和观测结构。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
