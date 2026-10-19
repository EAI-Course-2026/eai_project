# 学习策略生命周期

| 功能组 | 分工 |
|---|---|
| `agents/` | 文本聊天协议、可配置 HTTP provider、工具白名单与有界循环；不拥有硬件。 |
| `inference/` | runtime.py 模型运行器，artifacts.py 工件，transport.py 跨机器推理传输。 |
| `execution/` | runner.py 设备所有权/动作执行，safety.py 门禁，teleoperation.py 示范采集。 |
| `data/` | recording.py 记录与片段验证，datasets.py 导出/配方，evaluation.py 评估。 |
| `contracts.py、observations.py` | 被多个功能组共用的契约和观测结构。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。

## 决策层

`agents/` 已提供文本 LLM 工具循环；VLM、ASR/TTS 与 Omni 仍未接入。
`inference/` 继续专注 VLA，不混入聊天、ASR 或 detector。
接口与外部部署分类见[推理边界决定](../../../docs/inference_boundaries.md)。
