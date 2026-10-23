# 学习策略生命周期

| 功能组 | 分工 |
|---|---|
| `model_io/` | OpenAI SDK 消息/工具、Chat/Responses、流事件与环境凭据；不依赖 Agent。 |
| `agents/` | 会话历史/短时上下文、工具策略/进度、预算、取消和终态事件；不含机器人 prompt 或 HTTP。 |
| `inference/` | runtime.py 模型运行器，artifacts.py 工件，transport.py 跨机器推理传输。 |
| `execution/` | runner.py 设备所有权/动作执行，safety.py 门禁，teleoperation.py 示范采集。 |
| `data/` | recording.py 记录与片段验证，datasets.py 导出/配方，evaluation.py 评估。 |
| `contracts.py、observations.py` | 被多个功能组共用的契约和观测结构。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。

## 决策层

`model_io/` 与 `agents/` 构成模型 I/O 和通用核心；机器人 prompt、能力与技能在应用层注入。
见[三层架构](../../../docs/agent_layers.md)。VLM、ASR/TTS 与 Omni 仍未接入。
`inference/` 继续专注 VLA，不混入聊天、ASR 或 detector。
接口与外部部署分类见[推理边界决定](../../../docs/inference_boundaries.md)。
