> Historical source document, imported from eai-course-lerobot at 6a077907. For current commands, use docs/course/README.md.

# Voice

后续语音控制代码放在本目录。建议将流程拆成语音识别、命令解析和动作调度三层。
识别文本只能映射到预先定义的命令枚举，例如 `stand`、`wave`、`stop`，不能把
任意识别结果直接执行为代码。停止命令应始终能抢占普通动作。

完整接入顺序和安全约束见 [项目交接文档](../HANDOFF.md#9-语音功能的建议边界)。
