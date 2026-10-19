# 两条研发主线、推理边界与分支整理

2026-10-19。本轮基于 develop `1cb68c9`，任务分支 `codex/multimodal-agent-plan`。

## 交付范围

确定模型与规则控制、学习感知与决策两条主线，共用 main/develop。
已有 VLA 运行器、远端服务和检测器继续维护在本仓库；框架、权重、外部 ASR 进程、
自有远端服务与托管 API 分别管理。新增 Agent/语音接口目前是设计约定，未宣称已实现。

方案入口：[主线与分支](../development_workstreams.md)、[推理边界](../inference_boundaries.md)、
[Agent 开发切片](../agent_development_plan.md)。更新导航与里程碑，修订历史 PR 合并状态。

## CI 失败修正

已有 develop → main PR #19 的[运行 39242069814](https://github.com/EAI-Course-2026/eai_project/actions/runs/39242069814)
在 macOS 的 `test_connect_is_read_only_and_enable_gates_manual_control` 等待 worker 完成时超时。
同 SHA develop push [39242004086](https://github.com/EAI-Course-2026/eai_project/actions/runs/39242004086)三项成功，
两者分别记录，不用其中成功的一次掩盖失败。

该用例的 FakeArm 即时跟随目标，但 run_stage 仍每步真实等待 80ms，另有逐步 FK 刷新。
本轮仅在该用例的全行程动作期间替换 run_stage 模块的时钟对象，跳过其步间 sleep；
不修改全局 time，不替换 arm.move，不改生产代码或运动期限。原有终点、步长、启用与停止断言保留。

## 分支与验证回执

已删除四个远端临时引用：`codex/aruco-visual-servo`、`codex/visual-servo-round1`、
`codex/fixed-rule-object-servo`、`codex/voice-command-layer`。
删除前逐一核对远端头等于已合并 PR 的 head，且均为 develop 祖先、没有本地活动 worktree；
目标 develop SHA 的三项 push CI 成功。仍在使用的 worktree、本地历史与实验资料保留。
`codex/servo-feedback-guards` 因现场 worktree 仍使用而保留，未合并的 local-artifact 分支保留。

本地锁定环境同步、环境核验、文档与链接检查、前端语法检查及两个 wheel 构建通过。
完整测试与本轮托管检查的最终回执记录在 PR 描述/Checks，须核对对应 head。
本轮未合并，故没有本轮 post-merge CI；不代替另一位维护者审查或合并现有 PR #19。
本轮未采集相机/麦克风、调用模型服务或运行实物运动。
