# 两条开发主线与分支安排

2026-10-19。两条主线用于安排研发；模块通过数据契约组合，精度、反馈和停止是共同要求。

| 主线 | 范围 | 下一步与完成判据 |
|---|---|---|
| 模型与规则控制 | FK/IK、轨迹、固定规律/IBVS 视觉伺服、本地反馈执行 | 优先处理承重保持误差、肩肘配对及腕旋行程分配，再复验对齐/前伸/停靠；记录失败与成功，不把短程前伸当作完整停靠 |
| 学习感知与决策 | 学习检测、VLM 动作模仿、LLM Tools、语音多模态 Agent、VLA | 先完成文本 + 外部 LLM + 已有技能 dry-run；然后模仿/视觉语音；匹配数据与模型后再验收 VLA |

学习检测器属于 vision，规则视觉伺服属于 control，两者可以组合成同一个应用。
Agent 可选择规则控制或学习策略；voice 是交互方式，端侧/远端/托管 API 是部署方式。
不因此建立第三条长期开发线，也不把学习代码全部搬进 policy。

## Git 组织

永久分支只有 `main`、`develop`。main 是集成基线，develop 是共同集成点。
新增工作从最新 develop 创建短期分支，一项交付一个 PR，目标 develop：

| 任务类型 | 命名约定（示例，不预建空分支） |
|---|---|
| 控制改进 | `codex/control-hold-compensation`、`codex/control-wrist-allocation` |
| 学习与 Agent | `codex/learning-agent-tools`、`codex/learning-vlm-imitation` |
| 共用基础 | `codex/integration-skill-session` |

既有有内容的分支保留原名，不为统一命名改写历史。两条主线共享反馈、设备和停止契约，
不维护长期 `control`/`learning` 分叉；不要让 Agent 另建一条绕过本地门禁的硬件路径。

## 本轮分支核对

核对基线：develop `1cb68c9ed1828abb201bc853f804bc5535cdc456`，已包含 PR #18 伺服反馈修正。
该 SHA 的 push 工作流[39242004086](https://github.com/EAI-Course-2026/eai_project/actions/runs/39242004086)三项成功。
同一 SHA 的 develop → main PR #19 曾在 macOS 桌面模拟测试等待失败；后续修正与回执见
[本轮报告](reports/2026-10-19-agent-boundaries.md)，不能用 push 成功代替该 PR 检查。

| 分支 | 处理 |
|---|---|
| `main`、`develop` | 保留；不代替维护者审查或合并现有 PR #19 |
| `codex/aruco-visual-servo` | PR #14 已合并，已核对远端头及 develop 祖先关系，已删除远端引用 |
| `codex/visual-servo-round1` | PR #16 已合并，已核对并删除远端引用 |
| `codex/fixed-rule-object-servo` | PR #17 已合并，已核对并删除远端引用；保留本地历史 |
| `codex/voice-command-layer` | PR #12 已合并，已核对并删除远端引用 |
| `codex/servo-feedback-guards` | PR #18 已合并，但本地现场工作区仍使用，保留 |
| `codex/compact-source-layout` | 仍有本地 worktree，保留，不删除目录 |
| `codex/local-artifact-layout` | PR #9 关闭未合并，含独有提交，保留 |
| `codex/multimodal-agent-plan` | 本轮架构整理与接口设计任务分支，提交 PR 后保留至完成条件满足 |

清理仅针对已验证可删除的远端任务引用；本地环境、worktree、原始实验与视频保留。
未合并 PR、独有提交或仍在使用的分支均不按名称批量删除。

## 落地顺序与交付边界

控制线先解决现有伺服不完整之处，详见[反馈记录](reports/2026-10-19-servo-feedback-guards.md)。
学习线按[Agent 切片](agent_development_plan.md)推进；接口与外部分类以
[推理边界决定](inference_boundaries.md)为准。新代码随功能 PR 增加，当前不预建空运行框架。

每个 PR 分列离线软件、真实 API、GPU、实物证据；合并前检查准确 head，合并后检查目标 SHA。
控制线的能力稳定后可直接成为学习线工具，学习线无需等待精密停靠全部完成才开始 dry-run。
