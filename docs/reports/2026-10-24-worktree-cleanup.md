# 2026-10-24 分支与 worktree 整理回执

用户明确要求清理本地/远程分支与四个 worktree。本轮按 CONTRIBUTING 的合并、独有提交、
工作区状态、使用情况与合并后 CI 条件执行。没有合并新 PR、强推、删除 main/develop、
重置长期分支或清掉原始工作区中的未提交资料。

## 结果

| 项目 | 整理前 | 整理后 |
|---|---:|---:|
| worktree | 4 | 2 |
| 本地分支 | 11 | 6 |
| 远程分支，不计 origin/HEAD | 8 | 5 |

移除的两个 worktree 均无未提交/未跟踪源码、无独有提交，检查时无进程以它们为工作目录。
`eai_project-layout` 内容已进入 develop；`eai_project-agent-plan` 内容已进入 develop，
其中本机测试日志和构建产物共 7 个文件先复制、逐文件 SHA-256 校验后归档。
其可重建 `.venv` 和 Python 缓存随已完成 worktree 移除，不改其他工作区环境。
归档位于原始工作区被 Git 忽略的
`local_artifacts/worktree-archives/2026-10-24-agent-plan/`，含 manifest；不提交本机日志或 wheel。

## 删除依据

下列五条本地分支均无相对 origin/develop 的独有提交；涉及的目标合并 SHA
均确认对应 push 工作流三项必需检查全部 success。

| 本地分支 | 对应已合并 PR / 归属 | 合并后检查 |
|---|---|---|
| codex/compact-source-layout | head 已是 PR #14 merge `cbfaba2` | [39042543205](https://github.com/EAI-Course-2026/eai_project/actions/runs/39042543205) |
| codex/fixed-rule-object-servo | PR #17 merge `f778342` | [39107545985](https://github.com/EAI-Course-2026/eai_project/actions/runs/39107545985) |
| codex/multimodal-agent-plan | PR #20 merge `7877a4f` | [39247204654](https://github.com/EAI-Course-2026/eai_project/actions/runs/39247204654) |
| codex/learning-agent-runtime | PR #21 merge `2eb8470` | [39785380177](https://github.com/EAI-Course-2026/eai_project/actions/runs/39785380177) |
| codex/integration-standard-agent | PR #23 merge `aa19154` | [39987830847](https://github.com/EAI-Course-2026/eai_project/actions/runs/39987830847) |

远程已删除 `codex/multimodal-agent-plan`、`codex/learning-agent-runtime`、
`codex/integration-standard-agent`。另外两条对应的远程分支此前已不存在。
执行普通 `git branch -d` / `git push --delete`，未使用强制删除或强推。
之后 fetch --prune，未占用的本地 main/develop 仅以快进方式同步 origin：
main=`dc5d235`，develop=`aa19154`。

## 为什么保留两处工作区和其他分支

| 保留项 | 原因 |
|---|---|
| eai_project / codex/servo-feedback-guards | 有 README、课程首页、voice README 三处已跟踪修改，以及未跟踪的 voice ARCHITECTURE 文档；本机硬件配置、环境、输出、检查点和资料完整保留。为避免改变其工作上下文，关联本地/远程分支也保留 |
| eai_project-agent-runtime / codex/integration-cloud-speech | 正在交付和评审的 PR #25，含当前本机 API 配置和真实服务证据，不能作为已完成任务删除 |
| codex/integration-chat-acceptance，本地/远程 | PR #24 仍 OPEN，有尚未集成的聊天验收代码 |
| codex/local-artifact-layout，仅本地 | PR #9 CLOSED 且未合并，仍有 1 个相对 develop 的独有提交 `6b6c97a`；未以“远程已删”作为丢弃本地工作的理由 |
| main / develop | 永久分支；develop→main 的 PR #22 仍 OPEN，保留正常发布流程 |

本轮未提交、覆盖、stash 或搬动原始工作区的四份文档修改；未删除当前 Agent Key、录音证据、
训练环境、数据、模型、硬件配置或原始工作区输出。
后续整理应先评审/提交原始工作区文档，明确 PR #9 的独有内容归属，再按相同规则清理，
不通过强制切分支或 git clean -fdx 消除这些保留项。

当前 PR #25 只有在维护者批准、实际合并且目标分支对应 push CI 全绿后才符合最终删除条件。
本回执记录整理时刻的分支状态，不保证今后新建分支后的数量不变。
