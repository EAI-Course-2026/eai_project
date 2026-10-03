# 本地工作区与作业产物

正式开发目录为仓库根目录。系统临时目录不是项目工作目录；此前 `/private/tmp/eai-scs215-lerobot-plugin` 用于插件验证，已结束并移除，不应再用于启动命令。当前应用与插件从正式仓库加载。

## 按生命周期存放

| 目录 | 长期保留内容 | Git 与清理规则 |
|---|---|---|
| src、plugins、tests、scripts、configs、calibration | 源码、入口、共享配置和标定 | 通过 PR 维护；个人本机配置仍忽略 |
| docs | 团队说明、验收状态、项目阶段报告 | 提交 Git；不要混入个人手工作业包 |
| deliverables/assignment-NN/YYYY-MM-DD-rNN | 固定版本作业正文、视频、源码与提交 ZIP | payload 忽略；指南提交；正式修订不覆盖 |
| deliverables/assignment-NN/archive | 早期正式包、独有草稿与旧打包方式 | 本机冷归档，不作为当前提交入口 |
| local_artifacts/experiments | 原始实验日志、PID 数据和回读证据 | 本机保留；不删失败回合或重新解释为成功 |
| local_artifacts/validation | 环境、离线测试和 CI 回执 | 按日期存放，明确软件/模拟/实物范围 |
| local_artifacts/history | 已结束的源码快照、基线和会话记录 | 本机冷归档；无需保留其虚拟环境 |
| local_artifacts/notes、diagnostics | 本地方案草稿、主机诊断 | 不作为团队已实现功能或正式源码入口 |
| local_artifacts/tmp、worktrees | 当前任务的临时文件与必要工作区 | 每任务一个清晰名称；结束后检查并清理 |
| local_artifacts/cleanup/YYYY-MM-DD | 清理计划、迁移映射、校验和回执 | 先保存材料再删除；可追查来源与去向 |
| local_artifacts/cleanup/YYYY-MM-DD/recovery | 等待确认处置的原件与旧缓存 | 可恢复的临时保留区；确认后清理副本或归档独有内容 |
| outputs | 本次程序运行的生成文件 | 不长期堆放作业 ZIP；验收后挑选归入证据或成品 |
| .venv、environments/training/.venv | 当前控制与训练环境 | 保留两套有效环境，不再复制到验证目录 |

目前旧 outputs 的独有实验日志已归档到 local_artifacts/experiments；程序后续仍按既有命令写 outputs。PID 指南中的 outputs 路径描述新运行，不代表历史记录仍在原处。历史工程草稿归档到 local_artifacts/notes，正文保持原样，旧路径只用于说明来源。

## 命名与版本

作业格式见[成品指南](../deliverables/README.md)。正式日期目录的 rNN 表示内容修订，ZIP 中 commit7 表示源码快照。报告日期、源码提交、标定、实际测试设备与证据范围分别记录，不能把“最新主线”当作“当时已验收版本”。

新实验优先使用 `YYYY-MM-DD_HHMMSS_<topic>`；材料说明 simulated、read-only、hardware、gpu 等真实验证类型，避免只写 success/latest。无法确认的结果保留 unknown/failed，不清理掉失败证据。旧日志不为改名而修改正文。

## 临时工作区和清理

一般任务直接使用正式仓库的 codex/* 分支。确需隔离时在 local_artifacts/worktrees/<task>/ 建受 Git 管理的 worktree；完成后先检查本地修改、未跟踪文件、独有提交和远程目标 CI，再用 Git 移除。不要在 /private/tmp 建长期克隆、缓存环境或报告工作区。测试与库可用自动清除的 TemporaryDirectory。

复制或移动正式材料后核对 SHA-256，检查 ZIP 可解压与内部内容。删展开副本前证明其每个文件均在保留目录或归档中。清理可重建的 dist、缓存和旧验证环境；当前两套环境保留。没有提交或存在独有内容的工作区先保存，不能按名字批量删除。

新会话的临时文本统一放 local_artifacts/tmp/<task>/；验证输出直接放日期 validation 目录。任务结束写清理回执并清掉临时目录。本次移入 history 的旧会话材料是冷归档，不是新的默认输出目录。

仓库核验会拒绝被 Git 追踪的 deliverables payload，仅允许根指南和每项作业的 README。目录规约可协作维护，个人报告、视频和含姓名的提交包继续保留本机。
