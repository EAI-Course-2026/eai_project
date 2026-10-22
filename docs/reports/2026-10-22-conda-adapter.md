# Conda 薄适配与 Quick start

2026-10-22。uv 仍是主工作流。只新增独立的标准库脚本 `scripts/setup_conda.py`、
对应回归与文档，不增加业务 CLI、运行时模块、第二套包清单或依赖解析逻辑。

脚本通过标准 conda create/run 创建 Python 3.12 环境，在目标中引导 uv 0.11.7，
对子进程显式指定目标前缀后执行原项目的 `uv sync --locked --inexact`。
根控制与独立训练继续使用各自已有锁文件、来源与 CPU/CUDA 索引。Conda 引导包保留，
额外包不自动清除，冲突由依赖检查报告；这不是完整 Conda 包级别的可复现锁。

base/root 和仓库 .venv 禁止作为目标，已有环境需要显式 --reuse，错误立即停止。
控制默认运行完整离线测试与固定 demo；训练默认只验软件，真实 CUDA 运算显式选择。
README 给出一键命令、激活后普通命令与 conda run；环境指南补充更新、复用和故障边界。

本地仅保留 uv 验证：新增 5 项安装编排/目标门禁回归通过，完整 559 项通过。
真实 Conda smoke 放入现有 macOS/Windows CI 任务，固定 setup-miniconda action 提交与
Miniforge 版本；不修改三项检查名称或跳过原本的完整 uv 测试。
不在用户本机安装或保留 Conda。托管验证结果与准确 head 以 PR 描述/Checks 为准。
未执行硬件、相机、真实模型调用或训练任务；训练/GPU/实物能力仍分别验收。
