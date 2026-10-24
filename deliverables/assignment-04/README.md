# 第四次作业 · 固定运动规律视觉伺服

整理说明（2026-10-24）：以下保留 2026-10-18 本机成品指南对原快照的描述，不表示本轮重新打包、提交或实物验收。源码的未提交快照与后续 Git 版本应分别追溯。

本次成品尚未提交，`status` 为 `prepared`。固定目录 `2026-10-18-r01/`。

源码是 `codex/servo-feedback-guards` 分支的**工作区快照**：基线提交 `f778342`，
但当时有 14 个文件含未提交改动（`fixed_rule.py`、`fixed_rule_servo.py`、`ball.py`、
`joint_center.py`、`camera_step.py`、`empirical.py`、`config.py`、三个测试、
两个配置与两份文档），这些改动已全部进入本包。提交并合并后如需固定版本号，
另建新修订目录，不覆盖本次快照。

- `files/源码_eai_project_visual_servo_fixed_rule.zip`：265 个文件，约 2.2 MB。
  含 `src/`、`tests/`、`scripts/`、`configs/`、`calibration/`、`plugins/`、`docs/`
  与两个锁文件；不含 `.venv`、`__pycache__`、`local_artifacts`、`checkpoints`、`outputs`、`dist`。
  不含 `configs/hardware.local.toml`——串口与相机索引属于机器本地信息，
  使用前从 `configs/hardware.example.toml` 复制填写。
- `files/视觉伺服作业报告.md`：报告正文，与 `docs/course/visual_servo_homework.md` 同源。
- `files/提交说明.txt`：来源、排除项与复现命令。
- `files/SHA256SUMS.txt`：三个成品的 SHA-256。
- `manifest.json`：分支、基线提交、快照标记与逐文件哈希。

按[统一目录规则](../README.md)，已定版内容不随主线升级重打。
