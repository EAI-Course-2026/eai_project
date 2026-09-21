# SO-ARM101 Lab

SO-ARM101 遥操作、数据采集与模仿学习实验项目。依赖 LeRobot 发布版本，不直接复制上游源码。

## 环境准备

使用 Python 3.12 和 uv。LeRobot 固定为 0.6.1；完整传递依赖将在成功执行 uv sync 后记录在 uv.lock。

```sh
uv sync
uv run --no-sync python scripts/check_env.py
```

需要训练时安装可选依赖：

```sh
uv sync --extra training
```

## 文件组织

- `scripts/check_env.py`：软件导入、MPS 运算、FFmpeg 和候选串口检查；不打开摄像头，不驱动电机。
- `docs/environment.md`：首次环境检查结果及待办。
- `configs/hardware.example.toml`：硬件信息登记模板，不直接作为 LeRobot CLI 配置。
- `data/`、`outputs/`、`checkpoints/`、`calibration/`：本地数据，已在 Git 中忽略。

## 首次硬件接入

1. 确认是否有 leader 和 follower 两只机械臂、装配及供电情况。
2. 分别连接 USB，辨认各自串口，填写硬件登记表。
3. 安装完成后按官方 SO-101 指南执行电机配置和校准。
4. 确认摄像头权限及画面，先验证遥操作，再采集短 episode。
5. 数据读写和视频解码验证通过后，再开始 ACT 训练与部署。

初始仓库不包含可直接执行的电机运动命令，端口与校准信息必须来自真实设备。

## 官方参考

- https://huggingface.co/docs/lerobot/installation
- https://huggingface.co/docs/lerobot/so101
- https://pypi.org/project/lerobot/0.6.1/

主分支教程与发布版可能不同，执行具体命令前以本项目安装版本的 --help 为准。
