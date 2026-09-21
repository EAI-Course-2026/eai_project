# 环境检查：2026-09-21

## 已确认

- macOS 26.6.2，Apple M5，24 GB 内存，arm64。
- 可用磁盘空间约 596 GiB（检查时）。
- Git 2.54.0、uv 0.11.7、FFmpeg 8.1 已安装。
- Homebrew Python 3.12.13 可用；默认系统 python3 为 3.9.6。
- 所检查的系统 Python 和 Homebrew Python 3.12 未安装 torch、lerobot、cv2、serial。
- 项目目录初始化前为空；projects 本身不是 Git 仓库。
- 未发现 /dev/cu.usb* 或 /dev/cu.wch* 机械臂候选串口。

## 尚未验证

- 项目虚拟环境依赖安装与锁定。
- PyTorch MPS 实际可用性。
- LeRobot CLI、视频编码解码与摄像头采集。
- 机械臂连接、leader/follower 端口映射、校准、遥操作。

硬件缺失不代表软件安装失败；MPS 需要在安装 PyTorch 后实际检查。

官方 PyPI 页面显示 LeRobot 0.6.1，Python >=3.12。使用该发布版本初始化，
先安装 core_scripts 和 feetech；training 为可选依赖。

## 安装及验证结果

上述“尚未验证”中的软件安装、MPS 和视频编解码现已完成：

- 项目 `.venv` 使用 Python 3.12.13，`uv sync` 成功；`uv.lock` 已生成。
- LeRobot 0.6.1、PyTorch 2.11.0、Torchvision 0.26.0、TorchCodec 0.11.1。
- OpenCV headless 4.13.0.92、PyAV 15.1.0、pyserial 3.5、Feetech SDK 1.0.0。
- 环境检查脚本退出码 0，所有目标模块导入成功。
- 沙箱外 MPS available=True，小型张量加法/求和通过；沙箱内硬件检测为 False。
- FFmpeg 生成 10 帧 H.264 视频成功，PyAV 与 TorchCodec 均成功解码 10 帧。
- `lerobot-info` 与 `lerobot-record --help` 退出码均为 0。
- 未安装可选 training 依赖，未运行模型训练或模型级 GPU 推理。
- 摄像头采集、机械臂连接及校准仍未验证。

## 已观察到的兼容性问题

同时加载 OpenCV、PyAV 或系统 FFmpeg 时出现 Objective-C AVFFrameReceiver /
AVFAudioReceiver 重复类警告。本次导入、解码和 CLI 检查成功，但不代表摄像头采集链路
已通过验证。未隐藏警告或修改包内动态库；真实摄像头验证时需继续观察。

## 后续重新检查

```sh
uv sync --locked
uv run --no-sync python scripts/check_env.py
```

训练前执行 `uv sync --locked --extra training`。若依赖发生变化，重新验证并审阅锁文件。
