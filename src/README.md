# 源码目录与扩展规则

本项目只保留六个一级能力目录。第一层回答“属于哪种能力”，第二层回答“在哪个功能组”。
函数、类和文件名表达更细的职责；不为每个概念再创建目录。
独立 LeRobot 插件仍在 [`plugins/lerobot_robot_scs215/`](../plugins/lerobot_robot_scs215/)。

```text
eai_robot/
├── applications/              用户入口与设备会话
│   ├── desktop/               网页服务、硬件工作线程、assets/
│   ├── teleoperation/         键盘、姿态示教/回放、语音意图与调度
│   ├── visual_servo/          相机共享预览、中央区域MVP、只读演练与实验
│   └── tools/                 检查、维护、规划、演示和策略 CLI
├── calibration/               参数识别与标定结果
│   ├── servo/                 六关节结果契约、端点采集和候选保存
│   └── vision/                相机内参/手眼流程的协作者接入说明，算法待加入
├── control/                   从目标计算到在线控制
│   ├── kinematics/            关节映射、FK/IK、URDF、几何路径
│   ├── planning/              命名姿态、固定起点、动作计划、笛卡尔目标
│   ├── execution/             接管、反馈、到位/超时、动作阶段执行
│   └── visual_servo/          图像/关节雅可比、实测响应选择与观测门禁
├── vision/                    图像观测
│   ├── detection/             ArUco/黑球检测与标记位姿估计
│   ├── geometry/              目标几何表示与表观尺寸→距离，纯numpy
│   ├── perception/            模型规格、检测器注册表、单槽异步推理
│   ├── tracking/              支撑点/光流跟踪与检测器融合状态机
│   ├── targets/               可打印/展示的视觉目标
│   └── features.py            检测与跟踪共用的观测契约
├── policy/                    学习策略生命周期
│   ├── inference/             模型工件、推理运行器、远程传输
│   ├── execution/             策略设备所有权、门禁、示范采集
│   ├── data/                  片段记录、数据集导出与评估
│   ├── contracts.py           模型/动作/资源清单的共同契约
│   └── observations.py        各推理路径共用的观测结构
├── hardware/                  设备接入
│   ├── drivers/               原生协议、后端、寄存器与硬件限位检查
│   └── robots/                Robot 配置装配与关节范围读取
├── cli.py                     eai-course 命令分派
├── config.py                  仓库配置定位与读取
└── input.py                   终端/键盘输入与数值解析
```

## 如何找代码

| 要做的事 | 位置 | 边界 |
|---|---|---|
| 打开上位机或维护设备会话 | `applications/desktop/` | HTTP 线程排队，单硬件线程操作设备 |
| 键盘/语音控制、保存和回放姿态 | `applications/teleoperation/` | 组织输入、确认与会话；调用控制模块 |
| 相机预览与视觉控制流程 | `applications/visual_servo/` | 组织采集、共享帧、MVP执行/暂停和离线统计；[模块说明](eai_robot/applications/visual_servo/README.md)区分只读与真机入口 |
| 单次检查、规划、维护或演示命令 | `applications/tools/` | 薄 CLI，复用能力，不作为其他模块的公共库 |
| 求 FK/IK 或几何路径 | `control/kinematics/` | 接收数值/模型，不拥有设备 |
| 生成动作目标和序列 | `control/planning/` | `poses.py` 读取命名关节姿态，`demo.py` 生成演示计划，`cartesian.py` 接合反馈与几何目标；不下发运动 |
| 接管、下发与监测运动 | `control/execution/` | `arm.py` 为既有 ArmController，`feedback.py` 共用回读检查，`demo.py` 执行阶段 |
| 计算相机/关节控制建议 | `control/visual_servo/` | `aruco_ibvs.py` / `math.py` 为图像数学，`joint_adapter.py` 为关节适配，`empirical.py` 为MVP纯数值响应与反馈判据；不拥有设备 |
| 从图像检测或跟踪目标 | `vision/detection/`、`tracking/` | 输出观测；不创建机械臂或发动作 |
| 表示任意物体并估计距离 | `vision/geometry/` | `targets.py` 统一四角点/圆/手部关键点/掩膜；`scale.py` 用一次参考测量替代焦距+真实尺寸；纯数值 |
| 接入新模型 | `vision/perception/` | `spec.py` 规格与设备、`registry.py` 目录与缺依赖报文、`async_detector.py` 单槽工作线程；[说明](../docs/experiments/perception_infrastructure.md) |
| 标定舵机或加入内参/手眼求解 | `calibration/servo/`、`vision/` | [标定说明](eai_robot/calibration/README.md)；采集/求解与运行时读取分开 |
| 添加模型、数据工具或策略执行 | `policy/inference/`、`data/`、`execution/` | 保留策略专用门禁，不声称已与其他执行路径统一 |
| 新设备通信或机器人构造 | `hardware/drivers/`、`robots/` | 只做接入；可安装插件不反向依赖应用 |

## 放置与依赖约束

- 新实现先进入已有功能组；只有出现多个内聚模块和明确边界，才新增第二级目录。
  不用 `common/`、`misc/` 收集无法归类的内容，也不为单个函数建立一串空包。
- `applications/` 组织交互、线程与设备生命周期；其他能力不得反向导入应用入口。
  `cli.py` 只分派，命令参数保持现有使用方式。
- `control/` 内部按“几何 → 计划 → 执行”区分，属于一个完整控制能力。
  命名关节姿态是一组保存的关节目标，不是相机位姿或任意 TCP 位姿。
- 运行时读取 `calibration/servo/results.py`，不带入采集和求解流程。
  内参和手眼的新实现放 `calibration/vision/`，结果格式由实现与协作者约定。
- 图片特征属于 `vision/`，控制律属于 `control/visual_servo/`，实际闭环会话属于
  `applications/visual_servo/`。三个阶段通过数据契约连接。
- 共享标定仍在根 `calibration/`，原始样本/候选在本机实验目录；模型、数据与视频不进入源码。
- 顶层 `features.py`、`contracts.py`、`observations.py` 是被同能力多个功能组共同使用的小型契约，
  不是新的平铺业务区；其余实现归入第二级分组。

## 迁移与验证范围

旧 `arm/`、`poses/`、`motion/`、`kinematics/`、`voice/`、`robots/` 包及转发文件已退出源码。
仓库内脚本、测试、CLI 和资源定位全部迁往新路径，不再叠加永久兼容目录。
外部代码的 Python 导入需按[路径对照](../docs/source_layout_migration.md)更新；
`eai-course` 和 `scripts/` 的用户命令、参数不变。

目录迁移将 ArUco/物体跟踪代码一并归位，路径对照记录迁移前后的接口。
2026-10-18 已完成 ArUco 中央区域真机 MVP，10次纠偏/27步反馈动作；范围和不足见
[本轮报告](../docs/reports/2026-10-18-visual-servo-round1.md)，不代表精密四角点或任意物体跟随验收。
现场结束后已卸扭矩；此后源码整理与发布检查只做离线验证，未重新启用真机。
桌面/姿态仍经 `ArmController`，视觉/策略保留各自的插件执行路径与门禁。
目录迁移不扩大共享限位，也不把软件检查当作硬件验收。
