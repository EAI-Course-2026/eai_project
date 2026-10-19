# 相机内参与手眼标定接入

新增视觉标定代码直接放这里，按阶段组织模块，先不为每种算法再建空目录：

```text
vision/
  capture.py       相机/配对位姿采集与样本清单
  intrinsics.py    内参、畸变、逐视图与整体重投影误差
  hand_eye.py      手眼求解、坐标方向、残差与退化检查
  results.py       轻量结果契约/加载器，在 schema 确定后实现
```

上面的模块尚未实现。求解函数应可离线测试；设备通过参数注入，配对采集可读取设备，
不得隐式命令运动。需要自动采样运动时，另由显式启用的应用会话调用执行接口。
相机/手眼数据不能交给 `calibration.servo.results.load()`，它只处理现有舵机 JSON。

结果记录资产身份、镜头/分辨率、安装配置、坐标系与变换方向、单位、采样清单和哈希、
求解器/源码版本、误差、审核信息与失效条件。schema 和阈值由视觉实现明确后约定。
共享已审核结果仍在根
[`calibration/vision/`](../../../../calibration/vision/README.md)，原始样本和候选留在本机实验目录。

图像检测位于 `vision/detection/`，视觉控制律位于 `control/visual_servo/`，
用户预览与闭环流程位于 `applications/visual_servo/`；标定算法不混入这三者。
完整分工见 [`src/README.md`](../../../README.md)。
