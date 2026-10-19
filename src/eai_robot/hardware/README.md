# 设备接入

| 功能组 | 分工 |
|---|---|
| `drivers/` | scs.py 原生协议，backends.py 双后端，inspection.py 型号/限位检查与显式释放，lerobot_scs215.py 转导出独立插件总线。 |
| `robots/` | so101.py 装配插件 Robot，joint_ranges.py 读取关节范围；不复制插件或运动学实现。 |

应用层拥有会话；算法、数据与结果读取不隐式开启设备。不同执行路径继续保留原有门禁。
完整目录与依赖规则见 [`src/README.md`](../../README.md)。
