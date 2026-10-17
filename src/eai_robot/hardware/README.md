# 硬件通信与设备检查

`scs.py` 是原生协议，`backends.py` 提供 Serial/LeRobot 后端；
`inspection.py` 提供型号/硬件限位检查及显式尽力释放。
`lerobot_scs215.py` 转导出独立插件总线，不维护第二套 SDK 实现。
对象装配在 `robots/`，交互与设备所有权在 `applications/`；导入模块不打开设备。

完整分工与兼容规则见 [`src/README.md`](../../README.md)。
