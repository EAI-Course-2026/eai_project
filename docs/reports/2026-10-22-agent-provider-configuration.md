# 多提供方配置与 dotenv 接入

2026-10-22。延续[标准 SDK 修正](2026-10-22-agent-standard-sdk.md)，对应同一 PR 的后续提交。

## 交付

- `eai-course agent-config list/init/check` 与 `scripts/configure_agent.py`：生成 TOML + 空本机密钥文件，离线检查配置；不请求模型。
- 预设包含千问/百炼北京及其他区域、DeepSeek、Kimi、MiniMax、OpenAI 和 custom。
  URL 与文档来源随 wheel 安装，不依赖源码路径或本机其他项目。模型 ID 必须由操作者选择。
- 支持自定义 base_url、凭据变量、部署位置、Chat/Responses、流式模式和 token 参数。
- python-dotenv 显式读取 `--env-file`，进程环境优先；不发现 .env、不插值、不污染全局环境。
- 创建文件使用排他方式，不覆盖已有文件；POSIX 密钥文件权限 0600，默认路径受 Git 忽略。
- SDK extra_body 容纳提供方扩展；禁止替换路由、消息、工具、流、预算及会话状态。
- 默认省略服务端 strict，可显式选择；本地参数校验始终启用。标准注册定义不由适配器改写。
- 回归确认完整 assistant/reasoning_content 在 JSON 与 SSE 工具续轮中保留。

## 最终验证

完整本地 **554 项测试通过**，Agent 专项 **55 项通过**。配置新增回归覆盖所有预设生成、
密钥选择/优先级、不自动发现与不插值、不覆盖/回滚、离线检查、脱敏、CLI dotenv 端到端、
自定义 Responses/无认证本地服务、扩展参数门禁、reasoning 续轮和 strict 的本地校验。

锁定同步、环境/仓库/文档检查、JavaScript 与两个 wheel 构建通过；独立训练锁文件核验通过且未修改。
解包 wheel 核对模型模块、prompt 和提供方目录与源码一致，并在独立工作目录运行
离线 demo、标准工具导出、预设列表、配置生成与离线检查，全部通过。

模板端点于当天从官方文档核对，见[多服务 API 配置](../agent_providers.md)的来源。
不代表已验证各家真实账户/具体模型/工具能力。没有真实 API Key，未调用真实模型、采集设备或机器人串口。
最终准确 head 的托管三项 CI 回执以 PR 描述/Checks 为准。PR 未合并，无本轮合并后 CI。

## 托管回归修正

提交 `f64fe43` 的[运行 39799147898](https://github.com/EAI-Course-2026/eai_project/actions/runs/39799147898)
中，Windows 两项成功，macOS 554 项里首个 401 映射测试失败：普通 HTTP fixture 的 2 秒请求预算
先被 SDK 冷启动消耗，得到 timeout 而未读取到 401。修正将普通协议 fixture 预算与显式短期限测试分开，
普通默认 10 秒；真实超时用例仍使用独立的短期限。未改变生产预算、控制频率、运动租期或门禁。
最终托管结果核对后写入 PR，保留本次失败记录。
