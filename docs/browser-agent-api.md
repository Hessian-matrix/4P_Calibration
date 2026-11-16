# Agent 标定接口契约

状态：接口设计，不是已部署 API；当前 `web/design/` 只有交互原型，没有 `window.fourP`、MCP server 或标定引擎。本文定义正式实现应遵守的单一契约，不提供返回假成功的接口占位。配套 [技术架构](wasm-acceleration-plan.md) 与 [交互规格](web-workbench-design.md)。

## 1. 静态站点能做什么

GitHub Pages 可以提供 JS SDK、WASM、协议文档和静态 JSON Schema；不能监听外部 POST / HTTP RPC / SSE / WebSocket 请求。Service Worker 只能拦截其控制的浏览器客户端请求，不会把访问者设备变成公网 API 服务。

推荐入口是 **Agent 控制浏览器 → 页面内 JS API → 共享任务控制器 → Worker/WASM**。人类按钮也调用同一个控制器，不维护第二份配置或求解逻辑。计算发生在浏览器所在的机器：Agent 使用云端 headless 浏览器时，消耗云端机器算力，不是另一位网站访问者的电脑。

| 调用方式 | Pages 单独可承载 | 计算位置 / 条件 |
|---|---|---|
| 页面内 SDK + 浏览器自动化 | 是 | 浏览器所在设备；本地文件仍需选择/授权或自动化显式注入文件 |
| WebMCP 工具注册（可选适配） | 静态脚本可以注册，但取决于浏览器支持 | 浏览器所在设备；不能当成普遍支持的远程 MCP HTTP 服务 |
| 本地 stdio MCP 包装器 | 否，需另装本地进程 | 推荐让无浏览器 Agent 调用现有 CLI；复用同一配置和运行合同 |
| 远程 HTTP / MCP 服务 | 否，需后端 | 默认后端设备计算；若要调度用户浏览器，需显式连接、鉴权和在线会话，不在纯 Pages 范围 |

浏览器页面不是无人值守长期服务：关闭标签、刷新、休眠、浏览器回收都可能终止任务。需要稳定队列/后台批量标定时用 CLI + 本地 MCP，而不是承诺标签关闭后继续计算。

## 2. 建议公开面

API 名称为设计目标，版本为 `fourp.browser.v1`。通过页面 `window.fourP` 暴露 `request(envelope)` 与 `subscribe({runId, afterSeq}, callback)`，返回普通可序列化对象；订阅返回解除函数。接口安装完成后才发布就绪事件 `fourp:api-ready`。SDK 不是基于 CSS 选择器驱动表单的自动化脚本。

当前本地实现先显示用途与数据说明，用户或受其授权的浏览器自动化须点击“加载本地计算引擎”；`fourp:api-ready` 仅表示 JS 接口安装，不表示引擎已加载。首次 Service Worker 控制/隔离刷新会销毁此前的 JS 对象，自动化应等待页面引擎就绪后重新取得 `window.fourP`，再绑定文件。页面不在首次打开时偷偷下载引擎或启动计算线程。

同一页面内调用不是安全隔离边界，任意同源脚本都能读页面数据；因此不得引入不可信第三方脚本。跨窗口/iframe 的 MessagePort 适配若增加，必须在用户授权会话时绑定精确 origin、event.source 和独占端口；不广播文件，不使用 `targetOrigin: '*'`，不把会话凭据放进 URL。

### 请求信封

```json
{
  "protocol": "fourp.browser.v1",
  "id": "req-17",
  "method": "runs.start",
  "params": {
    "draftId": "draft-3",
    "expectedRevision": 12,
    "preflightId": "check-9",
    "idempotencyKey": "job-2026-11-12-001"
  }
}
```

成功：`{protocol, id, result}`。失败：`{protocol, id, error:{code,message,details,retryable}}`。每个请求恰好一个响应；启动响应只表示任务被接受，**不表示标定成功**。RPC 超时只说明未收到响应，不自动取消任务或重复启动。

### 方法合同

| 方法 | 输入 | 返回 / 语义 |
|---|---|---|
| `capabilities.get` | 无 | 引擎/build/schema 版本、初始化阶段、支持容器/模式、两种 CPU 执行档及实际线程数；只报告已初始化的真实能力 |
| `sources.list` | 无 | 当前会话已授权 sourceId 与文件元数据；不返回任意本地绝对路径 |
| `sources.bind` | 显式格式、文件绑定集合、relativePath | 返回 sourceId/sourceRevision；File 是结构化克隆/SDK对象，不嵌入JSON、不base64大bag；无已授权文件则拒绝 |
| `sources.inspect` | sourceId/sourceRevision | 返回 inspectionId；异步扫描，`inspections.get` 获取状态/真实话题/错误 |
| `inspections.get` | inspectionId | pending/running/completed/failed/cancelled；结果绑定sourceRevision |
| `inspections.cancel` | inspectionId | 请求取消，最终状态必须等待实际 Worker 退出 |
| `drafts.create` | sourceId、现有 RunSettings 配置及文件引用绑定 | draftId/revision；配置仍由 Rust 序列化/校验，未知字段错误，不另定义参数真值 |
| `drafts.replace` | draftId、expectedRevision、完整配置 | 新revision；并发修改冲突则拒绝，不默默覆盖人类草稿 |
| `drafts.get` | draftId | 当前配置、revision、源绑定；不返回正在运行的可变引用 |
| `drafts.validate` | draftId、expectedRevision、executionProfile（仅 `cpu-single` 或 `cpu-rayon`） | preflightId、blocking/warnings/unchecked、快照 hash 和能力 buildId；必要未检查项阻断启动，线程档缺少隔离、共享内存或线程池时明确失败 |
| `runs.start` | draftId、expectedRevision、preflightId、idempotencyKey | runId、accepted状态、配置hash；冻结快照，预检过期/资源变化则拒绝 |
| `runs.get` | runId | 生命周期、阶段、计时、最后seq、终止原因、实际后端；不捏造全流程百分比 |
| `runs.events` | runId、afterSeq、limit | 有界增量事件和nextSeq；落后于保留窗口返回EVENT_GAP，客户端重取snapshot，不静默漏日志 |
| `runs.cancel` | runId、mode=`cooperative`或`force` | 请求已接收/不支持；cancelled仅在任务真正终止后出现；force显式丢弃产物 |
| `runs.result` | runId | 仅完整运行成功时返回权威 CalibrationOutput + diagnostics引用，保留pending_human_review |
| `artifacts.list` | runId | 当前可用产物id、mediaType、长度、hash、complete；失败运行允许下载诊断但无成功标定结果 |
| `artifacts.read` | artifactId、offset、length | 有界二进制分块，通过transferable传送；无整包JSON/base64；校验范围与所有权 |
| `runs.release` | runId | 释放终态运行资源；运行中拒绝；删除持久化文件须单独授权，不混入release |

`File` 绑定适配由受信任宿主完成：人类文件选择器或自动化工具显式选择的文件。页面 API 不接受“/Users/.../bag”后自己读取。JS对象入口可传 File；纯JSON/MCP入口只能传已经授权的sourceId，或由本地宿主执行文件绑定。目录绑定显式保留relativePath，检测重名/路径穿越，不把路径拼到站点URL。

## 3. 状态、重试、精度

- 一次活动计算任务作为初始并发合同；重复启动返回BUSY，不偷偷排队。多任务调度必须未来显式扩展合同而非改变当前行为。
- 同一会话内相同 idempotencyKey + 同一冻结请求返回同一runId；不同请求内容复用key返回IDEMPOTENCY_CONFLICT。页面刷新后不声称跨会话exactly-once；新会话必须重新确认是否重跑。
- `preflightId` 绑定 draft revision、source revision、引擎版本、CPU 执行档及其 build id；任一变化则 STALE_PREFLIGHT。创建后改草稿不影响正在运行的快照。
- 所有纳秒时间、u64长度/序号跨JSON边界使用十进制字符串；不允许丢精度的Number往返。Rust f64参数保持有限值；不可用诊断使用null+reason，不把NaN伪造为0。
- 事件包含 `{runId, seq, stage, kind, elapsedMs, payload}`；afterSeq为排他游标，序号单调，客户端按seq去重。高频进度可合并，但告知策略；错误、状态变迁与最终事件不可悄悄丢弃。
- `inspections.get` 的 `progress` 可包含 `{bytesRead, readCalls}`，表示成功范围读取的累计字节/次数，没有可靠总工作量时不提供解析百分比。
- `detectionProgress.payload.preview` 为浏览器专用实时采样图像 `{width,height,pixels,corners}`：真实灰度缩略图最大 320×240，每相机首帧及按墙钟约 250 ms 间隔采样；角点映射到缩略图坐标。Rust 回调通过独立 `Uint8Array` 传递像素，Worker 通过 transferable ArrayBuffer 发送，不把像素编码进 JSON，不转移 WASM 共享内存。像素不写入 `detection_progress.jsonl`，也不保留在 `runs.events` 回放窗口；只有当前实时订阅可收到。
- Worker 数值事件以最多 32 条或 callback 检查到距离上次发送 50 ms 时合批；同步 WASM 期间不依赖 JS timer。阶段切换、正常完成、错误和协作取消前发送尾批，控制器恢复逐条公开事件及单调序号。无 callback 时不会凭空刷新；force 终止可能丢弃尚未发送的尾批，不承诺保留强制终止前所有遥测。
- 当前引擎仅缓存一个已授权源的容器 reader，绑定 `sourceId/sourceRevision` 与文件 handle/path/size/role；重新绑定、源切换和错误后失效。字节读取对小请求使用 256 KiB 窗口、每文件最多 1 MiB，64 KiB 及以上直读；SQLite 窗口起点按 4 KiB 对齐。`browser-performance.json` 记录每次运行的逻辑请求、实际 FileReaderSync 返回字节、命中和耗时，以及结束时 WASM 线性内存字节数；它们不是物理磁盘 I/O 或进程 RSS。
- 引擎源 descriptor 的 `previewEnabled` 是诊断选项，缺省 true，不参与 reader 缓存身份。false 从源头跳过预览生成，仍处理全部检测帧；产品 UI 保持预览开启。`input.yaml.performance` 提供经过时间归因，core 子阶段包含于 detection 总时间，不能重复相加。
- 可选 `latest_imu` 以及独立 `imuProgress` 载荷包含 `{sample_count,timestamp_ns,gyro_rad_s,specific_force_m_s2}`，来自实际选中的 IMU 读取；时间戳为十进制字符串。实时 IMU 事件以 100 ms 节流，并保留最终变化计数；未选择 IMU 时不伪造样本。
- 监听断开不取消计算，连接恢复用runId/get/events接续；整个标签已关闭则不能靠原runId恢复一个不存在的内存任务。
- 取消合同沿用 [技术方案 §4](wasm-acceleration-plan.md#4-取消与故障语义)。`postMessage` 不能打断长同步求解；没有协作能力时返回 `CANCEL_MODE_UNSUPPORTED`，Agent 必须显式选择 force，不能偷偷升级。

稳定错误码至少包括：`INVALID_REQUEST`、`UNSUPPORTED_PROTOCOL`、`ENGINE_NOT_READY`、`SOURCE_NOT_BOUND`、`UNSUPPORTED_FORMAT`、`STALE_REVISION`、`STALE_PREFLIGHT`、`PREFLIGHT_BLOCKED`、`CAPABILITY_UNAVAILABLE`、`BUSY`、`IDEMPOTENCY_CONFLICT`、`EVENT_GAP`、`CANCEL_MODE_UNSUPPORTED`、`RESOURCE_EXHAUSTED`、`RESULT_NOT_READY`、`RUN_FAILED`、`NOT_FOUND`。details 包含字段/阶段定位，绝不夹带 registry 凭据。

## 4. Agent 调用流程

1. 打开 Pages，在 API-ready 后读取 capabilities，确认计算机器、所选 CPU 执行档、build id 和实际线程数；初始化失败就停止，不能静默换档。
2. 用户或受信任自动化显式授权文件，调用sources.bind/inspect，读取真实扫描结果。
3. 创建配置，使用共享核心validate；修复blocking，不擅自换模型、时戳来源或放宽阈值。
4. 用校验所得preflightId和幂等键start；保存runId。
5. 订阅事件或有界轮询，遇到EVENT_GAP获取snapshot；只有完整完成后才请求result与artifacts。
6. 输出残差、参数、停止与诊断；不把pending_human_review改成已验收。

人类点击与Agent调用必须走同一校验、权限、快照与算力预算。已有用户授权可以明确涵盖一次完整本地标定，不需要每个只读RPC都弹确认；文件访问、数据外传、强制丢弃、持久数据删除和人工验收权限必须单独界定。

## 5. 实施与验证门槛

先在可移植任务控制器实现合同，再接UI和SDK，最后添加可选MCP适配。浏览器API不是另写CLI参数拼接器。正式SDK/JSON Schema从权威类型生成；当前文档中的方法表是设计合同，不能宣称已经生成或注册。

接口验收覆盖：相同配置 UI/API 运行一致；只接受 `cpu-single` 与 `cpu-rayon`；线程档能力不足明确失败且不静默降为单线程；人类并发编辑 revision 冲突；重复 start 幂等；取消竞态；事件窗口丢失；64 位时间精确往返；无文件授权不能读本地路径；长任务 RPC 无假完成；跨标签不泄露数据；浏览器重启明确会话丢失。每个可用标定模式/格式仍需完整端到端验证，并遵守固定 `baton_bag`、corelib/OpenCV 数值对照要求。接口可调用不等于数值已验证。
