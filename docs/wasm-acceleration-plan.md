# WASM CPU 实施与验证记录

## 1. 范围与边界

浏览器版只使用 CPU，执行档固定为两种：

| 执行档 | WASM 构建 | 并行方式 | 必要条件 |
|---|---|---|---|
| `cpu-single` | 非原子单线程构建 | 一个计算 Worker | WebAssembly、Worker |
| `cpu-rayon` | 原子共享内存构建 | Rayon 在线程 Worker 间共享一份 WASM memory | `crossOriginIsolated`、`SharedArrayBuffer`、WASM atomics、线程池初始化成功 |

容器解析、图像解码、AprilTag 检测与 `f64` 标定求解全部在 CPU 上执行。两档调用同一 Rust runner 和固定版本 corelib，不在 JavaScript 中复制检测、几何或求解算法，也不降低数值精度。用户明确选择执行档；所选档不可用时预检失败，不静默切换到另一档。

当前应用固定依赖 `ccl_calib =0.6.0`、`ccl_rosbag =0.5.0`、`ccl_geometry =0.1.0`、`ccl_image =0.3.0`。前两个版本的 WASM 可移植性改动仅通过隔离本地 registry 验证，尚未发布到远端；本轮 `ccl_image` workspace API 另通过清单 patch 使用相邻源码，尚未发布。构建前提与最新性能报告见 [输入与检测器复用施工记录](browser-performance-implementation.md)。

## 2. 运行架构

- UI 主线程只维护表单、预览、状态和下载，不执行标定，也不保留第二份完整数据集。
- 计算 Worker 通过随机范围读取接口访问用户授权的文件；图像 payload 有界读取，不把整包图像载入内存。
- 原生和浏览器入口共用 `RunSettings`、runner、进度事件和结果 schema。浏览器产物先写入内存或 OPFS 临时区，完整成功后才形成下载归档。
- 每条异步消息携带运行 id、阶段和单调序号；旧运行的迟到消息不能覆盖当前状态。
- 求解与几何继续使用 `f64`。数值收敛只表示当前约束下的驻点，结果仍为 `acceptance: pending_human_review`。

`cpu-rayon` 使用 `wasm-bindgen-rayon`。线程构建固定 nightly 日期、`rust-src`、`wasm-bindgen`/CLI 版本、atomics/bulk-memory、shared-memory linker 参数与 `-Z build-std`。发布物包含互不混用的单线程和线程 WASM；manifest 记录 build id、长度和 SHA-256，加载器只下载用户所选档对应的构建。线程数受 `hardwareConcurrency`、内存预算和配置上限约束。

检测输入保持原图尺寸、像素和精修配置不变。每 worker 复用 decoded/gray 缓冲区，L8 直接借用 decoded 数据；压缩输入仍做必要 ownership copy，但复用按 worker/有界队列规模限制的缓存。取消/错误先关闭并收尾已入队任务，不遗留后台检测；缓存限制对象数，不承诺固定字节峰值（取决于最大输入图像）。灰度转换与锁定的 `image 0.25.10` 逐字节对照，升级 image 时必须重新验证，包括目标平台的 FMA 行为。

浏览器预览每相机独立按 250 ms 间隔申请，首帧可预览，不以帧序号取模。该限流只减少缩略图生成与传输，不跳过检测帧。Worker 二进制像素与事件批发合同见 [浏览器 API](browser-agent-api.md#3-状态重试精度)。

## 3. 初始化与静态部署

初始化在用户选择文件前完成：

1. 加载并校验同一 release 的 manifest。
2. 确认所选执行档的隔离要求；线程档需要时先安装限定作用域的 COI Service Worker，并至多受控刷新一次。
3. 按 manifest 下载 JS、WASM 和 Worker；只在总长度可信时显示确定百分比。
4. 编译并实例化 WASM。浏览器没有可移植的编译百分比，因此只显示阶段和耗时。
5. 启动计算 Worker；线程档等待 `initThreadPool` 完成并核对实际线程数。
6. 全部资源 hash、build id、执行档和 Worker 状态一致后才进入 ready，并开放文件选择。

GitHub Pages 只承载静态文件，不能提供外部 HTTP、SSE、WebSocket 或 MCP 服务。项目子路径下的 HTML、Worker、WASM、Service Worker 和动态 import URL 必须由相对 base 生成。公开发布内部库派生产物前仍须完成许可证与分发授权审查。

## 4. 取消与故障语义

- `cpu-single` 的长同步调用不能被 `postMessage` 中断；强制取消通过终止 Worker 实现，并丢弃该次候选结果。
- `cpu-rayon` 通过共享内存中的原子标志请求协作取消；解析、检测与 corelib 安全点在有界工作单元间轮询。
- UI 必须区分“正在等待安全点”“已协作取消”和“已强制终止”，不能把取消或部分产物显示为成功。
- 初始化失败、内存不足、输入错误或求解失败均保留结构化诊断，不自动重试为另一个执行档。用户可修改选择后创建独立新运行。

## 5. 已验证性能与数值结果

真实 JPEG 整包验证使用 `ext_sync_exp_1600.bag` 的 cam0、默认 Double Sphere 配置：

| 指标 | `cpu-single` | `cpu-rayon`（4 Worker） |
|---|---:|---:|
| 扫描/读取/检测墙钟 | 148.572 s | 44.212 s |
| 求解/导出墙钟 | 6.357 s | 2.217 s |
| 总墙钟（不含初始化和话题扫描） | 154.929 s | 46.429 s |

单次端到端墙钟比约 3.34。两档是不同 WASM 构建，不能把全部差异归因于线程数，也不能外推为其他数据、设备或求解模式的加速倍数。

两档均处理 2265 个输入帧；按时间戳排序后的逐帧点数一致：1538 个有效帧、153376 个检测点、153368 个求解点、8 个拒绝点。4 Worker 结果 RMS 为 0.2950456545732174 px、P95 为 0.49660939540929877 px，均在 9 次迭代后停止；相对单线程结果的最大内参绝对差约 $9.44\times10^{-9}$（fx/fy），不是逐位相同。该记录是单相机验证，不覆盖 rig 或 IMU 外参。

CPU-only清理后重新构建两套引擎，并重新运行同一真实bag的4线程流程：2265帧，总墙钟47.985 s，其中读取/检测45.766 s、求解/导出2.219 s。最终标定参数与清理前4线程结果逐值一致，RMS/P95及有效/拒绝点数不变，浏览器无错误。页面只提供两档CPU选项，实时展示实际工作线程数；两套WASM的导入/导出已检查，旧计算接口及旧生成引擎目录均已移除。9项原生输入测试通过，CPU双档静态打包通过。

### 利用率与应用层开销检查（2026-11-13）

同一 Chromium、本地页面、`ext_sync_exp_1600.bag` cam0/Double Sphere/2265 帧、配置不变；浏览器报告并行度 8，宿主为 10 逻辑核。默认从固定 4 改为 `min(hardwareConcurrency, 8)`，缺失时 4，保留手动 1–32；自动上限不是任意大图的内存保证。

| 构建与线程 | 检测平均 FPS | 扫描/读取/检测墙钟 | 解码/灰度累计 worker 时间 | 检测累计 worker 时间 |
|---|---:|---:|---:|---:|
| a172649a…，4 线程 | 47.20 | 50.18 s | 21.99 s | 169.69 s |
| a172649a…，8 线程 | 70.14 | 34.53 s | 31.06 s | 226.66 s |
| f094f009…，8 线程，最终应用层优化 | 61.82 | 38.69 s | 42.86 s | 249.77 s |

顺序单次运行，不是稳定加速比；小优化的净吞吐收益未被隔离证明。进程 CPU 累计时间粗略折算，4/8 线程约占 4/6.6 个核心，包含页面与求解，不等于全机 CPU 百分比或 worker 空闲率。阶段时间与 worker 累计时间不可相加。

已消除额外工作：Rust 事件在已拥有的 JSON Value 上处理整数精度，不复制整棵 Value；预览 payload 移入事件，不重复构造 Value；角点结果按标签数预分配容量；页面按 canvas/尺寸复用 ImageData，不逐预览重设画布尺寸。最终真实运行 125 次预览只分配一次 ImageData。保留压缩输入 ownership copy 和 WASM→独立 JS 像素 copy：前者跨越 bag 回调借用生命周期，后者避免转移共享 WASM 内存。

库内确认仍有成本：`ccl_image 0.3.0` 的 `tight_pixels` 对紧密 Gray8 仍逐像素复制；每帧 `Matcher::new` 为同一字典重算旋转并分配表；分割阶段分配全尺寸缓冲。当前 API 不提供借用紧密图像或复用 detector workspace 的入口，本次未复制核心算法到应用层，也未修改 registry 缓存。这些是代码确认的候选，不是已测量的主要热点或已实现优化。

最终 8 线程与优化前 8 线程参数逐值一致，4/8 线程内参最大绝对差约 4.72e-9；逐帧 Tag/点数一致，1538 有效帧、153368 求解点、8 拒绝点；最终 RMS 0.29504565457322224 px、P95 0.49660939540949167 px。当前为单相机，无 rig/IMU 外参。13 项原生输入测试、11 项 Node 配置/传输测试通过；两档 release WASM 构建完成（线程构建仍有 init_thread_pool re-export 的 unused import warning）。单线程 SQLite 合成数据 24 帧运行完成；8 线程真实数据协作取消后未发布未完成结果。未更改检测像素或求解算法，仍不替代缺失的固定 baton_bag 双后端数值基线。

## 6. 数值与基线限制

- 固定 `baton_bag` 路径仍缺失，不能宣称完成正式基线验收；`ext_sync_exp_1600.bag` 不能替代该固定数据。
- OpenCV 4.12 参考曾使用同一批 corelib 角点尝试 KB4 求解：全量输入触发矩阵范围断言；剔除少于 8 点的帧后仍在 `fisheye::InitExtrinsics` 触发断言。因此没有可报告的 OpenCV RMS/P95 或内参，且 Double Sphere 与 KB4 参数并非同构。
- 求解器对照必须使用完全相同的 observations（帧、角点、像素坐标、时间戳及顺序），避免把检测差异误判为求解差异。
- 完整验收仍须分别覆盖 ROS1 bag、MCAP、SQLite3，单/多相机、预积分和连续时间模式，并记录输入/config hash、crate 版本、浏览器、OS、CPU、执行档、线程数和产物 hash。
- 性能报告必须区分冷启动、WASM 初始化、扫描、解码、检测、求解和导出，记录峰值 WASM memory 与取消延迟；一次运行不构成统计显著结论。

## 7. 浏览器功能边界

页面通过同一任务控制器提供 UI 与 `window.fourP` API。文件只来自用户明确授权的 `File`/目录句柄；页面不能凭本地绝对路径读取文件，也不默认上传数据。ROS1 bag、MCAP 和 SQLite3 的浏览器闭环、结果下载、幂等启动、单线程强制取消与 Rayon 协作取消已有验证记录，但 API 可调用不等于所有模式均通过正式数值验收。

交互规格见 [网页工作台设计](web-workbench-design.md)，页面内 API 合同见 [Agent 浏览器 API 契约](browser-agent-api.md)。
