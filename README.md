# 4P_Calibration

4P 离线标定应用：单相机内参、多相机联合内外参，以及预积分或连续时间 B 样条多相机—IMU 联合标定。

仓库只保留 ROS bag 输入适配、结果导出与薄 Ratatui 壳。AprilTag 检测、相机模型、投影、位姿、内外参、相机—IMU 和可观测性全部调用 `camera_corelib`；本仓库不维护第二套核心算法。

`camera_corelib` 的各 crate 作为独立 `.crate` 发布在内部 GitLab `corp` Cargo registry。4P 固定依赖 `ccl_image`、`ccl_calib`、`ccl_geometry` 和 `ccl_rosbag` 的精确版本，由 Cargo 在统一依赖图中编译；不再分发或注入预编译 `.rlib`。

当前应用清单固定 `ccl_calib =0.6.0`、`ccl_rosbag =0.5.0`、`ccl_geometry =0.1.0`、`ccl_image =0.3.0`。前三者从 `corp` registry 解析；本轮未发布的 `ccl_image` workspace API 通过清单内 patch 使用相邻源码。`ccl_calib 0.6.0` / `ccl_rosbag 0.5.0` 的 WASM 改动此前只用隔离本地 registry 验证，普通环境仍需要对应包；当前不是仅拉取已发布依赖即可构建的状态。集成与验证身份见下方性能报告。

## 浏览器版方案与交互设计

- [WASM CPU、Rayon 多线程与 GitHub Pages 方案](docs/wasm-acceleration-plan.md)
- [网页形态、交互与初始化加载设计](docs/web-workbench-design.md)
- [Agent 浏览器 API 契约与静态部署边界](docs/browser-agent-api.md)
- [浏览器安全、数据处理与本地发布边界](docs/browser-security.md)
- [可交互设计原型](web/design/index.html)：浅色工作台、文件元信息、板布局预览、真实环境检测和加载状态样式；**未接入标定引擎或 Agent API，不产生标定结果**。

设计原型：`python3 -m http.server 8765 --bind 127.0.0.1`，访问 `http://127.0.0.1:8765/web/design/`。

实际浏览器应用位于 `web/app/`，与原型分离。依赖可解析且已安装 `wasm32-unknown-unknown` target 后，安装与 `Cargo.lock` 对应的 `wasm-bindgen-cli`，执行：

```sh
python3 tools/build_web.py --debug
python3 tools/serve_web.py
# http://127.0.0.1:8766/
```

页面使用同一 Rust runner 和 corelib，在 Worker 中按范围读取用户授权的文件；配置、扫描、真实运行事件、结果及产物下载与 `window.fourP` 共用任务控制器。浏览器只提供 `cpu-single` 与 `cpu-rayon` 两个执行档：单线程档通过销毁 Worker 强制取消；Rayon 共享内存档使用原子标志协作取消。线程构建命令为 `python3 tools/build_web.py --threads --debug`，需要指定的 nightly/rust-src 与跨源隔离。开发服务器直接提供隔离响应头；无自定义响应头的静态子路径另通过 Service Worker 建立隔离。

生产页采用七个连续区块：引擎加载 → 统一文件导入 → 读取解析 → 话题选择 → Tab 配置 → 计算监控 → 标定结果。阶段完成后平滑滚动并高亮下一块，不做整屏翻页。下载显示真实累计资源字节；随机读取解析没有可信总工作量时只显示阶段动画与累计读取量。监控显示实际速度、线程数、最近逐帧记录和采样图像/IMU 数据，结果用紧凑表格与适用的内嵌外参视图展示；无对应数据时明确标记缺失。

首次进入只显示说明，点击“加载本地计算引擎”后才下载并初始化。页面提供项目/反馈入口，明确文件授权、本地计算及托管资源请求边界；就绪后显示版本、执行档和完整构建哈希。HTML CSP 限制脚本与网络来源，不允许普通字符串 eval。本轮安全加固只在本地修改，待 GPU 开发完成后再决定统一发布，不代表 Google Safe Browsing 标记已经解除。

网页默认选择多核 CPU，工作线程数为 `min(hardwareConcurrency, 8)`，浏览器未报告时使用 4；仍可手动设置 1–32。自动上限用于限制每线程图像缓冲内存，不是所有设备/数据的最优值，大图或内存受限时应减少线程。修改后自动应用，输入停顿 350 ms 后生效；初始化、运行和解析期间锁定算力设置。缺少共享内存或跨源隔离时明确失败，不静默降档。

首次解析默认启用所有可用图像话题，按话题路径自然排序（camera2 在 camera10 前）分配 cam0、cam1…；自动按相机数量与 IMU 选择推断标定类型。求解器支持一路 IMU，多路可用时默认选择排序后的第一路并提示可切换；重新解析保留手动选择、ID 与模型。

网页收敛图默认显示本次运行的全部代价报告：线性化活动不清空历史，相同迭代号的更新仍追加；阶段变化或迭代号回退只新增分段，不自动切换查看范围。横轴使用累计报告序号，悬停可查看原始迭代号、阶段与 cost；分段之间不连线，不同目标函数的 cost 不直接比较。可以手动筛选单段再返回全部历史，结束后保留曲线，仅开始新任务时重置。已用真实求解记录回放验证两段共 31 个报告点全部保留。

公开静态站点：<https://hessian-matrix.github.io/4P_Calibration/>，由独立 `gh-pages` 分支根目录发布；源码分支的修改不会自动上线。发布内容包含页面及两套 manifest 引用的 CPU 引擎，不能包含数据集、凭据或运行结果。上传必须保留文件原始字节（Git API 使用 base64 blob，不以文本读取后重编码），否则 CRLF 转 LF 会破坏 manifest 长度和 SHA-256。已在线验证可信 HTTPS、Service Worker 跨源隔离与实际 4 线程 WASM 初始化；此次检查不代表重新完成数值标定验收。

网页解析区分别显示所选文件总大小与累计读取量。累计读取统计每次文件读取，包含重复区间，因此可能超过文件大小；它不是文件体积或完成百分比。重新解析会重置读取显示，完成前保持不定进度；真实 `ext_sync_exp_1600.bag` 的文件大小显示 1.49 GB，解析累计读取 2.99 GB。

网页检测柱带先将旧柱左移，再升起新柱；位移使用 `transform`，队列与显示均有上限，不向计算 Worker 发送动画完成确认。检测表格、指标、日志按 100 ms 合批更新，预览按批次保留每相机最新一帧；完整数值事件与产物不受显示窗口裁剪影响。柱带队列仅保留数字元数据，不持有预览像素。减少动态效果时直接显示最新柱带。

预览像素通过独立 Uint8Array 与 transferable ArrayBuffer 传输，不进入 JSON；后端遥测在 Worker 中按最多 32 条或 callback 检查到 50 ms 合批，阶段切换和终态前先发送尾批。公开订阅仍逐条接收事件；强制销毁 Worker 可能丢失未发送尾批，不保证强制终止前遥测完整。

CPU 利用率验证：同一真实 bag 的 cam0、2265 帧，同一中间构建下 4/8 线程检测平均为 47.20/70.14 FPS；移除额外 JSON Value 复制、预分配角点结果及复用预览 ImageData 后，最终 8 线程构建为 61.82 FPS。均为顺序单次测量，不能将差值归因于某个小优化。最终运行 125 次预览只分配一次 ImageData，不重复调整相同尺寸的 canvas。逐帧 Tag/点数一致，同线程数最终参数逐值一致；4/8 线程最大内参差约 4.72e-9。详细边界见 [CPU 验证记录](docs/wasm-acceleration-plan.md)。

浏览器输入与检测器复用改造见 [施工与实测报告](docs/browser-performance-implementation.md)：交错对照中读取/检测均值减少约 11.2%，完整角点与最终结果一致；固定 `baton_bag` 缺失，正式标定验收仍未完成。本地开发构建通过 `Cargo.toml` 的 `ccl_image` patch 使用相邻 `../camera_corelib/crates/ccl_image` 源码；这是未发布的联合开发状态，不是 registry 中同版本已获得新 API。构建需要这两个工作树，不修改 registry 缓存。`input.yaml.performance` 记录读取准备、提交、worker 等待、结果回收延迟、预览和 core 检测子阶段；各 worker 经过时间相互重叠，不是 CPU 时间。

计算区显示当前阶段与阶段耗时；检测阶段按已解析的所选相机消息总数、逐相机完成数及后端累计平均 FPS 估算剩余时间，不包含后续求解。无有效速率时显示估算中，进度超过 5 秒未更新时暂停估算；求解与导出没有可靠剩余量时不提供虚构 ETA。停止或完成后冻结耗时。

内嵌外参视图遵循 CLI 的参考系 +Z 向上、XY 水平约定：有 IMU 时使用 IMU 参考系，仅旋转显示空间，不修改外参。右上角方块随视角旋转，仅通过点击方块面来对齐 YZ/XZ/XY 平面，不再提供对齐视角下拉框；保留当前观察中心和距离，以 480 ms 动画切换。拖动或重置可中断，减少动态效果时直接切换；静止时不运行持续渲染循环。

GitHub Pages 交互性能验证：同一 Chromium、`ext_sync_exp_1600.bag` 的 cam0 共 2265 帧、同一 `c446f902…` WASM、4 线程与相同配置，合批前后检测平均为 45.1 / 52.8 FPS，主线程布局累计约 2.12 / 0.51 s，结果 JSON 一致；均为顺序单次测量，不能视为稳定加速比。引擎就绪后断网仍完成标定且运行期没有新增资源请求，证明检测无需 Pages 网络；前端与 Worker 仍共享设备资源，不能承诺零开销。这是界面和性能验证，不替代固定 `baton_bag` 数值验收。

已实际验证 Chromium 的单线程与 4 线程 Rayon 构建均完成 24 帧/1532 点合成数据标定；corelib RMS 约 0.392643 px、P95 约 0.663777 px，与原生结果一致；OpenCV 相同原生角点的零畸变针孔参考 RMS 0.392642 px、P95 0.664173 px。ROS1 bag、MCAP、SQLite3 均已完成单相机浏览器闭环；API 幂等启动、强制取消和共享原子标志协作取消已验证。原生 `cargo test --locked --lib --tests` 通过 70 项测试。

纯静态子路径 `/app/` 已验证 Service Worker 建立跨源隔离后才开放文件选择。`python3 tools/package_web.py DESTINATION` 校验两套 CPU 引擎 manifest、长度和 SHA-256，生成本地静态目录；不会上传或发布。固定 `baton_bag` 路径仍缺失，不能宣称完成正式数值验收；真实数据 `ext_sync_exp_1600.bag` 只用于交互与性能验证，不能替代固定基线。

真实 JPEG 整包 CPU 多线程已验证：上述 cam0 配置使用 4 个 Rayon Worker，总墙钟 46.429 s，对照单线程 154.929 s，单次约 3.34 倍；逐帧点数一致，内参最大绝对差小于 $9.44\times10^{-9}$。两档是不同 WASM 构建，结果不作为纯线程数加速基准，也不能外推到其他数据、设备或求解模式。当前推荐 `cpu-rayon`，不满足共享内存与跨源隔离条件时可显式选择 `cpu-single`。

## 构建与运行

配置 `corp` registry 凭据后使用标准 Cargo 入口：

```sh
cargo run --release -- --config config/calibration.yaml
cargo test --locked
```

正式发布仅由内部 GitLab CI 执行。CI 在 Linux runner 上构建 Linux x86-64，并交叉构建 Linux ARM64、Windows x86-64 GNU LLVM 与 macOS ARM64；仅原生 Linux x86-64 产物执行 `--help` 启动检查。四项目标必须全部构建成功，随后 release job 才会先创建 GitLab 内部 Release，再在 GitHub 文档/发布仓库创建对应 Release；两边均上传同一组四个可执行文件和 `SHA256SUMS`。任一构建或任一发布端失败时，不会继续后续发布。

流程：数据源扫描 → 传感器与任务模式 → 标定板 → 求解设置 → 启动检查 → 运行监控 → 结果检查。勾选或取消相机/IMU 话题时自动推断模式：未选 IMU 时，1 路相机为单 Cam，2 路及以上为多 Cam；选中 IMU 时为多 Cam+IMU。零相机或相机–IMU 模式不足两路相机时仍由预检阻止启动。可手动调整模式，下一次话题选择变更会重新推断。配置各部分可返回修改，运行使用不可变快照；结果页可修改配置或使用相同配置创建独立的新运行。

TUI 交互：

- 配置按“参数 / 值 / 说明”表格组织；`↑/↓` 选择行，`←/→` 选择单元格，方向键不修改值；`Tab/Shift-Tab` 切换行。`Enter` 编辑/下拉，`Space` 勾选或展开高级组。`Ctrl-P` 上一分区、`Ctrl-N` 下一分区，运行/结果页也可用；输入框和 YAML 编辑器不触发切页。`Ctrl-←/→` 仍支持，但 macOS 桌面快捷键可能先截获；也可点击顶部导航或底部按钮。窄屏说明移入当前行详情。
- 路径、数值、板 ID 行和角点映射均通过输入框编辑；长文本支持换行和滚动。标定板 ID 行按原始顺序保留，不因导航或修改其他字段自动重新编号。
- Sensors 每行一个相机：启用、ID、Topic、模型、消息数、Hz、参考相机；Hz 为按 bag 记录时间估算的平均消息频率，样本不足显示 `—`，不等同于传感器采样频率。IMU 使用独立表格。选中行显示完整 Topic 与时间信息。扫描不抢页面或焦点。内参策略与参考相机集中在求解设置，Sensors 也可直接选择参考相机。
- 修改 bag 路径或容器格式会清除旧扫描、话题缓存、相机/IMU 选择和参考相机，保留板几何及求解设置；旧异步扫描结果不能回写。YAML 更换数据源时仅保留新 YAML 明确配置的绑定。同一数据源重新扫描刷新话题列表，不累积历史话题；已完成运行的快照、结果和图表不受配置草稿修改影响。
- 内参策略为“自动初始化并优化”“文件初值并优化”；纯视觉多相机模式还支持“文件内参固定”。文件 schema、相机 ID 和模型在预检检查；图像尺寸在解码首帧后检查，预检明确标记为尚未检查。
- 启动检查区分阻断、警告和未检查项；只有无阻断项时可启动。一次点击即开始运行，不再二次确认启动。
- 相机-IMU 模式可在配置页直接选择“预积分”或“连续时间 B 样条”。连续时间模式由视觉 rig、受 IMU 支持的最近陀螺样本旋转对齐及最早受支持种子时刻的比力和位姿建立初值；参考相机 shared-view 位姿与逐相机精确曝光时刻的 PnP 位姿按同一 rig 坐标系合并，按选定间隔拟合为四次样条。平移使用带状基函数求解，旋转使用局部 SO(3) 修正，避免仅用参考相机跨观测空档插值及全局旋转对数跳变造成投影越域。不调用预积分求解；随后固定视觉阶段内参，运行原始 IMU 与图像共同约束的批量优化。拟合初值不是联合标定结果。初始化、输入域或线性分解等无法形成候选的错误仍显式失败；达到 LM 迭代/阻尼边界或信息矩阵降秩不自动否决，最后接受的候选照常导出并等待人工验收。
- `imu.axis_calibration.refine: true` 显式开启第二次连续时间精化，使用 Basalt 的 21 参数静态 IMU 模型：完整 gyro 3×3（9 参数）、正对角下三角 accelerometer（6 参数）及各 3 维常量 bias。约定为 `physical = A × raw − bias`；仅加速度计三角结构固定共同旋转规范，gyro 保留相对加速度计的旋转失准。矩阵使用加性参数化，非法候选拒绝；默认不精化。预积分不支持轴内参，配置非单位矩阵或开启精化会明确报错。
- `imu.continuous_time.visual_motion_anisotropy.enabled: true` 可在各向同性连续时间解之后增加一次固定权重精化。应用由上一接受解预测每个角点跨曝光区间的像面位移；corelib 按该方向使用 `parallel/perpendicular = 1 + coefficient × motion_px²` 的协方差形状，并保持二维协方差行列式不变，避免通过整体放宽视觉权重获得伪收益。权重在该次 LM 内固定，因此解析 Jacobian 与被优化目标一致。默认关闭；必须为每帧提供曝光时长。
- 时间偏移、IMU 噪声、轴参数和求解设置通过表单编辑；高级 YAML 编辑器位于启动检查页。`Ctrl-S` 应用并校验，非法内容保留且不覆盖配置，放弃未应用修改需要确认。
- 表单、导航和动作支持鼠标。运行和结果页 `Tab` / `Shift-Tab` 切换只读视图，方向键与翻页键滚动；`Esc` 关闭弹窗或返回，不直接取消或退出。
- 标定运行页持续显示总耗时、当前阶段/活动耗时、最近回调距今时间及最近已完成迭代耗时；扫描检测阶段按总帧数和平均速度估算剩余时间。非线性求解剩余时间显示“未知”，不把迭代上限当完成目标；结束后耗时冻结
- `Ctrl+Y` 在“鼠标操作”和“文本复制”间切换；复制模式关闭 mouse capture，可直接拖选终端文本并使用终端自身的复制快捷键，`Ctrl+Y` 恢复鼠标操作
- 运行页实时显示流程、每个相机独立的 AprilGrid 角点折线、动态/平均 `frames/s`、worker 数、同步/检测统计，以及 PnP、陀螺对齐、运动方程组装/分解、LM 线性化/求解/候选评价、信息矩阵与残差评价活动。只有已知工作量才显示计数；初始化或后处理时 LM 标为“上次快照”，不冒充仍在该轮迭代
- 角点曲线显示每帧角点数（不是角点/秒），保留本次运行的完整观测，不再裁掉早期曲线。横轴覆盖从起点到当前进度；纵轴固定为配置标定板的角点总数，检测值不做平滑。
- 运行默认显示仪表盘：阶段与活动进度、计时、逐相机统计、检测历史和当前求解阶段收敛曲线。宽屏两图并排，中等窗口上下排列，80×24 保留一张图，通过 `g` 或图表选择按钮切换；求解阶段不删除检测历史。不同求解阶段不连接 cost 曲线，不显示虚构的总流程百分比。
- 结束后保留冻结的仪表盘，`r` 或按钮进入结果页，结果页 `Esc` 返回仪表盘。结果参数和诊断按表格组织；原始 YAML 保留在文件页。`F6` 打开目录，`F7` 打开外参图，`F9` 修改配置，`F10` 相同配置重跑。
- `F8/Ctrl-C` 经确认取消，等待 worker 实际退出；`Ctrl-Q` 确认退出，`Esc` 不取消任务。

无 TUI 批处理先按上述方式构建，再使用同一份 `RunSettings`、输入 adapter 和 runner：

```sh
target/release/four-p-calibration --no-tui --config config/calibration.yaml
```

当前 `config/calibration.yaml` 对应 `calib_ext.bag` 的连续时间相机—IMU 工作基线：gyro/accel 轴矩阵固定为单位阵（`axis_calibration.refine: false`），时差仍在 ±0.1 s 内估计，轨迹、外参、常量 bias 和重力方向继续联合优化。此设置不表示已验证硬件轴误差为零；历史运行快照不会被修改。无参数启动 TUI 自动读取此文件；显式 `--config` 优先，指定文件加载失败时不会回退。

也可使用配置预填已构建的 TUI：

```sh
target/release/four-p-calibration --config config/calibration.yaml
```

已固化两套 Double Sphere 数据预设；这些复现实验使用显式内参初值，初值与运行配置分开保存，配置中的相对路径从仓库根目录解析：

4P 默认及两份预设均使用 `detector.refinement.method: saddle`（十字细化），最多 20 次迭代，不默认使用外轮廓。旧配置若显式写有 `outer_contour`，需改为 `saddle`；历史运行快照不自动迁移。

梯度检测的簇数预算：4P 默认及 `1-bag` 预设使用 `max_clusters: 16384`，baton 预设保留 `4096`。`1.bag` 的高分辨率图像会超过旧的 4096 上限；16384 已通过两种细化的完整 bag 检测。旧 YAML 若显式配置 4096，不会被新默认覆盖，需在详细参数中修改 `detector.budgets.max_clusters`；预算耗尽仍显式失败，不截断候选。

```sh
# 1.bag：四相机纯视觉，固定 DS 内参后求 rig 外参
target/release/four-p-calibration --no-tui --config config/presets/1-bag-ds.yaml

# baton_bag：双相机—IMU；将 method 改为 continuous_time 可启用 B 样条联合优化
target/release/four-p-calibration --no-tui --config config/presets/baton-bag-ds.yaml
```

`1.bag` 的平面板序列无法独立约束四相机 DS 内参联合精化，因此该预设显式设置 `refine_intrinsics: false`。求解器不再按秩、条件数或 LM 停止原因设置发布阈值；`solve.yaml` 原样记录诊断，`result/calibration.yaml` 标为 `acceptance: pending_human_review`。对应初值位于 `config/intrinsics/1-bag-ds.yaml`；`baton_bag` 初值位于 `config/intrinsics/baton-bag-ds.yaml`。

连续时间配置片段：

```yaml
calibration:
  type: multi_camera_imu
  method: continuous_time
  imu:
    continuous_time:
      knot_interval_s: 0.01
      camera_time_offsets:
        refine: true
        minimum_s: -0.005
        maximum_s: 0.005
      clock_skews:
        refine_global: false
        refine_cameras: false
        minimum_ppm: -200.0
        maximum_ppm: 200.0
    axis_calibration:
      refine: true
      gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
      accelerometer_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
```

连续时间严格按“视觉/陀螺直接初始化 → 固定时钟参数与轴矩阵的连续时间基线 → 按配置释放全局/逐相机时钟参数和/或 IMU 轴误差的第二次精化”执行；两次求解均固定视觉阶段内参。每路 `cameras[].timing.timestamp_reference` 显式声明图像 `header.stamp` 是 `exposure_midpoint`（默认）还是曝光开始 `trigger`。trigger 必须同时配置 `fixed_exposure_duration_s` 或 `exposure_metadata_topic`，读取后先转换为 `midpoint = trigger + duration/2`；midpoint 不平移。固定时长与 metadata topic 二选一。metadata topic 必须是 `std_msgs/String`，JSON 至少含与图像 header 相同的 `camera_timestamp_ns` 和正数 `exposure.duration_ns`；缺帧 metadata 显式失败。配置了任一时长来源时，连续时间求解后生成 `exposure_diagnostics.yaml`：按实际逐帧时长计算曝光首/中/末投影、曝光运动分桶、方向性残差和 Simpson 几何平均相对 midpoint 的偏差；该文件当前只诊断，不自动改变 LM 视觉残差或权重。未配置时长的 midpoint 数据仍可标定，但明确没有有限曝光诊断。共享硬件时间域默认将 global/per-camera skew 固定为 0，只估计残余固定 offset；仿射 skew 实现保留为显式诊断选项，仅适用于独立时钟且已有共同时间事件证明线性漂移的情况。启用时模型以观测区间中点 `t0` 为参考：`t_imu = midpoint + global_offset + global_skew*(midpoint-t0) + camera_offset[camera] + camera_skew[camera]*(midpoint-t0)`。视觉参考相机的逐相机 offset/skew 固定为 0；斜率配置和输出使用 ppm，core 内部使用 s/s。不能把 cost 的微小下降或曝光时长相关趋势单独解释成物理 clock skew。不添加每帧自由时差、同步先验或结果钳位。`schema_version: 7` 输出 `clock_reference_time_s`、`clock_skew_ppm`、`camera_time_offsets_s` 和 `camera_clock_skews_ppm`；schema 4–6 仍可读取。预积分不支持 affine clock 精化。轨迹按全局 affine 时间线导出，不应用逐相机残余项。

所有联合优化入口统一使用 corelib 的 Basalt LM；旧 `Scaled` 策略及运行时选择器已删除。后端对未缩放正规矩阵使用历史最大对角阻尼，增益比按实际可行步的 `-gᵀs − ½sᵀHs` 计算；时差触界只限制该坐标，其他参数继续优化。仅投影 KKT 梯度达标才记为 `Converged(Gradient)`；反复小接受步、阻尼或迭代上限分别记为 `Stopped(Stagnation/DampingLimit/IterationLimit)`，不冒充收敛。信息矩阵取最终状态的未加阻尼、未屏蔽边界原系统；不添加固定姿态或平滑先验来消除秩亏。

`Converged(Gradient)` 只表示当前约束下的数值驻点，不代表标定已验收；时差贴住搜索边界时不能据此认定真实时差已找到。接近梯度阈值时，重复运行可能得到几乎相同的参数却分别报告梯度收敛或阻尼停止，应同时检查停止原因、残差、参数稳定性与边界命中情况，不为统一标签放宽阈值。

连续时间信息矩阵的带状 Schur 消元会精确移除局部正规矩阵及 global 交叉块中均为全零的坐标，不添加阻尼或先验。非零耦合的奇异局部块仍不能据此消除；真正的全局秩亏仍按原规则报告。此处理仅影响信息诊断，不改变 LM 更新或标定残差。

视觉噪声现在由每个 `cameras[].pixel_sigma` 独立配置。连续时间 IMU 噪声使用校正物理坐标系内的 gyro/accelerometer `3×3` 白噪声 PSD；每个样本按自身有效支持间隔计算 `covariance_i = Q / dt_i`，不再使用全局中位采样间隔，也不将 PSD 解释成随 A 变换的 raw covariance。预积分仍只接受各向同性对角 PSD，并取其对角平方根作为既有密度输入。

`solve.yaml` 同时导出逐相机 visual whitened RMS、连续时间 gyro/accelerometer whitened RMS，以及每个全局参数在优化坐标中的局部曲率边缘标准差。RMS 的平方可作工作 variance-scale 诊断，但当前不会自动重加权或改变 acceptance；参数标准差也未做 robust sandwich/REML 校准。

`config/presets/baton-bag-ds-continuous-basalt.yaml` 显式采用 0.01 s 四次样条、Huber-only 像点策略和参考 PSD；这些数值不代表设备噪声实测。`solver.point_rejection_px: null` 禁止硬剔除，正数则启用阈值剔除；二者均保留 Huber 权重。KB4 固定零 skew，仅优化 `fx/fy/cx/cy/k1..k4`，与 OpenCV fisheye `CALIB_FIX_SKEW` 的参数合同一致；含 `alpha` 字段的旧 KB4 文件会被拒绝。旧值为零时可显式移除该字段，非零值必须重新标定，不能静默丢弃。

**迁移：**旧 `gyro_matrix` / `accelerometer_matrix` 配置键及旧 `raw_from_physical` 结果键不再接受。含 IMU 的 schema 1–3 结果禁止作为新版结果加载；schema 4–5 的 IMU 合同相同。纯相机旧 schema 仍可读取，但 KB4 参数必须符合无 `alpha` 的新模型合同。一般非单位旧矩阵不能只重命名：同一轴系下先有 `A=M⁻¹`、`b_cal=A b_raw`，还须将加速度计改为下三角规范，并一致旋转 gyro、bias、外参及轨迹。建议重新标定；历史 run 保持原样。

## 输入

支持：

- ROS1 bag v2
- ROS2 MCAP
- ROS2 SQLite3（`.db3` 或含 `metadata.yaml` 的目录）
- `sensor_msgs/Image`：`mono8`、`mono16`、`rgba8`
- `sensor_msgs/CompressedImage`：PNG/JPEG/PNM
- `sensor_msgs/Imu`

图像读取使用有界队列交给按逻辑 CPU 数启动的检测 worker；队列只保留少量待处理图像，不会把整包图像载入内存。若配置逐帧曝光 metadata，输入层先单独扫描小型字符串 topic，再解码图像；不会缓存图像 payload。所有下游分组和连续时间残差统一使用归一化后的曝光中点。不同时间不合并、不因其他相机缺帧丢弃。视觉 rig、预积分及连续时间初始化另用参考相机时轴上的近同步 shared views；配对按曝光中点选唯一最近帧，再取其检测结果，检测失败不改配邻帧。`require_all_cameras` 要求组内各路原始曝光存在，不要求各路同时看到标定板。连续时间在初始化前按完整时差范围固定裁剪 exact/seed 两组相机观测：`[first_imu − minimum_offset, last_imu − maximum_offset]`；固定时差时两端使用同一个 offset。外围 IMU 全保留，内部大 gap 整体拒绝，求解中不随时差试步改变因子集合。样条从首 IMU 样本起覆盖整段 IMU，末端按结间隔向上取整；浮点整数边界不人为多加一段。联合标定不设置 `max_views`、总角点数或参数数量的人为上限，实际规模仅受可用内存与计算时间约束。记录时间只用于 topic 统计，不冒充传感器采样时间。

连续时间自动内参和视觉初始化只使用裁剪后的观测；corelib 两个入口再次校验相机时间支持，越界返回 `ImuCoverage`。gyro 使用保留种子的相邻位姿对，重力使用最早保留时刻的位姿和比力。样条除参考相机 shared-view 位姿外，还使用逐相机精确曝光时刻的固定内参 PnP，经同一 rig 外参转入参考坐标系，避免参考相机失去标定板时跨长空档插值；完全相同时间优先使用联合 rig 位姿。PnP 失败显式返回，不静默跳过。

## 产物

启动应用先创建独占 `runs/<YYYYMMDD-HHMMSS>-unconfigured/`，开始标定后改名为对应模式目录。同秒创建时使用独立序号（例如 `<stamp>-2-multi_camera`）；再次运行创建同级新目录，旧配置、日志和结果保持不变。

- `run.yaml`：状态、版本、阶段耗时和失败分类
- `config.yaml`：实际执行的运行设置
- `input.yaml`：topic、时间范围、图像尺寸和样本统计
- `run.log`：检测与求解诊断
- `detection_progress.jsonl`：逐帧时间戳、相机、tag/角点数量、动态/平均 `frames/s` 与 worker 数，检测期间每 250 ms 刷盘
- `detection.yaml`：完整检测汇总与各相机有效帧、tag、角点数量
- `time_support.yaml`：连续时间固定支持区间、仿射时钟参考时刻/搜索边界、时间原点、完整 IMU 样本数及逐相机裁剪/保留帧数与点数；`detection.yaml` 仍记录裁剪前检测总数
- `solve.yaml`：候选求解的停止原因、是否数值收敛、最终阻尼/梯度、信息秩/参数数、条件数、协方差是否为伪逆、时钟 offset/skew 标准差、逐相机重投影误差与剔除统计
- `exposure_diagnostics.yaml`：连续时间且有曝光时长时，逐相机记录实际时长范围，并按预测曝光运动 `<0.25/0.25–0.5/0.5–1/1–2/≥2 px` 输出瞬时 midpoint 与 Simpson 几何平均投影各自的重投影 RMS/P95、两者均方误差差、沿/垂直运动方向 RMS、平行方向均值及几何平均相对 midpoint 的偏差；无时长时状态为 unavailable。该文件只做求解后诊断，不把几何平均投影写回残差或参数。
- `report.txt`：TUI 完成/失败摘要、完整错误链、诊断建议和产物清单
- `result/calibration.yaml`：形成有限候选时生成的标定参数，顶层 `acceptance` 默认为 `pending_human_review`；应用不代替人工验收
- `result/extrinsics_3d.html`：与 YAML 一起原子发布的离线结果查看器，右栏展示中心时刻 offset、全局/逐相机 clock skew 和逐相机残余 offset。
- `result/trajectory.json`：最终求解位姿，`T_frame_from_body`，平移单位 m、旋转矩阵行主序。预积分使用最终状态；连续时间按全局 affine clock 求值最终样条，不应用逐相机残余项，也不把控制节点当作轨迹。
- 页面中采样间的平移线性插值与旋转 SLERP **仅用于播放显示**，不是额外求解点或连续样条的精确求值。静态构型单位 mm，轨迹单位 m；XYZ 为红绿蓝，相机线框只示意朝向，不是真实视场。内嵌 `D.board = { frame, tags: [{ id, corners_m }] }`：`frame` 与 `trajectory.frame` 相同（单相机/纯视觉为 `board`，相机–IMU 为 `world`；两种 IMU 求解器都把目标点直接当作世界坐标，故世界系即静止标定板系，标定板在帧内为恒等位姿）；`corners_m` 为米制四角，顺序同核心库目标几何 `(0,0) → (s,0) → (s,s) → (0,s)`，z=0。页面只绘制实际板面轮廓与配置中的 tag ID，不伪造 tag 位图。
- `result/summary.json`：本次最终求解摘要，与页面及 `solve.yaml` 同源。轨迹/摘要/参数/HTML 共同发布；旧运行不会自动补出当时未保存的轨迹，需重新运行。

`bag_exp.bag` 的逐帧曝光 metadata 验证使用固定 global/per-camera skew（共享硬件时钟域）并保持原 midpoint 求解不变。曝光时长为 `7.69–16.60 ms`，四路诊断点数分别为 `64,480 / 64,608 / 47,668 / 58,904`；将每点瞬时投影替换为曝光区间 Simpson 几何平均后，四路全量 RMS 分别由 `0.435963 / 0.429982 / 0.445776 / 0.414402 px` 变为 `0.437623 / 0.430280 / 0.446384 / 0.415338 px`，均方误差各增加 `0.001450 / 0.000257 / 0.000542 / 0.000776 px²`。即当前几何平均模型没有改善真实数据拟合，因此只保留 metadata 时间归一化和求解后诊断，不把曝光积分加入优化残差。

## 验证

```sh
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

功能边界、显式初值、配置合同、结果字段和故障语义见 [离线标定应用实施方案](docs/implementation-plan.md)。
