# 4P_Calibration

4P 离线标定应用：单相机内参、多相机联合内外参，以及预积分或连续时间 B 样条多相机—IMU 联合标定。

仓库只保留 ROS bag 输入适配、结果导出与薄 Ratatui 壳。AprilTag 检测、相机模型、投影、位姿、内外参、相机—IMU 和可观测性全部调用 `camera_corelib`；本仓库不维护第二套核心算法。

`camera_corelib` 以完整 workspace 的预编译 `.rlib` bundle 交付；bundle 固定 Rust 1.98.1、目标平台、corelib commit 和 `--all-features` profile。4P 只链接 `ccl_image`、`ccl_calib`、`ccl_geometry` 和 `ccl_rosbag`，但发布压缩包保留全部 workspace crate。

## 构建与运行

先从对应 GitHub Release 下载当前平台的 `camera_corelib-<version>-<target>-rlibs.tar.gz`，再通过唯一构建入口校验并构建：

```sh
python3 tools/build.py --bundle /path/to/camera_corelib-0.6.0-<target>-rlibs.tar.gz --release
target/release/four-p-calibration
```

`tools/build.py` 固定接受 corelib `0.6.0`、提交 `d279273e2ae6bd7a21738d108837ded12fc6c64a`，并拒绝 rustc release/commit、host/target、feature profile 或必需 `.rlib` 不匹配的包；不回退到相邻源码。运行记录写入 bundle 版本、commit、target、rustc 和压缩包 SHA-256。直接 `cargo build/run/test` 不构成受支持入口。

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
    axis_calibration:
      refine: true
      gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
      accelerometer_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
```

连续时间严格按“视觉/陀螺直接初始化 → 固定时差与轴矩阵的连续时间基线 → 按配置释放时差和/或 IMU 轴误差的第二次精化”执行；两次连续时间求解均固定视觉阶段内参。`refine: true` 释放 15 个矩阵参数，`time_offset.type: estimate` 释放时差参数，二者均未开启时不运行第二次精化。结果 `schema_version: 5` 在顶层记录人工验收状态，`inertial.axis_calibration` 保存上述两块校正矩阵，bias 位于校正后的物理坐标系；TUI 结果页直接显示完整 YAML。

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

图像读取使用有界队列交给按逻辑 CPU 数启动的检测 worker；队列只保留少量待处理图像，不会把整包图像载入内存。连续时间残差保留每路图像自己的 `header.stamp`，不同时间不合并、不因其他相机缺帧丢弃。视觉 rig、预积分及连续时间初始化另用参考相机时轴上的近同步 shared views；配对先按原始曝光时间选唯一最近帧，再取其检测结果，检测失败不改配邻帧。`require_all_cameras` 要求组内各路原始曝光存在，不要求各路同时看到标定板。连续时间在初始化前按完整时差范围固定裁剪 exact/seed 两组相机观测：`[first_imu − minimum_offset, last_imu − maximum_offset]`；固定时差时两端使用同一个 offset。外围 IMU 全保留，内部大 gap 整体拒绝，求解中不随时差试步改变因子集合。样条从首 IMU 样本起覆盖整段 IMU，末端按结间隔向上取整；浮点整数边界不人为多加一段。联合标定不设置 `max_views`、总角点数或参数数量的人为上限，实际规模仅受可用内存与计算时间约束。记录时间只用于 topic 统计，不冒充传感器采样时间。

连续时间自动内参和视觉初始化只使用裁剪后的观测；corelib 两个入口再次校验相机时间支持，越界返回 `ImuCoverage`。gyro 使用保留种子的相邻位姿对，重力使用最早保留时刻的位姿和比力。样条除参考相机 shared-view 位姿外，还使用逐相机精确曝光时刻的固定内参 PnP，经同一 rig 外参转入参考坐标系，避免参考相机失去标定板时跨长空档插值；完全相同时间优先使用联合 rig 位姿。PnP 失败显式返回，不静默跳过。

## 产物

启动应用先创建独占 `runs/<YYYYMMDD-HHMMSS>-unconfigured/`，开始标定后改名为对应模式目录。同秒创建时使用独立序号（例如 `<stamp>-2-multi_camera`）；再次运行创建同级新目录，旧配置、日志和结果保持不变。

- `run.yaml`：状态、版本、阶段耗时和失败分类
- `config.yaml`：实际执行的运行设置
- `input.yaml`：topic、时间范围、图像尺寸和样本统计
- `run.log`：检测与求解诊断
- `detection_progress.jsonl`：逐帧时间戳、相机、tag/角点数量、动态/平均 `frames/s` 与 worker 数，检测期间每 250 ms 刷盘
- `detection.yaml`：完整检测汇总与各相机有效帧、tag、角点数量
- `time_support.yaml`：连续时间固定支持区间、时间原点、完整 IMU 样本数及逐相机裁剪/保留帧数与点数；`detection.yaml` 仍记录裁剪前检测总数
- `solve.yaml`：候选求解的停止原因、是否数值收敛、最终阻尼/梯度、信息秩/参数数、条件数、协方差是否为伪逆、逐相机重投影误差与剔除统计
- `report.txt`：TUI 完成/失败摘要、完整错误链、诊断建议和产物清单
- `result/calibration.yaml`：形成有限候选时生成的标定参数，顶层 `acceptance` 默认为 `pending_human_review`；应用不代替人工验收
- `result/extrinsics_3d.html`：与 YAML 一起原子发布的离线结果查看器，内嵌参数、指标和轨迹，无需联网。默认展示最终求解轨迹，支持时间拖动、1× 播放/暂停、静态构型切换、旋转/缩放/平移和视图适配。右栏展示位姿 R/t、wxyz 四元数、相机内参、逐相机 RMS/P95 与使用/剔除点数、收敛/信息矩阵诊断，以及 IMU 偏置、重力、时间偏移和轴矩阵。无 IMU 时明确显示不适用。轨迹模式叠加实际标定板：页面数据 `D.board` 由运行快照 `config.yaml` 经 `camera_corelib` 的 `TargetGeometry::april_grid` 生成，不维护第二套板几何约定。
- `result/trajectory.json`：最终求解位姿，`T_frame_from_body`，平移单位 m、旋转矩阵行主序。单相机/纯视觉轨迹为参考相机相对标定板的运动；相机–IMU 为 IMU 在求解世界系的运动，不做额外重力对齐。`time_origin_ns` 用十进制字符串保存，`time_s` 为相机曝光时间相对该原点的秒数。预积分使用最终状态；连续时间在 `t_imu = t_camera + time_offset_s` 处求值最终样条，不把控制节点当作轨迹。仅含参与求解的观测时刻，不是完整数据集的独立定位结果。
- 页面中采样间的平移线性插值与旋转 SLERP **仅用于播放显示**，不是额外求解点或连续样条的精确求值。静态构型单位 mm，轨迹单位 m；XYZ 为红绿蓝，相机线框只示意朝向，不是真实视场。内嵌 `D.board = { frame, tags: [{ id, corners_m }] }`：`frame` 与 `trajectory.frame` 相同（单相机/纯视觉为 `board`，相机–IMU 为 `world`；两种 IMU 求解器都把目标点直接当作世界坐标，故世界系即静止标定板系，标定板在帧内为恒等位姿）；`corners_m` 为米制四角，顺序同核心库目标几何 `(0,0) → (s,0) → (s,s) → (0,s)`，z=0。页面只绘制实际板面轮廓与配置中的 tag ID，不伪造 tag 位图。
- `result/summary.json`：本次最终求解摘要，与页面及 `solve.yaml` 同源。轨迹/摘要/参数/HTML 共同发布；旧运行不会自动补出当时未保存的轨迹，需重新运行。

## 验证

```sh
cargo test --all-targets
cargo clippy --all-targets -- -D warnings
```

功能边界、显式初值、配置合同、结果字段和故障语义见 [离线标定应用实施方案](docs/implementation-plan.md)。
