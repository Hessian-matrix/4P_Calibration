# 运行手册（operations）

默认使用系统开发库，不要求项目专用 `.deps`、兄弟仓库或手工 export。非标准安装仍可显式覆盖绑定库的发现路径。

## 1. 系统构建环境

### 1.1 依赖与发现

- Rust ≥1.92、C/C++ 编译工具链、Clang/libclang、pkg-config；OpenCV 也支持 CMake/vcpkg 探测。
- OpenCV 接受范围：4.8+（4.x）或 5.x，需有 core、imgproc、imgcodecs、objdetect 与对应标定模块。4.x 使用 calib3d；5.x 使用 calib。最低 4.8 是为保留 `DICT_ARUCO_MIP_36h12` 字典。
- FFmpeg 接受范围：6.x–9.x，需有 libavcodec、libavformat、libavutil、libswscale 的头文件及库。Rust 绑定固定为 `ffmpeg-next 9.0.0`，不固定原生库补丁版本。
- `opencv` 绑定默认依次尝试显式配置、pkg-config（`opencv4`/`opencv`）、CMake 与 vcpkg；显式变量可调整优先级。FFmpeg 未指定 `FFMPEG_DIR` 时使用平台 vcpkg 或 pkg-config。
- `.pc`/CMake 元数据必须对应实际安装的头文件和库。只有 `ffmpeg` 可执行文件、Python `cv2` 或运行库文件，不足以从源码构建。

macOS/Homebrew 标准安装示例（仅在依赖缺失时执行）：

```bash
brew install pkgconf opencv ffmpeg
cargo build --locked
```

Linux 使用发行版的开发包（如 `libopencv-dev`、`libavcodec-dev`、`libavformat-dev`、`libavutil-dev`、`libswscale-dev`、`clang`、`libclang-dev`、`pkg-config`）；先确认其版本满足范围，较老发行版的 OpenCV 可能不足 4.8。Windows 可使用 vcpkg 开发包及 LLVM/MSVC 工具链；DLL 仍需处于系统加载器可搜索的位置。

### 1.2 版本与链接诊断

```bash
make check-deps
# 或构建后分别运行，无需采集配置：
target/debug/rigcal-camera --check-deps
target/debug/rigcal-gui --check-deps
```

普通程序启动先校验原生库；FFmpeg 的同步探针和解码线程入口也校验。OpenCV 要求头文件与运行库 major/minor 相同、运行 patch 不旧于头文件；FFmpeg 四个库分别要求 ABI major 相同、运行版本不旧于头文件。不匹配明确报错，不自动换库或换后端。

`--check-deps` 显示头文件版本、实际运行版本、可执行文件路径和系统链接记录：macOS 使用 `otool -L`，Linux 使用 `ldd`，Windows 使用 `dumpbin /DEPENDENTS`。该工具缺失或执行失败会明确报错；`@rpath`/DLL 名称按原样显示，不冒充已解析绝对路径。

构建阶段需要查看实际探测的头文件目录、库目录、feature 与链接参数时，使用 `cargo build --locked -vv`。这些信息由绑定的原生探测器输出，不另维护一份库清单。

### 1.3 非标准安装覆盖（可选）

| 变量 | 用途 |
|---|---|
| `PKG_CONFIG_PATH` | 加入自定义前缀中的 `.pc` 目录，同时影响 OpenCV/FFmpeg 的 pkg-config 探测 |
| `FFMPEG_DIR` | 显式选择含 `include/`、`lib/` 的 FFmpeg 开发前缀，优先于 pkg-config |
| `OpenCV_DIR` / `CMAKE_PREFIX_PATH` | 使用 OpenCV 的 CMake 包描述 |
| `OPENCV_INCLUDE_PATHS` / `OPENCV_LINK_PATHS` / `OPENCV_LINK_LIBS` | 无包元数据时完整指定 OpenCV；列表用逗号，模块名称必须匹配所选 4.x/5.x 版本 |
| `LIBCLANG_PATH` | 仅在自动发现 libclang 失败时指定其目录 |
| `VCPKG_ROOT` 等 | 使用绑定库原生支持的 vcpkg 配置 |

例如使用自定义前缀或 Homebrew 的 keg-only 版本包时，可为该次构建设置 `PKG_CONFIG_PATH=/path/to/prefix/lib/pkgconfig`；如另有搜索路径，用系统路径列表分隔符追加。不要把这类本机值提交到仓库。自定义动态库的运行期查找由系统加载器或用户自己的 rpath 配置负责，仓库不再写死 rpath；改变原生库主/次版本后应重新生成绑定并重新构建。

只覆盖一部分路径可能混入另一套开发库。构建时的头文件、链接时库文件和运行期库必须来自兼容版本；编译通过不替代 `--check-deps` 与实际采集/标定验收。

## 2. 现场检查清单（只读，先跑这个）

```bash
make drill-check
```

等价于三步：

1. `raw-probe`（板端 raw 服务：能否取到帧、frame_id 是否前进）；
2. `group-probe`（四路同刻帧组：`group_id` 全等、`max_skew_ns`）；
3. `decode-probe` ×4（四路 RTSP 能否解码，尺寸是否 1280×1088）。

**端口以板端为准**：`~/demo/config/sensor_config.yaml` 的 `raw_server.port`（当前 **30432**）。
换端口：`--evidence host:port`。

## 3. 配置

两种模式**二选一**（同时给出会被拒绝）：

### 3.1 单相机（`capture`）

```yaml
device: {camera_id: cam0, image_size: [1280, 1088]}
capture:
  guidance: {type: rtsp, url: "rtsp://10.21.12.162:554/PRR"}   # 或 type: video, path: …
  evidence: {host: 10.21.12.162, port: 30432, camera: 0}
board: {…}          # AprilGrid 内联参数，measured: true 才允许标定
guidance: {…}       # 门禁：detect_hz / detect_hz_max / jitter_* / trigger_*；质量阈值在 solver.quality
solver: {…}         # models / min_observations / max_solve_observations / holdout_* / quality
output: {root: calibration_runs}
```

### 3.2 四路 rig（`rig`）

```yaml
device: {rig_id: robobaton_4p, image_size: [1280, 1088]}
rig:
  cameras:
    - {camera_id: cam0, guidance: {type: rtsp, url: "…:554/PRR"}, raw_camera_id: 0}
    # cam1..cam3 同理（:555/:556/:557）
  evidence: {host: 10.21.12.162, port: 30432}
  required_edges: [[cam0, cam1], [cam1, cam2], [cam2, cam3], [cam3, cam0]]
  required_cycles: [[cam0, cam1, cam2, cam3]]
  min_groups_per_edge: 3
  max_edge_rms_px: 1.0
  max_cycle_rotation_deg: 1.0
  max_cycle_translation_mm: 10.0
board/guidance/solver/output: 同上
```

未知键、越界值、互相矛盾的组合在加载期拒绝。清晰度使用 `solver.quality.min_focus_score`（Tenengrad，初值 500）。

### 3.3 四路运行链路

- 同组 `group_id`、组时间戳一致作为设备的**同步曝光约定**，不是主机重新测得的逐路曝光时刻。
- 显示独占线程：最新灰度帧先缩到 480 px 宽，再叠加最近检测结果、转 RGBA；叠加允许落后，不参与触发帧身份判定。
- 引导只在检测节拍做单帧质量、检测、姿态新颖度；不要求静止，不做 LK 或相邻帧抖动门禁。
- 尝试只启动冷却；只有 `Committed` 回执且触发相机确实入库，才更新该路新颖度基线。
- 取组、证据审核、数值求解、导出各自独占线程。候选只保留一个待处理项，替换旧候选会返回拒绝回执；在途证据队列容量为 1。
- 求解消费不可变的数据前缀，忙时合并到最新版；新增证据不等求解。旧完整快照只用于版本明确的显示；最终导出必须等待最新前缀全量精修，不能导出旧解冒充成功。
- 每个已审核非空帧组先同步写入会话角点日志，再入库；所有已接受观测保留，在线代表子集上限不影响留存。结束采集后用全部训练视图精修，holdout 始终独立。

## 4. 排障对照表

| 现象 | 先看哪里 | 常见原因 |
|---|---|---|
| `已采 0 版` 且 views 不涨 | 各格底部质量行与 `CLOCK_WAIT` 状态 | 画面模糊/低对比、时钟未就绪、板未检出或姿态未达到新颖度 |
| 某格状态 `SOURCE_ERROR` | 该格 detail | RTSP 连不上/流断了（日志里有 ffmpeg 原因） |
| 某格状态 `PANIC` | `gui.log` | 单帧 panic 已被隔离，只废掉那一帧 |
| 预览里板有描边但一直不拍 | 质量行、状态和 `触发` 计数 | 质量不达标、时钟未就绪、姿态不够新或刚拍过（冷却 0.5 s） |
| 求解一直 `rank` 不满 | 右栏 `rank` 与 `used/excl` | 姿态多样性不足（板要**转**起来、别只平移） |
| 时钟首次失败后等待恢复 | 独立 `CLOCK_RETRY` 行、重试次数与倒计时 | 自动退避重试，也可点“重试未就绪时钟”；检测状态不会覆盖时钟原因 |
| 已采增长、完整版本不动 | 各路“诊断 Vn”及红色求解错误 | 最新求解/评估失败；指标仍显示其标注的旧完整版本，不能据已采数量判断求解完成 |

日志位置：`rigcal-gui` 默认 `<output.root>/gui.log`（`--log` 覆盖）；`rigcal-camera` 直接打终端。

## 5. 无相机演练

演练需要三样：**引导视频**、**证据帧**、**证据服务**。

```bash
# ① 将已有的连续编号板图 frame_000.png… 转为 1280×1088 灰度 PGM
mkdir -p /tmp/drill_frames
ffmpeg -framerate 1 -i /path/to/frames/frame_%03d.png -pix_fmt gray -start_number 0 /tmp/drill_frames/frame_%03d.pgm

# ② 时钟推进的证据服务（与引导视频相同帧率；每个姿态保持一秒）
make drills-mock FRAMES=/tmp/drill_frames PORT=4211 FPS=60

# ③ 同一序列生成 60 Hz 引导；关闭 B 帧，避免文件开头的重排突发
ffmpeg -loop 1 -framerate 1 -i /tmp/drill_frames/frame_%03d.pgm -t 90 -vf fps=60 \
       -c:v libx264 -preset ultrafast -bf 0 -pix_fmt yuv420p /tmp/guide_cam0.mp4
cp /tmp/guide_cam0.mp4 /tmp/guide_cam1.mp4
cp /tmp/guide_cam0.mp4 /tmp/guide_cam2.mp4
cp /tmp/guide_cam0.mp4 /tmp/guide_cam3.mp4

# ④ 配置里的 evidence 指向 127.0.0.1:4211
cargo run -p rigcal-gui -- --config crates/rigcal-gui/example.rig.yaml --max-groups 32
```

演练帧可从标定板录像抽帧；合成板必须使用配置指定的字典与打印几何。

直接运行 `mock-raw-server` 不给 `--fps` 是按 LATEST 请求推进的协议探针模式，不能与实时视频做时钟标定。
时钟模式在第一个请求到达时启动固定单调时钟，把初始采样放在帧格中部，避免服务启动时间的随机相位干扰回放；之后不随请求数调整。它只验证软件链路与导出，不证明真实时钟精度、机架外参或曝光同步。

## 6. 时钟标定（引导帧 ↔ raw 帧对齐）

**为什么需要**：RTSP 解码时间戳与板端 raw 时间戳同速异域；`GET <cam> <ts>` 在板端帧网格做最近邻半帧匹配。偏移标定用于把触发帧时间戳映射到该网格。它不能识别整数帧偏差；命中误差小不等于证明引导图像与 raw 像素来自同一曝光。

```bash
make clock-calibrate            # 只读；真机 RTSP 554 + raw 30432
make clock-calibrate URL=rtsp://10.21.12.162:555/PRR CAM=1
```

**当前 GUI 流程**：

1. 每路独立线程等带 PTS 的新解码帧，再请求 raw `LATEST`；收集 40 样本、间隔至少 100 ms。
2. 两域序号/时间戳步长估帧周期，校验同速；`raw_ns − pts_ns` 折到帧内后估相位。偏移取不大于最小实测差的相位一致支路。**帧内相位散布 > 5 ms** 才拒绝；含整数帧台阶的原始散布单独报告，不用于此门槛。
3. 触发携带源代、帧序号、PTS、时钟代和 `pts + offset`；排队超 150 ms 或时钟代失效即拒绝。
4. 只按该目标 `GET` 四路并校验同组；误差 `hit = group_timestamp_ns − target` 可正可负，绝对值须在半帧 + 0.1 ms 内。
5. 有效命中后 **`offset += hit`**，把目标推到已命中的帧网格，不声称消除整数帧偏差。
6. `NO_MATCH`、同组校验失败或超容差：拒绝、记错、使触发路时钟失效并请求后台重标定；**无 LATEST 回退**。
7. 初标失败也自动重试（2/4/8/10 秒退避，上限 10 秒），失败期间无偏移、不能触发；可手动提前重试未就绪路。流结束后显示 `CLOCK_STOPPED`，不会拿旧偏移继续采集。

### 6.1 GUI 里已经接好的部分（可直接自测）

四路仪表盘启动时**每路独立后台采样**（40 样本 × 100 ms ≈ 4 s，raw 网络等待会延长采样），
首次采样和后续重标定均不阻塞预览与检测。每格独立时钟行显示 `CLOCK_SAMPLING` / `CLOCK_RETRY` / `CLOCK_READY`，就绪后同时列出帧内散布、5 ms 门槛、原始散布和实测周期。失效立即停用旧偏移，成功后才重新启用同刻取组：

| 环节 | 行为 | 看得见的证据 |
|---|---|---|
| 标定 | 等新解码帧 → `LATEST <cam>` → 估周期和帧内相位；两种散布分开报告 | 日志 `camN 时钟对齐完成` 或含原因的 `时钟标定失败`，失败后重试 |
| 触发 | 候选携带源代、序号、PTS、时钟代与目标板端时刻 | 回执日志携带 cam、frame、pts |
| 取组 | `capture_at(目标)` 取**同刻**那组（最近邻半帧容差） | 入库成功；计数行 `命中误差 p95 … ms`（期望 < 1 ms） |
| 闭环 | `offset += hit_error` | 计数行报告绝对命中误差 p95 |
| 自检 | `abs(group_timestamp_ns − target)` 超过本路半帧 + 0.1 ms → 拒绝并重标定 | 日志 `命中误差 … 超过半帧容差` |
| 失败 | `NO_MATCH`/超窗或命中误差过大 → 本次拒绝、计数 `对齐失败`，并请求重标定；**不使用 LATEST 回退** | 日志写明原因 |

**现场自测三步**（10 分钟内出结论）：

1. **只读体检**：`make clock-calibrate`（真机）→ 看帧内散布、原始散布与短窗漂移；`offset ≈ 板端 realtime`。探针与 GUI 复用同一估计器及 5 ms 门槛，超限非零退出；可直接给探针加 `--verify 3` 验证 GET 和正向闭环。
2. **跑 GUI 看计数行**：`make gui-online`（或 `rigcal-gui --rtsp-base 10.21.12.162 --config …`），
   观察 `对齐`、`命中 |误差| p95`、`对齐失败`，并换清晰的新姿态。
3. **持续换姿态**：`已采` 增长，`已算` 可以暂时落后；`完整` 只在整版内外参可用时更新。

探针 JSON：`samples, period_ns, offset_ns, phase_ns, phase_uncertainty_ns, raw_spread_ns, fetch_median_ms, drift_ns, window_s, ppm`。漂移用两窗偏移锚点差在**同一整窗周期**上折算，不能比较不同周期下的绝对相位；无法估计时后三项为 `null`。

### 6.2 计数行怎么读

```
已采 32 版 · 已算 V29 · 完整 V29 · 算中 V32
触发 38 · 拒绝 6 · 取组失败 0
对齐通过 · 对齐失败 0 · 命中 |误差| p95 0.000 ms
```

- `已采`：审核通过、已提交的数据版本（每次非空帧组加一），不是求解次数。
- `已算` / `算中`：数值线程完成评估 / 正在处理的前缀；评估完成未必得到完整解。
- `完整`：同版四路内参与连通外参的最新完整快照；新版本不完整不会覆盖它。
- `触发`：新鲜候选开始实际取组的次数；`拒绝`：替换、过期、重复或质量不符等回执。
- `取组失败`：四路 GET 失败；`对齐失败`：同刻取组失败或命中误差越界。
- `对齐`：四路均有可用偏移；任一路未标定、采样失败或重标定中则不通过，该路拒绝触发。
- `命中 |误差| p95`：最近最多 64 次有效命中残差的绝对值 p95，不含网络往返。
- 各路“诊断 Vn”对应最新已算前缀，包含求解拒绝、可观测性和 holdout 原因；“指标Vn”对应实际显示指标的版本。失败不被下一次采集成功清空，也不刷新收敛 streak。

### 6.3 DS 模型的后端选择

将配置的 `solver.models` 设为 `[ds]` 即使用 **Rust 原生 DS**；`[kb4]` 仍使用 OpenCV。
GUI 与单相机共用 `rigcal_camera::backend_for`，原生 DS 不需要 Python。

```bash
make gui-online                             # 模型由配置选择；DS 默认是 Rust
RIGCAL_DS_BACKEND=rust make gui-online        # 显式指定原生 DS
RIGCAL_DS_BACKEND=scipy RIGCAL_DS_PYTHON=.venv/bin/python make gui-online  # 显式对拍参考
```

后端名称拼错会报错，不会自动改用其它后端。scipy 需要 numpy/scipy/cv2，
启动时检查解释器与脚本路径，导入/求解错误在求解时上报；不作隐式回退。

DS 首次求解使用 KB4 的内参角度映射与已优化位姿；后续复用已接受的 DS 解。
原生优化器是第三方 `levenberg-marquardt`，不是自研求解器。GUI 的 KB4/DS 都在独占数值线程求解：空闲时算最新版本，忙时合并中间版本。
原生 DS 投影按可逆域 `z > -w2·‖P‖` 拒绝背轴折叠，仍允许有效的 `z < 0` 广角射线；优化位姿以 `log(tz)` 参数化，输出正深度可直接热启动。NaN 不替换成有效深度。SciPy 参考模型的有效域判据与原生模型不同，不应用于背轴或退化点等价性比较。

默认收敛门禁：RMS 上限 0.2 px、焦距相对标准差上限 0.2%；未达标时会话保持 collecting，不能把“求解成功”冒充“标定完成”。

DS 的轴心角度斜率为 `fx/(1+xi)`，不能直接把 DS 原始 `fx` 与 KB4 `fx` 判同。

## 7. 从真机换到演练（或反过来）的检查点

1. 引导源：`--rtsp-base` 一键切 RTSP；配置里写 `type: video` 则是文件源（自动按帧率节流）。
2. 证据端点：`--evidence host:port`（不给就用配置里的）。
3. 尺寸：引导、证据、配置三者必须一致（1280×1088），不一致会 fail closed 并打印实际尺寸。

## 8. 四路标定结果导出

右栏 `结束采集并全量精修导出` 停止接收新候选，排空在途取组与审核，然后对最新前缀执行全量精修。达到 `--max-groups`、正常关窗或定时结束也走同一路径。
在线 `max_solve_observations` 仅限制实时优化代表子集；最终 `Session::refine()` 使用全部可用训练观测，holdout 不回流。四路解、连通外参和必需环均完整才导出；不完整或精修失败明确报错，绝不把旧在线结果冒充最终结果。质量阈值不变。
写盘仍归独立导出线程；到上限后窗口继续显示，采集停止。同版关闭复用已写目录。最终求解/写盘失败可从完整角点日志离线重放；重放也失败则非零退出，不静默降级。

产物位于 `<output.root>/exports/run-<时间戳>/`，每次新快照使用独立目录，不覆盖旧结果：

| 文件 | 内容与坐标约定 |
|---|---|
| `calibration.yaml` | `cam0..cam3` 的模型、带名称的内参向量、分辨率、质量指标，以及四个 `T_c0_ci`；满足 `P_c0 = T_c0_ci * P_ci`，cam0 为单位阵，平移单位 **m** |
| `camchain.yaml` | Kalibr 格式；KB4 为 `pinhole/equidistant`，DS 为 `ds/none`；相邻变换 `T_cn_cnm1` 满足 `P_cn = T_cn_cnm1 * P_cnm1`，**不是** `T_c0_ci` |

- `VALIDATED`：四路内参收敛，且配置要求的边/环质量门禁全部通过。
- `DRAFT`：几何结果完整，但还有质量门禁未通过；主文件的 `warnings` 与 camchain 注释明确列出原因，不能当作验收合格。
- 缺相机、模型不一致、无效参数/刚体矩阵、缺必需边/环或图不连通：拒绝导出。
- 两个文件写入同一临时目录、同步后原子发布目录；失败时清理临时目录，已有导出不受影响。

先看 `status` 和 `warnings`，再使用参数。`求解成功`、`导出成功`、`质量验收通过` 是三件不同的事。

### 8.1 完整观测留存与离线重算

每次采集创建独立的 `<output.root>/sessions/s-*/`，不覆盖旧会话：

- `config.yaml`：已应用 CLI 覆盖后的实际配置；重放使用同一 schema 校验。
- `observations.jsonl`：一行一组，保存版本、组 id/时间戳、相机 id、帧 id/时间戳、全部接受的 3D/2D 角点；不保存原始像素。
- 整行写入并 `sync_data` 成功后才提交内存入库。截断尾行、未知字段、坏 JSON、身份/单调性异常均明确报错，不偷偷丢行。

```bash
cargo run -p rigcal-gui -- \
  --replay-observations <会话目录>/observations.jsonl \
  --out <新的导出根目录>
```

重放不打开 GUI、不连接相机，与在线结束共用全量精修、外参求解及导出门禁；`--out` 可省略，默认使用留存配置的输出根目录。不要同时传 `--config` 或采集选项。日志保存的是接受的角点，不包含可重新检测的 raw 图像。
