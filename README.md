# 4P_Calibration

4P 四目标定工具（Rust）：**单相机内参 + 四相机外参**，在线采集、实时收敛判据、导出 Kalibr camchain。
默认产线不依赖 Python；DS 可显式选择 SciPy 参考后端。

## 仓库结构

```
crates/
  rigcal-core       纯算法：板几何 / DS·KB4 模型 / 位姿 / 可观测性 / 会话状态机 / holdout / 外参 / 旋转
  rigcal-io         RTSP / raw 取组 / 时钟相位对齐 / 角点会话日志 / 标定导出
  rigcal-opencv     OpenCV 适配：KB4 求解后端、单帧质量门禁
  rigcal-ds         DS 联合标定：第三方 LM + core 模型；SciPy 桥只作显式对拍
  rigcal-camera     单相机产线 bin（离线目录回放 / 在线 RTSP+raw）+ 可复用库（检测/预览/门禁）
  rigcal-gui        四路仪表盘 bin（Slint）：2×2 实时预览 + 检测叠加 + 右栏收敛进度
docs/               见 docs/README.md（运行手册、设备事实）
local/              现场配置（gitignored）：camera_session.yaml 等
```

## 下载可运行的发布包

GitHub Actions 的 `Portable release` 为 **Windows x86_64、Linux x86_64、Linux ARM64（aarch64）** 构建带原生运行库的压缩包。功能分支推送产出预览 artifacts；与工作区版本一致的 `v*` 标签在三平台全部通过后创建 **Release 草稿**。

解压完整目录后，修改 `config/*.example.yaml` 即可运行，不需要安装 Rust、Python 或 OpenCV／FFmpeg 开发包。Linux 要求 glibc ≥2.35；GUI 需要桌面环境与 OpenGL 驱动。下载、校验、启动与发布步骤见 [发布流程](docs/operations.md#14-跨平台发布包与-release-流程)。

## 构建

使用系统安装的 **OpenCV 4.8+ / 5.x** 与 **FFmpeg 6.x–9.x** 开发库，不固定补丁版本或机器路径。Rust 绑定版本由 `Cargo.lock` 固定。

构建需要 Rust ≥1.92、C/C++ 工具链、Clang/libclang 和 pkg-config（OpenCV 也支持 CMake/vcpkg 探测）。标准包管理器安装的开发库通常不需要手工 export；单独的 `ffmpeg` 命令或 Python `cv2` 不等于开发包齐全。

macOS/Homebrew 可安装 `brew install pkgconf opencv ffmpeg`；Linux 需要发行版对应的开发包，且版本应在上述范围内。构建与非标准安装覆盖方法见 [构建环境](docs/operations.md#1-系统构建环境)。

```bash
cargo build          # 只产出产线入口：rigcal-camera / rigcal-gui
cargo test           # 工作区回归
cargo clippy --all-targets -- -D warnings
```

`make check-deps` 检查两个产线入口的编译期/运行期版本和系统链接记录，不连接相机。普通启动也会执行 ABI 检查，不匹配立即拒绝运行。

演练/探针工具（`raw-probe` / `decode-probe` / `group-probe` / `mock-raw-server`）默认**不构建**，
需要时显式 `--features drills`。等价入口见 `make help`。

## 怎么跑

| 场景 | 命令 |
|---|---|
| 四路仪表盘（真机） | `cargo run -p rigcal-gui -- --config crates/rigcal-gui/example.rig.yaml --rtsp-base 10.21.12.162` |
| 四路仪表盘（无相机演练） | `make drills-mock FRAMES=<.pgm 帧目录>` + `cargo run -p rigcal-gui -- --config crates/rigcal-gui/example.rig.yaml` |
| 四路观测离线重算 | `cargo run -p rigcal-gui -- --replay-observations <会话目录>/observations.jsonl --out <输出目录>` |
| 单相机离线回放 | `cargo run -p rigcal-camera --bin rigcal-camera -- --config local/camera_session.yaml --frames <图片目录> --out <输出目录>` |
| 单相机在线 | `cargo run -p rigcal-camera --bin rigcal-camera -- --config local/camera_session.yaml --live --out <输出目录>` |
| 现场链路自检（只读） | `make drill-check` |

`rigcal-gui` 采集参数：`--config`（采集必填）、`--rtsp-base <host[:port]>`（一键切在线，`camN → :port+N/PRR`）、
`--rtsp-path`、`--evidence <host:port>`、`--max-groups`、`--run-seconds`、`--log <file>`。
GUI 没有控制台，诊断写日志（默认 `<output.root>/gui.log`），**关键状态同时显示在界面右栏**：
`已采 / 已算 / 完整 / 算中` 版本、触发/拒绝/取组失败、对齐状态与红色错误行。

四路结果通过右栏 `结束采集并全量精修导出` 保存；关窗或采满上限也先精修最新前缀，再导出，不沿用旧在线解。
产物在 `<output.root>/exports/run-*/{calibration,camchain}.yaml`；质量未达标明确标为 `DRAFT`。
坐标约定与失败边界见 [运行手册 §8](docs/operations.md#8-四路标定结果导出)。
接受的完整角点和实际配置持续留在 `<output.root>/sessions/s-*/`；`--replay-observations` 可不连设备重新全量求解，见 [观测留存与重算](docs/operations.md#81-完整观测留存与离线重算)。

模型由 `solver.models` 选择：`[kb4]` 用 OpenCV，`[ds]` 默认用原生 Rust DS；
SciPy 是显式可选后端（`RIGCAL_DS_BACKEND=scipy`）用于参考对拍，不作隐式回退。详见 [运行手册 §6.3](docs/operations.md#63-ds-模型的后端选择)。

## 现场事实速查

| 项 | 值 |
|---|---|
| 四路 RTSP | `rtsp://10.21.12.162:554..557/PRR`（端口为基址 554 + 通道号） |
| 板端 raw 帧服务 | 端口**以板端 `~/demo/config/sensor_config.yaml` 的 `raw_server.port` 为准**；当前为 **30432** |
| 触发门禁 | 单帧质量（清晰度/对比度/削顶）∧ AprilGrid 检出 ∧ 姿态新颖（×3）∧ 冷却 0.5 s |
| 四路 GUI 证据帧 | 只来自板端 raw 服务；按 `GET <cam> <pts+offset>` 取组，raw 或时钟不可用时 **fail closed**；时间戳不证明整数帧对应 |

## 文档

- `docs/operations.md` —— 运行手册与排障（构建环境、现场检查清单、配置参考、故障对照表、无相机演练）
- `docs/device-4p.md` —— 设备侧事实（RTSP/raw 协议契约、时间戳语义、硬限制）
