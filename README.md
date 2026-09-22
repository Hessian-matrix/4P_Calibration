# 4P_Calibration

RoboBaton 4P 主机端单相机内参标定工具，独立版本 `0.1.0`。从已有 DS/KB4 标定实现整理，支持逐颗标定 cam0–cam3；可独立安装，不依赖 4cam 主仓、ROS2 或 X5 SDK。

## 已有功能

- 单路 H.264/H.265 RTSP 解码为 `1280x1088 mono8`，最新帧采集，终端交互及可选 OpenCV 预览。
- AprilGrid 检测及原始亮度图上的亚像素角点细化；质量、覆盖、尺度和姿态采集引导。
- KB4 与 Double Sphere 双模型求解，独立阶段采集 official holdout，输出模型 YAML、残差报告和数据 manifest。
- 已有会话离线审计：重新检测、holdout、cross-tag、边缘残差、稠密投影有效域和去畸变预览。
- Kalibr 数据导出与文件级结果比较；单幅图像字典/板型诊断。

当前命令行采集入口使用 RTSP；离线命令用于已有会话的审计和导出。外参、相机–IMU 标定、EEPROM 写入和板端部署不在本工具范围内。当前整理未完成产品 V1.3.0 消费者兼容性验证。

## 安装

推荐 Linux / WSL、Python 3.10–3.12。FFmpeg 与 FFprobe 需位于 PATH。OpenCV 使用带 AprilTag 模块的 contrib 版本；请使用独立虚拟环境，避免同时安装多个提供 `cv2` 的包。

```bash
sudo apt install python3-venv ffmpeg libgl1 libglib2.0-0
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
robobaton-camera-calibrator --version
robobaton-camera-calibrator --self-check --json
```

开发时可用 `python -m pip install -e .`。安装后可从任意工作目录调用命令，也可用 `python -m robobaton_calibration`。默认输出位于调用目录的 `calibration_runs/`。

## 配置标定板

配置随 Python 包安装；源码中的位置为 `src/robobaton_calibration/configs/`。复制后填写实测尺寸：

```bash
mkdir -p local
cp src/robobaton_calibration/configs/targets/aprilgrid_6x6_36h11_template.yaml local/target.yaml
cp src/robobaton_calibration/configs/online_intrinsic_v1.yaml local/session.yaml
```

编辑 `local/target.yaml`：将 `tag_size_m` 填为实测 tag 边长（米），`tag_spacing_ratio` 填为相邻 tag 间距 / tag 边长，核对板型后设置 `measured: true`。模板不会被当成真实标定板使用。

板坐标原点位于正视图左下角，ID 从底行左到右递增，再逐行向上。`tag_corner_order` 是 OpenCV 返回角点对应的板坐标顺序，必须匹配实际打印方向。6×6 模板沿用已有板的 `[bottom_right, bottom_left, top_left, top_right]`；8×6 模板使用 `[bottom_left, bottom_right, top_right, top_left]`。两种顺序不可仅凭板尺寸互换。6×6 默认采集门禁要求 holdout 至少 28 tags、6 行和 6 列；换板时需同时核对 session 中的采集门禁。

## 在线采集

### 只输入相机编号

完成上面的 `.venv` 安装和 `local/target.yaml` 实测板配置后，在本工具目录创建一次本机配置：

```bash
printf 'CALIBRATION_HOST=%s\n' '<设备IPv4地址或主机名>' > local/calibration.conf
```

此文件是由启动脚本加载的可信 Bash 配置，只写自己的设备设置，不加载外部提供的脚本。地址和实测板参数留在已忽略的 `local/`，不作为其他设备或标定板的通用默认值。

以后启动只需：

```bash
./calibrate.sh 0
```

编号只接受 `0`、`1`、`2`、`3`，自动选择 `554 + 编号` 端口、`/PRR` 和 `cam编号`。主仓根目录可用 `./4P_Calibration/calibrate.sh 0`；脚本从其他工作目录调用也使用自身目录下的配置和虚拟环境，无须激活环境或设置 `PYTHONPATH`。

- 使用已安装包的默认 session 门禁、`robobaton_4p` rig 和 `accepted` 保存模式；不自动采集或求解。
- 存在 `DISPLAY` 或 `WAYLAND_DISPLAY` 时打开 0.5 倍预览，预览每 5 帧检测一次；无显示环境时只使用终端命令。
- 每次结果写入本工具目录下的 `calibration_runs/robobaton_4p/cam编号/UTC时间戳/`，启动时打印实际 URL、板配置和输出目录。
- 不连接 SSH、不启动或重启板端服务、不修改网络配置。RTSP 超时应先检查设备服务和主机到设备的路由/源地址。
- 需要自定义 session、保存调试标注等参数时，使用下面的完整命令行入口。更新工具源码后须重新安装到 `.venv`。

### 完整参数入口

将 URL 中的地址替换为实际设备地址。常用端口映射为 cam0→554、cam1→555、cam2→556、cam3→557，路径 `/PRR`；最终以设备运行配置为准。`--camera-id` 只是记录标识，URL 必须选择对应相机。

```bash
robobaton-camera-calibrator \
  --rtsp-url 'rtsp://<device-address>:554/PRR' \
  --camera-id cam0 \
  --target local/target.yaml \
  --session-config local/session.yaml \
  --save-data accepted \
  --output-dir calibration_runs/cam0-session
```

有图形桌面时追加 `--preview-window --preview-scale 0.5 --preview-detect-every-n 30`。WSL 需要可用的 WSLg/X11 显示环境；headless 环境不加该选项。窗口焦点下直接按键；终端输入命令后按 Enter：

| 按键 | 行为 |
|---|---|
| `s` | 从预览进入训练采集 |
| `h` | 训练覆盖 ready 后，进入独立 holdout 采集，使用新的姿态 |
| `f` | holdout 收足后完成采集并求解 |
| `r` | 清空当前内存采集状态，重新采集 |
| `q` | 退出当前会话 |

默认至少 12 帧训练候选、8 帧 official holdout，求解总数上限 30。训练帧还需满足网格、边角、尺度和姿态覆盖要求，终端的 `missing` / `next` 给出缺口。只达到 coverage 门禁的帧不会进入优化。求解阶段同步执行，预览可能暂时停止更新，终端继续输出 `SOLVER_PROGRESS`。

真实采集成功返回退出码 `0`；失败、未收足数据或按 `q` 取消返回 `1`。参数错误返回 `2`。监督器可能把主动取消显示为 failed；应结合 `session canceled` 和最终日志判断。

输入尺寸固定为 `1280x1088 mono8`，不做 resize、旋转、裁剪或去畸变。RTSP 解码亮度图仍经过有损编码，不等同于 sensor 原始亮度数据。镜头、焦距、分辨率、裁剪或 GDC 状态改变后需重新验证。

成功求解时输出：

```text
calibration_runs/cam0-session/
  target.yaml                 # 本次实测板配置快照
  session_config.yaml         # 本次采集/审计配置快照
  manifest.json               # 帧 hash、split、工具版本
  dataset/train/              # 优化帧
  dataset/holdout/            # 独立验证帧
  dataset/accepted/           # 未进入求解的采集证据（存在时）
  models/ds.yaml
  models/kb4.yaml
  model_comparison.json
  validation.json
  selected_model.yaml
  report.md / report.html / report.pdf
  plots/
  terminal.log
```

输出目录必须为空或不存在。正常求解前 accepted 图像保存在内存中；取消或中断不会得到完整可重放数据集。`--save-data off` 不保存图像，无法完成后续完整审计和导出；推荐 `accepted`。`--save-debug-overlays` 额外保存标注图，不改变求解输入。

DS 参数顺序为 `[xi, alpha, fx, fy, cx, cy]`，畸变为 `none`；KB4 为 `pinhole` + `equidistant`，内参 `[fx, fy, cx, cy]`，畸变 `[k1, k2, k3, k4]`，焦距/主点单位为像素。

`CALIBRATION_RESULT PASS` 表示当前求解/误差门禁通过。模型默认保持 `UNDECIDED`，不自动生成唯一权威 `calibration.yaml`；质量通过不等于模型已选择或获准生产使用。

### 板端 raw TCP 原帧入口

板端 `sensor_demo` 可开启 raw 帧服务，按请求返回对应时间戳的紧凑 NV12 原帧（未经 H.264 有损编码，Y 平面即传感器原始亮度）。主机工具通过 `--raw-tcp HOST:PORT` 直连该服务，替代 `--rtsp-url` 作为帧来源；`--camera-id` 必须是 `0`–`3`（或 `cam0`–`cam3`）数值编号。两个来源互斥，只能二选一：

```bash
robobaton-camera-calibrator \
  --raw-tcp '<device-address>:<raw-server-port>' \
  --camera-id cam0 \
  --target local/target.yaml \
  --session-config local/session.yaml \
  --save-data accepted \
  --output-dir calibration_runs/cam0-raw-session
```

采集流程、交互命令与求解门禁与 RTSP 入口完全相同；检测和求解都作用在 raw Y 平面上。板端服务每条 TCP 连接只应答一次请求（`LATEST` 取最近帧，`GET` 按 `group_timestamp_ns` 最近邻半帧容差匹配），主机端每帧新建连接请求 `LATEST`。协议详见板端 `docs/raw-frame-server-guide.md`；`terminal.log` 会记录 `raw_tcp` 端点，manifest 的 `source` 标记为 `raw-tcp` 以区分 RTSP 有损亮度。

## 离线审计与导出

审计不改输入会话或重新优化内参，必须使用采集时的实测板和配置。旧会话没有配置快照时，显式提供原始 YAML。

```bash
robobaton-calibration-audit \
  --run-dir calibration_runs/cam0-session \
  --target calibration_runs/cam0-session/target.yaml \
  --session-config calibration_runs/cam0-session/session_config.yaml \
  --output-dir calibration_runs/cam0-audit

robobaton-calibration-export-kalibr \
  --manifest calibration_runs/cam0-session/manifest.json \
  --target calibration_runs/cam0-session/target.yaml \
  --output-dir calibration_exports/cam0

robobaton-calibration-compare \
  --robobaton-report calibration_runs/cam0-session/model_comparison.json
```

审计输出 `validation_audit.json`、重检测记录、残差/覆盖/有效域图和去畸变预览。审计和导出目录同样必须为空或不存在。

Kalibr 导出在 `cam0/data/` 中仅放训练帧，holdout 单独置于 `holdout/cam0/data/`，不能加入外部优化。输出实测 `target.yaml`、模型 YAML、图像清单、原始板配置和 `export_metadata.json`。当前支持 36h11、从 0 开始的 ID；需核对实体板方向与 Kalibr 约定。文件名使用帧序号生成的 30Hz 合成时间序列，仅表达顺序，不能用于时间同步或相机–IMU 标定。导出不生成 ROS bag，也不执行 Kalibr。

比较命令可追加 `--kalibr-ds-report` / `--kalibr-kb4-report` 检查外部 YAML 的模型、分辨率和可用误差字段。它只是文件与指标检查，不代表已经完成独立实现复核。

单张图像诊断：

```bash
robobaton-camera-calibrator --diagnose-image frame.png --target local/target.yaml --json
```

## 源码与数据边界

源码来自原标定工具的算法、采集、报告和审计实现，移除了主仓相对路径、产品版本绑定、固定设备地址和内置测试 fixture 入口。内部回归测试、实拍 fixture、原始运行记录和候选模型不随本仓迁入。数据、导出和个人配置目录已列入 `.gitignore`。

当前仓库尚未指定开源许可证。公开发布前需由权利人确认迁入代码与素材的授权；本次整理不替原代码重新授权。Python 依赖与 FFmpeg/Kalibr 等外部工具遵循各自许可证。
