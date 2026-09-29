# RoboBaton 4P 设备使用方式（标定工具视角）

本文说明标定工具使用的设备接口、数据语义和运行约束。来源栏保留设备文档或板端源码位置；设备固件、端口及启用能力以实际交付配置为准。

## 1. 设备构成

| 项 | 值 | 来源 |
|---|---|---|
| 计算平台 | Degu X5，8×Cortex-A55 @1.5 GHz，RAM 4 GB，存储 32 GB | getting-started/Product_Introduction.html |
| 相机 | 4× SC132GS，丝印 CAM1–CAM4 ↔ 软件 cam0–cam3 | getting-started/hardware-and-safety.html |
| 原生画布 | sensor 轴向 1088×1280，挂载旋转 90°，交付画布 1280×1088 | RoboBaton_4p_demo `include/sc132camera.h` |
| FOV 形态 | A：H 148.4° / V 126.6° / D 193.8°；B：H 115.6° / V 96.8° / D 157.2° | Product_Introduction.html / development/data-contracts.html |
| IMU | TDK ICM-42688-P，`/dev/spidev2.0`，SPI mode0 4 MHz，DRDY GPIO395，默认 1000 Hz | development/data-contracts.html |
| 供电 | DC 12–24 V，≥600 mA；USB-C 仅调试，不作供电；相机 FPC/同轴不支持热插拔 | getting-started/hardware-and-safety.html |

相机/IMU 硬同步、TF 外参、内参与畸变标定**公开交付里都没有**（`CameraInfo` 只有宽高，标定字段为空）。

## 2. 视频流：RTSP 是唯一网络视频通路

```
CAM1/cam0 -> rtsp://<ip>:554/PRR
CAM2/cam1 -> rtsp://<ip>:555/PRR
CAM3/cam2 -> rtsp://<ip>:556/PRR
CAM4/cam3 -> rtsp://<ip>:557/PRR
4 合 1 拼图 -> rtsp://<ip>:558/PRR   （2560x2176，固定 H.264 / 8000 kbps，仅 fps 可配）
```
来源：quick-start.html、usage/non-ros-demo.html、development/data-contracts.html；端口=基址 554 + 通道号（`cam_demo_common.cpp`），路径固定 `/PRR`，默认 H.264，可切 H.265。

- 文档给的客户端自检命令（TCP 传输）：
  ```bash
  ffprobe -v error -rtsp_transport tcp -select_streams v:0 \
    -show_entries stream=codec_name,width,height,avg_frame_rate \
    -of default=noprint_wrappers=1 rtsp://<ip>:554/PRR
  ```
- **流只承载编码帧**：“RTSP 客户端接收 H.264/H.265 编码流，RTSP 不直接承载 NV12 原始帧”（data-contracts.html）。

## 3. 原始 NV12 取图：RWF1/TCP

证据帧只用板端 raw 帧服务，RTSP 只做引导预览。引导源也可为本地视频；已接受角点可离线重算。工具不连接 ROS2，raw 服务不可用时拒绝采集，不切换证据来源。

**raw 服务的协议契约**（Rust 侧实现在 `crates/rigcal-io/src/raw_tcp.rs`；板端为 `sensor_demo --raw-server-*`，对应板端 `RawFrameResponseHeader` 布局）：

| 项 | 约定 |
|---|---|
| 传输 | TCP，**每条连接只应答一次请求**：每次取帧 = 建连 + 一行 ASCII 请求 + 读响应 |
| 请求 | `LATEST <camera_id>\n`；`GET <camera_id> <timestamp_ns>\n`（按 `group_timestamp_ns` 最近邻、服务端半帧容差） |
| 响应头 | 96 字节小端 packed `<IIiiQQQQIIIIQ3Q`：`magic("RWF1",0x31574652)/version(=1)/status/camera_id/group_timestamp_ns/camera_timestamp_ns/frame_id/group_id/width/height/stride/vstride/payload_size/reserved×3` |
| status | `0 OK` / `1 NO_MATCH` / `2 BAD_REQUEST` / `3 CAMERA_DISABLED` / `4 NO_FRAME` |
| 负载 | 紧凑 NV12，总长 `w*h*3/2`；前 `w*h` 字节即 Y 平面。它是**未经 H.264 编码的亮度**（因此不受编码损失），但**已经过 sensor→ISP→VSE→旋转流水线，不等于 sensor RAW**；标定身份必须绑定 ISP tuning、画布尺寸、旋转与 FOV 形态 |
| 预检 | 四路各取一帧 `LATEST`，校验 magic/version/尺寸/`camera_id` 与 Y 平面长度；再扇出 N 组核验 `group_id` 一致性与 skew |

工具默认设备端点为 `10.21.12.162:30432`，可用 `--evidence host:port` 覆盖。端口以板端 `~/demo/config/sensor_config.yaml` 的 `raw_server.port` 为准，运行前确认相应服务已启用。

**两个必须分开的量**：`match_tolerance_ns`（服务端半帧容差，决定「给定锚点选中哪一帧」，30 fps 下 ≈16.7 ms）与 **ring 保留窗口**（锚点在被挤出 ring 前都能命中，与容差无关）。公开文档**没有 ring 深度契约**，因此回查预算只能由现场预检实测，不能从容差推算。

**`camera_timestamp_ns` 的使用边界**：当前设备的 `software_gpio` 组内各路时间戳相同，不能据此测得逐路曝光误差。主机计算的 timestamp 极差仅作诊断，不能替代生产者的同步合同；ring 保留窗口需单独检查。

## 4. 时间与同步

| 事实 | 值 | 来源 |
|---|---|---|
| 触发模式 | 默认使用 `software_gpio`（GPIO417 软件触发）；`none` 为实验性自由运行模式 | data-contracts.html / product-and-compatibility.html |
| 帧组契约 | `camera_count`（完整组=4）、`group_id`、`group_timestamp_ns`、`max_skew_ns`；每路 `camera_id / frame_id / timestamp_ns` | data-contracts.html |
| 组内时间戳 | `software_gpio` 组用 GPIO417 上升沿的 `CLOCK_MONOTONIC_RAW` 时间，**组内四路完全相同** | `include/sc132camera.h` |
| 准入偏差 | `max_skew_ns` 默认 **10 ms**；等待超时默认 **100 ms** | data-contracts.html / `sc132camera.h` |
| 墙钟映射 | 进程启动时冻结 `CLOCK_REALTIME − CLOCK_MONOTONIC_RAW`；启动后再改系统时间**不会**更新映射。ROS2 `header.stamp` = 该映射值，非发布时间 | `timestamp_mapper.cpp` / system-time-sync.html |
| RTSP 帧时间戳 | 解码 PTS 与板端 realtime 属于不同时间域，必须估偏移后才能请求 raw GET；帧网格匹配不证明整数帧身份 | `crates/rigcal-io/src/clock.rs` |
| 系统级同步 | NTP（需 UDP 123）、PPS（UART7 RX→GPIO379→`/dev/pps2`，3.3V）、X5 作 PTP master（`/dev/ptp0`，UDP 319/320，`phc2sys`/`ptp4l`）；**PPS 通过 ≠ 相机/IMU 已硬同步** | time-sync/* |

工具以相同 `group_id` 和 `group_timestamp_ns` 校验四路帧组，不把墙钟相同当作新的物理同步测量。若需要墙钟对齐，应在启动采集进程前完成系统时间同步；运行期仍以 GET 残差检查帧网格偏移。

## 5. 图像配置与画质

- **画布白名单**：原生 1280×1088；VSE 整幅缩放 640×480 / 720×480 / 1280×720（另有轴序互换形式）；其他值在启动前被拒。缩放为整幅拉伸、**FOV 不变**，故内参必须按「实际交付画布 + 旋转 + A/B 形态」分别估。
- **旋转**：`--rotate 0|90|180|270`（顺时针，外部角）；内部旋转 = `(外部 + 90) % 360`；`180` 由 Nano2D 在缩放后做，**仅 30 fps 支持**（25/40/50/60 被拒）。
- **帧率**：25/30/40/50/60，默认 30；40/50/60 在高 CPU 负载下可能丢帧（60 最明显）。
- **RTSP 编码**：`codec` 默认 h264，`bps` 默认 4000 kbps，`url` 默认 `/PRR`。
- **相机掩码**：`camera.camera_mask` 默认 15（四路）；**只支持单路或四路**，没有 2/3 路组合（单路诊断用 `./cam_demo --camera-id 0..3`）→ 外参标定只能整机四路一起开。
- **没有任何 crop / ROI / GDC 去畸变**：几何控制只有 VSE 整幅缩放与 0/90/180/270 旋转，畸变只能靠标定模型（DS/KB4）表达。
- **ISP 调参**：`patch/sc132gs_tuning.json` → `/usr/hobot/lib/sensor/sc132gs_tuning.json`，**需重启**生效。

## 6. 启动与服务

服务不变量：`cam-service` 必须常驻；相机/VIO/编码资源独占，**启动新相机程序前先退出旧的**（`Ctrl+C`，再 `pgrep -af 'sensor_demo|cam_demo|robobaton_sensors_node'`）；non-ROS 与 ROS2 两条路**不能同时开**。

non-ROS（`/root/demo`）：
```bash
cd /root/demo
./sensor_demo          # 四路 + PRRTSP + IMU
./cam_demo             # 四路 + RTSP
./mosaic_rtsp_demo     # 四路拼图，:558
./imu_reader_demo      # 仅 IMU
```
常用参数包括帧率、编码、旋转、码率、触发模式、诊断与输出尺寸。参数名称及可用性以设备交付版本为准。配置优先级：`${DEMO_DIR:-cwd}/config/sensor_config.yaml` → 显式 CLI 覆盖。自启管理：`./start_sensor_demo.sh enable|disable|status`。

ROS2（`/root/ros2_demo/install`，Humble）：
```bash
source /root/ros2_demo/install/robobaton_ros2_env.bash
ros2 launch robobaton_4p_ros2_demo robobaton_sensors.launch.py
```
默认参数要点：`camera.fps 30`、`camera.rotate_degrees 0`、`camera.camera_mask 15`、`camera.frame_set_max_skew_ns 10000000`、`camera.frame_set_timeout_ms 100`、`camera.queue_capacity 4`（`queue_policy block`；`drop_newest` 不保证四帧组完整）、`camera.image_encoding nv12`、`camera.trigger_mode software_gpio`。

## 7. 网络

- 出厂地址：板卡 `192.168.1.12/24`。直连主机应使用同网段空闲地址；认证配置以设备交付资料为准。
- 改 IP：改 `/etc/network/interfaces` 后 `reboot`（先 `cp -a` 备份）；恢复走 1.8V DEBUG_UART（921600）。
- Wi-Fi：`wifi_setup.sh`，AP 默认 SSID `RoboBaton-X5` / `192.168.5.1/24`（AP 模式无 NAT），STA 用 `udhcpc`；切换会重启 `wlan0` 并断开当前 SSH。
- 工具示例使用 `10.21.12.162`，部署时请检查实际 IP、SSH、RTSP 与 raw 服务端口。

## 8. 标定接口约束

1. 线上引导使用 RTSP，本地演练可使用视频文件。编码后的亮度只用于引导，不替代 raw 证据帧；客户端不依赖 SEI 传递时间戳。
2. 证据帧只走 RWF1/TCP。GUI 时钟采样使用 `LATEST`，触发后按 `pts + offset` 指定时刻 GET 四路；失败拒绝，不回退 LATEST。单相机采集路径和各探针的请求方式由对应入口决定。
3. 四路组身份由 `group_id/group_timestamp_ns` 表达；`software_gpio` 下组内时间戳相同。板端默认组准入偏差 10 ms，不等于主机测得的曝光误差。
4. 只有「单路」或「四路」两种相机组合；内参可以单路做，外参必须四路齐开。
5. 画布/旋转/FOV 形态一变，内参必须重标；无 GDC/crop，畸变全靠模型。
6. 若使用墙钟对齐，先同步系统时间再启动采集；进程启动后映射被冻结，不随系统时间修改而更新。
7. 超广角（A: 148.4°H）意味着畸变极强，标定必须覆盖画面边缘，且要能表达强畸变模型。
