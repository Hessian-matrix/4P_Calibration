# 4P 离线标定应用实施方案

状态：离线标定已实现；TUI 使用配置工作台、运行监控与结果检查闭环。
更新日期：2026-10-20
适用范围：单 Cam 内参、多 Cam 联合内外参、多 Cam—IMU 联合标定  
算法依赖：`camera_corelib` 是唯一算法实现和 ROS bag 读取实现

## 1. 决策摘要

本项目是离线标定应用，不再包含在线采集、在线标定或设备控制。产品主流程固定为：

1. 在 TUI 中选择 ROS bag 文件或目录，预加载并解析 IMAGE/IMU 话题，展示消息数量、header 时间范围、帧率与时间质量。
2. 配置传感器—话题配对、标定模式、相机模型、可选内参初值、AprilGrid 和求解参数；配置通过完整校验后才能启动。
3. 执行检测、分组、初始化、求解和原子导出；TUI 展示阶段活动、真实逐迭代收敛曲线、诊断和结果。

全应用统一使用 ROS 消息的 `header.stamp` 作为图像与 IMU 的传感器时间。`record_time_ns` 和 MCAP `publish_time_ns` 只用于诊断，不参与同步或标定，也不作为 header 缺失时的回退。

开发阶段所有 `ccl_*` crate 统一使用相邻 `../camera_corelib` 工作树的源码级 path 依赖。接口稳定后一次性切换到同一个不可变 Git revision；禁止 path/Git 混用造成两份领域类型。

## 2. 目标与边界

应用只负责：

- ROS bag 路径选择、格式选择、话题发现、统计和产品语义适配。
- 简单的运行设置、传感器—话题配对和本次运行快照。
- 通过 `ccl_rosbag` 读取容器和动态消息，通过 `ccl_image` 检测 AprilTag。
- 将检测和 IMU 数据转换为 `ccl_calib` 的权威请求类型。
- 编排初始化、求解、结果检查、运行记录和原子导出。
- 通过 Ratatui 展示数据状态、配置、作业进度、诊断和结果。

检测、投影、反投影、PnP、内参求解、多相机外参、视觉惯性联合优化、可观测性、协方差和收敛判定全部调用 `camera_corelib`。应用不得复制这些算法，不得定义第二套相机参数、位姿或求解结果真值。

明确不包含：

- RTSP、raw TCP、ROS graph 订阅、在线采集或在线标定。
- 设备控制、自动设备时钟校准和曝光时间推断。
- OpenCV、FFmpeg、Slint 或 ROS 安装依赖。
- 根据解析失败自动切换 bag backend、时间来源、相机模型或求解方法。
- 求解失败后返回部分成功结果。
- 终端图像预览作为标定前提。首版可视化只覆盖数值、时间、检测和收敛状态。

## 3. 标定模式与数值合同

### 3.1 单 Cam 内参

输入一路相机的多姿态 AprilGrid 图像，求解显式选择的相机模型。至少五个有效视图，每个接受视图至少四个有效点；这些是形成有限候选所需的结构条件，不是结果质量阈值。

内参默认由应用按图像尺寸和用户显式选择的模型调用 corelib `CameraParameters::initial_for_image` 生成可行居中 seed；应用不猜模型，也不维护第二套模型公式。用户可提供已有结果覆盖自动 seed，应用按 camera ID、图像尺寸和模型读取参数。solver 在该初值上执行 LM 精化并报告停止原因、信息秩、条件数和协方差性质，不据此自动否决候选。
目标位姿由 corelib 平面 bearing PnP 初始化，不要求用户填写。

### 3.2 多 Cam 联合内外参

同一 shared view 内的多路图像必须由 header 时间同步规则构造，不以文件名、消息序号或 bag record time 推断。联合求解每路内参和：

```text
T_camera_i_from_reference_camera
```

硬要求：

- 相机 ID 和绑定 topic 唯一。
- 同一 shared view 中同一路相机最多一帧。
- 每路相机至少五个有效视图。
- 共同观测图连通。
- 目标是同一个静止、尺度已知的 AprilGrid。
- 图闭环初始化必须形成有限且连通的候选。

应用只选择 `RigInitialization::Planar`；默认内参 seed 由 corelib 按模型和图像尺寸生成，也可由调用方提供已有结果覆盖。共享板位姿和相机图外参由 corelib 初始化，再进入统一 LM 精化。达到 LM 迭代/阻尼边界或信息矩阵降秩时仍返回最后接受状态；人工验收使用残差、信息谱和跨数据稳定性作判断。若显式关闭 `refine_intrinsics`，则必须提供已有内参结果。

### 3.3 多 Cam—IMU 联合标定

输入严格递增的视觉 shared views、原始 IMU 样本和模型匹配的自动或显式内参初值。`method` 显式选择 `preintegration` 或 `continuous_time`；连续时间通过 `initialize_continuous_time` 直接初始化，不调用 `PlanarRigGyro` 预积分入口。

- 视觉阶段从自动 seed 或显式内参初值出发，由共享板位姿稳健初始化 `T_camera_i_from_reference_camera`，再运行统一 LM 并保留停止与信息诊断。
- 预积分路径的参考相机—IMU 平移和 bias 初值由应用显式传入单位位姿和零 bias。corelib 用 gyro hand-eye 对齐初始化旋转，并用逐区间 Householder QR 求解速度—重力线性系统，工作量和存储均为 O(N)。QR 主元阈值不等于旧 SVD 奇异值截断；近退化输入显式拒绝。
- `preintegration` 发布最后接受的有限候选，不按收敛原因、信息秩或条件数自动拒绝。
- `continuous_time` 精化视觉 rig 内参，利用受 IMU 支持的最近陀螺样本对齐旋转，并由最早受支持种子时刻的物理比力及同刻位姿初始化重力方向；参考相机—IMU 平移和 bias 初值为零。将参考相机 shared-view 位姿与其他精确曝光时刻的固定内参 PnP 位姿经同一 rig 外参转换为 `T_world_from_imu`，按配置间隔拟合覆盖 IMU、全部图像及完整时差边界并集的四次样条。平移采用带状基函数求解；旋转在当前预测附近用局部 SO(3) 修正，不使用相对首帧的全局对数，避免跨 π/2π 的伪运动。此初值仍须后续联合优化；初始比力也不保证仅含重力。
- 连续时间基线固定视觉阶段内参、配置时差初值和轴矩阵。若请求时差估计或轴矩阵精化，再以基线候选运行第二次联合优化，仅释放所请求的参数。初始化、非法输入域或无法线性分解等没有有限候选的错误仍整体失败；LM 预算停止和降秩不是失败回退。

时差方向固定为：

```text
t_imu_eval = t_camera + delta_t
```

IMU 输入合同：

- gyro 单位为 `rad/s`，在 IMU 坐标系表达。
- linear acceleration 按比力 `m/s²` 解释。
- header 时间严格递增，样本间隙不超过 `max_gap_s`。预积分要求 IMU 覆盖视觉时间与时差边界；连续时间仅要求初始化 gyro/重力所用时刻有 IMU 支持，样条覆盖全部精确图像时间及其时差边界与 IMU 的并集，不丢弃 IMU 区间外图像。
- 噪声密度和 bias random walk 来自设备资料或独立测量，不使用示例值冒充事实。连续时间入口按实际 IMU 中位采样间隔把 density 换成 per-sample sigma。
- 静态 IMU 模型遵循 Basalt 的 `physical = A * raw - bias`：gyro 完整 3×3（9 参数），accelerometer 正对角下三角（6 参数），加各 3 维常量 bias 共 21 参数。仅加速度计结构固定共同旋转规范，gyro 保留相对轴旋转；校正矩阵使用加性参数化。配置是初值，默认单位矩阵且不精化；预积分拒绝非单位矩阵或轴精化请求。噪声 sigma 定义在校正物理系内，仍为各向同性，不隐式变换 raw 协方差。

连续时间缺省流程：

1. corelib 直接标定视觉 rig；将配置校正矩阵直接乘原始 IMU（种子 bias 为零），用于旋转对齐和重力播种，后续因子按同一模型校正原始样本。
2. 保留全部视觉 rig 位姿，并以固定的 rig 内参和外参将逐相机精确时刻的 PnP 位姿转换为 `T_world_from_imu`；同时间优先采用联合 rig 位姿，PnP 失败显式返回。以带状平移拟合和局部 SO(3) 修正创建覆盖 IMU 与全部图像加时差边界并集的均匀四次 B 样条，不经过预积分。
3. 固定视觉阶段内参、时差初值和 IMU 轴矩阵，运行连续时间联合 LM；无论数值收敛或到达预算边界，都保留最后接受的有限状态及诊断。
4. 当请求时差估计或 `axis_calibration.refine: true` 时，用第 3 步候选播种第二次 LM，分别释放时差和/或 15 个 IMU 矩阵参数；6 个 bias 参数两阶段均参与优化，内参继续固定。
5. 最后一个请求阶段的候选写入结果并标记 `pending_human_review`；不按收敛、秩或条件数设置自动发布门槛，也不静默切换方法。

## 4. ROS bag 输入合同

### 4.1 支持的容器

应用只通过 `ccl_rosbag` 读取：

| 配置值 | 输入 |
|---|---|
| `ros1_v2` | ROS 1 bag v2 `.bag` |
| `ros2_mcap` | ROS 2 `.mcap` |
| `ros2_sqlite3` | 单个 `.db3` 或含 `metadata.yaml` 的 rosbag2 目录 |

传给 `ccl_rosbag::open` 的 `BagFormat` 必须明确。文件扩展名可以帮助 TUI 预选格式，但解析失败后不得尝试另一 backend。

动态解码仅接受当前 corelib 明确支持的 ROS1 `.msg` 和 ROS2 `ros2msg` plain CDR/XCDR1。缺 schema、ROS2 IDL、XCDR2 parameter-list 和未知 serialization 显式失败。旧 SQLite bag 可由预先登记的 `StaticSchemaRegistry` 提供 schema；应用不得从本机 ROS 安装或网络猜测定义。

### 4.2 相关消息类型

预加载只把下列标准类型列入标定候选：

- `sensor_msgs/Image`
- `sensor_msgs/Imu`

类型名先规范化 ROS1/ROS2 的 `/msg/` 差异。`CompressedImage`、私有图像消息和私有 IMU 消息首版不支持；不能仅因字段形似标准消息就自动接受。

当前 `ImageAdapter` 支持：

- `mono8` / `8UC1`
- little-endian `mono16` / `16UC1`
- `rgba8`

RGB/BGR、压缩图像、未知编码和大端 16-bit 不隐式转换。AprilTag 检测直接消费 corelib 图像视图；消息回调结束前完成检测，不长期保存 bag payload，也不建立全量图像缓存。

### 4.3 唯一时间来源

图像与 IMU 的标定时间一律来自消息 `header.stamp`：

- ROS1：投影 `header.stamp`，读取 `ValueRef::Time { sec, nanosec }`。
- ROS2：投影 `header.stamp.sec` 和 `header.stamp.nanosec`，读取 `i32 sec` 与 `u32 nanosec`。

转换必须先校验 `nanosec < 1_000_000_000`，再以 checked integer arithmetic 合成 `i64` 纳秒。不得先转成 Unix epoch `f64` 再相减。

以下情况 fail closed：

- header 或 stamp 字段缺失。
- schema 字段类型不匹配。
- 纳秒越界或合成溢出。
- 同一 topic 按 header 时间归一排序后存在重复时间戳。

容器访问顺序中的 header 逆序单独记为诊断；应用必须显式按 header 时间排序，不能把访问顺序当作传感器时间顺序。`RawMessage::record_time_ns` 与 `publish_time_ns` 可以写入诊断，用于发现录制延迟，但不用于帧率、同步、IMU 时间或回退。

所有参与求解的时间先选共同整数纳秒 origin，再调用：

```rust
SampleTime::from_nanoseconds(timestamp_ns, origin_ns)
```

### 4.4 预加载与话题统计

选择 bag 后，预加载 worker：

1. 打开容器并枚举 topic、类型、serialization 和 schema 状态。
2. 为候选 IMAGE/IMU topic 编译一次 `CompiledSchema` 和 header 时间投影。
3. 扫描候选消息，统计数量、首末 header 时间、时间逆序、重复、最小/平均/最大间隔和最大 gap。
4. 图像 topic 额外读取 encoding、宽高和 stride；同一 topic 变化时明确失败。
5. 计算帧率：

```text
rate_hz = (message_count - 1) × 1e9 / (last_header_ns - first_header_ns)
```

6. 展示 IMU 与各图像 topic 的 header 时间覆盖交集。

ROS2 SQLite 多分片访问顺序不冒充全局 header 时间顺序。应用按解析出的 header 时间建立索引或排序，并单独报告原始访问顺序异常。

### 4.5 shared-view 构造

配置为每个 camera ID 绑定唯一图像 topic，并提供同步容差。anchor 相机由标定模式唯一确定：

- 单 Cam：所选相机。
- 多 Cam：`reference_camera`。
- 多 Cam—IMU：`visual_reference_camera`。

对每个 anchor 帧：

1. 在每路其他相机中查找 header 时间差不超过 `max_delta_ns` 的唯一最近帧。
2. 一条图像消息最多属于一个 shared view。
3. 最近距离相同时视为歧义并失败，不任意选择。
4. `require_all_cameras: true` 时缺任一路原始曝光即丢弃整组；检测失败不视为曝光缺失，也不改配相邻帧。为 `false` 时保留部分组，但最终共同观测图仍须连通。
5. 输出 shared views 按 anchor header 时间严格递增。

同步容差是产品配置，不由实测帧率自动推断。record time、消息序号和数组位置均不参与配对。

连续时间残差不使用上述近同步分组：按每路原始 header 时间构造精确时间并集，仅同时间观测合并；未匹配图像也保留。近同步组用于视觉 rig、预积分及连续时间的视觉/gyro 初值。gyro 角速度仍来自参考相机时轴原相邻姿态对，只跳过起点不受 IMU 支持的对，不跨过缺失样本重新配对；重力选最早受支持种子时刻。样条则额外使用其他相机精确曝光时刻的 PnP 位姿，经 rig 外参变换到统一坐标系，不能将这条合并时间线用于 gyro 相邻配对。

### 4.6 IMU 投影

标准 `sensor_msgs/Imu` 使用预绑定动态投影读取：

```text
header.stamp
angular_velocity.x/y/z
linear_acceleration.x/y/z
```

所有六个数值必须有限。covariance 字段不自动修改求解器噪声；噪声配置仍是显式输入。适配结果直接构造 `ccl_calib::camera::inertial::ImuSample`，不定义长期 ROS 镜像 struct。

## 5. 运行设置

首版只保留两个用户可理解的概念：

1. **bag 信息**：预加载自动得到的 topic、类型、帧率、尺寸和时间统计。
2. **运行设置**：用户在 TUI 中选择的相机 topic、模型、显式模式、标定板和内参策略。

不要求用户预先维护完整的 `config/calibration.yaml`，也不引入 preset、配置继承或多层配置合并。TUI 直接维护一份 `RunSettings`；按下“开始标定”时执行一次完整校验，成功后把同一份设置写入本次运行目录的 `config.yaml`。worker 使用该不可变快照。

相机数量由用户选中的有效 IMAGE topic 数量决定，不单独填写。不能直接使用 bag 中全部 IMAGE topic 数量，因为其中可能包含预览流、矫正流或不参与标定的辅助相机。每个选中 topic 只需填写：

- 简短且唯一的 camera ID；默认可由 topic 名生成，用户可修改。
- 相机模型；使用已有内参时必须与文件中同 camera ID 的模型一致。

内参默认自动初始化并优化，可选文件初值；纯视觉多相机还可固定文件内参。IMU 模式选择一个 IMU topic，纯视觉不展示惯性表单。表单与专家 YAML 编辑器共用一份配置；启动时展开实际使用值，保证运行可审计。

本次运行快照采用单一结构，示意如下：

```yaml
source:
  path: data/calibration
  format: ros2_sqlite3
  synchronization:
    max_delta_ns: 2000000
    require_all_cameras: true

cameras:
  - id: cam0
    topic: /cam0/image_raw
    model: kannala-brandt4
  - id: cam1
    topic: /cam1/image_raw
    model: double-sphere

calibration:
  type: multi_camera
  reference_camera: cam0
  refine_intrinsics: true
  max_closure_rotation_deg: 2.0
  max_closure_translation_m: 0.05

target:
  type: aprilgrid
  family: tag36h11
  border_bits: 2
  tag_size_m: 0.088
  tag_gap_m: 0.0264
  id_rows_front_view_left_to_right:
    - [30, 31, 32, 33, 34, 35]
    - [24, 25, 26, 27, 28, 29]
    - [18, 19, 20, 21, 22, 23]
    - [12, 13, 14, 15, 16, 17]
    - [6, 7, 8, 9, 10, 11]
    - [0, 1, 2, 3, 4, 5]
  decoded_corner_to_front_corner: [2, 3, 0, 1]
```

惯性模式在同一结构中增加 IMU topic、`method`、可选内参初值、噪声、积分、连续时间 knot 间隔、轴误差矩阵和时差字段。无界面运行读取同一 `RunSettings` YAML，例如 `--no-tui --config <file>`。连续时间轴误差配置如下：

```yaml
method: continuous_time
imu:
  continuous_time:
    knot_interval_s: 0.01
  axis_calibration:
    refine: true
    gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    accelerometer_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
```

启动前只做必要校验：

- 至少选择一路 IMAGE topic，topic、camera ID 均唯一。
- 选中的 topic 类型、encoding 和尺寸受支持。
- reference camera 属于已选相机。
- 惯性模式选中一个有效 IMU topic，纯视觉模式不携带 IMU 设置。
- 内参初值文件的 camera ID、模型和尺寸一致。
- target、同步容差、时差和高级数值有限且合法。

标定板参数与 `pangbot_16p_arm_calib/config/calibration.yaml` 保持同一物理合同：正视图底行 ID 为 0..5、向上递增，标签边长 0.088 m、边到边间隙 0.0264 m，解码角点到正视物理角点的映射为 `[2, 3, 0, 1]`。显式 ID 行与角点映射禁止用连续 ID 假设替代。

运行设置不保存可重新发现的消息数量、帧率、图像尺寸或时间范围；这些事实写入 `input.yaml`。


## 6. Ratatui 交互设计

### 6.1 配置工作台

导航为数据源、传感器、标定板、求解设置、启动检查。配置草稿、不可变运行快照、任务状态和当前页面分离；修改配置不改变已完成记录。选 IMU 不隐式改变任务类型。

配置使用参数/值/说明表格；Sensors 使用每行一个相机的启用/ID/Topic/模型/消息数/Hz/参考矩阵和独立 IMU 表。方向键只选行或单元格，不改值；Enter 编辑/下拉，Space 勾选或展开组。Tab/Shift-Tab 切换行；Ctrl-P/N、导航与底部按钮切分区。Ctrl-←/→ 保留，但可能被系统桌面切换截获；输入框和 YAML 编辑器不触发切页。高级同步、检测器和求解参数分组折叠，窄屏说明放入当前行详情。扫描更新不抢页面或焦点，板 ID 排列精确保留。

专家 YAML 编辑器与表单映射到同一 RunSettings；无效文本保留但不应用，离开未保存编辑需要确认。启动检查区分阻断、警告、未检查；文件 ID/模型不匹配阻断，未解码的图像尺寸不得冒充检查通过。无阻断时一次启动。

### 6.2 运行监控

概览默认显示仪表盘：真实阶段/活动/计时、逐相机统计、完整检测历史和当前求解阶段 cost 曲线。宽屏双图并排，中等窗口上下排列，窄屏用 g 或按钮选择图表而非隐藏。不同求解阶段不连接 cost；非线性求解没有虚构总进度或 ETA。结束后保留冻结仪表盘，r/按钮进入结果，结果页 Esc 返回图表。运行配置只读。

Esc 只返回或关闭弹窗，不取消、不退出。F8/Ctrl-C 经确认请求协作取消；不可中断内核返回前保持等待状态。Ctrl-Q 经确认退出，先等待 worker 停止再恢复终端。

### 6.3 结果检查

概览、相机参数、外参、诊断、原始文件分别展示。执行完成、数值收敛、人工验收是不同状态；结果始终等待人工审查。显示 RMS/P95、有效帧/点、剔除统计、完整错误链与产物路径，不允许直接编辑求解结果。

F6 打开目录、F7 打开已有离线三维结果页、F9 修改配置、F10 相同配置重跑。再次运行创建独立记录，不覆盖旧结果。运行/结果的 Tab 或 Ctrl-P/N 切换只读页，长报告可滚动，Ctrl-Y 保留终端复制模式。


### 6.6 可视化与日志

结果 HTML 内嵌最终轨迹、参数和求解摘要，保留静态构型视图，增加相对时间滑块与 1× 播放、选中位姿 R/t 和四元数、相机内参、IMU 参数、逐相机 RMS/P95 与点数以及收敛/秩诊断。纯视觉轨迹为标定板系中的参考相机运动；联合轨迹为世界系中的 IMU 运动。连续时间使用最终样条在相机时间加偏移处求值，不展示控制节点为真实轨迹。采样间线性平移/SLERP 仅供显示，页面明确标注。trajectory.json、summary.json 与 calibration.yaml、HTML 原子发布；轨迹以 m、静态构型以 mm 显示，不做额外重力对齐，不将示意相机线框标为真实视场。

日志保留具体错误类别和完整 error chain，不统一改写为“标定失败”。

## 7. corelib 求解进度合同

`camera_corelib` 已通过 `SolveControl::joint_progress` 暴露纯观察型 `JointSolveProgress`：迭代序号与上限、当前 cost、阻尼、缩放梯度无穷范数以及接受/拒绝步数。单相机、纯视觉 rig 和相机—IMU 后端共用该合同；回调不参与算法决策，且每个 LM 外层迭代至多调用一次。

应用层在 `runner` 中把快照转成 `Event::SolveProgress`。无 TUI 模式直接输出结构化进度行；TUI 通过有界通道接收快照并绘制真实收敛曲线。取消仍由 `SolveControl::cancelled` 协作完成，不以线程终止替代。

`SolveControl::activity` 提供 `SolveActivity` / `SolvePhase`，覆盖初始化、运动方程组装与分解、LM 子步骤、信息矩阵及残差评价；有限工作单元有 `completed/total`，不透明分解不虚构百分比。通知在协调线程同步发出，循环进度节流而阶段边界立即发出，不是内核心跳。`cancellable=false` 表示当前内核不能立即中断；应用保持取消请求，直到 worker 真正返回才显示终态。

runner 转发为 `Event::SolveActivity`；TUI 在 worker 发送时记录 `Instant`，避免批量收消息造成伪造的零耗时。界面显示阶段/活动耗时、最近通知距今、当前迭代计时及最近已完成迭代耗时。切换阶段/求解轮次重置旧历史；结束后总耗时冻结，取消期间晚到的事件不覆盖取消状态。检测 ETA 继续仅使用可计数总帧数与平均处理速度。

## 8. 代码架构

保持现有单一 Rust package，按实际复杂度最小改动：

```text
src/
├── main.rs
├── cli.rs
├── config/mod.rs
├── input/mod.rs
├── detection.rs
├── runner.rs
├── export.rs
├── run_record.rs
└── tui.rs
```

`input/mod.rs` 实现输入适配。`tui/` 按配置草稿、字段、预检、任务事件、结果、绘制与文本编辑分离；不引入另一套求解器、配置默认值或通用 UI 框架。

边界保持简单：

- `config` 定义本次 `RunSettings`、默认值和启动校验。
- `input` 调用 `ccl_rosbag` 并输出规范化观测，不含几何算法。
- `detection` 只调用 `ccl_image` 并映射 Tag 角点。
- `runner` 是唯一作业编排和 corelib 求解请求构造处。
- `export` 只发布结果，不重新计算。
- `tui` 只维护页面、用户选择和 worker 事件。

不创建 `common.rs`、`utils.rs`、`services/`、缓存框架或内部消息总线。

## 9. 依赖策略

开发阶段使用同一相邻源码树：

```toml
ccl_calib = { path = "../camera_corelib/crates/ccl_calib" }
ccl_geometry = { path = "../camera_corelib/crates/ccl_geometry" }
ccl_image = { path = "../camera_corelib/crates/ccl_image" }
ccl_rosbag = { path = "../camera_corelib/crates/ccl_rosbag", features = ["ccl-adapters"] }
```

约束：

- 四个 crate 必须全部使用 path，禁止与 Git 版本混用。
- `run.yaml` 记录 `corelib_source: path:../camera_corelib`，不冒充固定 commit。
- Cargo.lock 只锁定依赖解析，不声称能复现未提交的本地 corelib 源码。
- 接口稳定后，四个 crate 在同一改动中切换到同一个不可变 Git revision，并同步更新运行元数据和 README。

当前只保留离线壳所需的 `ratatui`、`crossterm`、`image`、`serde*`、`clap` 和运行记录依赖；`image` 负责 bag 图像 payload 解码。OpenCV、FFmpeg、Slint 及旧求解 crate 已从 manifest 和 lockfile 删除。

不引入异步运行时。一个主线程负责 TUI，一个 worker 执行预加载或标定；检测使用按 `available_parallelism` 定长创建的有界 worker pool，任务队列容量为 worker 数的两倍。

## 10. 运行记录与结果

应用启动创建独占 unconfigured 记录，实际标定启动时改为模式目录；同秒重复运行使用独立序号：

```text
runs/<YYYYMMDD-HHMMSS>-<mode>/
├── run.yaml
├── run.log
├── config.yaml
├── input.yaml
├── detection.yaml
├── detection_progress.jsonl
├── solve.yaml
└── result/
    └── calibration.yaml
```

再次运行通过 `RunContext::next_run` 创建独立同级记录。本次实际配置在启动时快照，旧运行的配置、日志和结果不被改写。

- `run.yaml`：状态、起止时间、模式、应用版本、corelib 来源、错误类别和阶段耗时。
- `input.yaml`：bag 路径、格式、topic 配对、header 时间统计、shared-view 和 IMU 摘要；不复制 payload。
- `run.log`：解析、检测、分组、初始化、求解和错误诊断。
- `result/calibration.yaml`：保存有限标定候选，顶层验收状态默认为 `pending_human_review`。

结果文件允许：

- schema 与标定模式。
- 人工验收状态；应用生成时只能标记为待验收，不自动声称接受。
- 相机 ID、图像尺寸、模型和完整内参。
- 多 Cam 的参考相机和 `T_camera_i_from_reference_camera`。
- 多 Cam—IMU 的 `T_camera_i_from_imu`、bias、重力和时差。
- 明确位姿方向、平移单位和四元数顺序。

结果文件禁止：

- RMS、p95、逐点误差、拒点数。
- cost、迭代数、阻尼、信息秩、条件数和协方差。
- bag 路径、阶段耗时、日志或错误。
- 每帧目标位姿和视觉惯性中间状态。

发布过程：内存校验 DTO → 写 `.result.tmp/` → sync → 原子 rename 为 `result/`。初始化、输入、数值分解失败或取消不得留下貌似成功的 `result/`；LM 未满足收敛判据或信息降秩仍是可审查候选，不归入失败。

## 11. 验证与验收

### 11.1 运行设置

- 三种模式的最小运行设置均可启动。
- 选中 topic、camera ID 和 reference camera 完整且唯一。
- 模型、目标和内参初值严格匹配。
- 非有限值、非法范围、无效时差边界和预算被拒绝。

### 11.2 bag 与时间合同

- ROS1 bag、ROS2 MCAP、ROS2 SQLite 文件和目录各有真实读取 smoke。
- IMAGE/IMU topic 发现、schema 缺失和不支持 encoding 有明确结果。
- ROS1 与 ROS2 header 时间得到相同整数纳秒语义。
- header 缺失、字段类型错误、纳秒越界、重复和逆序明确失败。
- record/publish time 与 header 不同的 fixture 证明标定只使用 header。
- 多分片 SQLite 按 header 时间归一，不依赖分片访问顺序。
- 帧率、间隔、最大 gap 和覆盖统计有可核算 fixture。

### 11.3 shared views 与 IMU

- 容差边界内外、最近匹配、等距歧义和消息不可复用均有测试。
- `require_all_cameras` 两种策略行为明确。
- 共同观测图断开时失败。
- IMU 字段、有限值、严格时间、gap 和 coverage 错误可定位到 topic 和消息时间。
- 时间 origin 在整数域选取，转换后保持亚毫秒差值精度。

### 11.4 模式 smoke

- 单 Cam：真实 bag 中的 AprilGrid 图像完成检测、求解和导出。
- 多 Cam：真实同步 bag 完成联合内外参并输出连通外参。
- 多 Cam—IMU：真实 bag 完成预积分联合求解并输出外参、bias、重力和时差。
- 各模式分别覆盖取消、不可观测、未收敛和损坏输入。

算法数值精度由 corelib 测试负责；本项目只测试适配、方向、单位、模式选择、状态转移和输出合同。

### 11.5 Ratatui smoke

- 不依赖预先存在的配置文件即可进入数据源页。
- 路径输入、格式选择和预加载期间界面保持响应。
- topic 表显示 header 帧率和错误状态。
- 运行设置不完整时阻止启动，并在对应位置显示原因。
- 标定期间可取消，终端始终完整恢复。
- 收敛曲线只使用 corelib 进度快照，展示的 cost、阻尼、梯度和步数可与无 TUI 输出逐项核对。
- 成功、失败、取消状态和结果路径清晰区分。
- `--no-tui --config <file>` 使用同一 `RunSettings`、输入 adapter 和 runner。

## 12. 实施状态

1. **已完成：应用精简**——单 package、移除在线路径、求解统一调用 corelib、保留薄 Ratatui 壳和原子结果合同。
2. **已完成：运行设置与 bag 输入**——单一 `RunSettings` 覆盖三种模式；接入容器选择、topic 扫描、header 时间、统计、流式检测、shared-view 分组和 IMU 适配。
3. **已完成：TUI 与批处理**——数据源、运行设置、运行、结果和日志页面共用同一个 runner；`--no-tui` 不维护第二套流程。
4. **已完成：运行记录与导出**——启动时创建 run、快照设置和输入事实，三种 corelib 求解均使用原子导出。
5. **已完成：真实进度**——corelib 观察回调已经接入 CLI 和 TUI，不显示推测进度。
6. **已完成：清理**——旧 manifest、在线输入、GUI、内部求解器、Python 桥、发布脚本和无调用依赖均已删除。

四个 `ccl_*` crate 当前故意统一使用相邻 path 来源；在 corelib 发布不可变 revision 前不伪造可复现版本。每次改动只保留可运行路径，禁止占位实现、自动成功回退、两套并行输入合同或无法被实际数据路径触发的 UI。

