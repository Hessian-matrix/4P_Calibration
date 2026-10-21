use std::path::Path;

use crate::config::{CalibrationConfig, RunSettings, TargetConfig};
use crate::export::load_result;
use crate::input::{BagInspection, TopicInspection};

use super::draft::{CalibrationMode, Draft, IntrinsicStrategy, TimeOffsetKind};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckLevel {
    Blocker,
    Warning,
    Ok,
    NotChecked,
}

impl CheckLevel {
    #[must_use]
    pub const fn badge(self) -> &'static str {
        match self {
            Self::Blocker => "X",
            Self::Warning => "!",
            Self::Ok => "OK",
            Self::NotChecked => "?",
        }
    }
}

#[derive(Clone, Debug)]
pub struct PreflightCheck {
    pub level: CheckLevel,
    pub title: String,
    pub detail: String,
}

#[derive(Clone, Debug)]
pub struct Preflight {
    pub checks: Vec<PreflightCheck>,
    pub settings: Option<RunSettings>,
}

impl Preflight {
    #[must_use]
    pub fn evaluate(draft: &Draft, inspection: Option<&BagInspection>, run_root: &Path) -> Self {
        let mut checks = Vec::new();
        let settings = match draft.to_settings() {
            Ok(settings) => {
                checks.push(ok("配置字段", "所有字段均可解析，内参策略与当前模式兼容"));
                Some(settings)
            }
            Err(error) => {
                checks.push(blocker("配置字段", format!("{error:#}")));
                None
            }
        };

        if let Some(settings) = &settings {
            match settings.validate() {
                Ok(()) => checks.push(ok(
                    "配置结构",
                    "模式、相机、同步、标定板、检测器、求解器数值均通过校验",
                )),
                Err(error) => checks.push(blocker("配置结构", format!("{error:#}"))),
            }
            check_source(settings, inspection, &mut checks);
            check_sensors(draft, settings, inspection, &mut checks);
            check_intrinsics(draft, settings, &mut checks);
            check_board(settings, &mut checks);
            check_imu(draft, settings, &mut checks);
        }

        if run_root.is_dir() {
            checks.push(ok(
                "运行记录目录",
                format!("{} 已存在；新运行将创建独立同级目录", run_root.display()),
            ));
        } else {
            checks.push(blocker(
                "运行记录目录",
                format!("{} 不存在；无法保存本次运行记录", run_root.display()),
            ));
        }

        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "bag 内容".to_owned(),
            detail: "未完整解码消息；图像编码、schema 兼容性、时间戳一致性与坏帧只在正式运行时暴露"
                .to_owned(),
        });
        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "可观测性与精度".to_owned(),
            detail: "预检不执行数值求解，也不判断视角覆盖、IMU 激励或最终精度；结果必须人工审查"
                .to_owned(),
        });
        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "磁盘空间与权限".to_owned(),
            detail: "仅确认运行目录存在；剩余空间、写权限和外部进程占用未检查".to_owned(),
        });

        Self { checks, settings }
    }

    #[must_use]
    pub fn blockers(&self) -> usize {
        self.checks
            .iter()
            .filter(|check| check.level == CheckLevel::Blocker)
            .count()
    }

    #[must_use]
    pub fn warnings(&self) -> usize {
        self.checks
            .iter()
            .filter(|check| check.level == CheckLevel::Warning)
            .count()
    }

    #[must_use]
    pub fn not_checked(&self) -> usize {
        self.checks
            .iter()
            .filter(|check| check.level == CheckLevel::NotChecked)
            .count()
    }

    #[must_use]
    pub fn can_start(&self) -> bool {
        self.blockers() == 0 && self.settings.is_some()
    }
}

fn check_source(
    settings: &RunSettings,
    inspection: Option<&BagInspection>,
    checks: &mut Vec<PreflightCheck>,
) {
    let path = &settings.source.path;
    if path.as_os_str().is_empty() {
        checks.push(blocker("数据源", "bag 路径为空"));
    } else if path.is_file() || path.is_dir() {
        checks.push(ok(
            "数据源",
            format!(
                "{} 存在（{}；文件或 ROS2 bag 目录均由核心库打开）",
                path.display(),
                super::draft::bag_format_label(settings.source.format)
            ),
        ));
    } else {
        checks.push(blocker(
            "数据源",
            format!("{} 不存在或不可访问", path.display()),
        ));
    }
    if !settings.source.synchronization.require_all_cameras {
        checks.push(warning(
            "同步策略",
            "require_all_cameras=false：缺失部分相机的视图会被保留，共享视图可能减少",
        ));
    }
    if let Some(inspection) = inspection {
        if inspection.source == settings.source.path {
            checks.push(ok(
                "扫描结果",
                format!(
                    "已读取 {} 个 topic 的元数据；路径与配置一致",
                    inspection.topics.len()
                ),
            ));
        } else {
            checks.push(warning(
                "扫描结果",
                format!(
                    "上次扫描的是 {}；当前路径已变化，请在数据源页重新扫描",
                    inspection.source.display()
                ),
            ));
        }
    } else {
        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "扫描结果".to_owned(),
            detail: "未扫描 bag；topic 存在性、类型、消息数与时间戳未检查".to_owned(),
        });
    }
}

#[allow(
    clippy::too_many_lines,
    reason = "sensor checks are intentionally reported individually before start"
)]
fn check_sensors(
    draft: &Draft,
    settings: &RunSettings,
    inspection: Option<&BagInspection>,
    checks: &mut Vec<PreflightCheck>,
) {
    let count = settings.cameras.len();
    let minimum = draft.mode.minimum_cameras();
    if count < minimum {
        checks.push(blocker(
            "相机数量",
            format!(
                "{} 模式至少需要 {minimum} 个相机，当前 {count} 个",
                draft.mode.label()
            ),
        ));
    } else if matches!(draft.mode, CalibrationMode::SingleCamera) && count != 1 {
        checks.push(blocker(
            "相机数量",
            format!("单相机模式要求恰好 1 个相机，当前 {count} 个"),
        ));
    } else {
        checks.push(ok(
            "相机数量",
            format!("{} 个相机，满足 {} 模式", count, draft.mode.label()),
        ));
    }

    if settings
        .cameras
        .iter()
        .all(|camera| camera.id != draft.reference_id)
    {
        checks.push(blocker(
            "参考相机",
            format!("参考相机 {} 不在已选相机中", draft.reference_id),
        ));
    } else {
        checks.push(ok("参考相机", draft.reference_label()));
    }

    let mut ids = std::collections::HashSet::new();
    let mut topics = std::collections::HashSet::new();
    for camera in &settings.cameras {
        if !ids.insert(camera.id.as_str()) {
            checks.push(blocker("相机 ID", format!("重复相机 ID {}", camera.id)));
        }
        if !topics.insert(camera.topic.as_str()) {
            checks.push(blocker(
                "相机 topic",
                format!("重复相机 topic {}", camera.topic),
            ));
        }
    }

    if draft.mode.uses_imu() {
        match settings.source.imu_topic.as_deref() {
            Some(topic) if !topic.trim().is_empty() => {
                checks.push(ok("IMU topic", topic.to_owned()));
            }
            _ => checks.push(blocker(
                "IMU topic",
                "相机-IMU 模式必须选择一个 IMU topic（模式不会因勾选 IMU 自动切换）",
            )),
        }
    } else if let Some(topic) = &draft.imu_topic {
        checks.push(warning(
            "IMU topic",
            format!(
                "已选择 {topic}，但当前模式 {} 不使用 IMU；不会自动切换模式，可在传感器页显式切换",
                draft.mode.label()
            ),
        ));
    }

    if let Some(inspection) = inspection {
        for camera in &settings.cameras {
            match find_topic(inspection, &camera.topic) {
                Some(topic) if is_image_topic(topic) => {
                    if topic.message_count == 0 {
                        checks.push(warning(
                            "相机 topic",
                            format!("{} 存在但消息数为 0", camera.topic),
                        ));
                    }
                    if !topic.header_time_valid {
                        checks.push(warning(
                            "相机时间戳",
                            format!("{} 的 header 时间戳无效", camera.topic),
                        ));
                    }
                }
                Some(topic) => checks.push(blocker(
                    "相机 topic",
                    format!(
                        "{} 的类型是 {}，不是 sensor_msgs/Image 或 CompressedImage",
                        camera.topic, topic.type_name
                    ),
                )),
                None => checks.push(blocker(
                    "相机 topic",
                    format!("扫描结果中不存在 {}", camera.topic),
                )),
            }
        }
        if draft.mode.uses_imu()
            && let Some(imu_topic) = settings.source.imu_topic.as_deref()
        {
            match find_topic(inspection, imu_topic) {
                Some(topic) if topic.type_name == "sensor_msgs/Imu" => {
                    if topic.message_count == 0 {
                        checks.push(warning("IMU topic", format!("{imu_topic} 消息数为 0")));
                    }
                }
                Some(topic) => checks.push(blocker(
                    "IMU topic",
                    format!(
                        "{imu_topic} 的类型是 {}，不是 sensor_msgs/Imu",
                        topic.type_name
                    ),
                )),
                None => checks.push(blocker(
                    "IMU topic",
                    format!("扫描结果中不存在 {imu_topic}"),
                )),
            }
        }
    } else {
        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "topic 可用性".to_owned(),
            detail: "未扫描 bag：所选相机与 IMU topic 是否存在、类型/消息数/时间戳未检查"
                .to_owned(),
        });
    }
}

fn check_intrinsics(draft: &Draft, settings: &RunSettings, checks: &mut Vec<PreflightCheck>) {
    let strategy = draft.intrinsic.strategy;
    if !strategy.supported(draft.mode) {
        checks.push(blocker(
            "内参策略",
            format!(
                "{} 不支持“{}”；可选 {}",
                draft.mode.label(),
                strategy.label(),
                draft
                    .strategy_options()
                    .iter()
                    .map(|option| option.label())
                    .collect::<Vec<_>>()
                    .join(" / ")
            ),
        ));
        return;
    }
    let initial = initial_intrinsics(settings);
    match strategy {
        IntrinsicStrategy::AutoOptimize => {
            checks.push(ok(
                "内参策略",
                "自动初值；内参在联合求解中优化（当前配置无初值文件）",
            ));
        }
        IntrinsicStrategy::FileOptimize | IntrinsicStrategy::FileFixed => {
            let Some(path) = initial.path.as_ref() else {
                checks.push(blocker("内参策略", "文件策略缺少内参初值文件路径"));
                return;
            };
            let fixed = matches!(strategy, IntrinsicStrategy::FileFixed);
            if path.is_file() {
                checks.push(ok(
                    "内参策略",
                    format!(
                        "{}（{}）",
                        path.display(),
                        if fixed {
                            "内参固定，不优化"
                        } else {
                            "内参在求解中优化"
                        }
                    ),
                ));
                match load_result(path) {
                    Ok(prior) => {
                        let mut problems = Vec::new();
                        for camera in &settings.cameras {
                            match prior.camera(&camera.id) {
                                Ok(previous) => {
                                    if previous.parameters.model() != camera.model.core() {
                                        problems.push(format!(
                                            "{} 模型不匹配：文件 {:?}，配置 {:?}",
                                            camera.id,
                                            previous.parameters.model(),
                                            camera.model.core()
                                        ));
                                    }
                                }
                                Err(_) => {
                                    problems.push(format!("文件缺少配置中的相机 ID {}", camera.id));
                                }
                            }
                        }
                        if problems.is_empty() {
                            checks.push(ok(
                                "内参文件 ID/模型",
                                "文件 schema 有效；配置中的相机 ID 均存在且模型一致",
                            ));
                        } else {
                            checks.push(blocker("内参文件 ID/模型", problems.join("；")));
                        }
                        checks.push(PreflightCheck {
                            level: CheckLevel::NotChecked,
                            title: "内参图像尺寸".to_owned(),
                            detail: "预检未解码图像，无法取得可用图像尺寸；正式读取首帧时按文件尺寸与图像尺寸阻断不匹配。"
                                .to_owned(),
                        });
                    }
                    Err(error) => checks.push(blocker(
                        "内参文件",
                        format!("无法读取/校验 {}: {error:#}", path.display()),
                    )),
                }
            } else if path.exists() {
                checks.push(blocker(
                    "内参策略",
                    format!("{} 是目录，不是内参结果文件", path.display()),
                ));
            } else {
                checks.push(blocker(
                    "内参策略",
                    format!("内参初值文件 {} 不存在", path.display()),
                ));
            }
        }
    }
}

fn check_board(settings: &RunSettings, checks: &mut Vec<PreflightCheck>) {
    let TargetConfig::Aprilgrid {
        family,
        border_bits,
        tag_size_m,
        tag_gap_m,
        id_rows_front_view_left_to_right,
        decoded_corner_to_front_corner,
    } = &settings.target;
    let columns = id_rows_front_view_left_to_right.first().map_or(0, Vec::len);
    let tags = id_rows_front_view_left_to_right
        .iter()
        .map(Vec::len)
        .sum::<usize>();
    if id_rows_front_view_left_to_right
        .iter()
        .all(|row| row.len() == columns)
        && columns > 0
    {
        checks.push(ok(
            "标定板几何",
            format!(
                "{}；{} 行 × {} 列；tag={tag_size_m} m，gap={tag_gap_m} m，border={border_bits} bits",
                super::draft::tag_family_label(*family),
                id_rows_front_view_left_to_right.len(),
                columns
            ),
        ));
        checks.push(ok(
            "tag ID 布局",
            format!("{tags} 个 tag，ID 取自当前行列表，未按尺寸重新生成"),
        ));
    } else {
        checks.push(blocker(
            "标定板几何",
            "ID 行必须为非空矩形；当前行列表为空或各行长度不一致",
        ));
    }
    let first = decoded_corner_to_front_corner[0];
    if first < 4
        && decoded_corner_to_front_corner
            .iter()
            .enumerate()
            .all(|(index, &corner)| corner == (first + index) % 4)
    {
        checks.push(ok(
            "角点映射",
            format!(
                "decoded→front 为 TL/TR/BR/BL 循环排列 [{}]",
                decoded_corner_to_front_corner
                    .iter()
                    .map(ToString::to_string)
                    .collect::<Vec<_>>()
                    .join(", ")
            ),
        ));
    } else {
        checks.push(blocker(
            "角点映射",
            "decoded_corner_to_front_corner 必须是 [k, k+1, k+2, k+3] mod 4 的循环排列",
        ));
    }
}

fn check_imu(draft: &Draft, settings: &RunSettings, checks: &mut Vec<PreflightCheck>) {
    if !draft.mode.uses_imu() {
        return;
    }
    let CalibrationConfig::MultiCameraImu {
        method,
        time_offset,
        imu,
        ..
    } = &settings.calibration
    else {
        return;
    };
    checks.push(ok(
        "惯性方法",
        super::draft::inertial_method_label(*method).to_owned(),
    ));
    match time_offset {
        crate::config::TimeOffsetConfig::Fixed { value_s } => checks.push(ok(
            "时间偏移",
            format!("固定 {value_s} s"),
        )),
        crate::config::TimeOffsetConfig::Estimate {
            initial_s,
            minimum_s,
            maximum_s,
            initialization_steps,
        } => checks.push(ok(
            "时间偏移",
            format!(
                "估计：初值 {initial_s} s，范围 [{minimum_s}, {maximum_s}] s，{initialization_steps} 步"
            ),
        )),
    }
    checks.push(ok(
        "IMU 参数",
        format!(
            "g={} m/s²；gyro σ={}；accel σ={}；max_gap={} s；max_samples={}；knot={} s",
            imu.gravity_magnitude_m_s2,
            imu.noise.gyro_density_rad_s_sqrt_hz,
            imu.noise.accelerometer_density_m_s2_sqrt_hz,
            imu.integration.max_gap_s,
            imu.integration.max_samples,
            imu.continuous_time.knot_interval_s
        ),
    ));
    let huber_px = settings.solver.huber_delta * settings.solver.pixel_sigma;
    checks.push(ok(
        "像素鲁棒策略",
        match settings.solver.point_rejection_px {
            Some(px) => format!("Huber {huber_px} px + 像素阈值 {px} px 截断"),
            None => format!("Huber {huber_px} px，仅鲁棒权重（不截断；Basalt 对齐）"),
        },
    ));
    if matches!(*method, crate::config::InertialMethod::Preintegration) {
        if imu.axis_calibration.refine || !draft.imu.axes_are_identity() {
            checks.push(blocker(
                "IMU 轴标定",
                "预积分要求轴矩阵为单位矩阵且 refine=false；轴标定只用于连续时间",
            ));
        } else {
            checks.push(ok("IMU 轴标定", "预积分使用单位轴矩阵，refine=false"));
        }
    } else {
        checks.push(ok(
            "IMU 轴标定",
            if imu.axis_calibration.refine {
                "连续时间：第二遍优化 IMU 轴矩阵"
            } else {
                "连续时间：固定轴矩阵"
            },
        ));
        let estimates_time = matches!(
            time_offset,
            crate::config::TimeOffsetConfig::Estimate { .. }
        );
        if imu.axis_calibration.refine || estimates_time {
            let parameters = match (imu.axis_calibration.refine, estimates_time) {
                (true, true) => "IMU 轴矩阵 + 有界时差",
                (true, false) => "IMU 轴矩阵（时差按配置保持固定）",
                (false, true) => "有界时差（轴矩阵保持固定）",
                (false, false) => unreachable!(),
            };
            checks.push(ok("连续时间第二遍", parameters));
        }
    }
    if draft.time_offset.kind == TimeOffsetKind::Estimate {
        checks.push(PreflightCheck {
            level: CheckLevel::NotChecked,
            title: "时间偏移可观测性".to_owned(),
            detail: "只检查范围与步数；时间偏移是否可观测取决于实际运动激励".to_owned(),
        });
    }
}

fn find_topic<'a>(inspection: &'a BagInspection, name: &str) -> Option<&'a TopicInspection> {
    inspection.topics.iter().find(|topic| topic.name == name)
}

fn is_image_topic(topic: &TopicInspection) -> bool {
    matches!(
        topic.type_name.as_str(),
        "sensor_msgs/Image" | "sensor_msgs/CompressedImage"
    )
}

fn initial_intrinsics(settings: &RunSettings) -> &crate::config::InitialIntrinsicsConfig {
    match &settings.calibration {
        CalibrationConfig::SingleCamera {
            initial_intrinsics, ..
        }
        | CalibrationConfig::MultiCamera {
            initial_intrinsics, ..
        }
        | CalibrationConfig::MultiCameraImu {
            initial_intrinsics, ..
        } => initial_intrinsics,
    }
}

fn ok(title: &str, detail: impl Into<String>) -> PreflightCheck {
    PreflightCheck {
        level: CheckLevel::Ok,
        title: title.to_owned(),
        detail: detail.into(),
    }
}

fn warning(title: &str, detail: impl Into<String>) -> PreflightCheck {
    PreflightCheck {
        level: CheckLevel::Warning,
        title: title.to_owned(),
        detail: detail.into(),
    }
}

fn blocker(title: &str, detail: impl Into<String>) -> PreflightCheck {
    PreflightCheck {
        level: CheckLevel::Blocker,
        title: title.to_owned(),
        detail: detail.into(),
    }
}

/// Used by the form to show a short, non-blocking note near the IMU topic
/// selector. Kept here so preflight and form agree on the exact meaning.
#[must_use]
pub fn imu_selection_note(draft: &Draft) -> String {
    match (draft.mode.uses_imu(), draft.imu_topic.as_deref()) {
        (true, Some(topic)) => format!("模式相机-IMU；IMU topic = {topic}"),
        (true, None) => "模式相机-IMU；必须选择 IMU topic".to_owned(),
        (false, Some(topic)) => {
            format!("已选 {topic}，但当前模式不使用；不会自动切换模式")
        }
        (false, None) => "IMU topic 仅用于相机-IMU 模式；选择不会改变模式".to_owned(),
    }
}
