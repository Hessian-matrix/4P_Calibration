use std::collections::HashSet;
use std::path::PathBuf;

use anyhow::{Result, bail};

use crate::config::{
    BagFormatConfig, BagSourceConfig, CalibrationConfig, CameraConfig, CameraModelConfig,
    CameraTimingConfig, ContinuousTimeConfig, DetectorBudgets, DetectorConfig,
    ImuAxisCalibrationConfig, ImuConfig, ImuNoiseConfig, InertialMethod, InitialIntrinsicsConfig,
    IntegrationConfig, IntegrationMethod, RefinementConfig, RefinementMethodConfig, RunSettings,
    SolverConfig, SynchronizationConfig, TagFamilyConfig, TargetConfig, TimeOffsetConfig,
};

const DEFAULT_MAX_DELTA_NS: i64 = 2_000_000;
const DEFAULT_CLOSURE_ROTATION_DEG: f64 = 2.0;
const DEFAULT_CLOSURE_TRANSLATION_M: f64 = 0.05;
const DEFAULT_TAG_SIZE_M: f64 = 0.088;
const DEFAULT_TAG_GAP_M: f64 = 0.0264;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CalibrationMode {
    SingleCamera,
    MultiCamera,
    MultiCameraImu,
}

impl CalibrationMode {
    pub const ALL: [Self; 3] = [Self::SingleCamera, Self::MultiCamera, Self::MultiCameraImu];

    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::SingleCamera => "single_camera",
            Self::MultiCamera => "multi_camera",
            Self::MultiCameraImu => "multi_camera_imu",
        }
    }

    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::SingleCamera => "单相机内参",
            Self::MultiCamera => "多相机视觉",
            Self::MultiCameraImu => "相机-IMU",
        }
    }

    #[must_use]
    pub const fn minimum_cameras(self) -> usize {
        match self {
            Self::SingleCamera => 1,
            Self::MultiCamera | Self::MultiCameraImu => 2,
        }
    }

    #[must_use]
    pub const fn uses_imu(self) -> bool {
        matches!(self, Self::MultiCameraImu)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntrinsicStrategy {
    AutoOptimize,
    FileOptimize,
    FileFixed,
}

impl IntrinsicStrategy {
    pub const ALL: [Self; 3] = [Self::AutoOptimize, Self::FileOptimize, Self::FileFixed];

    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::AutoOptimize => "自动初值 + 优化内参",
            Self::FileOptimize => "文件初值 + 优化内参",
            Self::FileFixed => "文件初值 + 固定内参",
        }
    }

    #[must_use]
    pub const fn short_label(self) -> &'static str {
        match self {
            Self::AutoOptimize => "自动+优化",
            Self::FileOptimize => "文件+优化",
            Self::FileFixed => "文件+固定",
        }
    }

    #[must_use]
    pub const fn supported(self, mode: CalibrationMode) -> bool {
        match self {
            Self::AutoOptimize | Self::FileOptimize => true,
            Self::FileFixed => matches!(mode, CalibrationMode::MultiCamera),
        }
    }
}

#[derive(Clone, Debug)]
pub struct IntrinsicDraft {
    pub strategy: IntrinsicStrategy,
    pub path: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TimeOffsetKind {
    Fixed,
    Estimate,
}

#[derive(Clone, Debug)]
pub struct TimeOffsetDraft {
    pub kind: TimeOffsetKind,
    pub fixed_s: String,
    pub initial_s: String,
    pub minimum_s: String,
    pub maximum_s: String,
    pub initialization_steps: String,
}

impl Default for TimeOffsetDraft {
    fn default() -> Self {
        Self {
            kind: TimeOffsetKind::Fixed,
            fixed_s: "0".to_owned(),
            initial_s: "0".to_owned(),
            minimum_s: "-0.1".to_owned(),
            maximum_s: "0.1".to_owned(),
            initialization_steps: "11".to_owned(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct ImuDraft {
    pub gravity_magnitude_m_s2: String,
    pub gyro_density_rad_s_sqrt_hz: String,
    pub accelerometer_density_m_s2_sqrt_hz: String,
    pub gyro_bias_walk_rad_s2_sqrt_hz: String,
    pub accelerometer_bias_walk_m_s3_sqrt_hz: String,
    pub integration_method: IntegrationMethod,
    pub max_gap_s: String,
    pub max_samples: String,
    pub refine_axes: bool,
    pub gyro_calibrated_from_raw: String,
    pub accelerometer_calibrated_from_raw: String,
    pub knot_interval_s: String,
    pub refine_camera_time_offsets: bool,
    pub camera_time_offset_minimum_s: String,
    pub camera_time_offset_maximum_s: String,
    pub refine_global_clock_skew: bool,
    pub refine_camera_clock_skews: bool,
    pub clock_skew_minimum_ppm: String,
    pub clock_skew_maximum_ppm: String,
    pub visual_motion_anisotropy_enabled: bool,
    pub visual_motion_anisotropy_ratio_per_motion_squared: String,
}

impl Default for ImuDraft {
    fn default() -> Self {
        Self {
            gravity_magnitude_m_s2: "9.80665".to_owned(),
            gyro_density_rad_s_sqrt_hz: "0.001".to_owned(),
            accelerometer_density_m_s2_sqrt_hz: "0.01".to_owned(),
            gyro_bias_walk_rad_s2_sqrt_hz: "0.00001".to_owned(),
            accelerometer_bias_walk_m_s3_sqrt_hz: "0.0001".to_owned(),
            integration_method: IntegrationMethod::Midpoint,
            max_gap_s: "0.02".to_owned(),
            max_samples: "2000000".to_owned(),
            refine_axes: false,
            gyro_calibrated_from_raw: format_matrix(&IDENTITY_MATRIX),
            accelerometer_calibrated_from_raw: format_matrix(&IDENTITY_MATRIX),
            knot_interval_s: "0.01".to_owned(),
            refine_camera_time_offsets: false,
            camera_time_offset_minimum_s: "-0.005".to_owned(),
            camera_time_offset_maximum_s: "0.005".to_owned(),
            refine_global_clock_skew: false,
            refine_camera_clock_skews: false,
            clock_skew_minimum_ppm: "-200".to_owned(),
            clock_skew_maximum_ppm: "200".to_owned(),
            visual_motion_anisotropy_enabled: false,
            visual_motion_anisotropy_ratio_per_motion_squared: "0.015".to_owned(),
        }
    }
}

const IDENTITY_MATRIX: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Exact `AprilGrid` geometry. Distances are stored in metres, matching the
/// configuration schema; the form labels the unit and never rescales values.
#[derive(Clone, Debug)]
pub struct TargetDraft {
    pub family: TagFamilyConfig,
    pub border_bits: String,
    pub tag_size_m: String,
    pub tag_gap_m: String,
    /// Exact row/ID layout. Rows are separated by `;`, IDs by commas or spaces.
    /// It is never regenerated from row/column counts.
    pub id_rows_text: String,
    /// Exact decoded-corner-to-front-corner mapping, in TL/TR/BR/BL order.
    pub corner_mapping_text: String,
}

impl TargetDraft {
    #[must_use]
    pub fn from_target(target: &TargetConfig) -> Self {
        let TargetConfig::Aprilgrid {
            family,
            border_bits,
            tag_size_m,
            tag_gap_m,
            id_rows_front_view_left_to_right,
            decoded_corner_to_front_corner,
        } = target;
        Self {
            family: *family,
            border_bits: border_bits.to_string(),
            tag_size_m: format_number(*tag_size_m),
            tag_gap_m: format_number(*tag_gap_m),
            id_rows_text: format_rows(id_rows_front_view_left_to_right.as_slice()),
            corner_mapping_text: decoded_corner_to_front_corner
                .iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", "),
        }
    }

    #[must_use]
    pub fn grid_summary(&self) -> String {
        let mut ignored_errors = Vec::new();
        let rows = parse_rows(&self.id_rows_text, "target.id_rows", &mut ignored_errors);
        if rows.is_empty() {
            "ID 行无法解析".to_owned()
        } else {
            let columns = rows[0].len();
            let tags = rows.iter().map(Vec::len).sum::<usize>();
            let rectangular = rows.iter().all(|row| row.len() == columns);
            format!(
                "{} 行 × {} 列，共 {tags} 个 tag{}",
                rows.len(),
                columns,
                if rectangular {
                    ""
                } else {
                    "（各行长度不一致）"
                }
            )
        }
    }

    #[must_use]
    pub fn to_config(&self, errors: &mut Vec<String>) -> TargetConfig {
        let rows = parse_rows(&self.id_rows_text, "board.id_rows", errors);
        let corners = parse_usize_values(
            &self.corner_mapping_text,
            "board.decoded_corner_to_front_corner",
            errors,
        );
        let corners = if corners.len() == 4 {
            [corners[0], corners[1], corners[2], corners[3]]
        } else {
            if corners.len() != 4 {
                errors.push(format!(
                    "board.decoded_corner_to_front_corner 需要 4 个整数，实际 {} 个",
                    corners.len()
                ));
            }
            [2, 3, 0, 1]
        };
        TargetConfig::Aprilgrid {
            family: self.family,
            border_bits: parse_u8(&self.border_bits, "board.border_bits", 2, errors),
            tag_size_m: parse_f64(
                &self.tag_size_m,
                "board.tag_size_m",
                DEFAULT_TAG_SIZE_M,
                errors,
            ),
            tag_gap_m: parse_f64(
                &self.tag_gap_m,
                "board.tag_gap_m",
                DEFAULT_TAG_GAP_M,
                errors,
            ),
            id_rows_front_view_left_to_right: rows,
            decoded_corner_to_front_corner: corners,
        }
    }
}

#[derive(Clone, Debug)]
pub struct DetectorDraft {
    pub max_hamming: String,
    pub refinement: RefinementMethodConfig,
    pub max_iterations: String,
    pub max_pixels: String,
    pub max_clusters: String,
    pub max_candidates: String,
}

impl DetectorDraft {
    #[must_use]
    pub fn from_config(detector: &DetectorConfig) -> Self {
        Self {
            max_hamming: detector.max_hamming.to_string(),
            refinement: detector.refinement.method,
            max_iterations: detector.refinement.max_iterations.to_string(),
            max_pixels: detector.budgets.max_pixels.to_string(),
            max_clusters: detector.budgets.max_clusters.to_string(),
            max_candidates: detector.budgets.max_candidates.to_string(),
        }
    }

    #[must_use]
    pub fn to_config(&self, errors: &mut Vec<String>) -> DetectorConfig {
        DetectorConfig {
            max_hamming: parse_u8(&self.max_hamming, "detector.max_hamming", 0, errors),
            refinement: RefinementConfig {
                method: self.refinement,
                max_iterations: parse_usize(
                    &self.max_iterations,
                    "detector.refinement.max_iterations",
                    20,
                    errors,
                ),
            },
            budgets: DetectorBudgets {
                max_pixels: parse_usize(
                    &self.max_pixels,
                    "detector.budgets.max_pixels",
                    16_777_216,
                    errors,
                ),
                max_clusters: parse_usize(
                    &self.max_clusters,
                    "detector.budgets.max_clusters",
                    16_384,
                    errors,
                ),
                max_candidates: parse_usize(
                    &self.max_candidates,
                    "detector.budgets.max_candidates",
                    512,
                    errors,
                ),
            },
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectionPolicy {
    PixelThreshold,
    HuberOnly,
}

#[derive(Clone, Debug)]
pub struct SolverDraft {
    pub max_iterations: String,
    pub pixel_sigma: String,
    pub huber_delta: String,
    pub point_rejection_px: String,
    pub rejection: RejectionPolicy,
}

impl SolverDraft {
    #[must_use]
    pub fn from_config(solver: &SolverConfig) -> Self {
        let (rejection, point_rejection_px) = match solver.point_rejection_px {
            Some(px) => (RejectionPolicy::PixelThreshold, format_number(px)),
            None => (RejectionPolicy::HuberOnly, format_number(6.0)),
        };
        Self {
            max_iterations: solver.max_iterations.to_string(),
            pixel_sigma: format_number(1.0),
            huber_delta: format_number(solver.huber_delta),
            point_rejection_px,
            rejection,
        }
    }

    #[must_use]
    pub fn to_config(&self, errors: &mut Vec<String>) -> SolverConfig {
        SolverConfig {
            max_iterations: parse_usize(
                &self.max_iterations,
                "solver.max_iterations",
                1_000,
                errors,
            ),
            huber_delta: parse_f64(&self.huber_delta, "solver.huber_delta", 3.0, errors),
            point_rejection_px: match self.rejection {
                RejectionPolicy::PixelThreshold => Some(parse_f64(
                    &self.point_rejection_px,
                    "solver.point_rejection_px",
                    6.0,
                    errors,
                )),
                RejectionPolicy::HuberOnly => None,
            },
        }
    }
}

#[derive(Clone, Debug)]
pub struct CameraDraft {
    pub id: String,
    pub topic: String,
    pub model: CameraModelConfig,
    pub timing: CameraTimingConfig,
}

/// Editable configuration intent. It is intentionally independent of the
/// immutable `RunSettings` snapshot handed to a running calibration, and it can
/// hold unparsable text so that navigating between sections never loses input.
#[derive(Clone, Debug)]
pub struct Draft {
    pub revision: u64,
    pub source_path: String,
    pub format: BagFormatConfig,
    pub max_delta_ns: String,
    pub require_all_cameras: bool,
    pub imu_topic: Option<String>,
    pub cameras: Vec<CameraDraft>,
    pub mode: CalibrationMode,
    pub reference_id: String,
    pub intrinsic: IntrinsicDraft,
    pub closure_rotation_deg: String,
    pub closure_translation_m: String,
    pub method: InertialMethod,
    pub time_offset: TimeOffsetDraft,
    pub imu: ImuDraft,
    pub target: TargetDraft,
    pub detector: DetectorDraft,
    pub solver: SolverDraft,
}

impl Default for Draft {
    fn default() -> Self {
        Self {
            revision: 0,
            source_path: String::new(),
            format: BagFormatConfig::Ros2Mcap,
            max_delta_ns: DEFAULT_MAX_DELTA_NS.to_string(),
            require_all_cameras: true,
            imu_topic: None,
            cameras: Vec::new(),
            mode: CalibrationMode::SingleCamera,
            reference_id: String::new(),
            intrinsic: IntrinsicDraft {
                strategy: IntrinsicStrategy::AutoOptimize,
                path: String::new(),
            },
            closure_rotation_deg: format_number(DEFAULT_CLOSURE_ROTATION_DEG),
            closure_translation_m: format_number(DEFAULT_CLOSURE_TRANSLATION_M),
            method: InertialMethod::Preintegration,
            time_offset: TimeOffsetDraft::default(),
            imu: ImuDraft::default(),
            target: TargetDraft::from_target(&TargetConfig::default()),
            detector: DetectorDraft::from_config(&DetectorConfig::default()),
            solver: SolverDraft::from_config(&SolverConfig::default()),
        }
    }
}

impl Draft {
    #[must_use]
    #[allow(
        clippy::too_many_lines,
        reason = "one exhaustive mapping from config sections into the editable draft"
    )]
    pub fn from_settings(settings: &RunSettings) -> Self {
        let target = TargetDraft::from_target(&settings.target);
        let detector = DetectorDraft::from_config(&settings.detector);
        let solver = SolverDraft::from_config(&settings.solver);
        let (mode, reference_id, intrinsic, closure_rotation_deg, closure_translation_m) =
            match &settings.calibration {
                CalibrationConfig::SingleCamera {
                    camera,
                    initial_intrinsics,
                } => (
                    CalibrationMode::SingleCamera,
                    camera.clone(),
                    IntrinsicDraft {
                        strategy: if initial_intrinsics.path.is_some() {
                            IntrinsicStrategy::FileOptimize
                        } else {
                            IntrinsicStrategy::AutoOptimize
                        },
                        path: initial_intrinsics
                            .path
                            .as_ref()
                            .map_or_else(String::new, |path| path.to_string_lossy().into_owned()),
                    },
                    format_number(DEFAULT_CLOSURE_ROTATION_DEG),
                    format_number(DEFAULT_CLOSURE_TRANSLATION_M),
                ),
                CalibrationConfig::MultiCamera {
                    reference_camera,
                    refine_intrinsics,
                    initial_intrinsics,
                    max_closure_rotation_deg,
                    max_closure_translation_m,
                } => (
                    CalibrationMode::MultiCamera,
                    reference_camera.clone(),
                    IntrinsicDraft {
                        strategy: if initial_intrinsics.path.is_some() {
                            if *refine_intrinsics {
                                IntrinsicStrategy::FileOptimize
                            } else {
                                IntrinsicStrategy::FileFixed
                            }
                        } else {
                            IntrinsicStrategy::AutoOptimize
                        },
                        path: initial_intrinsics
                            .path
                            .as_ref()
                            .map_or_else(String::new, |path| path.to_string_lossy().into_owned()),
                    },
                    format_number(*max_closure_rotation_deg),
                    format_number(*max_closure_translation_m),
                ),
                CalibrationConfig::MultiCameraImu {
                    visual_reference_camera,
                    initial_intrinsics,
                    ..
                } => (
                    CalibrationMode::MultiCameraImu,
                    visual_reference_camera.clone(),
                    IntrinsicDraft {
                        strategy: if initial_intrinsics.path.is_some() {
                            IntrinsicStrategy::FileOptimize
                        } else {
                            IntrinsicStrategy::AutoOptimize
                        },
                        path: initial_intrinsics
                            .path
                            .as_ref()
                            .map_or_else(String::new, |path| path.to_string_lossy().into_owned()),
                    },
                    format_number(DEFAULT_CLOSURE_ROTATION_DEG),
                    format_number(DEFAULT_CLOSURE_TRANSLATION_M),
                ),
            };
        let (method, time_offset, imu) = match &settings.calibration {
            CalibrationConfig::MultiCameraImu {
                method,
                time_offset,
                imu,
                ..
            } => (
                *method,
                TimeOffsetDraft::from_config(time_offset),
                ImuDraft::from_config(imu),
            ),
            _ => (
                InertialMethod::Preintegration,
                TimeOffsetDraft::default(),
                ImuDraft::default(),
            ),
        };
        let mut draft = Self {
            revision: 0,
            source_path: settings.source.path.to_string_lossy().into_owned(),
            format: settings.source.format,
            max_delta_ns: settings.source.synchronization.max_delta_ns.to_string(),
            require_all_cameras: settings.source.synchronization.require_all_cameras,
            imu_topic: settings.source.imu_topic.clone(),
            cameras: settings
                .cameras
                .iter()
                .map(|camera| CameraDraft {
                    id: camera.id.clone(),
                    topic: camera.topic.clone(),
                    model: camera.model,
                    timing: camera.timing.clone(),
                })
                .collect(),
            mode,
            reference_id,
            intrinsic,
            closure_rotation_deg,
            closure_translation_m,
            method,
            time_offset,
            imu,
            target,
            detector,
            solver,
        };
        draft.solver.pixel_sigma = format_number(
            settings
                .cameras
                .first()
                .map_or(1.0, |camera| camera.pixel_sigma),
        );
        draft
    }

    pub fn touch(&mut self) {
        self.revision = self.revision.wrapping_add(1);
    }

    pub fn source_config(&self) -> Result<BagSourceConfig> {
        let path = self.source_path.trim();
        if path.is_empty() {
            bail!("请输入 bag 路径");
        }
        Ok(BagSourceConfig {
            path: PathBuf::from(path),
            format: self.format,
            imu_topic: self.imu_topic.clone(),
            synchronization: SynchronizationConfig {
                max_delta_ns: self
                    .max_delta_ns
                    .trim()
                    .parse()
                    .unwrap_or(DEFAULT_MAX_DELTA_NS),
                require_all_cameras: self.require_all_cameras,
            },
        })
    }

    pub fn to_settings(&self) -> Result<RunSettings> {
        let mut errors = Vec::new();
        if !self.intrinsic.strategy.supported(self.mode) {
            errors.push(format!(
                "当前模式 {} 不支持内参策略“{}”；请选择自动+优化或文件+优化",
                self.mode.label(),
                self.intrinsic.strategy.label()
            ));
        }
        let reference = if self.reference_id.trim().is_empty() {
            self.cameras
                .first()
                .map_or_else(String::new, |camera| camera.id.clone())
        } else {
            self.reference_id.clone()
        };
        let intrinsic = self.intrinsic.to_config(&mut errors);
        let calibration = match self.mode {
            CalibrationMode::SingleCamera => CalibrationConfig::SingleCamera {
                camera: reference,
                initial_intrinsics: intrinsic,
            },
            CalibrationMode::MultiCamera => CalibrationConfig::MultiCamera {
                reference_camera: reference,
                refine_intrinsics: !matches!(self.intrinsic.strategy, IntrinsicStrategy::FileFixed),
                initial_intrinsics: intrinsic,
                max_closure_rotation_deg: parse_f64(
                    &self.closure_rotation_deg,
                    "multi_camera.max_closure_rotation_deg",
                    DEFAULT_CLOSURE_ROTATION_DEG,
                    &mut errors,
                ),
                max_closure_translation_m: parse_f64(
                    &self.closure_translation_m,
                    "multi_camera.max_closure_translation_m",
                    DEFAULT_CLOSURE_TRANSLATION_M,
                    &mut errors,
                ),
            },
            CalibrationMode::MultiCameraImu => CalibrationConfig::MultiCameraImu {
                method: self.method,
                visual_reference_camera: reference,
                initial_intrinsics: intrinsic,
                time_offset: self.time_offset.to_config(&mut errors),
                imu: Box::new(self.imu.to_config(&mut errors)),
            },
        };
        let settings = RunSettings {
            source: BagSourceConfig {
                path: PathBuf::from(self.source_path.trim()),
                format: self.format,
                imu_topic: if self.mode.uses_imu() {
                    self.imu_topic
                        .clone()
                        .filter(|topic| !topic.trim().is_empty())
                } else {
                    None
                },
                synchronization: SynchronizationConfig {
                    max_delta_ns: parse_i64(
                        &self.max_delta_ns,
                        "source.synchronization.max_delta_ns",
                        DEFAULT_MAX_DELTA_NS,
                        &mut errors,
                    ),
                    require_all_cameras: self.require_all_cameras,
                },
            },
            cameras: self
                .cameras
                .iter()
                .map(|camera| CameraConfig {
                    id: camera.id.clone(),
                    topic: camera.topic.clone(),
                    model: camera.model,
                    pixel_sigma: parse_f64(
                        &self.solver.pixel_sigma,
                        "cameras[].pixel_sigma",
                        1.0,
                        &mut errors,
                    ),
                    timing: camera.timing.clone(),
                })
                .collect(),
            target: self.target.to_config(&mut errors),
            detector: self.detector.to_config(&mut errors),
            calibration,
            solver: self.solver.to_config(&mut errors),
        };
        if !errors.is_empty() {
            bail!("{}", errors.join("\n"));
        }
        Ok(settings)
    }

    #[must_use]
    pub fn supports(&self, topic: &str) -> bool {
        self.cameras.iter().any(|camera| camera.topic == topic)
    }

    pub fn toggle_image_topic(&mut self, topic: &str) {
        if let Some(index) = self.cameras.iter().position(|camera| camera.topic == topic) {
            let removed = self.cameras.remove(index);
            if self.reference_id == removed.id {
                self.reference_id = self
                    .cameras
                    .first()
                    .map_or_else(String::new, |camera| camera.id.clone());
            }
        } else {
            let id = self.next_camera_id();
            self.cameras.push(CameraDraft {
                id,
                topic: topic.to_owned(),
                model: CameraModelConfig::KannalaBrandt4,
                timing: CameraTimingConfig::default(),
            });
            if self.reference_id.trim().is_empty() {
                self.reference_id = self.cameras[0].id.clone();
            }
        }
        self.infer_mode();
        self.touch();
    }

    fn next_camera_id(&self) -> String {
        let used = self
            .cameras
            .iter()
            .map(|camera| camera.id.as_str())
            .collect::<HashSet<_>>();
        for index in 0.. {
            let candidate = format!("cam{index}");
            if !used.contains(candidate.as_str()) {
                return candidate;
            }
        }
        unreachable!("unbounded camera id search always finds a free id")
    }

    pub fn set_imu_topic(&mut self, topic: Option<String>) {
        self.imu_topic = topic;
        self.infer_mode();
        self.touch();
    }

    fn infer_mode(&mut self) {
        let mode = if self.imu_topic.is_some() {
            CalibrationMode::MultiCameraImu
        } else if self.cameras.len() > 1 {
            CalibrationMode::MultiCamera
        } else {
            CalibrationMode::SingleCamera
        };
        self.set_mode(mode);
    }

    pub fn set_reference_to_id(&mut self, id: String) {
        self.reference_id = id;
        self.touch();
    }

    pub fn set_mode(&mut self, mode: CalibrationMode) {
        if self.mode != mode {
            self.mode = mode;
            if !mode.uses_imu() && self.solver.rejection == RejectionPolicy::HuberOnly {
                self.solver.rejection = RejectionPolicy::PixelThreshold;
            }
            if !self
                .cameras
                .iter()
                .any(|camera| camera.id == self.reference_id)
            {
                self.reference_id = self
                    .cameras
                    .first()
                    .map_or_else(String::new, |camera| camera.id.clone());
            }
            self.touch();
        }
    }

    pub fn set_camera_id(&mut self, index: usize, id: &str) {
        let Some(camera) = self.cameras.get_mut(index) else {
            return;
        };
        let old = std::mem::replace(&mut camera.id, id.trim().to_owned());
        if self.reference_id == old {
            self.reference_id.clone_from(&camera.id);
        }
        self.touch();
    }

    pub fn set_camera_model(&mut self, index: usize, model: CameraModelConfig) {
        if let Some(camera) = self.cameras.get_mut(index) {
            camera.model = model;
            self.touch();
        }
    }

    #[must_use]
    pub fn reference_index(&self) -> usize {
        self.cameras
            .iter()
            .position(|camera| camera.id == self.reference_id)
            .unwrap_or(0)
    }

    #[must_use]
    pub fn reference_label(&self) -> String {
        if self.cameras.is_empty() {
            "-".to_owned()
        } else {
            let index = self.reference_index();
            let camera = &self.cameras[index];
            format!("{} ({})", camera.id, camera.topic)
        }
    }

    #[must_use]
    pub fn strategy_options(&self) -> Vec<IntrinsicStrategy> {
        IntrinsicStrategy::ALL
            .into_iter()
            .filter(|strategy| strategy.supported(self.mode))
            .collect()
    }
}

impl IntrinsicDraft {
    #[must_use]
    fn to_config(&self, errors: &mut Vec<String>) -> InitialIntrinsicsConfig {
        match self.strategy {
            IntrinsicStrategy::AutoOptimize => InitialIntrinsicsConfig { path: None },
            IntrinsicStrategy::FileOptimize | IntrinsicStrategy::FileFixed => {
                let path = self.path.trim();
                if path.is_empty() {
                    errors.push("内参初值文件路径为空：文件策略需要非空路径".to_owned());
                    InitialIntrinsicsConfig { path: None }
                } else {
                    InitialIntrinsicsConfig {
                        path: Some(PathBuf::from(path)),
                    }
                }
            }
        }
    }
}

impl TimeOffsetDraft {
    #[must_use]
    fn from_config(config: &TimeOffsetConfig) -> Self {
        let mut draft = Self::default();
        match config {
            TimeOffsetConfig::Fixed { value_s } => {
                draft.kind = TimeOffsetKind::Fixed;
                draft.fixed_s = format_number(*value_s);
            }
            TimeOffsetConfig::Estimate {
                initial_s,
                minimum_s,
                maximum_s,
                initialization_steps,
            } => {
                draft.kind = TimeOffsetKind::Estimate;
                draft.initial_s = format_number(*initial_s);
                draft.minimum_s = format_number(*minimum_s);
                draft.maximum_s = format_number(*maximum_s);
                draft.initialization_steps = initialization_steps.to_string();
            }
        }
        draft
    }

    #[must_use]
    fn to_config(&self, errors: &mut Vec<String>) -> TimeOffsetConfig {
        match self.kind {
            TimeOffsetKind::Fixed => TimeOffsetConfig::Fixed {
                value_s: parse_f64(&self.fixed_s, "imu.time_offset.value_s", 0.0, errors),
            },
            TimeOffsetKind::Estimate => TimeOffsetConfig::Estimate {
                initial_s: parse_f64(&self.initial_s, "imu.time_offset.initial_s", 0.0, errors),
                minimum_s: parse_f64(&self.minimum_s, "imu.time_offset.minimum_s", -0.1, errors),
                maximum_s: parse_f64(&self.maximum_s, "imu.time_offset.maximum_s", 0.1, errors),
                initialization_steps: parse_usize(
                    &self.initialization_steps,
                    "imu.time_offset.initialization_steps",
                    11,
                    errors,
                ),
            },
        }
    }
}

impl ImuDraft {
    #[must_use]
    fn from_config(imu: &ImuConfig) -> Self {
        Self {
            gravity_magnitude_m_s2: format_number(imu.gravity_magnitude_m_s2),
            gyro_density_rad_s_sqrt_hz: format_number(
                imu.noise.gyro_covariance_rad2_s_hz[0][0].sqrt(),
            ),
            accelerometer_density_m_s2_sqrt_hz: format_number(
                imu.noise.accelerometer_covariance_m2_s4_hz[0][0].sqrt(),
            ),
            gyro_bias_walk_rad_s2_sqrt_hz: format_number(imu.noise.gyro_bias_walk_rad_s2_sqrt_hz),
            accelerometer_bias_walk_m_s3_sqrt_hz: format_number(
                imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz,
            ),
            integration_method: imu.integration.method,
            max_gap_s: format_number(imu.integration.max_gap_s),
            max_samples: imu.integration.max_samples.to_string(),
            refine_axes: imu.axis_calibration.refine,
            gyro_calibrated_from_raw: format_matrix(&imu.axis_calibration.gyro_calibrated_from_raw),
            accelerometer_calibrated_from_raw: format_matrix(
                &imu.axis_calibration.accelerometer_calibrated_from_raw,
            ),
            knot_interval_s: format_number(imu.continuous_time.knot_interval_s),
            refine_camera_time_offsets: imu.continuous_time.camera_time_offsets.refine,
            camera_time_offset_minimum_s: format_number(
                imu.continuous_time.camera_time_offsets.minimum_s,
            ),
            camera_time_offset_maximum_s: format_number(
                imu.continuous_time.camera_time_offsets.maximum_s,
            ),
            refine_global_clock_skew: imu.continuous_time.clock_skews.refine_global,
            refine_camera_clock_skews: imu.continuous_time.clock_skews.refine_cameras,
            clock_skew_minimum_ppm: format_number(imu.continuous_time.clock_skews.minimum_ppm),
            clock_skew_maximum_ppm: format_number(imu.continuous_time.clock_skews.maximum_ppm),
            visual_motion_anisotropy_enabled: imu.continuous_time.visual_motion_anisotropy.enabled,
            visual_motion_anisotropy_ratio_per_motion_squared: format_number(
                imu.continuous_time
                    .visual_motion_anisotropy
                    .variance_ratio_per_motion_squared,
            ),
        }
    }

    #[must_use]
    fn to_config(&self, errors: &mut Vec<String>) -> ImuConfig {
        ImuConfig {
            gravity_magnitude_m_s2: parse_f64(
                &self.gravity_magnitude_m_s2,
                "imu.gravity_magnitude_m_s2",
                9.80665,
                errors,
            ),
            noise: {
                let gyro = parse_f64(
                    &self.gyro_density_rad_s_sqrt_hz,
                    "imu.noise.gyro_density_rad_s_sqrt_hz",
                    0.001,
                    errors,
                );
                let accel = parse_f64(
                    &self.accelerometer_density_m_s2_sqrt_hz,
                    "imu.noise.accelerometer_density_m_s2_sqrt_hz",
                    0.01,
                    errors,
                );
                ImuNoiseConfig {
                    gyro_covariance_rad2_s_hz: [
                        [gyro * gyro, 0.0, 0.0],
                        [0.0, gyro * gyro, 0.0],
                        [0.0, 0.0, gyro * gyro],
                    ],
                    accelerometer_covariance_m2_s4_hz: [
                        [accel * accel, 0.0, 0.0],
                        [0.0, accel * accel, 0.0],
                        [0.0, 0.0, accel * accel],
                    ],
                    gyro_bias_walk_rad_s2_sqrt_hz: parse_f64(
                        &self.gyro_bias_walk_rad_s2_sqrt_hz,
                        "imu.noise.gyro_bias_walk_rad_s2_sqrt_hz",
                        0.00001,
                        errors,
                    ),
                    accelerometer_bias_walk_m_s3_sqrt_hz: parse_f64(
                        &self.accelerometer_bias_walk_m_s3_sqrt_hz,
                        "imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz",
                        0.0001,
                        errors,
                    ),
                }
            },
            integration: IntegrationConfig {
                method: self.integration_method,
                max_gap_s: parse_f64(&self.max_gap_s, "imu.integration.max_gap_s", 0.02, errors),
                max_samples: parse_usize(
                    &self.max_samples,
                    "imu.integration.max_samples",
                    2_000_000,
                    errors,
                ),
            },
            axis_calibration: ImuAxisCalibrationConfig {
                refine: self.refine_axes,
                gyro_calibrated_from_raw: parse_matrix(
                    &self.gyro_calibrated_from_raw,
                    "imu.axis_calibration.gyro_calibrated_from_raw",
                    errors,
                ),
                accelerometer_calibrated_from_raw: parse_matrix(
                    &self.accelerometer_calibrated_from_raw,
                    "imu.axis_calibration.accelerometer_calibrated_from_raw",
                    errors,
                ),
            },
            continuous_time: ContinuousTimeConfig {
                knot_interval_s: parse_f64(
                    &self.knot_interval_s,
                    "imu.continuous_time.knot_interval_s",
                    0.01,
                    errors,
                ),
                camera_time_offsets: crate::config::CameraTimeOffsetsConfig {
                    refine: self.refine_camera_time_offsets,
                    minimum_s: parse_f64(
                        &self.camera_time_offset_minimum_s,
                        "imu.continuous_time.camera_time_offsets.minimum_s",
                        -0.005,
                        errors,
                    ),
                    maximum_s: parse_f64(
                        &self.camera_time_offset_maximum_s,
                        "imu.continuous_time.camera_time_offsets.maximum_s",
                        0.005,
                        errors,
                    ),
                },
                clock_skews: crate::config::ClockSkewsConfig {
                    refine_global: self.refine_global_clock_skew,
                    refine_cameras: self.refine_camera_clock_skews,
                    minimum_ppm: parse_f64(
                        &self.clock_skew_minimum_ppm,
                        "imu.continuous_time.clock_skews.minimum_ppm",
                        -200.0,
                        errors,
                    ),
                    maximum_ppm: parse_f64(
                        &self.clock_skew_maximum_ppm,
                        "imu.continuous_time.clock_skews.maximum_ppm",
                        200.0,
                        errors,
                    ),
                },
                visual_motion_anisotropy: crate::config::VisualMotionAnisotropyConfig {
                    enabled: self.visual_motion_anisotropy_enabled,
                    variance_ratio_per_motion_squared: parse_f64(
                        &self.visual_motion_anisotropy_ratio_per_motion_squared,
                        "imu.continuous_time.visual_motion_anisotropy.variance_ratio_per_motion_squared",
                        0.015,
                        errors,
                    ),
                },
            },
        }
    }

    #[must_use]
    pub fn axes_are_identity(&self) -> bool {
        let mut errors = Vec::new();
        parse_matrix(&self.gyro_calibrated_from_raw, "gyro", &mut errors) == IDENTITY_MATRIX
            && parse_matrix(
                &self.accelerometer_calibrated_from_raw,
                "accelerometer",
                &mut errors,
            ) == IDENTITY_MATRIX
            && errors.is_empty()
    }
}

pub fn format_number(value: f64) -> String {
    format!("{value}")
}

pub fn format_rows(rows: &[Vec<u32>]) -> String {
    rows.iter()
        .map(|row| {
            row.iter()
                .map(ToString::to_string)
                .collect::<Vec<_>>()
                .join(", ")
        })
        .collect::<Vec<_>>()
        .join(" ; ")
}

fn format_matrix(matrix: &[[f64; 3]; 3]) -> String {
    matrix
        .iter()
        .map(|row| {
            row.iter()
                .map(|value| format_number(*value))
                .collect::<Vec<_>>()
                .join(", ")
        })
        .collect::<Vec<_>>()
        .join(" ; ")
}

fn parse_f64(text: &str, label: &str, fallback: f64, errors: &mut Vec<String>) -> f64 {
    match text.trim().parse::<f64>() {
        Ok(value) if value.is_finite() => value,
        Ok(_) => {
            errors.push(format!("{label}: 必须是有限数字"));
            fallback
        }
        Err(error) => {
            errors.push(format!("{label}: 无法解析为数字（{error}）"));
            fallback
        }
    }
}

fn parse_i64(text: &str, label: &str, fallback: i64, errors: &mut Vec<String>) -> i64 {
    match text.trim().parse::<i64>() {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{label}: 无法解析为整数（{error}）"));
            fallback
        }
    }
}

fn parse_usize(text: &str, label: &str, fallback: usize, errors: &mut Vec<String>) -> usize {
    match text.trim().parse::<usize>() {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{label}: 无法解析为非负整数（{error}）"));
            fallback
        }
    }
}

fn parse_u8(text: &str, label: &str, fallback: u8, errors: &mut Vec<String>) -> u8 {
    match text.trim().parse::<u8>() {
        Ok(value) => value,
        Err(error) => {
            errors.push(format!("{label}: 无法解析为 0..=255 整数（{error}）"));
            fallback
        }
    }
}

fn parse_usize_values(text: &str, _label: &str, _errors: &mut Vec<String>) -> Vec<usize> {
    text.split(|value: char| value == ',' || value.is_whitespace())
        .filter(|value| !value.is_empty())
        .filter_map(|value| value.parse::<usize>().ok())
        .collect()
}

fn parse_rows(text: &str, label: &str, errors: &mut Vec<String>) -> Vec<Vec<u32>> {
    let mut rows = Vec::new();
    for row in text.split([';', '\n']) {
        let row = row.trim();
        if row.is_empty() {
            continue;
        }
        let mut ids = Vec::new();
        for value in row.split(|value: char| value == ',' || value.is_whitespace()) {
            if value.is_empty() {
                continue;
            }
            match value.parse::<u32>() {
                Ok(id) => ids.push(id),
                Err(error) => {
                    errors.push(format!("{label}: 无法解析 tag ID {value}（{error}）"));
                }
            }
        }
        if ids.is_empty() {
            errors.push(format!("{label}: 存在空行"));
        } else {
            rows.push(ids);
        }
    }
    if rows.is_empty() {
        errors.push(format!("{label}: 至少需要一行 tag ID"));
    }
    rows
}

fn parse_matrix(text: &str, label: &str, errors: &mut Vec<String>) -> [[f64; 3]; 3] {
    let values = text
        .split(|value: char| value == ',' || value == ';' || value.is_whitespace())
        .filter(|value| !value.is_empty())
        .map(str::parse::<f64>)
        .collect::<Result<Vec<_>, _>>();
    match values {
        Ok(values) if values.len() == 9 && values.iter().all(|value| value.is_finite()) => [
            [values[0], values[1], values[2]],
            [values[3], values[4], values[5]],
            [values[6], values[7], values[8]],
        ],
        Ok(values) => {
            errors.push(format!(
                "{label}: 需要 9 个有限数字（行优先 3×3），实际 {} 个",
                values.len()
            ));
            IDENTITY_MATRIX
        }
        Err(error) => {
            errors.push(format!("{label}: 无法解析矩阵（{error}）"));
            IDENTITY_MATRIX
        }
    }
}

#[must_use]
pub const fn bag_format_index(format: BagFormatConfig) -> usize {
    match format {
        BagFormatConfig::Ros1V2 => 0,
        BagFormatConfig::Ros2Mcap => 1,
        BagFormatConfig::Ros2Sqlite3 => 2,
    }
}

#[must_use]
pub const fn camera_model_index(model: CameraModelConfig) -> usize {
    match model {
        CameraModelConfig::KannalaBrandt4 => 0,
        CameraModelConfig::DoubleSphere => 1,
        CameraModelConfig::ExtendedUnified => 2,
        CameraModelConfig::PinholeNone => 3,
        CameraModelConfig::PinholeRadtan5 => 4,
        CameraModelConfig::PinholeRadtan8 => 5,
        CameraModelConfig::PinholeD12 => 6,
    }
}

#[must_use]
pub const fn tag_family_index(family: TagFamilyConfig) -> usize {
    match family {
        TagFamilyConfig::Tag16h5 => 0,
        TagFamilyConfig::Tag25h9 => 1,
        TagFamilyConfig::Tag36h10 => 2,
        TagFamilyConfig::Tag36h11 => 3,
    }
}

#[must_use]
pub const fn refinement_index(method: RefinementMethodConfig) -> usize {
    match method {
        RefinementMethodConfig::Disabled => 0,
        RefinementMethodConfig::Saddle => 1,
        RefinementMethodConfig::OuterContour => 2,
    }
}

#[must_use]
pub const fn inertial_method_index(method: InertialMethod) -> usize {
    match method {
        InertialMethod::Preintegration => 0,
        InertialMethod::ContinuousTime => 1,
    }
}

#[must_use]
pub fn camera_model_label(model: CameraModelConfig) -> &'static str {
    match model {
        CameraModelConfig::KannalaBrandt4 => "KannalaBrandt4",
        CameraModelConfig::DoubleSphere => "DoubleSphere",
        CameraModelConfig::ExtendedUnified => "ExtendedUnified",
        CameraModelConfig::PinholeNone => "PinholeNone",
        CameraModelConfig::PinholeRadtan5 => "PinholeRadtan5",
        CameraModelConfig::PinholeRadtan8 => "PinholeRadtan8",
        CameraModelConfig::PinholeD12 => "PinholeD12",
    }
}

pub const CAMERA_MODELS: [CameraModelConfig; 7] = [
    CameraModelConfig::KannalaBrandt4,
    CameraModelConfig::DoubleSphere,
    CameraModelConfig::ExtendedUnified,
    CameraModelConfig::PinholeNone,
    CameraModelConfig::PinholeRadtan5,
    CameraModelConfig::PinholeRadtan8,
    CameraModelConfig::PinholeD12,
];

#[must_use]
pub const fn bag_format_label(format: BagFormatConfig) -> &'static str {
    match format {
        BagFormatConfig::Ros1V2 => "ROS1 bag v2",
        BagFormatConfig::Ros2Mcap => "ROS2 MCAP",
        BagFormatConfig::Ros2Sqlite3 => "ROS2 SQLite3",
    }
}

pub const BAG_FORMATS: [BagFormatConfig; 3] = [
    BagFormatConfig::Ros1V2,
    BagFormatConfig::Ros2Mcap,
    BagFormatConfig::Ros2Sqlite3,
];

#[must_use]
pub const fn tag_family_label(family: TagFamilyConfig) -> &'static str {
    match family {
        TagFamilyConfig::Tag16h5 => "tag16h5",
        TagFamilyConfig::Tag25h9 => "tag25h9",
        TagFamilyConfig::Tag36h10 => "tag36h10",
        TagFamilyConfig::Tag36h11 => "tag36h11",
    }
}

pub const TAG_FAMILIES: [TagFamilyConfig; 4] = [
    TagFamilyConfig::Tag16h5,
    TagFamilyConfig::Tag25h9,
    TagFamilyConfig::Tag36h10,
    TagFamilyConfig::Tag36h11,
];

#[must_use]
pub const fn refinement_label(method: RefinementMethodConfig) -> &'static str {
    match method {
        RefinementMethodConfig::Disabled => "disabled",
        RefinementMethodConfig::Saddle => "saddle",
        RefinementMethodConfig::OuterContour => "outer_contour",
    }
}

pub const REFINEMENTS: [RefinementMethodConfig; 3] = [
    RefinementMethodConfig::Disabled,
    RefinementMethodConfig::Saddle,
    RefinementMethodConfig::OuterContour,
];

#[must_use]
pub const fn inertial_method_label(method: InertialMethod) -> &'static str {
    match method {
        InertialMethod::Preintegration => "预积分",
        InertialMethod::ContinuousTime => "连续时间 B 样条",
    }
}

pub const INERTIAL_METHODS: [InertialMethod; 2] = [
    InertialMethod::Preintegration,
    InertialMethod::ContinuousTime,
];

#[must_use]
pub const fn time_offset_kind_label(kind: TimeOffsetKind) -> &'static str {
    match kind {
        TimeOffsetKind::Fixed => "固定偏移",
        TimeOffsetKind::Estimate => "估计偏移",
    }
}

#[must_use]
pub const fn rejection_policy_label(policy: RejectionPolicy) -> &'static str {
    match policy {
        RejectionPolicy::PixelThreshold => "像素阈值截断",
        RejectionPolicy::HuberOnly => "仅 Huber（不截断）",
    }
}

pub const REJECTION_POLICIES: [RejectionPolicy; 2] =
    [RejectionPolicy::PixelThreshold, RejectionPolicy::HuberOnly];

#[must_use]
pub const fn rejection_policy_index(policy: RejectionPolicy) -> usize {
    match policy {
        RejectionPolicy::PixelThreshold => 0,
        RejectionPolicy::HuberOnly => 1,
    }
}

#[must_use]
pub const fn integration_method_label(method: IntegrationMethod) -> &'static str {
    match method {
        IntegrationMethod::Midpoint => "midpoint",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    use crate::config::{
        CameraConfig, DetectorConfig, InitialIntrinsicsConfig, RunSettings, SolverConfig,
        SynchronizationConfig,
    };
    use crate::tui::preflight::Preflight;

    fn settings() -> RunSettings {
        RunSettings {
            source: BagSourceConfig {
                path: PathBuf::from("recording.bag"),
                format: BagFormatConfig::Ros1V2,
                imu_topic: None,
                synchronization: SynchronizationConfig::default(),
            },
            cameras: vec![CameraConfig {
                id: "cam0".to_owned(),
                topic: "/cam0/image_raw".to_owned(),
                model: CameraModelConfig::KannalaBrandt4,
                pixel_sigma: 1.0,
                timing: CameraTimingConfig::default(),
            }],
            target: TargetConfig::default(),
            detector: DetectorConfig::default(),
            calibration: CalibrationConfig::SingleCamera {
                camera: "cam0".to_owned(),
                initial_intrinsics: InitialIntrinsicsConfig::default(),
            },
            solver: SolverConfig::default(),
        }
    }

    fn target_parts(target: TargetConfig) -> (Vec<Vec<u32>>, [usize; 4]) {
        let TargetConfig::Aprilgrid {
            id_rows_front_view_left_to_right,
            decoded_corner_to_front_corner,
            ..
        } = target;
        (
            id_rows_front_view_left_to_right,
            decoded_corner_to_front_corner,
        )
    }

    #[test]
    fn board_geometry_round_trips_exactly() {
        let mut original = settings();
        let rows = vec![
            vec![30, 31, 32, 33, 34, 35],
            vec![24, 25, 26, 27, 28, 29],
            vec![18, 19, 20, 21, 22, 23],
            vec![12, 13, 14, 15, 16, 17],
            vec![6, 7, 8, 9, 10, 11],
            vec![0, 1, 2, 3, 4, 5],
        ];
        let TargetConfig::Aprilgrid {
            id_rows_front_view_left_to_right,
            decoded_corner_to_front_corner,
            ..
        } = &mut original.target;
        id_rows_front_view_left_to_right.clone_from(&rows);
        *decoded_corner_to_front_corner = [2, 3, 0, 1];

        let draft = Draft::from_settings(&original);
        let rebuilt = draft.to_settings().unwrap();
        let (rebuilt_rows, rebuilt_corners) = target_parts(rebuilt.target);

        assert_eq!(rebuilt_rows, rows);
        assert_eq!(rebuilt_corners, [2, 3, 0, 1]);
    }

    #[test]
    fn editing_board_text_never_regenerates_layout() {
        let mut draft = Draft::from_settings(&settings());
        draft.target.id_rows_text = "5, 4, 3; 2, 1, 0".to_owned();
        draft.target.corner_mapping_text = "1, 2, 3, 0".to_owned();

        let rebuilt = draft.to_settings().unwrap();
        let (rows, corners) = target_parts(rebuilt.target);

        assert_eq!(rows, vec![vec![5, 4, 3], vec![2, 1, 0]]);
        assert_eq!(corners, [1, 2, 3, 0]);
    }

    #[test]
    fn topic_selection_updates_mode_in_both_directions() {
        let mut draft = Draft::from_settings(&settings());
        draft.toggle_image_topic("/cam1/image_raw");
        assert_eq!(draft.mode, CalibrationMode::MultiCamera);
        draft.set_imu_topic(Some("/imu".to_owned()));
        assert_eq!(draft.mode, CalibrationMode::MultiCameraImu);
        assert_eq!(
            draft.to_settings().unwrap().source.imu_topic.as_deref(),
            Some("/imu")
        );
        draft.set_imu_topic(None);
        assert_eq!(draft.mode, CalibrationMode::MultiCamera);
        draft.toggle_image_topic("/cam1/image_raw");
        assert_eq!(draft.mode, CalibrationMode::SingleCamera);
        draft.set_imu_topic(Some("/imu".to_owned()));
        assert_eq!(draft.mode, CalibrationMode::MultiCameraImu);
    }

    #[test]
    fn intrinsic_strategies_map_to_supported_config_flags() {
        let mut draft = Draft::from_settings(&settings());
        draft.set_mode(CalibrationMode::MultiCamera);
        draft.cameras.push(CameraDraft {
            id: "cam1".to_owned(),
            topic: "/cam1/image_raw".to_owned(),
            model: CameraModelConfig::DoubleSphere,
            timing: CameraTimingConfig::default(),
        });
        draft.intrinsic.path = "prior.yaml".to_owned();

        draft.intrinsic.strategy = IntrinsicStrategy::FileOptimize;
        let settings = draft.to_settings().unwrap();
        let CalibrationConfig::MultiCamera {
            refine_intrinsics,
            initial_intrinsics,
            ..
        } = settings.calibration
        else {
            panic!("expected multi-camera config");
        };
        assert!(refine_intrinsics);
        assert_eq!(
            initial_intrinsics.path.as_deref(),
            Some(std::path::Path::new("prior.yaml"))
        );

        draft.intrinsic.strategy = IntrinsicStrategy::FileFixed;
        let settings = draft.to_settings().unwrap();
        let CalibrationConfig::MultiCamera {
            refine_intrinsics,
            initial_intrinsics,
            ..
        } = settings.calibration
        else {
            panic!("expected multi-camera config");
        };
        assert!(!refine_intrinsics);
        assert!(initial_intrinsics.path.is_some());

        draft.intrinsic.strategy = IntrinsicStrategy::AutoOptimize;
        let settings = draft.to_settings().unwrap();
        let CalibrationConfig::MultiCamera {
            refine_intrinsics,
            initial_intrinsics,
            ..
        } = settings.calibration
        else {
            panic!("expected multi-camera config");
        };
        assert!(refine_intrinsics);
        assert!(initial_intrinsics.path.is_none());

        draft.set_mode(CalibrationMode::SingleCamera);
        draft.intrinsic.strategy = IntrinsicStrategy::FileFixed;
        assert!(draft.to_settings().is_err());
    }

    #[test]
    fn valid_single_camera_config_can_start_without_scanning() {
        let directory = tempfile::tempdir().unwrap();
        let bag = directory.path().join("recording.bag");
        std::fs::write(&bag, b"").unwrap();
        let mut draft = Draft::from_settings(&settings());
        draft.source_path = bag.to_string_lossy().into_owned();

        let preflight = Preflight::evaluate(&draft, None, directory.path());

        assert_eq!(preflight.blockers(), 0);
        assert_eq!(preflight.warnings(), 0);
        assert!(preflight.not_checked() > 0);
        assert!(preflight.can_start());
    }

    #[test]
    fn file_strategy_without_a_file_is_a_preflight_blocker() {
        let directory = tempfile::tempdir().unwrap();
        let bag = directory.path().join("recording.bag");
        std::fs::write(&bag, b"").unwrap();
        let mut draft = Draft::from_settings(&settings());
        draft.source_path = bag.to_string_lossy().into_owned();
        draft.set_mode(CalibrationMode::MultiCamera);
        draft.cameras.push(CameraDraft {
            id: "cam1".to_owned(),
            topic: "/cam1/image_raw".to_owned(),
            model: CameraModelConfig::KannalaBrandt4,
            timing: CameraTimingConfig::default(),
        });
        draft.intrinsic.strategy = IntrinsicStrategy::FileOptimize;
        draft.intrinsic.path = directory
            .path()
            .join("missing-prior.yaml")
            .to_string_lossy()
            .into_owned();

        let preflight = Preflight::evaluate(&draft, None, directory.path());
        assert!(!preflight.can_start());
        assert!(preflight.checks.iter().any(|check| {
            check.level == super::super::preflight::CheckLevel::Blocker
                && check.detail.contains("内参初值文件")
                && check.detail.contains("不存在")
        }));
    }
}
