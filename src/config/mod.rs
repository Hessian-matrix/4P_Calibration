use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use ccl_calib::camera::continuous_time::ImuAxisCalibration;
use ccl_calib::camera::{CameraModel, PinholeModel};
use ccl_rosbag::BagFormat;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RunSettings {
    pub source: BagSourceConfig,
    pub cameras: Vec<CameraConfig>,
    pub target: TargetConfig,
    #[serde(default)]
    pub detector: DetectorConfig,
    pub calibration: CalibrationConfig,
    #[serde(default)]
    pub solver: SolverConfig,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BagSourceConfig {
    pub path: PathBuf,
    pub format: BagFormatConfig,
    #[serde(default)]
    pub imu_topic: Option<String>,
    #[serde(default)]
    pub synchronization: SynchronizationConfig,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BagFormatConfig {
    Ros1V2,
    Ros2Mcap,
    Ros2Sqlite3,
}

impl BagFormatConfig {
    #[must_use]
    pub const fn core(self) -> BagFormat {
        match self {
            Self::Ros1V2 => BagFormat::Ros1V2,
            Self::Ros2Mcap => BagFormat::Ros2Mcap,
            Self::Ros2Sqlite3 => BagFormat::Ros2Sqlite3,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SynchronizationConfig {
    pub max_delta_ns: i64,
    pub require_all_cameras: bool,
}

impl Default for SynchronizationConfig {
    fn default() -> Self {
        Self {
            max_delta_ns: 2_000_000,
            require_all_cameras: true,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraConfig {
    pub id: String,
    pub topic: String,
    pub model: CameraModelConfig,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CameraModelConfig {
    KannalaBrandt4,
    DoubleSphere,
    ExtendedUnified,
    PinholeNone,
    PinholeRadtan5,
    PinholeRadtan8,
    PinholeD12,
}

impl CameraModelConfig {
    #[must_use]
    pub const fn core(self) -> CameraModel {
        match self {
            Self::KannalaBrandt4 => CameraModel::KannalaBrandt4,
            Self::DoubleSphere => CameraModel::DoubleSphere,
            Self::ExtendedUnified => CameraModel::ExtendedUnified,
            Self::PinholeNone => CameraModel::Pinhole(PinholeModel::None),
            Self::PinholeRadtan5 => CameraModel::Pinhole(PinholeModel::Radtan5),
            Self::PinholeRadtan8 => CameraModel::Pinhole(PinholeModel::Radtan8),
            Self::PinholeD12 => CameraModel::Pinhole(PinholeModel::D12),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum TargetConfig {
    Aprilgrid {
        family: TagFamilyConfig,
        border_bits: u8,
        tag_size_m: f64,
        tag_gap_m: f64,
        id_rows_front_view_left_to_right: Vec<Vec<u32>>,
        decoded_corner_to_front_corner: [usize; 4],
    },
}

impl Default for TargetConfig {
    fn default() -> Self {
        Self::Aprilgrid {
            family: TagFamilyConfig::Tag36h11,
            border_bits: 2,
            tag_size_m: 0.088,
            tag_gap_m: 0.0264,
            id_rows_front_view_left_to_right: vec![
                vec![30, 31, 32, 33, 34, 35],
                vec![24, 25, 26, 27, 28, 29],
                vec![18, 19, 20, 21, 22, 23],
                vec![12, 13, 14, 15, 16, 17],
                vec![6, 7, 8, 9, 10, 11],
                vec![0, 1, 2, 3, 4, 5],
            ],
            decoded_corner_to_front_corner: [2, 3, 0, 1],
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TagFamilyConfig {
    Tag16h5,
    Tag25h9,
    Tag36h10,
    Tag36h11,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DetectorConfig {
    pub max_hamming: u8,
    pub refinement: RefinementConfig,
    pub budgets: DetectorBudgets,
}

impl Default for DetectorConfig {
    fn default() -> Self {
        Self {
            max_hamming: 0,
            refinement: RefinementConfig {
                method: RefinementMethodConfig::Saddle,
                max_iterations: 20,
            },
            budgets: DetectorBudgets {
                max_pixels: 16_777_216,
                max_clusters: 16_384,
                max_candidates: 512,
            },
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RefinementConfig {
    pub method: RefinementMethodConfig,
    pub max_iterations: usize,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RefinementMethodConfig {
    Disabled,
    Saddle,
    OuterContour,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DetectorBudgets {
    pub max_pixels: usize,
    pub max_clusters: usize,
    pub max_candidates: usize,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InitialIntrinsicsConfig {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<PathBuf>,
}

impl InitialIntrinsicsConfig {
    #[must_use]
    pub const fn is_automatic(&self) -> bool {
        self.path.is_none()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum CalibrationConfig {
    SingleCamera {
        camera: String,
        #[serde(default, skip_serializing_if = "InitialIntrinsicsConfig::is_automatic")]
        initial_intrinsics: InitialIntrinsicsConfig,
    },
    MultiCamera {
        reference_camera: String,
        refine_intrinsics: bool,
        #[serde(default, skip_serializing_if = "InitialIntrinsicsConfig::is_automatic")]
        initial_intrinsics: InitialIntrinsicsConfig,
        max_closure_rotation_deg: f64,
        max_closure_translation_m: f64,
    },
    MultiCameraImu {
        method: InertialMethod,
        visual_reference_camera: String,
        #[serde(default, skip_serializing_if = "InitialIntrinsicsConfig::is_automatic")]
        initial_intrinsics: InitialIntrinsicsConfig,
        time_offset: TimeOffsetConfig,
        imu: Box<ImuConfig>,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum InertialMethod {
    Preintegration,
    ContinuousTime,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum TimeOffsetConfig {
    Fixed {
        value_s: f64,
    },
    Estimate {
        initial_s: f64,
        minimum_s: f64,
        maximum_s: f64,
        initialization_steps: usize,
    },
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuConfig {
    pub gravity_magnitude_m_s2: f64,
    pub noise: ImuNoiseConfig,
    pub integration: IntegrationConfig,
    /// Optional per-axis scale and cross-axis non-orthogonality refinement.
    #[serde(default)]
    pub axis_calibration: ImuAxisCalibrationConfig,
    #[serde(default)]
    pub continuous_time: ContinuousTimeConfig,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ContinuousTimeConfig {
    /// Uniform quartic B-spline knot interval in seconds; defaults to 0.01 s.
    pub knot_interval_s: f64,
}

impl Default for ContinuousTimeConfig {
    fn default() -> Self {
        Self {
            knot_interval_s: 0.01,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImuAxisCalibrationConfig {
    /// Enable in the second continuous-time pass after the fixed-time, fixed-axis baseline.
    pub refine: bool,
    /// Row-major `A` in `physical_gyro = A * raw_gyro - bias`.
    ///
    /// Any finite nonsingular orientation-preserving matrix is accepted.
    pub gyro_calibrated_from_raw: [[f64; 3]; 3],
    /// Row-major `A` in `physical_specific_force = A * raw_accelerometer - bias`.
    ///
    /// Stored as the physical Basalt model: lower triangular with positive diagonal.
    pub accelerometer_calibrated_from_raw: [[f64; 3]; 3],
}

impl Default for ImuAxisCalibrationConfig {
    fn default() -> Self {
        Self {
            refine: false,
            gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            accelerometer_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

impl ImuAxisCalibrationConfig {
    #[must_use]
    pub const fn core(self) -> ImuAxisCalibration {
        ImuAxisCalibration {
            gyro_calibrated_from_raw: self.gyro_calibrated_from_raw,
            accelerometer_calibrated_from_raw: self.accelerometer_calibrated_from_raw,
        }
    }

    #[must_use]
    fn is_identity(&self) -> bool {
        let identity = Self::default();
        self.gyro_calibrated_from_raw == identity.gyro_calibrated_from_raw
            && self.accelerometer_calibrated_from_raw == identity.accelerometer_calibrated_from_raw
    }

    fn validate(&self) -> Result<()> {
        self.core()
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid imu.axis_calibration: {error}"))
    }
}

/// Sensor noise densities of the calibrated physical measurements, i.e. after the
/// axis matrix and before bias removal.
///
/// The continuous-time method keeps caller-supplied scalar isotropic per-sample
/// standard deviations, derived from these densities, and weights each calibrated
/// physical-frame sample directly; it deliberately does not transform a raw-frame
/// covariance through the axis matrix.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuNoiseConfig {
    pub gyro_density_rad_s_sqrt_hz: f64,
    pub accelerometer_density_m_s2_sqrt_hz: f64,
    pub gyro_bias_walk_rad_s2_sqrt_hz: f64,
    pub accelerometer_bias_walk_m_s3_sqrt_hz: f64,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IntegrationConfig {
    pub method: IntegrationMethod,
    pub max_gap_s: f64,
    pub max_samples: usize,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegrationMethod {
    Midpoint,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SolverConfig {
    pub max_iterations: usize,
    pub pixel_sigma: f64,
    /// Huber transition in whitened residual units; the pixel threshold is
    /// `pixel_sigma * huber_delta`.
    pub huber_delta: f64,
    /// Hard per-observation rejection threshold in pixels.
    ///
    /// `null` selects Huber-weighted least squares only (no hard pruning) and is
    /// accepted for `continuous_time` runs, matching Basalt's calibration policy.
    /// A missing key keeps the historical 6 px threshold.
    #[serde(default = "default_point_rejection_px")]
    pub point_rejection_px: Option<f64>,
}

const fn default_point_rejection_px() -> Option<f64> {
    Some(6.0)
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 1_000,
            pixel_sigma: 1.0,
            huber_delta: 3.0,
            point_rejection_px: default_point_rejection_px(),
        }
    }
}

impl RunSettings {
    pub fn load(path: &Path) -> Result<Self> {
        let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
        let settings: Self =
            serde_yaml::from_slice(&bytes).with_context(|| format!("parse {}", path.display()))?;
        settings.validate()?;
        Ok(settings)
    }

    pub fn validate(&self) -> Result<()> {
        ensure!(
            !self.source.path.as_os_str().is_empty(),
            "bag path must not be empty"
        );
        ensure!(
            self.source.synchronization.max_delta_ns >= 0,
            "max_delta_ns must be non-negative"
        );
        ensure!(!self.cameras.is_empty(), "select at least one image topic");
        let mut ids = HashSet::with_capacity(self.cameras.len());
        let mut topics = HashSet::with_capacity(self.cameras.len());
        for camera in &self.cameras {
            ensure!(!camera.id.trim().is_empty(), "camera id must not be empty");
            ensure!(
                !camera.topic.trim().is_empty(),
                "camera topic must not be empty"
            );
            ensure!(
                ids.insert(camera.id.as_str()),
                "duplicate camera id {}",
                camera.id
            );
            ensure!(
                topics.insert(camera.topic.as_str()),
                "duplicate camera topic {}",
                camera.topic
            );
        }
        self.validate_target()?;
        self.validate_detector()?;
        self.validate_solver()?;
        self.validate_mode(&ids)
    }

    fn validate_target(&self) -> Result<()> {
        let TargetConfig::Aprilgrid {
            border_bits,
            tag_size_m,
            tag_gap_m,
            id_rows_front_view_left_to_right,
            decoded_corner_to_front_corner,
            ..
        } = &self.target;
        ensure!(
            (1..=64).contains(&id_rows_front_view_left_to_right.len()),
            "target row count must be in 1..=64"
        );
        let columns = id_rows_front_view_left_to_right[0].len();
        ensure!(
            (1..=64).contains(&columns),
            "target column count must be in 1..=64"
        );
        ensure!(
            id_rows_front_view_left_to_right
                .iter()
                .all(|row| row.len() == columns),
            "target ID rows must be rectangular"
        );
        let ids = id_rows_front_view_left_to_right
            .iter()
            .flatten()
            .copied()
            .collect::<HashSet<_>>();
        ensure!(
            ids.len() == id_rows_front_view_left_to_right.len() * columns,
            "target tag IDs must be unique"
        );
        ensure!((1..=2).contains(border_bits), "border_bits must be 1 or 2");
        ensure!(
            tag_size_m.is_finite() && *tag_size_m > 0.0,
            "tag_size_m must be positive and finite"
        );
        ensure!(
            tag_gap_m.is_finite() && *tag_gap_m >= 0.0,
            "tag_gap_m must be finite and non-negative"
        );
        let first = decoded_corner_to_front_corner[0];
        ensure!(
            first < 4
                && decoded_corner_to_front_corner
                    .iter()
                    .enumerate()
                    .all(|(index, &corner)| corner == (first + index) % 4),
            "decoded_corner_to_front_corner must be a cyclic TL/TR/BR/BL permutation"
        );
        Ok(())
    }

    fn validate_detector(&self) -> Result<()> {
        ensure!(
            self.detector.refinement.max_iterations > 0,
            "refinement max_iterations must be positive"
        );
        ensure!(
            self.detector.budgets.max_pixels > 0
                && self.detector.budgets.max_clusters > 0
                && self.detector.budgets.max_candidates > 0,
            "detector budgets must be positive"
        );
        Ok(())
    }

    fn validate_solver(&self) -> Result<()> {
        let solver = self.solver;
        ensure!(
            (1..=1_000).contains(&solver.max_iterations),
            "solver max_iterations must be in 1..=1000"
        );
        for (name, value) in [
            ("pixel_sigma", solver.pixel_sigma),
            ("huber_delta", solver.huber_delta),
        ] {
            ensure!(
                value.is_finite() && value > 0.0,
                "{name} must be positive and finite"
            );
        }
        match solver.point_rejection_px {
            Some(value) => ensure!(
                value.is_finite() && value > 0.0,
                "point_rejection_px must be positive and finite"
            ),
            None => ensure!(
                matches!(
                    &self.calibration,
                    CalibrationConfig::MultiCameraImu { method, .. }
                        if matches!(method, InertialMethod::ContinuousTime)
                ),
                "solver.point_rejection_px = null (Huber-only) is only supported by continuous_time"
            ),
        }
        Ok(())
    }

    #[allow(
        clippy::too_many_lines,
        reason = "mode validation keeps all cross-field physical invariants in one exhaustive match"
    )]
    fn validate_mode(&self, ids: &HashSet<&str>) -> Result<()> {
        let initial_intrinsics = match &self.calibration {
            CalibrationConfig::SingleCamera {
                initial_intrinsics, ..
            }
            | CalibrationConfig::MultiCamera {
                initial_intrinsics, ..
            }
            | CalibrationConfig::MultiCameraImu {
                initial_intrinsics, ..
            } => initial_intrinsics,
        };
        validate_initial_intrinsics(initial_intrinsics)?;
        match &self.calibration {
            CalibrationConfig::SingleCamera { camera, .. } => {
                ensure!(
                    self.cameras.len() == 1,
                    "single_camera requires exactly one selected camera"
                );
                ensure!(
                    ids.contains(camera.as_str()),
                    "unknown single camera {camera}"
                );
                ensure!(
                    self.source.imu_topic.is_none(),
                    "single_camera does not use an IMU topic"
                );
            }
            CalibrationConfig::MultiCamera {
                reference_camera,
                refine_intrinsics,
                max_closure_rotation_deg,
                max_closure_translation_m,
                ..
            } => {
                ensure!(
                    self.cameras.len() >= 2,
                    "multi_camera requires at least two selected cameras"
                );
                ensure!(
                    ids.contains(reference_camera.as_str()),
                    "unknown reference camera {reference_camera}"
                );
                ensure!(
                    self.source.imu_topic.is_none(),
                    "multi_camera does not use an IMU topic"
                );
                ensure!(
                    *refine_intrinsics || initial_intrinsics.path.is_some(),
                    "multi_camera with refine_intrinsics=false requires an initial intrinsic result"
                );
                ensure!(
                    max_closure_rotation_deg.is_finite()
                        && *max_closure_rotation_deg > 0.0
                        && max_closure_translation_m.is_finite()
                        && *max_closure_translation_m > 0.0,
                    "invalid closure limits"
                );
            }
            CalibrationConfig::MultiCameraImu {
                method,
                visual_reference_camera,
                time_offset,
                imu,
                ..
            } => {
                ensure!(
                    ids.contains(visual_reference_camera.as_str()),
                    "unknown visual reference camera {visual_reference_camera}"
                );
                ensure!(
                    self.source
                        .imu_topic
                        .as_ref()
                        .is_some_and(|topic| !topic.trim().is_empty()),
                    "multi_camera_imu requires an IMU topic"
                );
                match time_offset {
                    TimeOffsetConfig::Fixed { value_s } => {
                        ensure!(value_s.is_finite(), "fixed time offset must be finite");
                    }
                    TimeOffsetConfig::Estimate {
                        initial_s,
                        minimum_s,
                        maximum_s,
                        initialization_steps,
                    } => {
                        ensure!(
                            initial_s.is_finite() && minimum_s.is_finite() && maximum_s.is_finite(),
                            "time offset values must be finite"
                        );
                        ensure!(
                            minimum_s < maximum_s
                                && initial_s >= minimum_s
                                && initial_s <= maximum_s,
                            "invalid time offset bounds"
                        );
                        ensure!(
                            (2..=101).contains(initialization_steps),
                            "time offset initialization_steps must be in 2..=101"
                        );
                    }
                }
                imu.axis_calibration.validate()?;
                ensure!(
                    !matches!(*method, InertialMethod::Preintegration)
                        || (!imu.axis_calibration.refine && imu.axis_calibration.is_identity()),
                    "imu.axis_calibration is only used by continuous_time; preintegration requires identity matrices and refine=false"
                );
                ensure!(
                    imu.continuous_time.knot_interval_s.is_finite()
                        && imu.continuous_time.knot_interval_s > 0.0,
                    "IMU continuous_time.knot_interval_s must be positive and finite"
                );
                for value in [
                    imu.gravity_magnitude_m_s2,
                    imu.noise.gyro_density_rad_s_sqrt_hz,
                    imu.noise.accelerometer_density_m_s2_sqrt_hz,
                    imu.noise.gyro_bias_walk_rad_s2_sqrt_hz,
                    imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz,
                    imu.integration.max_gap_s,
                ] {
                    ensure!(
                        value.is_finite() && value > 0.0,
                        "IMU parameters must be positive and finite"
                    );
                }
                ensure!(
                    imu.integration.max_samples >= 2,
                    "IMU max_samples must be at least 2"
                );
            }
        }
        Ok(())
    }

    #[must_use]
    pub fn mode_name(&self) -> &'static str {
        match self.calibration {
            CalibrationConfig::SingleCamera { .. } => "single_camera",
            CalibrationConfig::MultiCamera { .. } => "multi_camera",
            CalibrationConfig::MultiCameraImu { .. } => "multi_camera_imu",
        }
    }
}

fn validate_initial_intrinsics(initial: &InitialIntrinsicsConfig) -> Result<()> {
    if let Some(path) = &initial.path {
        ensure!(
            !path.as_os_str().is_empty(),
            "initial intrinsic result path must not be empty"
        );
    }
    Ok(())
}
