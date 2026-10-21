use std::collections::HashSet;
use std::fs;
use std::path::Path;

use anyhow::{Context, Result, ensure};
use ccl_calib::camera::continuous_time::ImuAxisCalibration;
use ccl_calib::camera::{CameraParameters, TargetGeometry};
use ccl_geometry::Pose3;
use serde::{Deserialize, Serialize};

use crate::config::{RunSettings, TargetConfig};
use crate::run_record::{RunContext, atomic_write, read_yaml};
use crate::runner::SolveSummary;
use crate::trajectory::Trajectory;

/// Version 5 defines KB4 as eight parameters without skew. Version 4 moved
/// inertial bias into the calibrated physical frame and renamed axis matrices.
pub const RESULT_SCHEMA_VERSION: u32 = 5;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CalibrationOutput {
    /// Version 5 removes KB4 skew. Inertial schemas before 4 are rejected;
    /// legacy KB4 payloads containing alpha are rejected by the model decoder.
    pub schema_version: u32,
    pub mode: String,
    #[serde(default = "default_acceptance")]
    pub acceptance: AcceptanceStatus,
    pub cameras: Vec<CameraOutput>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reference_camera: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub camera_from_reference: Vec<NamedPose>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub inertial: Option<InertialOutput>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AcceptanceStatus {
    #[default]
    PendingHumanReview,
    Accepted,
    Rejected,
}

const fn default_acceptance() -> AcceptanceStatus {
    AcceptanceStatus::PendingHumanReview
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CameraOutput {
    pub id: String,
    pub image_width: u32,
    pub image_height: u32,
    pub parameters: CameraParameters,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NamedPose {
    pub direction: String,
    pub pose: PoseOutput,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PoseOutput {
    pub translation_m: [f64; 3],
    pub quaternion_wxyz: [f64; 4],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InertialOutput {
    #[serde(default = "default_inertial_method")]
    pub method: String,
    pub camera_from_imu: Vec<NamedPose>,
    /// Constant bias in the calibrated physical measurement frame:
    /// `physical = axis_calibration * raw - bias`. Preintegration reports raw-frame
    /// bias, which is the same quantity because its axis matrices are identity.
    pub gyro_bias_rad_s: [f64; 3],
    /// Constant bias in the calibrated physical measurement frame.
    pub accelerometer_bias_m_s2: [f64; 3],
    pub gravity_world_m_s2: [f64; 3],
    pub time_offset_s: f64,
    #[serde(default)]
    pub axis_calibration: ImuAxisCalibrationOutput,
}

fn default_inertial_method() -> String {
    "preintegration".to_owned()
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuAxisCalibrationOutput {
    /// Row-major `A` in `physical_gyro = A * raw_gyro - bias`.
    pub gyro_calibrated_from_raw: [[f64; 3]; 3],
    /// Row-major `A` in `physical_specific_force = A * raw_accelerometer - bias`.
    pub accelerometer_calibrated_from_raw: [[f64; 3]; 3],
}

impl Default for ImuAxisCalibrationOutput {
    fn default() -> Self {
        Self {
            gyro_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            accelerometer_calibrated_from_raw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        }
    }
}

impl ImuAxisCalibrationOutput {
    #[must_use]
    const fn core(self) -> ImuAxisCalibration {
        ImuAxisCalibration {
            gyro_calibrated_from_raw: self.gyro_calibrated_from_raw,
            accelerometer_calibrated_from_raw: self.accelerometer_calibrated_from_raw,
        }
    }

    fn validate(&self) -> Result<()> {
        self.core()
            .validate()
            .map_err(|error| anyhow::anyhow!("invalid IMU axis calibration: {error}"))
    }
}

impl CalibrationOutput {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            (1..=RESULT_SCHEMA_VERSION).contains(&self.schema_version),
            "unsupported result schema version"
        );
        ensure!(
            self.schema_version >= 4 || self.inertial.is_none(),
            "schema {} inertial results use obsolete raw-frame bias and axis matrices; recalibrate or read them with an older version",
            self.schema_version
        );
        ensure!(!self.cameras.is_empty(), "result cameras must not be empty");
        let mut ids = HashSet::with_capacity(self.cameras.len());
        for camera in &self.cameras {
            ensure!(!camera.id.is_empty(), "result camera id must not be empty");
            ensure!(
                ids.insert(camera.id.as_str()),
                "duplicate result camera {}",
                camera.id
            );
            ensure!(
                camera.image_width > 0 && camera.image_height > 0,
                "invalid image size for {}",
                camera.id
            );
            camera.parameters.validate()?;
        }
        if let Some(reference) = &self.reference_camera {
            ensure!(
                ids.contains(reference.as_str()),
                "unknown result reference camera {reference}"
            );
        }
        for transform in &self.camera_from_reference {
            transform.validate()?;
        }
        if let Some(inertial) = &self.inertial {
            ensure!(
                matches!(
                    inertial.method.as_str(),
                    "preintegration" | "continuous_time"
                ),
                "unsupported inertial method"
            );
            ensure!(
                inertial.camera_from_imu.len() == self.cameras.len(),
                "camera_from_imu count mismatch"
            );
            for transform in &inertial.camera_from_imu {
                transform.validate()?;
            }
            ensure!(
                inertial
                    .gyro_bias_rad_s
                    .iter()
                    .chain(inertial.accelerometer_bias_m_s2.iter())
                    .chain(inertial.gravity_world_m_s2.iter())
                    .all(|value| value.is_finite()),
                "non-finite inertial result"
            );
            ensure!(inertial.time_offset_s.is_finite(), "non-finite time offset");
            inertial.axis_calibration.validate()?;
        }
        Ok(())
    }

    pub fn camera(&self, id: &str) -> Result<&CameraOutput> {
        self.cameras
            .iter()
            .find(|camera| camera.id == id)
            .with_context(|| format!("prior result has no camera {id}"))
    }

    pub fn pose(&self, direction: &str) -> Result<Pose3> {
        self.camera_from_reference
            .iter()
            .chain(
                self.inertial
                    .iter()
                    .flat_map(|inertial| inertial.camera_from_imu.iter()),
            )
            .find(|entry| entry.direction == direction)
            .with_context(|| format!("prior result has no transform {direction}"))?
            .pose
            .to_pose()
    }
}

impl NamedPose {
    fn validate(&self) -> Result<()> {
        ensure!(
            !self.direction.is_empty(),
            "pose direction must not be empty"
        );
        self.pose.to_pose().map(|_| ())
    }
}

impl PoseOutput {
    #[must_use]
    pub fn from_pose(pose: Pose3) -> Self {
        Self {
            translation_m: pose.translation(),
            quaternion_wxyz: rotation_to_quaternion(pose.rotation()),
        }
    }

    pub fn to_pose(self) -> Result<Pose3> {
        let [w, x, y, z] = self.quaternion_wxyz;
        ensure!(
            [w, x, y, z].iter().all(|value| value.is_finite()),
            "non-finite quaternion"
        );
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        ensure!(
            (norm - 1.0).abs() <= 1.0e-6,
            "quaternion is not unit length"
        );
        ensure!(
            self.translation_m.iter().all(|value| value.is_finite()),
            "non-finite translation"
        );
        let rotation = [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ];
        Pose3::try_new(rotation, self.translation_m).context("invalid pose")
    }
}

pub fn load_result(path: &Path) -> Result<CalibrationOutput> {
    let result: CalibrationOutput = read_yaml(path)?;
    result.validate()?;
    Ok(result)
}

/// Actual solved target geometry for the offline viewer.
///
/// Points come from the same `TargetGeometry::april_grid` constructor the solver
/// consumes, with tag IDs in the run's row-major cell order (cell `n` owns points
/// `4n..4n + 4`), so the drawing cannot drift from the corner correspondence used
/// by detection and the solvers. `TargetGeometry::april_grid` orders each cell's
/// corners `(0,0), (s,0), (s,s), (0,s)` in the board x/y plane at z = 0.
///
/// The board geometry frame and `Trajectory::frame` coincide for every mode: pure
/// visual trajectories are `board`-from-body, while both inertial solvers treat the
/// target points themselves as world coordinates (`continuous_time` projects
/// `object - T_world_from_imu.position`, `visual_inertial` inverse-transforms them
/// with the world pose), so the inertial `world` frame is the static board frame
/// and the board sits at identity in it.
fn board_payload(config: &RunSettings, frame: &str) -> Result<serde_json::Value> {
    let TargetConfig::Aprilgrid {
        tag_size_m,
        tag_gap_m,
        id_rows_front_view_left_to_right,
        ..
    } = &config.target;
    let rows = u16::try_from(id_rows_front_view_left_to_right.len())
        .context("target row count exceeds u16")?;
    let columns = id_rows_front_view_left_to_right
        .first()
        .context("target has no ID rows")?
        .len();
    let columns = u16::try_from(columns).context("target column count exceeds u16")?;
    let geometry = TargetGeometry::april_grid(rows, columns, *tag_size_m, *tag_gap_m)
        .context("build target geometry for visualization")?;
    let points = geometry.points();
    let mut tags = Vec::with_capacity(points.len() / 4);
    for (cell, &id) in id_rows_front_view_left_to_right
        .iter()
        .flatten()
        .enumerate()
    {
        let corners = points
            .get(cell * 4..cell * 4 + 4)
            .context("target geometry does not cover every configured tag")?;
        tags.push(serde_json::json!({"id": id, "corners_m": corners}));
    }
    let mut gap_squares = Vec::new();
    for row in 0..usize::from(rows).saturating_sub(1) {
        for column in 0..usize::from(columns).saturating_sub(1) {
            let cell = row * usize::from(columns) + column;
            let a = points[cell * 4 + 2];
            let b = points[(cell + 1) * 4 + 3];
            let c = points[(cell + usize::from(columns) + 1) * 4];
            let d = points[(cell + usize::from(columns)) * 4 + 1];
            if *tag_gap_m > 0.0 {
                gap_squares.push([a, b, c, d]);
            }
        }
    }
    Ok(serde_json::json!({"frame": frame, "tags": tags, "gap_squares_m": gap_squares}))
}

fn extrinsics_html(
    output: &CalibrationOutput,
    summary: &SolveSummary,
    trajectory: &Trajectory,
    board: &serde_json::Value,
) -> Result<String> {
    let reference = if output.inertial.is_some() {
        "imu"
    } else {
        output
            .reference_camera
            .as_deref()
            .unwrap_or(&output.cameras[0].id)
    };
    let mut poses = Vec::with_capacity(output.cameras.len());
    for camera in &output.cameras {
        let pose = if output.inertial.is_none() && output.reference_camera.is_none() {
            ensure!(
                output.cameras.len() == 1,
                "multi-camera visualization requires a reference"
            );
            Pose3::identity()
        } else {
            output.pose(&format!("T_{}_from_{reference}", camera.id))?
        };
        let r = pose.rotation();
        let t = pose.translation();
        let rotation: [[f64; 3]; 3] = std::array::from_fn(|i| std::array::from_fn(|j| r[j][i]));
        let position_mm: [f64; 3] =
            std::array::from_fn(|i| -1000.0 * (0..3).map(|j| rotation[i][j] * t[j]).sum::<f64>());
        poses.push(
            serde_json::json!({"id": camera.id, "position_mm": position_mm, "rotation": rotation}),
        );
    }
    let data = serde_json::to_string(&serde_json::json!({
        "reference": reference, "has_imu": output.inertial.is_some(),
        "acceptance": output.acceptance, "poses": poses,
        "calibration": output, "summary": summary, "trajectory": trajectory,
        "board": board,
    }))?;
    // JSON is embedded in a script element; sensor IDs must never close that element.
    let data = data
        .replace('<', "\\u003c")
        .replace('>', "\\u003e")
        .replace('&', "\\u0026");
    Ok(include_str!("extrinsics.html").replace("__DATA__", &data))
}

pub fn publish(run: &RunContext, output: &CalibrationOutput, summary: &SolveSummary) -> Result<()> {
    output.validate()?;
    let trajectory_bytes =
        fs::read(run.root().join("trajectory.json")).context("read final solved trajectory")?;
    let trajectory: Trajectory =
        serde_json::from_slice(&trajectory_bytes).context("parse final solved trajectory")?;
    // The run's immutable `config.yaml` snapshot is the only source for board geometry.
    let settings: RunSettings = read_yaml(&run.root().join("config.yaml"))
        .context("read run settings for visualization")?;
    settings
        .validate()
        .context("validate run settings snapshot for visualization")?;
    let board = board_payload(&settings, &trajectory.frame)?;
    let temporary = run.root().join(".result.tmp");
    let final_path = run.root().join("result");
    ensure!(!final_path.exists(), "result directory already exists");
    if temporary.exists() {
        fs::remove_dir_all(&temporary)
            .with_context(|| format!("remove stale {}", temporary.display()))?;
    }
    fs::create_dir(&temporary).with_context(|| format!("create {}", temporary.display()))?;
    let publish_result = (|| {
        let yaml = serde_yaml::to_string(output).context("serialize calibration result")?;
        atomic_write(&temporary.join("calibration.yaml"), yaml.as_bytes())?;
        atomic_write(&temporary.join("trajectory.json"), &trajectory_bytes)?;
        let summary_json = serde_json::to_vec_pretty(summary).context("serialize solve summary")?;
        atomic_write(&temporary.join("summary.json"), &summary_json)?;
        let html = extrinsics_html(output, summary, &trajectory, &board)
            .context("render calibration visualization")?;
        atomic_write(&temporary.join("extrinsics_3d.html"), html.as_bytes())?;
        fs::rename(&temporary, &final_path)
            .with_context(|| format!("publish {}", final_path.display()))?;
        Ok(())
    })();
    if publish_result.is_err() {
        let _ = fs::remove_dir_all(&temporary);
    }
    publish_result
}

fn rotation_to_quaternion(rotation: [[f64; 3]; 3]) -> [f64; 4] {
    let trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    let (mut w, mut x, mut y, mut z) = if trace > 0.0 {
        let scale = (trace + 1.0).sqrt() * 2.0;
        (
            0.25 * scale,
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        )
    } else if rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2] {
        let scale = (1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]).sqrt() * 2.0;
        (
            (rotation[2][1] - rotation[1][2]) / scale,
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
        )
    } else if rotation[1][1] > rotation[2][2] {
        let scale = (1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]).sqrt() * 2.0;
        (
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
        )
    } else {
        let scale = (1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]).sqrt() * 2.0;
        (
            (rotation[1][0] - rotation[0][1]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale,
        )
    };
    let norm = (w * w + x * x + y * y + z * z).sqrt();
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
    if w < 0.0 {
        [-w, -x, -y, -z]
    } else {
        [w, x, y, z]
    }
}
