use std::fs;
use std::path::Path;

use ccl_calib::camera::{CameraModel, CameraParameters, PinholeModel};

use crate::export::{AcceptanceStatus, CalibrationOutput, NamedPose, load_result};

use super::task::{self, Task};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RawSource {
    CalibrationYaml,
    Report,
    SolveYaml,
    RunLog,
}

impl RawSource {
    pub const ALL: [Self; 4] = [
        Self::CalibrationYaml,
        Self::Report,
        Self::SolveYaml,
        Self::RunLog,
    ];

    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::CalibrationYaml => "result/calibration.yaml",
            Self::Report => "report.txt",
            Self::SolveYaml => "solve.yaml",
            Self::RunLog => "run.log",
        }
    }

    #[must_use]
    pub fn next(self, delta: isize) -> Self {
        let index = Self::ALL
            .iter()
            .position(|source| *source == self)
            .unwrap_or(0);
        let count = Self::ALL.len();
        let target = if delta < 0 {
            (index + count - 1) % count
        } else {
            (index + 1) % count
        };
        Self::ALL[target]
    }
}

#[derive(Clone, Debug)]
pub struct ResultData {
    pub output: Option<CalibrationOutput>,
    pub output_error: Option<String>,
    calibration_yaml: String,
    report: String,
    solve_yaml: String,
    run_log: String,
}

impl ResultData {
    #[must_use]
    pub fn load(root: &Path) -> Self {
        let calibration_path = root.join("result/calibration.yaml");
        let (output, output_error) = match load_result(&calibration_path) {
            Ok(output) => (Some(output), None),
            Err(error) => (None, Some(format!("{error:#}"))),
        };
        Self {
            output,
            output_error,
            calibration_yaml: read_text(&calibration_path),
            report: read_text(&root.join("report.txt")),
            solve_yaml: read_text(&root.join("solve.yaml")),
            run_log: read_text(&root.join("run.log")),
        }
    }

    #[must_use]
    pub fn overview_text(&self, task: &Task) -> String {
        let mut lines = vec![
            task::result_summary_text(task),
            task::settings_snapshot_text(&task.settings),
        ];
        lines.push(String::new());
        lines.push("产物:".to_owned());
        for (relative, path) in task::artifact_paths(&task.run_root) {
            lines.push(format!(
                "  {relative}: {}{}",
                path.display(),
                if path.exists() { "" } else { "（未生成）" }
            ));
        }
        lines.push(String::new());
        match &self.output {
            Some(output) => {
                lines.push(format!(
                    "结果: mode={}  schema={}  验收={}  相机={}  外参={}  IMU={}",
                    output.mode,
                    output.schema_version,
                    acceptance_label(output.acceptance),
                    output.cameras.len(),
                    output.camera_from_reference.len(),
                    if output.inertial.is_some() {
                        "有"
                    } else {
                        "无"
                    }
                ));
                for camera in &output.cameras {
                    lines.push(format!(
                        "  {}: {}×{}  {}",
                        camera.id,
                        camera.image_width,
                        camera.image_height,
                        camera_parameter_kind(&camera.parameters)
                    ));
                }
            }
            None => lines.push(format!(
                "未加载结果: {}",
                self.output_error.as_deref().unwrap_or("未知错误")
            )),
        }
        lines.join("\n")
    }

    #[must_use]
    pub fn parameters_text(&self) -> String {
        let Some(output) = &self.output else {
            return format!(
                "无法加载 result/calibration.yaml:\n{}",
                self.output_error.as_deref().unwrap_or("未知错误")
            );
        };
        let mut lines = vec![
            "结果参数表（来源: result/calibration.yaml）".to_owned(),
            format!("  {:<12} {}", "mode", output.mode),
            format!(
                "  {:<12} {}",
                "acceptance",
                acceptance_label(output.acceptance)
            ),
            format!("  {:<12} {}", "schema_version", output.schema_version),
            String::new(),
            "相机概览".to_owned(),
            format!(
                "  {:<8} {:<12} {:<20} {:>12} {:>12} {:>10} {:>10}",
                "camera", "size", "model", "fx", "fy", "cx", "cy"
            ),
        ];
        for camera in &output.cameras {
            let (fx, fy, cx, cy) = focal_center(&camera.parameters);
            lines.push(format!(
                "  {:<8} {:<12} {:<20} {:>12.6} {:>12.6} {:>10.6} {:>10.6}",
                camera.id,
                format!("{}×{}", camera.image_width, camera.image_height),
                camera_parameter_kind(&camera.parameters),
                fx,
                fy,
                cx,
                cy
            ));
        }
        for camera in &output.cameras {
            lines.push(String::new());
            lines.push(format!(
                "== {}  {}×{}  {} ==",
                camera.id,
                camera.image_width,
                camera.image_height,
                camera_parameter_kind(&camera.parameters)
            ));
            lines.push(format!("  {:<12} {:>18}", "参数", "值"));
            for (name, value) in parameter_rows(&camera.parameters) {
                lines.push(format!("  {name:<12} {value:>18}"));
            }
        }
        if !output.camera_from_reference.is_empty() {
            lines.push(String::new());
            lines.push(format!(
                "参考相机: {}（外参见 Extrinsics 页）",
                output.reference_camera.as_deref().unwrap_or("-")
            ));
        }
        lines.join("\n")
    }

    #[must_use]
    pub fn extrinsics_text(&self) -> String {
        let Some(output) = &self.output else {
            return format!(
                "无法加载 result/calibration.yaml:\n{}",
                self.output_error.as_deref().unwrap_or("未知错误")
            );
        };
        let mut lines = vec![format!(
            "mode={}  验收={}",
            output.mode,
            acceptance_label(output.acceptance)
        )];
        if output.camera_from_reference.is_empty() {
            lines.push("相机间外参: 单相机模式（无）".to_owned());
        } else {
            lines.push(format!(
                "相机间外参（reference = {}）:",
                output.reference_camera.as_deref().unwrap_or("-")
            ));
            for pose in &output.camera_from_reference {
                lines.push(format_pose(pose));
            }
        }
        match &output.inertial {
            Some(inertial) => {
                lines.push(String::new());
                lines.push(format!("IMU 外参/状态（method={}）:", inertial.method));
                for pose in &inertial.camera_from_imu {
                    lines.push(format_pose(pose));
                }
                lines.push(format!(
                    "gyro bias [rad/s]: [{:.6}, {:.6}, {:.6}]",
                    inertial.gyro_bias_rad_s[0],
                    inertial.gyro_bias_rad_s[1],
                    inertial.gyro_bias_rad_s[2]
                ));
                lines.push(format!(
                    "accelerometer bias [m/s²]: [{:.6}, {:.6}, {:.6}]",
                    inertial.accelerometer_bias_m_s2[0],
                    inertial.accelerometer_bias_m_s2[1],
                    inertial.accelerometer_bias_m_s2[2]
                ));
                lines.push(format!(
                    "gravity world [m/s²]: [{:.6}, {:.6}, {:.6}]",
                    inertial.gravity_world_m_s2[0],
                    inertial.gravity_world_m_s2[1],
                    inertial.gravity_world_m_s2[2]
                ));
                lines.push(format!("time_offset_s: {:.9}", inertial.time_offset_s));
                lines.push("轴矩阵 physical = A * raw - bias:".to_owned());
                lines.push(format!(
                    "  gyro A: {}",
                    matrix_text(&inertial.axis_calibration.gyro_calibrated_from_raw)
                ));
                lines.push(format!(
                    "  accel A: {}",
                    matrix_text(&inertial.axis_calibration.accelerometer_calibrated_from_raw)
                ));
            }
            None => lines.push("IMU 外参: 无（视觉模式）".to_owned()),
        }
        lines.join("\n")
    }

    #[must_use]
    pub fn diagnostics_text(&self, task: &Task) -> String {
        let mut lines = Vec::new();
        append_run_diagnostics(&mut lines, task);
        append_result_diagnostics(&mut lines, self);
        append_detection_diagnostics(&mut lines, task);
        append_solve_diagnostics(&mut lines, task);
        append_error_diagnostics(&mut lines, task);
        append_raw_diagnostics(&mut lines, task);
        lines.join("\n")
    }

    #[must_use]
    pub fn raw_text(&self, source: RawSource) -> String {
        let value = match source {
            RawSource::CalibrationYaml => &self.calibration_yaml,
            RawSource::Report => &self.report,
            RawSource::SolveYaml => &self.solve_yaml,
            RawSource::RunLog => &self.run_log,
        };
        value.clone()
    }

    #[must_use]
    pub fn raw_path(root: &Path, source: RawSource) -> std::path::PathBuf {
        match source {
            RawSource::CalibrationYaml => root.join("result/calibration.yaml"),
            RawSource::Report => root.join("report.txt"),
            RawSource::SolveYaml => root.join("solve.yaml"),
            RawSource::RunLog => root.join("run.log"),
        }
    }

    #[must_use]
    pub fn has_html(root: &Path) -> bool {
        root.join("result/extrinsics_3d.html").is_file()
    }
}

fn read_text(path: &Path) -> String {
    fs::read_to_string(path).unwrap_or_else(|error| format!("无法读取 {}: {error}", path.display()))
}

fn kv_header() -> String {
    format!("  {:<12} {}", "项目", "值")
}

fn kv_rows(rows: &[(&str, String)]) -> Vec<String> {
    let mut lines = Vec::new();
    for (label, value) in rows {
        let mut parts = value.lines();
        lines.push(format!("  {label:<12} {}", parts.next().unwrap_or("")));
        for line in parts {
            lines.push(format!("  {:<12} {line}", ""));
        }
    }
    lines
}

fn append_run_diagnostics(lines: &mut Vec<String>, task: &Task) {
    lines.push("运行摘要".to_owned());
    lines.push(kv_header());
    lines.extend(kv_rows(&[
        ("运行状态", run_status_label(task)),
        (
            "阶段",
            task.failure_stage
                .clone()
                .unwrap_or_else(|| task::stage_label(task.current_stage()).to_owned()),
        ),
        (
            "总用时",
            task::format_duration(task.run_elapsed_at(std::time::Instant::now())),
        ),
        ("运行目录", task.run_root.display().to_string()),
    ]));
}

fn append_result_diagnostics(lines: &mut Vec<String>, data: &ResultData) {
    lines.push(String::new());
    lines.push("结果".to_owned());
    lines.push(kv_header());
    match &data.output {
        Some(output) => lines.extend(kv_rows(&[
            ("状态", "已加载 result/calibration.yaml".to_owned()),
            ("schema", output.schema_version.to_string()),
            ("mode", output.mode.clone()),
            ("验收", acceptance_label(output.acceptance).to_owned()),
            ("相机", output.cameras.len().to_string()),
            ("相机间外参", output.camera_from_reference.len().to_string()),
            (
                "IMU",
                if output.inertial.is_some() {
                    "有".to_owned()
                } else {
                    "无".to_owned()
                },
            ),
        ])),
        None => lines.extend(kv_rows(&[(
            "加载失败",
            data.output_error
                .as_deref()
                .unwrap_or("未知错误")
                .to_owned(),
        )])),
    }
}

fn append_detection_diagnostics(lines: &mut Vec<String>, task: &Task) {
    let Some(summary) = &task.dataset_summary else {
        return;
    };
    lines.push(String::new());
    lines.push("检测".to_owned());
    lines.push(kv_header());
    lines.extend(kv_rows(&[
        ("同步视图", summary.input_views.to_string()),
        ("求解选用", summary.usable_views.to_string()),
        ("忽略标签", summary.ignored_tags.to_string()),
        ("细化失败", summary.refinement_failures.to_string()),
    ]));
}

fn append_solve_diagnostics(lines: &mut Vec<String>, task: &Task) {
    if let Some(summary) = &task.solve_summary {
        lines.push(String::new());
        lines.push("求解".to_owned());
        lines.push(kv_header());
        lines.extend(kv_rows(&[
            ("终止", summary.termination.clone()),
            (
                "数值收敛",
                if summary.converged { "是" } else { "否" }.to_owned(),
            ),
            ("迭代", summary.iterations.to_string()),
            (
                "cost",
                format!("{:.3e} → {:.3e}", summary.initial_cost, summary.final_cost),
            ),
            (
                "信息矩阵",
                format!(
                    "rank={}/{}  condition={:.3e}  covariance={}",
                    summary.information_rank,
                    summary.information_parameters,
                    summary.condition_number,
                    if summary.covariance_is_pseudoinverse {
                        "伪逆"
                    } else {
                        "全秩逆"
                    }
                ),
            ),
        ]));
        append_camera_residuals(lines, summary);
        if let Some(rms) = summary.inertial_whitened_rms {
            lines.push(format!("  IMU whitened RMS: {rms:.3}"));
        }
        lines.push(String::new());
        lines.push(
            "数值诊断只描述本次求解；是否接受结果必须人工审查，应用不会自动否决。".to_owned(),
        );
    } else if let Some(last) = task.solve_progress.last() {
        lines.push(String::new());
        lines.push("求解停止快照".to_owned());
        lines.push(kv_header());
        lines.extend(kv_rows(&[
            (
                "迭代",
                format!("{}/{}", last.iteration, last.max_iterations),
            ),
            ("cost", format!("{:.3e}", last.cost)),
            ("λ", format!("{:.3e}", last.damping)),
            ("|g|∞", format!("{:.3e}", last.scaled_gradient_inf_norm)),
            (
                "接受/拒绝",
                format!("{}/{}", last.accepted_steps, last.rejected_steps),
            ),
        ]));
    }
}

fn append_camera_residuals(lines: &mut Vec<String>, summary: &crate::runner::SolveSummary) {
    if summary.camera_errors.is_empty() {
        return;
    }
    lines.push(String::new());
    lines.push("相机残差".to_owned());
    lines.push(format!(
        "  {:<8} {:>12} {:>12} {:>10} {:>10}",
        "camera", "RMS(px)", "P95(px)", "used", "rejected"
    ));
    for camera in &summary.camera_errors {
        lines.push(format!(
            "  {:<8} {:>12.3} {:>12.3} {:>10} {:>10}",
            format!("cam{}", camera.camera_index),
            camera.rms_px,
            camera.p95_px,
            camera.used_points,
            camera.rejected_points
        ));
    }
}

fn append_error_diagnostics(lines: &mut Vec<String>, task: &Task) {
    let Some(error) = &task.failure_error else {
        return;
    };
    lines.push(String::new());
    lines.push("错误摘要".to_owned());
    lines.push(kv_header());
    lines.extend(kv_rows(&[
        (
            "失败阶段",
            task.failure_stage
                .clone()
                .unwrap_or_else(|| "未知".to_owned()),
        ),
        (
            "错误类别",
            error.lines().next().unwrap_or("未知错误").to_owned(),
        ),
        ("诊断建议", task::failure_guidance(error).to_owned()),
    ]));
}

fn append_raw_diagnostics(lines: &mut Vec<String>, task: &Task) {
    lines.push(String::new());
    lines.push("原始诊断文件（完整错误链与日志保留在此）".to_owned());
    lines.push(kv_header());
    lines.extend(kv_rows(&[
        (
            "solve.yaml",
            exists_label(&task.run_root.join("solve.yaml")).to_owned(),
        ),
        (
            "report.txt",
            exists_label(&task.run_root.join("report.txt")).to_owned(),
        ),
        (
            "run.log",
            exists_label(&task.run_root.join("run.log")).to_owned(),
        ),
    ]));
}

fn run_status_label(task: &Task) -> String {
    let label = match task.final_status {
        Some(crate::run_record::RunStatus::Complete) => "成功",
        Some(crate::run_record::RunStatus::Failed) => "失败",
        Some(crate::run_record::RunStatus::Cancelled) => "已取消",
        Some(crate::run_record::RunStatus::ClosedWithoutCalibration) => "未执行",
        Some(crate::run_record::RunStatus::InProgress) | None => "进行中",
    };
    label.to_owned()
}

fn focal_center(parameters: &CameraParameters) -> (f64, f64, f64, f64) {
    match parameters {
        CameraParameters::Pinhole(parameters) => (
            parameters.fx(),
            parameters.fy(),
            parameters.cx(),
            parameters.cy(),
        ),
        CameraParameters::KannalaBrandt4(parameters) => {
            (parameters.fx, parameters.fy, parameters.cx, parameters.cy)
        }
        CameraParameters::ExtendedUnified(parameters) => {
            (parameters.fx, parameters.fy, parameters.cx, parameters.cy)
        }
        CameraParameters::DoubleSphere(parameters) => {
            (parameters.fx, parameters.fy, parameters.cx, parameters.cy)
        }
    }
}

fn parameter_rows(parameters: &CameraParameters) -> Vec<(&'static str, String)> {
    let value = |value: f64| format!("{value:.9}");
    match parameters {
        CameraParameters::Pinhole(parameters) => {
            let mut rows = vec![
                ("fx", value(parameters.fx())),
                ("fy", value(parameters.fy())),
                ("cx", value(parameters.cx())),
                ("cy", value(parameters.cy())),
                (
                    "distortion",
                    parameters
                        .model()
                        .id()
                        .trim_start_matches("pinhole-")
                        .to_owned(),
                ),
            ];
            for (name, coefficient) in parameters
                .model()
                .distortion_names()
                .iter()
                .zip(parameters.active_coefficients())
            {
                rows.push((*name, value(coefficient)));
            }
            rows
        }
        CameraParameters::KannalaBrandt4(parameters) => vec![
            ("fx", value(parameters.fx)),
            ("fy", value(parameters.fy)),
            ("cx", value(parameters.cx)),
            ("cy", value(parameters.cy)),
            ("k1", value(parameters.k1)),
            ("k2", value(parameters.k2)),
            ("k3", value(parameters.k3)),
            ("k4", value(parameters.k4)),
        ],
        CameraParameters::ExtendedUnified(parameters) => vec![
            ("fx", value(parameters.fx)),
            ("fy", value(parameters.fy)),
            ("cx", value(parameters.cx)),
            ("cy", value(parameters.cy)),
            ("alpha", value(parameters.alpha)),
            ("beta", value(parameters.beta)),
        ],
        CameraParameters::DoubleSphere(parameters) => vec![
            ("fx", value(parameters.fx)),
            ("fy", value(parameters.fy)),
            ("cx", value(parameters.cx)),
            ("cy", value(parameters.cy)),
            ("xi", value(parameters.xi)),
            ("alpha", value(parameters.alpha)),
        ],
    }
}

fn exists_label(path: &Path) -> &'static str {
    if path.is_file() {
        "存在"
    } else {
        "未生成"
    }
}

fn acceptance_label(acceptance: AcceptanceStatus) -> &'static str {
    match acceptance {
        AcceptanceStatus::PendingHumanReview => "待人工审查",
        AcceptanceStatus::Accepted => "已接受",
        AcceptanceStatus::Rejected => "已拒绝",
    }
}

fn camera_parameter_kind(parameters: &ccl_calib::camera::CameraParameters) -> &'static str {
    match parameters.model() {
        CameraModel::Pinhole(PinholeModel::None) => "PinholeNone",
        CameraModel::Pinhole(PinholeModel::Radtan5) => "PinholeRadtan5",
        CameraModel::Pinhole(PinholeModel::Radtan8) => "PinholeRadtan8",
        CameraModel::Pinhole(PinholeModel::D12) => "PinholeD12",
        CameraModel::Pinhole(_) => "Pinhole(其它畸变组合)",
        CameraModel::KannalaBrandt4 => "KannalaBrandt4",
        CameraModel::DoubleSphere => "DoubleSphere",
        CameraModel::ExtendedUnified => "ExtendedUnified",
    }
}

fn format_pose(pose: &NamedPose) -> String {
    let translation = pose.pose.translation_m;
    let quaternion = pose.pose.quaternion_wxyz;
    format_pose_parts(&pose.direction, translation, quaternion)
}

fn format_pose_parts(direction: &str, translation: [f64; 3], quaternion: [f64; 4]) -> String {
    let norm = (translation[0] * translation[0]
        + translation[1] * translation[1]
        + translation[2] * translation[2])
        .sqrt();
    format!(
        "  {direction}: t=[{:.6}, {:.6}, {:.6}] m  q(wxyz)=[{:.6}, {:.6}, {:.6}, {:.6}]  |t|={norm:.6} m",
        translation[0],
        translation[1],
        translation[2],
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3]
    )
}

fn matrix_text(matrix: &[[f64; 3]; 3]) -> String {
    matrix
        .iter()
        .map(|row| format!("[{:.6}, {:.6}, {:.6}]", row[0], row[1], row[2]))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use super::RawSource;

    #[test]
    fn raw_sources_cycle_forward_and_backward() {
        assert_eq!(RawSource::CalibrationYaml.next(1), RawSource::Report);
        assert_eq!(RawSource::Report.next(-1), RawSource::CalibrationYaml);
        assert_eq!(RawSource::RunLog.next(1), RawSource::CalibrationYaml);
        assert_eq!(RawSource::CalibrationYaml.next(-1), RawSource::RunLog);
    }
}
