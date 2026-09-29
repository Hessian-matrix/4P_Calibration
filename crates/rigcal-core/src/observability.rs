//! 数值可观测性：信息矩阵 → 协方差 → 参数不确定度与相关性。
//!
//! 视图位姿按 Schur 补消去（nuisance 参数），秩/条件数在**相关归一化**矩阵上判定——纯标度矩阵
//! 会被 `alpha`/`xi` 这类高灵敏参数的动态范围压垮，把量纲差误读成秩亏。

use nalgebra::{DMatrix, DVector};

use crate::estimator::Observation;
use crate::models::{self, ModelKind, Parameters};

const MATRIX_DAMPING: f64 = 1.0e-9;
const CENTRAL_DIFF_STEP: f64 = 1.0e-6;
const POSE_REQUIRED: usize = 6;
const RANK_RELATIVE_TOLERANCE: f64 = 1.0e-10;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ObservabilityError {
    EmptyViews,
    NonFiniteImagePoints,
    RankDeficient { rank: usize, parameters: usize },
    NoUsableView,
    PoseInformationSingular,
}

impl std::fmt::Display for ObservabilityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ObservabilityError::EmptyViews => write!(f, "observability requires at least one view"),
            ObservabilityError::NonFiniteImagePoints => {
                write!(f, "observation contains non-finite image points")
            }
            ObservabilityError::RankDeficient { rank, parameters } => write!(
                f,
                "intrinsic information matrix is rank deficient ({rank}/{parameters}); \
                 collect more diverse poses before judging convergence"
            ),
            ObservabilityError::NoUsableView => {
                write!(
                    f,
                    "no view has usable projections for the information matrix"
                )
            }
            ObservabilityError::PoseInformationSingular => {
                write!(f, "view pose information is singular")
            }
        }
    }
}

impl std::error::Error for ObservabilityError {}

#[derive(Clone, Debug)]
pub struct IntrinsicObservability {
    pub model: ModelKind,
    pub names: Vec<&'static str>,
    pub view_count: usize,
    pub used_view_count: usize,
    pub skipped_view_count: usize,
    pub point_count: usize,
    pub rms_px: f64,
    pub rank: usize,
    pub condition_number: f64,
    pub log_det_information: f64,
    pub info_gain: Option<f64>,
    /// fx/fy 已归一化为相对值（除以 |f|），其余为绝对值。
    pub parameter_stddev: Vec<f64>,
    pub parameter_scales: Vec<f64>,
    pub parameter_correlation: Vec<Vec<f64>>,
}

impl IntrinsicObservability {
    pub fn focal_relative_stddev(&self) -> (f64, f64) {
        let fx = self.names.iter().position(|name| *name == "fx").unwrap();
        let fy = self.names.iter().position(|name| *name == "fy").unwrap();
        (self.parameter_stddev[fx], self.parameter_stddev[fy])
    }

    pub fn principal_stddev_px(&self) -> (f64, f64) {
        let cx = self.names.iter().position(|name| *name == "cx").unwrap();
        let cy = self.names.iter().position(|name| *name == "cy").unwrap();
        (self.parameter_stddev[cx], self.parameter_stddev[cy])
    }

    pub fn goal_met(
        &self,
        max_focal_relative_stddev: f64,
        max_principal_stddev_px: f64,
        max_rms_px: f64,
    ) -> bool {
        let (fx_sigma, fy_sigma) = self.focal_relative_stddev();
        let (cx_sigma, cy_sigma) = self.principal_stddev_px();
        self.rank == self.names.len()
            && self.rms_px.is_finite()
            && self.rms_px <= max_rms_px
            && fx_sigma.max(fy_sigma) <= max_focal_relative_stddev
            && cx_sigma.max(cy_sigma) <= max_principal_stddev_px
    }
}

pub fn parameter_scales(kind: ModelKind, params: &Parameters) -> Vec<f64> {
    let values = params.as_vector();
    kind.parameter_names()
        .iter()
        .enumerate()
        .map(|(index, name)| {
            if *name == "fx" || *name == "fy" {
                values[index].abs().max(1.0)
            } else {
                1.0
            }
        })
        .collect()
}

fn project_object(
    kind: ModelKind,
    params: &Parameters,
    object_points: &[[f64; 3]],
    rvec: &[f64; 3],
    tvec: &[f64; 3],
) -> (Vec<[f64; 2]>, Vec<bool>) {
    let camera_points = models::transform_points(object_points, rvec, tvec);
    models::project(kind, &camera_points, params)
}

/// 内参雅可比：中心差分，列 = 仅扰第 i 个内参引起的像素变化 /(2·step)（含「除以未缩放步长」
/// 这一约定）。任一扰动越出有效域即整视图不参与（返回 None）。
fn intrinsic_jacobian(
    kind: ModelKind,
    params: &Parameters,
    observation: &Observation,
    rvec: &[f64; 3],
    tvec: &[f64; 3],
    scales: &[f64],
) -> Option<DMatrix<f64>> {
    let base = params.as_vector();
    let rows = observation.image_points.len() * 2;
    let mut matrix = DMatrix::zeros(rows, base.len());
    for index in 0..base.len() {
        let step = scales[index] * CENTRAL_DIFF_STEP;
        let mut plus = base.clone();
        plus[index] += step;
        let mut minus = base.clone();
        minus[index] -= step;
        let plus_params = Parameters::from_vector(kind, &plus)?;
        let minus_params = Parameters::from_vector(kind, &minus)?;
        let (plus_pixels, plus_valid) =
            project_object(kind, &plus_params, &observation.object_points, rvec, tvec);
        let (minus_pixels, minus_valid) =
            project_object(kind, &minus_params, &observation.object_points, rvec, tvec);
        if !plus_valid.iter().all(|value| *value) || !minus_valid.iter().all(|value| *value) {
            return None;
        }
        for point in 0..observation.image_points.len() {
            matrix[(2 * point, index)] =
                (plus_pixels[point][0] - minus_pixels[point][0]) / (2.0 * CENTRAL_DIFF_STEP);
            matrix[(2 * point + 1, index)] =
                (plus_pixels[point][1] - minus_pixels[point][1]) / (2.0 * CENTRAL_DIFF_STEP);
        }
    }
    Some(matrix)
}

fn pose_jacobian(
    kind: ModelKind,
    params: &Parameters,
    observation: &Observation,
    rvec: &[f64; 3],
    tvec: &[f64; 3],
) -> Option<DMatrix<f64>> {
    let rows = observation.image_points.len() * 2;
    let mut matrix = DMatrix::zeros(rows, POSE_REQUIRED);
    for index in 0..3 {
        let step = CENTRAL_DIFF_STEP;
        let mut plus = *rvec;
        let mut minus = *rvec;
        plus[index] += step;
        minus[index] -= step;
        let (plus_pixels, plus_valid) =
            project_object(kind, params, &observation.object_points, &plus, tvec);
        let (minus_pixels, minus_valid) =
            project_object(kind, params, &observation.object_points, &minus, tvec);
        if !plus_valid.iter().all(|value| *value) || !minus_valid.iter().all(|value| *value) {
            return None;
        }
        for point in 0..observation.image_points.len() {
            matrix[(2 * point, index)] =
                (plus_pixels[point][0] - minus_pixels[point][0]) / (2.0 * step);
            matrix[(2 * point + 1, index)] =
                (plus_pixels[point][1] - minus_pixels[point][1]) / (2.0 * step);
        }
    }
    for index in 0..3 {
        let step = tvec[index].abs().max(1.0) * CENTRAL_DIFF_STEP;
        let mut plus = *tvec;
        let mut minus = *tvec;
        plus[index] += step;
        minus[index] -= step;
        let (plus_pixels, plus_valid) =
            project_object(kind, params, &observation.object_points, rvec, &plus);
        let (minus_pixels, minus_valid) =
            project_object(kind, params, &observation.object_points, rvec, &minus);
        if !plus_valid.iter().all(|value| *value) || !minus_valid.iter().all(|value| *value) {
            return None;
        }
        for point in 0..observation.image_points.len() {
            matrix[(2 * point, index + 3)] =
                (plus_pixels[point][0] - minus_pixels[point][0]) / (2.0 * step);
            matrix[(2 * point + 1, index + 3)] =
                (plus_pixels[point][1] - minus_pixels[point][1]) / (2.0 * step);
        }
    }
    Some(matrix)
}

/// 残差 RMS 与被跳过（投影无效）的视图数；不静默丢残差。
fn residual_rms(
    kind: ModelKind,
    params: &Parameters,
    views: &[(Observation, [f64; 3], [f64; 3])],
) -> Result<(f64, usize), ObservabilityError> {
    let mut squared = Vec::new();
    let mut skipped = 0usize;
    for (observation, rvec, tvec) in views {
        let (pixels, valid) = project_object(kind, params, &observation.object_points, rvec, tvec);
        if !valid.iter().all(|value| *value) {
            skipped += 1;
            continue;
        }
        for (index, pixel) in pixels.iter().enumerate() {
            let observed = observation.image_points[index];
            let dx = pixel[0] - observed[0];
            let dy = pixel[1] - observed[1];
            squared.push(dx * dx + dy * dy);
        }
    }
    if squared.is_empty() {
        return Err(ObservabilityError::NoUsableView);
    }
    let mean = squared.iter().sum::<f64>() / squared.len() as f64;
    Ok((mean.sqrt(), skipped))
}

pub fn analyze_intrinsics(
    kind: ModelKind,
    params: &Parameters,
    views: &[(Observation, [f64; 3], [f64; 3])],
    previous: Option<&IntrinsicObservability>,
) -> Result<IntrinsicObservability, ObservabilityError> {
    if views.is_empty() {
        return Err(ObservabilityError::EmptyViews);
    }
    for (observation, _rvec, _tvec) in views {
        if observation
            .image_points
            .iter()
            .any(|point| !point[0].is_finite() || !point[1].is_finite())
        {
            return Err(ObservabilityError::NonFiniteImagePoints);
        }
    }
    let names: Vec<&'static str> = kind.parameter_names().to_vec();
    let parameter_count = names.len();
    let scales = parameter_scales(kind, params);
    let mut total = DMatrix::<f64>::zeros(parameter_count, parameter_count);
    let mut point_count = 0usize;
    let mut used_views = 0usize;
    let mut skipped_views = 0usize;
    let mut usable_views = Vec::new();
    for (observation, rvec, tvec) in views {
        let Some(intrinsic) = intrinsic_jacobian(kind, params, observation, rvec, tvec, &scales)
        else {
            skipped_views += 1;
            continue;
        };
        let Some(pose) = pose_jacobian(kind, params, observation, rvec, tvec) else {
            skipped_views += 1;
            continue;
        };
        let h_kk = intrinsic.transpose() * &intrinsic;
        let h_ke = intrinsic.transpose() * &pose;
        let mut h_ee = pose.transpose() * &pose;
        for index in 0..POSE_REQUIRED {
            h_ee[(index, index)] += MATRIX_DAMPING;
        }
        let Some(h_ee_inv) = h_ee
            .lu()
            .solve(&DMatrix::identity(POSE_REQUIRED, POSE_REQUIRED))
        else {
            return Err(ObservabilityError::PoseInformationSingular);
        };
        total += h_kk - &h_ke * h_ee_inv * h_ke.transpose();
        point_count += observation.image_points.len();
        used_views += 1;
        usable_views.push((observation.clone(), *rvec, *tvec));
    }
    if used_views == 0 {
        return Err(ObservabilityError::NoUsableView);
    }
    // 对称化
    let half = (&total + total.transpose()) * 0.5;
    total = half;

    // 秩/条件数在相关归一化矩阵上判定
    let mut diagonal = DVector::<f64>::zeros(parameter_count);
    for index in 0..parameter_count {
        diagonal[index] = total[(index, index)].max(1e-300).sqrt();
    }
    let mut correlation_information = DMatrix::<f64>::zeros(parameter_count, parameter_count);
    for row in 0..parameter_count {
        for column in 0..parameter_count {
            correlation_information[(row, column)] =
                total[(row, column)] / (diagonal[row] * diagonal[column]);
        }
    }
    correlation_information =
        (correlation_information.clone() + correlation_information.transpose()) * 0.5;
    let eigen = correlation_information.symmetric_eigen();
    let mut eigenvalues: Vec<f64> = eigen.eigenvalues.iter().copied().collect();
    eigenvalues.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let max_eigenvalue = *eigenvalues.last().unwrap();
    let tolerance = max_eigenvalue * RANK_RELATIVE_TOLERANCE;
    let positive: Vec<f64> = eigenvalues
        .iter()
        .copied()
        .filter(|value| value.is_finite() && *value > tolerance)
        .collect();
    let rank = positive.len();
    if rank < parameter_count {
        return Err(ObservabilityError::RankDeficient {
            rank,
            parameters: parameter_count,
        });
    }
    let condition_number = positive.last().copied().unwrap() / positive.first().copied().unwrap();
    let log_det = positive.iter().map(|value| value.ln()).sum::<f64>();

    let (rms, residual_skipped) = residual_rms(kind, params, &usable_views)?;
    skipped_views += residual_skipped;
    let sigma_squared = rms * rms;
    let mut damped = total.clone();
    for index in 0..parameter_count {
        damped[(index, index)] += MATRIX_DAMPING;
    }
    let Some(damped_inv) = damped
        .lu()
        .solve(&DMatrix::identity(parameter_count, parameter_count))
    else {
        return Err(ObservabilityError::PoseInformationSingular);
    };
    let mut covariance = DMatrix::<f64>::zeros(parameter_count, parameter_count);
    for row in 0..parameter_count {
        for column in 0..parameter_count {
            covariance[(row, column)] =
                sigma_squared * scales[row] * damped_inv[(row, column)] * scales[column];
        }
    }
    covariance = (covariance.clone() + covariance.transpose()) * 0.5;

    let values = params.as_vector();
    let mut standard_deviation = Vec::with_capacity(parameter_count);
    for index in 0..parameter_count {
        let stddev = covariance[(index, index)].max(0.0).sqrt();
        standard_deviation.push(if names[index] == "fx" || names[index] == "fy" {
            stddev / values[index].abs().max(1.0)
        } else {
            stddev
        });
    }

    let mut correlation = vec![vec![0.0; parameter_count]; parameter_count];
    for row in 0..parameter_count {
        for column in 0..parameter_count {
            let denominator =
                (covariance[(row, row)].max(0.0) * covariance[(column, column)].max(0.0)).sqrt();
            correlation[row][column] = if denominator > 0.0 {
                (covariance[(row, column)] / denominator).clamp(-1.0, 1.0)
            } else {
                0.0
            };
        }
        correlation[row][row] = 1.0;
    }
    let info_gain = previous.map(|report| log_det - report.log_det_information);

    Ok(IntrinsicObservability {
        model: kind,
        names,
        view_count: views.len(),
        used_view_count: used_views,
        skipped_view_count: skipped_views,
        point_count,
        rms_px: rms,
        rank,
        condition_number,
        log_det_information: log_det,
        info_gain,
        parameter_stddev: standard_deviation,
        parameter_scales: scales,
        parameter_correlation: correlation,
    })
}
