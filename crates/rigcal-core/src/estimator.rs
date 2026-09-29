//! 位姿估计与解评估。
//!
//! **内参求解不在本 crate**：内参求解交给已验证的后端（OpenCV `fisheye::calibrate` / scipy），
//! 本模块只负责：
//!
//! - 位姿播种（模型感知反投影 → 归一化 → 单应分解 + 候选筛选）；
//! - 固定内参的 6 自由度位姿（有界 LM + soft_l1）；
//! - 解评估（无效投影计数、rms 只看有效点）——用于审计外部解。
//!
//! 「后端」通过 [`SolveBackend`] 注入：会话层不关心是谁在解，只要给出 [`SolveOutcome`]。

use nalgebra::{DMatrix, DVector};

use crate::models::{self, ModelKind, Parameters};

const INVALID_PENALTY_PX: f64 = 1000.0;
const JACOBIAN_STEP: f64 = 1.0e-6;
/// 位姿边界：rvec ±π、x/y ±5 m、z 0.05–10 m。
const POSE_LOWER: [f64; 6] = [
    -std::f64::consts::PI,
    -std::f64::consts::PI,
    -std::f64::consts::PI,
    -5.0,
    -5.0,
    0.05,
];
const POSE_UPPER: [f64; 6] = [
    std::f64::consts::PI,
    std::f64::consts::PI,
    std::f64::consts::PI,
    5.0,
    5.0,
    10.0,
];

/// 一路观测的原始数据：板坐标点与像素点（等长）。
pub type ObservationArrays = (Vec<[f64; 3]>, Vec<[f64; 2]>);
/// 一路视图的位姿 `(rvec, tvec)`。
pub type IntrinsicPose = ([f64; 3], [f64; 3]);
/// 可观测性/评估用的一路视图：观测 + 位姿。
pub type ObservationView = (Observation, [f64; 3], [f64; 3]);

#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Observation {
    pub object_points: Vec<[f64; 3]>,
    pub image_points: Vec<[f64; 2]>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Fail,
}

#[derive(Clone, Debug)]
pub struct PoseEstimate {
    pub rvec: [f64; 3],
    pub tvec: [f64; 3],
    pub residuals_px: Vec<[f64; 2]>,
    pub invalid_projection_count: usize,
    pub rms_px: f64,
    pub status: Status,
    pub iterations: usize,
}

/// 解评估结果（无效投影计数；rms 只统计有效点）。
#[derive(Clone, Debug)]
pub struct Evaluation {
    pub rms_px: f64,
    pub invalid_projection_count: usize,
    pub per_view_rms: Vec<f64>,
}

/// 内参求解的请求：交给注入的后端。
pub struct SolveRequest<'a> {
    pub kind: ModelKind,
    pub observations: &'a [Observation],
    pub image_size: (u32, u32),
    pub initial: Option<Parameters>,
    pub initial_poses: Option<&'a [IntrinsicPose]>,
}

/// 内参求解的结果：与会话闸门/评估共用一套字段。
#[derive(Clone, Debug)]
pub struct SolveOutcome {
    pub parameters: Parameters,
    /// 与 `SolveRequest::observations` 同长、同序。
    pub poses: Vec<IntrinsicPose>,
    pub residuals_px: Vec<[f64; 2]>,
    pub invalid_projection_count: usize,
    pub rms_px: f64,
    pub status: Status,
}

/// 求解后端：已验证实现（OpenCV / 外部进程）的注入点。
pub type SolveBackend = dyn Fn(SolveRequest<'_>) -> Result<SolveOutcome, String>;

/// 用后端给出的解评估残差（会话采纳闸门与审计共用）。
pub fn evaluate_outcome(
    kind: ModelKind,
    observations: &[Observation],
    outcome: &SolveOutcome,
) -> Evaluation {
    evaluate_solution(kind, observations, &outcome.parameters, &outcome.poses)
}

pub fn project_with_pose(
    kind: ModelKind,
    params: &Parameters,
    object_points: &[[f64; 3]],
    rvec: &[f64; 3],
    tvec: &[f64; 3],
) -> (Vec<[f64; 2]>, Vec<bool>) {
    let camera_points = models::transform_points(object_points, rvec, tvec);
    models::project(kind, &camera_points, params)
}

/// 残差向量：无效投影保持长度不变并显式惩罚。
fn pose_residual(
    kind: ModelKind,
    params: &Parameters,
    observation: &Observation,
    pose: &[f64],
) -> Vec<f64> {
    let rvec = [pose[0], pose[1], pose[2]];
    let tvec = [pose[3], pose[4], pose[5]];
    let (pixels, valid) = project_with_pose(kind, params, &observation.object_points, &rvec, &tvec);
    let mut residuals = Vec::with_capacity(observation.image_points.len() * 2);
    for (index, pixel) in pixels.iter().enumerate() {
        let observed = observation.image_points[index];
        let (dx, dy) = if valid[index] {
            (pixel[0] - observed[0], pixel[1] - observed[1])
        } else {
            (INVALID_PENALTY_PX, INVALID_PENALTY_PX)
        };
        residuals.push(dx);
        residuals.push(dy);
    }
    residuals
}

/// 评估：无效投影**不计入** rms 但计数。
fn evaluate_pose(
    kind: ModelKind,
    params: &Parameters,
    observation: &Observation,
    pose: &[f64],
) -> (Vec<[f64; 2]>, usize, f64) {
    let rvec = [pose[0], pose[1], pose[2]];
    let tvec = [pose[3], pose[4], pose[5]];
    let (pixels, valid) = project_with_pose(kind, params, &observation.object_points, &rvec, &tvec);
    let mut residuals = Vec::new();
    let mut invalid = 0usize;
    let mut sum_squares = 0.0;
    for (index, pixel) in pixels.iter().enumerate() {
        if !valid[index] {
            invalid += 1;
            continue;
        }
        let observed = observation.image_points[index];
        let diff = [pixel[0] - observed[0], pixel[1] - observed[1]];
        sum_squares += diff[0] * diff[0] + diff[1] * diff[1];
        residuals.push(diff);
    }
    let rms = if residuals.is_empty() {
        f64::INFINITY
    } else {
        (sum_squares / residuals.len() as f64).sqrt()
    };
    (residuals, invalid, rms)
}

// ---------------------------------------------------------------------------
// 位姿 LM：**只用于 6 参数位姿**。
// ---------------------------------------------------------------------------

/// soft_l1 代价 `Σ 2·s²·(√(1 + (r/s)²) − 1)`；无 f_scale 时是平方和。
fn robust_cost(residuals: &[f64], f_scale: Option<f64>) -> f64 {
    match f_scale {
        None => residuals.iter().map(|value| value * value).sum(),
        Some(scale) => residuals
            .iter()
            .map(|value| {
                let ratio = value / scale;
                2.0 * value * value / (1.0 + ratio.hypot(1.0))
            })
            .sum(),
    }
}

/// soft_l1 的 IRLS 权重 `(1 + (r/s)²)^(-1/4)`。
fn irls_weights(residuals: &[f64], f_scale: f64) -> Vec<f64> {
    residuals
        .iter()
        .map(|value| {
            let z = (value / f_scale) * (value / f_scale);
            (1.0 + z).powf(-0.25)
        })
        .collect()
}

/// 有界 LM（中心差分雅可比 + λ·diag 阻尼 + ∞-范数信赖域 + 死锁后复位重试）。
fn pose_lm<F>(
    mut residual: F,
    x0: &[f64],
    lower: &[f64],
    upper: &[f64],
    max_iterations: usize,
    f_scale: f64,
    trace: bool,
) -> (Vec<f64>, usize)
where
    F: FnMut(&[f64]) -> Vec<f64>,
{
    let mut x: Vec<f64> = x0
        .iter()
        .zip(lower.iter().zip(upper.iter()))
        .map(|(value, (low, high))| value.clamp(*low, *high))
        .collect();
    let mut residuals = residual(&x);
    let mut current_cost = robust_cost(&residuals, Some(f_scale));
    let mut lambda: f64 = 1e-3;
    let mut radius: f64 = 1.0;
    let mut iterations = 0usize;
    let mut restarts = 0usize;

    while iterations < max_iterations {
        iterations += 1;
        if trace {
            let rms = (residuals.iter().map(|value| value * value).sum::<f64>()
                / residuals.len().max(1) as f64)
                .sqrt();
            eprintln!(
                "[pose-lm] it={iterations:<3} cost={current_cost:.6e} rms={rms:.4} lambda={lambda:.3e}"
            );
        }
        let weight = irls_weights(&residuals, f_scale);
        let scaled: Vec<f64> = residuals
            .iter()
            .zip(weight.iter())
            .map(|(r, w)| r * w)
            .collect();
        let rows = scaled.len();
        let columns = x.len();
        let mut jacobian = DMatrix::<f64>::zeros(rows, columns);
        let mut probe = x.clone();
        for column in 0..columns {
            let delta = JACOBIAN_STEP * x[column].abs().max(1.0);
            probe[column] = x[column] + delta;
            let plus = residual(&probe);
            probe[column] = x[column] - delta;
            let minus = residual(&probe);
            probe[column] = x[column];
            for row in 0..rows {
                jacobian[(row, column)] = weight[row] * (plus[row] - minus[row]) / (2.0 * delta);
            }
        }
        let gradient = jacobian.transpose() * DVector::from_column_slice(&scaled);
        let normal = jacobian.transpose() * &jacobian;
        let mut accepted = false;
        for _ in 0..12 {
            let mut damped = normal.clone();
            for index in 0..columns {
                damped[(index, index)] += lambda * normal[(index, index)].max(1e-12);
            }
            let Some(step) = damped.lu().solve(&(-&gradient)) else {
                lambda *= 10.0;
                continue;
            };
            let scaled_norm = step
                .iter()
                .zip(x.iter())
                .map(|(delta, value)| (delta / value.abs().max(1.0)).abs())
                .fold(0.0_f64, f64::max);
            let shrink = if scaled_norm > radius {
                radius / scaled_norm
            } else {
                1.0
            };
            let candidate: Vec<f64> = x
                .iter()
                .zip(step.iter())
                .zip(lower.iter().zip(upper.iter()))
                .map(|((value, delta), (low, high))| (value + delta * shrink).clamp(*low, *high))
                .collect();
            let candidate_residuals = residual(&candidate);
            let candidate_cost = robust_cost(&candidate_residuals, Some(f_scale));
            if candidate_cost < current_cost {
                x = candidate;
                residuals = candidate_residuals;
                current_cost = candidate_cost;
                lambda = (lambda * 0.3_f64).max(1e-12);
                radius = (radius * 1.5_f64).min(1e6);
                accepted = true;
                break;
            }
            lambda *= 3.0_f64;
            radius *= 0.5_f64;
        }
        if !accepted {
            // 半径/λ 被压到极小后步长恒为 0、代价冻结：复位重试，避免一路空转到上限。
            restarts += 1;
            if restarts > 3 {
                break;
            }
            radius = 1.0;
            lambda = 1e-3;
        }
    }
    (x, iterations)
}

/// 平面单应分解求位姿种子：在归一化坐标上分解单应，再按**原始像素**重投影误差挑最优。
///
/// **必须做候选筛选**：单应分解有 4 个候选解（含镜像分支），只取一个会让约一半视图拿到错误候选。
fn homography_pose(
    kind: ModelKind,
    params: &Parameters,
    object_points: &[[f64; 3]],
    image_points: &[[f64; 2]],
    normalized: &[[f64; 2]],
) -> Option<IntrinsicPose> {
    let count = normalized.len();
    if count < 4 {
        return None;
    }
    let mut design = DMatrix::<f64>::zeros(2 * count, 9);
    for index in 0..count {
        let [x, y] = [object_points[index][0], object_points[index][1]];
        let [u, v] = normalized[index];
        design[(2 * index, 0)] = x;
        design[(2 * index, 1)] = y;
        design[(2 * index, 2)] = 1.0;
        design[(2 * index, 6)] = -u * x;
        design[(2 * index, 7)] = -u * y;
        design[(2 * index, 8)] = -u;
        design[(2 * index + 1, 3)] = x;
        design[(2 * index + 1, 4)] = y;
        design[(2 * index + 1, 5)] = 1.0;
        design[(2 * index + 1, 6)] = -v * x;
        design[(2 * index + 1, 7)] = -v * y;
        design[(2 * index + 1, 8)] = -v;
    }
    // 恰好 4 点时 DLT 方程是 8×8 的**方阵**系统（h8 归一化为 1），直接解线性方程；
    // 不能走 SVD——nalgebra 的 `svd` 在行数 < 列数（8×9）时内部切片越界 panic
    // （单个 tag 只有 4 个角点时就会触发）。
    let mut h: nalgebra::DVector<f64> = if design.nrows() == 8 {
        let mut a = DMatrix::<f64>::zeros(8, 8);
        let mut b = DVector::<f64>::zeros(8);
        for index in 0..4 {
            let [x, y] = [object_points[index][0], object_points[index][1]];
            let [u, v] = normalized[index];
            let row = 2 * index;
            a[(row, 0)] = x;
            a[(row, 1)] = y;
            a[(row, 2)] = 1.0;
            a[(row, 6)] = -u * x;
            a[(row, 7)] = -u * y;
            b[row] = u;
            a[(row + 1, 3)] = x;
            a[(row + 1, 4)] = y;
            a[(row + 1, 5)] = 1.0;
            a[(row + 1, 6)] = -v * x;
            a[(row + 1, 7)] = -v * y;
            b[row + 1] = v;
        }
        let solution = a.lu().solve(&b)?;
        let mut full = nalgebra::DVector::<f64>::zeros(9);
        full.rows_mut(0, 8).copy_from(&solution);
        full[8] = 1.0;
        full
    } else {
        let svd = design.svd(true, true);
        svd.v_t?.row(8).transpose()
    };
    // SVD 的零空间向量只有比例意义。整张 H 同时变号，不能只翻转旋转列：
    // 否则 h8<0 时所有候选的 tz 都为负，正确位姿会被丢弃。
    if h[8] < 0.0 {
        h *= -1.0;
    }
    let scale = 0.5
        * ((h[0] * h[0] + h[3] * h[3] + h[6] * h[6]).sqrt()
            + (h[1] * h[1] + h[4] * h[4] + h[7] * h[7]).sqrt());
    if !(scale.is_finite() && scale > 1e-12) {
        return None;
    }

    let mut best: Option<(IntrinsicPose, f64)> = None;
    for flip_x in [1.0_f64, -1.0] {
        for flip_y in [1.0_f64, -1.0] {
            let first = nalgebra::Vector3::new(h[0], h[3], h[6]) * flip_x / scale;
            let second = nalgebra::Vector3::new(h[1], h[4], h[7]) * flip_y / scale;
            let third = nalgebra::Vector3::new(h[2], h[5], h[8]) / scale;
            if !third.iter().all(|value| value.is_finite()) || third.z <= 0.0 {
                continue;
            }
            let r3 = first.cross(&second);
            let mut rotation = nalgebra::Matrix3::from_columns(&[first, second, r3]);
            if rotation.determinant() <= 0.0 {
                continue;
            }
            let ortho = rotation.svd(true, true);
            let (Some(u), Some(v_t)) = (ortho.u, ortho.v_t) else {
                continue;
            };
            rotation = u * v_t;
            if rotation.determinant() <= 0.0 {
                continue;
            }
            let rotation = nalgebra::Rotation3::from_matrix_unchecked(rotation);
            let axis = rotation.scaled_axis();
            let rvec = [axis.x, axis.y, axis.z];
            let tvec = [third.x, third.y, third.z];
            let (pixels, valid) = models::project(
                kind,
                &models::transform_points(object_points, &rvec, &tvec),
                params,
            );
            if !valid.iter().all(|value| *value) {
                continue;
            }
            // 打分必须用**原始像素**：归一化坐标不是像素，拿它比重投影会全选错候选。
            let mut sum = 0.0;
            for (index, pixel) in pixels.iter().enumerate() {
                let expected = image_points[index];
                sum += (pixel[0] - expected[0]).powi(2) + (pixel[1] - expected[1]).powi(2);
            }
            let score = sum / count as f64;
            if best
                .map(|(_, best_score)| score < best_score)
                .unwrap_or(true)
            {
                best = Some(((rvec, tvec), score));
            }
        }
    }
    best.map(|(pose, _score)| pose)
}

/// 位姿种子：模型感知反投影 → 归一化 → 单应分解；不可用时退回平移前向的保守初值。
///
/// 公开是为了让离线诊断能衡量「播种本身有多好」（求解器内部也用它）。
pub fn seed_pose(kind: ModelKind, params: &Parameters, observation: &Observation) -> [f64; 6] {
    let (rays, valid) = models::unproject(kind, &observation.image_points, params);
    let mut object_points = Vec::new();
    let mut image_points = Vec::new();
    let mut normalized = Vec::new();
    for (index, ray) in rays.iter().enumerate() {
        if !valid[index] || ray.iter().any(|value| !value.is_finite()) || ray[2] <= 1e-4 {
            continue;
        }
        object_points.push(observation.object_points[index]);
        image_points.push(observation.image_points[index]);
        normalized.push([ray[0] / ray[2], ray[1] / ray[2]]);
    }
    if let Some((rvec, tvec)) =
        homography_pose(kind, params, &object_points, &image_points, &normalized)
    {
        return [rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]];
    }
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.2]
}

/// 固定内参的 6 自由度位姿。
pub fn estimate_fixed_intrinsics_pose(
    kind: ModelKind,
    params: &Parameters,
    observation: &Observation,
) -> PoseEstimate {
    let seed = seed_pose(kind, params, observation);
    let (x, iterations) = pose_lm(
        |pose: &[f64]| pose_residual(kind, params, observation, pose),
        &seed,
        &POSE_LOWER,
        &POSE_UPPER,
        100,
        2.0,
        false,
    );
    let pose = [x[0], x[1], x[2], x[3], x[4], x[5]];
    let (residuals, invalid, rms) = evaluate_pose(kind, params, observation, &pose);
    let status = if rms.is_finite() && invalid == 0 {
        Status::Pass
    } else {
        Status::Fail
    };
    PoseEstimate {
        rvec: [pose[0], pose[1], pose[2]],
        tvec: [pose[3], pose[4], pose[5]],
        residuals_px: residuals,
        invalid_projection_count: invalid,
        rms_px: rms,
        status,
        iterations,
    }
}

/// 对一份外部给出的解做评估（与求解同一套语义：无效投影计数、rms 只看有效点）。
pub fn evaluate_solution(
    kind: ModelKind,
    observations: &[Observation],
    params: &Parameters,
    poses: &[IntrinsicPose],
) -> Evaluation {
    assert_eq!(
        observations.len(),
        poses.len(),
        "poses must match observation count"
    );
    let mut squared = Vec::new();
    let mut invalid = 0usize;
    let mut per_view = Vec::with_capacity(observations.len());
    for (observation, (rvec, tvec)) in observations.iter().zip(poses.iter()) {
        let pose = [rvec[0], rvec[1], rvec[2], tvec[0], tvec[1], tvec[2]];
        let (residuals, view_invalid, view_rms) = evaluate_pose(kind, params, observation, &pose);
        invalid += view_invalid;
        per_view.push(if view_invalid > 0 { f64::NAN } else { view_rms });
        for diff in &residuals {
            squared.push(diff[0] * diff[0] + diff[1] * diff[1]);
        }
    }
    let rms = if squared.is_empty() {
        f64::INFINITY
    } else {
        (squared.iter().sum::<f64>() / squared.len() as f64).sqrt()
    };
    Evaluation {
        rms_px: rms,
        invalid_projection_count: invalid,
        per_view_rms: per_view,
    }
}
