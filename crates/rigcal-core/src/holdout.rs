//! 求解答集划分：代表性选择（train 限幅）+ holdout 划分与度量。
//!
//! - 代表性选择：`max_solve_observations` **不是简单截断早期帧**——用「像面中心 / 尺度 / 形状」
//!   四维特征做**最远点采样**，保住覆盖多样性；
//! - train/holdout 划分：按（位置 × 尺度 × 径向）分桶，桶内取**最后一帧**进 holdout（同桶相邻帧
//!   几乎同姿态，留最后一帧最有信息量），不足再从尾部补；
//! - holdout 度量：用**冻结内参**对每条 holdout 观测估位姿，逐点残差取 rms / 中位 / p95。
//!
//! 这些都只依赖像点分布，不依赖模型初值，因此可以和训练过程解耦。

use std::collections::BTreeMap;

use crate::estimator::{Observation, Status, estimate_fixed_intrinsics_pose};
use crate::models::{ModelKind, Parameters};

#[derive(Clone, Debug, PartialEq)]
pub struct HoldoutError(pub String);

impl std::fmt::Display for HoldoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for HoldoutError {}

fn points_of(observation: &Observation) -> Vec<[f64; 2]> {
    observation.image_points.clone()
}

fn span(points: &[[f64; 2]]) -> [f64; 2] {
    let mut minimum = [f64::INFINITY; 2];
    let mut maximum = [f64::NEG_INFINITY; 2];
    for point in points {
        for axis in 0..2 {
            minimum[axis] = minimum[axis].min(point[axis]);
            maximum[axis] = maximum[axis].max(point[axis]);
        }
    }
    [maximum[0] - minimum[0], maximum[1] - minimum[1]]
}

fn center(points: &[[f64; 2]]) -> [f64; 2] {
    let mut sum = [0.0; 2];
    for point in points {
        sum[0] += point[0];
        sum[1] += point[1];
    }
    let count = points.len().max(1) as f64;
    [sum[0] / count, sum[1] / count]
}

/// 位置 × 尺度 × 径向的分桶键。
pub type BucketKey = (i32, i32, i32, i32);

/// 位置 × 尺度 × 径向的分桶。
pub fn observation_bucket(observation: &Observation, image_size: (u32, u32)) -> BucketKey {
    let points = points_of(observation);
    if points.is_empty() {
        return (0, 0, 0, 0);
    }
    let (width, height) = (image_size.0.max(1) as f64, image_size.1.max(1) as f64);
    let center = center(&points);
    let x_bin = (center[0] / width * 4.0).clamp(0.0, 3.0) as i32;
    let y_bin = (center[1] / height * 3.0).clamp(0.0, 2.0) as i32;
    let span = span(&points);
    let scale = (span[0] * span[1]).max(0.0).sqrt() / width.min(height);
    let scale_bin = (scale * 4.0).clamp(0.0, 3.0) as i32;
    let normalized = [
        (center[0] - width * 0.5) / width.min(height),
        (center[1] - height * 0.5) / width.min(height),
    ];
    let radial = (normalized[0] * normalized[0] + normalized[1] * normalized[1]).sqrt();
    let radial_bin = (radial * 4.0).clamp(0.0, 3.0) as i32;
    (x_bin, y_bin, scale_bin, radial_bin)
}

/// 四维特征：`[中心 x/w, 中心 y/h, 尺度, log(长宽比)]`。
pub fn observation_feature_vector(observation: &Observation, image_size: (u32, u32)) -> [f64; 4] {
    let points = points_of(observation);
    if points.is_empty() {
        return [0.0; 4];
    }
    let (width, height) = (image_size.0.max(1) as f64, image_size.1.max(1) as f64);
    let center = center(&points);
    let span = span(&points);
    let span = [span[0].max(1.0), span[1].max(1.0)];
    let scale = (span[0] * span[1]).sqrt() / width.min(height);
    let aspect = (span[0] / span[1]).ln();
    [center[0] / width, center[1] / height, scale, aspect]
}

/// 代表性选择：`max_count <= 0` 或观测不足时原序返回；否则最远点采样（结果按原序排序）。
pub fn representative_indices(
    observations: &[Observation],
    max_count: usize,
    image_size: (u32, u32),
) -> Vec<usize> {
    if max_count == 0 || observations.len() <= max_count {
        return (0..observations.len()).collect();
    }
    if max_count == 1 {
        return vec![0];
    }
    let features: Vec<[f64; 4]> = observations
        .iter()
        .map(|observation| observation_feature_vector(observation, image_size))
        .collect();
    let distance = |left: &[f64; 4], right: &[f64; 4]| -> f64 {
        (0..4)
            .map(|axis| (left[axis] - right[axis]).powi(2))
            .sum::<f64>()
            .sqrt()
    };
    let mut selected = vec![0usize];
    let mut min_distances: Vec<f64> = features
        .iter()
        .map(|feature| distance(feature, &features[0]))
        .collect();
    while selected.len() < max_count {
        for index in &selected {
            min_distances[*index] = -1.0;
        }
        let mut next = 0usize;
        let mut best = f64::NEG_INFINITY;
        for (index, value) in min_distances.iter().enumerate() {
            if *value > best {
                best = *value;
                next = index;
            }
        }
        selected.push(next);
        for (index, feature) in features.iter().enumerate() {
            min_distances[index] = min_distances[index].min(distance(feature, &features[next]));
        }
    }
    selected.sort_unstable();
    selected
}

/// train/holdout 划分：返回 `(train 下标, holdout 下标)`，两者都按原序。
pub fn split_train_holdout(
    observations: &[Observation],
    holdout_fraction: f64,
    image_size: (u32, u32),
) -> Result<(Vec<usize>, Vec<usize>), HoldoutError> {
    if !(0.0 < holdout_fraction && holdout_fraction < 1.0) {
        return Err(HoldoutError(
            "holdout_fraction must be in (0, 1)".to_owned(),
        ));
    }
    if observations.len() < 2 {
        return Ok(((0..observations.len()).collect(), Vec::new()));
    }
    let holdout_count = ((observations.len() as f64 * holdout_fraction).round() as usize).max(1);
    let mut buckets: BTreeMap<BucketKey, Vec<usize>> = BTreeMap::new();
    for (index, observation) in observations.iter().enumerate() {
        buckets
            .entry(observation_bucket(observation, image_size))
            .or_default()
            .push(index);
    }
    // 桶按「先大后小、同大小按键序」排序；桶内取最后一帧。
    let mut ordered: Vec<(BucketKey, Vec<usize>)> = buckets.into_iter().collect();
    ordered.sort_by(|left, right| {
        right
            .1
            .len()
            .cmp(&left.1.len())
            .then_with(|| left.0.cmp(&right.0))
    });
    let mut holdout: Vec<usize> = Vec::new();
    for (_bucket, indices) in &ordered {
        if holdout.len() >= holdout_count {
            break;
        }
        if indices.len() >= 2 {
            holdout.push(*indices.last().expect("non-empty bucket"));
        }
    }
    if holdout.len() < holdout_count {
        for index in (0..observations.len()).rev() {
            if holdout.len() >= holdout_count {
                break;
            }
            if !holdout.contains(&index) {
                holdout.push(index);
            }
        }
    }
    holdout.sort_unstable();
    let train = (0..observations.len())
        .filter(|index| !holdout.contains(index))
        .collect();
    Ok((train, holdout))
}

/// holdout 度量：用冻结内参逐条估位姿，逐点残差取 rms / 中位 / p95。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldoutMetrics {
    pub views: usize,
    pub rms_px: f64,
    pub median_px: f64,
    pub p95_px: f64,
    /// 无效投影点数（含位姿求解失败的观测：那些观测的全部点多计为无效，fail closed）。
    pub invalid_projections: usize,
}

/// 与 `numpy.percentile(a, p)`（默认线性插值）一致。
fn percentile(sorted: &[f64], percent: f64) -> f64 {
    let count = sorted.len();
    if count == 0 {
        return f64::INFINITY;
    }
    if count == 1 {
        return sorted[0];
    }
    let position = (percent / 100.0) * (count - 1) as f64;
    let lower = position.floor() as usize;
    let upper = position.ceil() as usize;
    if lower == upper {
        return sorted[lower];
    }
    let fraction = position - lower as f64;
    sorted[lower] * (1.0 - fraction) + sorted[upper] * fraction
}

fn median(sorted: &[f64]) -> f64 {
    let count = sorted.len();
    if count == 0 {
        return f64::INFINITY;
    }
    if count % 2 == 1 {
        return sorted[count / 2];
    }
    (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0
}

pub fn holdout_metrics(
    model: ModelKind,
    parameters: &Parameters,
    observations: &[Observation],
) -> HoldoutMetrics {
    let mut norms: Vec<f64> = Vec::new();
    let mut squared_sum = 0.0f64;
    let mut count = 0usize;
    let mut invalid = 0usize;
    for observation in observations {
        let estimate = estimate_fixed_intrinsics_pose(model, parameters, observation);
        if estimate.status != Status::Pass {
            // 冻结解下仍拟合失败：该观测整体计为无效（不静默丢弃）
            invalid += observation.image_points.len();
            continue;
        }
        invalid += estimate.invalid_projection_count;
        for residual in &estimate.residuals_px {
            let norm = (residual[0] * residual[0] + residual[1] * residual[1]).sqrt();
            norms.push(norm);
            squared_sum += residual[0] * residual[0] + residual[1] * residual[1];
            count += 1;
        }
    }
    if count == 0 {
        return HoldoutMetrics {
            views: observations.len(),
            rms_px: f64::INFINITY,
            median_px: f64::INFINITY,
            p95_px: f64::INFINITY,
            invalid_projections: invalid,
        };
    }
    norms.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
    HoldoutMetrics {
        views: observations.len(),
        rms_px: (squared_sum / count as f64).sqrt(),
        median_px: median(&norms),
        p95_px: percentile(&norms, 95.0),
        invalid_projections: invalid,
    }
}

#[cfg(test)]
mod tests {
    use super::{HoldoutError, observation_bucket, representative_indices, split_train_holdout};
    use crate::estimator::Observation;

    fn observation(center: [f64; 2], span: f64) -> Observation {
        let points: Vec<[f64; 2]> = (0..36)
            .map(|index| {
                let x = index % 6;
                let y = index / 6;
                [
                    center[0] + (x as f64 - 2.5) * span,
                    center[1] + (y as f64 - 2.5) * span,
                ]
            })
            .collect();
        Observation {
            object_points: (0..36).map(|_| [0.0, 0.0, 0.0]).collect(),
            image_points: points,
        }
    }

    #[test]
    fn buckets_capture_position_and_scale() {
        let image_size = (1280, 1088);
        let centered_large = observation_bucket(&observation([640.0, 544.0], 60.0), image_size);
        let edge_small = observation_bucket(&observation([100.0, 100.0], 8.0), image_size);
        assert_ne!(centered_large, edge_small);
        assert_eq!(centered_large.1, 1, "中心 y 落在中桶");
        assert_eq!(edge_small.0, 0, "左上角 x 落在第 0 桶");
    }

    #[test]
    fn split_rejects_bad_fraction() {
        let observations = vec![observation([640.0, 544.0], 60.0); 4];
        assert!(matches!(
            split_train_holdout(&observations, 1.0, (1280, 1088)),
            Err(HoldoutError(_))
        ));
        assert!(split_train_holdout(&observations, 0.0, (1280, 1088)).is_err());
    }

    #[test]
    fn representative_sampling_keeps_spread_and_order() {
        let mut observations = Vec::new();
        for index in 0..20 {
            let center = [
                100.0 + index as f64 * 55.0,
                100.0 + (index % 5) as f64 * 180.0,
            ];
            observations.push(observation(center, 6.0 + index as f64 * 1.5));
        }
        let all = representative_indices(&observations, 0, (1280, 1088));
        assert_eq!(all.len(), 20, "0 表示不裁剪");
        let picked = representative_indices(&observations, 8, (1280, 1088));
        assert_eq!(picked.len(), 8);
        assert_eq!(picked[0], 0, "第一个总是被选中");
        assert!(
            picked.windows(2).all(|pair| pair[0] < pair[1]),
            "结果按原序排序"
        );
        let first = representative_indices(&observations, 1, (1280, 1088));
        assert_eq!(first, vec![0]);
    }
}
