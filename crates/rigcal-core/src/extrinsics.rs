//! 相机间外参：共视帧组 → 成对变换 → 环路校验 → camchain 导出。
//!
//! **变换约定（全文唯一写法）**：`T_target_source` 满足 `P_target = T_target_source · P_source`。
//! 位姿估计给的是 `T_cam_board`（板 → 相机），因此：
//!
//! ```text
//! 相机 a → 相机 b：  T_b_a = T_b_board · T_a_board⁻¹
//! 以 cam0 为参考：    T_c0_ci = T_c0_board · T_ci_board⁻¹        （把 ci 系下的点变到 c0）
//! Kalibr camchain：  T_cn_cnm1 满足 P_cn = T_cn_cnm1 · P_cnm1     （同一方向惯例）
//! ```
//!
//! 外参只在**同一帧组**内共视的相机之间成立；静态板采集时组内各路时间戳相同，所以
//! 「同一组」就是「同一时刻」。

use std::collections::{BTreeMap, HashMap};

use crate::estimator::{Observation, Status, estimate_fixed_intrinsics_pose};
use crate::models::{ModelKind, Parameters};
use crate::rotation::{Matrix3, mat_mul, matrix_to_rvec, rvec_to_matrix, transpose};

pub type Matrix4 = [[f64; 4]; 4];

#[derive(Clone, Debug, PartialEq)]
pub enum ExtrinsicsError {
    NoRotations,
    TooFewGroups {
        camera_a: String,
        camera_b: String,
        groups: usize,
        min_groups: usize,
    },
    NoSharedCorners,
    InvalidProjection,
    Disconnected {
        reference: String,
        missing: Vec<String>,
    },
    /// camchain 导出时，某一路缺内参——绝不静默补空数组。
    MissingIntrinsics {
        camera: String,
    },
    /// camchain 导出时，某一路的内参模型与本次导出模型不一致。
    IntrinsicsModelMismatch {
        camera: String,
        expected: ModelKind,
        found: ModelKind,
    },
    /// camchain 导出时，某一路内参含 NaN/±∞——写进 YAML 会让下游静默失真。
    NonFiniteIntrinsics {
        camera: String,
    },
}

impl std::fmt::Display for ExtrinsicsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ExtrinsicsError::NoRotations => {
                write!(f, "rotation averaging needs at least one rotation")
            }
            ExtrinsicsError::TooFewGroups {
                camera_a,
                camera_b,
                groups,
                min_groups,
            } => write!(
                f,
                "edge {camera_a}-{camera_b} has {groups} co-visible group(s), needs >= {min_groups}"
            ),
            ExtrinsicsError::NoSharedCorners => {
                write!(f, "co-visible records share no tag corners")
            }
            ExtrinsicsError::InvalidProjection => {
                write!(f, "edge reprojection produced invalid projections")
            }
            ExtrinsicsError::Disconnected { reference, missing } => {
                write!(
                    f,
                    "cameras not connected to reference {reference}: {missing:?}"
                )
            }
            ExtrinsicsError::MissingIntrinsics { camera } => {
                write!(f, "camera {camera} has no intrinsics to export")
            }
            ExtrinsicsError::IntrinsicsModelMismatch {
                camera,
                expected,
                found,
            } => write!(
                f,
                "camera {camera} intrinsics model {} does not match the export model {}",
                found.as_str(),
                expected.as_str()
            ),
            ExtrinsicsError::NonFiniteIntrinsics { camera } => {
                write!(f, "camera {camera} intrinsics contain a non-finite value")
            }
        }
    }
}

impl std::error::Error for ExtrinsicsError {}

/// 一条观测记录（帧组内某一路的一次检测）。
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ObservationRecord {
    pub camera_id: String,
    pub group_id: u64,
    pub observation: Observation,
}

pub fn identity4() -> Matrix4 {
    let mut out = [[0.0; 4]; 4];
    for (index, row) in out.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    out
}

/// 刚体逆：`R⁻¹ = Rᵀ`、`t⁻¹ = -Rᵀt`。
pub fn invert(transform: &Matrix4) -> Matrix4 {
    let mut rotation = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            rotation[row][column] = transform[column][row];
        }
    }
    let translation = [transform[0][3], transform[1][3], transform[2][3]];
    let mut out = identity4();
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = rotation[row][column];
        }
        out[row][3] = -(rotation[row][0] * translation[0]
            + rotation[row][1] * translation[1]
            + rotation[row][2] * translation[2]);
    }
    out
}

/// 由旋转与平移组装 4×4。
pub fn compose(rotation: &Matrix3, translation: &[f64; 3]) -> Matrix4 {
    let mut out = identity4();
    for row in 0..3 {
        for column in 0..3 {
            out[row][column] = rotation[row][column];
        }
        out[row][3] = translation[row];
    }
    out
}

/// 4×4 乘法。
pub fn multiply(left: &Matrix4, right: &Matrix4) -> Matrix4 {
    let mut out = [[0.0; 4]; 4];
    for row in 0..4 {
        for column in 0..4 {
            out[row][column] = (0..4)
                .map(|index| left[row][index] * right[index][column])
                .sum();
        }
    }
    out
}

/// 取 4×4 的旋转块。
pub fn rotation_of(transform: &Matrix4) -> Matrix3 {
    [
        [transform[0][0], transform[0][1], transform[0][2]],
        [transform[1][0], transform[1][1], transform[1][2]],
        [transform[2][0], transform[2][1], transform[2][2]],
    ]
}

/// 取 4×4 的平移列。
pub fn translation_of(transform: &Matrix4) -> [f64; 3] {
    [transform[0][3], transform[1][3], transform[2][3]]
}

/// 旋转角（度）：对数映射向量的模长。
pub fn rotation_angle_deg(rotation: &Matrix3) -> f64 {
    let rvec = matrix_to_rvec(rotation);
    (rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2])
        .sqrt()
        .to_degrees()
}

/// 旋转平均（**四元数特征向量法**）：逐元素平均会破坏正交性，这里取四元数外积矩阵的主特征向量。
pub fn average_rotation(rotations: &[Matrix3]) -> Result<Matrix3, ExtrinsicsError> {
    if rotations.is_empty() {
        return Err(ExtrinsicsError::NoRotations);
    }
    let mut accumulator = nalgebra::Matrix4::<f64>::zeros();
    for rotation in rotations {
        let matrix = nalgebra::Matrix3::from_row_slice(&[
            rotation[0][0],
            rotation[0][1],
            rotation[0][2],
            rotation[1][0],
            rotation[1][1],
            rotation[1][2],
            rotation[2][0],
            rotation[2][1],
            rotation[2][2],
        ]);
        let quaternion = nalgebra::UnitQuaternion::from_rotation_matrix(
            &nalgebra::Rotation3::from_matrix_unchecked(matrix),
        )
        .into_inner();
        // 四元数外积矩阵按 (i, j, k, w) 顺序累加；特征向量只用于还原同一旋转，分量排列一致即可。
        let vector = nalgebra::Vector4::new(quaternion.i, quaternion.j, quaternion.k, quaternion.w);
        accumulator += vector * vector.transpose();
    }
    let eigen = nalgebra::SymmetricEigen::new(accumulator);
    let mut best = 0usize;
    for index in 1..4 {
        if eigen.eigenvalues[index] > eigen.eigenvalues[best] {
            best = index;
        }
    }
    let column = eigen.eigenvectors.column(best);
    let quaternion = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        column[3], column[0], column[1], column[2],
    ));
    let matrix = quaternion.to_rotation_matrix().into_inner();
    Ok([
        [matrix[(0, 0)], matrix[(0, 1)], matrix[(0, 2)]],
        [matrix[(1, 0)], matrix[(1, 1)], matrix[(1, 2)]],
        [matrix[(2, 0)], matrix[(2, 1)], matrix[(2, 2)]],
    ])
}

/// 用冻结内参给一条观测估板位姿，返回 `T_cam_board`（状态非 PASS 时返回 None）。
pub fn board_pose(
    observation: &Observation,
    parameters: &Parameters,
    model: ModelKind,
) -> Option<Matrix4> {
    let estimate = estimate_fixed_intrinsics_pose(model, parameters, observation);
    if estimate.status != Status::Pass {
        return None;
    }
    Some(compose(&rvec_to_matrix(&estimate.rvec), &estimate.tvec))
}

fn group_records(records: &[ObservationRecord]) -> BTreeMap<u64, HashMap<String, usize>> {
    let mut grouped: BTreeMap<u64, HashMap<String, usize>> = BTreeMap::new();
    for (index, record) in records.iter().enumerate() {
        grouped
            .entry(record.group_id)
            .or_default()
            .insert(record.camera_id.clone(), index);
    }
    grouped
}

#[derive(Clone, Debug)]
pub struct EdgeEstimate {
    pub camera_a: String,
    pub camera_b: String,
    /// `T_b_a`：把 a 系下的点变到 b 系。
    pub transform: Matrix4,
    pub groups: usize,
    pub rotation_sigma_deg: f64,
    pub translation_sigma_mm: f64,
    pub reprojection_rms_px: f64,
}

#[derive(Clone, Debug)]
pub struct CycleCheck {
    pub cameras: Vec<String>,
    pub rotation_error_deg: f64,
    pub translation_error_mm: f64,
}

#[derive(Clone, Debug)]
pub struct RigEstimate {
    pub edges: Vec<EdgeEstimate>,
    pub cycles: Vec<CycleCheck>,
    pub rig_rms_px: f64,
}

impl RigEstimate {
    pub fn edge_for(&self, camera_a: &str, camera_b: &str) -> Option<&EdgeEstimate> {
        self.edges.iter().find(|edge| {
            (edge.camera_a == camera_a && edge.camera_b == camera_b)
                || (edge.camera_a == camera_b && edge.camera_b == camera_a)
        })
    }
}

/// 角点匹配键：同一格角点由同一段算术产生，取整到 1e-9 后逐位一致。
fn corner_key(point: &[f64; 3]) -> (i64, i64, i64) {
    let scale = 1e9;
    (
        (point[0] * scale).round() as i64,
        (point[1] * scale).round() as i64,
        (point[2] * scale).round() as i64,
    )
}

/// 成对变换：逐共视组算 `T_b_board · T_a_board⁻¹`，再做旋转/平移鲁棒平均。
pub fn solve_edge(
    camera_a: &str,
    camera_b: &str,
    records: &[ObservationRecord],
    parameters: &HashMap<String, Parameters>,
    model: ModelKind,
    min_groups: usize,
) -> Result<EdgeEstimate, ExtrinsicsError> {
    let (Some(parameters_a), Some(parameters_b)) =
        (parameters.get(camera_a), parameters.get(camera_b))
    else {
        return Err(ExtrinsicsError::TooFewGroups {
            camera_a: camera_a.to_owned(),
            camera_b: camera_b.to_owned(),
            groups: 0,
            min_groups,
        });
    };
    let grouped = group_records(records);
    let mut rotations: Vec<Matrix3> = Vec::new();
    let mut translations: Vec<[f64; 3]> = Vec::new();
    let mut residuals: Vec<f64> = Vec::new();
    for per_camera in grouped.values() {
        let (Some(index_a), Some(index_b)) = (per_camera.get(camera_a), per_camera.get(camera_b))
        else {
            continue;
        };
        let record_a = &records[*index_a];
        let record_b = &records[*index_b];
        let Some(pose_a) = board_pose(&record_a.observation, parameters_a, model) else {
            continue;
        };
        let Some(pose_b) = board_pose(&record_b.observation, parameters_b, model) else {
            continue;
        };
        let transform = multiply(&pose_b, &invert(&pose_a));
        rotations.push(rotation_of(&transform));
        translations.push(translation_of(&transform));
        // 残差必须把「板坐标系的点」用 T_b_board 投到 b 的像平面（用 T_b_a 会错一个位姿）。
        residuals.push(edge_reprojection_rms(
            record_a,
            record_b,
            &pose_b,
            parameters_b,
            model,
        )?);
    }
    if rotations.len() < min_groups {
        return Err(ExtrinsicsError::TooFewGroups {
            camera_a: camera_a.to_owned(),
            camera_b: camera_b.to_owned(),
            groups: rotations.len(),
            min_groups,
        });
    }
    let rotation_mean = average_rotation(&rotations)?;
    let mut translation_median = [0.0; 3];
    for axis in 0..3 {
        let mut values: Vec<f64> = translations.iter().map(|value| value[axis]).collect();
        values.sort_by(|left, right| left.partial_cmp(right).unwrap_or(std::cmp::Ordering::Equal));
        translation_median[axis] = median(&values);
    }
    let rotation_sigmas: Vec<f64> = rotations
        .iter()
        .map(|rotation| rotation_angle_deg(&mat_mul(&transpose(&rotation_mean), rotation)))
        .collect();
    let translation_sigmas: Vec<f64> = translations
        .iter()
        .map(|value| {
            let delta = [
                value[0] - translation_median[0],
                value[1] - translation_median[1],
                value[2] - translation_median[2],
            ];
            (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt() * 1000.0
        })
        .collect();
    Ok(EdgeEstimate {
        camera_a: camera_a.to_owned(),
        camera_b: camera_b.to_owned(),
        transform: compose(&rotation_mean, &translation_median),
        groups: rotations.len(),
        rotation_sigma_deg: standard_deviation(&rotation_sigmas),
        translation_sigma_mm: standard_deviation(&translation_sigmas),
        reprojection_rms_px: (residuals.iter().map(|value| value * value).sum::<f64>()
            / residuals.len() as f64)
            .sqrt(),
    })
}

/// 与 `numpy.median` 一致（偶数个取中间两值平均）。
pub(crate) fn median(sorted: &[f64]) -> f64 {
    let count = sorted.len();
    if count == 0 {
        return 0.0;
    }
    if count % 2 == 1 {
        return sorted[count / 2];
    }
    (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0
}

/// 与 `numpy.std`（默认 ddof=0，总体标准差）一致。
fn standard_deviation(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    (values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / values.len() as f64)
        .sqrt()
}

/// 把 a 看到的板点用 `T_b_board` 投到 b 的像平面，与 b 的观测比对。
///
/// 对应关系必须按**物点值**配对：角点顺序是检测器的输出顺序，同一组四路各不相同，各路可见 tag
/// 数也不同——按下标配对会把不同角点比到一起。
fn edge_reprojection_rms(
    record_a: &ObservationRecord,
    record_b: &ObservationRecord,
    pose_b: &Matrix4,
    parameters_b: &Parameters,
    model: ModelKind,
) -> Result<f64, ExtrinsicsError> {
    let mut observed_by_point: HashMap<(i64, i64, i64), [f64; 2]> = HashMap::new();
    for (point, pixels) in record_b
        .observation
        .object_points
        .iter()
        .zip(record_b.observation.image_points.iter())
    {
        observed_by_point.insert(corner_key(point), *pixels);
    }
    let mut points: Vec<[f64; 3]> = Vec::new();
    let mut observed: Vec<[f64; 2]> = Vec::new();
    for (point, _pixels) in record_a
        .observation
        .object_points
        .iter()
        .zip(record_a.observation.image_points.iter())
    {
        if let Some(pixels) = observed_by_point.get(&corner_key(point)) {
            points.push(*point);
            observed.push(*pixels);
        }
    }
    if points.is_empty() {
        return Err(ExtrinsicsError::NoSharedCorners);
    }
    let points_b = crate::models::transform_points(
        &points,
        &matrix_to_rvec(&rotation_of(pose_b)),
        &translation_of(pose_b),
    );
    let (pixels, valid) = crate::models::project(model, &points_b, parameters_b);
    if valid.iter().any(|value| !value) {
        return Err(ExtrinsicsError::InvalidProjection);
    }
    let mut squared = 0.0;
    for (pixel, expected) in pixels.iter().zip(observed.iter()) {
        squared += (pixel[0] - expected[0]).powi(2) + (pixel[1] - expected[1]).powi(2);
    }
    Ok((squared / points_b.len() as f64).sqrt())
}

/// 全机架：算要求的每条边与每个环路，`rig_rms_px` = 各边重投影 rms 的均方根。
pub fn solve_rig(
    records: &[ObservationRecord],
    parameters: &HashMap<String, Parameters>,
    model: ModelKind,
    required_edges: &[(String, String)],
    required_cycles: &[Vec<String>],
    min_groups_per_edge: usize,
) -> Result<RigEstimate, ExtrinsicsError> {
    let mut edges = Vec::with_capacity(required_edges.len());
    for (camera_a, camera_b) in required_edges {
        edges.push(solve_edge(
            camera_a,
            camera_b,
            records,
            parameters,
            model,
            min_groups_per_edge,
        )?);
    }
    let mut cycles = Vec::new();
    for cycle in required_cycles {
        let mut transform = identity4();
        let mut ok = true;
        for index in 0..cycle.len() {
            let a = &cycle[index];
            let b = &cycle[(index + 1) % cycle.len()];
            let Some(edge) = edges.iter().find(|edge| {
                (&edge.camera_a == a && &edge.camera_b == b)
                    || (&edge.camera_a == b && &edge.camera_b == a)
            }) else {
                ok = false;
                break;
            };
            let step = if &edge.camera_a == a {
                edge.transform
            } else {
                invert(&edge.transform)
            };
            transform = multiply(&step, &transform);
        }
        if !ok {
            continue;
        }
        cycles.push(CycleCheck {
            cameras: cycle.clone(),
            rotation_error_deg: rotation_angle_deg(&rotation_of(&transform)),
            translation_error_mm: {
                let translation = translation_of(&transform);
                (translation[0] * translation[0]
                    + translation[1] * translation[1]
                    + translation[2] * translation[2])
                    .sqrt()
                    * 1000.0
            },
        });
    }
    let rig_rms_px = if edges.is_empty() {
        f64::INFINITY
    } else {
        (edges
            .iter()
            .map(|edge| edge.reprojection_rms_px.powi(2))
            .sum::<f64>()
            / edges.len() as f64)
            .sqrt()
    };
    Ok(RigEstimate {
        edges,
        cycles,
        rig_rms_px,
    })
}

/// 从参考相机做 BFS，组合出每个相机的 `T_ci_reference`（找不到路的相机会被报出来）。
///
/// **确定性**：邻接表用 `BTreeMap`，邻居按相机 id 字典序遍历。真机机架常带轻微不一致的闭环
/// （边各自独立求解），此时「先到先得」的 BFS 结果取决于遍历顺序；`HashMap` 的迭代顺序跨
/// 进程/线程不稳定，会让同一次标定在两次运行里得到不同的 `T_ci_c0`，进而让 extrinsics.yaml
/// 与 camchain.yaml 互相矛盾。按 id 排序给出唯一的**最短路径 + 字典序打平**结果。
pub fn reference_transforms(
    rig: &RigEstimate,
    reference_camera: &str,
) -> Result<BTreeMap<String, Matrix4>, ExtrinsicsError> {
    let mut adjacency: BTreeMap<String, BTreeMap<String, Matrix4>> = BTreeMap::new();
    for edge in &rig.edges {
        adjacency
            .entry(edge.camera_a.clone())
            .or_default()
            .insert(edge.camera_b.clone(), edge.transform);
        adjacency
            .entry(edge.camera_b.clone())
            .or_default()
            .insert(edge.camera_a.clone(), invert(&edge.transform));
    }
    let mut transforms: BTreeMap<String, Matrix4> = BTreeMap::new();
    transforms.insert(reference_camera.to_owned(), identity4());
    let mut queue = vec![reference_camera.to_owned()];
    while let Some(node) = queue.first().cloned() {
        queue.remove(0);
        if let Some(neighbours) = adjacency.get(&node) {
            for (neighbour, step) in neighbours {
                if !transforms.contains_key(neighbour) {
                    let composed = multiply(step, &transforms[&node]);
                    transforms.insert(neighbour.clone(), composed);
                    queue.push(neighbour.clone());
                }
            }
        }
    }
    let missing: Vec<String> = adjacency
        .keys()
        .filter(|camera| !transforms.contains_key(*camera))
        .cloned()
        .collect();
    if !missing.is_empty() {
        let mut missing = missing;
        missing.sort();
        return Err(ExtrinsicsError::Disconnected {
            reference: reference_camera.to_owned(),
            missing,
        });
    }
    Ok(transforms)
}

/// `cam0 < cam1 < …` 的自然序（非 `camN` 的名字排在最后，按字典序）。
pub fn camera_sort_key(camera_id: &str) -> (i64, String) {
    let suffix = camera_id.strip_prefix("cam").unwrap_or("");
    match suffix.parse::<i64>() {
        Ok(index) => (index, camera_id.to_owned()),
        Err(_) => (i64::MAX, camera_id.to_owned()),
    }
}

/// 由**强类型参数**按官方 Kalibr 语义产出 `(intrinsics, distortion_coeffs)`。
///
/// 直接匹配 `Parameters` 的变体字段，绝不用 `as_vector()` 的下标切片——那样会沿用核心向量序，
/// 而 Kalibr 的 Double Sphere 序是 `[xi, alpha, fx, fy, cx, cy]`
/// （<https://github.com/ethz-asl/kalibr/wiki/Supported-models>），与核心序不同。
/// pinhole+equidistant（KB4）：intrinsics `[fx, fy, cx, cy]`、畸变 `[k1, k2, k3, k4]`。
///
/// 缺内参、模型不一致或含非有限值一律报错——绝不静默补空数组把坏数据写进 camchain。
fn kalibr_intrinsics(
    camera_id: &str,
    parameters: &HashMap<String, Parameters>,
    model: ModelKind,
) -> Result<(Vec<f64>, Vec<f64>), ExtrinsicsError> {
    let params = parameters
        .get(camera_id)
        .ok_or_else(|| ExtrinsicsError::MissingIntrinsics {
            camera: camera_id.to_owned(),
        })?;
    if params.kind() != model {
        return Err(ExtrinsicsError::IntrinsicsModelMismatch {
            camera: camera_id.to_owned(),
            expected: model,
            found: params.kind(),
        });
    }
    let (intrinsics, distortion) = match params {
        Parameters::Ds(p) => (vec![p.xi, p.alpha, p.fx, p.fy, p.cx, p.cy], Vec::new()),
        Parameters::Kb4(p) => (vec![p.fx, p.fy, p.cx, p.cy], vec![p.k1, p.k2, p.k3, p.k4]),
    };
    if intrinsics
        .iter()
        .chain(distortion.iter())
        .any(|value| !value.is_finite())
    {
        return Err(ExtrinsicsError::NonFiniteIntrinsics {
            camera: camera_id.to_owned(),
        });
    }
    Ok((intrinsics, distortion))
}

/// Kalibr camchain 的**载荷**（相机 id → 条目），顺序为自然序，`T_cn_cnm1` 满足
/// `P_cn = T_cn_cnm1 · P_cnm1`。
pub fn camchain_payload(
    rig: &RigEstimate,
    reference_camera: &str,
    parameters: &HashMap<String, Parameters>,
    model: ModelKind,
    resolution: (u32, u32),
    target: CamchainTarget,
) -> Result<serde_yaml::Mapping, ExtrinsicsError> {
    let reference_transforms = reference_transforms(rig, reference_camera)?;
    let mut camera_ids: Vec<String> = reference_transforms.keys().cloned().collect();
    camera_ids.sort_by_key(|camera_id| camera_sort_key(camera_id));
    let mut payload = serde_yaml::Mapping::new();
    for (index, camera_id) in camera_ids.iter().enumerate() {
        let (camera_model, distortion_model) = match model {
            ModelKind::Ds => ("ds", "none"),
            ModelKind::Kb4 => ("pinhole", "equidistant"),
        };
        let (intrinsics, distortion) = kalibr_intrinsics(camera_id, parameters, model)?;
        let mut entry = serde_yaml::Mapping::new();
        entry.insert(text_key("camera_model"), text_value(camera_model));
        entry.insert(text_key("distortion_model"), text_value(distortion_model));
        entry.insert(
            text_key("intrinsics"),
            serde_yaml::Value::Sequence(
                intrinsics
                    .iter()
                    .map(|value| number_value(*value))
                    .collect(),
            ),
        );
        entry.insert(
            text_key("distortion_coeffs"),
            serde_yaml::Value::Sequence(
                distortion
                    .iter()
                    .map(|value| number_value(*value))
                    .collect(),
            ),
        );
        entry.insert(
            text_key("resolution"),
            serde_yaml::Value::Sequence(vec![
                number_value(resolution.0 as f64),
                number_value(resolution.1 as f64),
            ]),
        );
        entry.insert(
            text_key("rostopic"),
            text_value(&format!("/{camera_id}/image_raw")),
        );
        entry.insert(text_key("target_type"), text_value("aprilgrid"));
        entry.insert(text_key("tagRows"), number_value(target.rows as f64));
        entry.insert(text_key("tagCols"), number_value(target.cols as f64));
        entry.insert(text_key("tagSize"), number_value(target.tag_size_m));
        entry.insert(
            text_key("tagSpacing"),
            number_value(target.tag_spacing_ratio),
        );
        if index > 0 {
            let previous = &camera_ids[index - 1];
            // T_cn_cnm1 = T_cn_c0 · T_cnm1_c0⁻¹（与 P_cn = T_cn_cnm1 · P_cnm1 同向）
            let transform = multiply(
                &reference_transforms[camera_id],
                &invert(&reference_transforms[previous]),
            );
            entry.insert(
                text_key("T_cn_cnm1"),
                serde_yaml::Value::Sequence(
                    transform
                        .iter()
                        .map(|row| {
                            serde_yaml::Value::Sequence(
                                row.iter().map(|value| number_value(*value)).collect(),
                            )
                        })
                        .collect(),
                ),
            );
        }
        payload.insert(text_value(camera_id), serde_yaml::Value::Mapping(entry));
    }
    Ok(payload)
}

fn text_key(key: &str) -> serde_yaml::Value {
    serde_yaml::Value::String(key.to_owned())
}

fn text_value(text: &str) -> serde_yaml::Value {
    serde_yaml::Value::String(text.to_owned())
}

fn number_value(value: f64) -> serde_yaml::Value {
    serde_yaml::Value::Number(serde_yaml::Number::from(value))
}

#[derive(Clone, Copy, Debug)]
pub struct CamchainTarget {
    pub rows: usize,
    pub cols: usize,
    pub tag_size_m: f64,
    pub tag_spacing_ratio: f64,
}

#[cfg(test)]
mod tests {
    use super::{average_rotation, compose, identity4, invert, multiply, rotation_angle_deg};

    fn rotation_z(angle: f64) -> [[f64; 3]; 3] {
        [
            [angle.cos(), -angle.sin(), 0.0],
            [angle.sin(), angle.cos(), 0.0],
            [0.0, 0.0, 1.0],
        ]
    }

    #[test]
    fn inversion_round_trips() {
        let transform = compose(&rotation_z(0.4), &[0.3, -0.2, 0.7]);
        let round_trip = multiply(&transform, &invert(&transform));
        let identity = identity4();
        for row in 0..4 {
            for column in 0..4 {
                assert!(
                    (round_trip[row][column] - identity[row][column]).abs() < 1e-12,
                    "T·T⁻¹ must be identity at ({row},{column})"
                );
            }
        }
    }

    #[test]
    fn rotation_angle_matches_the_known_angle() {
        for angle in [0.0, 0.05, 0.5, 3.1] {
            let measured = rotation_angle_deg(&rotation_z(angle));
            assert!(
                (measured - angle.to_degrees()).abs() < 1e-9,
                "angle {angle} rad → {measured} deg"
            );
        }
    }

    #[test]
    fn averaging_recovers_the_common_rotation() {
        let clean = rotation_z(0.7);
        // 五份完全相同的旋转：平均必须还原它（正交性由四元数法保证）
        let mean = average_rotation(&[clean; 5]).expect("mean");
        assert!(
            rotation_angle_deg(&super::mat_mul(&super::transpose(&mean), &clean)) < 1e-9,
            "average of identical rotations must be that rotation"
        );
        // 6 份 0.7 rad + 1 份 1.2 rad：平均应明显靠近多数派
        let outlier = rotation_z(1.2);
        let mut samples = vec![clean; 6];
        samples.push(outlier);
        let robust = average_rotation(&samples).expect("mean");
        let to_majority = rotation_angle_deg(&super::mat_mul(&super::transpose(&robust), &clean));
        let to_outlier = rotation_angle_deg(&super::mat_mul(&super::transpose(&robust), &outlier));
        assert!(
            to_majority < to_outlier,
            "average must sit closer to the majority ({to_majority} deg) than to the outlier ({to_outlier} deg)"
        );
        assert!(
            to_majority < 5.0,
            "average must stay near the majority: {to_majority} deg"
        );
    }

    use super::{
        CamchainTarget, EdgeEstimate, Matrix4, ModelKind, Parameters, RigEstimate,
        camchain_payload, reference_transforms,
    };
    use crate::models::ds::DsParameters;
    use crate::models::kb4::Kb4Parameters;

    fn ds_parameters() -> Parameters {
        Parameters::Ds(DsParameters {
            fx: 576.0,
            fy: 576.0,
            cx: 640.0,
            cy: 544.0,
            xi: 0.4,
            alpha: 0.55,
        })
    }

    fn kb4_parameters() -> Parameters {
        Parameters::Kb4(Kb4Parameters {
            fx: 640.0,
            fy: 632.0,
            cx: 640.0,
            cy: 544.0,
            k1: -0.012,
            k2: 0.003,
            k3: -0.001,
            k4: 0.0002,
        })
    }

    fn camera_map(parameters: Parameters) -> std::collections::HashMap<String, Parameters> {
        ["cam0", "cam1", "cam2", "cam3"]
            .iter()
            .map(|camera| ((*camera).to_owned(), parameters))
            .collect()
    }

    fn edge(camera_a: &str, camera_b: &str, transform: Matrix4) -> EdgeEstimate {
        EdgeEstimate {
            camera_a: camera_a.to_owned(),
            camera_b: camera_b.to_owned(),
            transform,
            groups: 5,
            rotation_sigma_deg: 0.0,
            translation_sigma_mm: 0.0,
            reprojection_rms_px: 0.0,
        }
    }

    /// 链式机架的 RigEstimate：cam0 → cam1 → cam2 → cam3。
    fn chain_rig() -> RigEstimate {
        RigEstimate {
            edges: vec![
                edge(
                    "cam0",
                    "cam1",
                    compose(&rotation_z(0.01), &[0.10, 0.0, 0.0]),
                ),
                edge(
                    "cam1",
                    "cam2",
                    compose(&rotation_z(-0.02), &[0.0, 0.05, 0.0]),
                ),
                edge(
                    "cam2",
                    "cam3",
                    compose(&rotation_z(0.03), &[0.0, 0.0, 0.08]),
                ),
            ],
            cycles: Vec::new(),
            rig_rms_px: 0.0,
        }
    }

    fn payload_for(
        rig: &RigEstimate,
        parameters: &std::collections::HashMap<String, Parameters>,
        model: ModelKind,
    ) -> serde_yaml::Mapping {
        camchain_payload(
            rig,
            "cam0",
            parameters,
            model,
            (1280, 1088),
            CamchainTarget {
                rows: 6,
                cols: 6,
                tag_size_m: 0.04,
                tag_spacing_ratio: 0.01,
            },
        )
        .expect("payload")
    }

    fn emitted_numbers(payload: &serde_yaml::Mapping, camera: &str, key: &str) -> Vec<f64> {
        payload
            .get(camera)
            .and_then(|entry| entry.get(key))
            .and_then(|value| value.as_sequence())
            .unwrap_or_else(|| panic!("{camera}.{key} must be a sequence"))
            .iter()
            .map(|value| value.as_f64().expect("number"))
            .collect()
    }

    fn assert_matrices_close(left: &Matrix4, right: &Matrix4) {
        for row in 0..4 {
            for column in 0..4 {
                assert!(
                    (left[row][column] - right[row][column]).abs() < 1e-12,
                    "[{row}][{column}]: {} vs {}",
                    left[row][column],
                    right[row][column]
                );
            }
        }
    }

    /// 导出的 DS intrinsics 必须按官方 Kalibr 序 `[xi, alpha, fx, fy, cx, cy]` 消费。
    ///
    /// 用该序重建 `DsParameters` 后投出的像素必须与原始强类型参数一致。若仍按核心向量序导出
    /// （`[fx, fy, cx, cy, xi, alpha]`），重建出的 `xi=576`、`alpha=576` 会超出 DS 有效域
    /// （`0 < alpha < 1`）→ 投影无效、本测试失败。
    #[test]
    fn ds_camchain_intrinsics_follow_kalibr_order() {
        let parameters = camera_map(ds_parameters());
        let payload = payload_for(&chain_rig(), &parameters, ModelKind::Ds);

        assert_eq!(
            emitted_numbers(&payload, "cam0", "intrinsics"),
            vec![0.4, 0.55, 576.0, 576.0, 640.0, 544.0],
            "DS intrinsics must be [xi, alpha, fx, fy, cx, cy]"
        );

        let emitted = emitted_numbers(&payload, "cam0", "intrinsics");
        let consumed = Parameters::Ds(DsParameters {
            xi: emitted[0],
            alpha: emitted[1],
            fx: emitted[2],
            fy: emitted[3],
            cx: emitted[4],
            cy: emitted[5],
        });
        let points = [[0.2, -0.15, 1.3], [-0.4, 0.3, 2.0]];
        let (native_pixels, native_valid) =
            crate::models::project(ModelKind::Ds, &points, &ds_parameters());
        let (consumed_pixels, consumed_valid) =
            crate::models::project(ModelKind::Ds, &points, &consumed);
        assert_eq!(native_valid, consumed_valid, "validity domain must match");
        assert!(
            native_valid.iter().all(|valid| *valid),
            "the guard points must stay inside the projection domain"
        );
        for (index, (native, observed)) in
            native_pixels.iter().zip(consumed_pixels.iter()).enumerate()
        {
            for axis in 0..2 {
                assert!(
                    (native[axis] - observed[axis]).abs() < 1e-9,
                    "point {index} axis {axis}: {native:?} vs {observed:?}"
                );
            }
        }
    }

    /// 导出的 KB4（pinhole + equidistant）必须是 intrinsics `[fx, fy, cx, cy]` + 畸变 `[k1..k4]`，
    /// 按该语义重建后投出的像素必须与原始强类型参数一致。
    #[test]
    fn kb4_camchain_intrinsics_follow_kalibr_order() {
        let parameters = camera_map(kb4_parameters());
        let payload = payload_for(&chain_rig(), &parameters, ModelKind::Kb4);

        assert_eq!(
            emitted_numbers(&payload, "cam0", "intrinsics"),
            vec![640.0, 632.0, 640.0, 544.0]
        );
        assert_eq!(
            emitted_numbers(&payload, "cam0", "distortion_coeffs"),
            vec![-0.012, 0.003, -0.001, 0.0002]
        );

        let intrinsics = emitted_numbers(&payload, "cam0", "intrinsics");
        let distortion = emitted_numbers(&payload, "cam0", "distortion_coeffs");
        let consumed = Parameters::Kb4(Kb4Parameters {
            fx: intrinsics[0],
            fy: intrinsics[1],
            cx: intrinsics[2],
            cy: intrinsics[3],
            k1: distortion[0],
            k2: distortion[1],
            k3: distortion[2],
            k4: distortion[3],
        });
        let points = [[0.2, -0.15, 1.3], [-0.4, 0.3, 2.0]];
        let (native_pixels, native_valid) =
            crate::models::project(ModelKind::Kb4, &points, &kb4_parameters());
        let (consumed_pixels, consumed_valid) =
            crate::models::project(ModelKind::Kb4, &points, &consumed);
        assert_eq!(native_valid, consumed_valid, "validity domain must match");
        for (index, (native, observed)) in
            native_pixels.iter().zip(consumed_pixels.iter()).enumerate()
        {
            for axis in 0..2 {
                assert!(
                    (native[axis] - observed[axis]).abs() < 1e-9,
                    "point {index} axis {axis}: {native:?} vs {observed:?}"
                );
            }
        }
    }

    /// 轻微不一致的闭环上，`reference_transforms` 必须给出唯一确定的结果：
    /// **最短路径 + 相机 id 字典序打平**。cam2 到 cam0 有两条同长路径
    /// （cam0-cam1-cam2 与 cam0-cam3-cam2），必须选字典序更小的 cam1。
    /// 导出的 camchain 链与参考系变换必须出自同一份结果（两个产物互相矛盾即为回归）。
    #[test]
    fn ring_reference_transforms_are_deterministic() {
        let t_1_0 = compose(&rotation_z(0.05), &[0.10, 0.0, 0.0]);
        let t_2_1 = compose(&rotation_z(0.06), &[0.0, 0.10, 0.0]);
        let t_3_2 = compose(&rotation_z(0.07), &[0.0, 0.0, 0.10]);
        // 收尾边刻意不闭合：四条边各自独立求解的典型「轻微不一致」闭环
        let t_0_3 = compose(&rotation_z(0.08), &[-0.10, -0.10, -0.10]);
        let rig = RigEstimate {
            edges: vec![
                edge("cam0", "cam1", t_1_0),
                edge("cam1", "cam2", t_2_1),
                edge("cam2", "cam3", t_3_2),
                edge("cam3", "cam0", t_0_3),
            ],
            cycles: Vec::new(),
            rig_rms_px: 0.0,
        };

        let reference = reference_transforms(&rig, "cam0").expect("reference");
        assert_matrices_close(&reference["cam0"], &identity4());
        assert_matrices_close(&reference["cam1"], &t_1_0);
        assert_matrices_close(&reference["cam3"], &invert(&t_0_3));
        assert_matrices_close(&reference["cam2"], &multiply(&t_2_1, &t_1_0));

        // 导出链逐环累乘必须还原同一份参考系变换
        let parameters = camera_map(ds_parameters());
        let payload = payload_for(&rig, &parameters, ModelKind::Ds);
        let mut accumulated = identity4();
        for camera in ["cam0", "cam1", "cam2", "cam3"] {
            let entry = payload.get(camera).expect("camera entry");
            if let Some(chain) = entry.get("T_cn_cnm1") {
                let mut step = identity4();
                for (row, target_row) in step.iter_mut().enumerate() {
                    for (column, target) in target_row.iter_mut().enumerate() {
                        *target = chain[row][column].as_f64().expect("value");
                    }
                }
                accumulated = multiply(&step, &accumulated);
            }
            assert_matrices_close(&accumulated, &reference[camera]);
        }
    }
}
