//! 位姿路径的两条回归：**角点数规模鲁棒性** 与 **DS 射线部分过滤时的对应一致性**。
//!
//! 第一条：板只被部分看到时角点数会很少，最极端就是单个 tag 的 4 点——那会走 `homography_pose`
//! 的 8×8 方阵 DLT 分支（等价 DLT 设计矩阵 8×9，nalgebra `svd` 在行数 < 列数时内部越界）。
//! 所以这里把 "4..=144 个角点" 全扫一遍，并在**无噪声、已知位姿**的合成观测上要求它把位姿
//! 还原到机器精度。
//!
//! 第二条：`seed_pose` 用反投影射线播种，会丢掉 `ray.z <= 1e-4` 的角点；丢弃必须**成对**进行
//! ——物点、像点、归一化坐标一起丢。只过滤物点却留下整段像点会让单应配对整体错位，播种位姿
//! 崩溃：无噪声数据上"过滤后重投影误差"从 ~1e-12 px 抬到几百 px。这里造一块倾斜的平面板横跨
//! "相机射线 z 为正/为负"的分界，断言播种位姿在**保留点**上准确重投影。
//!
//! 断言只落在调用方可见的量上：收敛状态、无效投影计数、重投影 rms、还原出的平移/旋转误差、
//! 播种位姿的重投影像素——不看迭代次数，也不依赖私有辅助函数的内部结构。

use rigcal_core::board::{AprilGridConfig, Corner};
use rigcal_core::estimator::{
    Observation, Status, estimate_fixed_intrinsics_pose, project_with_pose, seed_pose,
};
use rigcal_core::models::{self, ModelKind, Parameters};
use rigcal_core::rotation::relative_rotation_deg;

/// KB4 规模回归种下的板位姿。
const KB4_RVEC: [f64; 3] = [0.15, -0.05, 0.08];
const KB4_TVEC: [f64; 3] = [0.01, -0.02, 0.92];

/// DS 过滤回归种下的板位姿：绕 y 轴倾斜 1.1 rad，板沿 x 张开到 ~0.69 m，使相机点
/// `z ≈ -x·sin(1.1) + 0.3` 在 x ≈ 0.337 m 处变号——同一块板上正面/背面角点交错出现。
const DS_RVEC: [f64; 3] = [0.0, 1.1, 0.0];
const DS_TVEC: [f64; 3] = [0.0, 0.0, 0.3];

/// `seed_pose` 丢弃反投影射线的阈值（`ray.z <= RAY_Z_FLOOR`）。
const RAY_Z_FLOOR: f64 = 1e-4;

/// 无噪声已知位姿下求解器应还原到的量级（各留 6 个数量级余量）。
const RMS_TOL_PX: f64 = 1e-6;
const ROTATION_TOL_DEG: f64 = 1e-4;
const TRANSLATION_TOL_M: f64 = 1e-6;

/// 播种位姿把物点投回原始像素的允许误差。错位配对会到几百 px，正常实现 ~1e-12 px。
const SEED_REPROJECTION_TOL_PX: f64 = 1e-4;

fn board() -> AprilGridConfig {
    AprilGridConfig {
        rows: 6,
        cols: 6,
        tag_size_m: 0.088,
        tag_spacing_ratio: 0.3,
        first_tag_id: 0,
        dictionary: "DICT_APRILTAG_36h11".to_owned(),
        target_id: "pose-robustness".to_owned(),
        tag_corner_order: [
            Corner::BottomRight,
            Corner::BottomLeft,
            Corner::TopLeft,
            Corner::TopRight,
        ],
    }
}

/// KB4 内参（带轻微畸变：走真实投影路径，同时保持无噪声）。
fn kb4_parameters() -> Parameters {
    Parameters::from_vector(
        ModelKind::Kb4,
        &[576.0, 576.0, 640.0, 544.0, -0.01, 0.002, 0.0, 0.0],
    )
    .expect("kb4")
}

/// DS 内参（宽角：正反投影都允许部分相机点 z ≤ 0）。
fn ds_parameters() -> Parameters {
    Parameters::from_vector(ModelKind::Ds, &[576.0, 576.0, 640.0, 544.0, 0.2, 0.6]).expect("ds")
}

/// 用已知位姿把整块板投成一份无噪声观测；全部角点都必须投影有效，否则测试前提不成立。
fn observation(
    kind: ModelKind,
    parameters: &Parameters,
    rvec: &[f64; 3],
    tvec: &[f64; 3],
) -> Observation {
    let board = board();
    let object_points = board
        .object_points(&(0..board.tag_count()).collect::<Vec<_>>())
        .expect("object points");
    let (image_points, valid) = project_with_pose(kind, parameters, &object_points, rvec, tvec);
    assert!(
        valid.iter().all(|value| *value),
        "生成的观测必须让每个角点都投影有效"
    );
    Observation {
        object_points,
        image_points,
    }
}

fn pixel_distance(left: &[f64; 2], right: &[f64; 2]) -> f64 {
    ((left[0] - right[0]).powi(2) + (left[1] - right[1]).powi(2)).sqrt()
}

fn translation_distance(left: &[f64; 3], right: &[f64; 3]) -> f64 {
    ((left[0] - right[0]).powi(2) + (left[1] - right[1]).powi(2) + (left[2] - right[2]).powi(2))
        .sqrt()
}

/// 单个 tag 的 4 点会喂给 `svd` 一个 8×9 矩阵（`Matrix slicing out of bounds` 的边界）。
/// 既是"不 panic"的边界回归，也要求生成的已知位姿被还原到机器精度。
#[test]
fn pose_is_recovered_for_every_corner_count() {
    let parameters = kb4_parameters();
    let full = observation(ModelKind::Kb4, &parameters, &KB4_RVEC, &KB4_TVEC);
    let total = full.object_points.len();
    assert_eq!(total, 144, "6×6 AprilGrid 应有 144 个角点");

    for corners in 4..=total {
        // 模拟"板只被看到一部分"：只取前 `corners` 个角点（4 个 = 恰好一个 tag）。
        let observation = Observation {
            object_points: full.object_points[..corners].to_vec(),
            image_points: full.image_points[..corners].to_vec(),
        };
        let estimate = estimate_fixed_intrinsics_pose(ModelKind::Kb4, &parameters, &observation);

        assert_eq!(
            estimate.status,
            Status::Pass,
            "{corners} 个角点：无噪声已知位姿必须收敛为 Pass"
        );
        assert_eq!(
            estimate.invalid_projection_count, 0,
            "{corners} 个角点：不应出现无效投影"
        );
        assert_eq!(
            estimate.residuals_px.len(),
            corners,
            "{corners} 个角点：残差必须逐点对应观测"
        );
        assert!(
            estimate.rms_px < RMS_TOL_PX,
            "{corners} 个角点：rms={:.3e} px 超过 {RMS_TOL_PX:.0e} px",
            estimate.rms_px
        );

        let rotation_error_deg = relative_rotation_deg(&estimate.rvec, &KB4_RVEC)
            .iter()
            .fold(0.0_f64, |worst, axis| worst.max(axis.abs()));
        assert!(
            rotation_error_deg < ROTATION_TOL_DEG,
            "{corners} 个角点：旋转误差 {rotation_error_deg:.3e}° 超过 {ROTATION_TOL_DEG:.0e}°"
        );
        let translation_error_m = translation_distance(&estimate.tvec, &KB4_TVEC);
        assert!(
            translation_error_m < TRANSLATION_TOL_M,
            "{corners} 个角点：平移误差 {translation_error_m:.3e} m 超过 {TRANSLATION_TOL_M:.0e} m"
        );
    }
}

/// DS 射线部分过滤：板横跨相机射线 z 的正负分界，`seed_pose` 只保留 z > 1e-4 的射线。
/// 丢弃必须成对——否则单应把"过滤后的物点"和"未过滤的像点前缀"配在一起，播种彻底错位。
#[test]
fn ds_seed_keeps_correspondences_paired_when_rays_are_filtered() {
    let parameters = ds_parameters();
    let observation = observation(ModelKind::Ds, &parameters, &DS_RVEC, &DS_TVEC);
    let total = observation.object_points.len();

    // 用公开的反投影复现"哪些射线会被播种丢弃"：有效、有限、方向 z > 阈值。
    let (rays, valid) = models::unproject(ModelKind::Ds, &observation.image_points, &parameters);
    let retained: Vec<usize> = (0..total)
        .filter(|index| valid[*index] && rays[*index][2] > RAY_Z_FLOOR)
        .collect();
    assert!(
        retained.len() >= 4,
        "前方射线太少，无法播种：保留 {} 条",
        retained.len()
    );
    assert!(
        retained.len() < total,
        "场景必须真的触发射线过滤（当前全部 {} 条射线的 z > {RAY_Z_FLOOR}）",
        total
    );
    let is_prefix = retained
        .iter()
        .enumerate()
        .all(|(slot, index)| slot == *index);
    assert!(
        !is_prefix,
        "被丢弃的射线必须分散在序列中间；否则只过滤物点也不会暴露错位"
    );

    let seed = seed_pose(ModelKind::Ds, &parameters, &observation);
    let seed_rvec = [seed[0], seed[1], seed[2]];
    let seed_tvec = [seed[3], seed[4], seed[5]];

    // ① 保留点：播种位姿必须把它们的物点投回各自原来的像素。
    let retained_object: Vec<[f64; 3]> = retained
        .iter()
        .map(|index| observation.object_points[*index])
        .collect();
    let (retained_pixels, retained_valid) = project_with_pose(
        ModelKind::Ds,
        &parameters,
        &retained_object,
        &seed_rvec,
        &seed_tvec,
    );
    assert!(
        retained_valid.iter().all(|value| *value),
        "保留角点在播种位姿下必须全部有效投影"
    );
    for (slot, index) in retained.iter().enumerate() {
        let error = pixel_distance(&retained_pixels[slot], &observation.image_points[*index]);
        assert!(
            error < SEED_REPROJECTION_TOL_PX,
            "保留点 {index} 重投影误差 {error:.3e} px 超过 {SEED_REPROJECTION_TOL_PX:.0e} px"
        );
    }

    // ② 全观测：播种出的位姿应当解释每个角点（包括被过滤、但仍合法投影的背面点）。
    let (all_pixels, all_valid) = project_with_pose(
        ModelKind::Ds,
        &parameters,
        &observation.object_points,
        &seed_rvec,
        &seed_tvec,
    );
    assert!(
        all_valid.iter().all(|value| *value),
        "播种位姿下不应出现无效投影"
    );
    let worst_error = all_pixels
        .iter()
        .zip(observation.image_points.iter())
        .map(|(pixel, expected)| pixel_distance(pixel, expected))
        .fold(0.0_f64, f64::max);
    assert!(
        worst_error < SEED_REPROJECTION_TOL_PX,
        "播种位姿对全观测的最差重投影误差 {worst_error:.3e} px 超过 {SEED_REPROJECTION_TOL_PX:.0e} px"
    );
}
