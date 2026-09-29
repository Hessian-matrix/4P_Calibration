//! 外参的**合成已知机架**验收：数学正确性在这里卡死（真机夹具受"占位内参下位姿不唯一"限制，
//! 只能守结构线）。
//!
//! 做法：先种一个已知机架与已知板位姿，用**冻结内参**把板点投到每一路拿到无噪声观测，再让外参
//! 求解器反推——它必须把种下去的变换还原出来。观测刻意做成：
//! - 各路**可见 tag 子集不同**（cam2 少看一半、cam3 只看四分之一）；
//! - 每路的角点顺序**打乱**（检测器输出顺序本来就与 tag 顺序无关）。
//!
//! 覆盖的关键点：必须按**物点值**配对而非按下标配对——合成数据下配对错误会直接让"还原出的
//! 变换"偏离种下的真值 → 本测试失败。

use std::collections::HashMap;

use rigcal_core::board::{AprilGridConfig, Corner};
use rigcal_core::estimator::{Observation, project_with_pose};
use rigcal_core::extrinsics::{
    CamchainTarget, Matrix4, ObservationRecord, camchain_payload, compose, invert, multiply,
    rotation_angle_deg, rotation_of, solve_rig,
};
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_core::rotation::matrix_to_rvec;

/// 允许的还原误差。位姿求解是迭代的（LM 收敛到 ~1e-10 px 级别），20 组平均后应远优于该线。
const TOLERANCE: f64 = 1e-6;

fn board() -> AprilGridConfig {
    AprilGridConfig {
        rows: 6,
        cols: 6,
        tag_size_m: 0.088,
        tag_spacing_ratio: 0.3,
        first_tag_id: 0,
        dictionary: "DICT_APRILTAG_36h11".to_owned(),
        target_id: "synthetic".to_owned(),
        tag_corner_order: [
            Corner::BottomRight,
            Corner::BottomLeft,
            Corner::TopLeft,
            Corner::TopRight,
        ],
    }
}

/// 各相机的冻结内参（KB4，带轻微畸变：既走真实投影路径，又保持无噪声）。
fn parameters() -> HashMap<String, Parameters> {
    let mut map = HashMap::new();
    for (index, camera) in ["cam0", "cam1", "cam2", "cam3"].iter().enumerate() {
        let focal = 640.0 + index as f64 * 8.0;
        map.insert(
            (*camera).to_owned(),
            Parameters::from_vector(
                ModelKind::Kb4,
                &[focal, focal, 640.0, 544.0, -0.012, 0.003, -0.001, 0.0002],
            )
            .expect("kb4 parameters"),
        );
    }
    map
}

/// 种下去的机架：`T_ci_c0`（把 c0 系下的点变到 ci 系）。
fn planted_rig() -> HashMap<String, Matrix4> {
    let mut rig = HashMap::new();
    rig.insert(
        "cam0".to_owned(),
        compose(&rvec_to_matrix(&[0.0; 3]), &[0.0, 0.0, 0.0]),
    );
    rig.insert(
        "cam1".to_owned(),
        compose(&rvec_to_matrix(&[0.02, -0.01, 0.015]), &[0.12, 0.01, -0.02]),
    );
    rig.insert(
        "cam2".to_owned(),
        compose(
            &rvec_to_matrix(&[-0.01, 0.03, -0.02]),
            &[0.25, -0.015, 0.03],
        ),
    );
    rig.insert(
        "cam3".to_owned(),
        compose(&rvec_to_matrix(&[0.015, 0.02, -0.03]), &[0.13, 0.02, 0.04]),
    );
    rig
}

fn rvec_to_matrix(rvec: &[f64; 3]) -> [[f64; 3]; 3] {
    rigcal_core::rotation::rvec_to_matrix(rvec)
}

/// 每路可见的 tag（刻意不同）与其角点顺序（刻意打乱）。
fn visible_tags(camera_index: usize, total: usize) -> Vec<usize> {
    let mut tags: Vec<usize> = (0..total).collect();
    match camera_index {
        0 => {}
        1 => tags.retain(|tag| tag % 6 != 5),
        2 => tags.retain(|tag| *tag < 24),
        _ => tags.retain(|tag| *tag < 9),
    }
    // 确定性打乱（LCG）：顺序与 tag 顺序无关
    let mut state: u64 = 0x9e37_79b9_7f4a_7c15 ^ camera_index as u64;
    for index in (1..tags.len()).rev() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        let swap = (state % (index as u64 + 1)) as usize;
        tags.swap(index, swap);
    }
    tags
}

/// 造 N 组观测：板位姿逐组变化，四路各自投影（无噪声）。
fn records(groups: usize) -> Vec<ObservationRecord> {
    let board = board();
    let parameters = parameters();
    let rig = planted_rig();
    let mut records = Vec::new();
    for group in 0..groups {
        let step = group as f64;
        // 板位姿（相对 cam0）：轻微平移 + 三轴小角度，保证姿态多样
        let board_pose_c0 = compose(
            &rvec_to_matrix(&[
                0.05 * (0.7 * step).sin(),
                0.06 * (0.5 * step).cos(),
                0.08 * (0.3 * step).sin(),
            ]),
            &[0.02 * step.cos(), 0.01 * step, 0.9 + 0.01 * step],
        );
        for (camera_index, camera) in ["cam0", "cam1", "cam2", "cam3"].iter().enumerate() {
            let transform = multiply(&rig[*camera], &board_pose_c0); // T_ci_board
            let tags = visible_tags(camera_index, board.rows * board.cols);
            let object_points = board.object_points(&tags).expect("object points");
            let (pixels, valid) = project_with_pose(
                ModelKind::Kb4,
                &parameters[*camera],
                &object_points,
                &matrix_to_rvec(&rotation_of(&transform)),
                &[transform[0][3], transform[1][3], transform[2][3]],
            );
            assert!(
                valid.iter().all(|value| *value),
                "synthetic observations must stay inside the projection domain"
            );
            records.push(ObservationRecord {
                camera_id: (*camera).to_owned(),
                group_id: group as u64,
                observation: Observation {
                    object_points,
                    image_points: pixels,
                },
            });
        }
    }
    records
}

fn close(left: f64, right: f64, what: &str) {
    assert!(
        (left - right).abs() <= TOLERANCE,
        "{what}: {left} vs planted {right} (tolerance {TOLERANCE})"
    );
}

#[test]
fn planted_rig_is_recovered_exactly() {
    let records = records(16);
    let parameters = parameters();
    let rig = solve_rig(
        &records,
        &parameters,
        ModelKind::Kb4,
        &[
            ("cam0".to_owned(), "cam1".to_owned()),
            ("cam1".to_owned(), "cam2".to_owned()),
            ("cam2".to_owned(), "cam3".to_owned()),
            ("cam3".to_owned(), "cam0".to_owned()),
        ],
        &[vec![
            "cam0".to_owned(),
            "cam1".to_owned(),
            "cam2".to_owned(),
            "cam3".to_owned(),
        ]],
        3,
    )
    .expect("rig");

    let planted = planted_rig();
    for edge in &rig.edges {
        // 真值 T_b_a = T_b_c0 · T_a_c0⁻¹
        let expected = multiply(&planted[&edge.camera_b], &invert(&planted[&edge.camera_a]));
        assert_eq!(
            edge.groups, 16,
            "{}-{} uses every co-visible group",
            edge.camera_a, edge.camera_b
        );
        for (row, actual_row) in edge.transform.iter().enumerate() {
            for (column, actual) in actual_row.iter().enumerate() {
                close(
                    *actual,
                    expected[row][column],
                    &format!("{}-{} T[{row}][{column}]", edge.camera_a, edge.camera_b),
                );
            }
        }
        // 无噪声 + 同一真值：逐组离散度必须为 0，重投影 rms 也必须为 0
        assert!(
            edge.rotation_sigma_deg <= TOLERANCE && edge.translation_sigma_mm <= TOLERANCE,
            "{}-{} per-group spread must vanish: {:.3e}° / {:.3e} mm",
            edge.camera_a,
            edge.camera_b,
            edge.rotation_sigma_deg,
            edge.translation_sigma_mm
        );
        assert!(
            edge.reprojection_rms_px <= TOLERANCE,
            "{}-{} reprojection rms must vanish: {:.3e} px",
            edge.camera_a,
            edge.camera_b,
            edge.reprojection_rms_px
        );
    }

    // 环路必须精确闭合
    assert_eq!(rig.cycles.len(), 1, "the requested cycle must be checked");
    let cycle = &rig.cycles[0];
    assert!(
        cycle.rotation_error_deg <= TOLERANCE && cycle.translation_error_mm <= TOLERANCE,
        "cycle must close: {:.3e}° / {:.3e} mm",
        cycle.rotation_error_deg,
        cycle.translation_error_mm
    );
    assert!(
        rig.rig_rms_px <= TOLERANCE,
        "rig rms must vanish: {:.3e} px",
        rig.rig_rms_px
    );
    println!(
        "planted rig recovered: rig_rms={:.3e}px cycle={:.3e}°/​{:.3e}mm",
        rig.rig_rms_px, cycle.rotation_error_deg, cycle.translation_error_mm
    );
}

#[test]
fn camchain_reproduces_the_planted_transforms() {
    let records = records(12);
    let parameters = parameters();
    let edges: Vec<(String, String)> = vec![
        ("cam0".to_owned(), "cam1".to_owned()),
        ("cam1".to_owned(), "cam2".to_owned()),
        ("cam2".to_owned(), "cam3".to_owned()),
        ("cam3".to_owned(), "cam0".to_owned()),
    ];
    let rig = solve_rig(&records, &parameters, ModelKind::Kb4, &edges, &[], 3).expect("rig");
    let payload = camchain_payload(
        &rig,
        "cam0",
        &parameters,
        ModelKind::Kb4,
        (1280, 1088),
        CamchainTarget {
            rows: 6,
            cols: 6,
            tag_size_m: 0.088,
            tag_spacing_ratio: 0.3,
        },
    )
    .expect("payload");
    let planted = planted_rig();
    // 逐链组合：cam_i 的 T_cn_cnm1 必须满足 P_ci = T_ci_c{i-1} · P_c{i-1}
    let mut accumulated = rigcal_core::extrinsics::identity4();
    for (index, camera) in ["cam0", "cam1", "cam2", "cam3"].iter().enumerate() {
        let entry = payload[&serde_yaml::Value::String((*camera).to_owned())].clone();
        let yaml = serde_yaml::to_string(&entry).expect("entry to yaml");
        let value: serde_yaml::Value = serde_yaml::from_str(&yaml).expect("entry parse");
        if index == 0 {
            assert!(
                value.get("T_cn_cnm1").is_none(),
                "the reference camera carries no chain transform"
            );
            accumulated = rigcal_core::extrinsics::identity4();
            continue;
        }
        let matrix = value.get("T_cn_cnm1").expect("chain transform");
        let mut transform = rigcal_core::extrinsics::identity4();
        for (row, target_row) in transform.iter_mut().enumerate() {
            for (column, target) in target_row.iter_mut().enumerate() {
                *target = matrix[row][column].as_f64().expect("value");
            }
        }
        accumulated = multiply(&transform, &accumulated);
        // 累积链必须等于种下的 T_ci_c0
        for (row, accumulated_row) in accumulated.iter().enumerate() {
            for (column, value) in accumulated_row.iter().enumerate() {
                close(
                    *value,
                    planted[*camera][row][column],
                    &format!("{camera} chained T_ci_c0[{row}][{column}]"),
                );
            }
        }
    }
    println!("camchain chain reproduces the planted rig to {TOLERANCE:.0e}");
    let _ = rotation_angle_deg;
}

/// 诊断：无噪声合成数据下，位姿求解器能收敛到什么精度（外参精度的上界由它决定）。
#[test]
fn pose_accuracy_on_synthetic_data() {
    let records = records(4);
    let parameters = parameters();
    let planted = planted_rig();
    let mut worst_rotation_deg = 0.0_f64;
    let mut worst_translation_mm = 0.0_f64;
    let mut worst_rms_px = 0.0_f64;
    for record in &records {
        let board = board();
        let camera = &record.camera_id;
        let group = record.group_id;
        // 重算种下的板位姿（与 records() 同一算式）
        let step = group as f64;
        let board_pose_c0 = compose(
            &rvec_to_matrix(&[
                0.05 * (0.7 * step).sin(),
                0.06 * (0.5 * step).cos(),
                0.08 * (0.3 * step).sin(),
            ]),
            &[0.02 * step.cos(), 0.01 * step, 0.9 + 0.01 * step],
        );
        let truth = multiply(&planted[camera], &board_pose_c0);
        let estimate = rigcal_core::extrinsics::board_pose(
            &record.observation,
            &parameters[camera],
            ModelKind::Kb4,
        )
        .expect("pose");
        let rotation_error = rotation_angle_deg(&rigcal_core::rotation::mat_mul(
            &rigcal_core::extrinsics::rotation_of(&estimate),
            &rigcal_core::rotation::transpose(&rigcal_core::extrinsics::rotation_of(&truth)),
        ));
        let translation_error = {
            let delta = [
                estimate[0][3] - truth[0][3],
                estimate[1][3] - truth[1][3],
                estimate[2][3] - truth[2][3],
            ];
            (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt() * 1000.0
        };
        if rotation_error > 1.0 || translation_error > 20.0 {
            let estimate_direct = rigcal_core::estimator::estimate_fixed_intrinsics_pose(
                ModelKind::Kb4,
                &parameters[camera],
                &record.observation,
            );
            println!(
                "  {camera} group {group}: rot_err={rotation_error:.3}° trans_err={translation_error:.1}mm \
                 rms={:.3e}px status={:?} iterations={}",
                estimate_direct.rms_px, estimate_direct.status, estimate_direct.iterations
            );
        }
        worst_rotation_deg = worst_rotation_deg.max(rotation_error);
        worst_translation_mm = worst_translation_mm.max(translation_error);
        let _ = board;
        worst_rms_px = worst_rms_px.max(0.0);
    }
    println!(
        "pose accuracy on noiseless synthetic data: worst rotation {worst_rotation_deg:.3e}°, \
         worst translation {worst_translation_mm:.3e} mm, {:.0} records",
        records.len()
    );
    assert!(
        worst_rotation_deg < 1.0 && worst_translation_mm < 10.0,
        "pose solver must stay within a sane bound on exact data"
    );
    let _ = worst_rms_px;
}
