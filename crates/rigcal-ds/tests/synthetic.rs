//! 倾斜平面靶的无噪声回归：不仅要求低重投影误差，也必须还原 DS 真值。
//! 通用平面标定不存在“焦距和深度同时缩放就完全不变”的自由度。

use rigcal_core::estimator::{Observation, Status, project_with_pose};
use rigcal_core::models::{ModelKind, Parameters, ds::DsParameters};
use rigcal_ds::{DsBundleOptions, DsSeed, solve_ds_intrinsics};

fn planted() -> DsParameters {
    DsParameters {
        fx: 648.0,
        fy: 646.5,
        cx: 640.0,
        cy: 544.0,
        xi: 0.18,
        alpha: 0.56,
    }
}

fn observations(views: usize) -> Vec<Observation> {
    let params = Parameters::Ds(planted());
    let objects: Vec<[f64; 3]> = (0..8)
        .flat_map(|row| (0..8).map(move |column| [column as f64 * 0.10, row as f64 * 0.10, 0.0]))
        .collect();
    (0..views)
        .map(|view| {
            let step = view as f64;
            let rvec = [
                0.25 + 0.10 * (0.7 * step).sin(),
                -0.15 + 0.12 * (0.5 * step).cos(),
                0.10 + 0.15 * (0.3 * step).sin(),
            ];
            let tvec = [
                0.03 * (0.4 * step).sin(),
                -0.02 + 0.03 * (0.6 * step).cos(),
                0.75 + 0.07 * step,
            ];
            let (image_points, valid) =
                project_with_pose(ModelKind::Ds, &params, &objects, &rvec, &tvec);
            assert!(valid.iter().all(|v| *v));
            Observation {
                object_points: objects.clone(),
                image_points,
            }
        })
        .collect()
}

#[test]
fn recovers_a_planted_ds_camera_from_a_cold_start() {
    let views = observations(10);
    let result =
        solve_ds_intrinsics(&views, (1280, 1088), &DsBundleOptions::default()).expect("solve");
    assert_eq!(result.status, Status::Pass);
    assert_eq!(result.invalid_projection_count, 0);
    assert!(result.rms_px < 1e-7, "rms={}", result.rms_px);
    let Parameters::Ds(solved) = result.parameters else {
        panic!("expected DS")
    };
    let truth = planted();
    assert!((solved.fx / truth.fx - 1.0).abs() < 1e-5, "{solved:?}");
    assert!((solved.fy / truth.fy - 1.0).abs() < 1e-5, "{solved:?}");
    assert!((solved.cx - truth.cx).abs() < 1e-5, "{solved:?}");
    assert!((solved.cy - truth.cy).abs() < 1e-5, "{solved:?}");
    assert!((solved.xi - truth.xi).abs() < 1e-5, "{solved:?}");
    assert!((solved.alpha - truth.alpha).abs() < 1e-5, "{solved:?}");
}

#[test]
fn a_solved_pose_is_a_valid_warm_start_for_the_next_solve() {
    // 冷启动 → 用返回位姿热启动：输出位姿必须满足输入契约（tz > 0），
    // 否则自由 tz 会产出无法回喂给下一轮的位姿。
    let views = observations(10);
    let cold =
        solve_ds_intrinsics(&views, (1280, 1088), &DsBundleOptions::default()).expect("cold solve");
    assert!(
        cold.poses.iter().all(|(_, tvec)| tvec[2] > 0.0),
        "求解输出的 tz 必须为正，否则下一轮热启动会被输入检查拒绝"
    );
    let seed = match cold.parameters {
        Parameters::Ds(ds) => DsSeed::Parameters(ds),
        Parameters::Kb4(_) => panic!("期望 DS"),
    };
    let warm = solve_ds_intrinsics(
        &views,
        (1280, 1088),
        &DsBundleOptions {
            seed,
            initial_poses: Some(&cold.poses),
            ..DsBundleOptions::default()
        },
    )
    .expect("热启动不得被 tz 契约拒绝");
    assert_eq!(warm.status, Status::Pass);
    assert!(warm.poses.iter().all(|(_, tvec)| tvec[2] > 0.0));
    assert!(
        warm.rms_px <= cold.rms_px + 1e-6,
        "热启动退化：{} vs {}",
        warm.rms_px,
        cold.rms_px
    );
}

#[test]
fn negative_depth_warm_start_is_still_rejected() {
    // 正值重参数化不能退化成"删掉输入检查"：显式给出 tz ≤ 0 的位姿仍必须报错。
    let views = observations(10);
    let mut poses: Vec<_> = (0..views.len())
        .map(|_| ([0.0, 0.0, 0.0], [0.0, 0.0, 1.0]))
        .collect();
    poses[3].1[2] = -0.5;
    let result = solve_ds_intrinsics(
        &views,
        (1280, 1088),
        &DsBundleOptions {
            initial_poses: Some(&poses),
            ..DsBundleOptions::default()
        },
    );
    assert!(result.is_err(), "tz ≤ 0 的热启动必须被拒绝");
}
