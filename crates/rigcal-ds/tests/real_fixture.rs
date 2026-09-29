//! 真机 36 视图回归：联合优化、位姿重估和下游评估必须都保持约 0.22 px。
//! 同一镜头的 DS 和 KB4 原始焦距不可直接比较；DS 轴心角度斜率为 f/(1+xi)。

use rigcal_core::estimator::{
    Observation, SolveRequest, Status, estimate_fixed_intrinsics_pose, evaluate_outcome,
    evaluate_solution,
};
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_ds::{DsBundleOptions, kb4_seed, solve_ds_intrinsics};
use serde_json::Value;

const FIXTURE: &str = include_str!("../../rigcal-opencv/tests/fixtures/kb4_real_36views.json");

fn fixture() -> (Vec<Observation>, (u32, u32)) {
    let value: Value = serde_json::from_str(FIXTURE).expect("fixture");
    let image_size = (
        value["image_size"][0].as_u64().unwrap() as u32,
        value["image_size"][1].as_u64().unwrap() as u32,
    );
    let observations = value["observations"]
        .as_array()
        .expect("observations")
        .iter()
        .map(|pair| {
            let object_points = pair[0]
                .as_array()
                .unwrap()
                .iter()
                .map(|point| {
                    let point = point.as_array().unwrap();
                    [
                        point[0].as_f64().unwrap(),
                        point[1].as_f64().unwrap(),
                        point[2].as_f64().unwrap(),
                    ]
                })
                .collect();
            let image_points = pair[1]
                .as_array()
                .unwrap()
                .iter()
                .map(|point| {
                    let point = point.as_array().unwrap();
                    [point[0].as_f64().unwrap(), point[1].as_f64().unwrap()]
                })
                .collect();
            Observation {
                object_points,
                image_points,
            }
        })
        .collect();
    (observations, image_size)
}

#[test]
fn rust_ds_matches_the_scipy_reference_on_real_data() {
    let (observations, image_size) = fixture();
    assert_eq!(observations.len(), 36);
    let kb4 = rigcal_opencv::solve_kb4(SolveRequest {
        kind: ModelKind::Kb4,
        observations: &observations,
        image_size,
        initial: None,
        initial_poses: None,
    })
    .expect("KB4 seed");
    let seed = kb4_seed(&kb4.parameters).expect("DS angular seed");
    let started = std::time::Instant::now();
    let outcome = solve_ds_intrinsics(
        &observations,
        image_size,
        &DsBundleOptions {
            seed,
            initial_poses: Some(&kb4.poses),
            ..DsBundleOptions::default()
        },
    )
    .expect("solve");
    let elapsed = started.elapsed();
    let solved = match outcome.parameters {
        Parameters::Ds(ds) => ds,
        other => panic!("期望 DS，得到 {other:?}"),
    };
    println!(
        "rust ds: rms={:.4}px（scipy 对拍 0.219969）  fx={:.3}（KB4 655.880）  xi={:.4} alpha={:.4}  \
         invalid={}  {:.1}s",
        outcome.rms_px,
        solved.fx,
        solved.xi,
        solved.alpha,
        outcome.invalid_projection_count,
        elapsed.as_secs_f64()
    );
    assert_eq!(outcome.status, Status::Pass);
    let joint = evaluate_outcome(ModelKind::Ds, &observations, &outcome);
    assert!((joint.rms_px - outcome.rms_px).abs() < 1e-10);
    let refreshed: Vec<_> = observations
        .iter()
        .map(|observation| {
            let pose =
                estimate_fixed_intrinsics_pose(ModelKind::Ds, &outcome.parameters, observation);
            assert_eq!(pose.status, Status::Pass);
            (pose.rvec, pose.tvec)
        })
        .collect();
    let refreshed = evaluate_solution(
        ModelKind::Ds,
        &observations,
        &outcome.parameters,
        &refreshed,
    );
    println!(
        "pose refresh: rms={:.6}px max={:.6}px",
        refreshed.rms_px,
        refreshed.per_view_rms.iter().copied().fold(0.0, f64::max)
    );
    assert_eq!(refreshed.invalid_projection_count, 0);
    assert!(refreshed.rms_px <= 0.23, "{refreshed:?}");
    assert!(
        refreshed.per_view_rms.iter().all(|rms| *rms < 0.6),
        "{refreshed:?}"
    );
    assert_eq!(outcome.invalid_projection_count, 0, "不允许无效投影");
    // 不得靠丢掉困难视图或丢弃优化位姿来满足该回归线。
    assert!(
        outcome.rms_px <= 0.23,
        "rms {:.4}px 超出真机夹具 0.23px 验收线",
        outcome.rms_px
    );
    let Parameters::Kb4(kb4) = kb4.parameters else {
        unreachable!()
    };
    assert!((solved.fx / (1.0 + solved.xi) / kb4.fx - 1.0).abs() < 0.01);
    assert!((solved.fy / (1.0 + solved.xi) / kb4.fy - 1.0).abs() < 0.01);
}
