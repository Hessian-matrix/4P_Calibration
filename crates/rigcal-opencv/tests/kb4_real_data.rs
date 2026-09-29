//! OpenCV KB4 后端在**真机 36 视图夹具**上的验收。
//!
//! 夹具来自真机采集（`robobaton_4p` cam0 的 36 个帧组），参考解由 `cv2.fisheye.calibrate`
//! 给出：rms **0.2273 px**、fx 655.880。
//! 验收线：**rms ≤ 0.3 px** 且 fx 与参考解相对差 < 1%。

use rigcal_core::estimator::{Observation, SolveRequest, Status};
use rigcal_core::{ModelKind, Parameters};
use serde_json::Value;

const FIXTURE: &str = include_str!("fixtures/kb4_real_36views.json");
const REFERENCE_RMS_PX: f64 = 0.2273;
const REFERENCE_FX: f64 = 655.880;

fn fixture() -> (Vec<Observation>, (u32, u32), Parameters) {
    let value: Value = serde_json::from_str(FIXTURE).expect("fixture must parse");
    let image_size = (
        value["image_size"][0].as_u64().unwrap() as u32,
        value["image_size"][1].as_u64().unwrap() as u32,
    );
    let initial_values: Vec<f64> = value["initial"]
        .as_array()
        .unwrap()
        .iter()
        .map(|item| item.as_f64().unwrap())
        .collect();
    let initial = Parameters::from_vector(ModelKind::Kb4, &initial_values).expect("initial params");
    let observations = value["observations"]
        .as_array()
        .unwrap()
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
    (observations, image_size, initial)
}

#[test]
fn opencv_kb4_meets_the_acceptance_bar_on_real_data() {
    let (observations, image_size, initial) = fixture();
    assert_eq!(
        observations.len(),
        36,
        "fixture must carry the real 36 views"
    );

    let started = std::time::Instant::now();
    let outcome = rigcal_opencv::solve_kb4(SolveRequest {
        kind: ModelKind::Kb4,
        observations: &observations,
        image_size,
        initial: Some(initial),
        initial_poses: None,
    })
    .expect("OpenCV KB4 solve must succeed");
    let elapsed = started.elapsed();

    let parameters = outcome.parameters.as_vector();
    println!(
        "opencv kb4: rms={:.4}px (参考 {REFERENCE_RMS_PX:.4}) fx={:.3} (参考 {REFERENCE_FX:.3}) \
         views={} invalid={} elapsed={:.2}s",
        outcome.rms_px,
        parameters[0],
        observations.len(),
        outcome.invalid_projection_count,
        elapsed.as_secs_f64()
    );

    assert_eq!(
        outcome.status,
        Status::Pass,
        "solve must pass with no invalid projections"
    );
    assert!(
        outcome.rms_px <= 0.3,
        "rms {:.4}px must meet the ≤0.3px bar",
        outcome.rms_px
    );
    let fx = parameters[0];
    assert!(
        (fx - REFERENCE_FX).abs() / REFERENCE_FX < 0.01,
        "fx {fx:.3} must be within 1% of the reference {REFERENCE_FX:.3}"
    );
    assert_eq!(outcome.poses.len(), observations.len(), "one pose per view");
}
