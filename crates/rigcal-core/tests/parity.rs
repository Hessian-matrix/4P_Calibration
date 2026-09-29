//! 与参考实现的对拍回归：期望值已固化为仓库内 `fixtures/golden`，测试本身不依赖外部实现。
//!
//! 黄金值由 `tests/golden/parity.json` 提供，来自真机 12 组观测（`tests/fixtures/records_cam0.jsonl`）。
//! 三类断言：
//!
//! 1. 投影/反投影：纯公式，要求 1e-9 级一致；
//! 2. 可观测性：喂**同一份**解与位姿，要求 σ/秩/条件数/相关性一致（同为确定性线性代数）；
//! 3. 固定内参位姿：外部优化器与本仓有界 LM 必须收敛到同一极小；
//!
//! 内参**求解**不在本 crate（由 `rigcal-opencv` 调 OpenCV 完成），因此这里不含求解对拍；
//! 求解后端的验收在 `crates/rigcal-opencv/tests/kb4_real_data.rs`（真机 36 视图夹具）。

use rigcal_core::estimator::Observation;
use rigcal_core::{ModelKind, Parameters, ds, kb4, models, observability};
use serde_json::Value;

fn golden() -> Value {
    serde_json::from_str(include_str!("golden/parity.json")).expect("golden json must parse")
}

fn fixture_views() -> Vec<Observation> {
    include_str!("fixtures/records_cam0.jsonl")
        .lines()
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            let value: Value = serde_json::from_str(line).expect("record line must parse");
            let object_points = value["object_points"]
                .as_array()
                .expect("object_points")
                .iter()
                .map(|point| {
                    let point = point.as_array().expect("point");
                    [
                        point[0].as_f64().unwrap(),
                        point[1].as_f64().unwrap(),
                        point[2].as_f64().unwrap(),
                    ]
                })
                .collect();
            let image_points = value["image_points"]
                .as_array()
                .expect("image_points")
                .iter()
                .map(|point| {
                    let point = point.as_array().expect("point");
                    [point[0].as_f64().unwrap(), point[1].as_f64().unwrap()]
                })
                .collect();
            Observation {
                object_points,
                image_points,
            }
        })
        .collect()
}

fn numbers(value: &Value) -> Vec<f64> {
    value
        .as_array()
        .expect("array")
        .iter()
        .map(|item| item.as_f64().expect("number"))
        .collect()
}

fn vec3(value: &Value) -> [f64; 3] {
    let values = numbers(value);
    [values[0], values[1], values[2]]
}

fn vec2(value: &Value) -> [f64; 2] {
    let values = numbers(value);
    [values[0], values[1]]
}

#[test]
fn projections_match_python() {
    let golden = golden();
    let projection = &golden["projection"];
    let camera_points: Vec<[f64; 3]> = projection["camera_points"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec3)
        .collect();

    let ds_params = ds::DsParameters {
        fx: projection["ds_params"][0].as_f64().unwrap(),
        fy: projection["ds_params"][1].as_f64().unwrap(),
        cx: projection["ds_params"][2].as_f64().unwrap(),
        cy: projection["ds_params"][3].as_f64().unwrap(),
        xi: projection["ds_params"][4].as_f64().unwrap(),
        alpha: projection["ds_params"][5].as_f64().unwrap(),
    };
    let (pixels, valid) = ds::project(&camera_points, &ds_params);
    let expected: Vec<[f64; 2]> = projection["ds_pixels"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec2)
        .collect();
    let expected_valid: Vec<bool> = projection["ds_valid"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_bool().unwrap())
        .collect();
    assert_eq!(valid, expected_valid);
    for (actual, want) in pixels.iter().zip(expected.iter()) {
        assert!(
            (actual[0] - want[0]).abs() < 1e-9,
            "ds x: {actual:?} vs {want:?}"
        );
        assert!(
            (actual[1] - want[1]).abs() < 1e-9,
            "ds y: {actual:?} vs {want:?}"
        );
    }

    let kb_params = kb4::Kb4Parameters {
        fx: projection["kb4_params"][0].as_f64().unwrap(),
        fy: projection["kb4_params"][1].as_f64().unwrap(),
        cx: projection["kb4_params"][2].as_f64().unwrap(),
        cy: projection["kb4_params"][3].as_f64().unwrap(),
        k1: projection["kb4_params"][4].as_f64().unwrap(),
        k2: projection["kb4_params"][5].as_f64().unwrap(),
        k3: projection["kb4_params"][6].as_f64().unwrap(),
        k4: projection["kb4_params"][7].as_f64().unwrap(),
    };
    let (pixels, valid) = kb4::project(&camera_points, &kb_params);
    let expected: Vec<[f64; 2]> = projection["kb4_pixels"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec2)
        .collect();
    let expected_valid: Vec<bool> = projection["kb4_valid"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_bool().unwrap())
        .collect();
    assert_eq!(valid, expected_valid);
    for (actual, want) in pixels.iter().zip(expected.iter()) {
        assert!(
            (actual[0] - want[0]).abs() < 1e-9,
            "kb4 x: {actual:?} vs {want:?}"
        );
        assert!(
            (actual[1] - want[1]).abs() < 1e-9,
            "kb4 y: {actual:?} vs {want:?}"
        );
    }
}

#[test]
fn unprojections_match_python() {
    let golden = golden();
    let unprojection = &golden["unprojection"];
    let pixels: Vec<[f64; 2]> = unprojection["pixels"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec2)
        .collect();
    let projection = &golden["projection"];

    let ds_params = ds::DsParameters {
        fx: projection["ds_params"][0].as_f64().unwrap(),
        fy: projection["ds_params"][1].as_f64().unwrap(),
        cx: projection["ds_params"][2].as_f64().unwrap(),
        cy: projection["ds_params"][3].as_f64().unwrap(),
        xi: projection["ds_params"][4].as_f64().unwrap(),
        alpha: projection["ds_params"][5].as_f64().unwrap(),
    };
    let (rays, valid) = ds::unproject(&pixels, &ds_params);
    let expected: Vec<[f64; 3]> = unprojection["ds_rays"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec3)
        .collect();
    let expected_valid: Vec<bool> = unprojection["ds_valid"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_bool().unwrap())
        .collect();
    assert_eq!(valid, expected_valid);
    for (actual, want) in rays.iter().zip(expected.iter()) {
        for axis in 0..3 {
            assert!(
                (actual[axis] - want[axis]).abs() < 1e-9,
                "ds ray: {actual:?} vs {want:?}"
            );
        }
    }

    let kb_params = kb4::Kb4Parameters {
        fx: projection["kb4_params"][0].as_f64().unwrap(),
        fy: projection["kb4_params"][1].as_f64().unwrap(),
        cx: projection["kb4_params"][2].as_f64().unwrap(),
        cy: projection["kb4_params"][3].as_f64().unwrap(),
        k1: projection["kb4_params"][4].as_f64().unwrap(),
        k2: projection["kb4_params"][5].as_f64().unwrap(),
        k3: projection["kb4_params"][6].as_f64().unwrap(),
        k4: projection["kb4_params"][7].as_f64().unwrap(),
    };
    let (rays, valid) = kb4::unproject(&pixels, &kb_params);
    let expected: Vec<[f64; 3]> = unprojection["kb4_rays"]
        .as_array()
        .unwrap()
        .iter()
        .map(vec3)
        .collect();
    let expected_valid: Vec<bool> = unprojection["kb4_valid"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_bool().unwrap())
        .collect();
    assert_eq!(valid, expected_valid);
    for (actual, want) in rays.iter().zip(expected.iter()) {
        for axis in 0..3 {
            assert!(
                (actual[axis] - want[axis]).abs() < 1e-9,
                "kb4 ray: {actual:?} vs {want:?}"
            );
        }
    }
}

#[test]
fn fixed_intrinsics_poses_match_python() {
    let golden = golden();
    let projection = &golden["projection"];
    let params = Parameters::Kb4(kb4::Kb4Parameters {
        fx: projection["kb4_params"][0].as_f64().unwrap(),
        fy: projection["kb4_params"][1].as_f64().unwrap(),
        cx: projection["kb4_params"][2].as_f64().unwrap(),
        cy: projection["kb4_params"][3].as_f64().unwrap(),
        k1: projection["kb4_params"][4].as_f64().unwrap(),
        k2: projection["kb4_params"][5].as_f64().unwrap(),
        k3: projection["kb4_params"][6].as_f64().unwrap(),
        k4: projection["kb4_params"][7].as_f64().unwrap(),
    });
    let views = fixture_views();
    let expected = golden["fixed_intrinsics_poses"].as_array().unwrap();
    let mut worst_rms = 0.0_f64;
    for (index, want) in expected.iter().enumerate() {
        let estimate =
            rigcal_core::estimate_fixed_intrinsics_pose(ModelKind::Kb4, &params, &views[index]);
        let want_rms = want["rms_px"].as_f64().unwrap();
        worst_rms = worst_rms.max((estimate.rms_px - want_rms).abs());
        let want_rvec = vec3(&want["rvec"]);
        let want_tvec = vec3(&want["tvec"]);
        for axis in 0..3 {
            assert!(
                (estimate.rvec[axis] - want_rvec[axis]).abs() < 5e-3,
                "view {index} rvec[{axis}]: {} vs {want_rvec:?}",
                estimate.rvec[axis]
            );
            assert!(
                (estimate.tvec[axis] - want_tvec[axis]).abs() < 5e-3,
                "view {index} tvec[{axis}]: {} vs {want_tvec:?}",
                estimate.tvec[axis]
            );
        }
        assert_eq!(
            estimate.status,
            rigcal_core::estimator::Status::Pass,
            "view {index}"
        );
    }
    println!("pose rms worst |Δ| vs python = {worst_rms:.3e} px");
}

#[test]
fn observability_matches_python_given_same_solution() {
    let golden = golden();
    let views = fixture_views();
    for (model_name, solve_key) in [("kb4", "kb4_solve"), ("ds", "ds_solve")] {
        let kind = ModelKind::parse(model_name).unwrap();
        let solution = &golden[solve_key];
        let params = Parameters::from_vector(kind, &numbers(&solution["params"])).unwrap();
        let poses = solution["poses"].as_array().unwrap();
        let view_poses: Vec<(Observation, [f64; 3], [f64; 3])> = views
            .iter()
            .zip(poses.iter())
            .map(|(observation, pose)| (observation.clone(), vec3(&pose[0]), vec3(&pose[1])))
            .collect();
        let report =
            observability::analyze_intrinsics(kind, &params, &view_poses, None).expect("report");
        let want = &golden[if model_name == "kb4" {
            "kb4_observability"
        } else {
            "ds_observability"
        }];
        assert_eq!(report.rank, want["rank"].as_u64().unwrap() as usize);
        assert_eq!(
            report.point_count,
            want["point_count"].as_u64().unwrap() as usize
        );
        let want_rms = want["rms_px"].as_f64().unwrap();
        assert!(
            (report.rms_px - want_rms).abs() < 1e-9,
            "{model_name} rms {} vs {want_rms}",
            report.rms_px
        );
        let want_cond = want["condition_number"].as_f64().unwrap();
        assert!(
            (report.condition_number - want_cond).abs() / want_cond.abs().max(1.0) < 1e-6,
            "{model_name} cond {} vs {want_cond}",
            report.condition_number
        );
        let want_sigma = numbers(&want["focal_relative_stddev"]);
        let (fx_sigma, fy_sigma) = report.focal_relative_stddev();
        assert!(
            (fx_sigma - want_sigma[0]).abs() / want_sigma[0].abs().max(1.0) < 1e-6,
            "{model_name} fx sigma {fx_sigma} vs {}",
            want_sigma[0]
        );
        assert!(
            (fy_sigma - want_sigma[1]).abs() / want_sigma[1].abs().max(1.0) < 1e-6,
            "{model_name} fy sigma {fy_sigma} vs {}",
            want_sigma[1]
        );
        let want_principal = numbers(&want["principal_stddev_px"]);
        let (cx_sigma, cy_sigma) = report.principal_stddev_px();
        // 有限差分与协方差求逆会放大跨平台浮点差异，与相邻派生量使用同一容差。
        assert!(
            (cx_sigma - want_principal[0]).abs() / want_principal[0].abs().max(1.0) < 1e-6,
            "{model_name} cx sigma {cx_sigma} vs {}",
            want_principal[0]
        );
        assert!(
            (cy_sigma - want_principal[1]).abs() / want_principal[1].abs().max(1.0) < 1e-6,
            "{model_name} cy sigma {cy_sigma} vs {}",
            want_principal[1]
        );
        let want_logdet = want["log_det_information"].as_f64().unwrap();
        assert!(
            (report.log_det_information - want_logdet).abs() < 1e-6,
            "{model_name} logdet {} vs {want_logdet}",
            report.log_det_information
        );
    }
}

#[test]
fn models_dispatch_matches_direct_calls() {
    let points = [[0.1, -0.2, 0.9], [0.0, 0.0, 0.0], [-0.3, 0.25, 0.6]];
    let params = ds::DsParameters {
        fx: 500.0,
        fy: 505.0,
        cx: 640.0,
        cy: 544.0,
        xi: 0.2,
        alpha: 0.6,
    };
    let (direct, direct_valid) = ds::project(&points, &params);
    let (dispatch, dispatch_valid) =
        models::project(ModelKind::Ds, &points, &Parameters::Ds(params));
    assert_eq!(direct_valid, dispatch_valid);
    for (actual, want) in dispatch.iter().zip(direct.iter()) {
        for axis in 0..2 {
            assert!(
                actual[axis] == want[axis] || (actual[axis].is_nan() && want[axis].is_nan()),
                "dispatch mismatch: {actual:?} vs {want:?}"
            );
        }
    }
}
