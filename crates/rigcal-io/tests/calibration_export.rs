//! 导出产物的边界测试：方向/链一致性、结果与信息的分离、DRAFT 判定、以及 IO 失败不得毁掉既有导出。
//!
//! 全部在**消费端**核对：读回写盘的 YAML，按文档约定比对矩阵，而不是断言源码或内部字段。

use std::collections::BTreeMap;
use std::path::Path;

use rigcal_core::config::Config;
use rigcal_core::extrinsics::{
    CycleCheck, EdgeEstimate, Matrix4, RigEstimate, compose, identity4, invert, multiply,
};
use rigcal_core::models::ModelKind;
use rigcal_core::rotation::rvec_to_matrix;
use rigcal_core::session::SessionState;
use rigcal_io::{
    CAMCHAIN_FILE, EXTRINSICS_FILE, ExportError, INFO_FILE, export_calibration, intrinsics_file,
};
use serde_yaml::Value;

const EXPECTED_TOLERANCE: f64 = 1e-9;

fn config_from(cameras: &str, edges: &str, cycles: &str) -> Config {
    let yaml = format!(
        r#"schema_version: 2
rig_id: test_rig
image_size: [1280, 1088]
cameras:
{cameras}
evidence: {{host: 127.0.0.1, port: 4211}}
extrinsics:
  required_edges: {edges}
  required_cycles: {cycles}
  min_groups_per_edge: 3
  max_edge_rms_px: 1.0
  max_cycle_rotation_deg: 1.0
  max_cycle_translation_mm: 10.0
board:
  target_type: aprilgrid
  target_id: test_board
  measured: true
  rows: 6
  cols: 6
  first_tag_id: 0
  tag_corner_order: [bottom_right, bottom_left, top_left, top_right]
  tag_size_m: 0.088
  tag_spacing_ratio: 0.3
  dictionary: DICT_APRILTAG_36h11
guidance:
  detect_scale: 0.75
  detect_hz: 0.5
  detect_hz_max: 12.0
  jitter_xyz: 0.025
  jitter_z: 0.04
  jitter_rotation_deg: 2.0
  trigger_novelty_scale: 3.0
  trigger_min_interval_s: 0.5
solver:
  models: [kb4]
  min_observations: 12
  max_solve_observations: 30
  holdout_fraction: 0.25
  official_holdout_frames: 6
  max_holdout_rms_px: 1.0
  max_holdout_p95_px: 1.0
  quality:
    min_focus_score: 500.0
    min_contrast: 20.0
    max_saturated_fraction: 0.35
output:
  root: calibration_runs/test
"#
    );
    Config::from_yaml(&yaml).expect("config")
}

fn camera_lines(count: usize) -> String {
    (0..count)
        .map(|index| {
            format!(
                "  - {{id: cam{index}, guidance: {{type: video, path: /tmp/cam{index}.mp4}}}}"
            )
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn four_camera_config() -> Config {
    config_from(
        &camera_lines(4),
        "[[cam0, cam1], [cam1, cam2], [cam2, cam3], [cam3, cam0]]",
        "[[cam0, cam1, cam2, cam3]]",
    )
}

fn three_camera_config() -> Config {
    config_from(
        &camera_lines(3),
        "[[cam0, cam1], [cam1, cam2]]",
        "[[cam0, cam1, cam2]]",
    )
}

fn session_state(model: ModelKind, converged: bool) -> SessionState {
    SessionState {
        model,
        views: 40,
        used_views: 30,
        excluded_views: 2,
        rms_px: 0.15,
        rank: model.parameter_count(),
        condition_number: 1.2e3,
        focal_relative_stddev: (1.0e-3, 1.1e-3),
        principal_stddev_px: (0.1, 0.12),
        info_gain: Some(0.03),
        holdout_views: 8,
        holdout_rms_px: 0.3,
        holdout_p95_px: 0.5,
        holdout_invalid: 0,
        solves: 5,
        failures: 0,
        converged,
        streak: if converged { 3 } else { 0 },
        status: if converged {
            "ok"
        } else {
            "insufficient_views"
        },
        detail: String::new(),
        parameters: model.default_parameters((1280, 1088)).as_vector(),
    }
}

fn states(model: ModelKind) -> BTreeMap<String, SessionState> {
    let mut map = BTreeMap::new();
    for index in 0..4 {
        map.insert(format!("cam{index}"), session_state(model, true));
    }
    map
}

fn transform(rvec: [f64; 3], translation: [f64; 3]) -> Matrix4 {
    compose(&rvec_to_matrix(&rvec), &translation)
}

fn edge(a: &str, b: &str, transform: Matrix4, groups: usize, rms: f64) -> EdgeEstimate {
    EdgeEstimate {
        camera_a: a.to_owned(),
        camera_b: b.to_owned(),
        transform,
        groups,
        rotation_sigma_deg: 0.05,
        translation_sigma_mm: 0.2,
        reprojection_rms_px: rms,
    }
}

/// `T_c1_c0, T_c2_c1, T_c3_c2`；第四条边 `T_c0_c3` 与链严格一致，保证环闭合。
fn base_transforms() -> (Matrix4, Matrix4, Matrix4, Matrix4) {
    let m1 = transform([0.20, -0.10, 0.05], [0.30, -0.04, 0.02]);
    let m2 = transform([-0.08, 0.16, -0.04], [-0.12, 0.26, 0.03]);
    let m3 = transform([0.06, 0.12, 0.14], [0.02, 0.05, 0.31]);
    let m4 = multiply(&multiply(&invert(&m1), &invert(&m2)), &invert(&m3));
    (m1, m2, m3, m4)
}

fn estimate(
    edge_rms: f64,
    cycle_rotation_deg: f64,
    cycle_translation_mm: f64,
    ring: bool,
) -> RigEstimate {
    let (m1, m2, m3, m4) = base_transforms();
    let mut edges = vec![
        edge("cam0", "cam1", m1, 5, edge_rms),
        edge("cam1", "cam2", m2, 5, edge_rms),
        edge("cam2", "cam3", m3, 5, edge_rms),
    ];
    if ring {
        edges.push(edge("cam3", "cam0", m4, 5, edge_rms));
    }
    RigEstimate {
        edges,
        cycles: vec![CycleCheck {
            cameras: ["cam0", "cam1", "cam2", "cam3"]
                .iter()
                .map(|id| (*id).to_owned())
                .collect(),
            rotation_error_deg: cycle_rotation_deg,
            translation_error_mm: cycle_translation_mm,
        }],
        rig_rms_px: edge_rms,
    }
}

fn temp_root(tag: &str) -> std::path::PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    let root = std::env::temp_dir().join(format!(
        "rigcal-io-export-{tag}-{}-{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&root).expect("temp root");
    root
}

fn read_yaml(path: &Path) -> Value {
    serde_yaml::from_str(&std::fs::read_to_string(path).expect("read yaml")).expect("parse yaml")
}

fn keys_of(value: &Value) -> Vec<String> {
    let mut keys: Vec<String> = value
        .as_mapping()
        .expect("mapping")
        .keys()
        .map(|key| key.as_str().expect("key").to_owned())
        .collect();
    keys.sort();
    keys
}

fn matrix_of(value: &Value) -> Matrix4 {
    let mut matrix = [[0.0_f64; 4]; 4];
    for (row, target_row) in matrix.iter_mut().enumerate() {
        for (column, target) in target_row.iter_mut().enumerate() {
            *target = value[row][column].as_f64().expect("matrix element");
        }
    }
    matrix
}

fn close_matrix(actual: &Matrix4, expected: &Matrix4, label: &str) {
    for row in 0..4 {
        for column in 0..4 {
            assert!(
                (actual[row][column] - expected[row][column]).abs() <= EXPECTED_TOLERANCE,
                "{label}[{row}][{column}]: {:.3e} vs {:.3e}",
                actual[row][column],
                expected[row][column]
            );
        }
    }
}

fn expect_error(
    tag: &str,
    config: &Config,
    states: &BTreeMap<String, SessionState>,
    estimate: &RigEstimate,
    check: impl Fn(&ExportError) -> bool,
) {
    let root = temp_root(tag);
    let error = export_calibration(&root, config, states, estimate, 4, None).expect_err("must fail");
    assert!(check(&error), "unexpected error: {error:?}");
    assert!(
        !root.join("exports").exists(),
        "a rejected export must not publish anything"
    );
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn export_binds_rotations_and_translations_in_a_consistent_chain() {
    let root = temp_root("chain");
    let config = four_camera_config();
    let model = ModelKind::Kb4;
    let estimate = estimate(0.2, 0.0, 0.0, true);

    let receipt =
        export_calibration(&root, &config, &states(model), &estimate, 18, None).expect("export");
    assert!(receipt.validated);
    assert_eq!(
        receipt.directory,
        root.join("exports")
            .join(receipt.directory.file_name().expect("run id"))
    );

    // 结果：每路一个内参文件，只放模型与参数，不放误差/统计/判定。
    let cam1 = read_yaml(&receipt.directory.join(intrinsics_file("cam1")));
    assert_eq!(
        keys_of(&cam1),
        vec![
            "camera_id",
            "kind",
            "model",
            "parameter_names",
            "parameters",
            "resolution",
            "schema_version",
        ],
        "camN.yaml 只允许内参数值"
    );
    assert_eq!(cam1["kind"].as_str(), Some("camera-intrinsics"));
    assert_eq!(cam1["camera_id"].as_str(), Some("cam1"));
    assert_eq!(cam1["model"].as_str(), Some("kb4"));
    assert_eq!(cam1["schema_version"].as_u64(), Some(2));
    let resolution: Vec<u64> = cam1["resolution"]
        .as_sequence()
        .expect("resolution")
        .iter()
        .map(|value| value.as_u64().expect("dimension"))
        .collect();
    assert_eq!(resolution, vec![1280, 1088]);
    let names: Vec<&str> = cam1["parameter_names"]
        .as_sequence()
        .expect("parameter_names")
        .iter()
        .map(|value| value.as_str().expect("name"))
        .collect();
    assert_eq!(names, vec!["fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"]);
    assert_eq!(cam1["parameters"].as_sequence().expect("parameters").len(), 8);

    // 结果：四路外参在同一个文件里，只有 T_c0_ci。
    let extrinsics = read_yaml(&receipt.directory.join(EXTRINSICS_FILE));
    assert_eq!(
        keys_of(&extrinsics),
        vec![
            "cameras",
            "convention",
            "kind",
            "reference_camera",
            "schema_version",
            "translation_unit",
        ]
    );
    assert_eq!(extrinsics["kind"].as_str(), Some("rig-extrinsics"));
    assert_eq!(extrinsics["reference_camera"].as_str(), Some("cam0"));
    assert_eq!(extrinsics["translation_unit"].as_str(), Some("m"));
    assert_eq!(
        extrinsics["convention"].as_str(),
        Some("P_c0 = T_c0_ci * P_ci")
    );
    let ids: Vec<String> = keys_of(&extrinsics["cameras"]);
    assert_eq!(ids, vec!["cam0", "cam1", "cam2", "cam3"]);
    for id in &ids {
        assert_eq!(
            keys_of(&extrinsics["cameras"][id]),
            vec!["T_c0_ci"],
            "{id} 的外参条目只允许 T_c0_ci"
        );
    }

    // cam0 是参考：T_c0_c0 必须是**逐位精确**的单位阵（不是"数值上近似"）。
    let cam0 = matrix_of(&extrinsics["cameras"]["cam0"]["T_c0_ci"]);
    let identity = identity4();
    for row in 0..4 {
        for column in 0..4 {
            assert_eq!(
                cam0[row][column].to_bits(),
                identity[row][column].to_bits(),
                "cam0 T_c0_ci[{row}][{column}] must be exactly the identity"
            );
        }
    }

    // 混合旋转+平移下，逐路 T_c0_ci 必须等于按文档约定组合出的结果。
    let (m1, m2, m3, _) = base_transforms();
    let expected_c1 = invert(&m1);
    let expected_c2 = multiply(&invert(&m1), &invert(&m2));
    let expected_c3 = multiply(&expected_c2, &invert(&m3));
    let t_c0_c1 = matrix_of(&extrinsics["cameras"]["cam1"]["T_c0_ci"]);
    let t_c0_c2 = matrix_of(&extrinsics["cameras"]["cam2"]["T_c0_ci"]);
    let t_c0_c3 = matrix_of(&extrinsics["cameras"]["cam3"]["T_c0_ci"]);
    close_matrix(&t_c0_c1, &expected_c1, "cam1 T_c0_ci");
    close_matrix(&t_c0_c2, &expected_c2, "cam2 T_c0_ci");
    close_matrix(&t_c0_c3, &expected_c3, "cam3 T_c0_ci");
    // 消费端链式一致性：T_c0_c2 = T_c0_c1 · T_c1_c2，T_c0_c3 = T_c0_c2 · T_c2_c3。
    close_matrix(&multiply(&t_c0_c1, &invert(&m2)), &t_c0_c2, "chain 0<-1<-2");
    close_matrix(&multiply(&t_c0_c2, &invert(&m3)), &t_c0_c3, "chain 0<-2<-3");

    // 信息：判定、阈值、指标都在 info.yaml，且判定与逐条检查自洽。
    let info = read_yaml(&receipt.directory.join(INFO_FILE));
    assert_eq!(info["kind"].as_str(), Some("rig-calibration-info"));
    assert_eq!(info["judgement"]["status"].as_str(), Some("VALIDATED"));
    assert!(
        info["judgement"]["warnings"]
            .as_sequence()
            .expect("warnings")
            .is_empty()
    );
    let checks = info["judgement"]["checks"].as_sequence().expect("checks");
    assert!(
        !checks.is_empty() && checks.iter().all(|check| check["passed"] == Value::Bool(true)),
        "VALIDATED 的每条检查都必须通过: {checks:?}"
    );
    assert_eq!(info["rig"]["rig_id"].as_str(), Some("test_rig"));
    assert_eq!(info["rig"]["cameras"].as_sequence().expect("cameras").len(), 4);
    assert_eq!(info["metrics"]["cameras"]["cam0"]["views"].as_u64(), Some(40));
    assert_eq!(info["metrics"]["edges"].as_sequence().expect("edges").len(), 4);
    assert_eq!(info["metrics"]["cycles"].as_sequence().expect("cycles").len(), 1);
    assert_eq!(
        info["thresholds"]["extrinsics"]["min_groups_per_edge"].as_u64(),
        Some(3)
    );
    // 有效阈值必须是数值而不是"见配置"：会话判据的代码默认值也要落在这里。
    assert_eq!(info["thresholds"]["intrinsics"]["max_rms_px"].as_f64(), Some(0.2));
    assert_eq!(
        info["thresholds"]["intrinsics"]["max_holdout_rms_px"].as_f64(),
        Some(1.0)
    );
    assert!(info["session"].is_null(), "没有会话引用就记 null");
    assert!(!info["exported_at"]["local"].as_str().expect("local").is_empty());
    assert_eq!(info["software"]["name"].as_str(), Some("rigcal"));
    assert!(
        info["result_files"]
            .as_sequence()
            .expect("result_files")
            .iter()
            .any(|name| name.as_str() == Some("extrinsics.yaml"))
    );

    // 目录名是本地日期时间，不是纳秒计数。
    let name = receipt
        .directory
        .file_name()
        .and_then(|name| name.to_str())
        .expect("run id");
    assert!(
        name.len() == 4 + 8 + 1 + 6 && name.starts_with("run-"),
        "run-<YYYYMMDD-HHMMSS>: {name}"
    );
    let digits: Vec<char> = name[4..].chars().filter(|c| c.is_ascii_digit()).collect();
    assert_eq!(digits.len(), 14, "run id 必须全是数字时间戳: {name}");

    // camchain：T_cn_cnm1 就是输入边的 T_b_a，cam0 不携带链变换。
    let camchain_text =
        std::fs::read_to_string(receipt.directory.join(CAMCHAIN_FILE)).expect("camchain");
    assert!(camchain_text.contains("VALIDATED"), "{camchain_text}");
    assert!(camchain_text.contains(INFO_FILE), "{camchain_text}");
    let camchain: Value = serde_yaml::from_str(&camchain_text).expect("camchain yaml");
    assert!(camchain["cam0"].get("T_cn_cnm1").is_none());
    close_matrix(
        &matrix_of(&camchain["cam1"]["T_cn_cnm1"]),
        &m1,
        "camchain cam1",
    );
    close_matrix(
        &matrix_of(&camchain["cam2"]["T_cn_cnm1"]),
        &m2,
        "camchain cam2",
    );
    close_matrix(
        &matrix_of(&camchain["cam3"]["T_cn_cnm1"]),
        &m3,
        "camchain cam3",
    );

    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn session_reference_is_recorded_when_the_journal_is_known() {
    let root = temp_root("session");
    let config = four_camera_config();
    let model = ModelKind::Kb4;
    let estimate = estimate(0.2, 0.0, 0.0, true);
    let session = root.join("sessions/s-1");
    std::fs::create_dir_all(&session).expect("session dir");
    let journal = session.join("observations.jsonl");
    std::fs::write(&journal, b"").expect("journal");

    let receipt = export_calibration(
        &root,
        &config,
        &states(model),
        &estimate,
        12,
        Some(&journal),
    )
    .expect("export");
    let info = read_yaml(&receipt.directory.join(INFO_FILE));
    assert_eq!(
        info["session"]["journal"].as_str(),
        Some("sessions/s-1/observations.jsonl")
    );
    assert_eq!(
        info["session"]["config"].as_str(),
        Some("sessions/s-1/config.yaml")
    );
    assert_eq!(info["session"]["groups"].as_u64(), Some(12));

    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn missed_gates_are_labeled_draft() {
    let root = temp_root("draft");
    let config = four_camera_config();
    let model = ModelKind::Kb4;
    let mut state_map = states(model);
    let cam2 = state_map.get_mut("cam2").expect("cam2");
    cam2.converged = false;
    cam2.status = "insufficient_views";
    let estimate = estimate(2.5, 2.0, 25.0, true);

    let receipt =
        export_calibration(&root, &config, &state_map, &estimate, 9, None).expect("export");
    assert!(!receipt.validated);

    let info = read_yaml(&receipt.directory.join(INFO_FILE));
    assert_eq!(info["judgement"]["status"].as_str(), Some("DRAFT"));
    let warnings: Vec<String> = info["judgement"]["warnings"]
        .as_sequence()
        .expect("warnings")
        .iter()
        .map(|value| value.as_str().expect("warning").to_owned())
        .collect();
    assert!(
        warnings.iter().any(|warning| warning.contains("cam2")),
        "non-converged camera must be reported: {warnings:?}"
    );

    // 每条未通过的检查都必须有对应的人读警告，且 DRAFT 与 checks 自洽。
    let mut failed = 0;
    for check in info["judgement"]["checks"].as_sequence().expect("checks") {
        if check["passed"] == Value::Bool(true) {
            continue;
        }
        failed += 1;
        let scope = check["scope"].as_str().expect("scope");
        let metric = check["metric"].as_str().expect("metric");
        assert!(
            warnings
                .iter()
                .any(|warning| warning.contains(scope) && warning.contains(metric)),
            "failed check {scope}/{metric} has no warning: {warnings:?}"
        );
    }
    assert!(failed > 0, "DRAFT must have at least one failed check");
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("cam0-cam1-cam3") || warning.contains("cam0-cam1")),
        "failed edge gate must be reported: {warnings:?}"
    );
    assert!(
        warnings
            .iter()
            .any(|warning| warning.contains("cam0-cam1-cam2-cam3")),
        "failed cycle gate must be reported: {warnings:?}"
    );

    let camchain_text =
        std::fs::read_to_string(receipt.directory.join(CAMCHAIN_FILE)).expect("camchain");
    assert!(camchain_text.contains("DRAFT"), "{camchain_text}");
    assert!(
        camchain_text.contains("cam2"),
        "camchain header must carry the reasons: {camchain_text}"
    );

    // 结果文件不带判定：DRAFT 也一样。
    let cam2_file = read_yaml(&receipt.directory.join(intrinsics_file("cam2")));
    assert!(
        !keys_of(&cam2_file).iter().any(|key| key == "status"),
        "结果文件不得出现判定: {:?}",
        keys_of(&cam2_file)
    );

    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn incomplete_or_inconsistent_inputs_are_rejected() {
    let config = four_camera_config();
    let model = ModelKind::Kb4;
    let full = states(model);
    let complete = estimate(0.2, 0.0, 0.0, true);

    let mut missing = full.clone();
    missing.remove("cam3");
    expect_error(
        "missing-state",
        &config,
        &missing,
        &complete,
        |error| matches!(error, ExportError::MissingState(camera) if camera == "cam3"),
    );

    let mut unsolved = full.clone();
    unsolved.get_mut("cam1").expect("cam1").solves = 0;
    expect_error(
        "unsolved",
        &config,
        &unsolved,
        &complete,
        |error| matches!(error, ExportError::Unsolved { camera } if camera == "cam1"),
    );

    let mut mixed = full.clone();
    mixed.get_mut("cam2").expect("cam2").model = ModelKind::Ds;
    expect_error("model", &config, &mixed, &complete, |error| {
        matches!(error, ExportError::ModelInconsistent)
    });

    let mut invalid = full.clone();
    invalid.get_mut("cam0").expect("cam0").parameters[0] = -1.0;
    expect_error(
        "negative-focal",
        &config,
        &invalid,
        &complete,
        |error| matches!(error, ExportError::InvalidParameters { camera, .. } if camera == "cam0"),
    );

    // 这条边并非 cam0 BFS 到 cam2 的选中路径；坏边也不能被路径选择悄悄隐藏。
    let mut bad_edge = complete.clone();
    bad_edge.edges[2].transform[0][0] = f64::NAN;
    expect_error("invalid-unused-edge", &config, &full, &bad_edge, |error| {
        matches!(error, ExportError::InvalidTransform { .. })
    });

    // 必需边缺失：去掉闭环边，参考图仍连通 —— 仍必须拒绝。
    let ringless = estimate(0.2, 0.0, 0.0, false);
    expect_error("edge", &config, &full, &ringless, |error| {
        matches!(
            error,
            ExportError::MissingEdge { camera_a, camera_b }
                if camera_a == "cam3" && camera_b == "cam0"
        )
    });

    let no_cycle = RigEstimate {
        edges: complete.edges.clone(),
        cycles: Vec::new(),
        rig_rms_px: complete.rig_rms_px,
    };
    expect_error(
        "cycle",
        &config,
        &full,
        &no_cycle,
        |error| matches!(error, ExportError::MissingCycle { cameras } if cameras.len() == 4),
    );

    // 配置里的相机集合与快照不符。
    let three = three_camera_config();
    expect_error("camera-set", &three, &full, &complete, |error| {
        matches!(error, ExportError::CameraSet { .. })
    });
}

#[test]
fn a_single_camera_config_has_no_rig_export() {
    // 导出是 rig 产物：1 路配置没有外参段，必须明确拒绝而不是写半份结果。
    let root = temp_root("single");
    let mut single = four_camera_config();
    single
        .cameras
        .truncate(1);
    single.extrinsics = None;
    let mut one = BTreeMap::new();
    one.insert("cam0".to_owned(), session_state(ModelKind::Kb4, true));
    let error = export_calibration(
        &root,
        &single,
        &one,
        &estimate(0.2, 0.0, 0.0, true),
        4,
        None,
    )
    .expect_err("single camera rig export must fail");
    assert!(
        matches!(error, ExportError::MissingExtrinsics),
        "unexpected error: {error:?}"
    );
    assert!(!root.join("exports").exists());
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[cfg(unix)]
#[test]
fn io_failure_leaves_the_previous_export_untouched() {
    use std::os::unix::fs::PermissionsExt;

    let root = temp_root("io");
    let config = four_camera_config();
    let model = ModelKind::Kb4;
    let estimate = estimate(0.2, 0.0, 0.0, true);
    let state_map = states(model);

    let first =
        export_calibration(&root, &config, &state_map, &estimate, 5, None).expect("first export");
    let info_before = std::fs::read(first.directory.join(INFO_FILE)).expect("info");
    let intrinsics_before =
        std::fs::read(first.directory.join(intrinsics_file("cam0"))).expect("cam0");
    let camchain_before = std::fs::read(first.directory.join(CAMCHAIN_FILE)).expect("camchain");

    let exports = root.join("exports");
    let original = std::fs::metadata(&exports).expect("metadata").permissions();
    let mut locked = original.clone();
    locked.set_mode(0o555);
    std::fs::set_permissions(&exports, locked).expect("lock exports");

    let second = export_calibration(&root, &config, &state_map, &estimate, 6, None);
    std::fs::set_permissions(&exports, original).expect("unlock exports");
    assert!(matches!(second, Err(ExportError::Io(_))), "{second:?}");

    assert!(first.directory.join(INFO_FILE).exists());
    assert_eq!(
        std::fs::read(first.directory.join(INFO_FILE)).expect("info"),
        info_before
    );
    assert_eq!(
        std::fs::read(first.directory.join(intrinsics_file("cam0"))).expect("cam0"),
        intrinsics_before
    );
    assert_eq!(
        std::fs::read(first.directory.join(CAMCHAIN_FILE)).expect("camchain"),
        camchain_before
    );
    let entries: Vec<std::ffi::OsString> = std::fs::read_dir(&exports)
        .expect("read exports")
        .map(|entry| entry.expect("entry").file_name())
        .collect();
    assert_eq!(
        entries.len(),
        1,
        "no partial bundle may be published: {entries:?}"
    );

    std::fs::remove_dir_all(&root).expect("cleanup");
}
