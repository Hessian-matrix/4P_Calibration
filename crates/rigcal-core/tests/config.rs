//! 单一配置文件的解析与校验（对应 `src/config.rs`）。
//!
//! 关注点：**未知键必须报错**、板参数必须内联且已实测、越界值一律拒绝——
//! 静默忽略会让人以为「配了门禁」，实际没生效。

use rigcal_core::config::Config;
use rigcal_core::models::ModelKind;

const FIXTURE: &str = include_str!("fixtures/session_single.yaml");

#[test]
fn fixture_parses_and_resolves_board() {
    let config = Config::from_yaml(FIXTURE).expect("fixture must parse");
    assert_eq!(config.device.camera_id, "cam0");
    assert_eq!(config.image_size(), (1280, 1088));
    assert_eq!(config.solver.models, vec![ModelKind::Kb4, ModelKind::Ds]);
    let board = config.board_config().expect("board");
    assert_eq!((board.rows, board.cols), (6, 6));
    assert!((board.tag_size_m - 0.088).abs() < 1e-12);
    assert!((board.tag_pitch_m() - 0.088 * 1.3).abs() < 1e-12);
    // 角点顺序必须按配置生效：第一个角点是 bottom_right → (tag_size, 0, 0)
    let corners = board.object_corners_for_tag(0).expect("tag 0");
    assert_eq!(corners[0], [0.088, 0.0, 0.0]);
    assert_eq!(corners[2], [0.0, 0.088, 0.0]);
    // 物点按 tag 顺序拼接：36 个 tag → 144 个角点
    let points = board
        .object_points(&(0..36).collect::<Vec<_>>())
        .expect("points");
    assert_eq!(points.len(), 144);
}

#[test]
fn unknown_keys_are_rejected() {
    let with_unknown = FIXTURE.replace("output:", "ui:\n  preview_scale: 0.5\noutput:");
    let error = Config::from_yaml(&with_unknown).expect_err("unknown section must fail");
    assert!(
        error.0.contains("ui"),
        "error must name the offender: {}",
        error.0
    );

    let nested = FIXTURE.replace(
        "    min_contrast: 20.0",
        "    min_contrast: 20.0\n    skew_gate: enforce",
    );
    let error = Config::from_yaml(&nested).expect_err("unknown nested key must fail");
    assert!(
        error.0.contains("skew_gate"),
        "error must name the offender: {}",
        error.0
    );
}

#[test]
fn board_target_file_is_rejected() {
    // 单一配置文件：老写法（引用独立 target.yaml）必须报错，而不是静默丢掉板参数
    let legacy = FIXTURE.replace(
        "  target_type: aprilgrid",
        "  target: local/target.yaml\n  target_type: aprilgrid",
    );
    assert!(Config::from_yaml(&legacy).is_err());
}

#[test]
fn unmeasured_board_is_rejected() {
    let unmeasured = FIXTURE.replace("  measured: true", "  measured: false");
    let error = Config::from_yaml(&unmeasured).expect_err("unmeasured board must fail");
    assert!(error.0.contains("measured"), "{}", error.0);
}

#[test]
fn out_of_range_values_are_rejected() {
    for (patched, needle) in [
        (
            FIXTURE.replace("schema_version: 1", "schema_version: 2"),
            "schema_version",
        ),
        (
            FIXTURE.replace("  models: [kb4, ds]", "  models: [kb4, kb4]"),
            "repeats",
        ),
        (
            FIXTURE.replace("  models: [kb4, ds]", "  models: []"),
            "at least one model",
        ),
        (
            FIXTURE.replace("  holdout_fraction: 0.25", "  holdout_fraction: 1.0"),
            "holdout_fraction",
        ),
        (
            FIXTURE.replace(
                "    max_saturated_fraction: 0.35",
                "    max_saturated_fraction: 1.5",
            ),
            "saturated",
        ),
        (
            FIXTURE.replace("    - [-0.2, 0.35]", "    - [-1.5, 0.35]"),
            "xi",
        ),
        (
            FIXTURE.replace("    - [0.0, 0.5]", "    - [0.0, 1.0]"),
            "alpha",
        ),
        (
            FIXTURE.replace("  min_observations: 12", "  min_observations: 0"),
            "min_observations",
        ),
        (
            FIXTURE.replace(
                "  max_solve_observations: 30",
                "  max_solve_observations: 5",
            ),
            "max_solve_observations",
        ),
        (
            FIXTURE.replace("output:\n  root: calibration_runs", "output:\n  root: \"\""),
            "root",
        ),
        (
            FIXTURE.replace("    min_focus_score: 500.0", "    min_focus_score: -1.0"),
            "min_focus_score",
        ),
        (
            FIXTURE.replace("  detect_scale: 0.75", "  detect_scale: 1.5"),
            "detect_scale",
        ),
        (
            FIXTURE.replace(
                "  trigger_novelty_scale: 3.0",
                "  trigger_novelty_scale: 0.5",
            ),
            "trigger_novelty_scale",
        ),
        (
            FIXTURE.replace("  jitter_rotation_deg: 2.0", "  jitter_rotation_deg: 0.0"),
            "jitter_rotation_deg",
        ),
    ] {
        let error = Config::from_yaml(&patched).expect_err(&format!("must reject: {needle}"));
        assert!(
            error.0.contains(needle),
            "error must mention {needle}: {}",
            error.0
        );
    }
}

#[test]
fn legacy_flow_and_laplacian_fields_are_rejected() {
    // 已移除的光流门禁字段与清晰度字段旧名必须报未知键，而不是静默忽略。
    for legacy in ["flow_enabled", "flow_stable_px", "flow_window"] {
        let patched = FIXTURE.replace(
            "  detect_hz_max: 12.0\n",
            &format!("  detect_hz_max: 12.0\n  {legacy}: 1\n"),
        );
        let error = Config::from_yaml(&patched).expect_err("legacy flow field must fail");
        assert!(error.0.contains(legacy), "{}", error.0);
    }
    let legacy_quality =
        FIXTURE.replace("    min_focus_score: 500.0", "    min_laplacian_var: 15.0");
    let error = Config::from_yaml(&legacy_quality).expect_err("legacy Laplacian field must fail");
    assert!(error.0.contains("min_laplacian_var"), "{}", error.0);
}

#[test]
fn guidance_and_evidence_sources_are_validated() {
    let config = Config::from_yaml(FIXTURE).expect("fixture must parse");
    let capture = config
        .capture
        .as_ref()
        .expect("single-camera capture section");
    match &capture.guidance {
        rigcal_core::config::GuidanceSource::Rtsp { url } => {
            assert_eq!(url, "rtsp://10.21.12.162:554/PRR");
        }
        other => panic!("expected rtsp guidance source, got {other:?}"),
    }
    let evidence = capture.evidence.as_ref().expect("evidence source");
    assert_eq!(
        (evidence.host.as_str(), evidence.port, evidence.camera),
        ("10.21.12.162", 4200, 0)
    );

    // 证据源端口 0 / 引导源空 URL 一律拒绝
    let zero_port = FIXTURE.replace("    port: 4200", "    port: 0");
    let error = Config::from_yaml(&zero_port).expect_err("port 0 must fail");
    assert!(error.0.contains("port"), "{}", error.0);
    let empty_url = FIXTURE.replace("    url: rtsp://10.21.12.162:554/PRR", "    url: \"\"");
    let error = Config::from_yaml(&empty_url).expect_err("empty url must fail");
    assert!(error.0.contains("url"), "{}", error.0);

    // 离线回放：证据源可以缺省；本地视频走与 RTSP 同一条解码路径
    let replay = FIXTURE.replace(
        "  evidence:\n    host: 10.21.12.162\n    port: 4200\n    camera: 0\n",
        "",
    );
    let config = Config::from_yaml(&replay).expect("replay without evidence must parse");
    assert!(config.capture.as_ref().expect("capture").evidence.is_none());
    let video = replay.replace(
        "    type: rtsp\n    url: rtsp://10.21.12.162:554/PRR",
        "    type: video\n    path: /tmp/clip.mp4",
    );
    let config = Config::from_yaml(&video).expect("video guidance source must parse");
    assert!(matches!(
        config.capture.expect("capture").guidance,
        rigcal_core::config::GuidanceSource::Video { .. }
    ));
}
