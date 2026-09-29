//! 单一配置文件的解析与校验（对应 `src/config.rs`）。
//!
//! 关注点：**未知键必须报错**、板参数必须内联且已实测、越界值一律拒绝——
//! 静默忽略会让人以为「配了门禁」，实际没生效。

use rigcal_core::config::Config;
use rigcal_core::models::ModelKind;

/// 夹具原文。`include_str!` 会把检出时的换行原样带进来：Windows 上 git 默认
/// `core.autocrlf=true`，这里拿到的是 CRLF；Unix 检出是 LF。
const FIXTURE: &str = include_str!("fixtures/session_single.yaml");

/// 夹具里的证据源段落。按 LF 书写，`patch`/`try_patch` 负责换成源文本的换行。
const EVIDENCE_SECTION: &str =
    "  evidence:\n    host: 10.21.12.162\n    port: 4200\n    camera: 0\n";

/// 按源文本自身的换行风格做锚点替换：`from`/`to` 一律用 LF 书写，
/// 匹配与写入时转换成源文本的换行，使变异与检出换行无关。
/// 锚点缺失返回 `None`，而不是让 `replace` 静默返回未变异的文本。
fn try_patch(source: &str, from: &str, to: &str) -> Option<String> {
    let eol = if source.contains("\r\n") { "\r\n" } else { "\n" };
    let anchor = from.replace('\n', eol);
    source
        .contains(&anchor)
        .then(|| source.replace(&anchor, &to.replace('\n', eol)))
}

/// `try_patch` 的断言版：锚点缺失直接 panic。替换静默失效会让用例在
/// **未变异**的夹具上“通过”——正是 Windows 上三条用例失败的根因。
fn patch(source: &str, from: &str, to: &str) -> String {
    try_patch(source, from, to)
        .unwrap_or_else(|| panic!("fixture anchor missing (checkout-dependent): {from:?}"))
}

/// 夹具的两种检出形态：LF（Unix / `core.autocrlf=input`）与 CRLF（Windows 默认）。
/// 同一套断言必须在两种形态下都成立，否则 CRLF 检出要么漏测、要么假绿。
fn fixture_variants() -> [String; 2] {
    let lf = FIXTURE.replace("\r\n", "\n");
    assert!(!lf.contains('\r'), "fixture must normalize to bare LF");
    let crlf = lf.replace('\n', "\r\n");
    [lf, crlf]
}

#[test]
fn fixture_parses_and_resolves_board() {
    for fixture in fixture_variants() {
        let config = Config::from_yaml(&fixture).expect("fixture must parse");
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
}

#[test]
fn unknown_keys_are_rejected() {
    for fixture in fixture_variants() {
        let with_unknown = patch(&fixture, "output:", "ui:\n  preview_scale: 0.5\noutput:");
        let error = Config::from_yaml(&with_unknown).expect_err("unknown section must fail");
        assert!(
            error.0.contains("ui"),
            "error must name the offender: {}",
            error.0
        );

        let nested = patch(
            &fixture,
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
}

#[test]
fn board_target_file_is_rejected() {
    // 单一配置文件：老写法（引用独立 target.yaml）必须报错，而不是静默丢掉板参数
    for fixture in fixture_variants() {
        let legacy = patch(
            &fixture,
            "  target_type: aprilgrid",
            "  target: local/target.yaml\n  target_type: aprilgrid",
        );
        assert!(Config::from_yaml(&legacy).is_err());
    }
}

#[test]
fn unmeasured_board_is_rejected() {
    for fixture in fixture_variants() {
        let unmeasured = patch(&fixture, "  measured: true", "  measured: false");
        let error = Config::from_yaml(&unmeasured).expect_err("unmeasured board must fail");
        assert!(error.0.contains("measured"), "{}", error.0);
    }
}

#[test]
fn out_of_range_values_are_rejected() {
    for fixture in fixture_variants() {
        for (patched, needle) in [
            (
                patch(&fixture, "schema_version: 1", "schema_version: 2"),
                "schema_version",
            ),
            (
                patch(&fixture, "  models: [kb4, ds]", "  models: [kb4, kb4]"),
                "repeats",
            ),
            (
                patch(&fixture, "  models: [kb4, ds]", "  models: []"),
                "at least one model",
            ),
            (
                patch(&fixture, "  holdout_fraction: 0.25", "  holdout_fraction: 1.0"),
                "holdout_fraction",
            ),
            (
                patch(
                    &fixture,
                    "    max_saturated_fraction: 0.35",
                    "    max_saturated_fraction: 1.5",
                ),
                "saturated",
            ),
            (
                patch(&fixture, "    - [-0.2, 0.35]", "    - [-1.5, 0.35]"),
                "xi",
            ),
            (
                patch(&fixture, "    - [0.0, 0.5]", "    - [0.0, 1.0]"),
                "alpha",
            ),
            (
                patch(&fixture, "  min_observations: 12", "  min_observations: 0"),
                "min_observations",
            ),
            (
                patch(
                    &fixture,
                    "  max_solve_observations: 30",
                    "  max_solve_observations: 5",
                ),
                "max_solve_observations",
            ),
            (
                patch(
                    &fixture,
                    "output:\n  root: calibration_runs",
                    "output:\n  root: \"\"",
                ),
                "root",
            ),
            (
                patch(&fixture, "    min_focus_score: 500.0", "    min_focus_score: -1.0"),
                "min_focus_score",
            ),
            (
                patch(&fixture, "  detect_scale: 0.75", "  detect_scale: 1.5"),
                "detect_scale",
            ),
            (
                patch(
                    &fixture,
                    "  trigger_novelty_scale: 3.0",
                    "  trigger_novelty_scale: 0.5",
                ),
                "trigger_novelty_scale",
            ),
            (
                patch(&fixture, "  jitter_rotation_deg: 2.0", "  jitter_rotation_deg: 0.0"),
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
}

#[test]
fn legacy_flow_and_laplacian_fields_are_rejected() {
    // 已移除的光流门禁字段与清晰度字段旧名必须报未知键，而不是静默忽略。
    for fixture in fixture_variants() {
        for legacy in ["flow_enabled", "flow_stable_px", "flow_window"] {
            let patched = patch(
                &fixture,
                "  detect_hz_max: 12.0\n",
                &format!("  detect_hz_max: 12.0\n  {legacy}: 1\n"),
            );
            let error = Config::from_yaml(&patched).expect_err("legacy flow field must fail");
            assert!(error.0.contains(legacy), "{}", error.0);
        }
        let legacy_quality = patch(
            &fixture,
            "    min_focus_score: 500.0",
            "    min_laplacian_var: 15.0",
        );
        let error =
            Config::from_yaml(&legacy_quality).expect_err("legacy Laplacian field must fail");
        assert!(error.0.contains("min_laplacian_var"), "{}", error.0);
    }
}

#[test]
fn guidance_and_evidence_sources_are_validated() {
    for fixture in fixture_variants() {
        let config = Config::from_yaml(&fixture).expect("fixture must parse");
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
        let zero_port = patch(&fixture, "    port: 4200", "    port: 0");
        let error = Config::from_yaml(&zero_port).expect_err("port 0 must fail");
        assert!(error.0.contains("port"), "{}", error.0);
        let empty_url = patch(&fixture, "    url: rtsp://10.21.12.162:554/PRR", "    url: \"\"");
        let error = Config::from_yaml(&empty_url).expect_err("empty url must fail");
        assert!(error.0.contains("url"), "{}", error.0);

        // 离线回放：证据源可以缺省；本地视频走与 RTSP 同一条解码路径
        let replay = patch(&fixture, EVIDENCE_SECTION, "");
        let config = Config::from_yaml(&replay).expect("replay without evidence must parse");
        assert!(config.capture.as_ref().expect("capture").evidence.is_none());
        let video = patch(
            &replay,
            "    type: rtsp\n    url: rtsp://10.21.12.162:554/PRR",
            "    type: video\n    path: /tmp/clip.mp4",
        );
        let config = Config::from_yaml(&video).expect("video guidance source must parse");
        assert!(matches!(
            config.capture.expect("capture").guidance,
            rigcal_core::config::GuidanceSource::Video { .. }
        ));
    }
}

/// Windows 检出（CRLF）与 Unix 检出（LF）必须走同一条解析/校验路径：
/// 解析出同一份配置，同一处越界报同一条错——即三条 Windows 失败用例的根因。
#[test]
fn crlf_checkout_parses_and_rejects_like_lf() {
    let [lf, crlf] = fixture_variants();
    assert!(!lf.contains('\r'), "lf variant must be bare LF");
    assert!(crlf.contains("\r\n"), "crlf variant must keep CRLF: {crlf:?}");

    let lf_config = Config::from_yaml(&lf).expect("lf fixture must parse");
    let crlf_config = Config::from_yaml(&crlf).expect("crlf fixture must parse");
    assert_eq!(lf_config, crlf_config, "LF/CRLF must resolve identically");

    // 拒绝路径不依赖检出换行：包括跨行锚点（`output:` 段）在内。
    for (from, to) in [
        ("  measured: true", "  measured: false"),
        ("    port: 4200", "    port: 0"),
        ("  detect_scale: 0.75", "  detect_scale: 1.5"),
        (
            "  detect_hz_max: 12.0\n",
            "  detect_hz_max: 12.0\n  flow_enabled: 1\n",
        ),
        ("output:\n  root: calibration_runs", "output:\n  root: \"\""),
    ] {
        let lf_error = Config::from_yaml(&patch(&lf, from, to)).expect_err("lf must reject");
        let crlf_error = Config::from_yaml(&patch(&crlf, from, to)).expect_err("crlf must reject");
        assert_eq!(
            lf_error.0, crlf_error.0,
            "rejection differs by checkout newline"
        );
    }

    // 变异与检出换行无关：CRLF 结果就是 LF 结果换成 CRLF。
    let lf_replay = patch(&lf, EVIDENCE_SECTION, "");
    let crlf_replay = patch(&crlf, EVIDENCE_SECTION, "");
    assert_eq!(crlf_replay, lf_replay.replace('\n', "\r\n"));
    let replay = Config::from_yaml(&crlf_replay).expect("crlf replay must parse");
    assert!(replay.capture.as_ref().expect("capture").evidence.is_none());

    // 锚点缺失必须显式失败，而不是返回未变异的夹具。
    assert!(try_patch(&lf, "  no_such_key: 1", "  no_such_key: 2").is_none());
    assert!(try_patch(&crlf, "  no_such_key: 1", "  no_such_key: 2").is_none());
}
