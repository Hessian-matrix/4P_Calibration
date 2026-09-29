//! 单一配置文件的解析与校验（对应 `src/config.rs`）。
//!
//! 关注点：**未知键必须报错**、板参数必须内联且已实测、越界值一律拒绝——
//! 静默忽略会让人以为「配了门禁」，实际没生效。

use rigcal_core::config::Config;
use rigcal_core::models::ModelKind;

/// 夹具原文。`include_str!` 会把检出时的换行原样带进来：Windows 上 git 默认
/// `core.autocrlf=true`，这里拿到的是 CRLF；Unix 检出是 LF。
const FIXTURE: &str = include_str!("fixtures/session_single.yaml");

/// 夹具里的证据源段落（顶层，v2 起证据服务是全局一个）。按 LF 书写，
/// `patch`/`try_patch` 负责换成源文本的换行。
const EVIDENCE_SECTION: &str = "evidence:\n  host: 10.21.12.162\n  port: 4200\n";

/// 追加的第二路相机：`cameras` 逐路给引导源，通道号缺省取 `id` 里的 N。
const SECOND_CAMERA: &str =
    "  - id: cam1\n    guidance:\n      type: rtsp\n      url: rtsp://10.21.12.162:554/PRR1\n";

/// 外参段：**不写** `required_edges`/`required_cycles` 时默认值是 `cam0..cam3` 的四路链/整环，
/// 两路 rig 会因引用不存在的 cam2/cam3 而失败，所以这里显式给出两路的边与（空）环路。
const EXTRINSICS_TWO_CAMERAS: &str = "extrinsics:\n  required_edges: [[cam0, cam1]]\n  \
                                      required_cycles: []\n  min_groups_per_edge: 3\n  \
                                      max_edge_rms_px: 1.0\n  max_cycle_rotation_deg: 1.0\n  \
                                      max_cycle_translation_mm: 10.0\n";

/// 只含门禁、不含任何引用的外参段：单相机文件里出现它必须被专门的「仅 ≥2 路」校验拦下，
/// 而不是靠「引用了不存在的相机」这种副作用报错。
const EXTRINSICS_GATES_ONLY: &str = "extrinsics:\n  required_edges: []\n  required_cycles: []\n  \
                                     min_groups_per_edge: 3\n  max_edge_rms_px: 1.0\n  \
                                     max_cycle_rotation_deg: 1.0\n  max_cycle_translation_mm: 10.0\n";

/// 在 cam0 的引导源之后追加 cam1。
fn with_second_camera(fixture: &str) -> String {
    patch(
        fixture,
        "      url: rtsp://10.21.12.162:554/PRR\n",
        &format!("      url: rtsp://10.21.12.162:554/PRR\n{SECOND_CAMERA}"),
    )
}

/// 在 `board:` 之前插入一段顶层 YAML。
fn with_section(fixture: &str, section: &str) -> String {
    patch(
        fixture,
        "board:\n  target_type:",
        &format!("{section}board:\n  target_type:"),
    )
}

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
        assert_eq!(config.rig_id, "cam0_single");
        assert_eq!(config.image_size(), (1280, 1088));
        // 单相机 = 只有一路 `cameras`；通道号缺省取 id 里的 N。
        assert_eq!(config.cameras.len(), 1);
        assert_eq!(config.cameras[0].id, "cam0");
        assert_eq!(config.cameras[0].channel(), 0);
        let evidence = config.evidence.as_ref().expect("evidence source");
        assert_eq!(
            (evidence.host.as_str(), evidence.port),
            ("10.21.12.162", 4200)
        );
        // 外参段只对 ≥2 路有意义：单相机文件里必须是空的。
        assert!(config.extrinsics.is_none(), "single camera has no extrinsics");
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
                patch(&fixture, "schema_version: 2", "schema_version: 1"),
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
        let camera = config.cameras.first().expect("cam0");
        match &camera.guidance {
            rigcal_core::config::GuidanceSource::Rtsp { url } => {
                assert_eq!(url, "rtsp://10.21.12.162:554/PRR");
            }
            other => panic!("expected rtsp guidance source, got {other:?}"),
        }
        let evidence = config.evidence.as_ref().expect("evidence source");
        assert_eq!(
            (evidence.host.as_str(), evidence.port),
            ("10.21.12.162", 4200)
        );

        // 证据源端口 0 / 引导源空串一律拒绝
        let zero_port = patch(&fixture, "  port: 4200", "  port: 0");
        let error = Config::from_yaml(&zero_port).expect_err("port 0 must fail");
        assert!(error.0.contains("port"), "{}", error.0);
        let empty_url = patch(
            &fixture,
            "      url: rtsp://10.21.12.162:554/PRR",
            "      url: \"\"",
        );
        let error = Config::from_yaml(&empty_url).expect_err("empty url must fail");
        assert!(error.0.contains("url"), "{}", error.0);

        // 单相机离线回放：证据源可以缺省；本地视频走与 RTSP 同一条解码路径
        let replay = patch(&fixture, EVIDENCE_SECTION, "");
        let config = Config::from_yaml(&replay).expect("replay without evidence must parse");
        assert!(config.evidence.is_none(), "replay has no evidence source");
        let video = patch(
            &replay,
            "      type: rtsp\n      url: rtsp://10.21.12.162:554/PRR",
            "      type: video\n      path: /tmp/clip.mp4",
        );
        let config = Config::from_yaml(&video).expect("video guidance source must parse");
        assert!(matches!(
            &config.cameras[0].guidance,
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
        ("  port: 4200", "  port: 0"),
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
    assert!(replay.evidence.is_none());

    // 锚点缺失必须显式失败，而不是返回未变异的夹具。
    assert!(try_patch(&lf, "  no_such_key: 1", "  no_such_key: 2").is_none());
    assert!(try_patch(&crlf, "  no_such_key: 1", "  no_such_key: 2").is_none());
}

/// 外参约束/门禁只对 ≥2 路有意义：单相机文件里出现 `extrinsics` 必须报错，
/// 而不是静默忽略。这里刻意给一个**不含任何相机引用**的外参段，确保拦下它的是
/// 「仅 ≥2 路允许」这条规则本身，而不是「引用了不存在的相机」这种副作用。
#[test]
fn extrinsics_is_rejected_for_single_camera() {
    for fixture in fixture_variants() {
        let patched = with_section(&fixture, EXTRINSICS_GATES_ONLY);
        let error = Config::from_yaml(&patched).expect_err("1 路的 extrinsics 必须报错");
        assert!(
            error.0.contains("extrinsics"),
            "error must name extrinsics: {}",
            error.0
        );
    }
}

/// 合并后的核心规则：`cameras` 的长度决定一切——2 路就必须有 `extrinsics`（外参图与门禁）
/// 和 `evidence`（按同一时刻取多路的帧组来源），三者缺一不可；齐备才是一条两路 rig。
#[test]
fn multi_camera_requires_extrinsics_and_evidence() {
    for fixture in fixture_variants() {
        // 两路、仍只有单路证据：缺 extrinsics 段
        let two_cameras = with_second_camera(&fixture);
        let error = Config::from_yaml(&two_cameras).expect_err("2 路缺 extrinsics 必须报错");
        assert!(
            error.0.contains("extrinsics"),
            "error must ask for extrinsics: {}",
            error.0
        );

        // 补上 extrinsics 又删掉 evidence：帧组来源缺失
        let no_evidence = patch(&two_cameras, EVIDENCE_SECTION, "");
        let with_extrinsics = with_section(&no_evidence, EXTRINSICS_TWO_CAMERAS);
        let error = Config::from_yaml(&with_extrinsics).expect_err("2 路缺 evidence 必须报错");
        assert!(
            error.0.contains("evidence"),
            "error must ask for evidence: {}",
            error.0
        );

        // evidence + extrinsics 齐备：两路正例
        let complete = with_section(&two_cameras, EXTRINSICS_TWO_CAMERAS);
        let config = Config::from_yaml(&complete).expect("2 路 + evidence + extrinsics 必须通过");
        assert_eq!(config.cameras.len(), 2);
        assert_eq!(config.cameras[0].id, "cam0");
        assert_eq!(config.cameras[1].id, "cam1");
        assert_eq!(config.cameras[1].channel(), 1, "缺省通道号取 camN 的 N");
        assert!(config.extrinsics.is_some(), "2 路必须带外参段");
        let evidence = config.evidence.as_ref().expect("evidence source");
        assert_eq!(
            (evidence.host.as_str(), evidence.port),
            ("10.21.12.162", 4200)
        );
    }
}

/// 通道号是板端 raw 的取流索引：两路相机抢同一个通道会静默拿到别人的帧，
/// 因此显式 `channel` 与 `camN` 推出的缺省值重复时必须报错（这里 cam1 显式写 0）。
#[test]
fn duplicate_camera_channel_is_rejected() {
    for fixture in fixture_variants() {
        let complete = with_section(&with_second_camera(&fixture), EXTRINSICS_TWO_CAMERAS);
        let duplicated = patch(&complete, "  - id: cam1\n", "  - id: cam1\n    channel: 0\n");
        let error = Config::from_yaml(&duplicated).expect_err("通道号重复必须报错");
        assert!(
            error.0.contains("channel"),
            "error must name the channel clash: {}",
            error.0
        );
    }
}

/// `id` 必须形如 `camN`：N 既是板端 raw 的缺省通道号，也是外参边/环路里的相机名。
#[test]
fn camera_ids_must_look_like_cam_n() {
    for fixture in fixture_variants() {
        let patched = patch(&fixture, "  - id: cam0\n", "  - id: camera0\n");
        let error = Config::from_yaml(&patched).expect_err("id 不是 camN 必须报错");
        assert!(
            error.0.contains("camN"),
            "error must state the id shape: {}",
            error.0
        );
    }
}

/// v1 的顶层 `device:` / `capture:` 段落已合并进 `rig_id`/`image_size`/`cameras`：
/// 旧写法必须按未知键报错，而不是被静默丢掉（那会让人以为配置生效了）。
#[test]
fn legacy_device_and_capture_sections_are_rejected() {
    for fixture in fixture_variants() {
        let with_device = patch(&fixture, EVIDENCE_SECTION, "device:\n  camera_id: cam0\n");
        let error = Config::from_yaml(&with_device).expect_err("device 段是 v1 写法");
        assert!(
            error.0.contains("device"),
            "error must name the offender: {}",
            error.0
        );

        let with_capture = with_section(
            &fixture,
            "capture:\n  guidance:\n    type: rtsp\n    url: rtsp://10.21.12.162:554/PRR\n",
        );
        let error = Config::from_yaml(&with_capture).expect_err("capture 段是 v1 写法");
        assert!(
            error.0.contains("capture"),
            "error must name the offender: {}",
            error.0
        );
    }
}
