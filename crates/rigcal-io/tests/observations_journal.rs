//! 观测留存日志的边界回归：写盘-重放一致、不覆盖历史、拒绝非法组、坏日志不假装全量恢复。
//!
//! 全部在**消费端**核对：`create`/`append` 之后用 `read_observations` 读回，比对恢复出的配置与
//! 观测，而不是断言源码或内部字段。

use std::io::Write;

use rigcal_core::config::Config;
use rigcal_core::estimator::Observation;
use rigcal_io::observations::{
    JournalError, RecordedGroup, RecordedView, create, read_observations,
};

fn camera_lines(count: usize) -> String {
    (0..count)
        .map(|index| {
            format!("  - {{id: cam{index}, guidance: {{type: video, path: /tmp/cam{index}.mp4}}}}")
        })
        .collect::<Vec<_>>()
        .join("\n")
}

fn four_camera_config() -> Config {
    let yaml = format!(
        r#"schema_version: 2
rig_id: test_rig
image_size: [1280, 1088]
cameras:
{cameras}
evidence: {{host: 127.0.0.1, port: 4211}}
extrinsics:
  required_edges: [[cam0, cam1], [cam1, cam2], [cam2, cam3]]
  required_cycles: [[cam0, cam1, cam2, cam3]]
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
"#,
        cameras = camera_lines(4),
    );
    Config::from_yaml(&yaml).expect("config")
}

fn observation(seed: f64) -> Observation {
    Observation {
        object_points: vec![
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.1, 0.1, 0.0],
            [0.0, 0.1, 0.0],
        ],
        image_points: vec![
            [100.0 + seed, 100.0],
            [200.0 + seed, 100.0],
            [200.0 + seed, 200.0],
            [100.0 + seed, 200.0],
        ],
    }
}

fn group(version: usize, group_id: u64, group_timestamp_ns: u64) -> RecordedGroup {
    RecordedGroup {
        version,
        group_id,
        group_timestamp_ns,
        views: (0..4)
            .map(|index| RecordedView {
                camera_id: format!("cam{index}"),
                frame_id: group_id * 10 + index as u64,
                camera_timestamp_ns: group_timestamp_ns + index as u64,
                observation: observation(version as f64),
            })
            .collect(),
    }
}

fn temp_root(tag: &str) -> std::path::PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .expect("clock")
        .as_nanos();
    let root = std::env::temp_dir().join(format!(
        "rigcal-io-journal-{tag}-{}-{nanos}",
        std::process::id()
    ));
    std::fs::create_dir_all(&root).expect("temp root");
    root
}

#[test]
fn recorded_groups_replay_identically_after_a_restart() {
    let root = temp_root("roundtrip");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");
    let groups = vec![group(1, 100, 1_000), group(2, 101, 2_000)];
    for recorded in &groups {
        journal.append(recorded).expect("append");
    }
    let path = journal.path().to_path_buf();
    drop(journal);

    let (loaded, replayed) = read_observations(&path).expect("read");
    assert_eq!(loaded, config);
    assert_eq!(replayed, groups);
    assert_eq!(replayed[1].views[2].camera_id, "cam2");
    assert_eq!(replayed[1].views[2].observation, observation(2.0));
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn create_never_overwrites_a_previous_session() {
    let root = temp_root("distinct");
    let config = four_camera_config();
    let mut first = create(&root, &config).expect("first");
    first.append(&group(1, 10, 100)).expect("append");
    let first_path = first.path().to_path_buf();

    let second = create(&root, &config).expect("second");
    let second_path = second.path().to_path_buf();
    assert_ne!(first_path, second_path);

    let (_, replayed) = read_observations(&first_path).expect("first intact");
    assert_eq!(replayed.len(), 1);
    let (_, empty) = read_observations(&second_path).expect("second intact");
    assert!(empty.is_empty());

    let sessions = root.join("sessions");
    assert_eq!(
        std::fs::read_dir(&sessions).expect("sessions").count(),
        2,
        "each create must claim its own session directory"
    );
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn append_rejects_monotonicity_regression_and_stays_writable() {
    let root = temp_root("monotonic");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");
    journal.append(&group(1, 10, 100)).expect("v1");

    let error = journal
        .append(&group(1, 11, 200))
        .expect_err("same version");
    assert!(matches!(
        error,
        JournalError::VersionNotMonotonic {
            version: 1,
            last: 1
        }
    ));

    let error = journal
        .append(&group(2, 5, 300))
        .expect_err("group id regression");
    assert!(matches!(
        error,
        JournalError::GroupIdNotMonotonic {
            group_id: 5,
            last: 10
        }
    ));

    journal.append(&group(2, 12, 400)).expect("recovery");
    let path = journal.path().to_path_buf();
    let (_, replayed) = read_observations(&path).expect("read");
    let versions: Vec<usize> = replayed.iter().map(|recorded| recorded.version).collect();
    assert_eq!(versions, vec![1, 2]);
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn append_rejects_repeated_or_foreign_cameras() {
    let root = temp_root("cameras");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");

    let mut duplicated = group(1, 10, 100);
    duplicated.views[1].camera_id = "cam0".to_owned();
    let error = journal.append(&duplicated).expect_err("duplicate");
    assert!(matches!(error, JournalError::DuplicateCamera { .. }));

    let mut foreign = group(1, 10, 100);
    foreign.views[3].camera_id = "cam4".to_owned();
    let error = journal.append(&foreign).expect_err("foreign");
    assert!(matches!(error, JournalError::UnknownCamera { .. }));

    let path = journal.path().to_path_buf();
    let (_, replayed) = read_observations(&path).expect("read");
    assert!(
        replayed.is_empty(),
        "a rejected group must not reach the log"
    );
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn append_rejects_empty_mismatched_or_non_finite_corners() {
    let root = temp_root("corners");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");
    journal.append(&group(1, 10, 100)).expect("baseline");

    let mut mismatched = group(2, 11, 200);
    mismatched.views[0]
        .observation
        .object_points
        .push([0.0, 0.0, 0.0]);
    let error = journal.append(&mismatched).expect_err("mismatch");
    assert!(matches!(error, JournalError::CornerCountMismatch { .. }));

    let mut empty = group(2, 11, 200);
    empty.views[0].observation.object_points.clear();
    empty.views[0].observation.image_points.clear();
    let error = journal.append(&empty).expect_err("empty");
    assert!(matches!(error, JournalError::CornerCountMismatch { .. }));

    let mut non_finite = group(2, 11, 200);
    non_finite.views[0].observation.image_points[2][0] = f64::NAN;
    let error = journal.append(&non_finite).expect_err("nan");
    assert!(matches!(error, JournalError::NonFiniteCorner { .. }));

    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn read_rejects_a_truncated_journal() {
    let root = temp_root("truncated");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");
    journal.append(&group(1, 10, 100)).expect("v1");
    journal.append(&group(2, 11, 200)).expect("v2");
    let path = journal.path().to_path_buf();
    drop(journal);

    let bytes = std::fs::read(&path).expect("read bytes");
    std::fs::write(&path, &bytes[..bytes.len() - 10]).expect("truncate");

    let error = read_observations(&path).expect_err("truncated");
    assert!(matches!(error, JournalError::Truncated { line: 2, .. }));
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn read_rejects_a_line_carrying_unknown_fields() {
    let root = temp_root("corrupt");
    let config = four_camera_config();
    let mut journal = create(&root, &config).expect("create");
    journal.append(&group(1, 10, 100)).expect("v1");
    let path = journal.path().to_path_buf();
    drop(journal);

    let mut file = std::fs::OpenOptions::new()
        .append(true)
        .open(&path)
        .expect("open");
    file.write_all(
        b"{\"version\":2,\"group_id\":11,\"group_timestamp_ns\":200,\"views\":[],\"extra\":1}\n",
    )
    .expect("write");
    drop(file);

    let error = read_observations(&path).expect_err("unknown field");
    assert!(matches!(error, JournalError::Line { line: 2, .. }));
    std::fs::remove_dir_all(&root).expect("cleanup");
}

#[test]
fn read_requires_the_adjacent_config() {
    let root = temp_root("noconfig");
    let config = four_camera_config();
    let journal = create(&root, &config).expect("create");
    let path = journal.path().to_path_buf();
    let config_path = path.parent().expect("parent").join("config.yaml");
    std::fs::remove_file(&config_path).expect("remove config");

    let error = read_observations(&path).expect_err("missing config");
    assert!(matches!(error, JournalError::MissingConfig { .. }));
    std::fs::remove_dir_all(&root).expect("cleanup");
}
