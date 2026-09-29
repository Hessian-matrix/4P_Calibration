//! Bounded capture → evidence audit/commit → coalesced numerical snapshots.
//! A matching group id and timestamp is the device's same-exposure contract.

use std::collections::{BTreeMap, HashMap};
use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, SyncSender};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use opencv::core::{Mat, Scalar};
use opencv::prelude::*;
use rigcal_camera::{backend_for, detect, live::PoseMeasurement};
use rigcal_core::config::Config;
use rigcal_core::estimator::Observation;
use rigcal_core::extrinsics::{ObservationRecord, reference_transforms, solve_rig};
use rigcal_core::models::ModelKind;
use rigcal_core::session::{Session, SessionOptions, SessionState, SessionThresholds};
use rigcal_io::group::{CameraGroupSpec, FrameGroup, GroupCapture, GroupCaptureOptions};
use rigcal_io::observations::{self, ObservationJournal, RecordedGroup, RecordedView};
use rigcal_opencv::quality::{QualityThresholds, classify_frame_quality};

use crate::{
    CalibrationSnapshot, Shared, log_line, metrics_of, request_clock_calibration, save_calibration,
};

/// Identity belongs to the analysis frame, never to the displayed overlay.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameKey {
    pub source_epoch: u64,
    pub camera_index: usize,
    pub sequence: u64,
    pub pts_ns: i64,
}

pub struct Candidate {
    pub key: FrameKey,
    pub target_board_ns: u64,
    pub clock_revision: u64,
    pub frame_period_ns: i64,
    pub created_at: Instant,
    pub pose: PoseMeasurement,
    pub reply: Sender<CaptureResult>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CaptureStatus {
    Committed,
    Rejected,
    Failed,
}

pub struct CaptureResult {
    pub key: FrameKey,
    pub status: CaptureStatus,
    pub accepted_cameras: Vec<usize>,
    pub pose: PoseMeasurement,
    pub detail: String,
}

impl Candidate {
    fn finish(self, status: CaptureStatus, accepted_cameras: Vec<usize>, detail: String) {
        log_line(format!(
            "采集回执 {:?} cam{} frame={} pts={}：{}",
            status, self.key.camera_index, self.key.sequence, self.key.pts_ns, detail
        ));
        let _ = self.reply.send(CaptureResult {
            key: self.key,
            status,
            accepted_cameras,
            pose: self.pose,
            detail,
        });
    }
}

/// One replaceable pending value. Closing drains the last dataset, not an unbounded FIFO.
struct LatestState<T> {
    value: Option<T>,
    closed: bool,
}
pub struct Latest<T> {
    state: Mutex<LatestState<T>>,
    changed: Condvar,
}
impl<T> Latest<T> {
    fn new() -> Self {
        Self {
            state: Mutex::new(LatestState {
                value: None,
                closed: false,
            }),
            changed: Condvar::new(),
        }
    }
    fn replace(&self, value: T) -> Result<Option<T>, T> {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        if state.closed {
            return Err(value);
        }
        let previous = state.value.replace(value);
        self.changed.notify_one();
        Ok(previous)
    }
    fn recv(&self) -> Option<T> {
        let state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        let mut state = self
            .changed
            .wait_while(state, |s| s.value.is_none() && !s.closed)
            .unwrap_or_else(|e| e.into_inner());
        state.value.take()
    }
    pub fn is_closed(&self) -> bool {
        self.state.lock().unwrap_or_else(|e| e.into_inner()).closed
    }
    pub fn close(&self) {
        let mut state = self.state.lock().unwrap_or_else(|e| e.into_inner());
        state.closed = true;
        self.changed.notify_all();
    }
}

impl Latest<Candidate> {
    pub fn submit(&self, candidate: Candidate) {
        match self.replace(candidate) {
            Ok(Some(old)) => old.finish(
                CaptureStatus::Rejected,
                vec![],
                "忙时合并：由较新候选替代".to_owned(),
            ),
            Err(old) => old.finish(CaptureStatus::Rejected, vec![], "采集已停止".to_owned()),
            Ok(None) => {}
        }
    }
}

struct EvidenceView {
    camera_index: usize,
    record: ObservationRecord,
}
struct Dataset {
    version: usize,
    views: Vec<Arc<EvidenceView>>,
}
struct Auditable {
    candidate: Candidate,
    group: FrameGroup,
}

pub enum ExportCommand {
    Save(Arc<CalibrationSnapshot>),
    Finish(Option<Arc<CalibrationSnapshot>>),
}

pub struct Pipeline {
    pub candidates: Arc<Latest<Candidate>>,
    exports: SyncSender<ExportCommand>,
    capture: JoinHandle<Result<(), String>>,
    audit: JoinHandle<Result<(), String>>,
    solver: JoinHandle<Result<(), String>>,
    exporter: JoinHandle<Result<(), String>>,
    shared: Arc<Shared>,
}

pub fn quality_thresholds(config: &Config) -> QualityThresholds {
    QualityThresholds {
        min_focus_score: config.solver.quality.min_focus_score,
        min_contrast: config.solver.quality.min_contrast,
        max_saturated_fraction: config.solver.quality.max_saturated_fraction,
    }
}

pub fn request_export(shared: &Shared, candidates: &Latest<Candidate>) {
    if let Ok(mut state) = shared.rig.lock() {
        if state.export_requested || state.finalizing || state.finalized {
            return;
        }
        state.export_requested = true;
        state.export_failed = false;
        state.export_message = "正在结束采集；排空审核后全量精修，再导出最新版本".to_owned();
    }
    // Closing drains the one in-flight candidate; the audit/solver queues then drain in order.
    candidates.close();
}

impl Pipeline {
    pub fn start(
        config: Config,
        shared: Arc<Shared>,
        stop: Arc<AtomicBool>,
        max_groups: Option<usize>,
    ) -> Result<Self, String> {
        let evidence = config
            .evidence
            .as_ref()
            .ok_or("配置缺少 evidence 段（板端 raw 服务端点）")?;
        let specs: Vec<_> = config
            .cameras
            .iter()
            .map(|camera| CameraGroupSpec {
                camera_id: camera.id.clone(),
                raw_camera_id: camera.channel() as i32,
            })
            .collect();
        let primary = specs
            .first()
            .ok_or("配置没有 cameras（多路 rig 至少两路）")?;
        let capture = GroupCapture::new(
            &evidence.host,
            evidence.port,
            &specs,
            &primary.camera_id,
            config.image_size(),
            3.0,
            GroupCaptureOptions::default(),
        )
        .map_err(|e| e.to_string())?;
        let journal = observations::create(Path::new(&config.output.root), &config)
            .map_err(|error| format!("无法创建观测会话：{error}"))?;
        log_line(format!("完整角点观测：{}", journal.path().display()));
        let journal_path = journal.path().to_path_buf();
        let config = Arc::new(config);
        let candidates = Arc::new(Latest::new());
        let datasets = Arc::new(Latest::new());
        let (audit_tx, audit_rx) = mpsc::sync_channel(1);
        let (exports, export_rx) = mpsc::sync_channel(1);
        let exporter = std::thread::Builder::new()
            .name("calibration-export".to_owned())
            .spawn({
                let config = Arc::clone(&config);
                let shared = Arc::clone(&shared);
                let journal_path = journal_path.clone();
                move || {
                    let mut saved = None;
                    while let Ok(command) = export_rx.recv() {
                        match command {
                            ExportCommand::Save(snapshot) => {
                                let _ = save_calibration(
                                    Some(&snapshot),
                                    &config,
                                    Some(journal_path.as_path()),
                                    &shared,
                                    &mut saved,
                                );
                            }
                            ExportCommand::Finish(snapshot) => {
                                return save_calibration(
                                    snapshot.as_deref(),
                                    &config,
                                    Some(journal_path.as_path()),
                                    &shared,
                                    &mut saved,
                                );
                            }
                        }
                    }
                    Ok(())
                }
            })
            .map_err(|e| e.to_string())?;
        let solver = std::thread::Builder::new()
            .name("calibration-solve".to_owned())
            .spawn({
                let config = Arc::clone(&config);
                let shared = Arc::clone(&shared);
                let datasets = Arc::clone(&datasets);
                let candidates = Arc::clone(&candidates);
                let exports = exports.clone();
                move || {
                    let result = run_worker("求解", &shared, || {
                        solve_worker(&config, &shared, &datasets)
                    });
                    datasets.close();
                    candidates.close();
                    result?;
                    let snapshot = shared
                        .snapshot
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .clone()
                        .ok_or("全量精修没有生成完整结果")?;
                    exports
                        .send(ExportCommand::Save(snapshot))
                        .map_err(|error| error.to_string())?;
                    Ok(())
                }
            })
            .map_err(|e| e.to_string())?;
        let audit = std::thread::Builder::new()
            .name("evidence-audit".to_owned())
            .spawn({
                let config = Arc::clone(&config);
                let shared = Arc::clone(&shared);
                let datasets = Arc::clone(&datasets);
                let candidates = Arc::clone(&candidates);
                move || {
                    let result = run_worker("审核", &shared, || {
                        audit_worker(
                            &config,
                            &shared,
                            audit_rx,
                            &datasets,
                            &candidates,
                            max_groups,
                            journal,
                        )
                    });
                    datasets.close();
                    candidates.close();
                    result
                }
            })
            .map_err(|e| e.to_string())?;
        let capture = std::thread::Builder::new()
            .name("raw-capture".to_owned())
            .spawn({
                let shared = Arc::clone(&shared);
                let candidates = Arc::clone(&candidates);
                move || {
                    let result = run_worker("取组", &shared, || {
                        capture_worker(capture, &shared, &candidates, audit_tx, &stop);
                        Ok(())
                    });
                    candidates.close();
                    result
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            candidates,
            exports,
            capture,
            audit,
            solver,
            exporter,
            shared,
        })
    }

    pub fn finish(self) -> Result<(), String> {
        self.candidates.close();
        let capture = self
            .capture
            .join()
            .map_err(|_| "取组线程异常退出".to_owned())
            .and_then(|r| r);
        let audit = self
            .audit
            .join()
            .map_err(|_| "审核线程异常退出".to_owned())
            .and_then(|r| r);
        let solve = self
            .solver
            .join()
            .map_err(|_| "求解线程异常退出".to_owned())
            .and_then(|r| r);
        // Never export an earlier online snapshot when final refinement failed.
        let snapshot = if solve.is_ok() {
            self.shared
                .snapshot
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone()
        } else {
            None
        };
        let _ = self.exports.send(ExportCommand::Finish(snapshot));
        let export = self
            .exporter
            .join()
            .map_err(|_| "导出线程异常退出".to_owned())
            .and_then(|r| r);
        capture.and(audit).and(solve).and(export)
    }
}

// A failed stage must close its output and wake its neighbours, not strand finish() in join().
fn run_worker(
    name: &str,
    shared: &Shared,
    work: impl FnOnce() -> Result<(), String>,
) -> Result<(), String> {
    let result =
        std::panic::catch_unwind(std::panic::AssertUnwindSafe(work)).unwrap_or_else(|panic| {
            let reason = panic
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| {
                    panic
                        .downcast_ref::<&str>()
                        .map(|message| (*message).to_owned())
                })
                .unwrap_or_else(|| "panic".to_owned());
            Err(reason)
        });
    if let Err(error) = &result {
        let message = format!("{name}线程失败：{error}");
        log_line(&message);
        if let Ok(mut state) = shared.rig.lock() {
            state.last_error = Some(message);
            state.phase = "PIPELINE_ERROR".to_owned();
            state.finalizing = false;
            state.export_requested = false;
            state.export_failed = true;
            state.export_message = "流水线失败；完整角点仍留在会话日志，可离线重放".to_owned();
        }
    }
    result
}

fn capture_worker(
    capture: GroupCapture,
    shared: &Shared,
    candidates: &Latest<Candidate>,
    audit: SyncSender<Auditable>,
    stop: &AtomicBool,
) {
    while let Some(note) = candidates.recv() {
        if stop.load(Ordering::Acquire) {
            note.finish(
                CaptureStatus::Rejected,
                vec![],
                "已停止，取消尚未取图的候选".to_owned(),
            );
            continue;
        }
        let clock_valid = shared.tiles[note.key.camera_index]
            .lock()
            .is_ok_and(|tile| tile.aligner.is_some() && tile.clock_revision == note.clock_revision);
        // This is a freshness budget, not an assertion about the device's ring depth.
        if !clock_valid || note.created_at.elapsed() > Duration::from_millis(150) {
            note.finish(
                CaptureStatus::Rejected,
                vec![],
                "候选已过期或时钟已失效".to_owned(),
            );
            continue;
        }
        if let Ok(mut state) = shared.rig.lock() {
            state.triggers += 1;
        }
        let started = Instant::now();
        let group = match capture.capture_at(note.target_board_ns) {
            Ok(group) => group,
            Err(error) => {
                let message = format!("同刻取组失败（无 LATEST 回退）：{error}");
                if let Ok(mut state) = shared.rig.lock() {
                    state.capture_failures += 1;
                    state.alignment_failures += 1;
                    state.last_error = Some(message.clone());
                }
                request_clock_calibration(shared, note.key.camera_index);
                note.finish(CaptureStatus::Failed, vec![], message);
                continue;
            }
        };
        let hit = i128::from(group.group_timestamp_ns) - i128::from(note.target_board_ns);
        let tolerance = i128::from(note.frame_period_ns / 2 + 100_000);
        if hit.abs() > tolerance {
            let detail = format!(
                "命中误差 {} ns 超过半帧容差 {} ns，拒绝本组",
                hit, tolerance
            );
            if let Ok(mut state) = shared.rig.lock() {
                state.alignment_failures += 1;
                state.last_error = Some(detail.clone());
            }
            request_clock_calibration(shared, note.key.camera_index);
            note.finish(CaptureStatus::Rejected, vec![], detail);
            continue;
        }
        let mut clock = shared.tiles[note.key.camera_index]
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if clock.clock_revision != note.clock_revision || clock.aligner.is_none() {
            drop(clock);
            note.finish(
                CaptureStatus::Rejected,
                vec![],
                "取图期间时钟已失效".to_owned(),
            );
            continue;
        }
        clock
            .aligner
            .as_mut()
            .expect("checked clock")
            .fold_hit_error(hit as i64);
        drop(clock);
        if let Ok(mut state) = shared.rig.lock() {
            if state.hit_errors_ns.len() >= 64 {
                state.hit_errors_ns.remove(0);
            }
            state.hit_errors_ns.push(hit as i64);
        }
        log_line(format!(
            "raw 组 {}，同曝光合同通过，取组 {:.1} ms，命中误差 {:.3} ms",
            group.group_id,
            started.elapsed().as_secs_f64() * 1000.0,
            hit as f64 / 1e6
        ));
        if let Err(error) = audit.send(Auditable {
            candidate: note,
            group,
        }) {
            error
                .0
                .candidate
                .finish(CaptureStatus::Failed, vec![], "审核线程已退出".to_owned());
            break;
        }
    }
}

fn audit_worker(
    config: &Config,
    shared: &Shared,
    inbox: Receiver<Auditable>,
    datasets: &Latest<Dataset>,
    candidates: &Latest<Candidate>,
    max_groups: Option<usize>,
    mut journal: ObservationJournal,
) -> Result<(), String> {
    let board = config.board_config().expect("validated board");
    let quality = quality_thresholds(config);
    let mut border_bits = HashMap::new();
    let mut views: Vec<Arc<EvidenceView>> = Vec::new();
    let mut version = 0;
    let mut last_group: Option<(u64, u64)> = None;
    for Auditable { candidate, group } in inbox {
        if max_groups.is_some_and(|limit| version >= limit) {
            candidate.finish(CaptureStatus::Rejected, vec![], "已到入库上限".to_owned());
            continue;
        }
        if last_group
            .is_some_and(|(id, stamp)| group.group_id <= id || group.group_timestamp_ns <= stamp)
        {
            candidate.finish(
                CaptureStatus::Rejected,
                vec![],
                "重复或倒退组（设备重启须新建会话）".to_owned(),
            );
            continue;
        }
        let mut accepted = Vec::new();
        let mut accepted_cameras = Vec::new();
        let mut outcomes = Vec::new();
        let mut execution_failed = false;
        for entry in &group.frames {
            let Some(index) = config
                .cameras
                .iter()
                .position(|c| c.id == entry.camera_id)
            else {
                continue;
            };
            let audited = (|| -> Result<Option<Observation>, String> {
                let mut image = Mat::new_rows_cols_with_default(
                    entry.frame.header.height as i32,
                    entry.frame.header.width as i32,
                    opencv::core::CV_8UC1,
                    Scalar::default(),
                )
                .map_err(|e| e.to_string())?;
                image
                    .data_typed_mut::<u8>()
                    .map_err(|e| e.to_string())?
                    .copy_from_slice(&entry.frame.gray);
                let quality =
                    classify_frame_quality(&image, &quality).map_err(|e| e.to_string())?;
                if !quality.accepted {
                    outcomes.push(format!(
                        "{} 拒绝 {}",
                        entry.camera_id,
                        quality.reasons.join("/")
                    ));
                    return Ok(None);
                }
                let detection =
                    detect::detect_board(&image, &board, border_bits.get(&index).copied())
                        .map_err(|e| e.to_string())?;
                if let Some(bits) = detection.border_bits {
                    border_bits.insert(index, bits);
                }
                if !detection.detected() {
                    outcomes.push(format!("{} 无板", entry.camera_id));
                    return Ok(None);
                }
                Ok(Some(Observation {
                    object_points: detection.object_points,
                    image_points: detection.image_points,
                }))
            })();
            match audited {
                Ok(Some(observation)) => {
                    outcomes.push(format!(
                        "{} 接受 frame={}",
                        entry.camera_id, entry.frame.header.frame_id
                    ));
                    accepted_cameras.push(index);
                    accepted.push(RecordedView {
                        camera_id: entry.camera_id.clone(),
                        frame_id: entry.frame.header.frame_id,
                        camera_timestamp_ns: entry.frame.header.camera_timestamp_ns,
                        observation,
                    });
                }
                Ok(None) => {}
                Err(error) => {
                    execution_failed = true;
                    outcomes.push(format!("{} 执行失败 {error}", entry.camera_id));
                }
            }
        }
        let detail = outcomes.join("；");
        if accepted.is_empty() {
            if let Ok(mut state) = shared.rig.lock() {
                state.last_error = Some(detail.clone());
            }
            candidate.finish(
                if execution_failed {
                    CaptureStatus::Failed
                } else {
                    CaptureStatus::Rejected
                },
                vec![],
                detail,
            );
            continue;
        }
        // Recalibration/EOF also invalidates candidates already waiting in the audit queue.
        let clock = shared.tiles[candidate.key.camera_index]
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        if clock.clock_revision != candidate.clock_revision || clock.aligner.is_none() {
            drop(clock);
            candidate.finish(
                CaptureStatus::Rejected,
                vec![],
                "审核期间时钟已失效".to_owned(),
            );
            continue;
        }
        // This check is the acceptance point. Do not hold a GUI mutex across disk IO.
        drop(clock);
        let recorded = RecordedGroup {
            version: version + 1,
            group_id: group.group_id,
            group_timestamp_ns: group.group_timestamp_ns,
            views: accepted,
        };
        if let Err(error) = journal.append(&recorded) {
            let reason = format!("观测写盘失败，未入库：{error}");
            candidate.finish(CaptureStatus::Failed, vec![], reason.clone());
            return Err(reason);
        }
        views.extend(recorded.views.into_iter().zip(&accepted_cameras).map(
            |(view, &camera_index)| {
                Arc::new(EvidenceView {
                    camera_index,
                    record: ObservationRecord {
                        camera_id: view.camera_id,
                        group_id: group.group_id,
                        observation: view.observation,
                    },
                })
            },
        ));
        version += 1;
        last_group = Some((group.group_id, group.group_timestamp_ns));
        // The ledger and associations are committed before this receipt. Solving is not part of commit.
        for &index in &accepted_cameras {
            if let Ok(mut tile) = shared.tiles[index].lock() {
                tile.views += 1;
                tile.flash_until = Some(Instant::now() + Duration::from_millis(700));
            }
        }
        if let Ok(mut state) = shared.rig.lock() {
            state.groups = version;
            state.export_ready = version >= config.solver.min_observations;
            state.last_error = None;
            state.capture_message = format!("V{version} 组 {}：{detail}", group.group_id);
        }
        let solver_stopped = datasets
            .replace(Dataset {
                version,
                views: views.clone(),
            })
            .is_err();
        candidate.finish(
            CaptureStatus::Committed,
            accepted_cameras,
            format!("V{version} {detail}"),
        );
        if solver_stopped {
            return Err("证据已入库，但求解线程已退出".to_owned());
        }
        if max_groups.is_some_and(|limit| version >= limit) {
            candidates.close();
        }
    }
    if let Ok(mut state) = shared.rig.lock() {
        state
            .capture_message
            .push_str("；采集已停止，正在完成最新版本");
    }
    Ok(())
}

/// Live and journal replay share the same sessions, refinement, rig solve, and export gate.
struct RigSolver<'a> {
    config: &'a Config,
    model: ModelKind,
    sessions: Vec<Session>,
    records: Vec<ObservationRecord>,
    edges: Vec<(String, String)>,
}

impl<'a> RigSolver<'a> {
    fn new(config: &'a Config) -> Result<Self, String> {
        let extrinsics = config
            .extrinsics
            .as_ref()
            .ok_or("外参求解需要 extrinsics 段（≥2 路）")?;
        let model = config
            .solver
            .models
            .first()
            .copied()
            .unwrap_or(ModelKind::Kb4);
        let sessions = config
            .cameras
            .iter()
            .map(|_| {
                Ok(Session::new(
                    SessionOptions {
                        model,
                        image_size: config.image_size(),
                        cadence: 1,
                        max_solve_observations: config.solver.max_solve_observations,
                        holdout_fraction: config.solver.holdout_fraction,
                        min_holdout_views: config.solver.official_holdout_frames,
                    },
                    SessionThresholds::from_config(config),
                    backend_for(model, config)?,
                ))
            })
            .collect::<Result<_, String>>()?;
        Ok(Self {
            config,
            model,
            sessions,
            records: Vec::new(),
            edges: extrinsics
                .required_edges
                .iter()
                .map(|e| (e[0].clone(), e[1].clone()))
                .collect(),
        })
    }

    fn ingest(&mut self, camera_index: usize, record: ObservationRecord) {
        self.sessions[camera_index].ingest([record.observation.clone()]);
        self.records.push(record);
    }

    fn evaluate(&mut self, shared: &Shared, version: usize, refine: bool) -> Result<(), String> {
        let config = self.config;
        let extrinsics = config
            .extrinsics
            .as_ref()
            .ok_or("外参求解需要 extrinsics 段（≥2 路）")?;
        let started = Instant::now();
        if let Ok(mut state) = shared.rig.lock() {
            state.solving_version = Some(version);
            if refine {
                state.finalizing = true;
                state.export_requested = true;
                state.export_failed = false;
                state.phase = "REFINING".to_owned();
                state.export_message =
                    format!("V{version} 全量精修中；holdout 保持独立，不回流优化");
            }
        }
        let mut states = BTreeMap::new();
        for (camera, session) in config.cameras.iter().zip(&mut self.sessions) {
            let state = if refine {
                session.refine()
            } else {
                session.update(true)
            }
            .state;
            log_line(format!(
                "{} V{version} {} views={} used={} excl={} holdout={}：{}",
                camera.id,
                if refine {
                    "全量精修"
                } else {
                    "在线求解"
                },
                state.views,
                state.used_views,
                state.excluded_views,
                state.holdout_views,
                state.detail,
            ));
            states.insert(camera.id.clone(), state);
        }
        let mut complete = None;
        let mut error = None;
        if self.sessions.iter().all(Session::has_current_solution) {
            let parameters = config
                .cameras
                .iter()
                .zip(&self.sessions)
                .map(|(camera, session)| (camera.id.clone(), *session.parameters()))
                .collect();
            match solve_rig(
                &self.records,
                &parameters,
                self.model,
                &self.edges,
                &extrinsics.required_cycles,
                extrinsics.min_groups_per_edge,
            ) {
                Ok(estimate) => {
                    let connected =
                        reference_transforms(&estimate, "cam0").is_ok_and(|transforms| {
                            config
                                .cameras
                                .iter()
                                .all(|camera| transforms.contains_key(&camera.id))
                        });
                    if connected && estimate.cycles.len() == extrinsics.required_cycles.len() {
                        complete = Some(Arc::new(CalibrationSnapshot {
                            states: states.clone(),
                            estimate,
                            groups: version,
                        }));
                    } else {
                        error = Some("外参图未覆盖全部相机或必需环未计算".to_owned());
                    }
                }
                Err(reason) => error = Some(format!("外参：{reason}")),
            }
        } else {
            error = Some(
                config
                    .cameras
                    .iter()
                    .zip(&self.sessions)
                    .filter(|(_, session)| !session.has_current_solution())
                    .map(|(camera, _)| {
                        format!("{}：{}", camera.id, states[&camera.id].detail)
                    })
                    .collect::<Vec<_>>()
                    .join("；"),
            );
        }
        let result = error.clone().map_or(Ok(()), Err);
        publish_result(shared, self.config, version, &states, complete, error);
        if refine && let Ok(mut state) = shared.rig.lock() {
            state.finalizing = false;
            state.finalized = true;
            state.export_ready = result.is_ok();
            state.phase = if result.is_ok() {
                "REFINED"
            } else {
                "REFINE_FAILED"
            }
            .to_owned();
            if let Err(reason) = &result {
                state.export_requested = false;
                state.export_failed = true;
                state.export_message = format!("V{version} 全量精修失败，未导出旧解：{reason}");
            }
        }
        log_line(format!(
            "{} V{version} 完成 {:.1} ms，{}",
            if refine {
                "全量精修"
            } else {
                "求解快照"
            },
            started.elapsed().as_secs_f64() * 1000.0,
            if result.is_ok() {
                "完整"
            } else {
                "未生成完整结果"
            },
        ));
        result
    }
}

fn solve_worker(
    config: &Config,
    shared: &Shared,
    datasets: &Latest<Dataset>,
) -> Result<(), String> {
    let mut solver = RigSolver::new(config)?;
    let mut version = 0;
    while let Some(dataset) = datasets.recv() {
        for view in &dataset.views[solver.records.len()..] {
            solver.ingest(view.camera_index, view.record.clone());
        }
        version = dataset.version;
        // Online failures are diagnostics, not a reason to discard later evidence.
        let _ = solver.evaluate(shared, version, false);
    }
    // Even an unchanged prefix must bypass the online representative-view cap.
    solver.evaluate(shared, version, true)
}

pub fn replay(path: &Path, output: Option<&Path>) -> Result<(), String> {
    let (mut config, groups) =
        observations::read_observations(path).map_err(|error| error.to_string())?;
    if let Some(output) = output {
        config.output.root = output.to_string_lossy().into_owned();
    }
    let cameras = &config.cameras;
    if cameras.is_empty() {
        return Err("观测会话配置缺少 cameras（无法确定内参会话）".to_owned());
    }
    let shared = Shared {
        tiles: cameras
            .iter()
            .map(|_| Mutex::new(crate::TileState::default()))
            .collect(),
        rig: Mutex::new(crate::RigState::default()),
        snapshot: Mutex::new(None),
    };
    let mut solver = RigSolver::new(&config)?;
    let mut version = 0;
    for group in groups {
        version = group.version;
        for view in group.views {
            let index = cameras
                .iter()
                .position(|camera| camera.id == view.camera_id)
                .ok_or_else(|| format!("观测中的相机 {} 不在配置中", view.camera_id))?;
            solver.ingest(
                index,
                ObservationRecord {
                    camera_id: view.camera_id,
                    group_id: group.group_id,
                    observation: view.observation,
                },
            );
        }
    }
    log_line(format!(
        "离线重放 {}：V{version}，{} 路观测",
        path.display(),
        solver.records.len()
    ));
    solver.evaluate(&shared, version, true)?;
    let snapshot = shared.snapshot.lock().unwrap_or_else(|e| e.into_inner());
    // 重放的观测日志就是产出该快照的会话文件，直接记进 info.yaml。
    save_calibration(snapshot.as_deref(), &config, Some(path), &shared, &mut None)
}

fn publish_result(
    shared: &Shared,
    config: &Config,
    version: usize,
    states: &BTreeMap<String, SessionState>,
    complete: Option<Arc<CalibrationSnapshot>>,
    error: Option<String>,
) {
    let extrinsics = config.extrinsics.as_ref().expect("validated extrinsics");
    // GUI takes the same lock before reading all metric rows: never display new intrinsics
    // beside old extrinsics. An incomplete evaluation cannot replace a published result.
    let mut state = shared.rig.lock().unwrap_or_else(|e| e.into_inner());
    state.solved_version = Some(version);
    state.solving_version = None;
    state.solve_error = error.as_ref().map(|error| format!("V{version}：{error}"));
    state.message = error.unwrap_or_else(|| format!("完整内外参 V{version} 已发布"));
    let preserve_metrics = complete.is_none() && state.complete_version.is_some();
    for (index, camera) in config.cameras.iter().enumerate() {
        if let Some(result) = states.get(&camera.id)
            && let Ok(mut tile) = shared.tiles[index].lock()
        {
            tile.solve_detail = format!("诊断 V{version} [{}]：{}", result.status, result.detail);
            if preserve_metrics {
                continue;
            }
            tile.metrics = metrics_of(result, SessionThresholds::from_config(config));
            tile.note = format!(
                " 指标V{version}/{}张 used={} excl={} holdout={}",
                result.views, result.used_views, result.excluded_views, result.holdout_views
            );
        }
    }
    if preserve_metrics {
        return;
    }
    // Completeness is not quality approval: only the shared export gate declares VALIDATED.
    state.phase = if complete.is_some() {
        "SOLVED"
    } else {
        "TRAIN"
    }
    .to_owned();
    if let Some(snapshot) = complete {
        state.rows.clear();
        for edge in &snapshot.estimate.edges {
            state.rows.push((
                format!("V{version} 边 {}-{} rms", edge.camera_a, edge.camera_b),
                edge.reprojection_rms_px,
                extrinsics.max_edge_rms_px,
            ));
        }
        for cycle in &snapshot.estimate.cycles {
            state.rows.push((
                format!("V{version} 环 {} rot", cycle.cameras.join("-")),
                cycle.rotation_error_deg,
                extrinsics.max_cycle_rotation_deg,
            ));
            state.rows.push((
                format!("V{version} 环 {} trans", cycle.cameras.join("-")),
                cycle.translation_error_mm,
                extrinsics.max_cycle_translation_mm,
            ));
        }
        let mut published = shared.snapshot.lock().unwrap_or_else(|e| e.into_inner());
        // The single numerical worker processes immutable, strictly increasing prefixes.
        *published = Some(snapshot);
        state.export_ready = true;
        state.complete_version = Some(version);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn replacing_a_pending_candidate_reports_rejection_instead_of_replaying_it() {
        let candidates = Latest::new();
        let (tx, rx) = mpsc::channel();
        let make = |sequence| Candidate {
            key: FrameKey {
                source_epoch: 1,
                camera_index: 0,
                sequence,
                pts_ns: sequence as i64,
            },
            target_board_ns: sequence,
            clock_revision: 1,
            frame_period_ns: 16_666_667,
            created_at: Instant::now(),
            pose: PoseMeasurement {
                rvec: [0.0; 3],
                tvec: [0.0, 0.0, 1.0],
                rms_px: 0.1,
                tag_count: 36,
            },
            reply: tx.clone(),
        };
        candidates.submit(make(1));
        candidates.submit(make(2));
        let rejected = rx.recv().unwrap();
        assert_eq!(
            (rejected.key.sequence, rejected.status),
            (1, CaptureStatus::Rejected)
        );
        assert_eq!(candidates.recv().unwrap().key.sequence, 2);
        candidates.close();
        candidates.submit(make(3));
        assert_eq!(rx.recv().unwrap().status, CaptureStatus::Rejected);
    }
}
