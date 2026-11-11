use std::collections::BTreeMap;
use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, mpsc};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use anyhow::Result;
use ccl_calib::camera::{JointSolveProgress, SolveActivity, SolvePhase};

use crate::config::{RunSettings, TargetConfig};
use crate::detection::DetectionSummary;
use crate::input::{BagInspection, DetectionProgress};
use crate::run_record::{RunContext, RunStatus, atomic_write};
use crate::runner::{self, Event, SolveSummary};

enum CalibrationMessage {
    Event(Instant, Event),
    Finished(Instant, Result<(), String>),
}

struct CalibrationWorker {
    cancel: Arc<AtomicBool>,
    receiver: mpsc::Receiver<CalibrationMessage>,
    handle: Option<JoinHandle<()>>,
}

impl CalibrationWorker {
    fn start(settings: RunSettings, run: RunContext) -> Self {
        let cancel = Arc::new(AtomicBool::new(false));
        let worker_cancel = Arc::clone(&cancel);
        let (sender, receiver) = mpsc::sync_channel(4_096);
        let handle = thread::spawn(move || {
            let cancelled = || worker_cancel.load(Ordering::Relaxed);
            let mut emit = |event| {
                let _ = sender.send(CalibrationMessage::Event(Instant::now(), event));
            };
            let outcome = runner::execute(&settings, &run, &cancelled, &mut emit)
                .map(|_| ())
                .map_err(|error| format!("{error:#}"));
            let _ = sender.send(CalibrationMessage::Finished(Instant::now(), outcome));
        });
        Self {
            cancel,
            receiver,
            handle: Some(handle),
        }
    }

    fn join(&mut self) -> Result<()> {
        if let Some(handle) = self.handle.take() {
            handle
                .join()
                .map_err(|_| anyhow::anyhow!("calibration worker panicked"))?;
        }
        Ok(())
    }
}

/// Receive-side solver history. Event timestamps are captured in the worker
/// before the message is queued, so buffered bursts do not collapse to one time.
#[derive(Debug, Default)]
pub struct SolverActivityView {
    pub activity: Option<SolveActivity>,
    pub activity_at: Option<Instant>,
    pub phase_started_at: Option<Instant>,
    pub last_progress: Option<JointSolveProgress>,
    pub last_progress_at: Option<Instant>,
    pub current_iteration_started_at: Option<Instant>,
    pub recent_iteration_durations: Vec<Duration>,
}

impl SolverActivityView {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn on_activity(&mut self, activity: SolveActivity, at: Instant) {
        if self
            .activity
            .is_none_or(|previous| previous.phase != activity.phase)
        {
            self.phase_started_at = Some(at);
        }
        self.activity = Some(activity);
        self.activity_at = Some(at);
    }

    /// Returns true when this snapshot starts a new run-local history.
    pub fn on_progress(&mut self, progress: JointSolveProgress, at: Instant) -> bool {
        let new_run = self.last_progress.is_none_or(|previous| {
            progress.iteration == 0 || progress.iteration < previous.iteration
        });
        if new_run {
            self.recent_iteration_durations.clear();
            self.current_iteration_started_at = Some(at);
        } else if let Some(previous) = self.last_progress
            && progress.iteration > previous.iteration
        {
            if progress.iteration == previous.iteration + 1
                && let Some(started) = self.current_iteration_started_at
            {
                self.recent_iteration_durations
                    .push(at.saturating_duration_since(started));
                if self.recent_iteration_durations.len() > 8 {
                    self.recent_iteration_durations.remove(0);
                }
            }
            self.current_iteration_started_at = Some(at);
        }
        self.last_progress = Some(progress);
        self.last_progress_at = Some(at);
        new_run
    }

    #[must_use]
    pub fn phase_elapsed(&self, at: Instant) -> Option<Duration> {
        self.phase_started_at
            .map(|started| at.saturating_duration_since(started))
    }

    #[must_use]
    pub fn update_age(&self, at: Instant) -> Option<Duration> {
        self.activity_at
            .map(|updated| at.saturating_duration_since(updated))
    }

    #[must_use]
    pub fn last_progress_age(&self, at: Instant) -> Option<Duration> {
        self.last_progress_at
            .map(|updated| at.saturating_duration_since(updated))
    }
}

/// Which dashboard graph is focused. Both graphs stay available: wide terminals
/// show them side by side and medium terminals stack them; only the smallest
/// terminals render the focused graph alone.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DashboardGraph {
    #[default]
    Detection,
    Convergence,
}

impl DashboardGraph {
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Detection => "检测历史",
            Self::Convergence => "LM 收敛",
        }
    }
}

/// One calibration task. The settings and inspection are frozen at start, so
/// editing the draft while a run is active cannot mutate the running snapshot.
pub struct Task {
    pub settings: RunSettings,
    pub inspection: Option<BagInspection>,
    pub run: RunContext,
    pub run_root: PathBuf,
    worker: Option<CalibrationWorker>,
    pub started_at: Instant,
    pub stage_started_at: Option<Instant>,
    pub finished_at: Option<Instant>,
    pub cancel_requested_at: Option<Instant>,
    pub state: String,
    pub run_stage: Option<&'static str>,
    pub messages: Vec<String>,
    pub dataset_summary: Option<DetectionSummary>,
    pub detection_progress: Vec<DetectionProgress>,
    pub detection_time_range_ns: Option<(i64, i64)>,
    pub detection_updated_at: Option<Instant>,
    pub solve_progress: Vec<JointSolveProgress>,
    pub solver_activity: SolverActivityView,
    pub solve_summary: Option<SolveSummary>,
    pub final_status: Option<RunStatus>,
    pub failure_stage: Option<String>,
    pub failure_error: Option<String>,
    dashboard_graph: DashboardGraph,
}

impl Task {
    #[must_use]
    pub fn start(
        settings: RunSettings,
        inspection: Option<BagInspection>,
        run: RunContext,
    ) -> Self {
        let run_root = run.root();
        let now = Instant::now();
        let mut messages = vec![format!("run={}", run_root.display())];
        messages.push(format!("mode={}", settings.mode_name()));
        let worker = Some(CalibrationWorker::start(settings.clone(), run.clone()));
        Self {
            settings,
            inspection,
            run,
            run_root,
            worker,
            started_at: now,
            stage_started_at: Some(now),
            finished_at: None,
            cancel_requested_at: None,
            state: "starting".to_owned(),
            run_stage: None,
            messages,
            dataset_summary: None,
            detection_progress: Vec::new(),
            detection_time_range_ns: None,
            detection_updated_at: None,
            solve_progress: Vec::new(),
            solver_activity: SolverActivityView::default(),
            solve_summary: None,
            final_status: None,
            failure_stage: None,
            failure_error: None,
            dashboard_graph: DashboardGraph::default(),
        }
    }

    #[must_use]
    pub fn is_running(&self) -> bool {
        self.worker.is_some()
    }

    #[must_use]
    pub const fn dashboard_graph(&self) -> DashboardGraph {
        self.dashboard_graph
    }

    /// Cycle the dashboard graph focus, returning the newly focused graph.
    pub fn cycle_dashboard_graph(&mut self) -> DashboardGraph {
        self.dashboard_graph = match self.dashboard_graph {
            DashboardGraph::Detection => DashboardGraph::Convergence,
            DashboardGraph::Convergence => DashboardGraph::Detection,
        };
        self.dashboard_graph
    }

    /// Focus a dashboard graph directly, used by mouse selection.
    pub fn select_dashboard_graph(&mut self, graph: DashboardGraph) {
        self.dashboard_graph = graph;
    }

    pub fn request_cancel(&mut self) {
        if let Some(worker) = &self.worker {
            worker.cancel.store(true, Ordering::Relaxed);
            self.cancel_requested_at.get_or_insert_with(Instant::now);
            "cancelling".clone_into(&mut self.state);
        }
    }

    /// Returns true exactly once, when the worker has finished and the result
    /// view should be shown.
    pub fn poll(&mut self) -> bool {
        // `runner::execute` calls `RunContext::set_mode`, which renames the run
        // directory; refresh the display/artifact root on every poll.
        self.run_root = self.run.root();
        if let Some(first) = self.messages.first_mut()
            && first.starts_with("run=")
        {
            *first = format!("run={}", self.run_root.display());
        }
        let mut finished = None;
        let mut events = Vec::new();
        if let Some(worker) = &mut self.worker {
            loop {
                match worker.receiver.try_recv() {
                    Ok(CalibrationMessage::Event(at, event)) => events.push((at, event)),
                    Ok(CalibrationMessage::Finished(at, result)) => {
                        finished = Some((at, result));
                        break;
                    }
                    Err(mpsc::TryRecvError::Disconnected) => {
                        finished = Some((
                            Instant::now(),
                            Err(
                                "calibration worker disconnected without reporting an outcome"
                                    .to_owned(),
                            ),
                        ));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                }
            }
        }
        for (at, event) in events {
            apply_event(self, event, at);
        }
        if let Some((at, result)) = finished {
            self.finish_after_worker(at, result);
            true
        } else {
            false
        }
    }

    fn finish_after_worker(&mut self, at: Instant, outcome: Result<(), String>) {
        self.run_root = self.run.root();
        let Some(mut worker) = self.worker.take() else {
            return;
        };
        let cancelled = worker.cancel.load(Ordering::Relaxed);
        if let Err(error) = worker.join() {
            self.messages.push(format!("{error:#}"));
        }
        self.finished_at = Some(at);
        self.cancel_requested_at = None;
        match outcome {
            Ok(()) => {
                "complete".clone_into(&mut self.state);
                self.final_status = Some(RunStatus::Complete);
                self.record_finish(RunStatus::Complete, None);
            }
            Err(error) if cancelled => {
                "cancelled".clone_into(&mut self.state);
                self.messages.push(error);
                self.final_status = Some(RunStatus::Cancelled);
                self.record_finish(RunStatus::Cancelled, Some("user"));
            }
            Err(error) => {
                self.failure_stage = Some(
                    self.run_stage
                        .map_or_else(|| self.state.clone(), |stage| stage_label(stage).to_owned()),
                );
                self.failure_error = Some(error.clone());
                "failed".clone_into(&mut self.state);
                self.messages.push(error.clone());
                self.final_status = Some(RunStatus::Failed);
                if let Err(log_error) = self.run.log("ERROR", &error) {
                    self.messages
                        .push(format!("记录错误日志失败: {log_error:#}"));
                }
                self.record_finish(RunStatus::Failed, Some("calibration"));
            }
        }
        let report = calibration_report_text(self);
        if let Err(error) = atomic_write(&self.run_root.join("report.txt"), report.as_bytes()) {
            self.messages
                .push(format!("写入 report.txt 失败: {error:#}"));
        }
    }

    fn record_finish(&mut self, status: RunStatus, category: Option<&str>) {
        if let Err(error) = self.run.finish(status, category) {
            self.messages.push(format!("记录运行状态失败: {error:#}"));
        }
    }

    #[must_use]
    pub fn run_elapsed_at(&self, at: Instant) -> Duration {
        self.finished_at
            .unwrap_or(at)
            .saturating_duration_since(self.started_at)
    }

    /// Terminal runs stop their clocks at the moment the worker reported back.
    #[must_use]
    pub fn effective_now(&self, at: Instant) -> Instant {
        self.finished_at.unwrap_or(at)
    }

    #[must_use]
    pub fn current_stage(&self) -> &str {
        self.run_stage.unwrap_or(&self.state)
    }
}

fn apply_event(task: &mut Task, event: Event, at: Instant) {
    match event {
        Event::Stage(stage) => {
            if task.run_stage != Some(stage) {
                task.run_stage = Some(stage);
                task.stage_started_at = Some(at);
                if stage.starts_with("solve") {
                    task.solver_activity.reset();
                    task.solve_progress.clear();
                }
            }
            if task.cancel_requested_at.is_none() {
                stage_label(stage).clone_into(&mut task.state);
            }
            task.messages.push(format!("stage: {stage}"));
        }
        Event::DetectionProgress(progress) => {
            task.detection_updated_at = Some(at);
            if task.cancel_requested_at.is_none() {
                task.state = format!("scan_detect {:.1} frames/s", progress.frames_per_second);
            }
            let timestamp = progress.timestamp_ns;
            task.detection_time_range_ns = Some(
                task.detection_time_range_ns
                    .map_or((timestamp, timestamp), |(start, end)| {
                        (start.min(timestamp), end.max(timestamp))
                    }),
            );
            task.detection_progress.push(progress);
        }
        Event::Dataset(summary) => task.dataset_summary = Some(summary),
        Event::SolveProgress(snapshot) => {
            let new_run = task.solver_activity.on_progress(snapshot, at);
            if new_run {
                task.solve_progress.clear();
            }
            task.solve_progress.push(snapshot);
        }
        Event::SolveActivity(activity) => {
            task.solver_activity.on_activity(activity, at);
            if task.cancel_requested_at.is_none()
                && let Some(stage) = task.run_stage
                && stage.starts_with("solve")
            {
                task.state = format!(
                    "{} · {}",
                    stage_label(stage),
                    solve_phase_label(activity.phase)
                );
            }
        }
        Event::SolveSummary(summary) => task.solve_summary = Some(summary),
        Event::Message(message) => task.messages.push(message),
    }
}

#[must_use]
pub fn stage_label(stage: &str) -> &str {
    match stage {
        "scan_detect" => "扫描与检测",
        "solve" => "联合求解",
        "solve_intrinsics" => "内参初始化",
        "solve_preintegration" => "预积分求解",
        "solve_visual_inertial_seed" => "视觉/陀螺初值与四次样条初始化",
        "solve_continuous_time" => "连续时间求解",
        "solve_refine_imu_axes" => "IMU 轴参数细化",
        "solve_refine_time_offset" => "相机-IMU 时间偏移细化",
        "solve_refine_imu_axes_time_offset" => "IMU 轴参数与时间偏移联合细化",
        "solve_refine_continuous_time_parameters" => "连续时间时钟/轴参数联合细化",
        "export" => "导出结果",
        other => other,
    }
}

#[must_use]
pub fn solve_phase_label(phase: SolvePhase) -> &'static str {
    match phase {
        SolvePhase::PoseInitialization => "位姿初始化",
        SolvePhase::GyroAlignment => "陀螺对齐",
        SolvePhase::MotionAssembly => "IMU 预积分/运动方程装配",
        SolvePhase::MotionSolve => "速度与重力初始化（结构化 QR）",
        SolvePhase::Linearization => "线性化",
        SolvePhase::LinearSolve => "线性求解",
        SolvePhase::CandidateEvaluation => "候选评估",
        SolvePhase::Information => "信息矩阵",
        SolvePhase::ResidualEvaluation => "残差评估",
    }
}

#[must_use]
pub fn format_duration(duration: Duration) -> String {
    let seconds = duration.as_secs();
    let hours = seconds / 3_600;
    let minutes = (seconds % 3_600) / 60;
    let seconds = seconds % 60;
    format!("{hours:02}:{minutes:02}:{seconds:02}")
}

#[must_use]
pub fn format_short_duration(duration: Duration) -> String {
    let seconds = duration.as_secs_f64();
    if seconds < 1.0 {
        format!("{:.2}ms", seconds * 1_000.0)
    } else if seconds < 10.0 {
        format!("{seconds:.2}s")
    } else if seconds < 60.0 {
        format!("{seconds:.1}s")
    } else {
        format_duration(duration)
    }
}

fn count_to_f64(value: u64) -> f64 {
    let upper = u32::try_from(value >> 32).expect("shifted u64 fits u32");
    let lower = u32::try_from(value & u64::from(u32::MAX)).expect("masked u64 fits u32");
    f64::from(upper) * 4_294_967_296.0 + f64::from(lower)
}

/// Frames already processed by the detector and the total exposed by the bag
/// inspection for the configured camera topics. Both counts are facts from the
/// input layer; no frame total is invented when a topic was not inspected.
#[must_use]
pub fn detection_frame_progress(task: &Task) -> Option<(usize, usize)> {
    let inspection = task.inspection.as_ref()?;
    let mut total_frames = 0_u64;
    for camera in &task.settings.cameras {
        let topic = inspection
            .topics
            .iter()
            .find(|topic| topic.name == camera.topic)?;
        total_frames = total_frames.checked_add(topic.message_count)?;
    }
    let mut processed_by_camera = BTreeMap::new();
    for progress in &task.detection_progress {
        processed_by_camera.insert(progress.camera_id.as_str(), progress.frames_processed);
    }
    let processed = processed_by_camera.values().copied().sum::<usize>();
    Some((processed, usize::try_from(total_frames).ok()?))
}

#[must_use]
pub fn detection_remaining(task: &Task) -> Option<Duration> {
    let (processed, total_frames) = detection_frame_progress(task)?;
    let average_fps = task.detection_progress.last()?.average_frames_per_second;
    let remaining_frames = total_frames.saturating_sub(processed);
    if average_fps.is_finite() && average_fps > 0.0 {
        Some(Duration::from_secs_f64(
            count_to_f64(u64::try_from(remaining_frames).ok()?) / average_fps,
        ))
    } else {
        None
    }
}

#[must_use]
pub fn estimated_remaining(task: &Task) -> Option<Duration> {
    if task.cancel_requested_at.is_some() || task.finished_at.is_some() {
        return None;
    }
    if task.current_stage() == "scan_detect" {
        detection_remaining(task)
    } else {
        None
    }
}

#[must_use]
pub fn run_timing_text(task: &Task) -> String {
    let now = Instant::now();
    let elapsed = format_duration(task.run_elapsed_at(now));
    let remaining = if task.finished_at.is_some() {
        "—".to_owned()
    } else if task.cancel_requested_at.is_some() {
        "取消中（等待当前任务退出）".to_owned()
    } else if task.current_stage() == "scan_detect" {
        estimated_remaining(task).map_or_else(
            || "未知（尚无可用平均检测吞吐）".to_owned(),
            |remaining| {
                let updated = task.detection_updated_at.map_or_else(
                    || "未上报".to_owned(),
                    |updated| {
                        format!(
                            "{} 前",
                            format_short_duration(now.saturating_duration_since(updated))
                        )
                    },
                );
                format!(
                    "{}（按检测平均吞吐估算，检测更新 {updated}）",
                    format_duration(remaining)
                )
            },
        )
    } else if task.current_stage().starts_with("solve") {
        "未知（求解器不按迭代上限或收敛速度猜测）".to_owned()
    } else {
        "计算中".to_owned()
    };
    format!("当前耗时: {elapsed}  预计剩余: {remaining}")
}

#[must_use]
#[allow(
    clippy::cast_precision_loss,
    reason = "activity counters are small display-only values"
)]
pub fn known_work_text(activity: SolveActivity) -> Option<String> {
    match (activity.completed, activity.total) {
        (Some(completed), Some(total)) if total > 0 => Some(format!(
            "{completed}/{total} ({:.1}%，仅当前活动)",
            100.0 * completed as f64 / total as f64
        )),
        (Some(completed), Some(total)) => Some(format!("{completed}/{total}")),
        (Some(completed), None) => Some(format!("已完成 {completed}")),
        (None, Some(total)) => Some(format!("总量 {total}")),
        (None, None) => None,
    }
}

#[must_use]
pub fn lm_snapshot_state(task: &Task) -> (&'static str, &'static str) {
    match task.solver_activity.activity {
        Some(activity)
            if task.finished_at.is_none()
                && activity.iteration.is_some()
                && matches!(
                    activity.phase,
                    SolvePhase::Linearization
                        | SolvePhase::LinearSolve
                        | SolvePhase::CandidateEvaluation
                ) =>
        {
            ("当前", "")
        }
        Some(_) => ("上次快照", "（当前不在 LM 迭代，仅显示最后快照）"),
        None => ("快照", "（无活动上报，无法判断当前是否仍在迭代）"),
    }
}

#[must_use]
pub fn solver_activity_text(task: &Task, now: Instant) -> String {
    let at = task.effective_now(now);
    let stage = task
        .run_stage
        .map_or("等待启动", |stage| stage_label(stage));
    let stage_elapsed = task.stage_started_at.map_or_else(
        || "--".to_owned(),
        |started| format_short_duration(at.saturating_duration_since(started)),
    );
    let run_state = if task.finished_at.is_some() {
        "已结束"
    } else if task.cancel_requested_at.is_some() {
        "取消中"
    } else {
        "运行中"
    };
    let mut lines = vec![format!(
        "阶段: {stage}  阶段用时 {stage_elapsed}  {run_state}"
    )];

    if let Some(activity) = task.solver_activity.activity {
        let phase = solve_phase_label(activity.phase);
        let phase_elapsed = task
            .solver_activity
            .phase_elapsed(at)
            .map_or_else(|| "--".to_owned(), format_short_duration);
        let update_age = task
            .solver_activity
            .update_age(at)
            .map_or_else(|| "--".to_owned(), format_short_duration);
        let mut line = format!("活动: {phase}  本阶段 {phase_elapsed}  更新 {update_age} 前");
        if let Some(iteration) = activity.iteration {
            let _ = write!(line, "  LM iter {iteration}");
        }
        if activity.cancellable {
            line.push_str("  可取消");
        } else {
            line.push_str("  不可中断（当前内核，取消需等待返回）");
        }
        lines.push(line);
        lines.push(known_work_text(activity).map_or_else(
            || "已知工作量: 未上报（当前内核无可靠进度单位）".to_owned(),
            |counts| format!("已知工作量: {counts}"),
        ));
    } else {
        lines.push("活动: 尚未收到求解器活动上报（不显示推测进度）".to_owned());
    }

    if let Some(last) = task.solve_progress.last() {
        let (label, note) = lm_snapshot_state(task);
        let age = task
            .solver_activity
            .last_progress_age(at)
            .map_or_else(|| "--".to_owned(), format_short_duration);
        lines.push(format!(
            "LM {label}: iter={}/{} cost={:.3e} λ={:.2e} |g|∞={:.2e}  快照 {age} 前{note}",
            last.iteration,
            last.max_iterations,
            last.cost,
            last.damping,
            last.scaled_gradient_inf_norm,
        ));
    } else {
        lines.push("LM: 尚无迭代快照（初始化/装配/线性求解中）".to_owned());
    }

    let durations = &task.solver_activity.recent_iteration_durations;
    if durations.is_empty() {
        lines.push("已完成迭代耗时: 尚无（不推算收敛剩余时间）".to_owned());
    } else {
        let start = durations.len().saturating_sub(6);
        let recent = durations[start..]
            .iter()
            .map(|duration| format_short_duration(*duration))
            .collect::<Vec<_>>();
        lines.push(format!(
            "已完成迭代耗时（最近 {} 次，旧→新）: {}",
            recent.len(),
            recent.join("  "),
        ));
    }

    if let Some(requested) = task
        .cancel_requested_at
        .filter(|_| task.finished_at.is_none())
    {
        let waited = format_short_duration(at.saturating_duration_since(requested));
        let (phase, interruption) =
            task.solver_activity
                .activity
                .map_or(("未知活动", "状态未知"), |activity| {
                    (
                        solve_phase_label(activity.phase),
                        if activity.cancellable {
                            "可取消检查点"
                        } else {
                            "不可中断内核，取消需等待其返回"
                        },
                    )
                });
        lines.push(format!(
            "取消: 已请求 {waited} 前，等待 worker 退出；当前 {phase}（{interruption}）"
        ));
    }
    lines.join("\n")
}

#[must_use]
pub fn dataset_summary_text(summary: Option<&DetectionSummary>) -> String {
    let Some(summary) = summary else {
        return "正在读取消息、同步视图并检测 AprilGrid…".to_owned();
    };
    let mut lines = vec![format!(
        "同步视图: {}  可用: {}  忽略标签: {}  角点细化失败: {}",
        summary.input_views,
        summary.usable_views,
        summary.ignored_tags,
        summary.refinement_failures
    )];
    for (camera, frames) in &summary.usable_frames_by_camera {
        lines.push(format!(
            "{camera}: 帧={frames}  标签={}  角点={}",
            summary.tags_by_camera.get(camera).copied().unwrap_or(0),
            summary.points_by_camera.get(camera).copied().unwrap_or(0)
        ));
    }
    lines.join("\n")
}

#[derive(Debug)]
pub struct DetectionChartData {
    pub series: BTreeMap<String, Vec<(f64, f64)>>,
    pub x_max: f64,
    pub y_max: f64,
}

#[must_use]
pub fn detection_chart_series(
    progress: &[DetectionProgress],
    (origin, latest): (i64, i64),
) -> DetectionChartData {
    let mut series = BTreeMap::<String, Vec<(f64, f64)>>::new();
    let x_max = Duration::from_nanos(latest.abs_diff(origin)).as_secs_f64();
    let mut y_max = 1.0_f64;
    for sample in progress {
        let elapsed_ns = sample.timestamp_ns.saturating_sub(origin).cast_unsigned();
        let x = Duration::from_nanos(elapsed_ns).as_secs_f64();
        let y = f64::from(u32::try_from(sample.points).unwrap_or(u32::MAX));
        y_max = y_max.max(y);
        series
            .entry(sample.camera_id.clone())
            .or_default()
            .push((x, y));
    }
    for points in series.values_mut() {
        points.sort_by(|left, right| left.0.total_cmp(&right.0));
    }
    DetectionChartData {
        series,
        x_max: x_max.max(0.001),
        y_max,
    }
}

/// Facts for one row of the live detection table. Counts come straight from
/// detection progress and the completed detection summary; a missing fact stays
/// `None` instead of being replaced by a guess.
#[derive(Clone, Debug)]
pub struct CameraDetectionRow {
    pub camera: String,
    pub processed: Option<usize>,
    pub detected_frames: Option<usize>,
    pub corners: Option<usize>,
    pub corners_are_cumulative: bool,
    pub frames_per_second: Option<f64>,
    pub average_frames_per_second: Option<f64>,
    pub workers: Option<usize>,
}

#[must_use]
pub fn camera_detection_rows(task: &Task) -> Vec<CameraDetectionRow> {
    let mut latest = BTreeMap::<&str, &DetectionProgress>::new();
    for progress in &task.detection_progress {
        latest.insert(progress.camera_id.as_str(), progress);
    }
    task.settings
        .cameras
        .iter()
        .map(|camera| {
            let sample = latest.get(camera.id.as_str()).copied();
            let summary = task.dataset_summary.as_ref();
            CameraDetectionRow {
                camera: camera.id.clone(),
                processed: sample.map(|sample| sample.frames_processed),
                detected_frames: summary
                    .and_then(|summary| summary.usable_frames_by_camera.get(&camera.id).copied()),
                corners: summary.map_or_else(
                    || sample.map(|sample| sample.points),
                    |summary| summary.points_by_camera.get(&camera.id).copied(),
                ),
                corners_are_cumulative: summary.is_some(),
                frames_per_second: sample.map(|sample| sample.frames_per_second),
                average_frames_per_second: sample.map(|sample| sample.average_frames_per_second),
                workers: sample.map(|sample| sample.workers),
            }
        })
        .collect()
}

#[must_use]
pub fn board_corner_limit(settings: &RunSettings) -> f64 {
    let TargetConfig::Aprilgrid {
        id_rows_front_view_left_to_right,
        ..
    } = &settings.target;
    count_to_f64(
        id_rows_front_view_left_to_right
            .iter()
            .map(Vec::len)
            .sum::<usize>() as u64,
    ) * 4.0
}

/// Actual LM snapshots for the current solver stage, plotted as
/// `(iteration, log10(cost))`. The task clears this history when the runner
/// enters another solve stage, so a line never joins unrelated stages.
#[derive(Clone, Debug)]
pub struct ConvergenceChartData {
    pub points: Vec<(f64, f64)>,
    pub y_min: f64,
    pub y_max: f64,
}

#[must_use]
pub fn convergence_chart_data(progress: &[JointSolveProgress]) -> Option<ConvergenceChartData> {
    if progress.is_empty() {
        return None;
    }
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    let points = progress
        .iter()
        .map(|snapshot| {
            let y = snapshot.cost.max(f64::MIN_POSITIVE).log10();
            y_min = y_min.min(y);
            y_max = y_max.max(y);
            let iteration =
                u32::try_from(snapshot.iteration).map_or(f64::from(u32::MAX), f64::from);
            (iteration, y)
        })
        .collect::<Vec<_>>();
    if !(y_min.is_finite() && y_max.is_finite()) {
        return None;
    }
    if (y_max - y_min) < 1.0e-9 {
        y_min -= 0.5;
        y_max += 0.5;
    }
    Some(ConvergenceChartData {
        points,
        y_min,
        y_max,
    })
}

#[must_use]
pub fn settings_snapshot_text(settings: &RunSettings) -> String {
    let (reference, intrinsic) = match &settings.calibration {
        crate::config::CalibrationConfig::SingleCamera {
            camera,
            initial_intrinsics,
        } => (
            camera.as_str(),
            initial_intrinsics.path.as_ref().map_or_else(
                || "自动+优化".to_owned(),
                |path| format!("文件+优化 ({})", path.display()),
            ),
        ),
        crate::config::CalibrationConfig::MultiCamera {
            reference_camera,
            refine_intrinsics,
            initial_intrinsics,
            ..
        } => (
            reference_camera.as_str(),
            match (initial_intrinsics.path.as_ref(), *refine_intrinsics) {
                (Some(path), true) => format!("文件+优化 ({})", path.display()),
                (Some(path), false) => format!("文件+固定 ({})", path.display()),
                (None, _) => "自动+优化".to_owned(),
            },
        ),
        crate::config::CalibrationConfig::MultiCameraImu {
            visual_reference_camera,
            initial_intrinsics,
            ..
        } => (
            visual_reference_camera.as_str(),
            initial_intrinsics.path.as_ref().map_or_else(
                || "自动+优化".to_owned(),
                |path| format!("文件+优化 ({})", path.display()),
            ),
        ),
    };
    format!(
        "配置快照: mode={}  相机={}  参考={}  bag={}  IMU={}  内参={}",
        settings.mode_name(),
        settings.cameras.len(),
        reference,
        settings.source.path.display(),
        settings.source.imu_topic.as_deref().unwrap_or("-"),
        intrinsic
    )
}

#[must_use]
#[allow(
    clippy::too_many_lines,
    reason = "single coherent human-readable result summary"
)]
pub fn result_summary_text(task: &Task) -> String {
    let status = match task.final_status {
        Some(RunStatus::Complete) => "成功",
        Some(RunStatus::Failed) => "失败",
        Some(RunStatus::Cancelled) => "已取消",
        Some(RunStatus::ClosedWithoutCalibration) => "未执行",
        Some(RunStatus::InProgress) | None => "进行中",
    };
    let mut lines = vec![
        format!("状态: {status}"),
        format!("模式: {}", task.settings.mode_name()),
        format!("运行目录: {}", task.run_root.display()),
    ];
    lines.push(format!(
        "用时: {}",
        format_duration(task.run_elapsed_at(Instant::now()))
    ));
    if let Some(stage) = &task.failure_stage {
        lines.push(format!("失败阶段: {stage}"));
    }
    if let Some(summary) = &task.dataset_summary {
        lines.push(format!(
            "检测: 同步视图={}  求解选用={}  忽略标签={}  细化失败={}",
            summary.input_views,
            summary.usable_views,
            summary.ignored_tags,
            summary.refinement_failures,
        ));
        for (camera, frames) in &summary.usable_frames_by_camera {
            lines.push(format!(
                "{camera}: 有效帧={frames}  tags={}  corners={}",
                summary.tags_by_camera.get(camera).copied().unwrap_or(0),
                summary.points_by_camera.get(camera).copied().unwrap_or(0),
            ));
        }
    }
    if let Some(progress) = task.detection_progress.last() {
        lines.push(format!(
            "检测吞吐: {:.1} frames/s（平均 {:.1}） workers={}",
            progress.frames_per_second, progress.average_frames_per_second, progress.workers,
        ));
    }
    if let Some(summary) = &task.solve_summary {
        let reduction = if summary.initial_cost > 0.0 {
            summary.final_cost / summary.initial_cost
        } else {
            f64::NAN
        };
        lines.push(format!(
            "求解终止: {}  数值收敛={}  迭代={}  cost {:.3e} → {:.3e} ({:.3e}×)",
            summary.termination,
            if summary.converged { "是" } else { "否" },
            summary.iterations,
            summary.initial_cost,
            summary.final_cost,
            reduction
        ));
        lines.push(format!(
            "信息矩阵: rank={}/{}  condition={:.3e}  covariance={}",
            summary.information_rank,
            summary.information_parameters,
            summary.condition_number,
            if summary.covariance_is_pseudoinverse {
                "伪逆"
            } else {
                "全秩逆"
            }
        ));
        lines.push(
            "验收状态: 候选结果已保存，待人工审查；应用不按收敛或条件数自动否决。".to_owned(),
        );
        for camera in &summary.camera_errors {
            lines.push(format!(
                "cam{}: RMS={:.3}px  P95={:.3}px  使用/剔除={}/{}",
                camera.camera_index,
                camera.rms_px,
                camera.p95_px,
                camera.used_points,
                camera.rejected_points
            ));
        }
        if let Some(rms) = summary.inertial_whitened_rms {
            lines.push(format!("IMU whitened RMS={rms:.3}"));
        }
    } else if let Some(last) = task.solve_progress.last() {
        lines.push(format!(
            "求解停止: iter={}/{}  cost={:.3e}  λ={:.3e}  |g|∞={:.3e}  接受/拒绝={}/{}",
            last.iteration,
            last.max_iterations,
            last.cost,
            last.damping,
            last.scaled_gradient_inf_norm,
            last.accepted_steps,
            last.rejected_steps,
        ));
    }
    if let Some(error) = &task.failure_error {
        lines.push(format!(
            "错误: {}",
            error.lines().next().unwrap_or("未知错误")
        ));
    }
    lines.join("\n")
}

#[must_use]
pub fn calibration_report_text(task: &Task) -> String {
    let mut report = result_summary_text(task);
    if let Some(error) = &task.failure_error {
        report.push_str("\n\n完整错误链:\n");
        report.push_str(error);
        report.push_str("\n\n诊断建议:\n");
        report.push_str(failure_guidance(error));
    }
    report.push_str("\n\n产物:\n");
    for (relative, path) in artifact_paths(&task.run_root) {
        let generated = relative == "report.txt" || path.exists();
        let _ = writeln!(
            report,
            "- {relative}: {}{}",
            path.display(),
            if generated { "" } else { "（未生成）" }
        );
    }
    report
}

#[must_use]
pub fn artifact_paths(root: &Path) -> Vec<(&'static str, PathBuf)> {
    [
        "run.yaml",
        "config.yaml",
        "input.yaml",
        "detection.yaml",
        "detection_progress.jsonl",
        "solve.yaml",
        "result/calibration.yaml",
        "result/extrinsics_3d.html",
        "result/trajectory.json",
        "result/summary.json",
        "report.txt",
        "run.log",
    ]
    .into_iter()
    .map(|relative| (relative, root.join(relative)))
    .collect()
}

#[must_use]
pub fn failure_guidance(error: &str) -> &'static str {
    if error.contains("rank-deficient")
        || error.contains("Unobservable")
        || error.contains("equations are singular")
    {
        "- 检测数量充足不等于参数可观测；需要覆盖不同距离、倾角、画面位置和多轴旋转。\n- 核对每个 topic 与实际相机编号、相机模型、AprilGrid 尺寸和角点方向。\n- 检查各相机在同一同步视图中是否同时看到足够目标，避免相机图连接不稳定。\n- 相机-IMU 模式还需要平移加速度和多轴角速度激励；纯静止、匀速或单轴旋转会被拒绝。"
    } else if error.contains("ProjectionDomain") || error.contains("projection domain") {
        "- 内参初值使观测落到模型定义域外；核对相机模型、图像尺寸、角点映射、初值和视场覆盖。"
    } else if error.contains("cancel") {
        "- 本次运行由用户取消；数据与检测产物仍可用于检查。"
    } else {
        "- 查看完整错误链、run.log、input.yaml 与 detection.yaml；先修复最内层错误，再重新运行。"
    }
}

#[must_use]
pub fn messages_text(task: &Task, limit: usize) -> String {
    task.messages
        .iter()
        .rev()
        .take(limit)
        .rev()
        .cloned()
        .collect::<Vec<_>>()
        .join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn progress(iteration: usize, max_iterations: usize) -> JointSolveProgress {
        JointSolveProgress {
            iteration,
            max_iterations,
            cost: 1.0,
            damping: 1e-3,
            scaled_gradient_inf_norm: 1e-2,
            accepted_steps: iteration,
            rejected_steps: 0,
        }
    }

    fn activity(phase: SolvePhase, iteration: Option<usize>, cancellable: bool) -> SolveActivity {
        SolveActivity {
            phase,
            completed: None,
            total: None,
            iteration,
            cancellable,
        }
    }

    #[test]
    fn solver_activity_tracks_completed_iterations_without_growing_during_waits() {
        let now = Instant::now();
        let mut view = SolverActivityView::default();
        assert!(view.on_progress(progress(0, 100), now));
        assert!(!view.on_progress(progress(1, 100), now + Duration::from_secs(2)));
        assert!(!view.on_progress(progress(2, 100), now + Duration::from_secs(5)));
        assert_eq!(
            view.recent_iteration_durations,
            [Duration::from_secs(2), Duration::from_secs(3)]
        );
        assert_eq!(
            view.last_progress_age(now + Duration::from_secs(30)),
            Some(Duration::from_secs(25))
        );
    }

    #[test]
    fn solver_activity_new_run_drops_previous_iteration_history() {
        let now = Instant::now();
        let mut view = SolverActivityView::default();
        view.on_progress(progress(0, 100), now);
        view.on_progress(progress(1, 100), now + Duration::from_secs(1));
        view.on_progress(progress(2, 100), now + Duration::from_secs(3));
        assert_eq!(view.recent_iteration_durations.len(), 2);

        assert!(view.on_progress(progress(0, 100), now + Duration::from_secs(10)));
        assert!(view.recent_iteration_durations.is_empty());
        assert_eq!(
            view.last_progress.map(|progress| progress.iteration),
            Some(0)
        );
    }

    #[test]
    fn solver_activity_phase_clock_restarts_only_on_phase_change() {
        let now = Instant::now();
        let mut view = SolverActivityView::default();
        view.on_activity(activity(SolvePhase::MotionAssembly, None, false), now);
        view.on_activity(
            activity(SolvePhase::MotionAssembly, None, false),
            now + Duration::from_secs(3),
        );
        assert_eq!(
            view.phase_elapsed(now + Duration::from_secs(5)),
            Some(Duration::from_secs(5))
        );

        view.on_activity(
            activity(SolvePhase::LinearSolve, Some(4), true),
            now + Duration::from_secs(5),
        );
        assert_eq!(
            view.phase_elapsed(now + Duration::from_secs(9)),
            Some(Duration::from_secs(4))
        );
    }

    #[test]
    fn detection_chart_keeps_one_time_sorted_line_per_camera() {
        let sample = |camera_id: &str, timestamp_ns: i64, points: usize| DetectionProgress {
            timestamp_ns,
            camera_id: camera_id.to_owned(),
            frames_processed: 1,
            tags: points / 4,
            points,
            frames_per_second: 10.0,
            average_frames_per_second: 10.0,
            workers: 4,
        };
        let progress = [
            sample("cam1", 3_000_000_000, 12),
            sample("cam0", 2_000_000_000, 8),
            sample("cam0", 1_000_000_000, 4),
            sample("cam1", 2_000_000_000, 16),
        ];

        let chart = detection_chart_series(&progress, (1_000_000_000, 3_000_000_000));

        assert_eq!(
            chart.series.keys().map(String::as_str).collect::<Vec<_>>(),
            ["cam0", "cam1"]
        );
        assert_eq!(chart.series["cam0"], [(0.0, 4.0), (1.0, 8.0)]);
        assert_eq!(chart.series["cam1"], [(1.0, 16.0), (2.0, 12.0)]);
        assert_eq!((chart.x_max, chart.y_max), (2.0, 16.0));
    }
}
