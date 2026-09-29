//! 四路标定仪表盘（Slint）：2×2 实时预览 + 检测叠加 + 右栏内参/外参收敛进度。
//!
//! 线程模型：
//! - **GUI 在主线**（winit 后端要求），采集/检测/求解/显示全在后台线程，两边只通过
//!   「最新快照」交换；
//! - **显示**：`display` 模块独占显示链路——唯一 worker 从每路 `FrameSlot` 取最新帧、
//!   叠加分析线程写下的 `Arc<Detection>`、转 RGBA 后由 GUI 线程写进持久 `frames` 模型。
//!   GUI 定时器只搬元数据（状态/计数/指标），不碰像素，更不逐帧深拷贝；
//! - **采集**：`pipeline` 模块独占取组/审核/求解/导出——分析线程（本文件 `guidance_worker`）
//!   只对**引导帧**做检测、把帧身份（epoch/序号/pts）与目标板端时刻打包成 `Candidate` 提交，
//!   从不阻塞等取图回执；取组、质量审核、入库、求解、导出都在 `pipeline` 的线程里完成。
//!
//! 数据面：引导帧（RTSP/视频，逐路门禁）→ 检测 → 候选（带时钟代 + 帧身份）→ 板端 raw
//! 同刻取组 → 每路证据帧质量/检测审核 → 该路内参会话入库 → 外参随共视组增加重算。
//! 没有时钟偏移就**拒绝触发**，绝不退回 LATEST。

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use opencv::core::Mat;
use slint::Model;

use rigcal_camera::detect::Detection;
use rigcal_camera::live::{self, Gate, GateState};
use rigcal_core::board::AprilGridConfig;
use rigcal_core::config::{Config, Guidance, GuidanceSource, RigSection};
use rigcal_core::session::{SessionState, SessionThresholds};
use rigcal_io::calibration::{ExportReceipt, export_calibration};
use rigcal_io::clock::{ClockAligner, PHASE_UNCERTAINTY_LIMIT_NS};
use rigcal_io::raw_tcp::RawTcpFrameSource;
use rigcal_io::rtsp::{FrameSlot, FrameSource, MonoTile};
use rigcal_opencv::quality::QualityThresholds;

mod display;
mod pipeline;

use pipeline::{
    Candidate, CaptureResult, CaptureStatus, FrameKey, Latest, Pipeline, quality_thresholds,
    request_export,
};

slint::include_modules!();

/// 板端 raw 帧服务的现场端口（`~/demo/config/sensor_config.yaml` 的 `raw_server.port`）。
const FIELD_RAW_PORT: u16 = 30432;

#[derive(Default)]
enum ClockStatus {
    #[default]
    Waiting,
    Sampling(u32),
    Retry {
        attempt: u32,
        at: Instant,
        reason: String,
    },
    Ready,
    Stopped,
}

impl ClockStatus {
    fn label(&self, now: Instant) -> String {
        match self {
            Self::Waiting => "CLOCK_WAIT：等待标定".to_owned(),
            Self::Sampling(attempt) => format!("CLOCK_SAMPLING：第 {attempt} 次"),
            Self::Retry {
                attempt,
                at,
                reason,
            } => format!(
                "CLOCK_RETRY：第 {attempt} 次失败，{:.1}s 后重试；{reason}",
                at.saturating_duration_since(now).as_secs_f64()
            ),
            Self::Ready => "CLOCK_READY".to_owned(),
            Self::Stopped => "CLOCK_STOPPED".to_owned(),
        }
    }
}

/// 一路的展示状态（后台写、GUI 读）。注意：**不含像素**——帧归 `display` 所有。
#[derive(Default)]
struct TileState {
    status: String,
    detail: String,
    /// 本帧质量行（清晰度/对比度 vs 门限）。
    quality: String,
    views: usize,
    note: String,
    solve_detail: String,
    metrics: Vec<(String, f64, f64)>,
    /// 入库高亮的截止时刻（由 UI 线程按时熄灭；None = 不亮）。
    flash_until: Option<Instant>,
    /// 该路的时钟对齐器（时钟线程标定，分析线程读偏移，pipeline 做闭环修正）。
    aligner: Option<ClockAligner>,
    /// 时钟代：每次标定成功/失败、重连/失效都推进；候选必须与它同代才有效。
    clock_revision: u64,
    recalibrate: bool,
    clock_status: ClockStatus,
}

/// 右栏与全局状态。
#[derive(Default)]
struct RigState {
    phase: String,
    message: String,
    hint: String,
    /// 已入库的证据版本数。
    groups: usize,
    rows: Vec<(String, f64, f64)>,
    /// 触发次数与取组失败次数（失败必须看得见，否则"绿框亮着但不涨"无从诊断）。
    triggers: usize,
    capture_failures: usize,
    /// 对齐失败次数（同刻取不到）与命中误差统计（ns）。
    alignment_failures: usize,
    hit_errors_ns: Vec<i64>,
    /// 审核拒绝的候选数（质量/检测/重复组）。
    rejections: usize,
    /// 采集侧最近一次审核结果（每版一行）。
    capture_message: String,
    /// 求解线程正在算的版本 / 已算（评估）的版本 / 已发布的完整版本。
    solving_version: Option<usize>,
    solved_version: Option<usize>,
    complete_version: Option<usize>,
    /// 求解错误不被下一次成功采集清空。
    solve_error: Option<String>,
    finalizing: bool,
    finalized: bool,
    /// 最近一次失败的原因（成功时清空）。
    last_error: Option<String>,
    /// 结束采集后，全量精修与导出由 pipeline 完成，GUI 不阻塞。
    export_ready: bool,
    export_requested: bool,
    export_failed: bool,
    export_message: String,
}

struct Shared {
    tiles: Vec<Mutex<TileState>>,
    rig: Mutex<RigState>,
    /// 最近一份完整快照；最终导出必须由全量精修替换，不得沿用旧在线版本。
    snapshot: Mutex<Option<Arc<CalibrationSnapshot>>>,
}

/// 同一组更新完成后的内参与外参；不混用前一组的外参和当前内参。
struct CalibrationSnapshot {
    states: BTreeMap<String, SessionState>,
    estimate: rigcal_core::extrinsics::RigEstimate,
    groups: usize,
}

fn save_calibration(
    snapshot: Option<&CalibrationSnapshot>,
    config: &Config,
    shared: &Shared,
    saved: &mut Option<(usize, ExportReceipt)>,
) -> Result<(), String> {
    let result = snapshot
        .ok_or_else(|| "尚无完整四路标定结果：各路须成功求解，且 cam0 外参图连通".to_owned())
        .and_then(|snapshot| {
            if let Some((groups, receipt)) = saved
                && *groups == snapshot.groups
            {
                return Ok((snapshot.groups, receipt.clone()));
            }
            export_calibration(
                Path::new(&config.output.root),
                config,
                &snapshot.states,
                &snapshot.estimate,
                snapshot.groups,
            )
            .map(|receipt| (snapshot.groups, receipt))
            .map_err(|error| error.to_string())
        });
    let message = match &result {
        Ok((groups, receipt)) => format!(
            "已导出组 #{groups} · {} · cam0 基准\n{}",
            if receipt.validated {
                "VALIDATED"
            } else {
                "DRAFT（未通过全部质量门禁）"
            },
            receipt.directory.display(),
        ),
        Err(error) => format!("导出失败：{error}"),
    };
    log_line(&message);
    if let Ok(mut state) = shared.rig.lock() {
        state.export_requested = false;
        state.export_failed = result.is_err();
        state.export_message = message.clone();
    }
    match result {
        Ok(receipt) => {
            println!("{message}");
            *saved = Some(receipt);
            Ok(())
        }
        Err(error) => Err(error),
    }
}

fn request_clock_calibration(shared: &Shared, camera_index: usize) {
    if let Some(tile) = shared.tiles.get(camera_index)
        && let Ok(mut tile) = tile.lock()
    {
        // 失效旧对齐并推进时钟代：在途候选随即作废（pipeline 按代校验）。
        tile.aligner = None;
        tile.recalibrate = true;
        tile.clock_status = ClockStatus::Waiting;
        tile.clock_revision = tile.clock_revision.wrapping_add(1);
    }
}

/// 展示原始诊断值；非有限值显示 n/a，不能冒充误差为零。
fn metrics_of(state: &SessionState, thresholds: SessionThresholds) -> Vec<(String, f64, f64)> {
    vec![
        ("rms_px".to_owned(), state.rms_px, thresholds.max_rms_px),
        (
            "focal_sigma".to_owned(),
            state
                .focal_relative_stddev
                .0
                .max(state.focal_relative_stddev.1),
            thresholds.max_focal_relative_stddev,
        ),
        (
            "principal_px".to_owned(),
            state.principal_stddev_px.0.max(state.principal_stddev_px.1),
            thresholds.max_principal_stddev_px,
        ),
        (
            "holdout_rms".to_owned(),
            state.holdout_rms_px,
            thresholds.max_holdout_rms_px,
        ),
    ]
}

/// GUI 诊断日志（无控制台，必须落文件；`--log` 可覆盖路径）。
static LOG_PATH: Mutex<Option<PathBuf>> = Mutex::new(None);

fn log_line(message: impl AsRef<str>) {
    // 整次日志追加持锁，避免各线程的消息相互交错。
    let Ok(mut slot) = LOG_PATH.lock() else {
        return;
    };
    let Some(path) = slot.clone() else {
        return;
    };
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
    {
        use std::io::Write;
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|value| value.as_secs_f64())
            .unwrap_or(0.0);
        let _ = writeln!(file, "[{stamp:.3}] {}", message.as_ref());
    }
    let _ = &mut slot;
}

/// 数值转文案：Rust 没有 `%g`，这里按量级选固定/科学计数（右侧栏与预览都用它）。
fn fmt_num(value: f64) -> String {
    if !value.is_finite() {
        return "n/a".to_owned();
    }
    let magnitude = value.abs();
    if magnitude != 0.0 && !(1.0e-3..1.0e4).contains(&magnitude) {
        format!("{value:.3e}")
    } else {
        format!("{value:.4}")
    }
}

fn version_label(value: Option<usize>) -> String {
    match value {
        Some(version) => format!("V{version}"),
        None => "-".to_owned(),
    }
}

/// 保留模型及重复布局，只通知实际变化的行。
fn update_rows<T: Clone + PartialEq + 'static>(
    model: &slint::VecModel<T>,
    values: impl IntoIterator<Item = T>,
) {
    let mut count = 0;
    for (index, value) in values.into_iter().enumerate() {
        if index == model.row_count() {
            model.push(value);
        } else if model.row_data(index).as_ref() != Some(&value) {
            model.set_row_data(index, value);
        }
        count = index + 1;
    }
    while model.row_count() > count {
        model.remove(model.row_count() - 1);
    }
}

fn tile_state(tile: &MonoTile) -> Result<Mat, opencv::Error> {
    live::tile_to_mat(tile)
}

/// 检测结果 → 预览标题文案。
fn detection_detail(detection: &Detection) -> String {
    format!(
        "tags={} corners={} rejected={}",
        detection.tag_ids.len(),
        detection.image_points.len(),
        detection.rejected
    )
}

/// 一路分析 worker 的静态上下文（打包传参，避免长参数表）。
struct GuidanceJob {
    index: usize,
    camera_id: String,
    image_size: (u32, u32),
    guidance: Guidance,
    board: AprilGridConfig,
    quality: QualityThresholds,
    /// 本进程本次启动的唯一源代：解码源没有重连（fail-closed），重连即新进程新代。
    source_epoch: u64,
    /// 板端 raw 端点 `(host, port, camera)`——时钟标定要直接问它要帧。
    raw: (String, u16, i32),
}

/// 分析 worker：读最新帧 → 门禁（不到检测时刻就不拷 Mat）→ 触发时提交候选 → 刷新叠加/元数据。
///
/// 只碰 `Gate` 与 `FrameSlot`/overlay；取组、入库、求解、导出全在 `pipeline` 的线程里，
/// 本函数**从不阻塞等回执**（每轮 `try_recv` 收一次，收到 Committed 且本路入库才更新新颖度基线）。
fn guidance_worker(
    job: GuidanceJob,
    slot: Arc<FrameSlot>,
    overlay: Arc<Mutex<Option<Arc<Detection>>>>,
    shared: Arc<Shared>,
    candidates: Arc<Latest<Candidate>>,
    stop: Arc<AtomicBool>,
) {
    let GuidanceJob {
        index,
        camera_id,
        image_size,
        guidance,
        board,
        quality,
        source_epoch,
        raw,
    } = job;
    let mut gate = match Gate::new(&guidance, board, image_size, quality) {
        Ok(gate) => gate,
        Err(error) => {
            if let Ok(mut tile) = shared.tiles[index].lock() {
                tile.status = format!("GATE_ERROR: {error}");
            }
            return;
        }
    };
    // 时钟采样有 raw 网络等待和约 4 秒采样窗，绝不能占用分析线程。
    // FrameSlot 是非消费式 latest-wins 单槽，解码器广播唤醒，采样与分析可独立读取。
    let clock_thread = std::thread::Builder::new()
        .name(format!("clock-{camera_id}"))
        .spawn({
            let shared = Arc::clone(&shared);
            let stop = Arc::clone(&stop);
            let slot = Arc::clone(&slot);
            let camera_id = camera_id.clone();
            move || {
                let mut attempt = 0u32;
                let mut next_attempt = Some(Instant::now() + Duration::from_millis(index as u64 * 150));
                let mut generation = slot.generation();
                while !stop.load(Ordering::Acquire) && !slot.finished() {
                    let requested = shared.tiles[index]
                        .lock()
                        .map(|mut tile| std::mem::take(&mut tile.recalibrate))
                        .unwrap_or(false);
                    if requested {
                        next_attempt = Some(Instant::now());
                    }
                    if next_attempt.is_none_or(|at| Instant::now() < at) {
                        slot.wait_changed(generation, Duration::from_millis(200));
                        generation = slot.generation();
                        continue;
                    }
                    attempt = attempt.saturating_add(1);
                    if let Ok(mut tile) = shared.tiles[index].lock() {
                        tile.aligner = None;
                        tile.clock_status = ClockStatus::Sampling(attempt);
                        tile.clock_revision = tile.clock_revision.wrapping_add(1);
                    }
                    log_line(format!("{camera_id} 开始后台时钟采样 #{attempt}（预览继续）"));
                    let calibrated = RawTcpFrameSource::new(&raw.0, raw.1, raw.2, image_size, 3.0)
                        .map_err(|error| error.to_string())
                        .and_then(|mut link| {
                            ClockAligner::calibrate(
                                &slot, &mut link, 40, Duration::from_millis(100),
                                PHASE_UNCERTAINTY_LIMIT_NS, &stop,
                            ).map_err(|error| error.to_string())
                        });
                    if stop.load(Ordering::Acquire) || slot.finished() {
                        break;
                    }
                    let failed = calibrated.is_err();
                    match &calibrated {
                        Ok(active) => log_line(format!(
                            "{camera_id} 时钟对齐完成：offset={} ns 帧周期={} ns 帧内散布={} ns 原始散布={} ns 样本={}（整数帧身份未判定）",
                            active.offset_ns(), active.frame_period_ns(), active.uncertainty_ns(),
                            active.raw_spread_ns(), active.samples(),
                        )),
                        Err(error) => log_line(format!("{camera_id} 时钟标定失败：{error}；将自动重试，期间拒绝触发")),
                    }
                    if let Ok(mut tile) = shared.tiles[index].lock() {
                        if tile.recalibrate {
                            tile.aligner = None;
                            tile.clock_status = ClockStatus::Waiting;
                            next_attempt = Some(Instant::now());
                        } else {
                            match calibrated {
                                Ok(active) => {
                                    tile.aligner = Some(active);
                                    tile.clock_status = ClockStatus::Ready;
                                    next_attempt = None;
                                    attempt = 0;
                                }
                                Err(reason) => {
                                    let delay = Duration::from_secs((1u64 << attempt.min(4)).min(10));
                                    let at = Instant::now() + delay;
                                    tile.aligner = None;
                                    tile.clock_status = ClockStatus::Retry { attempt, at, reason };
                                    next_attempt = Some(at);
                                }
                            }
                        }
                        tile.clock_revision = tile.clock_revision.wrapping_add(1);
                    }
                    if failed && let Ok(mut state) = shared.rig.lock() {
                        state.alignment_failures += 1;
                    }
                }
                if let Ok(mut tile) = shared.tiles[index].lock() {
                    tile.aligner = None;
                    tile.clock_status = ClockStatus::Stopped;
                    tile.clock_revision = tile.clock_revision.wrapping_add(1);
                }
            }
        });
    if let Err(error) = &clock_thread {
        log_line(format!(
            "{camera_id} 时钟线程启动失败（无时钟则不触发）：{error}"
        ));
    }

    let mut generation = 0u64;
    // 每路最多一个在途候选：新鲜度预算只有 150 ms，回执不阻塞分析循环。
    let mut pending: Option<Receiver<CaptureResult>> = None;
    while !stop.load(Ordering::Acquire) {
        // 收一次回执（非阻塞）：Committed 且本路入库才更新新颖度基线。
        let mut clear = false;
        if let Some(receiver) = pending.as_ref() {
            match receiver.try_recv() {
                Ok(result) => {
                    clear = true;
                    if result.key.source_epoch == source_epoch
                        && result.status == CaptureStatus::Committed
                        && result.accepted_cameras.contains(&index)
                    {
                        gate.on_capture(result.pose);
                    }
                    if let Ok(mut state) = shared.rig.lock() {
                        if result.status == CaptureStatus::Rejected {
                            state.rejections += 1;
                        }
                        state.capture_message = result.detail;
                    }
                }
                Err(mpsc::TryRecvError::Empty) => {}
                Err(mpsc::TryRecvError::Disconnected) => clear = true,
            }
        }
        if clear {
            pending = None;
        }
        if !slot.wait_changed(generation, Duration::from_millis(100)) {
            if slot.finished() {
                // 解码结束（含 RTSP 连不上）：fail-closed，写清原因并把该路时钟作废。
                let reason = slot
                    .error()
                    .unwrap_or_else(|| "解码结束（无更多帧）".to_owned());
                if let Ok(mut tile) = shared.tiles[index].lock() {
                    tile.status = "SOURCE_ERROR".to_owned();
                    tile.detail = reason.clone();
                    tile.aligner = None;
                    tile.clock_revision = tile.clock_revision.wrapping_add(1);
                }
                log_line(format!("{camera_id} 引导源结束：{reason}"));
                break;
            }
            continue;
        }
        generation = slot.generation();
        let Some(tile) = slot.latest() else {
            continue;
        };
        let now = Instant::now();
        // 不到检测时刻就**不拷 Mat**（省掉一次全分辨率拷贝）。
        if !gate.due(now) {
            continue;
        }
        let Ok(frame) = tile_state(&tile) else {
            continue;
        };
        // 隔离单帧 panic：保留分析线程，并将失败原因显示在对应预览格。
        let advanced =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| gate.advance(&frame, now)));
        let outcome = match advanced {
            Ok(Ok(outcome)) => outcome,
            Ok(Err(error)) => {
                if let Ok(mut tile) = shared.tiles[index].lock() {
                    tile.status = "GATE_ERROR".to_owned();
                    tile.detail = error.to_string();
                }
                continue;
            }
            Err(panic) => {
                let reason = panic
                    .downcast_ref::<&str>()
                    .map(|text| (*text).to_owned())
                    .or_else(|| panic.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "panic".to_owned());
                log_line(format!("{camera_id} 单帧 panic（已跳过）：{reason}"));
                if let Ok(mut tile) = shared.tiles[index].lock() {
                    tile.status = "PANIC".to_owned();
                    tile.detail = reason;
                }
                continue;
            }
        };
        // 叠加只是反馈：检测与画面不同源、可以差若干帧，显示端按需取这份 Arc。
        if let Ok(mut overlay) = overlay.lock() {
            *overlay = outcome.detection.clone().map(Arc::new);
        }
        let mut status = match outcome.state {
            GateState::Throttled => "TRACKING",
            GateState::Searching => "SEARCHING",
            GateState::WaitingSharp => "LOW_QUALITY",
            GateState::Detected => "DETECTED",
            GateState::Triggered => "TRIGGERED",
        }
        .to_owned();
        if candidates.is_closed() {
            status = "CAPTURE_STOPPED".to_owned();
        } else if outcome.triggered() && pending.is_none() {
            // 触发即算一次取图尝试（冷却/计数），**不**更新新颖度基线——回执 Committed 才算成功。
            gate.on_attempt(now);
            if let Some(measurement) = outcome.measurement {
                let clock = shared.tiles[index].lock().ok().map(|tile| {
                    (
                        tile.aligner
                            .as_ref()
                            .map(|active| (active.offset_ns(), active.frame_period_ns())),
                        tile.clock_revision,
                    )
                });
                let target = match (clock, tile.pts_ns) {
                    (Some((Some((offset, period)), revision)), Some(pts)) => pts
                        .checked_add(offset)
                        .filter(|board_ns| *board_ns >= 0)
                        .map(|board_ns| (board_ns as u64, revision, pts, period)),
                    _ => None,
                };
                match target {
                    Some((target_board_ns, clock_revision, pts_ns, frame_period_ns)) => {
                        let (reply, receiver) = mpsc::channel();
                        pending = Some(receiver);
                        candidates.submit(Candidate {
                            key: FrameKey {
                                source_epoch,
                                camera_index: index,
                                sequence: tile.index,
                                pts_ns,
                            },
                            target_board_ns,
                            clock_revision,
                            frame_period_ns,
                            created_at: now,
                            pose: measurement,
                            reply,
                        });
                    }
                    None => {
                        log_line(format!(
                            "{camera_id} 触发被拒：时钟未就绪或帧无时间戳（fail-closed，无 LATEST 回退）"
                        ));
                    }
                }
            }
        }
        if let Ok(mut tile) = shared.tiles[index].lock() {
            tile.status = status;
            tile.detail = outcome
                .detection
                .as_ref()
                .map(detection_detail)
                .unwrap_or_default();
            tile.quality = match &outcome.quality {
                Some(frame_quality) => format!(
                    "focus {:.0}/{:.0} · contrast {:.0}/{:.0}{}",
                    frame_quality.focus_score,
                    quality.min_focus_score,
                    frame_quality.contrast,
                    quality.min_contrast,
                    if frame_quality.accepted {
                        ""
                    } else {
                        " · REJECT"
                    },
                ),
                None => tile.quality.clone(),
            };
        }
    }
    if let Ok(clock_thread) = clock_thread
        && clock_thread.join().is_err()
    {
        log_line(format!("{camera_id} 时钟采样线程异常退出"));
    }
}

/// 解析 `--rtsp-base`：`host` 或 `host:port`，默认端口 554。
fn parse_rtsp_base(base: &str) -> Result<(String, u16), Box<dyn std::error::Error>> {
    match base.split_once(':') {
        Some((host, port)) => Ok((
            host.to_owned(),
            port.parse::<u16>()
                .map_err(|error| format!("--rtsp-base 端口: {error}"))?,
        )),
        None => Ok((base.to_owned(), 554)),
    }
}

/// 把四路引导源整体切到 RTSP：`camN → rtsp://host:(base_port + N)path`
fn apply_rtsp_base(
    rig: &mut RigSection,
    host: &str,
    base_port: u16,
    path: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    for camera in rig.cameras.iter_mut() {
        let channel = camera
            .camera_id
            .strip_prefix("cam")
            .unwrap_or(&camera.camera_id);
        let channel = channel
            .parse::<u16>()
            .map_err(|error| format!("相机编号 {}: {error}", camera.camera_id))?;
        camera.guidance = GuidanceSource::Rtsp {
            url: format!("rtsp://{host}:{}{path}", base_port + channel),
        };
    }
    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut config_path: Option<PathBuf> = None;
    let mut max_groups: Option<usize> = None;
    let mut run_seconds: Option<f64> = None;
    let mut log_path: Option<PathBuf> = None;
    let mut rtsp_base: Option<String> = None;
    let mut rtsp_path: Option<String> = None;
    let mut evidence_override: Option<String> = None;
    let mut replay_path: Option<PathBuf> = None;
    let mut replay_output: Option<PathBuf> = None;
    let mut index = 0;
    while index < argv.len() {
        let value = |shift: usize| argv.get(index + shift).cloned().unwrap_or_default();
        match argv[index].as_str() {
            "--config" => {
                config_path = Some(PathBuf::from(value(1)));
                index += 2;
            }
            "--max-groups" => {
                max_groups = value(1).parse::<usize>().ok().filter(|limit| *limit > 0);
                index += 2;
            }
            "--run-seconds" => {
                run_seconds = value(1).parse().ok();
                index += 2;
            }
            "--log" => {
                log_path = Some(PathBuf::from(value(1)));
                index += 2;
            }
            // 在线开关：不改 YAML 就能把演示视频换成现场相机
            "--rtsp-base" => {
                rtsp_base = Some(value(1));
                index += 2;
            }
            "--rtsp-path" => {
                rtsp_path = Some(value(1));
                index += 2;
            }
            "--evidence" => {
                evidence_override = Some(value(1));
                index += 2;
            }
            "--replay-observations" => {
                replay_path = Some(PathBuf::from(value(1)));
                index += 2;
            }
            "--out" => {
                replay_output = Some(PathBuf::from(value(1)));
                index += 2;
            }
            "--check-deps" => {
                println!("{}", rigcal_camera::native_dependency_report()?);
                return Ok(());
            }
            "-h" | "--help" => {
                println!(
                    "rigcal-gui —— 四路仪表盘（Slint）\n\n\
                     用法:\n  \
                     rigcal-gui --config <rig.yaml> [选项]\n  \
                     rigcal-gui --replay-observations <observations.jsonl> [--out <目录>]\n  \
                     rigcal-gui --check-deps\n\n\
                     选项:\n  \
                     --config <yaml>       四路配置（rig 段：cameras + evidence + 图约束）\n  \
                     --rtsp-base <host[:port]>\n                        把四路引导源整体切到 RTSP：\n                        \
                     camN → rtsp://host:(port+N)path（默认 port=554）\n  \
                     --rtsp-path <path>    RTSP 路径（默认 /PRR）\n  \
                     --evidence <host:port>\n                        覆盖证据帧服务端点（默认取配置里 rig.evidence）\n  \
                     --max-groups <n>      抓满 n 组后停止采集并导出；窗口保留（0/缺省 = 一直跑）\n  \
                     --run-seconds <s>     跑 s 秒后自动退出（无人值守验证用）\n  \
                     --log <file>          诊断日志（默认 <output.root>/gui.log）\n  \
                     --replay-observations <jsonl>\n                        从会话角点日志及相邻 config.yaml 全量精修，不连接相机、不启动 GUI\n  \
                     --out <目录>         仅重放时覆盖导出根目录（不改变留存的求解配置）\n  \
                     --check-deps          检查原生依赖版本与链接记录，不启动 GUI\n  \
                     正常关窗或定时结束时先全量精修，再导出至 <output.root>/exports/；失败不导出旧解，未达标结果标 DRAFT。\n  \
                     -h, --help"
                );
                return Ok(());
            }
            other => {
                eprintln!("rigcal-gui: unknown argument {other}");
                std::process::exit(2);
            }
        }
    }
    let dependency_info = rigcal_camera::native_dependency_info()?;
    eprintln!("{dependency_info}");
    if let Some(path) = replay_path {
        if config_path.is_some()
            || rtsp_base.is_some()
            || rtsp_path.is_some()
            || evidence_override.is_some()
            || max_groups.is_some()
            || run_seconds.is_some()
        {
            return Err("观测重放使用会话 config.yaml，不接受采集配置或采集选项".into());
        }
        *LOG_PATH.lock().unwrap_or_else(|error| error.into_inner()) = log_path;
        return pipeline::replay(&path, replay_output.as_deref()).map_err(Into::into);
    }
    if replay_output.is_some() {
        return Err("--out 仅用于 --replay-observations；在线输出根目录由配置指定".into());
    }
    let Some(config_path) = config_path else {
        eprintln!("rigcal-gui --config <rig.yaml>");
        std::process::exit(2);
    };
    let text = std::fs::read_to_string(&config_path)?;
    let mut config = Config::from_yaml(&text)?;
    // 在线开关：在配置之上做覆盖（改完重新校验，避免"以为改了其实没生效"）
    if let Some(base) = &rtsp_base {
        let (host, base_port) = parse_rtsp_base(base)?;
        let path = rtsp_path.clone().unwrap_or_else(|| "/PRR".to_owned());
        let Some(rig) = config.rig.as_mut() else {
            return Err("--rtsp-base 需要配置里的 rig 段".into());
        };
        apply_rtsp_base(rig, &host, base_port, &path)?;
        if evidence_override.is_none() {
            // 证据服务与相机同机：换 host，并把端口切到**板端实际端口**（30432）。
            // 端口以板端 `~/demo/config/sensor_config.yaml` 的 `raw_server.port` 为准，
            // 实际端点写入日志；可用 `--evidence host:port` 覆盖。
            rig.evidence.host = host;
            rig.evidence.port = FIELD_RAW_PORT;
        }
    }
    if let Some(evidence) = &evidence_override {
        let (host, port) = evidence
            .split_once(':')
            .ok_or("--evidence 需要 host:port")?;
        let Some(rig) = config.rig.as_mut() else {
            return Err("--evidence 需要配置里的 rig 段".into());
        };
        rig.evidence.host = host.to_owned();
        rig.evidence.port = port
            .parse::<u16>()
            .map_err(|error| format!("--evidence 端口: {error}"))?;
    }
    config.validate()?;
    let rig = config
        .rig
        .clone()
        .ok_or("rigcal-gui 需要配置里的 rig 段（四路：cameras + evidence + 图约束）")?;
    let image_size = config.image_size();
    {
        let default_log = PathBuf::from(&config.output.root).join("gui.log");
        let path = log_path.unwrap_or(default_log);
        if let Ok(mut slot) = LOG_PATH.lock() {
            *slot = Some(path.clone());
        }
        log_line(format!(
            "启动：配置 {}，日志 {}",
            config_path.display(),
            path.display()
        ));
        for camera in &rig.cameras {
            let source = match &camera.guidance {
                GuidanceSource::Rtsp { url } => format!("rtsp {url}"),
                GuidanceSource::Video { path } => format!("video {path}"),
            };
            log_line(format!("  引导 {} ← {source}", camera.camera_id));
        }
        log_line(format!(
            "  证据 raw-tcp://{}:{}",
            rig.evidence.host, rig.evidence.port
        ));
    }
    log_line(dependency_info);

    let mut tiles = Vec::new();
    for _ in &rig.cameras {
        tiles.push(Mutex::new(TileState {
            status: "WAITING".to_owned(),
            ..TileState::default()
        }));
    }
    let shared = Arc::new(Shared {
        tiles,
        rig: Mutex::new(RigState {
            phase: "TRAIN".to_owned(),
            message: "等待第一组证据帧".to_owned(),
            hint: format!(
                "证据端点 raw-tcp://{}:{}；门禁：检测 {}→{} Hz，新颖度 ×{:.1}，冷却 {:.1}s",
                rig.evidence.host,
                rig.evidence.port,
                config.guidance.detect_hz,
                config.guidance.detect_hz_max,
                config.guidance.trigger_novelty_scale,
                config.guidance.trigger_min_interval_s
            ),
            export_message: "完整角点持续留存；结束采集后全量精修，再导出最新结果".to_owned(),
            ..RigState::default()
        }),
        snapshot: Mutex::new(None),
    });
    let stop = Arc::new(AtomicBool::new(false));
    // 解码源没有重连（fail-closed）：每次启动给一个唯一源代，重连=新进程新代。
    let source_epoch = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|value| value.as_nanos() as u64)
        .unwrap_or(1)
        .max(1);

    // ---- 每路解码源：主线程起，槽位交给显示与分析共享（帧在槽里，不在 TileState 里）----
    let mut slots: Vec<Arc<FrameSlot>> = Vec::new();
    let mut overlays: Vec<Arc<Mutex<Option<Arc<Detection>>>>> = Vec::new();
    let mut sources: Vec<FrameSource> = Vec::new();
    let mut failed: Vec<bool> = Vec::new();
    for (index, camera) in rig.cameras.iter().enumerate() {
        let (locator, pace) = match &camera.guidance {
            GuidanceSource::Rtsp { url } => (url.clone(), false),
            GuidanceSource::Video { path } => (path.clone(), true),
        };
        match FrameSource::start(&locator, image_size, Duration::from_secs(10), pace) {
            Ok(source) => {
                slots.push(source.slot());
                sources.push(source);
                failed.push(false);
            }
            Err(error) => {
                if let Ok(mut tile) = shared.tiles[index].lock() {
                    tile.status = format!("SOURCE_ERROR: {error}");
                }
                log_line(format!(
                    "{} 引导源启动失败（fail-closed，无回退）：{error}",
                    camera.camera_id
                ));
                slots.push(Arc::new(FrameSlot::new()));
                failed.push(true);
            }
        }
        overlays.push(Arc::new(Mutex::new(None)));
    }

    // ---- 后台：采集/审核/求解/导出（pipeline 全权）；分析线程逐路提交候选 ----
    let pipeline = Pipeline::start(
        config.clone(),
        Arc::clone(&shared),
        Arc::clone(&stop),
        max_groups,
    )?;
    let board = config.board_config()?;
    let guidance = config.guidance.clone();
    let quality = quality_thresholds(&config);
    let mut workers = Vec::new();
    for (index, camera) in rig.cameras.iter().enumerate() {
        if failed[index] {
            continue;
        }
        let raw_camera = camera
            .raw_camera_id
            .unwrap_or_else(|| camera.camera_id[3..].parse().unwrap_or(0))
            as i32;
        let handle = std::thread::Builder::new()
            .name(format!("guide-{}", camera.camera_id))
            .spawn({
                let shared = Arc::clone(&shared);
                let stop = Arc::clone(&stop);
                let slot = Arc::clone(&slots[index]);
                let overlay = Arc::clone(&overlays[index]);
                let candidates = Arc::clone(&pipeline.candidates);
                let camera_id = camera.camera_id.clone();
                let guidance = guidance.clone();
                let board = board.clone();
                let raw = (rig.evidence.host.clone(), rig.evidence.port, raw_camera);
                move || {
                    guidance_worker(
                        GuidanceJob {
                            index,
                            camera_id,
                            image_size,
                            guidance,
                            board,
                            quality,
                            source_epoch,
                            raw,
                        },
                        slot,
                        overlay,
                        shared,
                        candidates,
                        stop,
                    )
                }
            })?;
        workers.push(handle);
    }

    // ---- 主线程：Slint ----
    let ui = Dashboard::new()?;
    {
        let shared = Arc::clone(&shared);
        let candidates = Arc::clone(&pipeline.candidates);
        ui.on_export_results(move || request_export(&shared, &candidates));
    }
    {
        let shared = Arc::clone(&shared);
        ui.on_retry_clocks(move || {
            for index in 0..shared.tiles.len() {
                let needs_retry = shared.tiles[index].lock().is_ok_and(|tile| {
                    tile.aligner.is_none()
                        && !matches!(
                            tile.clock_status,
                            ClockStatus::Sampling(_) | ClockStatus::Stopped
                        )
                });
                if needs_retry {
                    request_clock_calibration(&shared, index);
                }
            }
        });
    }
    let preview_inputs: Vec<display::PreviewInput> = slots
        .iter()
        .cloned()
        .zip(overlays.iter().cloned())
        .map(|(slot, overlay)| display::PreviewInput { slot, overlay })
        .collect();
    let preview = display::start(&ui, preview_inputs, Arc::clone(&stop))?;
    let camera_tiles = std::rc::Rc::new(slint::VecModel::from(vec![
        CameraTile::default();
        shared.tiles.len()
    ]));
    let metric_models: Vec<_> = shared
        .tiles
        .iter()
        .map(|_| std::rc::Rc::new(slint::VecModel::<Bar>::default()))
        .collect();
    let rig_rows = std::rc::Rc::new(slint::VecModel::<RigRow>::default());
    ui.set_tiles(camera_tiles.clone().into());
    ui.set_rig_rows(rig_rows.clone().into());

    let ui_weak = ui.as_weak();
    let timer = slint::Timer::default();
    {
        let shared = Arc::clone(&shared);
        let slots = slots.clone();
        // 元数据只需 10 Hz；像素由独立 display 通路按源帧率提交。
        timer.start(
            slint::TimerMode::Repeated,
            Duration::from_millis(100),
            move || {
                let Some(ui) = ui_weak.upgrade() else {
                    return;
                };
                // 与 publish_result 同一锁序：一轮读取的内外参一定属于同一完整版本。
                let Ok(state) = shared.rig.lock() else { return; };
                let mut alignment_ready = true;
                for (index, tile) in shared.tiles.iter().enumerate() {
                    let Ok(tile) = tile.lock() else {
                        alignment_ready = false;
                        continue;
                    };
                    alignment_ready &= tile.aligner.is_some();
                    let metrics = tile.metrics.iter().map(|(label, value, limit)| Bar {
                        label: label.clone().into(),
                        text: format!("{label}: {} / {}", fmt_num(*value), fmt_num(*limit)).into(),
                        ratio: if value.is_finite() && *limit > 0.0 {
                            (value / limit).clamp(0.0, 1.0) as f32
                        } else { 0.0 },
                    });
                    update_rows(&metric_models[index], metrics);
                    let model = CameraTile {
                        id: format!("cam{index}").into(),
                        status: tile.status.clone().into(),
                        detail: tile.detail.clone().into(),
                        quality: tile.quality.clone().into(),
                        clock_status: tile.aligner.as_ref().map_or_else(
                            || tile.clock_status.label(Instant::now()),
                            |active| format!(
                                "CLOCK_READY · 相位 {:.3} / {:.1} ms · 原始 {:.3} ms · 周期 {:.3} ms",
                                active.uncertainty_ns() as f64 / 1e6,
                                PHASE_UNCERTAINTY_LIMIT_NS as f64 / 1e6,
                                active.raw_spread_ns() as f64 / 1e6,
                                active.frame_period_ns() as f64 / 1e6,
                            ),
                        ).into(),
                        solve_detail: tile.solve_detail.clone().into(),
                        views: tile.views as i32,
                        note: tile.note.clone().into(),
                        metrics: metric_models[index].clone().into(),
                        capture_flash: tile.flash_until.is_some_and(|until| until > Instant::now()),
                        has_image: slots[index].latest().is_some(),
                    };
                    if camera_tiles.row_data(index).as_ref() != Some(&model) {
                        camera_tiles.set_row_data(index, model);
                    }
                }
                    // 计数行是现场唯一的"运行体征"：已采/已算/完整 + 触发/拒绝/失败都要看得见。
                    let p95_ms = if state.hit_errors_ns.is_empty() {
                        f64::NAN
                    } else {
                        let mut sorted: Vec<_> = state.hit_errors_ns.iter().map(|value| value.unsigned_abs()).collect();
                        sorted.sort_unstable();
                        let index = ((sorted.len() as f64 * 0.95).ceil() as usize)
                            .min(sorted.len())
                            - 1;
                        sorted[index] as f64 / 1.0e6
                    };
                    let solving = match state.solving_version {
                        Some(version) => format!(" · 算中 V{version}"),
                        None => String::new(),
                    };
                    ui.set_counters(
                        format!(
                            "已采 {} 版 · 已算 {} · 完整 {}{solving}\n触发 {} · 拒绝 {} · 取组失败 {}\n对齐{} · 对齐失败 {} · 命中 |误差| p95 {} ms",
                            state.groups,
                            version_label(state.solved_version),
                            version_label(state.complete_version),
                            state.triggers,
                            state.rejections,
                            state.capture_failures,
                            if alignment_ready { "✓" } else { "✗" },
                            state.alignment_failures,
                            if p95_ms.is_finite() {
                                format!("{p95_ms:.3}")
                            } else {
                                "-".to_owned()
                            }
                        )
                        .into(),
                    );
                    ui.set_error([state.last_error.as_deref(), state.solve_error.as_deref()].into_iter().flatten().collect::<Vec<_>>().join("；").into());
                    ui.set_phase(state.phase.clone().into());
                    let message = if state.capture_message.is_empty() {
                        state.message.clone()
                    } else {
                        format!("{} · {}", state.capture_message, state.message)
                    };
                    ui.set_message(message.into());
                    ui.set_hint(state.hint.clone().into());
                    ui.set_groups(state.groups as i32);
                    ui.set_can_export(state.export_ready && !state.export_requested && !state.finalizing && !state.finalized);
                    ui.set_export_pending(state.export_requested);
                    ui.set_export_error(state.export_failed);
                    ui.set_export_message(state.export_message.clone().into());
                    let rows = state.rows.iter().map(|(label, value, limit)| RigRow {
                        label: label.clone().into(),
                        text: format!("{label}: {} / {}", fmt_num(*value), fmt_num(*limit)).into(),
                        ratio: if value.is_finite() && *limit > 0.0 {
                            (value / limit).clamp(0.0, 1.0) as f32
                        } else { 0.0 },
                    });
                    update_rows(&rig_rows, rows);
            },
        );
    }
    // Timer 在 drop 时停止；必须保持其存活直到事件循环返回。
    let mut stopper: Option<slint::Timer> = None;
    if let Some(seconds) = run_seconds {
        let ui_weak = ui.as_weak();
        let timer = slint::Timer::default();
        timer.start(
            slint::TimerMode::SingleShot,
            Duration::from_secs_f64(seconds.max(0.1)),
            move || {
                if let Some(ui) = ui_weak.upgrade() {
                    let _ = ui.window().hide();
                }
                // 结束事件循环，让 `run()` 返回
                slint::quit_event_loop().ok();
            },
        );
        stopper = Some(timer);
    }
    ui.run()?;
    drop(stopper);

    // ---- 收尾：停源与采集 → 排空审核 → 最新版本全量精修 → 导出 ----
    stop.store(true, Ordering::Release);
    for worker in workers {
        let _ = worker.join();
    }
    for source in &mut sources {
        source.stop();
    }
    log_line(format!("预览统计：{:?}", preview.stats()));
    let preview_result = preview.finish();
    let pipeline_result = pipeline.finish();
    log_line(format!(
        "收尾：显示 {}, 采集 {}",
        match &preview_result {
            Ok(()) => "已停止",
            Err(_) => "异常",
        },
        match &pipeline_result {
            Ok(()) => "已停止并完成导出",
            Err(_) => "异常（结果未确认导出）",
        }
    ));
    preview_result?;
    pipeline_result?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{apply_rtsp_base, parse_rtsp_base};
    use rigcal_core::config::{Config, GuidanceSource};

    const CONFIG: &str = include_str!("../example.rig.yaml");

    #[test]
    fn rtsp_base_parses_host_with_and_without_port() {
        assert_eq!(
            parse_rtsp_base("10.21.12.162").unwrap(),
            ("10.21.12.162".to_owned(), 554)
        );
        assert_eq!(
            parse_rtsp_base("cam.local:8554").unwrap(),
            ("cam.local".to_owned(), 8554)
        );
        assert!(parse_rtsp_base("host:abc").is_err());
    }

    #[test]
    fn rtsp_base_rewrites_every_camera_like_python_does() {
        let mut config = Config::from_yaml(CONFIG).expect("example config must parse");
        let rig = config.rig.as_mut().expect("rig section");
        apply_rtsp_base(rig, "10.21.12.162", 554, "/PRR").expect("apply");
        let urls: Vec<String> = rig
            .cameras
            .iter()
            .map(|camera| match &camera.guidance {
                GuidanceSource::Rtsp { url } => url.clone(),
                other => panic!("expected rtsp guidance, got {other:?}"),
            })
            .collect();
        assert_eq!(
            urls,
            vec![
                "rtsp://10.21.12.162:554/PRR",
                "rtsp://10.21.12.162:555/PRR",
                "rtsp://10.21.12.162:556/PRR",
                "rtsp://10.21.12.162:557/PRR",
            ],
            "camN 必须落在 base_port + N（与 Python preview.py 同约定）"
        );
        // 覆盖后配置仍然合法（否则在线开关会把配置改坏）
        config
            .validate()
            .expect("overridden config must stay valid");
    }
}
