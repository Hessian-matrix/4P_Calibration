//! 单相机内参产线：取图 → 检测 → 观测入库 → 求解与收敛分析。
//!
//! 会话使用 `rigcal-core` 的求解采纳、视图排除与收敛门禁。
//!
//! 阶段：
//! 1. **取图与检测**：逐帧读入 → 0.75× 检出 + 全分辨率亚像素精修 → 终端实时预览（ANSI 真彩色）
//!    + 首帧 PNG 快照 → 命中即入数据集；
//! 2. **求解与分析**：对配置里的每个模型跑一遍会话循环，每次入库重建信息矩阵（σ/秩/条件数/
//!    Δlogdet）并按节奏做真实优化，进度条实时刷新；
//! 3. **收敛**：连续 `window` 次评估满足 goal（满秩 ∧ rms ∧ 焦距 σ ∧ 主点 σ）即停，写 YAML 结果。

use rigcal_camera::{backend_for, detect, live, preview};

use std::io::Write;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use opencv::prelude::*;
use rigcal_core::ModelKind;
use rigcal_core::board::AprilGridConfig;
use rigcal_core::config::{Config, GuidanceSource};
use rigcal_core::estimator::Observation;
use rigcal_core::session::{Session, SessionOptions, SessionState, SessionThresholds};
use rigcal_io::raw_tcp::RawTcpFrameSource;
use rigcal_io::rtsp::FrameSource;
use rigcal_opencv::quality::{QualityThresholds, classify_frame_quality};
use serde::Serialize;

/// 会话阈值：默认值 + 配置里的 holdout 门禁两项。
fn session_thresholds(config: &Config) -> SessionThresholds {
    SessionThresholds {
        max_holdout_rms_px: config.solver.max_holdout_rms_px,
        max_holdout_p95_px: config.solver.max_holdout_p95_px,
        ..SessionThresholds::default()
    }
}

/// 采集质量门禁阈值（配置 `solver.quality`）。
fn quality_thresholds(config: &Config) -> QualityThresholds {
    QualityThresholds {
        min_focus_score: config.solver.quality.min_focus_score,
        min_contrast: config.solver.quality.min_contrast,
        max_saturated_fraction: config.solver.quality.max_saturated_fraction,
    }
}

const USAGE: &str = "\
rigcal-camera —— 单相机内参产线（Rust，会话循环）

用法:
  rigcal-camera --config <single-file.yaml> --frames <dir> [--out <dir>] [选项]   # 离线：目录回放
  rigcal-camera --config <single-file.yaml> --live [--out <dir>] [选项]          # 在线：RTSP + 板端 raw
  rigcal-camera --check-deps                                               # 依赖版本与链接记录

在线模式（--live）用配置里的 `capture.guidance`（RTSP / 本地视频）做引导，用 `capture.evidence`
（板端 raw 服务）取证据帧；触发判据见 guidance 段。

选项:
  --live               在线取图（RTSP 引导 + raw 证据帧），与 --frames 互斥
  --limit <n>          离线：只处理前 n 帧；在线：最多抓 n 张证据帧
  --preview-every <n>  每 n 帧刷新一次终端预览（默认 1；0 = 关闭预览）
  --render-width <n>   终端预览宽度（字符列，默认 96）
  --preview-out <png>  首帧带叠加的画面落盘
  --cadence <n>        每入库 n 张做一次真实优化（默认 1）
  --check-deps        检查原生依赖版本与链接记录，不读取采集配置
  -h, --help
";

#[derive(Debug)]
struct Args {
    config: PathBuf,
    live: bool,
    frames: Option<PathBuf>,
    out: PathBuf,
    limit: Option<usize>,
    preview_every: usize,
    render_width: usize,
    preview_out: Option<String>,
    dump_corners: Option<String>,
    cadence: usize,
}

fn parse_args() -> Result<Args, String> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut config = None;
    let mut live = false;
    let mut frames = None;
    let mut out = PathBuf::from("calibration_runs/camera");
    let mut limit = None;
    let mut preview_every = 1usize;
    let mut render_width = 96usize;
    let mut preview_out = None;
    let mut dump_corners = None;
    let mut cadence = 1usize;
    let mut index = 0;
    while index < argv.len() {
        let value = |index: usize| -> Result<String, String> {
            argv.get(index + 1)
                .cloned()
                .ok_or_else(|| format!("{} needs a value", argv[index]))
        };
        match argv[index].as_str() {
            "--config" => config = Some(PathBuf::from(value(index)?)),
            "--live" => live = true,
            "--frames" => frames = Some(PathBuf::from(value(index)?)),
            "--out" => out = PathBuf::from(value(index)?),
            "--limit" => limit = Some(value(index)?.parse().map_err(|e| format!("--limit: {e}"))?),
            "--preview-every" => {
                preview_every = value(index)?
                    .parse()
                    .map_err(|e| format!("--preview-every: {e}"))?
            }
            "--render-width" => {
                render_width = value(index)?
                    .parse()
                    .map_err(|e| format!("--render-width: {e}"))?
            }
            "--preview-out" => preview_out = Some(value(index)?),
            "--dump-corners" => dump_corners = Some(value(index)?),
            "--cadence" => {
                cadence = value(index)?
                    .parse()
                    .map_err(|e| format!("--cadence: {e}"))?
            }
            "--check-deps" => {
                match rigcal_camera::native_dependency_report() {
                    Ok(report) => println!("{report}"),
                    Err(error) => {
                        eprintln!("rigcal-camera: {error}");
                        std::process::exit(1);
                    }
                }
                std::process::exit(0);
            }
            "-h" | "--help" => {
                print!("{USAGE}");
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument {other}\n\n{USAGE}")),
        }
        index += if argv[index].starts_with("--") && argv[index] != "--live" {
            2
        } else {
            1
        };
    }
    Ok(Args {
        config: config.ok_or("--config is required")?,
        live,
        frames,
        out,
        limit,
        preview_every,
        render_width,
        preview_out,
        dump_corners,
        cadence,
    })
}

fn list_frames(dir: &Path) -> Result<Vec<PathBuf>, String> {
    let mut frames = Vec::new();
    for entry in
        std::fs::read_dir(dir).map_err(|error| format!("cannot read {}: {error}", dir.display()))?
    {
        let path = entry.map_err(|error| error.to_string())?.path();
        let extension = path
            .extension()
            .and_then(|value| value.to_str())
            .map(|value| value.to_ascii_lowercase())
            .unwrap_or_default();
        // PGM 与 PNG/JPEG 均通过 OpenCV 灰度读取。
        if matches!(extension.as_str(), "png" | "jpg" | "jpeg" | "pgm") {
            frames.push(path);
        }
    }
    frames.sort();
    Ok(frames)
}

#[derive(Serialize)]
struct ModelResult {
    model: String,
    converged: bool,
    views: usize,
    used_views: usize,
    excluded_views: usize,
    rms_px: f64,
    rank: usize,
    condition_number: f64,
    focal_relative_stddev: Vec<f64>,
    principal_stddev_px: Vec<f64>,
    solves: usize,
    failures: usize,
    parameter_names: Vec<String>,
    parameters: Vec<f64>,
    detail: String,
}

/// 每个模型一个跑器：观测入库 → 会话推进 → 终端进度 → 结论落盘。
/// 离线回放与在线取图两条路径共用它，避免两份求解/渲染逻辑。
struct ModelRunner {
    model: ModelKind,
    session: Session,
    started: Instant,
    last: Option<SessionState>,
    observed: usize,
}

impl ModelRunner {
    fn new(
        model: ModelKind,
        config: &Config,
        image_size: (u32, u32),
        cadence: usize,
        thresholds: SessionThresholds,
    ) -> Self {
        let image_size_for_session = image_size;
        let _ = image_size;
        Self {
            model,
            session: Session::new(
                SessionOptions {
                    model,
                    image_size: image_size_for_session,
                    cadence,
                    max_solve_observations: config.solver.max_solve_observations,
                    holdout_fraction: config.solver.holdout_fraction,
                    min_holdout_views: config.solver.official_holdout_frames,
                },
                thresholds,
                backend_for(model, config)
                    .unwrap_or_else(|error| Box::new(move |_request| Err(error.clone()))),
            ),
            started: Instant::now(),
            last: None,
            observed: 0,
        }
    }

    fn converged(&self) -> bool {
        self.last
            .as_ref()
            .map(|state| state.converged)
            .unwrap_or(false)
    }

    /// 入库一个观测并刷新进度；返回是否已收敛。
    fn observe(
        &mut self,
        observation: Observation,
        label: &str,
        thresholds: SessionThresholds,
    ) -> bool {
        let outcome = self.session.observe(observation);
        self.observed += 1;
        let rows = preview::render_session_bars(&outcome.state, thresholds);
        let converged = outcome.state.converged;
        print!(
            "\x1b[2J\x1b[H=== {} === 观测 {}{}  {:.1}s\n{}\n",
            self.model.as_str(),
            self.observed,
            label,
            self.started.elapsed().as_secs_f64(),
            rows.join("\n")
        );
        std::io::stdout().flush().ok();
        self.last = Some(outcome.state);
        converged
    }

    /// 写 `models/<model>.yaml` 并返回可直接序列化的结论。
    fn finish(self, models_dir: &Path) -> Result<ModelResult, String> {
        let state = self.last.ok_or("session produced no state")?;
        let result = ModelResult {
            model: self.model.as_str().to_owned(),
            converged: state.converged,
            views: state.views,
            used_views: state.used_views,
            excluded_views: state.excluded_views,
            rms_px: state.rms_px,
            rank: state.rank,
            condition_number: state.condition_number,
            focal_relative_stddev: vec![
                state.focal_relative_stddev.0,
                state.focal_relative_stddev.1,
            ],
            principal_stddev_px: vec![state.principal_stddev_px.0, state.principal_stddev_px.1],
            solves: state.solves,
            failures: state.failures,
            parameter_names: self
                .model
                .parameter_names()
                .iter()
                .map(|name| name.to_string())
                .collect(),
            parameters: state.parameters.clone(),
            detail: state.detail.clone(),
        };
        let path = models_dir.join(format!("{}.yaml", self.model.as_str()));
        std::fs::write(
            &path,
            serde_yaml::to_string(&result).map_err(|error| error.to_string())?,
        )
        .map_err(|error| format!("cannot write {}: {error}", path.display()))?;
        println!(
            "\n=== {} 结论 ===\n  converged={} views={} solves={} failures={}\n  {}\n  写入 {}",
            self.model.as_str(),
            state.converged,
            state.views,
            state.solves,
            state.failures,
            state.detail,
            path.display()
        );
        Ok(result)
    }
}

fn main() -> std::process::ExitCode {
    let args = match parse_args() {
        Ok(args) => args,
        Err(message) => {
            eprintln!("{message}");
            return std::process::ExitCode::from(2);
        }
    };
    match run(&args) {
        Ok(code) => code,
        Err(message) => {
            eprintln!("rigcal-camera: {message}");
            std::process::ExitCode::from(1)
        }
    }
}

fn run(args: &Args) -> Result<std::process::ExitCode, String> {
    eprintln!("{}", rigcal_camera::native_dependency_info()?);
    let text = std::fs::read_to_string(&args.config)
        .map_err(|error| format!("cannot read {}: {error}", args.config.display()))?;
    let config = Config::from_yaml(&text).map_err(|error| error.to_string())?;
    let board = config.board_config().map_err(|error| error.to_string())?;
    if args.live {
        return run_live(args, &config, &board);
    }
    let image_size = config.image_size();
    let frames_dir = args
        .frames
        .as_ref()
        .ok_or("--frames is required unless --live")?;
    let frames = list_frames(frames_dir)?;
    if frames.is_empty() {
        return Err(format!("no PNG/JPG frames in {}", frames_dir.display()));
    }
    let total = args.limit.unwrap_or(frames.len()).min(frames.len());
    println!(
        "rigcal-camera: {} 帧（用 {total}）{}x{} 板={}x{} 模型={:?}",
        frames.len(),
        image_size.0,
        image_size.1,
        board.rows,
        board.cols,
        config
            .solver
            .models
            .iter()
            .map(|model| model.as_str())
            .collect::<Vec<_>>()
    );

    // ---- 阶段 1：循环取图 → 检测 → 入数据集（+ 预览）----
    let started = Instant::now();
    let mut observations: Vec<Observation> = Vec::new();
    let mut locked_border_bits: Option<i32> = None;
    let mut snapshot_written = false;
    let mut rejected = 0usize;
    let mut corner_dump: Vec<serde_json::Value> = Vec::new();
    for (index, path) in frames.iter().take(total).enumerate() {
        let gray = detect::load_gray(&path.to_string_lossy())
            .map_err(|error| format!("cannot read {}: {error}", path.display()))?;
        if gray.empty() {
            return Err(format!("frame {} decoded empty", path.display()));
        }
        if (gray.cols(), gray.rows()) != (image_size.0 as i32, image_size.1 as i32) {
            return Err(format!(
                "frame {} is {}x{}, config expects {}x{}（尺寸不符必须 fail closed）",
                path.display(),
                gray.cols(),
                gray.rows(),
                image_size.0,
                image_size.1
            ));
        }
        let detection = detect::detect_board(&gray, &board, locked_border_bits)
            .map_err(|error| format!("detection failed on {}: {error}", path.display()))?;
        if let (Some(bits), None) = (detection.border_bits, locked_border_bits) {
            // 首次成功后锁定黑边比特数，后续每帧只检测一次。
            locked_border_bits = Some(bits);
        }
        if args.dump_corners.is_some() {
            corner_dump.push(serde_json::json!({
                "frame": path.file_name().and_then(|name| name.to_str()),
                "status": detection.status,
                "tag_ids": detection.tag_ids,
                "border_bits": detection.border_bits,
                "image_points": detection.image_points,
                "object_points": detection.object_points,
            }));
        }
        if detection.detected() {
            // 质量门禁：清晰度/对比度/削顶——不合格的帧不进数据集（理由打印出来便于现场定位）
            let quality = classify_frame_quality(&gray, &quality_thresholds(&config))
                .map_err(|error| format!("quality gate failed: {error}"))?;
            if quality.accepted {
                observations.push(Observation {
                    object_points: detection.object_points.clone(),
                    image_points: detection.image_points.clone(),
                });
            } else {
                rejected += 1;
                eprintln!(
                    "{} 质量门禁拒绝：{:?}（清晰度 {:.1}，对比度 {:.1}，削顶 {:.3}）",
                    path.display(),
                    quality.reasons,
                    quality.focus_score,
                    quality.contrast,
                    quality.saturated_fraction
                );
            }
        }
        let show_preview = args.preview_every > 0 && index % args.preview_every == 0;
        if show_preview || (args.preview_out.is_some() && !snapshot_written) {
            let mut canvas = detect::to_canvas(&gray).map_err(|error| error.to_string())?;
            let header = format!("rigcal-camera  {}/{}", index + 1, total);
            preview::draw_detection(&mut canvas, &detection, &header)
                .map_err(|error| error.to_string())?;
            if let Some(path) = &args.preview_out
                && !snapshot_written
            {
                preview::save_snapshot(&canvas, path).map_err(|error| error.to_string())?;
                snapshot_written = true;
            }
            if show_preview {
                let rendered = preview::render_terminal(&canvas, args.render_width)
                    .map_err(|error| error.to_string())?;
                print!("\x1b[2J\x1b[H{rendered}");
                print!(
                    "frame {}/{}  {}  tags={}  corners={}  rejected={}  accepted={}",
                    index + 1,
                    total,
                    detection.status,
                    detection.tag_ids.len(),
                    detection.image_points.len(),
                    detection.rejected,
                    observations.len()
                );
                std::io::stdout().flush().ok();
            }
        }
    }
    if let Some(path) = &args.dump_corners {
        std::fs::write(
            path,
            serde_json::to_string_pretty(&corner_dump).map_err(|e| e.to_string())?,
        )
        .map_err(|error| format!("cannot write {path}: {error}"))?;
    }
    println!(
        "\n\x1b[2J\x1b[H检测完成：{} 个观测入库 / {} 帧（质量门禁拒绝 {rejected}），用时 {:.1}s",
        observations.len(),
        total,
        started.elapsed().as_secs_f64()
    );
    if observations.len() < config.solver.min_observations {
        return Err(format!(
            "只有 {} 个观测，少于 solver.min_observations={}",
            observations.len(),
            config.solver.min_observations
        ));
    }

    // ---- 阶段 2：每个模型跑一遍会话循环（求解 → 分析 → 直到收敛 goal）----
    let thresholds = session_thresholds(&config);
    let models_dir = args.out.join("models");
    std::fs::create_dir_all(&models_dir)
        .map_err(|error| format!("cannot create {}: {error}", models_dir.display()))?;
    let mut results = Vec::new();
    for model in &config.solver.models {
        let mut runner = ModelRunner::new(*model, &config, image_size, args.cadence, thresholds);
        for observation in &observations {
            if runner.observe(
                observation.clone(),
                &format!(" / {}", observations.len()),
                thresholds,
            ) {
                break;
            }
        }
        results.push(runner.finish(&models_dir)?);
    }

    let all_converged = results.iter().all(|result| result.converged);
    println!("\n结果目录：{}", args.out.display());
    if all_converged {
        println!(
            "RIGCAL_CAMERA CONVERGED models={:?}",
            results
                .iter()
                .map(|result| result.model.as_str())
                .collect::<Vec<_>>()
        );
        Ok(std::process::ExitCode::SUCCESS)
    } else {
        println!(
            "RIGCAL_CAMERA INCOMPLETE failed={:?}",
            results
                .iter()
                .filter(|r| !r.converged)
                .map(|r| r.model.as_str())
                .collect::<Vec<_>>()
        );
        Ok(std::process::ExitCode::from(1))
    }
}

/// 在线取图：引导帧源（RTSP / 本地视频）驱动质量门禁与检测，触发后从板端 raw 服务取**证据帧**入库。
///
/// RTSP 只做引导，标定输入只来自 raw 服务；证据帧锁定黑边比特数后单遍检测。
fn run_live(
    args: &Args,
    config: &Config,
    board: &AprilGridConfig,
) -> Result<std::process::ExitCode, String> {
    let image_size = config.image_size();
    let capture = config
        .capture
        .as_ref()
        .ok_or("--live 需要 capture 段（单相机：guidance + evidence）；四路配置请用 rigcal-gui")?;
    let (locator, label) = match &capture.guidance {
        GuidanceSource::Rtsp { url } => (url.clone(), format!("rtsp {url}")),
        GuidanceSource::Video { path } => (path.clone(), format!("video {path}")),
    };
    let evidence = capture
        .evidence
        .as_ref()
        .ok_or("在线模式需要 capture.evidence（板端 raw 服务）；离线回放请用 --frames")?;

    // 文件源按流帧率节流：让它像在线源一样按真实节奏出帧（门禁与冷却都是 wall-clock 判据）。
    let pace = matches!(capture.guidance, GuidanceSource::Video { .. });
    let mut source = FrameSource::start(&locator, image_size, Duration::from_secs(10), pace)
        .map_err(|error| error.to_string())?;
    let slot = source.slot();
    if !source.wait_first_frame(Duration::from_secs(15)) {
        let reason = slot.error().unwrap_or_else(|| "等待首帧超时".to_owned());
        source.stop();
        return Err(format!("引导源无帧：{reason}"));
    }
    let mut raw = RawTcpFrameSource::new(
        &evidence.host,
        evidence.port,
        evidence.camera as i32,
        image_size,
        5.0,
    )
    .map_err(|error| error.to_string())?;
    let mut gate = live::Gate::new(
        &config.guidance,
        board.clone(),
        image_size,
        quality_thresholds(config),
    )
    .map_err(|error| error.to_string())?;
    let thresholds = session_thresholds(config);
    let mut runners: Vec<ModelRunner> = config
        .solver
        .models
        .iter()
        .map(|model| ModelRunner::new(*model, config, image_size, args.cadence, thresholds))
        .collect();
    let models_dir = args.out.join("models");
    std::fs::create_dir_all(&models_dir)
        .map_err(|error| format!("cannot create {}: {error}", models_dir.display()))?;
    let capture_dir = args.out.join("captures");
    std::fs::create_dir_all(&capture_dir)
        .map_err(|error| format!("cannot create {}: {error}", capture_dir.display()))?;

    let max_captures = args.limit.unwrap_or(usize::MAX);
    println!(
        "rigcal-camera 在线：引导={label}  证据={}  {}x{}  模型={:?}",
        raw.source_label(),
        image_size.0,
        image_size.1,
        config
            .solver
            .models
            .iter()
            .map(|model| model.as_str())
            .collect::<Vec<_>>()
    );
    println!(
        "  门禁：清晰度≥{:.0}  对比度≥{:.0}  削顶≤{:.2}  新颖度×{:.1}  冷却 {:.1}s  检测 {:?}Hz→{:?}Hz",
        config.solver.quality.min_focus_score,
        config.solver.quality.min_contrast,
        config.solver.quality.max_saturated_fraction,
        config.guidance.trigger_novelty_scale,
        config.guidance.trigger_min_interval_s,
        config.guidance.detect_hz,
        config.guidance.detect_hz_max
    );

    let started = Instant::now();
    let mut generation = 0u64;
    let mut frames_seen = 0usize;
    let mut captures = 0usize;
    let mut evidence_border_bits: Option<i32> = None;
    let mut snapshot_written = false;
    let mut capture_log: Vec<serde_json::Value> = Vec::new();

    loop {
        if captures >= max_captures {
            break;
        }
        if runners.iter().all(ModelRunner::converged) {
            println!("\n全部模型收敛，结束在线取图。");
            break;
        }
        if !slot.wait_changed(generation, Duration::from_millis(500)) {
            if slot.finished() {
                println!("\n引导源结束。");
                break;
            }
            continue;
        }
        generation = slot.generation();
        let Some(tile) = slot.latest() else {
            continue;
        };
        frames_seen += 1;
        let frame = live::tile_to_mat(&tile).map_err(|error| error.to_string())?;
        let now = Instant::now();
        let outcome = gate
            .advance(&frame, now)
            .map_err(|error| error.to_string())?;

        let show_preview = args.preview_every > 0 && frames_seen.is_multiple_of(args.preview_every);
        if show_preview || (args.preview_out.is_some() && !snapshot_written) {
            let mut canvas = detect::to_canvas(&frame).map_err(|error| error.to_string())?;
            let header = format!(
                "在线 帧 {frames_seen}  捕获 {captures}  检测 {}  尝试 {}",
                gate.detections, gate.attempts
            );
            if let Some(detection) = &outcome.detection {
                preview::draw_detection(&mut canvas, detection, &header)
                    .map_err(|error| error.to_string())?;
            }
            if let Some(path) = &args.preview_out
                && !snapshot_written
            {
                preview::save_snapshot(&canvas, path).map_err(|error| error.to_string())?;
                snapshot_written = true;
            }
            if show_preview {
                let rendered = preview::render_terminal(&canvas, args.render_width)
                    .map_err(|error| error.to_string())?;
                print!("\x1b[2J\x1b[H{rendered}");
                print!(
                    "帧 {frames_seen}  {:?}{}  捕获 {captures}/{max_captures}  检测 {}  尝试 {}  \
                     清晰度 {}",
                    outcome.state,
                    outcome
                        .measurement
                        .map(|measurement| format!("  {}", measurement.summary()))
                        .unwrap_or_default(),
                    gate.detections,
                    gate.attempts,
                    outcome
                        .quality
                        .as_ref()
                        .map(|quality| format!(
                            "{:.0}{}",
                            quality.focus_score,
                            if quality.accepted { "" } else { " 拒绝" }
                        ))
                        .unwrap_or_else(|| "-".to_owned())
                );
                std::io::stdout().flush().ok();
            }
        }
        if !outcome.triggered() {
            continue;
        }

        // ---- 触发：取证据帧 → 检测 → 入库 ----
        let measurement = outcome
            .measurement
            .ok_or("triggered without a pose measurement")?;
        gate.on_attempt(now);
        let evidence_frame = match raw.fetch(None) {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                eprintln!("证据帧服务返回空帧，跳过本次触发");
                continue;
            }
            Err(error) => {
                eprintln!("证据帧取图失败：{error}，跳过本次触发");
                continue;
            }
        };
        if (evidence_frame.header.width, evidence_frame.header.height) != image_size {
            eprintln!(
                "证据帧尺寸 {}x{} 与配置 {}x{} 不符，丢弃",
                evidence_frame.header.width,
                evidence_frame.header.height,
                image_size.0,
                image_size.1
            );
            continue;
        }
        let evidence_tile = rigcal_io::rtsp::MonoTile {
            width: evidence_frame.header.width,
            height: evidence_frame.header.height,
            index: evidence_frame.header.frame_id,
            gray: evidence_frame.gray,
            pts_ns: None, // 证据帧不经过解码器
            board_timestamp_ns: Some(evidence_frame.header.camera_timestamp_ns as i64),
        };
        if let Err(error) = write_evidence_png(&evidence_tile, &capture_dir, captures + 1) {
            eprintln!("证据帧落盘失败：{error}");
        }
        let evidence_mat = live::tile_to_mat(&evidence_tile).map_err(|error| error.to_string())?;
        let detection = detect::detect_board(&evidence_mat, board, evidence_border_bits)
            .map_err(|error| format!("证据帧检测失败：{error}"))?;
        if let (Some(bits), None) = (detection.border_bits, evidence_border_bits) {
            evidence_border_bits = Some(bits);
        }
        if !detection.detected() {
            eprintln!(
                "证据帧未检出板（{}，tags={}），跳过",
                detection.status,
                detection.tag_ids.len()
            );
            continue;
        }
        let quality = classify_frame_quality(&evidence_mat, &quality_thresholds(config))
            .map_err(|error| format!("quality gate failed: {error}"))?;
        if !quality.accepted {
            eprintln!(
                "证据帧质量门禁拒绝：{:?}（清晰度 {:.1}，对比度 {:.1}，削顶 {:.3}），跳过",
                quality.reasons, quality.focus_score, quality.contrast, quality.saturated_fraction
            );
            continue;
        }
        captures += 1;
        gate.on_capture(measurement);
        let observation = Observation {
            object_points: detection.object_points.clone(),
            image_points: detection.image_points.clone(),
        };
        capture_log.push(serde_json::json!({
            "capture": captures,
            "frame_id": evidence_frame.header.frame_id,
            "camera_timestamp_ns": evidence_frame.header.camera_timestamp_ns,
            "group_timestamp_ns": evidence_frame.header.group_timestamp_ns,
            "tags": detection.tag_ids.len(),
            "corners": detection.image_points.len(),
            "border_bits": detection.border_bits,
            "guide_rms_px": measurement.rms_px,
            "novelty": outcome.novelty,
            "quality": {
                "focus_score": quality.focus_score,
                "contrast": quality.contrast,
                "saturated_fraction": quality.saturated_fraction,
            },
            "elapsed_s": started.elapsed().as_secs_f64(),
        }));
        let label = format!(
            "（证据 #{} id={} tags={} novelty={}）",
            captures,
            evidence_frame.header.frame_id,
            detection.tag_ids.len(),
            outcome
                .novelty
                .map(|value| format!("{value:.2}"))
                .unwrap_or_else(|| "-".to_owned())
        );
        for runner in runners.iter_mut() {
            runner.observe(observation.clone(), &label, thresholds);
        }
    }

    source.stop();
    let log_path = args.out.join("captures.jsonl");
    let log_text = capture_log
        .iter()
        .map(|entry| entry.to_string())
        .collect::<Vec<_>>()
        .join("\n");
    std::fs::write(&log_path, format!("{log_text}\n"))
        .map_err(|error| format!("cannot write {}: {error}", log_path.display()))?;

    let mut results = Vec::new();
    for runner in runners {
        results.push(runner.finish(&models_dir)?);
    }
    println!(
        "\n在线小结：引导帧 {frames_seen}，检测 {} 次，取图尝试 {} 次，证据帧入库 {captures} 张，用时 {:.1}s",
        gate.detections,
        gate.attempts,
        started.elapsed().as_secs_f64()
    );
    println!(
        "结果目录：{}（captures.jsonl 记录每张证据帧）",
        args.out.display()
    );
    let all_converged = !results.is_empty() && results.iter().all(|result| result.converged);
    if all_converged {
        println!(
            "RIGCAL_CAMERA CONVERGED models={:?}",
            results
                .iter()
                .map(|result| result.model.as_str())
                .collect::<Vec<_>>()
        );
        Ok(std::process::ExitCode::SUCCESS)
    } else {
        println!(
            "RIGCAL_CAMERA INCOMPLETE failed={:?}",
            results
                .iter()
                .filter(|result| !result.converged)
                .map(|result| result.model.as_str())
                .collect::<Vec<_>>()
        );
        Ok(std::process::ExitCode::from(1))
    }
}

/// 证据帧落盘（PNG）：现场复盘"当时到底拍到了什么"，也便于离线重放。
fn write_evidence_png(
    tile: &rigcal_io::rtsp::MonoTile,
    directory: &Path,
    index: usize,
) -> Result<(), String> {
    let canvas = live::tile_to_mat(tile).map_err(|error| error.to_string())?;
    let path = directory.join(format!("evidence_{index:04}.png"));
    preview::save_snapshot(&canvas, &path.to_string_lossy()).map_err(|error| error.to_string())
}
