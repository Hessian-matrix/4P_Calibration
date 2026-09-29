//! 在线引导门禁：检测节拍 → 单帧清晰度/质量 → 检测 → 新颖度/冷却 → 触发。
//!
//! - 同组按设备合同视为同曝光，不要求画面静止；
//! - 检测前在原始分辨率上计算 Tenengrad、对比度和削顶，质量不合格则跳过检测；
//! - 搜索态使用 `detect_hz`，跟踪态使用 `detect_hz_max`；
//! - 姿态相对上一张成功入库观测足够新，且尝试冷却已过，才允许触发。
//! - 取图闭环：[`Gate::on_attempt`] 只计尝试与冷却（失败不消耗新颖度），
//!   [`Gate::on_capture`] 才把真正入库的姿态记为新颖度基线。
//!
//! [`Gate::due`] 与 [`Gate::advance`] 分开：调用方（GUI）先 `due` 判断是否值得做
//! `MonoTile → Mat` 转换，避免非检测帧的整帧拷贝。

use std::time::{Duration, Instant};

use opencv::core::Mat;
use opencv::prelude::*;
use rigcal_core::board::AprilGridConfig;
use rigcal_core::config::Guidance;
use rigcal_core::estimator::{Observation, estimate_fixed_intrinsics_pose};
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_core::rotation::relative_rotation_deg;
use rigcal_opencv::quality::{self, FrameQuality, QualityThresholds};

use crate::detect::{self, Detection};
use rigcal_io::rtsp::MonoTile;

/// 解码帧（mono8，无 padding）→ OpenCV Mat（`CV_8UC1`）。
pub fn tile_to_mat(tile: &MonoTile) -> Result<Mat, opencv::Error> {
    let mut mat = Mat::new_rows_cols_with_default(
        tile.height as i32,
        tile.width as i32,
        opencv::core::CV_8UC1,
        opencv::core::Scalar::default(),
    )?;
    {
        let target = mat.data_typed_mut::<u8>()?;
        target.copy_from_slice(&tile.gray);
    }
    Ok(mat)
}

/// "刚检出过板"的判定窗口：决定跟踪态（`detect_hz_max`）还是搜索态（`detect_hz`）。
pub const TRACKING_WINDOW: Duration = Duration::from_millis(500);

#[derive(Debug, thiserror::Error)]
pub enum GateError {
    #[error("opencv: {0}")]
    OpenCv(#[from] opencv::Error),
}

/// 一次检测折算出的位姿测量（引导用，不参与标定）。
#[derive(Clone, Copy, Debug)]
pub struct PoseMeasurement {
    pub rvec: [f64; 3],
    pub tvec: [f64; 3],
    pub rms_px: f64,
    pub tag_count: usize,
}

impl PoseMeasurement {
    /// 预览状态行用：`tags=36 rms=0.42px`。
    pub fn summary(&self) -> String {
        format!("tags={} rms={:.2}px", self.tag_count, self.rms_px)
    }

    fn is_finite(&self) -> bool {
        self.tvec.iter().all(|value| value.is_finite())
            && self.rvec.iter().all(|value| value.is_finite())
    }
}

/// 新颖度尺度：相对一张已成功取图姿态的归一化偏差，`> 1` 即超过单位容差。
///
/// 位移按**深度归一化**后比 `jitter_xyz`，深度方向比 `jitter_z`，姿态取相对旋转向量的
/// **各轴最大偏差**比 `jitter_rotation_deg`。这里只做「够不够新」的度量，不再做运动/静止门禁。
pub fn jitter_score(
    previous: &PoseMeasurement,
    current: &PoseMeasurement,
    guidance: &Guidance,
) -> f64 {
    if !previous.is_finite() || !current.is_finite() {
        return f64::INFINITY;
    }
    let depth_scale = previous.tvec[2].abs().max(current.tvec[2].abs()).max(1.0);
    let lateral = (current.tvec[0] - previous.tvec[0])
        .abs()
        .max((current.tvec[1] - previous.tvec[1]).abs())
        / depth_scale
        / guidance.jitter_xyz;
    let depth = (current.tvec[2] - previous.tvec[2]).abs() / guidance.jitter_z;
    let rotation = relative_rotation_deg(&previous.rvec, &current.rvec);
    let rotation_score = rotation
        .iter()
        .fold(0.0_f64, |worst, value| worst.max(value.abs()))
        / guidance.jitter_rotation_deg;
    lateral.max(depth).max(rotation_score)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateState {
    /// 本帧没到检测时刻（节奏限制）；调用方无需做 Mat 转换。
    Throttled,
    /// 检测了，但没检出板。
    Searching,
    /// 到了检测时刻但画面质量不合格（模糊/低对比/削顶）：跳过检测（省一次开销）。
    WaitingSharp,
    /// 检出板但还不该拍（姿态不够新/冷却未到）。
    Detected,
    /// 检出板且满足触发条件 → 该拍证据帧了。
    Triggered,
}

#[derive(Debug)]
pub struct GateOutcome {
    pub state: GateState,
    /// 本拍的质量度量（`Throttled` 帧没算，故为 `None`）。
    pub quality: Option<FrameQuality>,
    /// 引导帧上的检测结果（预览叠加用）。
    pub detection: Option<Detection>,
    pub measurement: Option<PoseMeasurement>,
    /// 触发时相对上一张成功取图姿态的新颖度分数（诊断用）。
    pub novelty: Option<f64>,
}

impl GateOutcome {
    pub fn triggered(&self) -> bool {
        self.state == GateState::Triggered
    }
}

pub struct Gate {
    guidance: Guidance,
    board: AprilGridConfig,
    thresholds: QualityThresholds,
    parameters: Parameters,
    locked_border_bits: Option<i32>,
    /// 新颖度基线：只在成功取图（入数据集）后更新。
    last_captured_pose: Option<PoseMeasurement>,
    /// 最近一次取图尝试（含失败）：用于冷却。
    last_attempt_at: Option<Instant>,
    last_detection_at: Option<Instant>,
    last_detect_at: Option<Instant>,
    /// 统计：检测次数 / 取图尝试次数。
    pub detections: usize,
    pub attempts: usize,
}

impl Gate {
    pub fn new(
        guidance: &Guidance,
        board: AprilGridConfig,
        image_size: (u32, u32),
        thresholds: QualityThresholds,
    ) -> Result<Self, GateError> {
        // 引导内参固定（等于模型的默认初值）：只用来把像素变到同一度量下比较姿态新颖度，与求解无关。
        let parameters = ModelKind::Kb4.default_parameters(image_size);
        Ok(Self {
            guidance: guidance.clone(),
            board,
            thresholds,
            parameters,
            locked_border_bits: None,
            last_captured_pose: None,
            last_attempt_at: None,
            last_detection_at: None,
            last_detect_at: None,
            detections: 0,
            attempts: 0,
        })
    }

    pub fn tracked_detection_recent(&self, now: Instant) -> bool {
        self.last_detection_at
            .map(|stamp| now.duration_since(stamp) <= TRACKING_WINDOW)
            .unwrap_or(false)
    }

    /// 是否到了检测时刻（按搜索/跟踪态的节奏）。`false` 时调用方不必做 `MonoTile → Mat`。
    pub fn due(&self, now: Instant) -> bool {
        let hz = if self.tracked_detection_recent(now) {
            self.guidance.detect_hz_max
        } else {
            self.guidance.detect_hz
        };
        let period = Duration::from_secs_f64(1.0 / hz);
        self.last_detect_at
            .map(|stamp| now.duration_since(stamp) >= period)
            .unwrap_or(true)
    }

    /// 一次取图尝试（无论后续取帧成功与否）：刷新冷却并计一次尝试。
    /// **不**更新新颖度基线——失败的尝试不能消耗已采姿态。
    pub fn on_attempt(&mut self, now: Instant) {
        self.last_attempt_at = Some(now);
        self.attempts += 1;
    }

    /// 一次成功取图（证据帧通过检测与质量门禁、进入数据集）：记录新颖度基线。
    pub fn on_capture(&mut self, measurement: PoseMeasurement) {
        self.last_captured_pose = Some(measurement);
    }

    /// 推进一帧：节奏 → 质量 →（必要时）检测 → 触发判定。
    pub fn advance(&mut self, frame: &Mat, now: Instant) -> Result<GateOutcome, GateError> {
        if !self.due(now) {
            return Ok(GateOutcome {
                state: GateState::Throttled,
                quality: None,
                detection: None,
                measurement: None,
                novelty: None,
            });
        }
        self.last_detect_at = Some(now);
        let quality = quality::classify_frame_quality(frame, &self.thresholds)?;
        if !quality.accepted {
            return Ok(GateOutcome {
                state: GateState::WaitingSharp,
                quality: Some(quality),
                detection: None,
                measurement: None,
                novelty: None,
            });
        }

        self.detections += 1;
        let detection = detect::detect_board(frame, &self.board, self.locked_border_bits)?;
        if let (Some(bits), None) = (detection.border_bits, self.locked_border_bits) {
            // 首次命中即锁定黑边比特数，之后每帧只跑一遍检测
            self.locked_border_bits = Some(bits);
        }
        if !detection.detected() {
            return Ok(GateOutcome {
                state: GateState::Searching,
                quality: Some(quality),
                detection: Some(detection),
                measurement: None,
                novelty: None,
            });
        }
        self.last_detection_at = Some(now);

        let observation = Observation {
            object_points: detection.object_points.clone(),
            image_points: detection.image_points.clone(),
        };
        let pose = estimate_fixed_intrinsics_pose(ModelKind::Kb4, &self.parameters, &observation);
        let measurement = PoseMeasurement {
            rvec: pose.rvec,
            tvec: pose.tvec,
            rms_px: pose.rms_px,
            tag_count: detection.tag_ids.len(),
        };

        // 新颖度只相对真正入库的姿态：同组/相邻帧本来就在同一姿态附近，不值得重复抓。
        let novelty = self
            .last_captured_pose
            .map(|previous| jitter_score(&previous, &measurement, &self.guidance));
        let novel = novelty
            .map(|score| score > self.guidance.trigger_novelty_scale)
            .unwrap_or(true);
        let cooled = self
            .last_attempt_at
            .map(|stamp| {
                now.duration_since(stamp).as_secs_f64() >= self.guidance.trigger_min_interval_s
            })
            .unwrap_or(true);
        let state = if novel && cooled {
            GateState::Triggered
        } else {
            GateState::Detected
        };
        Ok(GateOutcome {
            state,
            quality: Some(quality),
            detection: Some(detection),
            measurement: Some(measurement),
            novelty,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{Gate, GateState, PoseMeasurement, jitter_score};
    use opencv::core::{Mat, Scalar};
    use opencv::prelude::*;
    use rigcal_core::board::{AprilGridConfig, Corner};
    use rigcal_core::config::Guidance;
    use rigcal_opencv::quality::QualityThresholds;

    fn measurement(tvec: [f64; 3], rvec: [f64; 3]) -> PoseMeasurement {
        PoseMeasurement {
            rvec,
            tvec,
            rms_px: 0.5,
            tag_count: 36,
        }
    }

    fn board() -> AprilGridConfig {
        AprilGridConfig {
            rows: 6,
            cols: 6,
            tag_size_m: 0.088,
            tag_spacing_ratio: 0.3,
            first_tag_id: 0,
            dictionary: "DICT_APRILTAG_36h11".to_owned(),
            target_id: "test".to_owned(),
            tag_corner_order: [
                Corner::BottomRight,
                Corner::BottomLeft,
                Corner::TopLeft,
                Corner::TopRight,
            ],
        }
    }

    fn thresholds() -> QualityThresholds {
        QualityThresholds {
            min_focus_score: 500.0,
            min_contrast: 20.0,
            max_saturated_fraction: 0.35,
        }
    }

    fn gate() -> Gate {
        Gate::new(&Guidance::default(), board(), (1280, 1088), thresholds()).expect("gate")
    }

    /// 合成帧：`values(x, y)` → 灰度。
    fn frame(width: i32, height: i32, values: impl Fn(i32, i32) -> u8) -> Mat {
        let mut mat =
            Mat::new_rows_cols_with_default(height, width, opencv::core::CV_8UC1, Scalar::all(0.0))
                .expect("mat");
        {
            let data = mat.data_typed_mut::<u8>().expect("data");
            for y in 0..height {
                for x in 0..width {
                    data[(y * width + x) as usize] = values(x, y);
                }
            }
        }
        mat
    }

    #[test]
    fn jitter_thresholds_are_exactly_one_at_the_limit() {
        let guidance = Guidance::default();
        // 深度归一化：z=2 m 时横向位移 0.05 m = 2·0.025 → 恰好 1.0
        let base = measurement([0.0, 0.0, 2.0], [0.0, 0.0, 0.0]);
        let lateral = measurement([0.05, 0.0, 2.0], [0.0, 0.0, 0.0]);
        assert!((jitter_score(&base, &lateral, &guidance) - 1.0).abs() < 1e-9);
        // 深度方向阈值更松（jitter_z = 0.04）
        let depth = measurement([0.0, 0.0, 2.04], [0.0, 0.0, 0.0]);
        assert!((jitter_score(&base, &depth, &guidance) - 1.0).abs() < 1e-9);
        // 姿态：绕 y 轴转 2° → 恰好 1.0
        let rotated = measurement([0.0, 0.0, 2.0], [0.0, 2.0_f64.to_radians(), 0.0]);
        assert!((jitter_score(&base, &rotated, &guidance) - 1.0).abs() < 1e-9);
        // 同一姿态 → 0
        assert_eq!(jitter_score(&base, &base, &guidance), 0.0);
        // 非有限值 → 无穷（判定为"很新"，交给冷却兜底）
        let broken = measurement([f64::NAN, 0.0, 2.0], [0.0; 3]);
        assert!(jitter_score(&base, &broken, &guidance).is_infinite());
    }

    #[test]
    fn blurry_frame_is_skipped_without_detection() {
        let mut gate = gate();
        let now = std::time::Instant::now();
        // 平场：质量门禁判低清晰度/低对比度。
        let flat = frame(1280, 1088, |_, _| 128);
        let outcome = gate.advance(&flat, now).expect("advance");
        assert_eq!(outcome.state, GateState::WaitingSharp);
        assert!(!outcome.quality.expect("quality").accepted);
        assert!(outcome.detection.is_none());
        assert_eq!(gate.detections, 0, "模糊帧不应消耗一次检测");
        // 紧接着的帧还没到下一个检测时刻 → Throttled（调用方连 Mat 都不用转换）。
        let outcome = gate.advance(&flat, now).expect("advance");
        assert_eq!(outcome.state, GateState::Throttled);
        assert!(outcome.quality.is_none());
        assert!(!gate.due(now));
    }

    #[test]
    fn sharp_frame_without_board_searches() {
        let mut gate = gate();
        let now = std::time::Instant::now();
        // 高频棋盘：清晰度/对比度合格，但不是 AprilGrid → 只进 Searching。
        let sharp = frame(1280, 1088, |x, y| {
            if ((x / 2) + (y / 2)) % 2 == 0 { 0 } else { 255 }
        });
        let outcome = gate.advance(&sharp, now).expect("advance");
        assert_eq!(outcome.state, GateState::Searching);
        assert!(outcome.quality.expect("quality").accepted);
        assert_eq!(gate.detections, 1);
    }

    #[test]
    fn failed_capture_retries_but_committed_pose_requires_novelty() {
        use opencv::{core::Rect, objdetect};
        use std::time::{Duration, Instant};

        let board_image = |shift: i32| {
            let mut image = frame(1280, 1088, |_, _| 128);
            let dictionary = objdetect::get_predefined_dictionary(
                objdetect::PredefinedDictionaryType::DICT_APRILTAG_36h11,
            )
            .unwrap();
            for row in 0..6 {
                for col in 0..6 {
                    let mut marker = Mat::default();
                    objdetect::generate_image_marker(
                        &dictionary,
                        row * 6 + col,
                        80,
                        &mut marker,
                        1,
                    )
                    .unwrap();
                    let mut target = Mat::roi_mut(
                        &mut image,
                        Rect::new(180 + shift + col * 104, 200 + row * 104, 80, 80),
                    )
                    .unwrap();
                    marker.copy_to(&mut target).unwrap();
                }
            }
            image
        };
        let mut gate = gate();
        let image = board_image(0);
        let now = Instant::now();
        // A sharp first frame needs no multi-frame stability window.
        assert_eq!(
            gate.advance(&image, now).unwrap().state,
            GateState::Triggered
        );
        gate.on_attempt(now);
        let cooled = gate
            .advance(&image, now + Duration::from_millis(100))
            .unwrap();
        assert_eq!(cooled.state, GateState::Detected);
        let retry = gate
            .advance(&image, now + Duration::from_millis(600))
            .unwrap();
        assert_eq!(
            retry.state,
            GateState::Triggered,
            "failed capture must not consume novelty"
        );
        gate.on_capture(retry.measurement.unwrap());
        assert_eq!(
            gate.advance(&image, now + Duration::from_millis(700))
                .unwrap()
                .state,
            GateState::Detected
        );
        // Moving the board between sharp frames is allowed: novelty is not a motion gate.
        let moved = gate
            .advance(&board_image(180), now + Duration::from_millis(800))
            .unwrap();
        assert_eq!(moved.state, GateState::Triggered);
    }
}
