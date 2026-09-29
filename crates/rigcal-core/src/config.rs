//! 单一配置文件的 schema 与校验（单相机内参与四路 rig 共用一套）。
//!
//! **一份文件、一种写法**：`cameras` 列出参与标定的相机（1 路 = 单相机产线，≥2 路 = 四路 rig），
//! 引导源逐路给，证据服务全局一个。没有「模式」二选一，也没有 `device`/`capture` 这类只在某一种
//! 模式下才写的段落——同一台设备的配置在单相机与四路之间只是 `cameras` 的长度不同。
//!
//! **精简原则**：只保留「会改变标定结果或门禁」的项。未知键一律拒绝
//! （`deny_unknown_fields`）：静默忽略会让人以为「配了门禁」，实际没生效。
//!
//! 外参约束与门禁只对多路有意义，因此 `extrinsics` 段**仅** ≥2 路时允许：单相机文件里出现它
//! 直接报错，而不是静默忽略。

use serde::{Deserialize, Serialize};

use crate::board::{AprilGridConfig, Corner};
use crate::models::ModelKind;

pub const SCHEMA_VERSION: u32 = 2;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub schema_version: u32,
    /// 设备/工装标识：单相机与四路用同一个字段，写入产物用于追溯。
    pub rig_id: String,
    /// `[width, height]`，像素；引导、证据与配置必须一致。
    pub image_size: [u32; 2],
    /// 参与标定的相机：1 路 = 单相机产线，≥2 路 = 四路 rig。
    pub cameras: Vec<Camera>,
    /// 证据帧来源（板端 raw 服务）。≥2 路必填；单相机离线回放可省略。
    #[serde(default)]
    pub evidence: Option<EvidenceSource>,
    /// 外参图约束与门禁；仅 ≥2 路允许。
    #[serde(default)]
    pub extrinsics: Option<Extrinsics>,
    pub board: BoardSection,
    pub guidance: Guidance,
    pub solver: Solver,
    pub output: Output,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    /// `camN`：N 同时是板端 raw 的默认通道号。
    pub id: String,
    /// 引导帧源（RTSP / 本地视频）：只用于单帧质量、检测与预览，不产生标定输入。
    pub guidance: GuidanceSource,
    /// 板端 raw 服务的通道号；缺省取 `id` 里的 N（重编号/演练时可显式覆盖）。
    #[serde(default)]
    pub channel: Option<u32>,
}

impl Camera {
    /// 该相机在板端 raw 服务上的通道号：显式 `channel` 优先，否则取 `camN` 的 N。
    pub fn channel(&self) -> u32 {
        self.channel
            .unwrap_or_else(|| self.id[3..].parse().unwrap_or(0))
    }
}

/// 多路 rig 的外参图约束与门禁。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Extrinsics {
    /// 必须成立的外参边（无向）；默认相邻链 `cam0-cam1, cam1-cam2, cam2-cam3`。
    #[serde(default = "default_required_edges")]
    pub required_edges: Vec<[String; 2]>,
    /// 必须闭合的环路；默认整环 `cam0-cam1-cam2-cam3`。
    #[serde(default = "default_required_cycles")]
    pub required_cycles: Vec<Vec<String>>,
    #[serde(default = "default_min_groups_per_edge")]
    pub min_groups_per_edge: usize,
    /// 外参门禁：边重投影 rms、环路旋转/平移误差。
    pub max_edge_rms_px: f64,
    pub max_cycle_rotation_deg: f64,
    pub max_cycle_translation_mm: f64,
}

fn default_required_edges() -> Vec<[String; 2]> {
    vec![
        ["cam0".to_owned(), "cam1".to_owned()],
        ["cam1".to_owned(), "cam2".to_owned()],
        ["cam2".to_owned(), "cam3".to_owned()],
    ]
}

fn default_required_cycles() -> Vec<Vec<String>> {
    vec![vec![
        "cam0".to_owned(),
        "cam1".to_owned(),
        "cam2".to_owned(),
        "cam3".to_owned(),
    ]]
}

fn default_min_groups_per_edge() -> usize {
    3
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceSource {
    pub host: String,
    pub port: u16,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum GuidanceSource {
    /// RTSP 拉流（现场）。
    Rtsp { url: String },
    /// 本地视频/容器文件：与 RTSP 走**同一条解码路径**（离线回放用）。
    Video { path: String },
}

/// 引导门禁参数。
///
/// `jitter_*` 只用作**新颖度尺度**：把当前候选姿态的平移/旋转换算成相对上一张**成功取图**姿态
/// 的归一化偏差（同组默认同曝光，不要求画面静止或相邻帧位姿稳定）。
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Guidance {
    /// 检测尺度：0.75 是「检出率」与「检测成本」之间的折中。
    pub detect_scale: f64,
    /// 搜索态（还没检出板）的检测节奏上限。
    pub detect_hz: f64,
    /// 跟踪态（刚检出过板）的检测节奏上限。
    pub detect_hz_max: f64,
    /// 新颖度尺度：位移按深度归一化后的单位容差（越大越严）。
    pub jitter_xyz: f64,
    /// 新颖度尺度：深度方向的单位容差。
    pub jitter_z: f64,
    /// 新颖度尺度：相对旋转向量各轴的单位容差（度）。
    pub jitter_rotation_deg: f64,
    /// 取图新颖度：把单位容差放大这么多倍，只有真正的「新姿态」才值得抓。
    pub trigger_novelty_scale: f64,
    /// 两次取图尝试之间的最小间隔。
    pub trigger_min_interval_s: f64,
}

impl Default for Guidance {
    fn default() -> Self {
        Self {
            detect_scale: 0.75,
            detect_hz: 0.5,
            detect_hz_max: 12.0,
            jitter_xyz: 0.025,
            jitter_z: 0.04,
            jitter_rotation_deg: 2.0,
            trigger_novelty_scale: 3.0,
            trigger_min_interval_s: 0.5,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardSection {
    pub target_type: String,
    pub target_id: String,
    pub measured: bool,
    pub rows: usize,
    pub cols: usize,
    #[serde(default)]
    pub first_tag_id: usize,
    pub tag_corner_order: Vec<String>,
    pub tag_size_m: f64,
    pub tag_spacing_ratio: f64,
    #[serde(default = "default_dictionary")]
    pub dictionary: String,
}

fn default_dictionary() -> String {
    "DICT_APRILTAG_36h11".to_owned()
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Solver {
    /// 要解的模型；顺序即报告顺序（单相机产线默认两个都解，便于比较）。
    pub models: Vec<ModelKind>,
    pub min_observations: usize,
    pub max_solve_observations: usize,
    pub holdout_fraction: f64,
    pub official_holdout_frames: usize,
    pub max_holdout_rms_px: f64,
    pub max_holdout_p95_px: f64,
    pub quality: Quality,
    /// DS 多初值候选 `(xi, alpha)`：DS 单起点会落进局部极小，故给出候选表。
    #[serde(default = "default_ds_candidates")]
    pub ds_initial_candidates: Vec<[f64; 2]>,
}

fn default_ds_candidates() -> Vec<[f64; 2]> {
    vec![[-0.2, 0.35], [0.0, 0.5], [0.2, 0.65]]
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Quality {
    /// 清晰度下限：原始分辨率上的 Tenengrad（Sobel 梯度平方均值，见 `rigcal_opencv::quality`）。
    /// 初值，需现场按本机噪声/景深调优。
    pub min_focus_score: f64,
    pub min_contrast: f64,
    pub max_saturated_fraction: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Output {
    pub root: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl std::error::Error for ConfigError {}

fn invalid(message: impl Into<String>) -> ConfigError {
    ConfigError(message.into())
}

impl Config {
    /// 解析 + 校验；任何越界都在这里拒绝，不做静默夹取。
    pub fn from_yaml(text: &str) -> Result<Self, ConfigError> {
        let config: Config =
            serde_yaml::from_str(text).map_err(|error| invalid(error.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    pub fn validate(&self) -> Result<(), ConfigError> {
        if self.schema_version != SCHEMA_VERSION {
            return Err(invalid(format!(
                "schema_version must be {SCHEMA_VERSION}, got {} \
                 (v1 的 device/capture/rig 写法已合并为 rig_id/image_size/cameras/extrinsics)",
                self.schema_version
            )));
        }
        if self.rig_id.is_empty() {
            return Err(invalid("rig_id must not be empty"));
        }
        if self.image_size[0] == 0 || self.image_size[1] == 0 {
            return Err(invalid(
                "image_size must be [width, height] with positive values",
            ));
        }
        if self.cameras.is_empty() {
            return Err(invalid(
                "cameras must list the participating cameras (1 = 单相机, >=2 = 四路 rig)",
            ));
        }
        let mut seen_ids: Vec<&str> = Vec::new();
        let mut seen_channels: Vec<u32> = Vec::new();
        for camera in &self.cameras {
            if !camera.id.starts_with("cam") || camera.id[3..].parse::<usize>().is_err() {
                return Err(invalid(format!(
                    "cameras.id must look like camN, got {}",
                    camera.id
                )));
            }
            if seen_ids.contains(&camera.id.as_str()) {
                return Err(invalid(format!("cameras repeats {}", camera.id)));
            }
            seen_ids.push(camera.id.as_str());
            let channel = camera.channel();
            if seen_channels.contains(&channel) {
                return Err(invalid(format!(
                    "cameras repeats raw channel {channel} ({} and an earlier camera)",
                    camera.id
                )));
            }
            seen_channels.push(channel);
            match &camera.guidance {
                GuidanceSource::Rtsp { url } if url.is_empty() => {
                    return Err(invalid(format!("cameras[{}].guidance.url must not be empty", camera.id)));
                }
                GuidanceSource::Video { path } if path.is_empty() => {
                    return Err(invalid(format!("cameras[{}].guidance.path must not be empty", camera.id)));
                }
                _ => {}
            }
        }
        if let Some(evidence) = &self.evidence {
            if evidence.host.is_empty() {
                return Err(invalid("evidence.host must not be empty"));
            }
            if evidence.port == 0 {
                return Err(invalid(
                    "evidence.port must be the board raw frame service port",
                ));
            }
        }
        match (&self.extrinsics, self.cameras.len()) {
            (None, 1) => {}
            (None, count) => {
                return Err(invalid(format!(
                    "{count} 路需要 extrinsics 段（required_edges / required_cycles / 外参门禁）"
                )));
            }
            (Some(_), 1) => {
                return Err(invalid(
                    "extrinsics 段只对 ≥2 路有意义；单相机请删除它",
                ));
            }
            (Some(extrinsics), _) => {
                if self.evidence.is_none() {
                    return Err(invalid(
                        "≥2 路需要 evidence（板端 raw 服务）：帧组由它按同一时刻取四路",
                    ));
                }
                for edge in &extrinsics.required_edges {
                    for camera_id in edge {
                        if !seen_ids.contains(&camera_id.as_str()) {
                            return Err(invalid(format!(
                                "extrinsics.required_edges references unknown camera {camera_id}"
                            )));
                        }
                    }
                    if edge[0] == edge[1] {
                        return Err(invalid(
                            "extrinsics.required_edges must join two distinct cameras",
                        ));
                    }
                }
                for cycle in &extrinsics.required_cycles {
                    if cycle.len() < 3 {
                        return Err(invalid(
                            "extrinsics.required_cycles must list at least three cameras",
                        ));
                    }
                    for camera_id in cycle {
                        if !seen_ids.contains(&camera_id.as_str()) {
                            return Err(invalid(format!(
                                "extrinsics.required_cycles references unknown camera {camera_id}"
                            )));
                        }
                    }
                }
                if extrinsics.min_groups_per_edge == 0 {
                    return Err(invalid("extrinsics.min_groups_per_edge must be positive"));
                }
                for (name, value) in [
                    (
                        "extrinsics.max_edge_rms_px",
                        extrinsics.max_edge_rms_px,
                    ),
                    (
                        "extrinsics.max_cycle_rotation_deg",
                        extrinsics.max_cycle_rotation_deg,
                    ),
                    (
                        "extrinsics.max_cycle_translation_mm",
                        extrinsics.max_cycle_translation_mm,
                    ),
                ] {
                    if !value.is_finite() || value <= 0.0 {
                        return Err(invalid(format!("{name} must be positive")));
                    }
                }
            }
        }
        let guidance = &self.guidance;
        if !guidance.detect_scale.is_finite()
            || !(0.0..=1.0).contains(&guidance.detect_scale)
            || guidance.detect_scale <= 0.0
        {
            return Err(invalid("guidance.detect_scale must be within (0, 1]"));
        }
        if !guidance.detect_hz.is_finite() || guidance.detect_hz <= 0.0 {
            return Err(invalid("guidance.detect_hz must be positive"));
        }
        if !guidance.detect_hz_max.is_finite() || guidance.detect_hz_max <= 0.0 {
            return Err(invalid("guidance.detect_hz_max must be positive"));
        }
        for (name, value) in [
            ("guidance.jitter_xyz", guidance.jitter_xyz),
            ("guidance.jitter_z", guidance.jitter_z),
            ("guidance.jitter_rotation_deg", guidance.jitter_rotation_deg),
        ] {
            if !value.is_finite() || value <= 0.0 {
                return Err(invalid(format!("{name} must be positive")));
            }
        }
        if !guidance.trigger_novelty_scale.is_finite() || guidance.trigger_novelty_scale < 1.0 {
            return Err(invalid(
                "guidance.trigger_novelty_scale must be at least 1.0",
            ));
        }
        if !guidance.trigger_min_interval_s.is_finite() || guidance.trigger_min_interval_s < 0.0 {
            return Err(invalid(
                "guidance.trigger_min_interval_s must be non-negative",
            ));
        }
        self.board_config()?;
        if self.solver.models.is_empty() {
            return Err(invalid(
                "solver.models must list at least one model (kb4 | ds)",
            ));
        }
        let mut seen = Vec::new();
        for model in &self.solver.models {
            if seen.contains(model) {
                return Err(invalid(format!("solver.models repeats {}", model.as_str())));
            }
            seen.push(*model);
        }
        if self.solver.min_observations == 0 {
            return Err(invalid("solver.min_observations must be positive"));
        }
        if self.solver.max_solve_observations < self.solver.min_observations {
            return Err(invalid(
                "solver.max_solve_observations must be >= solver.min_observations",
            ));
        }
        if !(0.0..1.0).contains(&self.solver.holdout_fraction) {
            return Err(invalid("solver.holdout_fraction must be within [0, 1)"));
        }
        if self.solver.official_holdout_frames == 0 {
            return Err(invalid("solver.official_holdout_frames must be positive"));
        }
        if !self.solver.max_holdout_rms_px.is_finite() || self.solver.max_holdout_rms_px <= 0.0 {
            return Err(invalid("solver.max_holdout_rms_px must be positive"));
        }
        if !self.solver.max_holdout_p95_px.is_finite() || self.solver.max_holdout_p95_px <= 0.0 {
            return Err(invalid("solver.max_holdout_p95_px must be positive"));
        }
        let quality = &self.solver.quality;
        if !quality.min_focus_score.is_finite() || quality.min_focus_score < 0.0 {
            return Err(invalid(
                "solver.quality.min_focus_score must be finite and non-negative",
            ));
        }
        if !quality.min_contrast.is_finite() || quality.min_contrast < 0.0 {
            return Err(invalid(
                "solver.quality.min_contrast must be finite and non-negative",
            ));
        }
        if !(0.0..=1.0).contains(&quality.max_saturated_fraction) {
            return Err(invalid(
                "solver.quality.max_saturated_fraction must be within [0, 1]",
            ));
        }
        for candidate in &self.solver.ds_initial_candidates {
            let [xi, alpha] = *candidate;
            if !xi.is_finite() || !(-1.0..1.0).contains(&xi) {
                return Err(invalid(
                    "solver.ds_initial_candidates xi must be within (-1, 1)",
                ));
            }
            if !alpha.is_finite() || !(0.0..1.0).contains(&alpha) {
                return Err(invalid(
                    "solver.ds_initial_candidates alpha must be within (0, 1)",
                ));
            }
        }
        if self.output.root.is_empty() {
            return Err(invalid("output.root must not be empty"));
        }
        Ok(())
    }

    /// 板参数（内联 section → 领域配置）；未标 `measured: true` 时拒绝。
    pub fn board_config(&self) -> Result<AprilGridConfig, ConfigError> {
        let section = &self.board;
        if section.target_type != "aprilgrid" {
            return Err(invalid("board.target_type must be aprilgrid"));
        }
        if !section.measured {
            return Err(invalid("board must be measured before calibration"));
        }
        if section.tag_corner_order.len() != 4 {
            return Err(invalid(
                "board.tag_corner_order must contain bottom_left, bottom_right, top_right, top_left exactly once",
            ));
        }
        let mut order = [Corner::BottomLeft; 4];
        for (index, name) in section.tag_corner_order.iter().enumerate() {
            let Some(corner) = Corner::parse(name) else {
                return Err(invalid(format!(
                    "board.tag_corner_order has unknown corner {name}"
                )));
            };
            if order[..index].contains(&corner) {
                return Err(invalid(format!("board.tag_corner_order repeats {name}")));
            }
            order[index] = corner;
        }
        let config = AprilGridConfig {
            rows: section.rows,
            cols: section.cols,
            tag_size_m: section.tag_size_m,
            tag_spacing_ratio: section.tag_spacing_ratio,
            first_tag_id: section.first_tag_id,
            dictionary: section.dictionary.clone(),
            target_id: section.target_id.clone(),
            tag_corner_order: order,
        };
        config
            .validate()
            .map_err(|error| invalid(error.to_string()))?;
        Ok(config)
    }

    pub fn image_size(&self) -> (u32, u32) {
        (self.image_size[0], self.image_size[1])
    }
}
