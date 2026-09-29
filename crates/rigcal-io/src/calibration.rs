//! 四路 rig 标定产物的落盘（本 crate 唯一的外参写盘逻辑）。
//!
//! 一次导出 = `root/exports/run-<本地日期时间>/`，按**性质分三类文件**：
//!
//! - **结果**：`camN.yaml`（每路内参：模型 + 参数名/参数 + 分辨率）与 `extrinsics.yaml`
//!   （四路外参 `T_c0_ci`，约定 `P_c0 = T_c0_ci · P_ci`，`cam0` 为单位阵）。只有数值与自解释
//!   所需的枚举/单位，没有判定、阈值、误差或统计——结果可以被单独取走、单独存档。
//! - **信息**：`info.yaml`。来源（会话/配置引用、相机与证据端点）、板几何、**有效判据阈值**
//!   （配置值与代码默认值合并后的实际数值）、观测量指标（重投影 rms、秩、条件数、留出、
//!   边/环指标）、判定 `status` 与逐条 `checks`。复核不用回看运行日志。
//! - **交换**：`camchain.yaml`（Kalibr 格式，派生自上面的结果与信息），文件头注明派生自哪个
//!   目录、以及判定真值在 `info.yaml`。
//!
//! 时序：先在内存里完成全部校验与序列化，成功后才在 `exports/.staging-<run-id>/` 写全部文件并
//! flush，最后**原子重命名**发布。任何一步失败都不会产生半成品，也不会动到既有导出。
//!
//! 门禁（不新增第二套收敛判据）：`VALIDATED` 只代表「各路内参均 `converged` **且** 每条必需边、
//! 每个必需环的组数与质量阈值全部达标」；否则只要数据完整可导出，就落 `DRAFT` 并把原因同时写进
//! `judgement.warnings`（人读）与 `judgement.checks`（机读）。数据本身不完整（缺相机/状态、未解、
//! 模型不一致、参数或变换非有限非刚体、必需边/环缺失）一律在写盘前 `Err`。

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

use chrono::{DateTime, Local, Utc};
use rigcal_core::config::{Config, GuidanceSource};
use rigcal_core::extrinsics::{
    CamchainTarget, ExtrinsicsError, Matrix4, RigEstimate, camchain_payload, identity4, invert,
    reference_transforms, rotation_of,
};
use rigcal_core::models::{ModelKind, Parameters, project};
use rigcal_core::session::{SessionState, SessionThresholds};
use serde::Serialize;

/// 参考相机（固定 cam0）：`T_c0_ci` 满足 `P_c0 = T_c0_ci · P_ci`。
pub const REFERENCE_CAMERA: &str = "cam0";
/// 上下文与判定（非结果）。
pub const INFO_FILE: &str = "info.yaml";
/// 四路外参（一个文件）。
pub const EXTRINSICS_FILE: &str = "extrinsics.yaml";
/// Kalibr 交换格式。
pub const CAMCHAIN_FILE: &str = "camchain.yaml";
const SCHEMA_VERSION: u32 = 2;
const CONVENTION: &str = "P_c0 = T_c0_ci * P_ci";
/// 刚体判据容差（正交性 / det=1 / 末行）。
const RIGIDITY_TOLERANCE: f64 = 1e-6;

/// 一路内参的文件名：`camN.yaml`。
pub fn intrinsics_file(camera_id: &str) -> String {
    format!("{camera_id}.yaml")
}

/// 一次导出的回执。
#[derive(Clone, Debug)]
pub struct ExportReceipt {
    /// 已发布的 bundle 目录（`root/exports/run-<日期时间>`）。
    pub directory: PathBuf,
    /// `true` = 全部门禁达标（`judgement.status: VALIDATED`）。
    pub validated: bool,
}

#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("configuration has no extrinsics section; a rig export needs >=2 cameras with extrinsics")]
    MissingExtrinsics,
    #[error("rig must configure exactly the exported cameras, found {found:?}")]
    CameraSet { found: Vec<String> },
    #[error("missing session state for {0}")]
    MissingState(String),
    #[error("camera {camera} has no accepted solve")]
    Unsolved { camera: String },
    #[error("cameras disagree on the projection model")]
    ModelInconsistent,
    #[error("camera {camera} parameters are not a valid finite {model} vector")]
    InvalidParameters { camera: String, model: &'static str },
    #[error("camera {camera}: {detail}")]
    InvalidTransform {
        camera: String,
        detail: &'static str,
    },
    #[error("reference graph cameras are not exactly {expected:?}: found {found:?}")]
    IncompleteGraph {
        expected: Vec<String>,
        found: Vec<String>,
    },
    #[error("required edge {camera_a}-{camera_b} is missing from the estimate")]
    MissingEdge { camera_a: String, camera_b: String },
    #[error("required cycle {} is missing from the estimate", cameras.join("-"))]
    MissingCycle { cameras: Vec<String> },
    #[error("could not allocate a unique export directory under exports/")]
    RunIdExhausted,
    #[error("extrinsics unavailable: {0}")]
    Extrinsics(#[from] ExtrinsicsError),
    #[error("configuration: {0}")]
    Config(#[from] rigcal_core::config::ConfigError),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("yaml: {0}")]
    Yaml(#[from] serde_yaml::Error),
}

/// 导出四路标定产物：`root/exports/run-<日期时间>/{camN.yaml, extrinsics.yaml, info.yaml,
/// camchain.yaml}`。
///
/// `states` 为同一帧组快照下四路内参会话状态（键为相机 id），`estimate` 为同一快照的 rig 外参
/// 估计，`groups` 为已入库的共视帧组数，`journal` 为产出该快照的观测日志路径（`observations.jsonl`，
/// 取不到就传 `None`——信息里记 `session: null`，不猜）。
pub fn export_calibration(
    root: &Path,
    config: &Config,
    states: &BTreeMap<String, SessionState>,
    estimate: &RigEstimate,
    groups: usize,
    journal: Option<&Path>,
) -> Result<ExportReceipt, ExportError> {
    let bundle = build_bundle(root, config, states, estimate, groups, journal)?;

    let exports_root = root.join("exports");
    std::fs::create_dir_all(&exports_root)?;
    let (published, staging) = claim_staging(&exports_root, &Local::now())?;
    let written = bundle.write(&staging);
    if let Err(error) = written {
        let _ = std::fs::remove_dir_all(&staging);
        return Err(ExportError::Io(error));
    }
    if let Err(error) = std::fs::rename(&staging, &published) {
        let _ = std::fs::remove_dir_all(&staging);
        return Err(ExportError::Io(error));
    }
    Ok(ExportReceipt {
        directory: published,
        validated: bundle.validated,
    })
}

/// 一次导出要写的全部文件：名字 → 内容。写盘顺序无关，失败即整体丢弃。
struct Bundle {
    files: Vec<(String, String)>,
    validated: bool,
}

impl Bundle {
    fn write(&self, directory: &Path) -> std::io::Result<()> {
        for (name, content) in &self.files {
            write_synced(&directory.join(name), content.as_bytes())?;
        }
        Ok(())
    }
}

fn build_bundle(
    root: &Path,
    config: &Config,
    states: &BTreeMap<String, SessionState>,
    estimate: &RigEstimate,
    groups: usize,
    journal: Option<&Path>,
) -> Result<Bundle, ExportError> {
    config.validate()?;
    let extrinsics = config
        .extrinsics
        .as_ref()
        .ok_or(ExportError::MissingExtrinsics)?;
    if config.cameras.len() < 2 {
        return Err(ExportError::MissingExtrinsics);
    }
    let expected: Vec<String> = config.cameras.iter().map(|camera| camera.id.clone()).collect();
    if states.len() > expected.len() {
        return Err(ExportError::CameraSet {
            found: states.keys().cloned().collect(),
        });
    }

    let mut checked: Vec<(&SessionState, Parameters)> = Vec::with_capacity(expected.len());
    let mut model: Option<ModelKind> = None;
    for camera in &expected {
        let state = states
            .get(camera)
            .ok_or_else(|| ExportError::MissingState(camera.clone()))?;
        if state.solves == 0 {
            return Err(ExportError::Unsolved {
                camera: camera.clone(),
            });
        }
        match model {
            None => model = Some(state.model),
            Some(existing) if existing != state.model => {
                return Err(ExportError::ModelInconsistent);
            }
            Some(_) => {}
        }
        if !state.parameters.iter().all(|value| value.is_finite()) {
            return Err(ExportError::InvalidParameters {
                camera: camera.clone(),
                model: state.model.as_str(),
            });
        }
        let parameters =
            Parameters::from_vector(state.model, &state.parameters).ok_or_else(|| {
                ExportError::InvalidParameters {
                    camera: camera.clone(),
                    model: state.model.as_str(),
                }
            })?;
        // 复用模型自己的有效域规则，不能把有限但无法投影的参数写成标定结果。
        let (_, valid) = project(state.model, &[[0.0, 0.0, 1.0]], &parameters);
        if !valid[0] {
            return Err(ExportError::InvalidParameters {
                camera: camera.clone(),
                model: state.model.as_str(),
            });
        }
        checked.push((state, parameters));
    }
    let model = model.ok_or(ExportError::MissingExtrinsics)?;

    for edge in &estimate.edges {
        if !is_rigid(&edge.transform) {
            return Err(ExportError::InvalidTransform {
                camera: format!("{}->{}", edge.camera_a, edge.camera_b),
                detail: "edge is not a finite rigid transform",
            });
        }
    }
    // 参考图：必须恰好覆盖配置里的相机（不能只覆盖边端点里出现过的相机）。
    let reference = reference_transforms(estimate, REFERENCE_CAMERA)?;
    let found: Vec<String> = reference.keys().cloned().collect();
    if found != expected {
        return Err(ExportError::IncompleteGraph { expected, found });
    }
    for camera in &found {
        let t_ci_c0 = reference[camera];
        if !is_rigid(&t_ci_c0) {
            return Err(ExportError::InvalidTransform {
                camera: camera.clone(),
                detail: "T_ci_c0 is not a finite rigid transform",
            });
        }
        if !is_rigid(&invert(&t_ci_c0)) {
            return Err(ExportError::InvalidTransform {
                camera: camera.clone(),
                detail: "T_c0_ci is not a finite rigid transform",
            });
        }
    }

    let mut parameters_map: HashMap<String, Parameters> = HashMap::new();
    for (camera, (_, parameters)) in expected.iter().zip(&checked) {
        parameters_map.insert(camera.clone(), *parameters);
    }

    // ---- 结果：每路内参一个文件 + 四路外参一个文件 ----
    let thresholds = SessionThresholds::from_config(config);
    let mut files: Vec<(String, String)> = Vec::with_capacity(expected.len() + 4);
    let mut result_files: Vec<String> = Vec::new();
    for (camera, (_, parameters)) in expected.iter().zip(&checked) {
        let name = intrinsics_file(camera);
        files.push((
            name.clone(),
            serde_yaml::to_string(&CameraIntrinsicsDocument {
                schema_version: SCHEMA_VERSION,
                kind: "camera-intrinsics",
                camera_id: camera.clone(),
                model: model.as_str(),
                resolution: [config.image_size[0], config.image_size[1]],
                parameter_names: model.parameter_names().to_vec(),
                parameters: parameters.as_vector(),
            })?,
        ));
        result_files.push(name);
    }
    let mut extrinsics_cameras: BTreeMap<String, ExtrinsicsCamera> = BTreeMap::new();
    for camera in &expected[1..] {
        extrinsics_cameras.insert(
            camera.clone(),
            ExtrinsicsCamera {
                t_c0_ci: invert(&reference[camera]),
            },
        );
    }
    extrinsics_cameras.insert(
        expected[0].clone(),
        ExtrinsicsCamera {
            t_c0_ci: identity4(),
        },
    );
    files.push((
        EXTRINSICS_FILE.to_owned(),
        serde_yaml::to_string(&ExtrinsicsDocument {
            schema_version: SCHEMA_VERSION,
            kind: "rig-extrinsics",
            reference_camera: REFERENCE_CAMERA,
            convention: CONVENTION,
            translation_unit: "m",
            cameras: extrinsics_cameras,
        })?,
    ));
    result_files.push(EXTRINSICS_FILE.to_owned());

    // ---- 信息：来源 + 阈值 + 指标 + 判定 ----
    let mut warnings: Vec<String> = Vec::new();
    let mut checks: Vec<Check> = Vec::new();
    let mut camera_metrics: BTreeMap<String, CameraMetrics> = BTreeMap::new();
    for (camera, (state, _)) in expected.iter().zip(&checked) {
        if !state.converged {
            warnings.push(format!(
                "{camera}: intrinsic session not converged (status {}, streak {})",
                state.status, state.streak
            ));
        }
        push_limit(
            &mut checks,
            &mut warnings,
            camera,
            "rms_px",
            finite(state.rms_px),
            thresholds.max_rms_px,
            Bound::AtMost,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            camera,
            "focal_relative_stddev",
            finite_pair_max(state.focal_relative_stddev),
            thresholds.max_focal_relative_stddev,
            Bound::AtMost,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            camera,
            "principal_stddev_px",
            finite_pair_max(state.principal_stddev_px),
            thresholds.max_principal_stddev_px,
            Bound::AtMost,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            camera,
            "holdout_rms_px",
            finite(state.holdout_rms_px),
            thresholds.max_holdout_rms_px,
            Bound::AtMost,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            camera,
            "holdout_p95_px",
            finite(state.holdout_p95_px),
            thresholds.max_holdout_p95_px,
            Bound::AtMost,
        );
        camera_metrics.insert(camera.clone(), CameraMetrics::from_state(state));
    }

    // 必需边：缺失即拒（不允许"没算过"被当成质量通过）。
    let mut edges: Vec<EdgeMetrics> = Vec::with_capacity(extrinsics.required_edges.len());
    for edge in &extrinsics.required_edges {
        let [camera_a, camera_b] = edge;
        let estimate_edge =
            estimate
                .edge_for(camera_a, camera_b)
                .ok_or_else(|| ExportError::MissingEdge {
                    camera_a: camera_a.clone(),
                    camera_b: camera_b.clone(),
                })?;
        let scope = format!("{camera_a}-{camera_b}");
        push_limit(
            &mut checks,
            &mut warnings,
            &scope,
            "edge_groups",
            Some(estimate_edge.groups as f64),
            extrinsics.min_groups_per_edge as f64,
            Bound::AtLeast,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            &scope,
            "edge_reprojection_rms_px",
            finite(estimate_edge.reprojection_rms_px),
            extrinsics.max_edge_rms_px,
            Bound::AtMost,
        );
        edges.push(EdgeMetrics {
            camera_a: camera_a.clone(),
            camera_b: camera_b.clone(),
            groups: estimate_edge.groups,
            rotation_sigma_deg: finite(estimate_edge.rotation_sigma_deg),
            translation_sigma_mm: finite(estimate_edge.translation_sigma_mm),
            reprojection_rms_px: finite(estimate_edge.reprojection_rms_px),
        });
    }

    // 必需环：同样缺失即拒；质量阈值不达标只降级为 DRAFT。
    let mut cycles: Vec<CycleMetrics> = Vec::with_capacity(extrinsics.required_cycles.len());
    for cycle in &extrinsics.required_cycles {
        let check = estimate
            .cycles
            .iter()
            .find(|check| &check.cameras == cycle)
            .ok_or_else(|| ExportError::MissingCycle {
                cameras: cycle.clone(),
            })?;
        let scope = cycle.join("-");
        push_limit(
            &mut checks,
            &mut warnings,
            &scope,
            "cycle_rotation_error_deg",
            finite(check.rotation_error_deg),
            extrinsics.max_cycle_rotation_deg,
            Bound::AtMost,
        );
        push_limit(
            &mut checks,
            &mut warnings,
            &scope,
            "cycle_translation_error_mm",
            finite(check.translation_error_mm),
            extrinsics.max_cycle_translation_mm,
            Bound::AtMost,
        );
        cycles.push(CycleMetrics {
            cameras: cycle.clone(),
            rotation_error_deg: finite(check.rotation_error_deg),
            translation_error_mm: finite(check.translation_error_mm),
        });
    }

    let validated = warnings.is_empty();
    let info = InfoDocument {
        schema_version: SCHEMA_VERSION,
        kind: "rig-calibration-info",
        exported_at: ExportedAt::now(),
        software: Software {
            name: "rigcal",
            version: env!("CARGO_PKG_VERSION"),
            git_commit: option_env!("RIGCAL_GIT_COMMIT"),
            target: format!("{}-{}", std::env::consts::ARCH, std::env::consts::OS),
        },
        result_files: result_files.clone(),
        session: session_document(root, journal, groups),
        rig: RigDocument {
            rig_id: config.rig_id.clone(),
            image_size: [config.image_size[0], config.image_size[1]],
            cameras: config
                .cameras
                .iter()
                .map(|camera| CameraSource {
                    id: camera.id.clone(),
                    channel: camera.channel(),
                    guidance: match &camera.guidance {
                        GuidanceSource::Rtsp { url } => GuidanceDocument::Rtsp { url },
                        GuidanceSource::Video { path } => GuidanceDocument::Video { path },
                    },
                })
                .collect(),
            evidence: config.evidence.as_ref().map(|evidence| EvidenceDocument {
                host: evidence.host.clone(),
                port: evidence.port,
            }),
        },
        board: BoardDocument {
            target_type: config.board.target_type.clone(),
            target_id: config.board.target_id.clone(),
            dictionary: config.board.dictionary.clone(),
            measured: config.board.measured,
            rows: config.board.rows,
            cols: config.board.cols,
            first_tag_id: config.board.first_tag_id,
            tag_corner_order: config.board.tag_corner_order.clone(),
            tag_size_m: config.board.tag_size_m,
            tag_spacing_ratio: config.board.tag_spacing_ratio,
        },
        thresholds: ThresholdsDocument {
            intrinsics: IntrinsicsThresholds {
                max_rms_px: thresholds.max_rms_px,
                max_focal_relative_stddev: thresholds.max_focal_relative_stddev,
                max_principal_stddev_px: thresholds.max_principal_stddev_px,
                max_holdout_rms_px: thresholds.max_holdout_rms_px,
                max_holdout_p95_px: thresholds.max_holdout_p95_px,
                window: thresholds.window,
            },
            extrinsics: ExtrinsicsThresholds {
                min_groups_per_edge: extrinsics.min_groups_per_edge,
                max_edge_rms_px: extrinsics.max_edge_rms_px,
                max_cycle_rotation_deg: extrinsics.max_cycle_rotation_deg,
                max_cycle_translation_mm: extrinsics.max_cycle_translation_mm,
            },
            solver: SolverThresholds {
                models: config
                    .solver
                    .models
                    .iter()
                    .map(|model| model.as_str())
                    .collect(),
                min_observations: config.solver.min_observations,
                max_solve_observations: config.solver.max_solve_observations,
                holdout_fraction: config.solver.holdout_fraction,
                official_holdout_frames: config.solver.official_holdout_frames,
                ds_initial_candidates: config.solver.ds_initial_candidates.clone(),
            },
            acquisition: AcquisitionThresholds {
                min_focus_score: config.solver.quality.min_focus_score,
                min_contrast: config.solver.quality.min_contrast,
                max_saturated_fraction: config.solver.quality.max_saturated_fraction,
                detect_scale: config.guidance.detect_scale,
                detect_hz: config.guidance.detect_hz,
                detect_hz_max: config.guidance.detect_hz_max,
                jitter_xyz: config.guidance.jitter_xyz,
                jitter_z: config.guidance.jitter_z,
                jitter_rotation_deg: config.guidance.jitter_rotation_deg,
                trigger_novelty_scale: config.guidance.trigger_novelty_scale,
                trigger_min_interval_s: config.guidance.trigger_min_interval_s,
            },
        },
        metrics: MetricsDocument {
            cameras: camera_metrics,
            edges,
            cycles,
            rig_rms_px: finite(estimate.rig_rms_px),
        },
        judgement: JudgementDocument {
            status: if validated {
                ExportStatus::Validated
            } else {
                ExportStatus::Draft
            },
            warnings: warnings.clone(),
            checks,
        },
    };
    files.push((
        INFO_FILE.to_owned(),
        serde_yaml::to_string(&info)?,
    ));

    // ---- 交换格式：Kalibr camchain（派生，头注释指向 info.yaml） ----
    let target = CamchainTarget {
        rows: config.board.rows,
        cols: config.board.cols,
        tag_size_m: config.board.tag_size_m,
        tag_spacing_ratio: config.board.tag_spacing_ratio,
    };
    let payload = camchain_payload(
        estimate,
        REFERENCE_CAMERA,
        &parameters_map,
        model,
        (config.image_size[0], config.image_size[1]),
        target,
    )?;
    let mut camchain = camchain_header(&config.rig_id, validated, &warnings);
    camchain.push_str(&serde_yaml::to_string(&serde_yaml::Value::Mapping(
        payload,
    ))?);
    files.push((CAMCHAIN_FILE.to_owned(), camchain));

    Ok(Bundle { files, validated })
}

/// 判定的方向：外参边「至少多少组」是下限，rms/误差类是上限。
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "snake_case")]
enum Bound {
    AtMost,
    AtLeast,
}

/// 判定用的一条「指标 vs 界」：`scope` 是相机或边/环，`value: null` 表示求解器没给出。
#[derive(Clone, Debug, Serialize)]
struct Check {
    scope: String,
    metric: &'static str,
    value: Option<f64>,
    limit: f64,
    bound: Bound,
    passed: bool,
}

/// 记一条检查；不通过时同时产生一条人读 `warnings`（两者同源，不各写一份）。
fn push_limit(
    checks: &mut Vec<Check>,
    warnings: &mut Vec<String>,
    scope: &str,
    metric: &'static str,
    value: Option<f64>,
    limit: f64,
    bound: Bound,
) {
    let passed = value.is_some_and(|value| match bound {
        Bound::AtMost => value <= limit,
        Bound::AtLeast => value >= limit,
    });
    if !passed {
        match value {
            Some(value) => {
                let comparison = match bound {
                    Bound::AtMost => '>',
                    Bound::AtLeast => '<',
                };
                warnings.push(format!("{scope}: {metric} {value:.4} {comparison} {limit:.4}"));
            }
            None => warnings.push(format!("{scope}: {metric} is not finite")),
        }
    }
    checks.push(Check {
        scope: scope.to_owned(),
        metric,
        value,
        limit,
        bound,
        passed,
    });
}

fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

/// 一对标准差取较严的那个：阈值对两个分量各判一次，这里合并成一条检查。
fn finite_pair_max(values: (f64, f64)) -> Option<f64> {
    (values.0.is_finite() && values.1.is_finite()).then(|| values.0.max(values.1))
}

/// `T_target_source` 是否为有限刚体变换：末行 `[0,0,0,1]`、旋转块正交且 `det = +1`。
fn is_rigid(transform: &Matrix4) -> bool {
    if !transform.iter().flatten().all(|value| value.is_finite()) {
        return false;
    }
    let bottom = [
        transform[3][0],
        transform[3][1],
        transform[3][2],
        transform[3][3],
    ];
    if bottom[0].abs() > RIGIDITY_TOLERANCE
        || bottom[1].abs() > RIGIDITY_TOLERANCE
        || bottom[2].abs() > RIGIDITY_TOLERANCE
        || (bottom[3] - 1.0).abs() > RIGIDITY_TOLERANCE
    {
        return false;
    }
    let rotation = rotation_of(transform);
    for column in 0..3 {
        for other in 0..3 {
            let mut dot = 0.0;
            for row in &rotation {
                dot += row[column] * row[other];
            }
            let expected = if column == other { 1.0 } else { 0.0 };
            if (dot - expected).abs() > RIGIDITY_TOLERANCE {
                return false;
            }
        }
    }
    let determinant = rotation[0][0]
        * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]);
    (determinant - 1.0).abs() <= RIGIDITY_TOLERANCE
}

#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum ExportStatus {
    Validated,
    Draft,
}

// --------------------------------------------------------------------------- #
// 结果文件
// --------------------------------------------------------------------------- #

#[derive(Serialize)]
struct CameraIntrinsicsDocument {
    schema_version: u32,
    kind: &'static str,
    camera_id: String,
    model: &'static str,
    resolution: [u32; 2],
    parameter_names: Vec<&'static str>,
    parameters: Vec<f64>,
}

#[derive(Serialize)]
struct ExtrinsicsDocument {
    schema_version: u32,
    kind: &'static str,
    reference_camera: &'static str,
    convention: &'static str,
    translation_unit: &'static str,
    /// 键为相机 id；`T_c0_ci` 满足 `P_c0 = T_c0_ci · P_ci`，cam0 逐位精确单位阵。
    cameras: BTreeMap<String, ExtrinsicsCamera>,
}

#[derive(Serialize)]
struct ExtrinsicsCamera {
    #[serde(rename = "T_c0_ci")]
    t_c0_ci: Matrix4,
}

// --------------------------------------------------------------------------- #
// 信息文件
// --------------------------------------------------------------------------- #

#[derive(Serialize)]
struct InfoDocument<'a> {
    schema_version: u32,
    kind: &'static str,
    exported_at: ExportedAt,
    software: Software,
    /// 本次导出的结果文件（不含本文件与 camchain）。
    result_files: Vec<String>,
    session: Option<SessionDocument>,
    rig: RigDocument<'a>,
    board: BoardDocument,
    thresholds: ThresholdsDocument,
    metrics: MetricsDocument,
    judgement: JudgementDocument,
}

#[derive(Serialize)]
struct ExportedAt {
    local: String,
    utc: String,
    offset_seconds: i32,
}

impl ExportedAt {
    fn now() -> Self {
        let now = Local::now();
        Self::from_datetime(&now)
    }

    fn from_datetime(now: &DateTime<Local>) -> Self {
        Self {
            local: now.format("%Y-%m-%dT%H:%M:%S%:z").to_string(),
            utc: now.with_timezone(&Utc).format("%Y-%m-%dT%H:%M:%SZ").to_string(),
            offset_seconds: now.offset().local_minus_utc(),
        }
    }
}

#[derive(Serialize)]
struct Software {
    name: &'static str,
    version: &'static str,
    /// 由构建注入（`RIGCAL_GIT_COMMIT`）；未注入记 null，不用别的值冒充。
    git_commit: Option<&'static str>,
    target: String,
}

#[derive(Serialize)]
struct SessionDocument {
    /// 观测日志相对 `output.root` 的路径（不在 root 下时记绝对路径）。
    journal: String,
    /// 同一会话的已解析配置。
    config: Option<String>,
    groups: usize,
}

fn session_document(root: &Path, journal: Option<&Path>, groups: usize) -> Option<SessionDocument> {
    let journal = journal?;
    let relative = |path: &Path| {
        path.strip_prefix(root)
            .unwrap_or(path)
            .to_string_lossy()
            .into_owned()
    };
    Some(SessionDocument {
        journal: relative(journal),
        config: journal
            .parent()
            .map(|directory| relative(&directory.join("config.yaml"))),
        groups,
    })
}

#[derive(Serialize)]
struct RigDocument<'a> {
    rig_id: String,
    image_size: [u32; 2],
    cameras: Vec<CameraSource<'a>>,
    evidence: Option<EvidenceDocument>,
}

#[derive(Serialize)]
struct CameraSource<'a> {
    id: String,
    channel: u32,
    guidance: GuidanceDocument<'a>,
}

#[derive(Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum GuidanceDocument<'a> {
    Rtsp { url: &'a str },
    Video { path: &'a str },
}

#[derive(Serialize)]
struct EvidenceDocument {
    host: String,
    port: u16,
}

#[derive(Serialize)]
struct BoardDocument {
    target_type: String,
    target_id: String,
    dictionary: String,
    measured: bool,
    rows: usize,
    cols: usize,
    first_tag_id: usize,
    tag_corner_order: Vec<String>,
    tag_size_m: f64,
    tag_spacing_ratio: f64,
}

/// **有效阈值**：配置值与代码默认值合并后的实际数值，直接写数值而不是"见配置"。
#[derive(Serialize)]
struct ThresholdsDocument {
    intrinsics: IntrinsicsThresholds,
    extrinsics: ExtrinsicsThresholds,
    solver: SolverThresholds,
    acquisition: AcquisitionThresholds,
}

#[derive(Serialize)]
struct IntrinsicsThresholds {
    max_rms_px: f64,
    max_focal_relative_stddev: f64,
    max_principal_stddev_px: f64,
    max_holdout_rms_px: f64,
    max_holdout_p95_px: f64,
    window: usize,
}

#[derive(Serialize)]
struct ExtrinsicsThresholds {
    min_groups_per_edge: usize,
    max_edge_rms_px: f64,
    max_cycle_rotation_deg: f64,
    max_cycle_translation_mm: f64,
}

#[derive(Serialize)]
struct SolverThresholds {
    models: Vec<&'static str>,
    min_observations: usize,
    max_solve_observations: usize,
    holdout_fraction: f64,
    official_holdout_frames: usize,
    ds_initial_candidates: Vec<[f64; 2]>,
}

#[derive(Serialize)]
struct AcquisitionThresholds {
    min_focus_score: f64,
    min_contrast: f64,
    max_saturated_fraction: f64,
    detect_scale: f64,
    detect_hz: f64,
    detect_hz_max: f64,
    jitter_xyz: f64,
    jitter_z: f64,
    jitter_rotation_deg: f64,
    trigger_novelty_scale: f64,
    trigger_min_interval_s: f64,
}

#[derive(Serialize)]
struct MetricsDocument {
    cameras: BTreeMap<String, CameraMetrics>,
    edges: Vec<EdgeMetrics>,
    cycles: Vec<CycleMetrics>,
    rig_rms_px: Option<f64>,
}

/// 观测量指标与求解器运行结果；求解器尚未给出的量记 `null`，不伪造数值。
#[derive(Serialize)]
struct CameraMetrics {
    views: usize,
    used_views: usize,
    excluded_views: usize,
    holdout_views: usize,
    holdout_invalid: usize,
    rms_px: Option<f64>,
    rank: usize,
    condition_number: Option<f64>,
    focal_relative_stddev: Option<[f64; 2]>,
    principal_stddev_px: Option<[f64; 2]>,
    info_gain: Option<f64>,
    holdout_rms_px: Option<f64>,
    holdout_p95_px: Option<f64>,
    solves: usize,
    failures: usize,
    converged: bool,
    streak: usize,
    status: String,
    detail: String,
}

impl CameraMetrics {
    fn from_state(state: &SessionState) -> Self {
        Self {
            views: state.views,
            used_views: state.used_views,
            excluded_views: state.excluded_views,
            holdout_views: state.holdout_views,
            holdout_invalid: state.holdout_invalid,
            rms_px: finite(state.rms_px),
            rank: state.rank,
            condition_number: finite(state.condition_number),
            focal_relative_stddev: finite_pair(state.focal_relative_stddev),
            principal_stddev_px: finite_pair(state.principal_stddev_px),
            info_gain: state.info_gain.filter(|value| value.is_finite()),
            holdout_rms_px: finite(state.holdout_rms_px),
            holdout_p95_px: finite(state.holdout_p95_px),
            solves: state.solves,
            failures: state.failures,
            converged: state.converged,
            streak: state.streak,
            status: state.status.to_owned(),
            detail: state.detail.clone(),
        }
    }
}

#[derive(Serialize)]
struct EdgeMetrics {
    camera_a: String,
    camera_b: String,
    groups: usize,
    rotation_sigma_deg: Option<f64>,
    translation_sigma_mm: Option<f64>,
    reprojection_rms_px: Option<f64>,
}

#[derive(Serialize)]
struct CycleMetrics {
    cameras: Vec<String>,
    rotation_error_deg: Option<f64>,
    translation_error_mm: Option<f64>,
}

#[derive(Serialize)]
struct JudgementDocument {
    status: ExportStatus,
    warnings: Vec<String>,
    /// 阈值 × 指标的逐条比对：`status: DRAFT` 时这里必有 `passed: false`。
    checks: Vec<Check>,
}

fn finite_pair(values: (f64, f64)) -> Option<[f64; 2]> {
    (values.0.is_finite() && values.1.is_finite()).then_some([values.0, values.1])
}

// --------------------------------------------------------------------------- #
// 目录与写盘
// --------------------------------------------------------------------------- #

/// 导出目录名：本地日期时间（`run-20260929-153012`），人能直接读。
fn run_id(stamp: &DateTime<Local>) -> String {
    format!("run-{}", stamp.format("%Y%m%d-%H%M%S"))
}

/// 唯一 run-id + 私有 staging 目录：`published` 尚不存在时才创建 staging。
fn claim_staging(
    exports_root: &Path,
    stamp: &DateTime<Local>,
) -> Result<(PathBuf, PathBuf), ExportError> {
    let base = run_id(stamp);
    for attempt in 0..10_000u32 {
        let run_id = if attempt == 0 {
            base.clone()
        } else {
            format!("{base}-{attempt}")
        };
        let published = exports_root.join(&run_id);
        if published.exists() {
            continue;
        }
        let staging = exports_root.join(format!(".staging-{run_id}"));
        match std::fs::create_dir(&staging) {
            Ok(()) => return Ok((published, staging)),
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(ExportError::Io(error)),
        }
    }
    Err(ExportError::RunIdExhausted)
}

fn write_synced(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let mut file = std::fs::File::create(path)?;
    file.write_all(bytes)?;
    file.sync_all()
}

fn camchain_header(rig_id: &str, validated: bool, warnings: &[String]) -> String {
    let mut header = String::new();
    header.push_str(&format!("# Kalibr camchain for rig {rig_id}\n"));
    header.push_str(&format!(
        "# derived from this directory's results; judgement lives in {INFO_FILE} (status: {})\n",
        if validated { "VALIDATED" } else { "DRAFT" }
    ));
    for warning in warnings {
        header.push_str(&format!("# warning: {warning}\n"));
    }
    header
}
