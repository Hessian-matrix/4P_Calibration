//! 四路 rig 标定产物的落盘（本 crate 唯一的外参写盘逻辑）。
//!
//! 产物是**同一份快照**的两个面：
//!
//! - `calibration.yaml`：本仓 schema。逐路给出 `parameter_names` / `parameters`（沿用
//!   [`ModelKind::parameter_names`] 的生产顺序）、分辨率、`T_c0_ci`（约定
//!   `P_c0 = T_c0_ci · P_ci`，`cam0` 为单位阵）与观测量指标，另附边/环指标。
//! - `camchain.yaml`：Kalibr 格式，**直接由** [`camchain_payload`] 生成（方向/模型顺序由
//!   core 负责），只在文件头加一行状态注释。
//!
//! 时序：先在内存里完成全部校验与序列化，成功后才在 `root/exports/<run-id>` 下用私有
//! staging 目录写两个文件并 flush，最后**原子重命名**发布。任何一步失败都不会产生半成品，
//! 也不会动到既有导出。
//!
//! 门禁（不新增第二套收敛判据）：`VALIDATED` 只代表「四路内参均 `converged` **且** 每条必需
//! 边、每个必需环的组数与质量阈值全部达标」；否则只要数据完整可导出，就落 `DRAFT` 并把原因
//! 写进 `warnings` 与 camchain 注释。数据本身不完整（缺相机/状态、未解、模型不一致、参数或
//! 变换非有限非刚体、必需边/环缺失）一律在写盘前 `Err`。

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};

use rigcal_core::config::{Config, ConfigError, RigSection};
use rigcal_core::extrinsics::{
    CamchainTarget, ExtrinsicsError, Matrix4, RigEstimate, camchain_payload, camera_sort_key,
    identity4, invert, reference_transforms, rotation_of,
};
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_core::session::SessionState;
use serde::Serialize;

/// 参考相机（固定 cam0）：`T_c0_ci` 满足 `P_c0 = T_c0_ci · P_ci`。
pub const REFERENCE_CAMERA: &str = "cam0";
pub const CALIBRATION_FILE: &str = "calibration.yaml";
pub const CAMCHAIN_FILE: &str = "camchain.yaml";
const SCHEMA_VERSION: u32 = 1;
const CONVENTION: &str = "P_c0 = T_c0_ci * P_ci";
/// 刚体判据容差（正交性 / det=1 / 末行）。
const RIGIDITY_TOLERANCE: f64 = 1e-6;

/// 一次导出的回执。
#[derive(Clone, Debug)]
pub struct ExportReceipt {
    /// 已发布的 bundle 目录（`root/exports/<run-id>`）。
    pub directory: PathBuf,
    /// `true` = 全部门禁达标（`status: VALIDATED`）。
    pub validated: bool,
}

#[derive(Debug, thiserror::Error)]
pub enum ExportError {
    #[error("configuration has no rig section; a four-camera export requires rig")]
    NotRig,
    #[error("rig must configure exactly cam0..cam3, found {found:?}")]
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
    Config(#[from] ConfigError),
    #[error("io: {0}")]
    Io(#[from] std::io::Error),
    #[error("yaml: {0}")]
    Yaml(#[from] serde_yaml::Error),
}

/// 导出四路标定产物：`root/exports/<run-id>/{calibration,camchain}.yaml`。
///
/// `states` 为同一帧组快照下四路内参会话状态（键为 `cam0..cam3`），`estimate` 为同一快照的
/// rig 外参估计，`groups` 为已入库的共视帧组数。
pub fn export_calibration(
    root: &Path,
    config: &Config,
    states: &BTreeMap<String, SessionState>,
    estimate: &RigEstimate,
    groups: usize,
) -> Result<ExportReceipt, ExportError> {
    let bundle = build_bundle(config, states, estimate, groups)?;

    let exports_root = root.join("exports");
    std::fs::create_dir_all(&exports_root)?;
    let (published, staging) = claim_staging(&exports_root)?;

    let written = write_synced(
        &staging.join(CALIBRATION_FILE),
        bundle.calibration.as_bytes(),
    )
    .and_then(|()| write_synced(&staging.join(CAMCHAIN_FILE), bundle.camchain.as_bytes()));
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

struct Bundle {
    calibration: String,
    camchain: String,
    validated: bool,
}

fn build_bundle(
    config: &Config,
    states: &BTreeMap<String, SessionState>,
    estimate: &RigEstimate,
    groups: usize,
) -> Result<Bundle, ExportError> {
    let rig = config.rig.as_ref().ok_or(ExportError::NotRig)?;
    config.validate()?;
    let expected = expected_cameras(rig)?;
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
        let (_, valid) = rigcal_core::models::project(state.model, &[[0.0, 0.0, 1.0]], &parameters);
        if !valid[0] {
            return Err(ExportError::InvalidParameters {
                camera: camera.clone(),
                model: state.model.as_str(),
            });
        }
        checked.push((state, parameters));
    }
    let model = model.ok_or(ExportError::NotRig)?;

    for edge in &estimate.edges {
        if !is_rigid(&edge.transform) {
            return Err(ExportError::InvalidTransform {
                camera: format!("{}->{}", edge.camera_a, edge.camera_b),
                detail: "edge is not a finite rigid transform",
            });
        }
    }
    // 参考图：必须恰好覆盖 cam0..cam3（不能只覆盖边端点里出现过的相机）。
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

    let mut warnings: Vec<String> = Vec::new();
    let mut cameras: BTreeMap<String, CameraDocument> = BTreeMap::new();
    for (camera, (state, parameters)) in expected.iter().zip(&checked) {
        if !state.converged {
            warnings.push(format!(
                "{camera}: intrinsic session not converged (status {}, streak {})",
                state.status, state.streak
            ));
        }
        let t_c0_ci = if camera == &expected[0] {
            identity4()
        } else {
            invert(&reference[camera])
        };
        cameras.insert(
            camera.clone(),
            CameraDocument {
                parameter_names: model.parameter_names().to_vec(),
                parameters: parameters.as_vector(),
                t_c0_ci,
                quality: CameraQuality::from_state(state),
            },
        );
    }

    // 必需边：缺失即拒（不允许"没算过"被当成质量通过）。
    let mut edges: Vec<EdgeDocument> = Vec::with_capacity(rig.required_edges.len());
    for edge in &rig.required_edges {
        let [camera_a, camera_b] = edge;
        let estimate_edge =
            estimate
                .edge_for(camera_a, camera_b)
                .ok_or_else(|| ExportError::MissingEdge {
                    camera_a: camera_a.clone(),
                    camera_b: camera_b.clone(),
                })?;
        let groups_ok = estimate_edge.groups >= rig.min_groups_per_edge;
        let rms = finite(estimate_edge.reprojection_rms_px);
        let rms_ok = rms.is_some_and(|value| value <= rig.max_edge_rms_px);
        if !groups_ok {
            warnings.push(format!(
                "edge {camera_a}-{camera_b}: {} co-visible group(s) < {}",
                estimate_edge.groups, rig.min_groups_per_edge
            ));
        }
        match rms {
            Some(value) if !rms_ok => warnings.push(format!(
                "edge {camera_a}-{camera_b}: reprojection rms {value:.4} px > {:.4} px",
                rig.max_edge_rms_px
            )),
            None => warnings.push(format!(
                "edge {camera_a}-{camera_b}: reprojection rms is not finite"
            )),
            Some(_) => {}
        }
        edges.push(EdgeDocument {
            camera_a: camera_a.clone(),
            camera_b: camera_b.clone(),
            groups: estimate_edge.groups,
            rotation_sigma_deg: finite(estimate_edge.rotation_sigma_deg),
            translation_sigma_mm: finite(estimate_edge.translation_sigma_mm),
            reprojection_rms_px: rms,
            min_groups: rig.min_groups_per_edge,
            max_reprojection_rms_px: rig.max_edge_rms_px,
            within_limits: groups_ok && rms_ok,
        });
    }

    // 必需环：同样缺失即拒；quality 阈值不达标只降级为 DRAFT。
    let mut cycles: Vec<CycleDocument> = Vec::with_capacity(rig.required_cycles.len());
    for cycle in &rig.required_cycles {
        let check = estimate
            .cycles
            .iter()
            .find(|check| &check.cameras == cycle)
            .ok_or_else(|| ExportError::MissingCycle {
                cameras: cycle.clone(),
            })?;
        let rotation = finite(check.rotation_error_deg);
        let translation = finite(check.translation_error_mm);
        let rotation_ok = rotation.is_some_and(|value| value <= rig.max_cycle_rotation_deg);
        let translation_ok = translation.is_some_and(|value| value <= rig.max_cycle_translation_mm);
        let label = cycle.join("-");
        match rotation {
            Some(value) if !rotation_ok => warnings.push(format!(
                "cycle {label}: rotation error {value:.4}° > {:.4}°",
                rig.max_cycle_rotation_deg
            )),
            None => warnings.push(format!("cycle {label}: rotation error is not finite")),
            Some(_) => {}
        }
        match translation {
            Some(value) if !translation_ok => warnings.push(format!(
                "cycle {label}: translation error {value:.4} mm > {:.4} mm",
                rig.max_cycle_translation_mm
            )),
            None => warnings.push(format!("cycle {label}: translation error is not finite")),
            Some(_) => {}
        }
        cycles.push(CycleDocument {
            cameras: cycle.clone(),
            rotation_error_deg: rotation,
            translation_error_mm: translation,
            max_rotation_deg: rig.max_cycle_rotation_deg,
            max_translation_mm: rig.max_cycle_translation_mm,
            within_limits: rotation_ok && translation_ok,
        });
    }

    let validated = warnings.is_empty();
    let document = CalibrationDocument {
        schema_version: SCHEMA_VERSION,
        rig_id: rig.rig_id.clone(),
        reference_camera: REFERENCE_CAMERA,
        translation_unit: "m",
        convention: CONVENTION,
        status: if validated {
            ExportStatus::Validated
        } else {
            ExportStatus::Draft
        },
        warnings: warnings.clone(),
        groups,
        model: model.as_str(),
        resolution: [config.device.image_size[0], config.device.image_size[1]],
        cameras,
        edges,
        cycles,
        rig_rms_px: finite(estimate.rig_rms_px),
    };
    let calibration = serde_yaml::to_string(&document)?;

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
        (config.device.image_size[0], config.device.image_size[1]),
        target,
    )?;
    let mut camchain = camchain_header(&rig.rig_id, validated, &warnings);
    camchain.push_str(&serde_yaml::to_string(&serde_yaml::Value::Mapping(
        payload,
    ))?);

    Ok(Bundle {
        calibration,
        camchain,
        validated,
    })
}

/// 配置里的相机集合必须**恰好**是 `cam0..cam3`（自然序）。
fn expected_cameras(rig: &RigSection) -> Result<Vec<String>, ExportError> {
    let mut found: Vec<String> = rig
        .cameras
        .iter()
        .map(|camera| camera.camera_id.clone())
        .collect();
    found.sort_by_key(|camera| camera_sort_key(camera));
    let expected: Vec<String> = (0..4).map(|index| format!("cam{index}")).collect();
    if found != expected {
        return Err(ExportError::CameraSet { found });
    }
    Ok(expected)
}

fn camchain_header(rig_id: &str, validated: bool, warnings: &[String]) -> String {
    let mut header = String::new();
    header.push_str(&format!("# Kalibr camchain for rig {rig_id}\n"));
    header.push_str(
        "# T_cn_cnm1 maps P_cnm1 -> P_cn (P_cn = T_cn_cnm1 * P_cnm1); translation unit: m.\n",
    );
    header
        .push_str("# camera order: cam0 < cam1 < cam2 < cam3; cam0 carries no chain transform.\n");
    if validated {
        header.push_str("# status: VALIDATED\n");
    } else {
        header.push_str("# status: DRAFT -- gates not met, not a validated calibration:\n");
        for warning in warnings {
            header.push_str(&format!("#   - {warning}\n"));
        }
    }
    header
}

/// 唯一 run-id + 私有 staging 目录：`published` 尚不存在时才创建 staging。
fn claim_staging(exports_root: &Path) -> Result<(PathBuf, PathBuf), ExportError> {
    let stamp = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|elapsed| elapsed.as_nanos())
        .unwrap_or(0);
    for attempt in 0..10_000u32 {
        let run_id = if attempt == 0 {
            format!("run-{stamp:020}")
        } else {
            format!("run-{stamp:020}-{attempt}")
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

fn finite(value: f64) -> Option<f64> {
    value.is_finite().then_some(value)
}

fn finite_pair(values: (f64, f64)) -> Option<[f64; 2]> {
    (values.0.is_finite() && values.1.is_finite()).then_some([values.0, values.1])
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

#[derive(Serialize)]
struct CalibrationDocument {
    schema_version: u32,
    rig_id: String,
    reference_camera: &'static str,
    translation_unit: &'static str,
    convention: &'static str,
    status: ExportStatus,
    warnings: Vec<String>,
    groups: usize,
    model: &'static str,
    resolution: [u32; 2],
    cameras: BTreeMap<String, CameraDocument>,
    edges: Vec<EdgeDocument>,
    cycles: Vec<CycleDocument>,
    rig_rms_px: Option<f64>,
}

#[derive(Serialize)]
struct CameraDocument {
    parameter_names: Vec<&'static str>,
    parameters: Vec<f64>,
    #[serde(rename = "T_c0_ci")]
    t_c0_ci: Matrix4,
    quality: CameraQuality,
}

/// 观测量指标；求解器尚未给出的量（无 report / 无 holdout）记 `null`，不伪造数值。
#[derive(Serialize)]
struct CameraQuality {
    views: usize,
    used_views: usize,
    excluded_views: usize,
    rms_px: Option<f64>,
    rank: usize,
    condition_number: Option<f64>,
    focal_relative_stddev: Option<[f64; 2]>,
    principal_stddev_px: Option<[f64; 2]>,
    info_gain: Option<f64>,
    holdout_views: usize,
    holdout_rms_px: Option<f64>,
    holdout_p95_px: Option<f64>,
    holdout_invalid: usize,
    solves: usize,
    failures: usize,
    converged: bool,
    streak: usize,
    status: String,
    detail: String,
}

impl CameraQuality {
    fn from_state(state: &SessionState) -> Self {
        Self {
            views: state.views,
            used_views: state.used_views,
            excluded_views: state.excluded_views,
            rms_px: finite(state.rms_px),
            rank: state.rank,
            condition_number: finite(state.condition_number),
            focal_relative_stddev: finite_pair(state.focal_relative_stddev),
            principal_stddev_px: finite_pair(state.principal_stddev_px),
            info_gain: state.info_gain.filter(|value| value.is_finite()),
            holdout_views: state.holdout_views,
            holdout_rms_px: finite(state.holdout_rms_px),
            holdout_p95_px: finite(state.holdout_p95_px),
            holdout_invalid: state.holdout_invalid,
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
struct EdgeDocument {
    camera_a: String,
    camera_b: String,
    groups: usize,
    rotation_sigma_deg: Option<f64>,
    translation_sigma_mm: Option<f64>,
    reprojection_rms_px: Option<f64>,
    min_groups: usize,
    max_reprojection_rms_px: f64,
    within_limits: bool,
}

#[derive(Serialize)]
struct CycleDocument {
    cameras: Vec<String>,
    rotation_error_deg: Option<f64>,
    translation_error_mm: Option<f64>,
    max_rotation_deg: f64,
    max_translation_mm: f64,
    within_limits: bool,
}
