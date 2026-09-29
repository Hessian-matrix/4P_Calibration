//! DS（Double Sphere）求解后端：桥到 scipy 参考实现。
//!
//! OpenCV 没有 DS 模型（只有 fisheye=KB4、omnidir=UCM/Mei），所以 DS 另有一条后端：第三方 LM
//! 加本仓 DS 模型（见 `problem.rs`）与这里的 scipy 参考桥。本模块把 `tools/ds_solver.py`
//! （scipy 参考实现）包成 `SolveBackend`：请求/响应各一行 JSON 过 `stdin`/`stdout`，
//! **桥本身不做数值**——所以它的结果是参考实现的结果，而不是第二份求解器。
//!
//! 该参考桥冷启动较慢，仅显式启用，不作为原生后端的自动回退。
//!
//! 协议：`stdin` 一行 JSON → `stdout` 一行 JSON（字段见 `tools/ds_solver.py` 头部注释）。

use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::time::Duration;

use rigcal_core::estimator::{IntrinsicPose, Observation, SolveOutcome, SolveRequest, Status};
use rigcal_core::models::{ModelKind, Parameters};

#[derive(Debug, thiserror::Error)]
pub enum DsSolveError {
    #[error("DS 求解后端只处理 ds 模型，收到 {0:?}")]
    WrongModel(ModelKind),
    #[error("找不到 DS 求解脚本（{0}）：设 RIGCAL_DS_SOLVER 指向 tools/ds_solver.py")]
    ScriptMissing(PathBuf),
    #[error("找不到 Python 解释器（{0}）：设 RIGCAL_DS_PYTHON（脚本需要 numpy + scipy + opencv）")]
    PythonMissing(String),
    #[error("启动 DS 求解进程失败：{0}")]
    Spawn(String),
    #[error("DS 求解器退出码 {code}：{stderr}")]
    SolverFailed { code: i32, stderr: String },
    #[error("DS 求解器输出无法解析：{0}")]
    BadOutput(String),
    #[error("DS 求解器返回的参数量不对：{0}")]
    BadParameterCount(usize),
}

/// 桥的配置：解释器、脚本路径、多初值候选（来自 `solver.ds_initial_candidates`）。
#[derive(Clone, Debug)]
pub struct DsBridgeOptions {
    pub python: String,
    pub script: PathBuf,
    pub candidates: Vec<[f64; 2]>,
    pub timeout: Duration,
}

impl DsBridgeOptions {
    /// 默认：`RIGCAL_DS_PYTHON` / `RIGCAL_DS_SOLVER` 覆盖；
    /// 脚本默认取仓库内 `tools/ds_solver.py`（按可执行文件与当前目录两级查找）。
    pub fn discover(candidates: Vec<[f64; 2]>) -> Result<Self, DsSolveError> {
        let python = std::env::var("RIGCAL_DS_PYTHON").unwrap_or_else(|_| "python3".to_owned());
        if let Ok(path) = std::env::var("RIGCAL_DS_SOLVER") {
            let script = PathBuf::from(path);
            if !script.exists() {
                return Err(DsSolveError::ScriptMissing(script));
            }
            return Ok(Self {
                python,
                script,
                candidates,
                timeout: Duration::from_secs(600),
            });
        }
        let mut roots: Vec<PathBuf> = Vec::new();
        if let Ok(cwd) = std::env::current_dir() {
            roots.push(cwd);
        }
        roots.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."));
        for root in roots {
            let script = root.join("tools/ds_solver.py");
            if script.exists() {
                return Ok(Self {
                    python,
                    script,
                    candidates,
                    timeout: Duration::from_secs(600),
                });
            }
        }
        Err(DsSolveError::ScriptMissing(PathBuf::from(
            "tools/ds_solver.py",
        )))
    }
}

/// 调 `tools/ds_solver.py` 解 DS：带上热启动参数/位姿，把优化后的参数、位姿与残差带回来。
pub fn solve_ds_via_scipy(
    request: SolveRequest<'_>,
    options: &DsBridgeOptions,
) -> Result<SolveOutcome, DsSolveError> {
    if request.kind != ModelKind::Ds {
        return Err(DsSolveError::WrongModel(request.kind));
    }
    let payload = serde_json::json!({
        "image_size": [request.image_size.0, request.image_size.1],
        "initial": request.initial.map(|params| params.as_vector()),
        "candidates": options.candidates,
        "initial_poses": request.initial_poses.map(|poses| {
            poses
                .iter()
                .map(|(rvec, tvec)| serde_json::json!([rvec, tvec]))
                .collect::<Vec<_>>()
        }),
        "observations": request
            .observations
            .iter()
            .map(|observation| serde_json::json!({
                "object_points": observation.object_points,
                "image_points": observation.image_points,
            }))
            .collect::<Vec<_>>(),
    });
    let mut child = Command::new(&options.python)
        .arg(&options.script)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|error| match error.kind() {
            std::io::ErrorKind::NotFound => DsSolveError::PythonMissing(options.python.clone()),
            _ => DsSolveError::Spawn(error.to_string()),
        })?;
    child
        .stdin
        .as_mut()
        .ok_or_else(|| DsSolveError::Spawn("无法写入求解器 stdin".to_owned()))?
        .write_all(payload.to_string().as_bytes())
        .map_err(|error| DsSolveError::Spawn(error.to_string()))?;
    let output = child
        .wait_with_output()
        .map_err(|error| DsSolveError::Spawn(error.to_string()))?;
    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr).trim().to_owned();
        return Err(DsSolveError::SolverFailed {
            code: output.status.code().unwrap_or(-1),
            stderr: if stderr.is_empty() {
                "（无 stderr）".to_owned()
            } else {
                stderr
            },
        });
    }
    let stdout = String::from_utf8_lossy(&output.stdout);
    let parsed: serde_json::Value = serde_json::from_str(stdout.trim())
        .map_err(|error| DsSolveError::BadOutput(format!("{error}：{}", stdout.trim())))?;
    let values: Vec<f64> = parsed["parameters"]
        .as_array()
        .ok_or_else(|| DsSolveError::BadOutput("缺少 parameters".to_owned()))?
        .iter()
        .map(|value| value.as_f64().unwrap_or(f64::NAN))
        .collect();
    if values.iter().any(|value| !value.is_finite()) {
        return Err(DsSolveError::BadOutput("parameters 含非有限值".to_owned()));
    }
    let parameters = Parameters::from_vector(ModelKind::Ds, &values)
        .ok_or(DsSolveError::BadParameterCount(values.len()))?;
    let rms_px = parsed["rms_px"].as_f64().unwrap_or(f64::NAN);
    let invalid_projection_count = parsed["invalid_projection_count"]
        .as_u64()
        .and_then(|value| usize::try_from(value).ok())
        .ok_or_else(|| DsSolveError::BadOutput("缺少合法的 invalid_projection_count".to_owned()))?;
    let poses = parse_poses(&parsed, request.observations.len())?;
    let residuals_px = parse_residuals(&parsed, request.observations, invalid_projection_count)?;
    // 求解器自报的 status 必须被**尊重**：显式 FAIL 不能因为 rms 有限而被升级成 PASS；
    // 反过来，无效投影 > 0 或 rms 非有限也一律 FAIL（fail closed）。
    let status = if parsed["status"].as_str() == Some("PASS")
        && invalid_projection_count == 0
        && rms_px.is_finite()
    {
        Status::Pass
    } else {
        Status::Fail
    };
    Ok(SolveOutcome {
        parameters,
        poses,
        residuals_px,
        invalid_projection_count,
        rms_px,
        status,
    })
}

/// 解析 `poses`：必须与观测同长、同序，且每项为有限的 `[rvec, tvec]`。
fn parse_poses(
    parsed: &serde_json::Value,
    expected: usize,
) -> Result<Vec<IntrinsicPose>, DsSolveError> {
    let entries = parsed["poses"]
        .as_array()
        .ok_or_else(|| DsSolveError::BadOutput("缺少 poses".to_owned()))?;
    if entries.len() != expected {
        return Err(DsSolveError::BadOutput(format!(
            "poses 数量 {} 与观测数 {expected} 不一致",
            entries.len()
        )));
    }
    entries
        .iter()
        .map(|entry| {
            let pair = entry
                .as_array()
                .filter(|pair| pair.len() == 2)
                .ok_or_else(|| {
                    DsSolveError::BadOutput(format!("poses 项不是 [rvec,tvec]：{entry}"))
                })?;
            Ok((
                parse_vec3(&pair[0], "poses.rvec")?,
                parse_vec3(&pair[1], "poses.tvec")?,
            ))
        })
        .collect()
}

/// 解析 `residuals_px`：逐点 `[du, dv]` 且有限；无无效投影时必须逐点给出（数量 = 总点数）。
fn parse_residuals(
    parsed: &serde_json::Value,
    observations: &[Observation],
    invalid_projection_count: usize,
) -> Result<Vec<[f64; 2]>, DsSolveError> {
    let entries = parsed["residuals_px"]
        .as_array()
        .ok_or_else(|| DsSolveError::BadOutput("缺少 residuals_px".to_owned()))?;
    let mut residuals = Vec::with_capacity(entries.len());
    for entry in entries {
        let pair = entry
            .as_array()
            .filter(|pair| pair.len() == 2)
            .ok_or_else(|| {
                DsSolveError::BadOutput(format!("residuals_px 项不是 [du,dv]：{entry}"))
            })?;
        match (pair[0].as_f64(), pair[1].as_f64()) {
            (Some(du), Some(dv)) if du.is_finite() && dv.is_finite() => residuals.push([du, dv]),
            _ => {
                return Err(DsSolveError::BadOutput(format!(
                    "residuals_px 含非有限值：{entry}"
                )));
            }
        }
    }
    let total_points: usize = observations
        .iter()
        .map(|observation| observation.object_points.len())
        .sum();
    if total_points.checked_sub(invalid_projection_count) != Some(residuals.len()) {
        return Err(DsSolveError::BadOutput(format!(
            "residuals_px 数量 {} 与总点数 {total_points} / 无效点数 {invalid_projection_count} 不一致",
            residuals.len()
        )));
    }
    Ok(residuals)
}

/// 解析一个有限的三元向量。
fn parse_vec3(value: &serde_json::Value, what: &str) -> Result<[f64; 3], DsSolveError> {
    let entries = value
        .as_array()
        .filter(|entries| entries.len() == 3)
        .ok_or_else(|| DsSolveError::BadOutput(format!("{what} 不是三元数组：{value}")))?;
    let mut out = [0.0; 3];
    for (slot, entry) in out.iter_mut().zip(entries) {
        *slot = entry
            .as_f64()
            .ok_or_else(|| DsSolveError::BadOutput(format!("{what} 含非数值：{value}")))?;
    }
    if out.iter().any(|value| !value.is_finite()) {
        return Err(DsSolveError::BadOutput(format!(
            "{what} 含非有限值：{value}"
        )));
    }
    Ok(out)
}

/// 解释器与脚本**路径**是否可见（启动期自检用）。
///
/// 只检查可执行文件/脚本是否存在，**不**探测 numpy / scipy / cv2 是否装好：真正的导入错误会
/// 等求解进程启动后暴露（stderr 随 [`DsSolveError::SolverFailed`] 带出）。
pub fn ds_backend_available(options: &DsBridgeOptions) -> bool {
    (Path::new(&options.python).exists() || which(&options.python)) && options.script.exists()
}

fn which(program: &str) -> bool {
    std::env::var_os("PATH")
        .map(|paths| std::env::split_paths(&paths).any(|dir| dir.join(program).exists()))
        .unwrap_or(false)
}
