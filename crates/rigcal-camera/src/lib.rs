//! 单相机产线的可复用部分：检测、终端/PNG 预览、在线门禁。
//!
//! `rigcal-camera` 既是产线 bin（`src/main.rs`：配置 → 取图 → 会话 → 结果落盘），
//! 也是库：把「检测 / 预览渲染 / 在线门禁」暴露给别的前端复用（当前消费者是
//! `rigcal-gui` 的四路仪表盘）——**同一套实现，不复制第二份**。

pub mod detect;
pub mod live;
pub mod preview;

use rigcal_core::SolveBackend;
use rigcal_core::config::Config;
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_ds::bridge::{DsBridgeOptions, ds_backend_available, solve_ds_via_scipy};
use rigcal_ds::model::DsSeed;
use rigcal_ds::{DsBackendKind, DsBundleOptions, kb4_seed, solve_ds_intrinsics};

/// 单相机与 GUI 共用的后端：KB4 → OpenCV，DS → Rust LM（默认）。
/// DS 首次求解用 KB4 的角度映射与位姿播种，之后复用已经接受的 DS 解。
/// `RIGCAL_DS_BACKEND=scipy` 显式选择参考桥，不发生隐式后端回退。
///
/// 两条 DS 路径都 fail closed。桥启动时检查路径，Python 导入/求解错误原样上报。
pub fn backend_for(model: ModelKind, config: &Config) -> Result<Box<SolveBackend>, String> {
    match model {
        ModelKind::Kb4 => Ok(Box::new(|request| {
            rigcal_opencv::solve_kb4(request).map_err(|error| error.to_string())
        })),
        ModelKind::Ds => match DsBackendKind::from_env()? {
            DsBackendKind::Scipy => {
                let options =
                    DsBridgeOptions::discover(config.solver.ds_initial_candidates.clone())
                        .map_err(|error| error.to_string())?;
                if !ds_backend_available(&options) {
                    return Err(format!(
                        "DS 求解器不可用：解释器 {:?} 或脚本 {:?} 找不到（设 RIGCAL_DS_PYTHON / RIGCAL_DS_SOLVER）",
                        options.python, options.script
                    ));
                }
                Ok(Box::new(move |request| {
                    solve_ds_via_scipy(request, &options).map_err(|error| error.to_string())
                }))
            }
            DsBackendKind::Rust => Ok(Box::new(|request| {
                let bootstrap = if request.initial.is_none() {
                    let outcome = rigcal_opencv::solve_kb4(rigcal_core::SolveRequest {
                        kind: ModelKind::Kb4,
                        observations: request.observations,
                        image_size: request.image_size,
                        initial: None,
                        initial_poses: None,
                    })
                    .map_err(|error| format!("DS 的 KB4 初始化失败：{error}"))?;
                    if outcome.status != rigcal_core::Status::Pass
                        || outcome.invalid_projection_count > 0
                    {
                        return Err("DS 的 KB4 初始化未收敛".to_owned());
                    }
                    Some(outcome)
                } else {
                    None
                };
                let seed = match request.initial {
                    Some(Parameters::Ds(ds)) => DsSeed::Parameters(ds),
                    Some(other) => return Err(format!("DS 求解收到非 DS 初值：{other:?}")),
                    None => kb4_seed(&bootstrap.as_ref().expect("KB4 initialization").parameters)
                        .expect("KB4 seed parameters"),
                };
                solve_ds_intrinsics(
                    request.observations,
                    request.image_size,
                    &DsBundleOptions {
                        seed,
                        initial_poses: bootstrap
                            .as_ref()
                            .map(|result| result.poses.as_slice())
                            .or(request.initial_poses),
                        ..DsBundleOptions::default()
                    },
                )
                .map_err(|error| error.to_string())
            })),
        },
    }
}

/// 校验编译期头文件与已加载的原生库，供所有产线入口在取图和求解前调用。
pub fn native_dependency_info() -> Result<String, String> {
    let ffmpeg = rigcal_io::rtsp::native_dependency_info();
    let opencv = rigcal_opencv::native_dependency_info();
    let valid = ffmpeg.is_ok() && opencv.is_ok();
    let report = format!(
        "{}\n{}",
        ffmpeg.as_deref().unwrap_or_else(|error| error),
        opencv.as_deref().unwrap_or_else(|error| error)
    );
    if valid { Ok(report) } else { Err(report) }
}

/// 只读依赖诊断：版本校验与当前可执行文件的系统链接记录。
///
/// `@rpath`、DLL 名称等仍按系统工具原样显示，不冒充已解析的绝对路径。
pub fn native_dependency_report() -> Result<String, String> {
    let validation = native_dependency_info();
    let valid = validation.is_ok();
    let versions = validation.unwrap_or_else(|error| error);
    let executable = std::env::current_exe().map_err(|error| error.to_string())?;
    let (tool, arguments): (&str, &[&str]) = match std::env::consts::OS {
        "macos" => ("otool", &["-L"]),
        "linux" => ("ldd", &[]),
        "windows" => ("dumpbin", &["/DEPENDENTS"]),
        platform => {
            return Err(format!(
                "{versions}\n不支持在 {platform} 上查询原生库链接记录"
            ));
        }
    };
    let output = std::process::Command::new(tool)
        .args(arguments)
        .arg(&executable)
        .output()
        .map_err(|error| format!("{versions}\n无法运行 {tool} 查询链接记录：{error}"))?;
    if !output.status.success() {
        return Err(format!(
            "{versions}\n{tool} 查询链接记录失败（{}）：{}{}",
            output.status,
            String::from_utf8_lossy(&output.stdout),
            String::from_utf8_lossy(&output.stderr)
        ));
    }
    let report = format!(
        "{versions}\nexecutable: {}\nlinked libraries ({tool}):\n{}",
        executable.display(),
        String::from_utf8_lossy(&output.stdout)
    );
    if valid { Ok(report) } else { Err(report) }
}
