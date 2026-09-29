//! OpenCV 求解后端（适配层）。
//!
//! 分工：`rigcal-core` 是纯 Rust 的算法层（板几何/投影模型/位姿/可观测性/会话闸门），
//! **不做内参求解**；求解交给 OpenCV 的 `calibrateCamera` / `fisheye::calibrate`。
//!
//! 本 crate 只做三件事：把本仓观测转成 OpenCV 的入参，把结果转回 [`rigcal_core::SolveOutcome`]，
//! 以及用 [`native_dependency_info`] 在启动时校验编译期头文件版本与运行库版本是否自洽。

pub mod fisheye;
pub mod quality;

pub use fisheye::{OpenCvSolveError, solve_kb4};
pub use quality::{FrameQuality, QualityThresholds, classify_frame_quality};

/// OpenCV 版本号（major/minor/patch）。
#[derive(Clone, Copy, Debug)]
struct OpenCvVersion {
    major: i32,
    minor: i32,
    patch: i32,
}

impl std::fmt::Display for OpenCvVersion {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{}.{}.{}", self.major, self.minor, self.patch)
    }
}

/// 运行期 OpenCV 依赖自检：**编译期头文件版本**（`CV_VERSION_*` 常量）与**实际加载的运行库版本**
/// （`core::get_version_*`）是否自洽。成功返回一行诊断文本，失败返回可操作的原因。
///
/// 绑定由编译期头文件生成；本项目保守要求相同 major/minor：
/// - 头文件版本必须在 `[4.8, 6)` 内（4.8 才包含 MIP_36h12 字典）；
/// - 运行库与头文件的 major/minor 必须**完全一致**（否则 ABI 可能不同）；
/// - 运行库 patch 不得低于头文件 patch（否则可能缺符号）。
///
/// 成功结果进程内缓存，重复调用只读一次版本号；失败**不缓存**（一次失败的启动不会永久毒化
/// 后续调用）。只读版本号，没有逐帧/逐像素开销，也不会替调用方降级或修正。
pub fn native_dependency_info() -> Result<&'static str, String> {
    if let Some(cached) = NATIVE_DEPS.get() {
        return Ok(cached.as_str());
    }
    let info = verify_native_dependencies()?;
    Ok(NATIVE_DEPS.get_or_init(|| info).as_str())
}

/// 成功结果缓存；数值求解入口重复检查不分配字符串。
static NATIVE_DEPS: std::sync::OnceLock<String> = std::sync::OnceLock::new();

fn verify_native_dependencies() -> Result<String, String> {
    let header = OpenCvVersion {
        major: opencv::core::CV_VERSION_MAJOR,
        minor: opencv::core::CV_VERSION_MINOR,
        patch: opencv::core::CV_VERSION_REVISION,
    };
    let supported_header = header.major == 5 || (header.major == 4 && header.minor >= 8);
    if !supported_header {
        return Err(format!(
            "OpenCV headers {header} are outside the supported range (need >= 4.8, < 6); \
             rebuild the opencv bindings against a supported OpenCV"
        ));
    }

    let runtime = OpenCvVersion {
        major: opencv::core::get_version_major(),
        minor: opencv::core::get_version_minor(),
        patch: opencv::core::get_version_revision(),
    };
    if runtime.major != header.major || runtime.minor != header.minor {
        return Err(format!(
            "OpenCV ABI mismatch: headers {header}, loaded runtime {runtime}; matching major/minor \
             is required, so install a {header} runtime or rebuild the bindings against {runtime}"
        ));
    }
    if runtime.patch < header.patch {
        return Err(format!(
            "OpenCV runtime {runtime} is older than the headers {header} the bindings were compiled \
             against; upgrade the runtime to {header} or newer"
        ));
    }

    let runtime_text = opencv::core::get_version_string()
        .map_err(|error| format!("can't read the OpenCV runtime version string: {error}"))?;
    Ok(format!(
        "OpenCV headers {header}, runtime {runtime} ({runtime_text})"
    ))
}
