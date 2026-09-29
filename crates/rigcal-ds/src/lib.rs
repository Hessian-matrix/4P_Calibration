//! DS（Double Sphere）联合标定：第三方 `levenberg-marquardt` + core 的 DS 模型。
//!
//! 默认 Rust；scipy 桥仅用于显式对拍，不作自动回退。
//! 内参用 log/logit 参数化，soft_l1 残差用有理化形式保留零点附近的导数。
//! 联合优化位姿必须随内参一起返回，不能丢弃后重新播种来评估同一份解。
//!
//! KB4 初始化使用角度映射 `f_DS/(1+xi)=f_KB4` 和三阶项，而不是直接复制焦距。
//! xi=0 的内参雅可比有一阶零方向；默认初始化不位于该退化点。

pub mod bridge;
pub mod model;
mod problem;

pub use bridge::{
    DsBridgeOptions, DsSolveError as DsBridgeError, ds_backend_available, solve_ds_via_scipy,
};
pub use model::{DsSeed, ds_from_parameters, parameters_from_ds, soft_l1};
pub use problem::kb4_seed;
pub use problem::{DsBundleOptions, solve_ds_intrinsics};

use rigcal_core::models::ModelKind;

/// 显式后端选择。未知值报错，不会悄悄改用其它实现。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DsBackendKind {
    Rust,
    Scipy,
}

impl DsBackendKind {
    /// 未设置或 `rust` 使用原生求解；`scipy` 使用对拍桥。
    pub fn from_env() -> Result<Self, String> {
        match std::env::var("RIGCAL_DS_BACKEND") {
            Ok(value) if value == "rust" => Ok(Self::Rust),
            Ok(value) if value == "scipy" => Ok(Self::Scipy),
            Err(std::env::VarError::NotPresent) => Ok(Self::Rust),
            other => Err(format!("RIGCAL_DS_BACKEND 必须是 rust 或 scipy：{other:?}")),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            DsBackendKind::Rust => "rust",
            DsBackendKind::Scipy => "scipy",
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DsSolveError {
    #[error("后端只处理 ds 模型，收到 {0:?}")]
    WrongModel(ModelKind),
    #[error("DS（Rust）：{0}")]
    Rust(String),
    #[error("DS（scipy 桥）：{0}")]
    Bridge(#[from] bridge::DsSolveError),
}
