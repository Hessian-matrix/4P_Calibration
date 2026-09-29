//! 标定核心算法（lib）：纯计算，无 IO、无 GUI、无 OpenCV。
//!
//! 分层约定：
//!
//! - **本 crate = 核心算法**：板几何、投影模型（Double Sphere / KB4）、位姿估计、数值可观测性。
//!   它不知道相机、文件、线程、窗口的存在，**也不做内参求解**——求解由外部已验证的后端
//!   （OpenCV / scipy）通过 [`SolveBackend`] 注入。
//! - **产线 = 独立 bin crate**：单相机内参产线（`rigcal-camera`）与四路 rig 产线
//!   各自拥有采集、检测、会话、报告与 GUI，依赖本 crate；反向依赖一律禁止。
//!
//! `tests/parity.rs` 用真机黄金值对拍投影、位姿与可观测性。

pub mod board;
pub mod config;
pub mod estimator;
pub mod extrinsics;
pub mod holdout;
pub mod models;
pub mod observability;
pub mod rotation;
pub mod session;

pub use board::AprilGridConfig;
pub use estimator::{
    Evaluation, IntrinsicPose, Observation, ObservationArrays, ObservationView, PoseEstimate,
    SolveBackend, SolveOutcome, SolveRequest, Status, estimate_fixed_intrinsics_pose,
    evaluate_outcome, evaluate_solution, seed_pose,
};
pub use models::{ModelKind, Parameters, ds, kb4, project, unproject};
pub use session::{
    Session, SessionError, SessionOptions, SessionState, SessionThresholds, StepOutcome,
};
