//! 单相机标定**会话循环**：入库 → 增量求解 → 分析 → 收敛判定。
//!
//! 会话状态机：
//!
//! ```text
//! 循环: 新观测入库
//!   → 用当前内参给该视图估位姿（残差超限的视图暂不参与，参数变好后自动回归）
//!   → 每次入库都重建信息矩阵（可观测性报告：σ/秩/条件数/相关性/Δlogdet）
//!   → 每 `cadence` 张（或首次达到门槛）做一次真实优化，解要过采纳闸门才落地
//!   → 连续 `window` 次评估都满足 goal → 收敛
//! ```
//!
//! goal：**满秩 ∧ rms ≤ 阈值 ∧ 焦距相对 σ ≤ 阈值 ∧ 主点 σ ≤ 阈值**。
//!
//! 两条入口共享同一份状态机：
//! - 逐张产线用 [`Session::observe`]（= `ingest` + `update(false)`，按 `cadence` 排程）；
//! - 独占数值 worker 用 [`Session::ingest`] 冻结一批观测，再 [`Session::update(true)`] 无视
//!   `cadence` 求解该批次，并用 [`Session::state`] / [`Session::has_current_solution`] 读取结果。
//!   `update` 对未变化的前缀是幂等的（不重复求解/评估、不刷 `streak`）；
//!   `has_current_solution` 只认「覆盖当前全部观测且评估一致、采纳成功」的解——采纳失败
//!   或新增观测都不得沿用旧 holdout 冒充本版。

use crate::estimator::{Observation, estimate_fixed_intrinsics_pose};
use crate::estimator::{SolveBackend, SolveOutcome, SolveRequest};
use crate::models::{ModelKind, Parameters};
use crate::observability::{self, IntrinsicObservability, ObservabilityError};

/// 求解门槛按模型给：DS 在少视图上会落进坏极小，KB4 从 5 视图起稳定。
pub const MIN_VIEWS_FOR_SOLVE_BY_MODEL: [usize; 2] = [10, 5]; // [ds, kb4]
/// 解质量闸门：新解 rms 超当前最优这么多倍就拒绝。
pub const MAX_RMS_REGRESSION: f64 = 3.0;
pub const MIN_ACCEPT_RMS_PX: f64 = 1.0;
pub const MAX_ACCEPT_RMS_PX: f64 = 3.0;
/// 视图数翻倍时允许最优点移动。
pub const VIEWS_TO_OVERRIDE_REGRESSION: usize = 2;
/// 单视图位姿残差上限（相对判据）：`rms > max(FACTOR × 中位, 绝对下限)` 才排除。
pub const VIEW_OUTLIER_FACTOR: f64 = 3.0;
pub const VIEW_OUTLIER_MIN_PX: f64 = 2.0;
/// 播种前剔除离群视图的判据（比视图排除更宽松）。
pub const SEED_OUTLIER_FACTOR: f64 = 4.0;
pub const SEED_OUTLIER_ABS_PX: f64 = 2.0;

#[derive(Clone, Copy, Debug)]
pub struct SessionThresholds {
    pub max_rms_px: f64,
    pub max_focal_relative_stddev: f64,
    pub max_principal_stddev_px: f64,
    /// holdout 门禁：冻结解在 holdout 视图上的 rms 与 p95。
    pub max_holdout_rms_px: f64,
    pub max_holdout_p95_px: f64,
    /// 连续多少次评估达标才算收敛。
    pub window: usize,
}

impl Default for SessionThresholds {
    fn default() -> Self {
        Self {
            max_rms_px: 0.2,
            max_focal_relative_stddev: 2.0e-3,
            max_principal_stddev_px: 0.2,
            max_holdout_rms_px: 1.0,
            max_holdout_p95_px: 1.0,
            window: 3,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SessionOptions {
    pub model: ModelKind,
    pub image_size: (u32, u32),
    /// 每入库多少张做一次真实优化（1 = 每张）。
    pub cadence: usize,
    /// 求解帧数上限（0 = 不裁剪）。**不是简单截断**：用最远点采样保住中心/尺度/形状多样性。
    pub max_solve_observations: usize,
    /// holdout 划分比例（train 之外的视图只用于验证，不回流优化）。
    pub holdout_fraction: f64,
    /// 宣布收敛前 holdout 至少要有这么多视图。
    pub min_holdout_views: usize,
}

impl SessionOptions {
    pub fn min_views_for_solve(&self) -> usize {
        match self.model {
            ModelKind::Ds => MIN_VIEWS_FOR_SOLVE_BY_MODEL[0],
            ModelKind::Kb4 => MIN_VIEWS_FOR_SOLVE_BY_MODEL[1],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SessionError {
    NoObservations,
    InsufficientViews {
        usable: usize,
        needed: usize,
    },
    SolveFailed(String),
    Rejected(String),
    Observability(String),
    /// 求解后端不可用或失败（后端由调用方注入：OpenCV / 外部进程）。
    Backend(String),
}

impl std::fmt::Display for SessionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SessionError::NoObservations => write!(f, "session has no observations"),
            SessionError::InsufficientViews { usable, needed } => {
                write!(
                    f,
                    "{usable} usable view(s); need >= {needed} before a full solve"
                )
            }
            SessionError::SolveFailed(detail) => write!(f, "solve failed: {detail}"),
            SessionError::Rejected(reason) => write!(f, "solve rejected: {reason}"),
            SessionError::Observability(detail) => write!(f, "observability unavailable: {detail}"),
            SessionError::Backend(detail) => write!(f, "solve backend failed: {detail}"),
        }
    }
}

impl std::error::Error for SessionError {}

impl From<ObservabilityError> for SessionError {
    fn from(error: ObservabilityError) -> Self {
        SessionError::Observability(error.to_string())
    }
}

/// 一步（一次入库）之后的会话快照：TUI/GUI 消费的就是它。
#[derive(Clone, Debug)]
pub struct SessionState {
    pub model: ModelKind,
    pub views: usize,
    pub used_views: usize,
    pub excluded_views: usize,
    pub rms_px: f64,
    pub rank: usize,
    pub condition_number: f64,
    pub focal_relative_stddev: (f64, f64),
    pub principal_stddev_px: (f64, f64),
    pub info_gain: Option<f64>,
    /// holdout（冻结解验证集）指标。
    pub holdout_views: usize,
    pub holdout_rms_px: f64,
    pub holdout_p95_px: f64,
    pub holdout_invalid: usize,
    pub solves: usize,
    pub failures: usize,
    pub converged: bool,
    pub streak: usize,
    pub status: &'static str,
    pub detail: String,
    pub parameters: Vec<f64>,
}

/// 一次入库的结果：是否发起了求解、是否达标。
#[derive(Clone, Debug)]
pub struct StepOutcome {
    pub state: SessionState,
    pub solved: bool,
    pub goal_met: bool,
}

/// 可用视图集合：观测与位姿两份**同长**列表（下标一一对应）。
pub type ViewSet = (Vec<Observation>, Vec<([f64; 3], [f64; 3])>);

pub struct Session {
    options: SessionOptions,
    thresholds: SessionThresholds,
    backend: Box<SolveBackend>,
    observations: Vec<Observation>,
    poses: Vec<([f64; 3], [f64; 3])>,
    view_rms: Vec<f64>,
    parameters: Parameters,
    report: Option<IntrinsicObservability>,
    streak: usize,
    solves: usize,
    failures: usize,
    submitted_views: usize,
    last_detail: String,
    /// 最近一次被闸门拒绝的原因（诊断用，界面展示）。
    last_rejection: Option<String>,
    /// 最近一次划分出的 holdout 观测（冻结解只在这里验证，不回流优化）。
    holdout_observations: Vec<Observation>,
    /// 最近一次求解后的 holdout 指标。
    holdout: Option<crate::holdout::HoldoutMetrics>,
    /// 最后一次**成功采纳**的解所覆盖的观测前缀长度（`None` = 尚无被采纳的解）。
    solution_views: Option<usize>,
    /// 最近一次**成功评估**所覆盖的观测前缀长度（`None` = `report` 对新前缀/新参数不再新鲜）。
    report_views: Option<usize>,
    /// 前缀/解自上次 `update` 以来是否变化（决定是否需要重新排程求解与评估）。
    dirty: bool,
    /// 最近一次 `update` 的结果：同前缀空闲调用直接复用，不重复评估刷 streak。
    last_outcome: Option<StepOutcome>,
    /// 最近一次全量精修（[`Session::refine`]）的目标观测前缀长度与求解是否被采纳。
    /// `None` = 尚未精修；同一前缀**成功**精修后重复调用直接复用，**失败**则允许再次尝试。
    last_refine: Option<(usize, bool)>,
}

impl Session {
    pub fn new(
        options: SessionOptions,
        thresholds: SessionThresholds,
        backend: Box<SolveBackend>,
    ) -> Self {
        let parameters = options.model.default_parameters(options.image_size);
        Self {
            options,
            thresholds,
            backend,
            observations: Vec::new(),
            poses: Vec::new(),
            view_rms: Vec::new(),
            parameters,
            report: None,
            streak: 0,
            solves: 0,
            failures: 0,
            submitted_views: 0,
            last_detail: "waiting for observations".to_owned(),
            last_rejection: None,
            holdout_observations: Vec::new(),
            holdout: None,
            solution_views: None,
            report_views: None,
            dirty: false,
            last_outcome: None,
            last_refine: None,
        }
    }

    pub fn thresholds(&self) -> SessionThresholds {
        self.thresholds
    }

    pub fn observations(&self) -> &[Observation] {
        &self.observations
    }

    pub fn parameters(&self) -> &Parameters {
        &self.parameters
    }

    pub fn report(&self) -> Option<&IntrinsicObservability> {
        self.report.as_ref()
    }

    pub fn converged(&self) -> bool {
        self.streak >= self.thresholds.window
    }

    /// 位姿残差合格的视图下标；坏视图**不删除**，参数变好后自动回归。
    pub fn usable_view_indices(&self) -> Vec<usize> {
        let everything: Vec<usize> = (0..self.observations.len()).collect();
        if self.view_rms.len() != self.observations.len() || self.view_rms.is_empty() {
            return everything;
        }
        let mut finite: Vec<f64> = self
            .view_rms
            .iter()
            .copied()
            .filter(|value| value.is_finite())
            .collect();
        if finite.is_empty() {
            return everything;
        }
        finite.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = finite[finite.len() / 2];
        let threshold = (VIEW_OUTLIER_FACTOR * median).max(VIEW_OUTLIER_MIN_PX);
        let good: Vec<usize> = self
            .view_rms
            .iter()
            .enumerate()
            .filter(|(_index, value)| **value <= threshold)
            .map(|(index, _value)| index)
            .collect();
        // 排除过多说明是参数差而不是数据差：退回全集，避免把自己饿死。
        let floor = (self.options.min_views_for_solve() / 2).max(2);
        if good.len() < floor {
            return everything;
        }
        good
    }

    fn usable_views(&self) -> ViewSet {
        let indices = self.usable_view_indices();
        (
            indices
                .iter()
                .map(|index| self.observations[*index].clone())
                .collect(),
            indices.iter().map(|index| self.poses[*index]).collect(),
        )
    }

    /// 播种用的视图子集：按当前参数算每视图 rms，剔除离群（子集太狠就退回全集）。
    fn seed_subset(&self, views: &[Observation], poses: &[([f64; 3], [f64; 3])]) -> Vec<usize> {
        if views.len() < 4 {
            return (0..views.len()).collect();
        }
        let residuals: Vec<f64> = views
            .iter()
            .map(|view| {
                let estimate =
                    estimate_fixed_intrinsics_pose(self.options.model, &self.parameters, view);
                if estimate.status == crate::estimator::Status::Pass {
                    estimate.rms_px
                } else {
                    f64::INFINITY
                }
            })
            .collect();
        let mut finite: Vec<f64> = residuals
            .iter()
            .copied()
            .filter(|value| value.is_finite())
            .collect();
        if finite.is_empty() {
            return (0..views.len()).collect();
        }
        finite.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = finite[finite.len() / 2];
        let threshold = (SEED_OUTLIER_FACTOR * median).max(SEED_OUTLIER_ABS_PX);
        let subset: Vec<usize> = residuals
            .iter()
            .enumerate()
            .filter(|(_index, value)| **value <= threshold)
            .map(|(index, _value)| index)
            .collect();
        let _ = poses;
        if subset.len() < 4 {
            return (0..views.len()).collect();
        }
        subset
    }

    /// 单张入口：入库 + `update(false)`（保持旧的逐张 `observe` 调度节奏）。
    pub fn observe(&mut self, observation: Observation) -> StepOutcome {
        self.ingest([observation]);
        self.update(false)
    }

    /// 批量入库：逐条用**当前内参**估位姿并追加到冻结前缀；**不做**求解/可观测性。
    ///
    /// 数值 worker 独占 [`Session`]，因此这里不做线程同步：批次内只追加，计算期间不追加。
    /// 入库改变了观测前缀，`report`/解随之不再新鲜，等待下一次 [`Session::update`] 重新排程。
    pub fn ingest(&mut self, observations: impl IntoIterator<Item = Observation>) {
        let mut appended = 0usize;
        for observation in observations {
            let estimate =
                estimate_fixed_intrinsics_pose(self.options.model, &self.parameters, &observation);
            let (pose, rms) = if estimate.status == crate::estimator::Status::Pass {
                ((estimate.rvec, estimate.tvec), estimate.rms_px)
            } else {
                // 位姿不可用时保留一个前向保守位姿，残差记 inf（下一次重估会自动修正）
                let mean_z = observation
                    .object_points
                    .iter()
                    .map(|point| point[2])
                    .sum::<f64>()
                    / observation.object_points.len().max(1) as f64;
                (
                    ([0.0, 0.0, 0.0], [0.0, 0.0, (2.0 * mean_z).max(0.5)]),
                    f64::INFINITY,
                )
            };
            self.observations.push(observation);
            self.poses.push(pose);
            self.view_rms.push(rms);
            appended += 1;
        }
        if appended > 0 {
            self.dirty = true;
        }
    }

    /// 一次调度与评估：对**冻结的当前前缀**只调用一次（不逐观测调用昂贵的 `evaluate`）。
    ///
    /// - `force=true` 时无视 `cadence`：只要前缀里有未求解观测且视图数达标就求解最新批次；
    /// - 前缀（观测）与解自上次调用以来未变化的**空闲**调用直接复用上一次结果，
    ///   不重新评估、不推进 `streak`；`force` 也不会对已求解过的前缀重复求解；
    /// - 求解被拒/后端失败时**不**评估旧解：失败计数只记一次、清零 `streak`，原因留在
    ///   `detail`（否则旧参数评估会把拒解刷成"又一次收敛"）。
    ///
    /// 最终**全量精修**（越过 `max_solve_observations` 上限）走 [`Session::refine`]。
    pub fn update(&mut self, force: bool) -> StepOutcome {
        if self.observations.is_empty() {
            let outcome = StepOutcome {
                state: self.state(),
                solved: false,
                goal_met: false,
            };
            self.last_outcome = Some(outcome.clone());
            return outcome;
        }
        let min_views = self.options.min_views_for_solve();
        let new_views = self.observations.len().saturating_sub(self.submitted_views);
        // 有未求解的观测，且视图数够（不足时无法求解，仍要返回完整诊断）。
        let pending = new_views > 0 && self.observations.len() >= min_views;
        if !self.dirty && !(force && pending) {
            return self.cached_outcome();
        }
        let due = pending && (force || new_views >= self.options.cadence || self.solves == 0);
        let mut solved = false;
        let mut failed = false;
        if due {
            match self.solve_with(false) {
                Ok(_) => solved = true,
                Err(error) => {
                    failed = true;
                    self.record_failure(&error);
                }
            }
        }
        // 失败/未采纳的解不得借旧参数评估去刷收敛 streak：失败原因留在 detail。
        let goal_met = if failed {
            false
        } else {
            match self.evaluate() {
                Ok(()) => self.goal_met_now(),
                Err(_) => false,
            }
        };
        self.dirty = false;
        let outcome = StepOutcome {
            state: self.state(),
            solved,
            goal_met,
        };
        self.last_outcome = Some(outcome.clone());
        outcome
    }

    /// 最终**全量精修**：用全部可用训练观测重新求解并采纳（越过
    /// `max_solve_observations` 代表子集上限；holdout 仍独立、不回流优化）。
    ///
    /// 与 [`Session::update`] 共用同一套求解/采纳/评估代码，但**无视 `cadence` 与幂等缓存**：
    /// 即使同一前缀刚做过在线限量解，也必须真正重解一次。同一前缀**成功**精修后的重复调用
    /// 直接复用上次结果（不重复求解、不刷 `streak`）；**失败**则保留失败诊断并允许再次尝试。
    ///
    /// 精修失败时 `has_current_solution` 必须为 `false`：本前缀较早的在线解不得冒充精修成功。
    pub fn refine(&mut self) -> StepOutcome {
        if self.observations.is_empty() {
            let outcome = StepOutcome {
                state: self.state(),
                solved: false,
                goal_met: false,
            };
            self.last_outcome = Some(outcome.clone());
            return outcome;
        }
        // 已对本前缀**成功求解且评估成功**：幂等复用，不重复求解也不刷 streak。
        // 求解成功但评估失败时不算完整精修，下一次仍会重试。
        if self.last_refine == Some((self.observations.len(), true))
            && self.report_views == Some(self.observations.len())
        {
            return self.cached_outcome();
        }
        let solved = match self.solve_with(true) {
            Ok(_) => true,
            Err(error) => {
                self.record_failure(&error);
                false
            }
        };
        let goal_met = if solved {
            match self.evaluate() {
                Ok(()) => self.goal_met_now(),
                Err(_) => false,
            }
        } else {
            false
        };
        self.last_refine = Some((self.observations.len(), solved));
        self.dirty = false;
        let outcome = StepOutcome {
            state: self.state(),
            solved,
            goal_met,
        };
        self.last_outcome = Some(outcome.clone());
        outcome
    }

    /// 纯读取当前状态快照：不求解、不评估、不推进 `streak`（未新增观测的相机可直接复用）。
    pub fn state(&self) -> SessionState {
        let report = self.report.as_ref();
        let usable = self.usable_view_indices().len();
        SessionState {
            model: self.options.model,
            views: self.observations.len(),
            used_views: report.map(|report| report.used_view_count).unwrap_or(0),
            excluded_views: self.observations.len().saturating_sub(usable),
            rms_px: report.map(|report| report.rms_px).unwrap_or(f64::NAN),
            rank: report.map(|report| report.rank).unwrap_or(0),
            condition_number: report
                .map(|report| report.condition_number)
                .unwrap_or(f64::INFINITY),
            focal_relative_stddev: report
                .map(|report| report.focal_relative_stddev())
                .unwrap_or((f64::NAN, f64::NAN)),
            principal_stddev_px: report
                .map(|report| report.principal_stddev_px())
                .unwrap_or((f64::NAN, f64::NAN)),
            info_gain: report.and_then(|report| report.info_gain),
            holdout_views: self.holdout.map(|metrics| metrics.views).unwrap_or(0),
            holdout_rms_px: self
                .holdout
                .map(|metrics| metrics.rms_px)
                .unwrap_or(f64::NAN),
            holdout_p95_px: self
                .holdout
                .map(|metrics| metrics.p95_px)
                .unwrap_or(f64::NAN),
            holdout_invalid: self
                .holdout
                .map(|metrics| metrics.invalid_projections)
                .unwrap_or(0),
            solves: self.solves,
            failures: self.failures,
            converged: self.converged(),
            streak: self.streak,
            status: self.status_str(),
            detail: self.last_detail.clone(),
            parameters: self.parameters.as_vector(),
        }
    }

    /// 状态标签：本前缀最终精修失败必须显式可见，不能被较早的在线解标成 `ok`。
    fn status_str(&self) -> &'static str {
        if matches!(self.last_refine, Some((views, false)) if views == self.observations.len()) {
            "refine_failed"
        } else if self.report.is_some() {
            "ok"
        } else {
            "insufficient_views"
        }
    }

    /// 当前**整个观测前缀**是否已拥有「成功采纳且评估一致」的解。
    ///
    /// 必须同时满足：最后一次采纳的解正好覆盖当前全部观测（没有新增的未解观测），
    /// 且该前缀上的评估成功过（`report` 与当前参数/前缀一致）。
    ///
    /// **采纳失败不会沿用旧 holdout 冒充本版**：只要前缀增长或解被改写后未重新评估，
    /// 就返回 `false`。针对**本前缀**的最终精修失败时同样为 `false`——较早的在线限量解
    /// 不得冒充精修结果。
    pub fn has_current_solution(&self) -> bool {
        let len = self.observations.len();
        if self.last_refine == Some((len, false)) {
            return false;
        }
        self.solution_views == Some(len) && self.report_views == Some(len)
    }

    /// 当前 `report` 的判据 + 最近一次采纳解的 holdout 门禁（纯读取）。
    fn goal_met_now(&self) -> bool {
        let Some(report) = self.report.as_ref() else {
            return false;
        };
        report.goal_met(
            self.thresholds.max_focal_relative_stddev,
            self.thresholds.max_principal_stddev_px,
            self.thresholds.max_rms_px,
        ) && self.holdout_gate_met()
    }

    /// 同前缀空闲调用复用上一次 `update` 结果（不重新评估）。
    fn cached_outcome(&self) -> StepOutcome {
        self.last_outcome.clone().unwrap_or_else(|| StepOutcome {
            state: self.state(),
            solved: false,
            goal_met: self.goal_met_now(),
        })
    }

    /// 记录一次**未被采纳**的求解：失败计数 +1（**只此一处**，避免 `solve` 内部与调度入口
    /// 重复计数）、清零收敛 `streak`，并把原因写进 `detail`（界面与最终诊断可见）。
    fn record_failure(&mut self, error: &SessionError) {
        self.failures += 1;
        self.streak = 0;
        let reason = error.to_string();
        self.last_rejection = Some(reason.clone());
        self.last_detail = reason;
    }

    /// 真优化：把（已剔除离群视图的）观测交给注入的后端；本 crate 不做内参求解。
    ///
    /// 走**在线路径**：应用 `max_solve_observations` 代表子集上限。最终全量精修请用
    /// [`Session::refine`]。失败只返回错误，**不在此处记账**（失败计数/streak 由调度入口
    /// [`Session::record_failure`] 统一处理，避免重复计数）。
    pub fn solve(&mut self) -> Result<SolveOutcome, SessionError> {
        self.solve_with(false)
    }

    /// 求解核心：整条 `usable → train → 代表子集 → 原始观测下标` 的映射在**本函数内一次固定**，
    /// 后端返回后不再重算；`initial`（旧参数）与 `initial_poses`（旧位姿）只在已有解时
    /// 作为热启动给出，坏视图不删除（参数变好后自动回归）。
    ///
    /// `all_training=true`（[`Session::refine`]）时用**全部可用训练观测**，跳过代表子集上限与
    /// KB4 播种子集；holdout 仍在两侧都独立、不回流优化。
    fn solve_with(&mut self, all_training: bool) -> Result<SolveOutcome, SessionError> {
        if self.observations.is_empty() {
            return Err(SessionError::NoObservations);
        }
        self.dirty = true;
        // 冻结当前可用视图（观测 + 位姿）：本次求解内不再变化。
        let usable_indices = self.usable_view_indices();
        let usable: Vec<Observation> = usable_indices
            .iter()
            .map(|index| self.observations[*index].clone())
            .collect();
        let usable_poses: Vec<crate::estimator::IntrinsicPose> = usable_indices
            .iter()
            .map(|index| self.poses[*index])
            .collect();
        let min_views = self.options.min_views_for_solve();
        if usable.len() < min_views {
            return Err(SessionError::InsufficientViews {
                usable: usable.len(),
                needed: min_views,
            });
        }
        // 划分 train / holdout：holdout 只用于验证。
        let (train_indices, holdout_indices) = crate::holdout::split_train_holdout(
            &usable,
            self.options.holdout_fraction,
            self.options.image_size,
        )
        .map_err(|error| SessionError::Rejected(error.to_string()))?;
        let holdout_observations: Vec<Observation> = holdout_indices
            .iter()
            .map(|index| usable[*index].clone())
            .collect();
        let train_views: Vec<crate::estimator::Observation> = train_indices
            .iter()
            .map(|index| usable[*index].clone())
            .collect();
        let train_poses: Vec<crate::estimator::IntrinsicPose> = train_indices
            .iter()
            .map(|index| usable_poses[*index])
            .collect();
        if train_views.len() < min_views {
            return Err(SessionError::InsufficientViews {
                usable: train_views.len(),
                needed: min_views,
            });
        }
        // 视图集合：全量精修直接用全部训练视图；在线路径 KB4 只用剔除离群后的子集、
        // DS 用全量训练视图，再按 `max_solve_observations` 代表性限幅。
        let indices: Vec<usize> = if all_training {
            (0..train_views.len()).collect()
        } else {
            let selected: Vec<usize> = match self.options.model {
                ModelKind::Kb4 => self.seed_subset(&train_views, &train_poses),
                ModelKind::Ds => (0..train_views.len()).collect(),
            };
            let keep = crate::holdout::representative_indices(
                &train_views,
                self.options.max_solve_observations,
                self.options.image_size,
            );
            selected
                .into_iter()
                .filter(|index| keep.contains(index))
                .collect()
        };
        let views: Vec<crate::estimator::Observation> = indices
            .iter()
            .map(|index| train_views[*index].clone())
            .collect();
        let poses: Vec<crate::estimator::IntrinsicPose> =
            indices.iter().map(|index| train_poses[*index]).collect();
        // 冻结到**原始观测**的下标映射：后端返回后直接用它，不重新跑可用性筛选。
        let original_indices: Vec<usize> = indices
            .iter()
            .map(|index| usable_indices[train_indices[*index]])
            .collect();
        let request = SolveRequest {
            kind: self.options.model,
            observations: &views,
            image_size: self.options.image_size,
            initial: (self.solves > 0).then_some(self.parameters),
            initial_poses: (self.solves > 0).then_some(poses.as_slice()),
        };
        let outcome = (self.backend)(request).map_err(SessionError::Backend)?;

        self.submitted_views = self.observations.len();
        if let Some(reason) = self.rejection_reason(&outcome) {
            // 拒绝原因留在 detail（失败计数/streak 由调度入口 `record_failure` 统一处理，
            // 避免这里有重复计数）。
            self.last_detail = format!("solve rejected ({reason})");
            self.last_rejection = Some(reason.clone());
            return Err(SessionError::Rejected(reason));
        }
        self.last_rejection = None;
        let outcome = self.apply(outcome, &original_indices)?;
        // holdout 只在**采纳成功**后落地：拒绝不会用旧 holdout 冒充本版。
        self.holdout_observations = holdout_observations;
        self.holdout = Some(crate::holdout::holdout_metrics(
            self.options.model,
            &self.parameters,
            &self.holdout_observations,
        ));
        self.solution_views = Some(self.observations.len());
        Ok(outcome)
    }

    /// holdout 门禁：视图数够 + rms/p95 达标 + 无无效投影。
    fn holdout_gate_met(&self) -> bool {
        self.holdout
            .map(|metrics| {
                metrics.views >= self.options.min_holdout_views
                    && metrics.invalid_projections == 0
                    && metrics.rms_px.is_finite()
                    && metrics.rms_px <= self.thresholds.max_holdout_rms_px
                    && metrics.p95_px.is_finite()
                    && metrics.p95_px <= self.thresholds.max_holdout_p95_px
            })
            .unwrap_or(false)
    }

    fn rejection_reason(&self, result: &SolveOutcome) -> Option<String> {
        if result.status != crate::estimator::Status::Pass {
            return Some("backend did not converge".to_owned());
        }
        if result.invalid_projection_count > 0 {
            return Some(format!(
                "{} invalid projections",
                result.invalid_projection_count
            ));
        }
        let new_rms = result.rms_px;
        if !new_rms.is_finite() {
            return Some("non-finite rms".to_owned());
        }
        if new_rms > MAX_ACCEPT_RMS_PX {
            return Some(format!(
                "rms {new_rms:.3}px exceeds {MAX_ACCEPT_RMS_PX:.1}px"
            ));
        }
        let Some(report) = &self.report else {
            return None;
        };
        let current_rms = report.rms_px;
        if !current_rms.is_finite() {
            return None;
        }
        let accepted_views = self.observations.len().max(1);
        if accepted_views >= VIEWS_TO_OVERRIDE_REGRESSION * (self.solves.max(1)) {
            return None;
        }
        if new_rms > (MAX_RMS_REGRESSION * current_rms).max(MIN_ACCEPT_RMS_PX) {
            return Some(format!(
                "rms {new_rms:.3}px worse than current {current_rms:.3}px"
            ));
        }
        None
    }

    /// 采用解：indices 已复合 usable/train/代表子集选择，指向原始观测。
    fn apply(
        &mut self,
        result: SolveOutcome,
        indices: &[usize],
    ) -> Result<SolveOutcome, SessionError> {
        if result.poses.len() != indices.len() {
            return Err(SessionError::Backend(
                "optimized poses must match solve observations".to_owned(),
            ));
        }
        let mut optimized = vec![false; self.observations.len()];
        for (index, pose) in indices.iter().zip(&result.poses) {
            self.poses[*index] = *pose;
            optimized[*index] = true;
        }
        self.parameters = result.parameters;
        self.solves += 1;
        self.refresh_poses(&optimized);
        // 解已改写：`report` 与旧参数不再一致，等待重新评估。
        self.report_views = None;
        self.dirty = true;
        Ok(result)
    }

    /// 保留已优化视图位姿；只重估未参加求解的视图（包括 holdout）。
    fn refresh_poses(&mut self, optimized: &[bool]) {
        let mut residuals = Vec::with_capacity(self.observations.len());
        for (index, is_optimized) in optimized.iter().enumerate() {
            if *is_optimized {
                let evaluation = crate::estimator::evaluate_solution(
                    self.options.model,
                    std::slice::from_ref(&self.observations[index]),
                    &self.parameters,
                    std::slice::from_ref(&self.poses[index]),
                );
                residuals.push(if evaluation.invalid_projection_count == 0 {
                    evaluation.rms_px
                } else {
                    f64::INFINITY
                });
                continue;
            }
            let estimate = estimate_fixed_intrinsics_pose(
                self.options.model,
                &self.parameters,
                &self.observations[index],
            );
            if estimate.status == crate::estimator::Status::Pass && estimate.rms_px.is_finite() {
                self.poses[index] = (estimate.rvec, estimate.tvec);
                residuals.push(estimate.rms_px);
            } else {
                residuals.push(f64::INFINITY);
            }
        }
        self.view_rms = residuals;
    }

    /// 评估：重建信息矩阵（σ/秩/条件数/Δlogdet）并推进收敛 streak。
    pub fn evaluate(&mut self) -> Result<(), SessionError> {
        // 先作废：只有真正跑完（成功）才重新标定为对当前前缀新鲜。
        self.report_views = None;
        if self.observations.is_empty() {
            return Err(SessionError::NoObservations);
        }
        let (usable, usable_poses) = self.usable_views();
        if usable.len() < 2 {
            self.streak = 0;
            self.last_detail = format!(
                "{} usable view(s); need >= 2 for observability",
                usable.len()
            );
            return Err(SessionError::InsufficientViews {
                usable: usable.len(),
                needed: 2,
            });
        }
        let views: Vec<(Observation, [f64; 3], [f64; 3])> = usable
            .iter()
            .cloned()
            .zip(usable_poses.iter())
            .map(|(observation, (rvec, tvec))| (observation, *rvec, *tvec))
            .collect();
        let report = match observability::analyze_intrinsics(
            self.options.model,
            &self.parameters,
            &views,
            self.report.as_ref(),
        ) {
            Ok(report) => report,
            Err(error) => {
                self.streak = 0;
                self.last_detail = error.to_string();
                return Err(error.into());
            }
        };
        let intrinsic_goal = report.goal_met(
            self.thresholds.max_focal_relative_stddev,
            self.thresholds.max_principal_stddev_px,
            self.thresholds.max_rms_px,
        );
        let holdout_ok = self.holdout_gate_met();
        let goal = intrinsic_goal && holdout_ok;
        self.streak = if goal { self.streak + 1 } else { 0 };
        let (fx_sigma, _fy_sigma) = report.focal_relative_stddev();
        let (cx_sigma, _cy_sigma) = report.principal_stddev_px();
        let holdout_text = match self.holdout {
            Some(metrics) => format!(
                "holdout {}/{} rms {:.4}px p95 {:.4}px invalid {}",
                metrics.views,
                self.options.min_holdout_views,
                metrics.rms_px,
                metrics.p95_px,
                metrics.invalid_projections
            ),
            None => "holdout 未划分（尚未求解）".to_owned(),
        };
        self.last_detail = if goal {
            format!(
                "goal met {}/{} (rms {:.4}px, fσ {:.4}%, cσ {:.4}px, rank {}/{}, {holdout_text})",
                self.streak,
                self.thresholds.window,
                report.rms_px,
                fx_sigma * 100.0,
                cx_sigma,
                report.rank,
                report.names.len()
            )
        } else {
            let rejected = self
                .last_rejection
                .as_ref()
                .map(|reason| format!(" | last solve rejected: {reason}"))
                .unwrap_or_default();
            format!(
                "collecting (rms {:.4}px, fσ {:.4}%, cσ {:.4}px, rank {}/{}, streak {}, {holdout_text}){rejected}",
                report.rms_px,
                fx_sigma * 100.0,
                cx_sigma,
                report.rank,
                report.names.len(),
                self.streak
            )
        };
        self.report = Some(report);
        self.report_views = Some(self.observations.len());
        Ok(())
    }
}
