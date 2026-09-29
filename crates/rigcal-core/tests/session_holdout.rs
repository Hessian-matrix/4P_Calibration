//! 会话层的划分与 holdout 门禁：证明配置里的 `max_solve_observations` / `holdout_*`
//! 真的在改变行为（而不是"配了但不生效"）。
//!
//! 用一个**假后端**：它记录每次收到的观测条数，并返回可控的结果。这样可以把
//! 「划分是否生效」与「门禁是否拦得住」分别断言出来，不依赖真实求解。

use std::sync::{Arc, Mutex};

use rigcal_core::board::{AprilGridConfig, Corner};
use rigcal_core::estimator::{
    Observation, SolveOutcome, Status, estimate_fixed_intrinsics_pose, evaluate_solution,
    project_with_pose,
};
use rigcal_core::models::{ModelKind, Parameters};
use rigcal_core::session::{Session, SessionOptions, SessionThresholds};

/// 造一条**可解**的观测：用会话的默认内参把板点投到像平面（位姿逐条变化）。
///
/// 关键点：像点必须来自真实投影，否则会话的位姿刷新会判视图不可用（残差 inf），
/// 划分与门禁就测不到了。
fn observation(index: usize) -> Observation {
    let image_size = (1280, 1088);
    let model = ModelKind::Ds;
    let parameters = model.default_parameters(image_size);
    let board = AprilGridConfig {
        rows: 6,
        cols: 6,
        tag_size_m: 0.088,
        tag_spacing_ratio: 0.3,
        first_tag_id: 0,
        dictionary: "DICT_APRILTAG_36h11".to_owned(),
        target_id: "session-holdout".to_owned(),
        tag_corner_order: [
            Corner::BottomRight,
            Corner::BottomLeft,
            Corner::TopLeft,
            Corner::TopRight,
        ],
    };
    let object_points = board
        .object_points(&(0..board.rows * board.cols).collect::<Vec<_>>())
        .expect("object points");
    let step = index as f64;
    let rvec = [
        0.25 * (0.6 * step).sin(),
        0.3 * (0.4 * step).cos(),
        0.4 * (0.25 * step).sin(),
    ];
    let tvec = [
        0.12 * (0.5 * step).sin(),
        0.10 * (0.35 * step).cos(),
        0.75 + 0.02 * step,
    ];
    let (image_points, valid) = project_with_pose(model, &parameters, &object_points, &rvec, &tvec);
    assert!(
        valid.iter().all(|value| *value),
        "observation {index} must stay inside the projection domain"
    );
    Observation {
        object_points,
        image_points,
    }
}

struct FakeBackend {
    seen: Arc<Mutex<Vec<usize>>>,
    parameters: Parameters,
}

impl FakeBackend {
    fn boxed(
        seen: Arc<Mutex<Vec<usize>>>,
        model: ModelKind,
        image_size: (u32, u32),
    ) -> Box<rigcal_core::SolveBackend> {
        let parameters = model.default_parameters(image_size);
        let backend = FakeBackend { seen, parameters };
        Box::new(move |request| {
            backend
                .seen
                .lock()
                .expect("seen")
                .push(request.observations.len());
            let estimates: Vec<_> = request
                .observations
                .iter()
                .map(|obs| estimate_fixed_intrinsics_pose(model, &backend.parameters, obs))
                .collect();
            let poses: Vec<_> = estimates
                .iter()
                .map(|pose| (pose.rvec, pose.tvec))
                .collect();
            let evaluation =
                evaluate_solution(model, request.observations, &backend.parameters, &poses);
            Ok(SolveOutcome {
                parameters: backend.parameters,
                poses,
                residuals_px: estimates
                    .into_iter()
                    .flat_map(|pose| pose.residuals_px)
                    .collect(),
                rms_px: evaluation.rms_px,
                invalid_projection_count: evaluation.invalid_projection_count,
                status: Status::Pass,
            })
        })
    }
}

fn options(
    model: ModelKind,
    max_solve: usize,
    fraction: f64,
    min_holdout: usize,
) -> SessionOptions {
    SessionOptions {
        model,
        image_size: (1280, 1088),
        cadence: 1,
        max_solve_observations: max_solve,
        holdout_fraction: fraction,
        min_holdout_views: min_holdout,
    }
}

#[test]
fn solving_uses_only_the_train_subset() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds::default(),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    // 喂 20 条：train 子集（0.25 划分 → 15）先要满足最小求解视图数，之后才会真正求解
    for index in 0..20 {
        session.observe(observation(index));
    }
    let counts = seen.lock().expect("seen").clone();
    assert!(
        !counts.is_empty(),
        "求解必须发生过（会话状态：{}）",
        session.observe(observation(0)).state.detail
    );
    // 后端每次最多只能看到 train 子集（20 条 → train 15）
    assert!(
        counts.iter().all(|count| *count <= 15),
        "后端只能看到 train 子集，实测 {counts:?}"
    );
    assert!(
        counts.contains(&15),
        "train 子集应为 15 条，实测 {counts:?}"
    );
}

#[test]
fn representative_limit_shrinks_the_solve_set() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 5, 0.25, 2),
        SessionThresholds::default(),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    for index in 0..16 {
        session.observe(observation(index));
    }
    let counts = seen.lock().expect("seen").clone();
    assert!(
        counts.iter().all(|count| *count <= 5),
        "max_solve_observations=5 必须限住求解条数，实测 {counts:?}"
    );
}

#[test]
fn holdout_gate_blocks_convergence_until_it_passes() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    // holdout 需要 8 条视图：先喂 20 条（train 15 / holdout 5）→ 门禁不达标
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 8),
        SessionThresholds {
            max_rms_px: 1e9,
            max_focal_relative_stddev: 1e9,
            max_principal_stddev_px: 1e9,
            max_holdout_rms_px: 1e9,
            max_holdout_p95_px: 1e9,
            window: 1,
        },
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    let mut last = session.observe(observation(0)).state;
    for index in 1..20 {
        last = session.observe(observation(index)).state;
    }
    assert!(
        !last.converged,
        "holdout 只有 5/8 时不得宣布收敛：{}",
        last.detail
    );
    assert_eq!(last.holdout_views, 5, "20 条按 0.25 划分 → holdout 5");
    // 继续喂到 holdout ≥ 8（总数 40 → holdout 10）
    for index in 20..40 {
        last = session.observe(observation(index)).state;
    }
    let state = last;
    assert!(state.holdout_views >= 8, "holdout 视图数应已达标");
    assert!(
        state.converged,
        "内部判据全放宽 + holdout 达标后应收敛：{}",
        state.detail
    );
}

#[test]
fn holdout_metrics_gate_on_rms() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    // holdout 门禁设成 0：即使其它判据全放宽，也必须拦住收敛
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds {
            max_rms_px: 1e9,
            max_focal_relative_stddev: 1e9,
            max_principal_stddev_px: 1e9,
            max_holdout_rms_px: 0.0,
            max_holdout_p95_px: 0.0,
            window: 1,
        },
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    let mut last = session.observe(observation(0)).state;
    for index in 1..20 {
        last = session.observe(observation(index)).state;
    }
    let state = last;
    assert!(
        !state.converged,
        "holdout rms 门禁为 0 时必须拦住收敛（实测 holdout rms {:.4}px）",
        state.holdout_rms_px
    );
    assert!(state.holdout_rms_px > 0.0);
}

#[test]
fn optimized_pose_indices_survive_train_and_representative_selection() {
    let mut settings = options(ModelKind::Ds, 5, 0.25, 2);
    settings.cadence = usize::MAX;
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        settings,
        SessionThresholds::default(),
        FakeBackend::boxed(seen, ModelKind::Ds, (1280, 1088)),
    );
    for index in 0..20 {
        session.observe(observation(index));
    }
    session.solve().expect("solve selected training views");
    session
        .evaluate()
        .expect("evaluate all original observations");
    let report = session.report().expect("report");
    assert!(
        report.rms_px < 1e-7,
        "wrong pose assigned after subset selection: {}",
        report.rms_px
    );
    assert_eq!(
        report.used_view_count, 20,
        "bad pose mapping must not be hidden as outliers"
    );
}

/// 可控后端：前 `good_calls` 次返回与 [`FakeBackend`] 相同的「完美」解，之后返回不收敛，
/// 用来验证「采纳失败 / 新增观测不得冒充当前解」。
fn scripted_backend(
    seen: Arc<Mutex<Vec<usize>>>,
    model: ModelKind,
    image_size: (u32, u32),
    good_calls: usize,
) -> Box<rigcal_core::SolveBackend> {
    let parameters = model.default_parameters(image_size);
    let calls = Arc::new(Mutex::new(0usize));
    Box::new(move |request| {
        seen.lock().expect("seen").push(request.observations.len());
        let index = {
            let mut calls = calls.lock().expect("calls");
            let index = *calls;
            *calls += 1;
            index
        };
        if index >= good_calls {
            return Ok(SolveOutcome {
                parameters,
                poses: Vec::new(),
                residuals_px: Vec::new(),
                rms_px: f64::INFINITY,
                invalid_projection_count: 0,
                status: Status::Fail,
            });
        }
        let estimates: Vec<_> = request
            .observations
            .iter()
            .map(|obs| estimate_fixed_intrinsics_pose(model, &parameters, obs))
            .collect();
        let poses: Vec<_> = estimates
            .iter()
            .map(|pose| (pose.rvec, pose.tvec))
            .collect();
        let evaluation = evaluate_solution(model, request.observations, &parameters, &poses);
        Ok(SolveOutcome {
            parameters,
            poses,
            residuals_px: estimates
                .into_iter()
                .flat_map(|pose| pose.residuals_px)
                .collect(),
            rms_px: evaluation.rms_px,
            invalid_projection_count: evaluation.invalid_projection_count,
            status: Status::Pass,
        })
    })
}

/// 批量入库 + 一次强制 update：入库本身绝不求解；两批 20 条只合并成一次真实求解。
#[test]
fn batch_ingest_defers_to_a_single_forced_solve() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds::default(),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    // 一批 10 条：入库本身不得求解，也不得声称拥有当前解。
    session.ingest((0..10).map(observation));
    assert!(seen.lock().expect("seen").is_empty(), "ingest 不得触发求解");
    assert!(!session.has_current_solution(), "尚未求解不得声称有当前解");
    session.update(true);
    assert!(
        seen.lock().expect("seen").is_empty(),
        "10 条还凑不出 train 侧最小视图数，强制求解也不应抵达后端"
    );
    // 再补一批，一次 update 就应把两批合并成一次真实求解。
    session.ingest((10..20).map(observation));
    let outcome = session.update(true);
    assert!(outcome.solved, "force 应无视 cadence 求解最新批次");
    assert_eq!(
        seen.lock().expect("seen").len(),
        1,
        "两批 20 条只应产生一次真实求解"
    );
    assert!(session.has_current_solution());
}

/// 前缀未变时的重复 `update` 必须复用上一次结果：不重跑求解，也不刷 streak 收敛。
#[test]
fn repeated_update_on_same_prefix_does_not_advance_streak() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds {
            max_rms_px: 1e9,
            max_focal_relative_stddev: 1e9,
            max_principal_stddev_px: 1e9,
            max_holdout_rms_px: 1e9,
            max_holdout_p95_px: 1e9,
            window: 3,
        },
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    session.ingest((0..40).map(observation));
    let first = session.update(true);
    assert!(
        first.goal_met,
        "判据全放宽后本批应达标：{}",
        first.state.detail
    );
    assert_eq!(first.state.streak, 1, "一次评估只应推进一格");
    for _ in 0..5 {
        let repeated = session.update(false);
        assert_eq!(
            repeated.state.streak, 1,
            "同前缀空闲 update 不得推进 streak：{}",
            repeated.state.detail
        );
        // 复用上一次结果：solved/goal 保持与首次一致（没有再次真正求解）。
        assert_eq!(repeated.solved, first.solved);
        assert_eq!(repeated.goal_met, first.goal_met);
    }
    assert_eq!(
        seen.lock().expect("seen").len(),
        1,
        "空闲 update 不得再次求解"
    );
}

/// 新增观测 + 采纳失败都不得让上一版解冒充「当前前缀的解」。
#[test]
fn rejected_solve_does_not_impersonate_the_current_solution() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds::default(),
        scripted_backend(Arc::clone(&seen), ModelKind::Ds, (1280, 1088), 1),
    );
    session.ingest((0..20).map(observation));
    let first = session.update(true);
    assert!(first.solved, "首批应被采纳");
    assert!(session.has_current_solution(), "采纳并评估后应声称有当前解");
    let holdout_before = session.state().holdout_views;
    assert!(holdout_before > 0, "采纳后应有 holdout 指标");
    // 新增观测：前缀增长，旧解不再覆盖整个当前前缀。
    session.ingest((20..25).map(observation));
    assert!(
        !session.has_current_solution(),
        "新增观测后旧解不得冒充当前前缀"
    );
    // 强制求解：后端不收敛 → 解被拒绝。
    let second = session.update(true);
    assert!(!second.solved, "不收敛的解必须被拒绝");
    assert!(
        !session.has_current_solution(),
        "采纳失败不得沿用旧 holdout 冒充本版"
    );
    assert_eq!(
        session.state().holdout_views,
        holdout_before,
        "拒绝不得改写已采纳解的 holdout"
    );
}

/// 批次求解（含代表性限幅）后位姿仍须映射回原始观测，不能被误判成离群。
#[test]
fn forced_batch_solve_maps_poses_to_original_views() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 5, 0.25, 2),
        SessionThresholds::default(),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    session.ingest((0..20).map(observation));
    let outcome = session.update(true);
    assert!(outcome.solved, "20 条应能求解");
    session
        .evaluate()
        .expect("evaluate all original observations");
    let report = session.report().expect("report");
    assert!(
        report.rms_px < 1e-7,
        "批次映射错位会把正确位姿记成离群：{}",
        report.rms_px
    );
    assert_eq!(report.used_view_count, 20, "映射错位不得被离群掩盖");
    assert!(session.has_current_solution());
}

/// 判据全放宽、holdout 门槛也放宽的阈值：用于专注收敛 streak / 成败行为。
fn relaxed_thresholds(window: usize) -> SessionThresholds {
    SessionThresholds {
        max_rms_px: 1e9,
        max_focal_relative_stddev: 1e9,
        max_principal_stddev_px: 1e9,
        max_holdout_rms_px: 1e9,
        max_holdout_p95_px: 1e9,
        window,
    }
}

/// 全量精修必须越过 `max_solve_observations` 上限，且即使同前缀刚做过在线限量解，
/// 也要真的重解一次（用全部 train 视图）。
#[test]
fn refine_uses_all_training_views_even_after_a_limited_online_solve() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 5, 0.25, 2),
        SessionThresholds::default(),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    session.ingest((0..20).map(observation));
    let online = session.update(true);
    assert!(online.solved, "在线限量解应被采纳：{}", online.state.detail);
    let limited = seen.lock().expect("seen").clone();
    assert!(
        limited.iter().all(|count| *count <= 5),
        "在线求解受代表子集上限约束，实测 {limited:?}"
    );
    // 同一前缀已在线求解成功：最终精修仍必须真正重解，且用全部 train 视图。
    let refined = session.refine();
    assert!(refined.solved, "全量精修应被采纳：{}", refined.state.detail);
    let counts = seen.lock().expect("seen").clone();
    assert_eq!(counts.len(), 2, "精修必须真的再求解一次，实测 {counts:?}");
    assert_eq!(
        counts[1], 15,
        "精修须用全部 train 视图（20 条按 0.25 划分 → 15），实测 {counts:?}"
    );
    assert!(session.has_current_solution());
}

/// 同一前缀**成功**精修后的重复调用必须幂等：不重复求解，也不刷收敛 streak。
#[test]
fn repeated_refine_on_a_succeeded_prefix_is_idempotent() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        relaxed_thresholds(3),
        FakeBackend::boxed(Arc::clone(&seen), ModelKind::Ds, (1280, 1088)),
    );
    session.ingest((0..40).map(observation));
    let first = session.refine();
    assert!(
        first.solved && first.goal_met,
        "全量精修应被采纳并达标：{}",
        first.state.detail
    );
    let streak = first.state.streak;
    let calls = seen.lock().expect("seen").len();
    for _ in 0..3 {
        let repeated = session.refine();
        assert_eq!(repeated.solved, first.solved);
        assert_eq!(
            repeated.state.streak, streak,
            "同前缀重复精修不得刷 streak：{}",
            repeated.state.detail
        );
    }
    assert_eq!(
        seen.lock().expect("seen").len(),
        calls,
        "同前缀重复精修不得再求解"
    );
}

/// 最终精修失败不得沿用本前缀较早的在线解冒充成功；失败理由与状态必须显式保留，
/// 一次拒解只记一次失败计数。
#[test]
fn failed_refine_does_not_impersonate_the_earlier_online_solution() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        SessionThresholds::default(),
        scripted_backend(Arc::clone(&seen), ModelKind::Ds, (1280, 1088), 1),
    );
    session.ingest((0..20).map(observation));
    let online = session.update(true);
    assert!(online.solved, "首个在线解应被采纳");
    assert!(session.has_current_solution(), "在线解采纳后应声称有当前解");
    assert_eq!(session.state().failures, 0);
    // 同一前缀的最终精修：后端第二次起不收敛 → 必须失败，且不得沿用在线解冒充。
    let refined = session.refine();
    assert!(
        !refined.solved,
        "不收敛的精修必须失败：{}",
        refined.state.detail
    );
    assert!(
        !session.has_current_solution(),
        "精修失败不得沿用较早在线解冒充精修成功"
    );
    let state = session.state();
    assert_eq!(
        state.failures, 1,
        "一次拒解只应记一次失败，实测 {}",
        state.failures
    );
    assert_eq!(state.status, "refine_failed", "失败状态必须显式可见");
    assert!(
        state.detail.contains("backend did not converge"),
        "失败理由必须留在 detail：{}",
        state.detail
    );
    // 失败允许明确再次尝试：重试会再次抵达后端。
    let calls = seen.lock().expect("seen").len();
    let retry = session.refine();
    assert!(!retry.solved, "后端仍不收敛，重试仍失败");
    assert!(
        seen.lock().expect("seen").len() > calls,
        "失败精修必须允许再次尝试"
    );
}

/// 求解被拒后不得用旧解评估把收敛 streak 刷回来，也不得重复计数失败。
#[test]
fn rejected_solve_does_not_refresh_convergence_streak() {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let mut session = Session::new(
        options(ModelKind::Ds, 0, 0.25, 2),
        relaxed_thresholds(3),
        scripted_backend(Arc::clone(&seen), ModelKind::Ds, (1280, 1088), 1),
    );
    session.ingest((0..40).map(observation));
    let first = session.update(true);
    assert!(first.solved && first.goal_met, "首解应被采纳并达标");
    assert_eq!(first.state.streak, 1, "一次评估只推进一格");
    // 新增观测后的求解被后端拒绝：旧解评估不得把 streak 重新刷回来，也不得重复计数。
    session.ingest((40..50).map(observation));
    let failed = session.update(true);
    assert!(!failed.solved, "不收敛的解必须被拒绝");
    assert_eq!(
        failed.state.streak, 0,
        "拒解后 streak 必须归零且不被旧解评估刷新：{}",
        failed.state.detail
    );
    assert_eq!(
        failed.state.failures, 1,
        "一次拒解只记一次失败，实测 {}",
        failed.state.failures
    );
    assert!(
        failed.state.detail.contains("backend did not converge"),
        "失败理由留在 detail：{}",
        failed.state.detail
    );
}
