//! DS 模型的**求解侧**封装：重参数化、鲁棒化残差、种子。
//!
//! 投影/反投影本体在 `rigcal_core::models::ds`，这里只补"优化器要用的那层"。

use rigcal_core::estimator::IntrinsicPose;
use rigcal_core::models::{ModelKind, Parameters, ds::DsParameters};

/// 内参在优化器里的**无约束表示**：`[ln fx, ln fy, cx, cy, logit xi, logit alpha]`。
///
/// 优化器（`levenberg-marquardt`）不给 box 约束，所以用重参数化把
/// `fx>0`、`xi∈(-1,1)`、`alpha∈(0,1)` 作为本优化器的搜索域（主点不加约束）。
pub const INTRINSIC_PARAMS: usize = 6;
/// 每个视图 6 个位姿参数（rvec 3 + tvec 3）。
pub const POSE_PARAMS: usize = 6;

fn logit(value: f64, low: f64, high: f64) -> f64 {
    let normalized = ((value - low) / (high - low)).clamp(1e-6, 1.0 - 1e-6);
    (normalized / (1.0 - normalized)).ln()
}

fn sigmoid(value: f64, low: f64, high: f64) -> f64 {
    let normalized = 1.0 / (1.0 + (-value).exp());
    // 避免浮点饱和取到搜索区间端点。
    let clamped = normalized.clamp(1e-9, 1.0 - 1e-9);
    low + (high - low) * clamped
}

/// DS 内参 ↔ 无约束参数向量（前 6 维）。
pub fn intrinsics_to_unconstrained(params: &DsParameters) -> [f64; INTRINSIC_PARAMS] {
    [
        params.fx.max(1e-6).ln(),
        params.fy.max(1e-6).ln(),
        params.cx,
        params.cy,
        logit(params.xi, -1.0, 1.0),
        logit(params.alpha, 0.0, 1.0),
    ]
}

/// 无约束向量（前 6 维）→ DS 内参。
pub fn intrinsics_from_unconstrained(values: &[f64]) -> DsParameters {
    DsParameters {
        fx: values[0].exp().clamp(1e-3, 1e7),
        fy: values[1].exp().clamp(1e-3, 1e7),
        cx: values[2],
        cy: values[3],
        xi: sigmoid(values[4], -1.0, 1.0),
        alpha: sigmoid(values[5], 0.0, 1.0),
    }
}

/// 本仓 `Parameters` → DS 参数（类型不符返回 None）。
pub fn ds_from_parameters(params: &Parameters) -> Option<DsParameters> {
    match params {
        Parameters::Ds(ds) => Some(*ds),
        Parameters::Kb4(_) => None,
    }
}

/// DS 参数 → 本仓 `Parameters`。
pub fn parameters_from_ds(ds: DsParameters) -> Parameters {
    Parameters::Ds(ds)
}

/// 解码深度的下界：`tz = exp(s) ≥ MIN_POSE_DEPTH > 0`。
pub const MIN_POSE_DEPTH: f64 = 1e-6;
/// 解码深度的上界：`exp` 溢出前截断，保证输出有限。
pub const MAX_POSE_DEPTH: f64 = 1e9;

fn depth_from_unconstrained(value: f64) -> f64 {
    value.clamp(MIN_POSE_DEPTH.ln(), MAX_POSE_DEPTH.ln()).exp()
}

/// 位姿在优化器里的**无约束表示**：`rvec` 原样，平移的 z 分量取自然对数。
///
/// LM 不给 box 约束，直接优化自由 `tz` 会产出 `tz ≤ 0` 的解，而 `solve_ds_intrinsics`
/// 的输入契约（以及上层热启动）要求 `tz > 0`。把 `tz = exp(s)` 放进搜索域后，
/// 解码结果天然满足该契约：既不删输入检查，也不在失败后伪造位姿。
/// 编码端把 `tz` 下限压到 `MIN_POSE_DEPTH`（调用方传入的位姿已由输入检查保证 `tz > 0`）。
pub fn pose_to_unconstrained(pose: &IntrinsicPose) -> [f64; POSE_PARAMS] {
    let (rvec, tvec) = pose;
    [
        rvec[0],
        rvec[1],
        rvec[2],
        tvec[0],
        tvec[1],
        tvec[2].max(MIN_POSE_DEPTH).ln(),
    ]
}

/// 无约束参数向量 → DS 位姿；非 NaN 的深度解码为正且有限，NaN 留给投影有效域拒绝。
pub fn pose_from_unconstrained(values: &[f64; POSE_PARAMS]) -> IntrinsicPose {
    (
        [values[0], values[1], values[2]],
        [values[3], values[4], depth_from_unconstrained(values[5])],
    )
}

/// 满足 `r̃² = 2 f² (sqrt(1 + (r/f)²) − 1)` 的 soft_l1 残差。
/// 有理化避免小残差时 `sqrt(1+ε)−1` 消减成零，保留零点附近的单位导数。
pub fn soft_l1(residual: f64, f_scale: f64) -> f64 {
    let ratio = residual / f_scale;
    residual * (2.0 / (1.0 + ratio.hypot(1.0))).sqrt()
}

/// 内参种子（DS 求解的初值来源）。
#[derive(Clone, Copy, Debug)]
pub enum DsSeed {
    /// 直接从一组 DS 参数出发（调用方给，比如上一次的解）。
    Parameters(DsParameters),
    /// 使用请求尺寸及 core 中统一的模型默认值。
    Default,
}

impl DsSeed {
    pub fn to_parameters(&self, image_size: (u32, u32)) -> DsParameters {
        match *self {
            DsSeed::Parameters(params) => params,
            DsSeed::Default => match ModelKind::Ds.default_parameters(image_size) {
                Parameters::Ds(params) => params,
                Parameters::Kb4(_) => unreachable!("DS default parameters"),
            },
        }
    }
}

/// 参数向量的模型种类（DS 专用；留给上层做断言用）。
pub fn model_kind() -> ModelKind {
    ModelKind::Ds
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reparameterization_round_trips_inside_bounds() {
        let original = DsParameters {
            fx: 656.0,
            fy: 655.5,
            cx: 643.5,
            cy: 541.0,
            xi: 0.12,
            alpha: 0.55,
        };
        let values = intrinsics_to_unconstrained(&original);
        let back = intrinsics_from_unconstrained(&values);
        assert!((back.fx - original.fx).abs() < 1e-9);
        assert!((back.fy - original.fy).abs() < 1e-9);
        assert!((back.cx - original.cx).abs() < 1e-12);
        assert!((back.cy - original.cy).abs() < 1e-12);
        assert!((back.xi - original.xi).abs() < 1e-9);
        assert!((back.alpha - original.alpha).abs() < 1e-9);
        // 无约束取值永远不会跑出合法域
        let extreme = intrinsics_from_unconstrained(&[1e3, -1e3, 0.0, 0.0, 1e3, -1e3]);
        assert!(extreme.fx > 0.0 && extreme.xi > -1.0 && extreme.xi < 1.0);
        assert!(extreme.alpha > 0.0 && extreme.alpha < 1.0);
    }

    #[test]
    fn soft_l1_preserves_small_residuals_and_scale() {
        // 直接用 `sqrt(1+r²)-1` 在 r→0 时会消减成零，数值雅可比失去导数。
        for residual in [-1e-10, 1e-10] {
            assert!((soft_l1(residual, 2.0) / residual - 1.0).abs() < 1e-12);
        }
        // 非单位尺度时必须仍是像素残差；少乘一个 f 会改变目标函数的单位。
        let expected_cost = 8.0 * (26.0_f64.sqrt() - 1.0);
        assert!((soft_l1(10.0, 2.0).powi(2) - expected_cost).abs() < 1e-12);
        assert_eq!(soft_l1(-10.0, 2.0), -soft_l1(10.0, 2.0));
    }

    #[test]
    fn pose_reparameterization_keeps_depth_positive() {
        // 极端无约束取值解码后深度恒 > 0 且有限：LM 输出因此总能作为下轮热启动输入，
        // 不必删 `tz > 0` 输入检查，也不会在失败后伪造位姿。
        for value in [
            f64::NEG_INFINITY,
            -1e3,
            MIN_POSE_DEPTH.ln(),
            0.0,
            MAX_POSE_DEPTH.ln(),
            1e3,
            f64::INFINITY,
        ] {
            let (_, tvec) = pose_from_unconstrained(&[0.0, 0.0, 0.0, 0.1, 0.2, value]);
            assert!(
                tvec[2] > 0.0 && tvec[2].is_finite(),
                "value={value} tz={}",
                tvec[2]
            );
        }
        // 正常深度范围内编解码无损，且旋转/横向平移原样保留。
        let original = ([0.1, -0.2, 0.3], [0.4, -0.5, 0.6]);
        let back = pose_from_unconstrained(&pose_to_unconstrained(&original));
        assert_eq!(back.0, original.0);
        assert!((back.1[0] - original.1[0]).abs() < 1e-12);
        assert!((back.1[1] - original.1[1]).abs() < 1e-12);
        assert!((back.1[2] - original.1[2]).abs() < 1e-12);
        let (_, invalid_translation) =
            pose_from_unconstrained(&[0.0, 0.0, 0.0, 0.0, 0.0, f64::NAN]);
        let (_, valid) = rigcal_core::models::ds::project(
            &[invalid_translation],
            &DsSeed::Default.to_parameters((1280, 1088)),
        );
        assert!(!valid[0], "NaN 深度不能伪装成最小正深度投影");
    }
}
