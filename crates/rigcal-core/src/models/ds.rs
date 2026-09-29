//! Double Sphere 投影/反投影（Usenko et al. 2018，闭式解）。

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DsParameters {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub xi: f64,
    pub alpha: f64,
}

/// 全局参数域：全部有限，`fx, fy > 0`，`alpha ∈ (0, 1)`；非有限参数一律 fail closed。
fn globals_ok(p: &DsParameters) -> bool {
    p.fx > 0.0
        && p.fy > 0.0
        && p.alpha > 0.0
        && p.alpha < 1.0
        && p.fx.is_finite()
        && p.fy.is_finite()
        && p.cx.is_finite()
        && p.cy.is_finite()
        && p.xi.is_finite()
        && p.alpha.is_finite()
}

/// Double Sphere 可逆域的 `w2`（Usenko et al. 2018, Eq. 44–45）。
///
/// `w1` 按 `alpha` 分支；`alpha ∈ (0,1)` 保证 `w1 ∈ (0,1)`、`w2` 有限。
fn domain_w2(p: &DsParameters) -> f64 {
    let w1 = if p.alpha > 0.5 {
        (1.0 - p.alpha) / p.alpha
    } else {
        p.alpha / (1.0 - p.alpha)
    };
    (w1 + p.xi) / (2.0 * w1 * p.xi + p.xi * p.xi + 1.0).sqrt()
}

/// 有效域 `Ω = {z > -w2·d1}`：背轴 `(0, 0, -1)` 在域外，
/// 而 `z ≤ 0` 的宽角射线只要方向足够靠前仍然合法。
fn domain_ok(z: f64, d1: f64, p: &DsParameters) -> bool {
    z > -domain_w2(p) * d1
}

/// 相机点 → 像素；只有落在可逆域内的点才有效，其余返回 NaN 且掩码为 `false`。
pub fn project(points: &[[f64; 3]], p: &DsParameters) -> (Vec<[f64; 2]>, Vec<bool>) {
    let global = globals_ok(p);
    let mut pixels = vec![[f64::NAN, f64::NAN]; points.len()];
    let mut valid = vec![false; points.len()];
    for (index, point) in points.iter().enumerate() {
        let [x, y, z] = *point;
        let d1 = (x * x + y * y + z * z).sqrt();
        let shifted_z = p.xi * d1 + z;
        let d2 = (x * x + y * y + shifted_z * shifted_z).sqrt();
        let denominator = p.alpha * d2 + (1.0 - p.alpha) * shifted_z;
        let ok = global
            && d1.is_finite()
            && d1 > 0.0
            && domain_ok(z, d1, p)
            && denominator.is_finite()
            && denominator > 1e-12;
        if ok {
            pixels[index] = [p.fx * x / denominator + p.cx, p.fy * y / denominator + p.cy];
            valid[index] = true;
        }
    }
    (pixels, valid)
}

/// 闭式逆模型；有效域判据与投影保持同一坐标合同。
pub fn unproject(pixels: &[[f64; 2]], p: &DsParameters) -> (Vec<[f64; 3]>, Vec<bool>) {
    let global = globals_ok(p);
    let mut rays = vec![[f64::NAN; 3]; pixels.len()];
    let mut valid = vec![false; pixels.len()];
    for (index, pixel) in pixels.iter().enumerate() {
        let mx = (pixel[0] - p.cx) / p.fx;
        let my = (pixel[1] - p.cy) / p.fy;
        let r2 = mx * mx + my * my;
        let inside = 1.0 - (2.0 * p.alpha - 1.0) * r2;
        if !global || !r2.is_finite() || inside < 0.0 {
            continue;
        }
        let sqrt_inside = inside.max(0.0).sqrt();
        let mz = (1.0 - p.alpha * p.alpha * r2) / (p.alpha * sqrt_inside + 1.0 - p.alpha);
        let denominator = mz * mz + r2;
        let sqrt_argument = mz * mz + (1.0 - p.xi * p.xi) * r2;
        if sqrt_argument < 0.0 {
            continue;
        }
        let scale = (mz * p.xi + sqrt_argument.sqrt()) / denominator;
        let unnormalized = [scale * mx, scale * my, scale * mz - p.xi];
        let norm = (unnormalized[0] * unnormalized[0]
            + unnormalized[1] * unnormalized[1]
            + unnormalized[2] * unnormalized[2])
            .sqrt();
        if norm <= 1e-12 {
            continue;
        }
        rays[index] = [
            unnormalized[0] / norm,
            unnormalized[1] / norm,
            unnormalized[2] / norm,
        ];
        valid[index] = true;
    }
    (rays, valid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(alpha: f64, xi: f64) -> DsParameters {
        DsParameters {
            fx: 648.0,
            fy: 646.5,
            cx: 640.0,
            cy: 544.0,
            xi,
            alpha,
        }
    }

    /// 单位方向：横向固定方位，纵向 `uz`。
    fn ray(uz: f64) -> [f64; 3] {
        let transverse = (1.0 - uz * uz).sqrt();
        [0.8 * transverse, -0.6 * transverse, uz]
    }

    #[test]
    fn back_axis_is_outside_the_invertible_domain() {
        // 仅判 denominator > 0 在 alpha > 0.5 时对背轴恰为正，会把 (0, 0, -1) 投到主点；
        // 可逆域 z > -w2·d1 必须拒绝它。
        for alpha in [0.51, 0.56, 0.7, 0.9] {
            let p = params(alpha, 0.2);
            let (pixels, valid) = project(&[[0.0, 0.0, -1.0]], &p);
            assert_eq!(valid, vec![false], "alpha={alpha}");
            assert!(pixels[0][0].is_nan() && pixels[0][1].is_nan());
        }
        // alpha ≤ 0.5 分支同样拒绝背轴。
        let p = params(0.4, 0.2);
        assert_eq!(project(&[[0.0, 0.0, -1.0]], &p).1, vec![false]);
    }

    #[test]
    fn wide_angle_backward_rays_stay_valid_and_round_trip() {
        // z < 0 但仍在可逆域内的宽角射线不能被误拒；
        // project→unproject 必须还原同一方向（单射有效域）。
        let p = params(0.56, 0.18);
        let w2 = domain_w2(&p);
        assert!(w2 > 0.0 && w2 < 1.0, "w2={w2}");

        let inside = ray(-0.6 * w2);
        let (pixels, valid) = project(&[inside], &p);
        assert_eq!(valid, vec![true], "域内 z<0 射线被误拒");
        let (rays, ray_valid) = unproject(&pixels, &p);
        assert_eq!(ray_valid, vec![true]);
        for axis in 0..3 {
            assert!(
                (rays[0][axis] - inside[axis]).abs() < 1e-9,
                "方向不一致：{:?} vs {inside:?}",
                rays[0]
            );
        }

        // 越过域边界必须 fail closed（严格 z > -w2·d1）。
        let outside = ray(-1.01 * w2);
        assert_eq!(project(&[outside], &p).1, vec![false]);
    }

    #[test]
    fn unproject_domain_is_fail_closed_beyond_the_image_disk() {
        // alpha > 0.5 时 Θ 是有界圆盘 r² ≤ 1/(2α-1)；圆盘外必须 fail closed，
        // 否则反投影会给出投影域之外的射线，破坏 project↔unproject 合同。
        let p = params(0.56, 0.18);
        let radius = 1.0 / (2.0 * p.alpha - 1.0);
        let inside = [p.cx + p.fx * (0.5 * radius).sqrt(), p.cy];
        let outside = [p.cx + p.fx * (1.01 * radius).sqrt(), p.cy];
        assert_eq!(unproject(&[inside], &p).1, vec![true]);
        assert_eq!(unproject(&[outside], &p).1, vec![false]);
    }

    #[test]
    fn non_finite_parameters_fail_closed() {
        let base = params(0.56, 0.18);
        let point = [0.2, -0.1, 0.9];
        let broken = [
            DsParameters {
                fx: f64::NAN,
                ..base
            },
            DsParameters {
                fy: f64::INFINITY,
                ..base
            },
            DsParameters {
                cx: f64::NAN,
                ..base
            },
            DsParameters {
                cy: f64::NEG_INFINITY,
                ..base
            },
            DsParameters {
                xi: f64::NAN,
                ..base
            },
            DsParameters {
                alpha: f64::NAN,
                ..base
            },
        ];
        for p in broken {
            let (pixels, valid) = project(&[point], &p);
            assert_eq!(valid, vec![false], "{p:?}");
            assert!(pixels[0][0].is_nan() && pixels[0][1].is_nan());
        }
    }
}
