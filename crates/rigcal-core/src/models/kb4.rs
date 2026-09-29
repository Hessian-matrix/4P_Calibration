//! KB4（OpenCV fisheye / 等距）投影与反投影。
//!
//! 关键约定：只在**第一单调分支**内有效（`theta <= theta_limit`），正反投影共用同一分支判据，
//! 保证 project/unproject 合同一致；反解用有界 Newton（16 次迭代）并以残差 ≤ 1e-10 收尾。

use serde::{Deserialize, Serialize};

const MONOTONIC_SAMPLES: usize = 2049;
const NEWTON_ITERATIONS: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Kb4Parameters {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub k1: f64,
    pub k2: f64,
    pub k3: f64,
    pub k4: f64,
}

fn theta_distorted(theta: f64, p: &Kb4Parameters) -> f64 {
    let theta2 = theta * theta;
    let theta4 = theta2 * theta2;
    let theta6 = theta4 * theta2;
    let theta8 = theta4 * theta4;
    theta * (1.0 + p.k1 * theta2 + p.k2 * theta4 + p.k3 * theta6 + p.k4 * theta8)
}

/// `theta` 的第一单调可逆上界：在 `linspace(0, pi, 2049)` 上找导数首次 ≤ 1e-9 的位置前一点。
pub fn first_monotonic_theta_limit(p: &Kb4Parameters) -> f64 {
    let step = std::f64::consts::PI / (MONOTONIC_SAMPLES - 1) as f64;
    for index in 0..MONOTONIC_SAMPLES {
        let theta = index as f64 * step;
        let theta2 = theta * theta;
        let derivative = 1.0
            + 3.0 * p.k1 * theta2
            + 5.0 * p.k2 * theta2 * theta2
            + 7.0 * p.k3 * theta2 * theta2 * theta2
            + 9.0 * p.k4 * theta2 * theta2 * theta2 * theta2;
        if derivative <= 1e-9 {
            return (index.saturating_sub(1)).max(1) as f64 * step;
        }
    }
    std::f64::consts::PI
}

pub fn project(points: &[[f64; 3]], p: &Kb4Parameters) -> (Vec<[f64; 2]>, Vec<bool>) {
    let global = p.fx > 0.0 && p.fy > 0.0;
    let theta_limit = first_monotonic_theta_limit(p);
    let mut pixels = vec![[f64::NAN, f64::NAN]; points.len()];
    let mut valid = vec![false; points.len()];
    for (index, point) in points.iter().enumerate() {
        let [x, y, z] = *point;
        let norm = (x * x + y * y + z * z).sqrt();
        if !(global && x.is_finite() && y.is_finite() && z.is_finite() && norm > 1e-12) {
            continue;
        }
        let xy_radius = (x * x + y * y).sqrt();
        let theta = xy_radius.atan2(z);
        let theta_d = theta_distorted(theta, p);
        let (dir_x, dir_y) = if xy_radius > 1e-12 {
            (x / xy_radius, y / xy_radius)
        } else {
            (0.0, 0.0)
        };
        let pixel = [p.fx * theta_d * dir_x + p.cx, p.fy * theta_d * dir_y + p.cy];
        // 分支外的点即使算得出数也必须判无效：否则与反投影的有效域不一致。
        if theta <= theta_limit + 1e-12 && pixel[0].is_finite() && pixel[1].is_finite() {
            pixels[index] = pixel;
            valid[index] = true;
        }
    }
    (pixels, valid)
}

pub fn unproject(pixels: &[[f64; 2]], p: &Kb4Parameters) -> (Vec<[f64; 3]>, Vec<bool>) {
    let global = p.fx > 0.0 && p.fy > 0.0;
    let theta_limit = first_monotonic_theta_limit(p);
    let max_radius = theta_distorted(theta_limit, p);
    let mut rays = vec![[f64::NAN; 3]; pixels.len()];
    let mut valid = vec![false; pixels.len()];
    for (index, pixel) in pixels.iter().enumerate() {
        let mx = (pixel[0] - p.cx) / p.fx;
        let my = (pixel[1] - p.cy) / p.fy;
        let theta_d = (mx * mx + my * my).sqrt();
        if !(global && theta_d.is_finite() && theta_d <= max_radius + 1e-12) {
            continue;
        }
        let mut theta = theta_d.min(theta_limit);
        let mut alive = true;
        for _ in 0..NEWTON_ITERATIONS {
            let theta2 = theta * theta;
            let theta4 = theta2 * theta2;
            let theta6 = theta4 * theta2;
            let theta8 = theta4 * theta4;
            let value = theta
                * (1.0 + p.k1 * theta2 + p.k2 * theta4 + p.k3 * theta6 + p.k4 * theta8)
                - theta_d;
            let derivative = 1.0
                + 3.0 * p.k1 * theta2
                + 5.0 * p.k2 * theta4
                + 7.0 * p.k3 * theta6
                + 9.0 * p.k4 * theta8;
            let step_ok = derivative.abs() > 1e-12;
            if step_ok {
                theta = (theta - value / derivative).clamp(0.0, theta_limit);
            }
            alive = step_ok || theta_d <= 1e-12;
            if !alive {
                break;
            }
        }
        if !alive {
            continue;
        }
        let (dir_x, dir_y) = if theta_d > 1e-12 {
            (mx / theta_d, my / theta_d)
        } else {
            (0.0, 0.0)
        };
        // KB4 逆模型返回单位球面射线，z 分量是 cos(theta) 而不是固定 1。
        let unnormalized = [theta.sin() * dir_x, theta.sin() * dir_y, theta.cos()];
        let norm = (unnormalized[0] * unnormalized[0]
            + unnormalized[1] * unnormalized[1]
            + unnormalized[2] * unnormalized[2])
            .sqrt();
        let residual = (theta_distorted(theta, p) - theta_d).abs();
        if unnormalized.iter().all(|value| value.is_finite()) && norm > 1e-12 && residual <= 1e-10 {
            rays[index] = [
                unnormalized[0] / norm,
                unnormalized[1] / norm,
                unnormalized[2] / norm,
            ];
            valid[index] = true;
        }
    }
    (rays, valid)
}
