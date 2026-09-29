//! 投影模型：Double Sphere（`ds`）与 KB4/等距（`kb4`）。
//!
//! 两个模型使用同一套有效域判据与 NaN 约定：
//! 无效投影一律返回 NaN 并由布尔掩码标出，绝不静默夹取到边界。

pub mod ds;
pub mod kb4;

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ModelKind {
    Ds,
    Kb4,
}

impl ModelKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ModelKind::Ds => "ds",
            ModelKind::Kb4 => "kb4",
        }
    }

    pub fn parse(text: &str) -> Option<Self> {
        match text {
            "ds" => Some(ModelKind::Ds),
            "kb4" => Some(ModelKind::Kb4),
            _ => None,
        }
    }

    /// 参数向量顺序。
    pub fn parameter_names(self) -> &'static [&'static str] {
        match self {
            ModelKind::Ds => &["fx", "fy", "cx", "cy", "xi", "alpha"],
            ModelKind::Kb4 => &["fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4"],
        }
    }

    pub fn parameter_count(self) -> usize {
        self.parameter_names().len()
    }

    /// 由配置给出的模型默认初值（0.45·max(w,h)、主点居中）。
    pub fn default_parameters(self, image_size: (u32, u32)) -> Parameters {
        let (width, height) = (image_size.0 as f64, image_size.1 as f64);
        let focal = 0.45 * width.max(height);
        match self {
            ModelKind::Ds => Parameters::Ds(ds::DsParameters {
                fx: focal,
                fy: focal,
                cx: width / 2.0,
                cy: height / 2.0,
                xi: 0.4,
                alpha: 0.55,
            }),
            ModelKind::Kb4 => Parameters::Kb4(kb4::Kb4Parameters {
                fx: focal,
                fy: focal,
                cx: width / 2.0,
                cy: height / 2.0,
                k1: 0.0,
                k2: 0.0,
                k3: 0.0,
                k4: 0.0,
            }),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Parameters {
    Ds(ds::DsParameters),
    Kb4(kb4::Kb4Parameters),
}

impl Parameters {
    pub fn kind(&self) -> ModelKind {
        match self {
            Parameters::Ds(_) => ModelKind::Ds,
            Parameters::Kb4(_) => ModelKind::Kb4,
        }
    }

    pub fn as_vector(&self) -> Vec<f64> {
        match self {
            Parameters::Ds(p) => vec![p.fx, p.fy, p.cx, p.cy, p.xi, p.alpha],
            Parameters::Kb4(p) => vec![p.fx, p.fy, p.cx, p.cy, p.k1, p.k2, p.k3, p.k4],
        }
    }

    pub fn from_vector(kind: ModelKind, values: &[f64]) -> Option<Self> {
        if values.len() != kind.parameter_count() {
            return None;
        }
        match kind {
            ModelKind::Ds => Some(Parameters::Ds(ds::DsParameters {
                fx: values[0],
                fy: values[1],
                cx: values[2],
                cy: values[3],
                xi: values[4],
                alpha: values[5],
            })),
            ModelKind::Kb4 => Some(Parameters::Kb4(kb4::Kb4Parameters {
                fx: values[0],
                fy: values[1],
                cx: values[2],
                cy: values[3],
                k1: values[4],
                k2: values[5],
                k3: values[6],
                k4: values[7],
            })),
        }
    }

    pub fn focal_mean(&self) -> f64 {
        let values = self.as_vector();
        0.5 * (values[0].abs() + values[1].abs()).max(1.0)
    }
}

/// 板点 → 相机点（`P_cam = R(rvec)·P_board + tvec`）。
pub fn transform_points(
    object_points: &[[f64; 3]],
    rvec: &[f64; 3],
    tvec: &[f64; 3],
) -> Vec<[f64; 3]> {
    let rotation =
        nalgebra::Rotation3::from_scaled_axis(nalgebra::Vector3::new(rvec[0], rvec[1], rvec[2]));
    object_points
        .iter()
        .map(|point| {
            let vector = nalgebra::Vector3::new(point[0], point[1], point[2]);
            let rotated = rotation * vector;
            [
                rotated.x + tvec[0],
                rotated.y + tvec[1],
                rotated.z + tvec[2],
            ]
        })
        .collect()
}

/// 相机点 → 像素；无效点返回 NaN 且掩码为 `false`。
pub fn project(
    kind: ModelKind,
    points: &[[f64; 3]],
    params: &Parameters,
) -> (Vec<[f64; 2]>, Vec<bool>) {
    match (kind, params) {
        (ModelKind::Ds, Parameters::Ds(p)) => ds::project(points, p),
        (ModelKind::Kb4, Parameters::Kb4(p)) => kb4::project(points, p),
        _ => panic!("parameters do not match the requested model kind"),
    }
}

/// 像素 → 单位射线；无效点返回 NaN 且掩码为 `false`。
pub fn unproject(
    kind: ModelKind,
    pixels: &[[f64; 2]],
    params: &Parameters,
) -> (Vec<[f64; 3]>, Vec<bool>) {
    match (kind, params) {
        (ModelKind::Ds, Parameters::Ds(p)) => ds::unproject(pixels, p),
        (ModelKind::Kb4, Parameters::Kb4(p)) => kb4::unproject(pixels, p),
        _ => panic!("parameters do not match the requested model kind"),
    }
}
