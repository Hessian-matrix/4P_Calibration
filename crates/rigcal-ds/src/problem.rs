//! DS bundle adjustment：第三方 LM，6 个内参 + 每视图 6 个位姿。
//! 中心差分只计算受扰动参数影响的视图；位姿列不会重算其它视图。

use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::{DMatrix, DVector, Dyn, Owned};
use rigcal_core::estimator::{
    IntrinsicPose, Observation, SolveOutcome, Status, evaluate_solution, seed_pose,
};
use rigcal_core::models::{ModelKind, Parameters, ds::DsParameters};

use crate::DsSolveError;
use crate::model::{
    DsSeed, INTRINSIC_PARAMS, POSE_PARAMS, intrinsics_from_unconstrained,
    intrinsics_to_unconstrained, parameters_from_ds, pose_from_unconstrained,
    pose_to_unconstrained, soft_l1,
};

const INVALID_RESIDUAL: f64 = 1e3;
const DIFFERENCE_STEP: f64 = 6.0e-6;

#[derive(Clone, Copy, Debug)]
pub struct DsBundleOptions<'a> {
    /// soft_l1 尺度（像素）。
    pub soft_l1_scale: f64,
    /// LM 的求值预算因子：最多 patience × (参数数 + 1) 次残差求值，不是迭代数。
    pub patience: usize,
    pub seed: DsSeed,
    /// 与 observations 同长同序；如有已收敛的上游位姿则保留，而不是重新播种。
    pub initial_poses: Option<&'a [IntrinsicPose]>,
}

impl Default for DsBundleOptions<'_> {
    fn default() -> Self {
        Self {
            soft_l1_scale: 2.0,
            patience: 10,
            seed: DsSeed::Default,
            initial_poses: None,
        }
    }
}

struct DsBundle<'a> {
    observations: &'a [Observation],
    soft_l1_scale: f64,
    params: DVector<f64>,
    residual_count: usize,
}

impl<'a> DsBundle<'a> {
    fn new(
        observations: &'a [Observation],
        image_size: (u32, u32),
        options: &DsBundleOptions<'_>,
    ) -> Self {
        let seed = options.seed.to_parameters(image_size);
        let parameters = parameters_from_ds(seed);
        let mut values = Vec::with_capacity(INTRINSIC_PARAMS + POSE_PARAMS * observations.len());
        values.extend_from_slice(&intrinsics_to_unconstrained(&seed));
        for (index, observation) in observations.iter().enumerate() {
            if let Some(poses) = options.initial_poses {
                values.extend_from_slice(&pose_to_unconstrained(&poses[index]));
            } else {
                let seeded = seed_pose(ModelKind::Ds, &parameters, observation);
                let pose = (
                    [seeded[0], seeded[1], seeded[2]],
                    [seeded[3], seeded[4], seeded[5]],
                );
                values.extend_from_slice(&pose_to_unconstrained(&pose));
            }
        }
        Self {
            observations,
            soft_l1_scale: options.soft_l1_scale,
            params: DVector::from_vec(values),
            residual_count: observations
                .iter()
                .map(|obs| 2 * obs.image_points.len())
                .sum(),
        }
    }

    fn view_residuals(&self, view: usize, ds: DsParameters, params: &[f64], out: &mut [f64]) {
        let observation = &self.observations[view];
        let base = INTRINSIC_PARAMS + view * POSE_PARAMS;
        let pose =
            <&[f64; POSE_PARAMS]>::try_from(&params[base..base + POSE_PARAMS]).expect("pose");
        let (rvec, tvec) = pose_from_unconstrained(pose);
        let camera_points =
            rigcal_core::models::transform_points(&observation.object_points, &rvec, &tvec);
        let (pixels, valid) = rigcal_core::models::ds::project(&camera_points, &ds);
        for (((pixel, valid), observed), residual) in pixels
            .iter()
            .zip(valid)
            .zip(&observation.image_points)
            .zip(out.as_chunks_mut::<2>().0)
        {
            for axis in 0..2 {
                let difference = if valid {
                    pixel[axis] - observed[axis]
                } else {
                    INVALID_RESIDUAL
                };
                residual[axis] = soft_l1(difference, self.soft_l1_scale);
            }
        }
    }

    fn compute_residuals(&self, params: &[f64], out: &mut [f64]) {
        let ds = intrinsics_from_unconstrained(params);
        let mut start = 0;
        for (view, observation) in self.observations.iter().enumerate() {
            let end = start + 2 * observation.image_points.len();
            self.view_residuals(view, ds, params, &mut out[start..end]);
            start = end;
        }
    }
}

impl LeastSquaresProblem<f64, Dyn, Dyn> for DsBundle<'_> {
    type ParameterStorage = Owned<f64, Dyn>;
    type ResidualStorage = Owned<f64, Dyn>;
    type JacobianStorage = Owned<f64, Dyn, Dyn>;

    fn set_params(&mut self, params: &DVector<f64>) {
        self.params.copy_from(params);
    }
    fn params(&self) -> DVector<f64> {
        self.params.clone()
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        let mut residuals = DVector::zeros(self.residual_count);
        self.compute_residuals(self.params.as_slice(), residuals.as_mut_slice());
        Some(residuals)
    }

    fn jacobian(&self) -> Option<DMatrix<f64>> {
        let mut jacobian = DMatrix::zeros(self.residual_count, self.params.len());
        let mut probe = self.params.clone();
        let mut plus = vec![0.0; self.residual_count];
        let mut minus = vec![0.0; self.residual_count];
        let ds = intrinsics_from_unconstrained(self.params.as_slice());
        let mut row_start = 0;
        for column in 0..self.params.len() {
            let value = self.params[column];
            let step = DIFFERENCE_STEP * value.abs().max(1.0);
            let (start, end) = if column < INTRINSIC_PARAMS {
                probe[column] = value + step;
                self.compute_residuals(probe.as_slice(), &mut plus);
                probe[column] = value - step;
                self.compute_residuals(probe.as_slice(), &mut minus);
                (0, self.residual_count)
            } else {
                let view = (column - INTRINSIC_PARAMS) / POSE_PARAMS;
                let len = 2 * self.observations[view].image_points.len();
                probe[column] = value + step;
                self.view_residuals(view, ds, probe.as_slice(), &mut plus[..len]);
                probe[column] = value - step;
                self.view_residuals(view, ds, probe.as_slice(), &mut minus[..len]);
                (row_start, row_start + len)
            };
            let offset = if column < INTRINSIC_PARAMS { start } else { 0 };
            for (index, row) in (start..end).enumerate() {
                jacobian[(row, column)] =
                    (plus[offset + index] - minus[offset + index]) / (2.0 * step);
            }
            if column >= INTRINSIC_PARAMS
                && (column - INTRINSIC_PARAMS + 1).is_multiple_of(POSE_PARAMS)
            {
                row_start = end;
            }
            probe[column] = value;
        }
        Some(jacobian)
    }
}

/// 联合求解后直接返回优化的位姿，评估与优化使用同一个解。
pub fn solve_ds_intrinsics(
    observations: &[Observation],
    image_size: (u32, u32),
    options: &DsBundleOptions<'_>,
) -> Result<SolveOutcome, DsSolveError> {
    if observations.is_empty()
        || image_size.0 == 0
        || image_size.1 == 0
        || !options.soft_l1_scale.is_finite()
        || options.soft_l1_scale <= 0.0
        || options.patience == 0
        || observations.iter().any(|obs| {
            obs.object_points.len() < 4
                || obs.object_points.len() != obs.image_points.len()
                || obs
                    .object_points
                    .iter()
                    .flatten()
                    .chain(obs.image_points.iter().flatten())
                    .any(|x| !x.is_finite())
        })
        || options.initial_poses.is_some_and(|poses| {
            poses.len() != observations.len()
                || poses
                    .iter()
                    .any(|(r, t)| r.iter().chain(t).any(|x| !x.is_finite()) || t[2] <= 0.0)
        })
    {
        return Err(DsSolveError::Rust("无效的观测、位姿或求解选项".to_owned()));
    }
    let problem = DsBundle::new(observations, image_size, options);
    let (result, report) = LevenbergMarquardt::new()
        .with_ftol(1e-10)
        .with_xtol(1e-10)
        .with_gtol(1e-10)
        .with_patience(options.patience)
        .minimize(problem);
    if !report.termination.was_successful() || !report.objective_function.is_finite() {
        return Err(DsSolveError::Rust(format!(
            "LM 未收敛：{:?}（objective={}）",
            report.termination, report.objective_function
        )));
    }
    let parameters = parameters_from_ds(intrinsics_from_unconstrained(result.params.as_slice()));
    let poses: Vec<IntrinsicPose> = result.params.as_slice()[INTRINSIC_PARAMS..]
        .as_chunks::<POSE_PARAMS>()
        .0
        .iter()
        .map(pose_from_unconstrained)
        .collect();
    let evaluation = evaluate_solution(ModelKind::Ds, observations, &parameters, &poses);
    let mut residuals_px = Vec::with_capacity(result.residual_count / 2);
    for (observation, (rvec, tvec)) in observations.iter().zip(&poses) {
        let (pixels, valid) = rigcal_core::estimator::project_with_pose(
            ModelKind::Ds,
            &parameters,
            &observation.object_points,
            rvec,
            tvec,
        );
        residuals_px.extend(
            pixels
                .iter()
                .zip(&observation.image_points)
                .zip(valid)
                .filter_map(|((pixel, observed), valid)| {
                    valid.then_some([pixel[0] - observed[0], pixel[1] - observed[1]])
                }),
        );
    }
    Ok(SolveOutcome {
        parameters,
        poses,
        residuals_px,
        invalid_projection_count: evaluation.invalid_projection_count,
        rms_px: evaluation.rms_px,
        status: if evaluation.invalid_projection_count == 0 && evaluation.rms_px.is_finite() {
            Status::Pass
        } else {
            Status::Fail
        },
    })
}

/// KB4 → DS 初值，保留轴心处的角度斜率，而不是直接比较两模型的原始焦距。
pub fn kb4_seed(parameters: &Parameters) -> Option<DsSeed> {
    let Parameters::Kb4(kb4) = parameters else {
        return None;
    };
    // xi=0 的内参雅可比存在一阶零方向。选非零 xi，匹配 r(theta) 的一阶和三阶项：
    // f_DS/(1+xi)=f_KB4；k1=(s-alpha)/(2s²)-1/6，s=1+xi。
    let xi = 0.2;
    let s = 1.0 + xi;
    Some(DsSeed::Parameters(DsParameters {
        fx: kb4.fx * s,
        fy: kb4.fy * s,
        cx: kb4.cx,
        cy: kb4.cy,
        xi,
        alpha: (s - 2.0 * s * s * (kb4.k1 + 1.0 / 6.0)).clamp(0.05, 0.95),
    }))
}
