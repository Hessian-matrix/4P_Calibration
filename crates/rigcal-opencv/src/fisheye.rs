//! KB4（OpenCV fisheye / 等距）求解后端。
//!
//! 走 `cv::fisheye::calibrate`（C++ 实现的 LM）。
//!
//! 绑定细节：生成的绑定把 fisheye 函数**展平**在标定模块里（不是 `calib::fisheye::calibrate`）；
//! 入参用 `Vector<Mat>`（未生成嵌套 `Vector<Vector<Point3f>>` 的 `VectorExtern`）。
//!
//! 版本差异（4.x ↔ 5.x）——OpenCV 5 把标定 API 收进新模块 `calib`，4.x 仍在 `calib3d`：
//! - 模块：5.x `calib`，4.x `calib3d`；
//! - 复算外参常量：5.x 与 pinhole 标志同枚举，`calib::CALIB_RECOMPUTE_EXTRINSIC`（`1 << 23`）；
//!   4.x 在 `fisheye` 命名空间里单独一份，生成为 `calib3d::Fisheye_CALIB_RECOMPUTE_EXTRINSIC`（值 `2`）；
//! - `CALIB_USE_INTRINSIC_GUESS` 两版都在模块根，值都是 `1`。
//!
//! opencv crate 只在它**自己**的构建里设置 `ocvrs_opencv_branch_*` cfg，外部 crate 拿不到这些
//! cfg，所以用 opencv 导出的条件宏做编译期二选一：未命中的那一支整体丢弃（其中的名字不会被解析）。

use opencv::core::{self, Mat, Point2f, Point3f, Size, TermCriteria, Vector};
use opencv::prelude::*;
use rigcal_core::estimator::{IntrinsicPose, Observation, SolveOutcome, SolveRequest, Status};
use rigcal_core::kb4::Kb4Parameters;
use rigcal_core::{ModelKind, Parameters};

opencv::not_opencv_branch_5! {
    use opencv::calib3d as fisheye;
}
opencv::opencv_branch_5! {
    use opencv::calib as fisheye;
}

/// 5.x 的 `CALIB_RECOMPUTE_EXTRINSIC`（`1 << 23`）/ 4.x fisheye 命名空间的同名标志（`2`）。
///
/// 两版都表示“每次迭代重算板位姿”，语义一致，只是枚举位置与取值不同。
const FISHEYE_RECOMPUTE_EXTRINSIC: i32 = opencv::opencv_branch_5! {
    { fisheye::CALIB_RECOMPUTE_EXTRINSIC } else { fisheye::Fisheye_CALIB_RECOMPUTE_EXTRINSIC }
};
/// `CALIB_USE_INTRINSIC_GUESS | CALIB_RECOMPUTE_EXTRINSIC`。
const FISHEYE_FLAGS: i32 = fisheye::CALIB_USE_INTRINSIC_GUESS | FISHEYE_RECOMPUTE_EXTRINSIC;
/// `COUNT|EPS` 终止判据的迭代上限。
const MAX_ITERATIONS: i32 = 200;
const EPSILON: f64 = 1.0e-10;

#[derive(Debug, thiserror::Error)]
pub enum OpenCvSolveError {
    #[error("observation {view} has no points")]
    EmptyView { view: usize },
    #[error("OpenCV {operation} failed: {message}")]
    OpenCv {
        operation: &'static str,
        message: String,
    },
    #[error("OpenCV returned {got} poses for {expected} views")]
    PoseCount { got: usize, expected: usize },
    #[error("unsupported request: {0}")]
    Unsupported(String),
}

fn cv(operation: &'static str, error: opencv::Error) -> OpenCvSolveError {
    OpenCvSolveError::OpenCv {
        operation,
        message: error.to_string(),
    }
}

fn default_kb4(image_size: (u32, u32)) -> Kb4Parameters {
    match ModelKind::Kb4.default_parameters(image_size) {
        Parameters::Kb4(params) => params,
        Parameters::Ds(_) => unreachable!("default_parameters(kb4) must return kb4"),
    }
}

/// 一路观测 →（物点 Mat，像点 Mat）：`N×1 CV_32FC3` 与 `N×1 CV_32FC2`。
fn to_opencv_views(observation: &Observation, view: usize) -> Result<(Mat, Mat), OpenCvSolveError> {
    if observation.object_points.is_empty() {
        return Err(OpenCvSolveError::EmptyView { view });
    }
    let object_points: Vec<Point3f> = observation
        .object_points
        .iter()
        .map(|point| Point3f::new(point[0] as f32, point[1] as f32, point[2] as f32))
        .collect();
    let image_points: Vec<Point2f> = observation
        .image_points
        .iter()
        .map(|point| Point2f::new(point[0] as f32, point[1] as f32))
        .collect();
    // 形状必须是 **1×N**（单行）：`fisheye::calibrate` 对每视图点位走 `checkVector`，
    // N×1 会在内部 arithm_op 报尺寸不匹配。
    // 另：`Mat::from_slice` 返回借用包装（BoxedRef），这里要 owned Mat：建空 Mat 再写入。
    let mut object_view = Mat::new_rows_cols_with_default(
        1,
        object_points.len() as i32,
        core::CV_32FC3,
        core::Scalar::default(),
    )
    .map_err(|error| cv("object points alloc", error))?;
    {
        let destination = object_view
            .data_typed_mut::<Point3f>()
            .map_err(|error| cv("object points write", error))?;
        destination.copy_from_slice(&object_points);
    }
    let mut image_view = Mat::new_rows_cols_with_default(
        1,
        image_points.len() as i32,
        core::CV_32FC2,
        core::Scalar::default(),
    )
    .map_err(|error| cv("image points alloc", error))?;
    {
        let destination = image_view
            .data_typed_mut::<Point2f>()
            .map_err(|error| cv("image points write", error))?;
        destination.copy_from_slice(&image_points);
    }
    Ok((object_view, image_view))
}

/// KB4 求解（`SolveRequest::kind` 必须是 `Kb4`）。
pub fn solve_kb4(request: SolveRequest<'_>) -> Result<SolveOutcome, OpenCvSolveError> {
    if request.kind != ModelKind::Kb4 {
        return Err(OpenCvSolveError::Unsupported(format!(
            "solve_kb4 only handles kb4, got {}",
            request.kind.as_str()
        )));
    }
    if request.observations.is_empty() {
        return Err(OpenCvSolveError::Unsupported("no observations".to_owned()));
    }
    crate::native_dependency_info().map_err(|message| OpenCvSolveError::OpenCv {
        operation: "native dependency check",
        message,
    })?;

    let mut object_points = Vector::<Mat>::new();
    let mut image_points = Vector::<Mat>::new();
    for (index, observation) in request.observations.iter().enumerate() {
        let (object_view, image_view) = to_opencv_views(observation, index)?;
        object_points.push(object_view);
        image_points.push(image_view);
    }

    let initial = match request.initial {
        Some(Parameters::Kb4(params)) => params,
        _ => default_kb4(request.image_size),
    };
    // 相机矩阵 3×3（CV_64F）与畸变 4×1（CV_64F）
    let mut camera_matrix = Mat::from_slice_2d(&[
        &[initial.fx, 0.0, initial.cx],
        &[0.0, initial.fy, initial.cy],
        &[0.0, 0.0, 1.0],
    ])
    .map_err(|error| cv("camera matrix", error))?;
    let mut distortion =
        Mat::from_slice_2d(&[&[initial.k1], &[initial.k2], &[initial.k3], &[initial.k4]])
            .map_err(|error| cv("distortion", error))?;
    let mut rotation_vectors = Vector::<Mat>::new();
    let mut translation_vectors = Vector::<Mat>::new();
    let criteria = TermCriteria::new(
        core::TermCriteria_COUNT | core::TermCriteria_EPS,
        MAX_ITERATIONS,
        EPSILON,
    )
    .map_err(|error| cv("TermCriteria", error))?;

    fisheye::calibrate(
        &object_points,
        &image_points,
        Size::new(request.image_size.0 as i32, request.image_size.1 as i32),
        &mut camera_matrix,
        &mut distortion,
        &mut rotation_vectors,
        &mut translation_vectors,
        FISHEYE_FLAGS,
        criteria,
    )
    .map_err(|error| cv("fisheye::calibrate", error))?;

    if rotation_vectors.len() != request.observations.len()
        || translation_vectors.len() != request.observations.len()
    {
        return Err(OpenCvSolveError::PoseCount {
            got: rotation_vectors.len(),
            expected: request.observations.len(),
        });
    }

    let mut parameters = initial;
    let camera = camera_matrix
        .data_typed::<f64>()
        .map_err(|error| cv("camera matrix read", error))?;
    let coefficients = distortion
        .data_typed::<f64>()
        .map_err(|error| cv("distortion read", error))?;
    parameters.fx = camera[0];
    parameters.fy = camera[4];
    parameters.cx = camera[2];
    parameters.cy = camera[5];
    parameters.k1 = coefficients[0];
    parameters.k2 = coefficients[1];
    parameters.k3 = coefficients[2];
    parameters.k4 = coefficients[3];

    let mut poses: Vec<IntrinsicPose> = Vec::with_capacity(request.observations.len());
    for index in 0..rotation_vectors.len() {
        let rotation = rotation_vectors
            .get(index)
            .map_err(|error| cv("rvec read", error))?;
        let translation = translation_vectors
            .get(index)
            .map_err(|error| cv("tvec read", error))?;
        let r = rotation
            .data_typed::<f64>()
            .map_err(|error| cv("rvec data", error))?;
        let t = translation
            .data_typed::<f64>()
            .map_err(|error| cv("tvec data", error))?;
        poses.push(([r[0], r[1], r[2]], [t[0], t[1], t[2]]));
    }

    // 用**本仓投影模型**复算残差与无效投影数：采纳闸门看到的是本仓模型下的指标，
    // 而不是 OpenCV 自报的 rms。
    let parameters = Parameters::Kb4(parameters);
    let evaluation =
        rigcal_core::evaluate_solution(ModelKind::Kb4, request.observations, &parameters, &poses);
    let status = if evaluation.rms_px.is_finite() && evaluation.invalid_projection_count == 0 {
        Status::Pass
    } else {
        Status::Fail
    };
    let mut residuals = Vec::new();
    for (observation, pose) in request.observations.iter().zip(poses.iter()) {
        let (pixels, valid) = rigcal_core::estimator::project_with_pose(
            ModelKind::Kb4,
            &parameters,
            &observation.object_points,
            &pose.0,
            &pose.1,
        );
        for (index, pixel) in pixels.iter().enumerate() {
            if !valid[index] {
                continue;
            }
            let observed = observation.image_points[index];
            residuals.push([pixel[0] - observed[0], pixel[1] - observed[1]]);
        }
    }

    Ok(SolveOutcome {
        parameters,
        poses,
        residuals_px: residuals,
        invalid_projection_count: evaluation.invalid_projection_count,
        rms_px: evaluation.rms_px,
        status,
    })
}
