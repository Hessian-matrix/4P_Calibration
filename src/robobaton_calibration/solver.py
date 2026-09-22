"""提供DS/KB4共用的批量最小二乘求解合同。"""

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from .double_sphere import DSParameters, project_points as project_ds, unproject_pixels as unproject_ds
from .kb4 import KB4Parameters, project_points as project_kb4, unproject_pixels as unproject_kb4


@dataclass(frozen=True)
class Observation:
    object_points: np.ndarray
    image_points: np.ndarray
    index: int = 0

    def __post_init__(self) -> None:
        object_points = np.asarray(self.object_points, dtype=np.float64)
        image_points = np.asarray(self.image_points, dtype=np.float64)
        if object_points.ndim != 2 or object_points.shape[1] != 3:
            raise ValueError("object_points must have shape (N, 3)")
        if image_points.ndim != 2 or image_points.shape[1] != 2:
            raise ValueError("image_points must have shape (N, 2)")
        if object_points.shape[0] != image_points.shape[0]:
            raise ValueError("object_points and image_points must contain the same point count")
        object.__setattr__(self, "object_points", object_points)
        object.__setattr__(self, "image_points", image_points)


@dataclass(frozen=True)
class PoseEstimate:
    rvec: np.ndarray
    tvec: np.ndarray
    residuals_px: np.ndarray
    invalid_projection_count: int
    rms_px: float
    status: str


@dataclass(frozen=True)
class CalibrationResult:
    model: str
    parameters: object
    rms_px: float
    status: str
    residuals_px: np.ndarray
    invalid_projection_count: int
    train_observation_count: int
    message: str
    poses: Tuple[Tuple[np.ndarray, np.ndarray], ...]


def transform_points(object_points: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return np.asarray(object_points, dtype=np.float64) @ rotation.T + np.asarray(tvec, dtype=np.float64).reshape(1, 3)


def _default_ds(image_size: Tuple[int, int]) -> DSParameters:
    width, height = image_size
    focal = 0.45 * max(width, height)
    return DSParameters(fx=focal, fy=focal, cx=width / 2.0, cy=height / 2.0, xi=0.4, alpha=0.55)


def _default_kb4(image_size: Tuple[int, int]) -> KB4Parameters:
    width, height = image_size
    focal = 0.45 * max(width, height)
    return KB4Parameters(fx=focal, fy=focal, cx=width / 2.0, cy=height / 2.0)


def _initial_pose(observation: Observation, image_size: Tuple[int, int], focal_hint: float) -> np.ndarray:
    width, height = image_size
    camera_matrix = np.array(
        [[focal_hint, 0.0, width / 2.0], [0.0, focal_hint, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    if observation.object_points.shape[0] >= 4:
        ok, rvec, tvec = cv2.solvePnP(
            observation.object_points,
            observation.image_points,
            camera_matrix,
            np.zeros((4, 1), dtype=np.float64),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if ok and np.isfinite(rvec).all() and np.isfinite(tvec).all() and float(tvec[2]) > 0.0:
            return np.concatenate([rvec.reshape(3), tvec.reshape(3)])
    return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.2], dtype=np.float64)

def _initial_pose_vectors(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    focal_hint: float,
    initial_poses: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]],
) -> List[np.ndarray]:
    if initial_poses is not None:
        if len(initial_poses) != len(observations):
            raise ValueError("initial_poses must match observation count")
        pose_vectors: List[np.ndarray] = []
        for rvec, tvec in initial_poses:
            pose = np.concatenate([np.asarray(rvec, dtype=np.float64).reshape(3), np.asarray(tvec, dtype=np.float64).reshape(3)])
            if not np.isfinite(pose).all() or pose[5] <= 0.0:
                raise ValueError("initial_poses must be finite and in front of the camera")
            pose_vectors.append(pose)
        return pose_vectors
    # 无上游模型姿态时才回退到针孔PnP初值，保持独立DS单元测试和离线调用可用。
    return [_initial_pose(obs, image_size, focal_hint) for obs in observations]


def _split_pose_vector(values: np.ndarray, count: int) -> List[Tuple[np.ndarray, np.ndarray]]:
    poses = []
    for idx in range(count):
        offset = 6 * idx
        pose = values[offset : offset + 6]
        poses.append((pose[:3], pose[3:6]))
    return poses


def _make_residual_function(
    model: str,
    observations: Sequence[Observation],
    optimize_poses: bool,
):
    projector = project_ds if model == "ds" else project_kb4
    parameter_class = DSParameters if model == "ds" else KB4Parameters
    intrinsic_size = 6 if model == "ds" else 8

    def residuals(values: np.ndarray) -> np.ndarray:
        params = parameter_class.from_vector(values[:intrinsic_size])
        poses = _split_pose_vector(values[intrinsic_size:], len(observations)) if optimize_poses else []
        chunks = []
        for obs_idx, observation in enumerate(observations):
            points = observation.object_points
            if optimize_poses:
                rvec, tvec = poses[obs_idx]
                points = transform_points(points, rvec, tvec)
            projected, valid = projector(points, params)
            diff = projected - observation.image_points
            # 无效投影保持残差长度不变并显式惩罚,避免优化器静默丢点。
            diff[~valid] = 1000.0
            chunks.append(diff.reshape(-1))
        return np.concatenate(chunks) if chunks else np.array([], dtype=np.float64)

    return residuals


def _bounds(model: str, image_size: Tuple[int, int], pose_count: int, optimize_poses: bool) -> Tuple[np.ndarray, np.ndarray]:
    width, height = image_size
    if model == "ds":
        low = [50.0, 50.0, -0.25 * width, -0.25 * height, -1.5, 0.05]
        high = [5000.0, 5000.0, 1.25 * width, 1.25 * height, 1.5, 0.95]
    else:
        low = [50.0, 50.0, -0.25 * width, -0.25 * height, -2.0, -2.0, -2.0, -2.0]
        high = [5000.0, 5000.0, 1.25 * width, 1.25 * height, 2.0, 2.0, 2.0, 2.0]
    if optimize_poses:
        low.extend([-np.pi, -np.pi, -np.pi, -5.0, -5.0, 0.05] * pose_count)
        high.extend([np.pi, np.pi, np.pi, 5.0, 5.0, 10.0] * pose_count)
    return np.array(low, dtype=np.float64), np.array(high, dtype=np.float64)


def _evaluate_result(
    model: str,
    params: object,
    observations: Sequence[Observation],
    values: np.ndarray,
    intrinsic_size: int,
    optimize_poses: bool,
) -> Tuple[np.ndarray, int, float]:
    projector = project_ds if model == "ds" else project_kb4
    poses = _split_pose_vector(values[intrinsic_size:], len(observations)) if optimize_poses else []
    residuals = []
    invalid_count = 0
    for obs_idx, observation in enumerate(observations):
        points = observation.object_points
        if optimize_poses:
            rvec, tvec = poses[obs_idx]
            points = transform_points(points, rvec, tvec)
        projected, valid = projector(points, params)
        invalid_count += int((~valid).sum())
        if np.any(valid):
            residuals.append(projected[valid] - observation.image_points[valid])
    if not residuals:
        return np.empty((0, 2), dtype=np.float64), invalid_count, float("inf")
    residual_array = np.vstack(residuals)
    rms = float(np.sqrt(np.mean(np.sum(residual_array * residual_array, axis=1))))
    return residual_array, invalid_count, rms

def _fixed_intrinsics_initial_pose(
    model: str,
    params: object,
    observation: Observation,
    image_size: Tuple[int, int],
) -> np.ndarray:
    """Use model-aware rays for hold-out pose seeding; intrinsic optimization keeps its legacy seed."""
    unprojector = unproject_ds if model == "ds" else unproject_kb4
    rays, valid = unprojector(observation.image_points, params)
    usable = valid & np.isfinite(rays).all(axis=1) & (rays[:, 2] > 1e-4)
    if int(np.count_nonzero(usable)) >= 4:
        normalized = rays[usable, :2] / rays[usable, 2, None]
        object_points = observation.object_points[usable]
        try:
            ok, rvec, tvec = cv2.solvePnP(
                object_points,
                normalized,
                np.eye(3, dtype=np.float64),
                np.zeros((4, 1), dtype=np.float64),
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
        except cv2.error:
            ok = False
        if ok and np.isfinite(rvec).all() and np.isfinite(tvec).all() and float(tvec[2]) > 0.0:
            return np.concatenate([rvec.reshape(3), tvec.reshape(3)])
    focal_hint = float((params.fx + params.fy) * 0.5)
    return _initial_pose(observation, image_size, focal_hint)


def estimate_fixed_intrinsics_pose(
    model: str,
    params: object,
    observation: Observation,
    image_size: Tuple[int, int],
) -> PoseEstimate:
    projector = project_ds if model == "ds" else project_kb4
    initial = _fixed_intrinsics_initial_pose(model, params, observation, image_size)
    lower = np.array([-np.pi, -np.pi, -np.pi, -5.0, -5.0, 0.05], dtype=np.float64)
    upper = np.array([np.pi, np.pi, np.pi, 5.0, 5.0, 10.0], dtype=np.float64)

    def residuals(values: np.ndarray) -> np.ndarray:
        points = transform_points(observation.object_points, values[:3], values[3:6])
        projected, valid = projector(points, params)
        diff = projected - observation.image_points
        # hold-out不允许删点；无效投影按固定大残差参与失败判定。
        diff[~valid] = 1000.0
        return diff.reshape(-1)

    result = least_squares(
        residuals,
        np.minimum(np.maximum(initial, lower + 1e-9), upper - 1e-9),
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=2.0,
        max_nfev=200,
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
    )
    points = transform_points(observation.object_points, result.x[:3], result.x[3:6])
    projected, valid = projector(points, params)
    invalid_count = int((~valid).sum())
    if np.any(valid):
        residual_array = projected[valid] - observation.image_points[valid]
        rms = float(np.sqrt(np.mean(np.sum(residual_array * residual_array, axis=1))))
    else:
        residual_array = np.empty((0, 2), dtype=np.float64)
        rms = float("inf")
    status = "PASS" if result.success and np.isfinite(rms) and invalid_count == 0 else "FAIL"
    return PoseEstimate(
        rvec=result.x[:3].copy(),
        tvec=result.x[3:6].copy(),
        residuals_px=residual_array,
        invalid_projection_count=invalid_count,
        rms_px=rms,
        status=status,
    )


def evaluate_fixed_intrinsics_holdout(
    model: str,
    params: object,
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
) -> Tuple[np.ndarray, int, float]:
    if not observations:
        return np.empty((0, 2), dtype=np.float64), 0, float("inf")
    residual_chunks = []
    invalid_count = 0
    for observation in observations:
        pose = estimate_fixed_intrinsics_pose(model, params, observation, image_size)
        invalid_count += pose.invalid_projection_count
        if pose.status != "PASS":
            invalid_count += int(observation.object_points.shape[0])
        if pose.residuals_px.size:
            residual_chunks.append(pose.residuals_px)
    if not residual_chunks:
        return np.empty((0, 2), dtype=np.float64), invalid_count, float("inf")
    residual_array = np.vstack(residual_chunks)
    rms = float(np.sqrt(np.mean(np.sum(residual_array * residual_array, axis=1))))
    return residual_array, invalid_count, rms


def _solve(
    model: str,
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    initial: Optional[object],
    optimize_poses: bool,
    initial_poses: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
) -> CalibrationResult:
    if not observations:
        raise ValueError("at least one observation is required")
    initial_params = initial or (_default_ds(image_size) if model == "ds" else _default_kb4(image_size))
    intrinsic_vector = initial_params.as_vector()
    focal_hint = float((intrinsic_vector[0] + intrinsic_vector[1]) * 0.5)
    pose_vectors = []
    if optimize_poses:
        # DS可复用KB4已收敛姿态作为初值，避免宽角/平面板场景落入PnP局部极小值。
        pose_vectors = _initial_pose_vectors(observations, image_size, focal_hint, initial_poses)
    x0 = np.concatenate([intrinsic_vector, *pose_vectors]) if pose_vectors else intrinsic_vector
    lower, upper = _bounds(model, image_size, len(observations), optimize_poses)
    x0 = np.minimum(np.maximum(x0, lower + 1e-9), upper - 1e-9)
    residual_function = _make_residual_function(model, observations, optimize_poses)
    result = least_squares(
        residual_function,
        x0,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=2.0,
        max_nfev=600,
        xtol=1e-10,
        ftol=1e-10,
        gtol=1e-10,
    )
    intrinsic_size = 6 if model == "ds" else 8
    parameter_class = DSParameters if model == "ds" else KB4Parameters
    params = parameter_class.from_vector(result.x[:intrinsic_size])
    residuals, invalid_count, rms = _evaluate_result(model, params, observations, result.x, intrinsic_size, optimize_poses)
    status = "PASS" if result.success and np.isfinite(rms) and invalid_count == 0 else "FAIL"
    poses = tuple(_split_pose_vector(result.x[intrinsic_size:], len(observations))) if optimize_poses else tuple()
    return CalibrationResult(
        model=model,
        parameters=params,
        rms_px=rms,
        status=status,
        residuals_px=residuals,
        invalid_projection_count=invalid_count,
        train_observation_count=len(observations),
        message=str(result.message),
        poses=poses,
    )

def _solve_kb4_opencv(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    initial: Optional[KB4Parameters],
) -> CalibrationResult:
    initial_params = initial or _default_kb4(image_size)
    object_points = [obs.object_points.reshape(-1, 1, 3).astype(np.float64) for obs in observations]
    image_points = [obs.image_points.reshape(-1, 1, 2).astype(np.float64) for obs in observations]
    if any(points.shape[0] < 4 for points in object_points):
        return CalibrationResult(
            model="kb4",
            parameters=initial_params,
            rms_px=float("inf"),
            status="FAIL",
            residuals_px=np.empty((0, 2), dtype=np.float64),
            invalid_projection_count=0,
            train_observation_count=len(observations),
            message="each KB4 observation needs at least four points",
            poses=tuple(),
        )
    camera_matrix = np.array(
        [[initial_params.fx, 0.0, initial_params.cx], [0.0, initial_params.fy, initial_params.cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.array([[initial_params.k1], [initial_params.k2], [initial_params.k3], [initial_params.k4]], dtype=np.float64)
    flags = cv2.fisheye.CALIB_USE_INTRINSIC_GUESS | cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
    try:
        # KB4/equidistant正式结果使用OpenCV fisheye合同,再用统一投影函数复算残差。
        _opencv_rms, camera_matrix, distortion, rvecs, tvecs = cv2.fisheye.calibrate(
            object_points,
            image_points,
            image_size,
            camera_matrix,
            distortion,
            flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-10),
        )
    except cv2.error as exc:
        return CalibrationResult(
            model="kb4",
            parameters=initial_params,
            rms_px=float("inf"),
            status="FAIL",
            residuals_px=np.empty((0, 2), dtype=np.float64),
            invalid_projection_count=0,
            train_observation_count=len(observations),
            message=str(exc),
            poses=tuple(),
        )
    params = KB4Parameters(
        fx=float(camera_matrix[0, 0]),
        fy=float(camera_matrix[1, 1]),
        cx=float(camera_matrix[0, 2]),
        cy=float(camera_matrix[1, 2]),
        k1=float(distortion[0, 0]),
        k2=float(distortion[1, 0]),
        k3=float(distortion[2, 0]),
        k4=float(distortion[3, 0]),
    )
    poses = tuple((np.asarray(rvec).reshape(3), np.asarray(tvec).reshape(3)) for rvec, tvec in zip(rvecs, tvecs))
    values = np.concatenate([params.as_vector(), *[np.concatenate([rvec, tvec]) for rvec, tvec in poses]])
    residuals, invalid_count, rms = _evaluate_result("kb4", params, observations, values, 8, True)
    status = "PASS" if np.isfinite(rms) and invalid_count == 0 else "FAIL"
    return CalibrationResult(
        model="kb4",
        parameters=params,
        rms_px=rms,
        status=status,
        residuals_px=residuals,
        invalid_projection_count=invalid_count,
        train_observation_count=len(observations),
        message="OpenCV fisheye.calibrate",
        poses=poses,
    )


def solve_double_sphere(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    initial: Optional[DSParameters] = None,
    optimize_poses: bool = True,
    initial_poses: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
) -> CalibrationResult:
    return _solve("ds", observations, image_size, initial, optimize_poses, initial_poses=initial_poses)

def solve_kb4(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    initial: Optional[KB4Parameters] = None,
    optimize_poses: bool = True,
) -> CalibrationResult:
    if optimize_poses:
        return _solve_kb4_opencv(observations, image_size, initial)
    return _solve("kb4", observations, image_size, initial, optimize_poses)


def residual_metrics(residuals_px: np.ndarray) -> Tuple[float, float]:
    if residuals_px.size == 0:
        return float("inf"), float("inf")
    norms = np.linalg.norm(np.asarray(residuals_px, dtype=np.float64), axis=1)
    return float(np.median(norms)), float(np.percentile(norms, 95.0))
