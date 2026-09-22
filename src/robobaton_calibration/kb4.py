"""实现OpenCV fisheye/KB4等距模型的数学核心。"""

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


@dataclass(frozen=True)
class KB4Parameters:
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    k3: float = 0.0
    k4: float = 0.0

    def as_vector(self) -> np.ndarray:
        return np.array([self.fx, self.fy, self.cx, self.cy, self.k1, self.k2, self.k3, self.k4], dtype=np.float64)

    @classmethod
    def from_vector(cls, values: Iterable[float]) -> "KB4Parameters":
        fx, fy, cx, cy, k1, k2, k3, k4 = [float(v) for v in values]
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, k1=k1, k2=k2, k3=k3, k4=k4)


def _as_points(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    return points


def _as_pixels(pixels: np.ndarray) -> np.ndarray:
    pixels = np.asarray(pixels, dtype=np.float64)
    if pixels.ndim != 2 or pixels.shape[1] != 2:
        raise ValueError("pixels must have shape (N, 2)")
    return pixels


def _theta_distorted(theta: np.ndarray, params: KB4Parameters) -> np.ndarray:
    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4
    return theta * (1.0 + params.k1 * theta2 + params.k2 * theta4 + params.k3 * theta6 + params.k4 * theta8)


def project_points(points: np.ndarray, params: KB4Parameters) -> Tuple[np.ndarray, np.ndarray]:
    points = _as_points(points)
    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    valid = (
        np.isfinite(points).all(axis=1)
        & (np.linalg.norm(points, axis=1) > 1e-12)
        & (params.fx > 0.0)
        & (params.fy > 0.0)
    )
    if not np.any(valid):
        return pixels, valid

    xy_radius = np.sqrt(x[valid] * x[valid] + y[valid] * y[valid])
    theta = np.arctan2(xy_radius, z[valid])
    theta_limit = _first_monotonic_theta_limit(params)
    # forward 投影也必须限制在第一单调可逆分支内，保证 project/unproject 合同一致。
    branch_valid = theta <= theta_limit + 1e-12
    theta_d = _theta_distorted(theta, params)
    # KB4等距模型按xy单位方向缩放theta_d，避免z<0时atan(xy/z)落到错误分支。
    dir_x = np.zeros_like(xy_radius)
    dir_y = np.zeros_like(xy_radius)
    nonzero = xy_radius > 1e-12
    dir_x[nonzero] = x[valid][nonzero] / xy_radius[nonzero]
    dir_y[nonzero] = y[valid][nonzero] / xy_radius[nonzero]
    pixels[valid, 0] = params.fx * theta_d * dir_x + params.cx
    pixels[valid, 1] = params.fy * theta_d * dir_y + params.cy
    valid_indices = np.flatnonzero(valid)
    valid[valid_indices] &= branch_valid
    valid[valid] &= np.isfinite(pixels[valid]).all(axis=1)
    pixels[~valid] = np.nan
    return pixels, valid


def unproject_pixels(pixels: np.ndarray, params: KB4Parameters) -> Tuple[np.ndarray, np.ndarray]:
    pixels = _as_pixels(pixels)
    mx = (pixels[:, 0] - params.cx) / params.fx
    my = (pixels[:, 1] - params.cy) / params.fy
    theta_d = np.sqrt(mx * mx + my * my)
    theta_limit = _first_monotonic_theta_limit(params)
    max_radius = float(_theta_distorted(np.array([theta_limit], dtype=np.float64), params)[0])
    valid = np.isfinite(theta_d) & (theta_d <= max_radius + 1e-12) & (params.fx > 0.0) & (params.fy > 0.0)
    theta = np.minimum(theta_d.copy(), theta_limit)
    # 反解只在第一单调分支内做有界Newton，分支外像素fail closed。
    for _ in range(16):
        theta2 = theta * theta
        theta4 = theta2 * theta2
        theta6 = theta4 * theta2
        theta8 = theta4 * theta4
        value = theta * (
            1.0 + params.k1 * theta2 + params.k2 * theta4 + params.k3 * theta6 + params.k4 * theta8
        ) - theta_d
        derivative = (
            1.0
            + 3.0 * params.k1 * theta2
            + 5.0 * params.k2 * theta4
            + 7.0 * params.k3 * theta6
            + 9.0 * params.k4 * theta8
        )
        step_ok = valid & (np.abs(derivative) > 1e-12)
        theta[step_ok] = np.clip(theta[step_ok] - value[step_ok] / derivative[step_ok], 0.0, theta_limit)
        valid &= step_ok | (theta_d <= 1e-12)

    rays = np.full((pixels.shape[0], 3), np.nan, dtype=np.float64)
    dir_x = np.zeros_like(theta_d)
    dir_y = np.zeros_like(theta_d)
    nonzero = theta_d > 1e-12
    dir_x[nonzero] = mx[nonzero] / theta_d[nonzero]
    dir_y[nonzero] = my[nonzero] / theta_d[nonzero]
    # KB4逆模型返回单位球面射线，z分量必须是cos(theta)而不是固定1。
    unnormalized = np.column_stack((np.sin(theta) * dir_x, np.sin(theta) * dir_y, np.cos(theta)))
    norms = np.linalg.norm(unnormalized, axis=1)
    residual = np.abs(_theta_distorted(theta, params) - theta_d)
    valid &= np.isfinite(unnormalized).all(axis=1) & (norms > 1e-12) & (residual <= 1e-10)
    rays[valid] = unnormalized[valid] / norms[valid, None]
    return rays, valid


def _first_monotonic_theta_limit(params: KB4Parameters) -> float:
    theta = np.linspace(0.0, np.pi, 2049, dtype=np.float64)
    theta2 = theta * theta
    derivative = (
        1.0
        + 3.0 * params.k1 * theta2
        + 5.0 * params.k2 * theta2 * theta2
        + 7.0 * params.k3 * theta2 * theta2 * theta2
        + 9.0 * params.k4 * theta2 * theta2 * theta2 * theta2
    )
    nonpositive = np.flatnonzero(derivative <= 1e-9)
    if nonpositive.size == 0:
        return float(np.pi)
    return float(theta[max(1, int(nonpositive[0]) - 1)])
