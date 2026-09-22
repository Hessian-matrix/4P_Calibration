"""实现Double Sphere投影/反投影的最小数学核心。"""

from dataclasses import dataclass
from typing import Iterable, Tuple

import numpy as np


@dataclass(frozen=True)
class DSParameters:
    fx: float
    fy: float
    cx: float
    cy: float
    xi: float
    alpha: float

    def as_vector(self) -> np.ndarray:
        return np.array([self.fx, self.fy, self.cx, self.cy, self.xi, self.alpha], dtype=np.float64)

    @classmethod
    def from_vector(cls, values: Iterable[float]) -> "DSParameters":
        fx, fy, cx, cy, xi, alpha = [float(v) for v in values]
        return cls(fx=fx, fy=fy, cx=cx, cy=cy, xi=xi, alpha=alpha)


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


def project_points(points: np.ndarray, params: DSParameters) -> Tuple[np.ndarray, np.ndarray]:
    points = _as_points(points)
    pixels = np.full((points.shape[0], 2), np.nan, dtype=np.float64)
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    d1 = np.linalg.norm(points, axis=1)
    shifted_z = params.xi * d1 + z
    d2 = np.sqrt(x * x + y * y + shifted_z * shifted_z)
    denominator = params.alpha * d2 + (1.0 - params.alpha) * shifted_z
    # Double Sphere有效域由模型分母决定，宽角镜头可接受部分z<=0射线。
    valid = (
        np.isfinite(denominator)
        & np.isfinite(d1)
        & (d1 > 0.0)
        & (denominator > 1e-12)
        & (params.fx > 0.0)
        & (params.fy > 0.0)
        & (0.0 < params.alpha < 1.0)
    )
    pixels[valid, 0] = params.fx * x[valid] / denominator[valid] + params.cx
    pixels[valid, 1] = params.fy * y[valid] / denominator[valid] + params.cy
    return pixels, valid


def unproject_pixels(pixels: np.ndarray, params: DSParameters) -> Tuple[np.ndarray, np.ndarray]:
    pixels = _as_pixels(pixels)
    mx = (pixels[:, 0] - params.cx) / params.fx
    my = (pixels[:, 1] - params.cy) / params.fy
    r2 = mx * mx + my * my
    inside = 1.0 - (2.0 * params.alpha - 1.0) * r2
    sqrt_argument = np.full_like(r2, np.nan, dtype=np.float64)
    valid = (
        np.isfinite(r2)
        & (inside >= 0.0)
        & (params.fx > 0.0)
        & (params.fy > 0.0)
        & (0.0 < params.alpha < 1.0)
    )
    rays = np.full((pixels.shape[0], 3), np.nan, dtype=np.float64)
    if not np.any(valid):
        return rays, valid

    # 反投影公式按Double Sphere闭式逆模型计算,保持与投影模型同一坐标合同。
    sqrt_inside = np.sqrt(np.maximum(inside[valid], 0.0))
    mz = (1.0 - params.alpha * params.alpha * r2[valid]) / (
        params.alpha * sqrt_inside + 1.0 - params.alpha
    )
    denominator = mz * mz + r2[valid]
    valid_indices = np.flatnonzero(valid)
    sqrt_argument[valid_indices] = mz * mz + (1.0 - params.xi * params.xi) * r2[valid_indices]
    sqrt_good = sqrt_argument[valid_indices] >= 0.0
    valid[valid_indices[~sqrt_good]] = False
    active_indices = valid_indices[sqrt_good]
    if active_indices.size == 0:
        return rays, valid
    mz = mz[sqrt_good]
    denominator = denominator[sqrt_good]
    sqrt_term = np.sqrt(sqrt_argument[active_indices])
    scale = (mz * params.xi + sqrt_term) / denominator
    unnormalized = np.column_stack((scale * mx[active_indices], scale * my[active_indices], scale * mz - params.xi))
    norms = np.linalg.norm(unnormalized, axis=1)
    good = norms > 1e-12
    rays[active_indices[good]] = unnormalized[good] / norms[good, None]
    valid[active_indices[~good]] = False
    return rays, valid
