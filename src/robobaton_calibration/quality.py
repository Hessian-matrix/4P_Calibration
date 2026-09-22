"""在线筛帧使用统一质量和覆盖合同。"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .solver import Observation


@dataclass(frozen=True)
class QualityThresholds:
    min_laplacian_var: float = 15.0
    min_contrast: float = 20.0
    max_saturated_fraction: float = 0.35


@dataclass(frozen=True)
class FrameQuality:
    accepted: bool
    reasons: List[str]
    laplacian_var: float
    contrast: float
    saturated_fraction: float


@dataclass(frozen=True)
class CoverageDecision:
    accepted: bool
    reasons: List[str]
    new_bins: int
    covered_bins: int
    total_bins: int
    classification: str = "SOLVE_ELIGIBLE"
    solve_eligible: bool = True


@dataclass(frozen=True)
class CaptureAssistantConfig:
    grid_cols: int = 10
    grid_rows: int = 8
    min_observations: int = 12
    min_grid_frames: int = 1
    min_edge_frames: int = 5
    min_corner_frames: int = 3
    outer_radius_start: float = 0.75
    min_radial_bucket_frames: int = 1
    min_outer_quadrant_frames: int = 1
    min_scale_bucket_frames: int = 2
    min_pose_bucket_frames: int = 2
    min_center_distance_px: float = 35.0
    min_scale_change_ratio: float = 0.08
    min_aspect_change_ratio: float = 0.10
    scale_small_max: float = 0.22
    scale_large_min: float = 0.38
    roll_strong_ratio: float = 0.10
    tilt_strong_ratio: float = 0.18
    oblique_ratio: float = 0.22
    min_coverage_tags: int = 6
    min_coverage_tag_rows: int = 2
    min_coverage_tag_cols: int = 2
    min_solve_tags: int = 18
    min_solve_tag_rows: int = 4
    min_solve_tag_cols: int = 4
    min_holdout_tags: int = 28
    min_holdout_tag_rows: int = 6
    min_holdout_tag_cols: int = 6
    min_holdout_hull_area_ratio: float = 0.28
    min_holdout_margin_px: float = 24.0

    def __post_init__(self) -> None:
        numeric_counts = (
            self.grid_cols,
            self.grid_rows,
            self.min_observations,
            self.min_grid_frames,
            self.min_edge_frames,
            self.min_corner_frames,
            self.min_radial_bucket_frames,
            self.min_outer_quadrant_frames,
            self.min_scale_bucket_frames,
            self.min_pose_bucket_frames,
            self.min_coverage_tags,
            self.min_coverage_tag_rows,
            self.min_coverage_tag_cols,
            self.min_solve_tags,
            self.min_solve_tag_rows,
            self.min_solve_tag_cols,
            self.min_holdout_tags,
            self.min_holdout_tag_rows,
            self.min_holdout_tag_cols,
        )
        if any(int(value) < 0 for value in numeric_counts) or self.grid_cols <= 0 or self.grid_rows <= 0:
            raise ValueError("capture assistant counts must be non-negative and grid dimensions positive")
        if not np.isfinite(float(self.outer_radius_start)) or not 0.0 < self.outer_radius_start < 1.0:
            raise ValueError("outer_radius_start must be finite and in (0, 1)")
        if not np.isfinite(float(self.scale_small_max)) or not np.isfinite(float(self.scale_large_min)) or not 0.0 < self.scale_small_max < self.scale_large_min <= 1.0:
            raise ValueError("scale thresholds must be finite, ordered, and within (0, 1]")
        proxy_thresholds = (
            self.min_center_distance_px,
            self.min_scale_change_ratio,
            self.min_aspect_change_ratio,
            self.roll_strong_ratio,
            self.tilt_strong_ratio,
            self.oblique_ratio,
        )
        if any(not np.isfinite(float(value)) or float(value) < 0.0 for value in proxy_thresholds):
            raise ValueError("capture assistant proxy thresholds must be finite and non-negative")
        geometry_thresholds = (self.min_holdout_hull_area_ratio, self.min_holdout_margin_px)
        if any(not np.isfinite(float(value)) or float(value) < 0.0 for value in geometry_thresholds):
            raise ValueError("holdout geometry thresholds must be finite and non-negative")


@dataclass(frozen=True)
class CaptureSnapshot:
    accepted_count: int
    ready: bool
    missing: List[str]
    next: List[str]
    grid_frame_counts: np.ndarray
    zones: Dict[str, int]
    radial: Dict[str, int]
    scale: Dict[str, int]
    solve_eligible_count: int
    coverage_only_count: int
    pose: Dict[str, int]


@dataclass(frozen=True)
class _PoseSignature:
    center: np.ndarray
    scale: float
    aspect: float
    roll_slope: float = 0.0
    tilt_x: float = 0.0
    tilt_y: float = 0.0
    pose_valid: bool = False


def coverage_cell_indices(
    points: np.ndarray,
    image_size: Tuple[int, int],
    grid_cols: int,
    grid_rows: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """在线助手和离线审计必须共用同一像素到网格映射。"""
    width, height = image_size
    array = np.asarray(points, dtype=np.float64)
    cols = np.clip((array[:, 0] / width * grid_cols).astype(np.int64), 0, grid_cols - 1)
    rows = np.clip((array[:, 1] / height * grid_rows).astype(np.int64), 0, grid_rows - 1)
    return rows, cols


def coverage_zone_masks(points: np.ndarray, image_size: Tuple[int, int]) -> Dict[str, np.ndarray]:
    """固定20%边角定义只能有一份，避免在线PASS后离线FAIL。"""
    width, height = image_size
    array = np.asarray(points, dtype=np.float64)
    x, y = array[:, 0], array[:, 1]
    left, right = x < 0.2 * width, x >= 0.8 * width
    top, bottom = y < 0.2 * height, y >= 0.8 * height
    return {
        "left_20": left,
        "right_20": right,
        "top_20": top,
        "bottom_20": bottom,
        "top_left_20": top & left,
        "top_right_20": top & right,
        "bottom_left_20": bottom & left,
        "bottom_right_20": bottom & right,
    }


def _normalized_radius(points: np.ndarray, image_size: Tuple[int, int]) -> np.ndarray:
    width, height = image_size
    center = np.array([width * 0.5, height * 0.5], dtype=np.float64)
    return np.linalg.norm((np.asarray(points, dtype=np.float64) - center) / center, axis=1)


def _convex_hull_area_ratio(points: np.ndarray, image_size: Tuple[int, int]) -> float:
    width, height = image_size
    hull = cv2.convexHull(np.asarray(points, dtype=np.float32))
    return float(abs(cv2.contourArea(hull)) / max(float(width * height), 1.0))


def _target_pose_proxy(observation: Observation, points: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """返回不依赖内参的 roll/tilt 代理；退化目标点返回 None，不能伪造姿态覆盖。"""
    object_points = np.asarray(observation.object_points, dtype=np.float64)
    if object_points.ndim != 2 or object_points.shape[0] != points.shape[0] or object_points.shape[0] < 4 or object_points.shape[1] < 2:
        return None
    object_xy = object_points[:, :2]
    if not np.isfinite(object_xy).all() or np.linalg.matrix_rank(object_xy - object_xy.mean(axis=0)) < 2:
        return None
    homography, _mask = cv2.findHomography(object_xy, points, 0)
    if homography is None or not np.isfinite(homography).all():
        return None
    minimum = object_xy.min(axis=0)
    maximum = object_xy.max(axis=0)
    if np.any(maximum - minimum <= 1e-9):
        return None
    board_box = np.array(
        [[minimum[0], minimum[1]], [maximum[0], minimum[1]], [maximum[0], maximum[1]], [minimum[0], maximum[1]]],
        dtype=np.float64,
    ).reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(board_box, homography).reshape(-1, 2)
    if not np.isfinite(projected).all():
        return None
    top_vector = projected[1] - projected[0]
    top = float(np.linalg.norm(top_vector))
    bottom = float(np.linalg.norm(projected[2] - projected[3]))
    left = float(np.linalg.norm(projected[3] - projected[0]))
    right = float(np.linalg.norm(projected[2] - projected[1]))
    if min(top, bottom, left, right) <= 1e-6:
        return None
    denominator = top_vector[0] if abs(float(top_vector[0])) > 1e-6 else np.copysign(1e-6, float(top_vector[0]) or 1.0)
    roll_slope = float(top_vector[1] / denominator)
    tilt_x = abs(top - bottom) / max(top, bottom)
    tilt_y = abs(left - right) / max(left, right)
    return roll_slope, float(tilt_x), float(tilt_y)


def _tag_grid_stats(observation: Observation) -> Tuple[int, int, int]:
    """求解帧必须含足够AprilGrid tag数量和板面跨度，低信息量帧只用于覆盖引导。"""
    object_points = np.asarray(observation.object_points, dtype=np.float64)
    if object_points.ndim != 2 or object_points.shape[0] < 4 or object_points.shape[1] < 2:
        return 0, 0, 0
    tag_count = int(object_points.shape[0] // 4)
    if tag_count <= 0:
        return 0, 0, 0
    tag_corners = object_points[: tag_count * 4, :2].reshape(tag_count, 4, 2)
    centers = tag_corners.mean(axis=1)
    finite = np.isfinite(centers).all(axis=1)
    if not np.any(finite):
        return 0, 0, 0
    centers = centers[finite]
    # AprilGrid几何来自浮点配置，按亚纳米级舍入聚类同一行/列以抵抗文本解析噪声。
    col_span = len({round(float(value), 9) for value in centers[:, 0]})
    row_span = len({round(float(value), 9) for value in centers[:, 1]})
    return int(centers.shape[0]), row_span, col_span


class CaptureAssistant:
    def __init__(self, image_size: Tuple[int, int], config: CaptureAssistantConfig = CaptureAssistantConfig()) -> None:
        if len(image_size) != 2 or int(image_size[0]) <= 0 or int(image_size[1]) <= 0:
            raise ValueError("capture assistant image_size must be positive")
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.config = config
        self._observations: Dict[int, Observation] = {}
        self._pose_signatures: List[_PoseSignature] = []
        self._signatures_by_index: Dict[int, _PoseSignature] = {}
        self._solve_eligible_indices: set[int] = set()
        self._coverage_only_indices: set[int] = set()

    @property
    def accepted_count(self) -> int:
        return len(self._observations)

    @property
    def solve_eligible_count(self) -> int:
        return len(self._solve_eligible_indices)

    @property
    def coverage_only_count(self) -> int:
        return len(self._coverage_only_indices)

    def reset(self) -> None:
        self._observations.clear()
        self._pose_signatures.clear()
        self._signatures_by_index.clear()
        self._solve_eligible_indices.clear()
        self._coverage_only_indices.clear()

    def consider_and_accept(self, observation: Observation) -> CoverageDecision:
        signature = self._pose_signature(observation)
        coverage_gate_reasons = self._coverage_gate_reasons(observation)
        solve_gate_reasons = self._solve_gate_reasons(observation)
        reasons: List[str] = []
        if coverage_gate_reasons:
            reasons.extend(coverage_gate_reasons)
            return CoverageDecision(False, reasons, 0, int(np.count_nonzero(self._grid_counts())), self.config.grid_cols * self.config.grid_rows, "REJECTED", False)
        # 低信息量coverage_only帧不能阻止同姿态高质量帧进入求解，只约束后续coverage_only重复采集。
        if self._is_duplicate_pose(signature, solve_eligible_only=not solve_gate_reasons):
            reasons.append("DUPLICATE_POSE")
        reasons.extend(solve_gate_reasons)
        before = int(np.count_nonzero(self._grid_counts()))
        accepted = "DUPLICATE_POSE" not in reasons
        solve_eligible = accepted and not solve_gate_reasons
        classification = "SOLVE_ELIGIBLE" if solve_eligible else "COVERAGE_ONLY" if accepted else "REJECTED"
        if accepted:
            self._store(observation, signature, solve_eligible=solve_eligible)
        after = int(np.count_nonzero(self._grid_counts()))
        return CoverageDecision(accepted, reasons, max(0, after - before), after, self.config.grid_cols * self.config.grid_rows, classification, solve_eligible)

    def accept(self, observation: Observation, solve_eligible: bool = True) -> None:
        if observation.index in self._observations:
            return
        self._store(observation, self._pose_signature(observation), solve_eligible=solve_eligible)

    def solve_eligible_observations(self) -> List[Observation]:
        return [observation for index, observation in self._observations.items() if index in self._solve_eligible_indices]

    def _coverage_gate_reasons(self, observation: Observation) -> List[str]:
        tag_count, row_span, col_span = _tag_grid_stats(observation)
        if (
            tag_count < self.config.min_coverage_tags
            or row_span < self.config.min_coverage_tag_rows
            or col_span < self.config.min_coverage_tag_cols
        ):
            return ["TOO_FEW_SPREAD_TAGS"]
        return []

    def _solve_gate_reasons(self, observation: Observation) -> List[str]:
        tag_count, row_span, col_span = _tag_grid_stats(observation)
        reasons: List[str] = []
        if tag_count < self.config.min_solve_tags:
            reasons.append("LOW_TAG_COUNT")
        if row_span < self.config.min_solve_tag_rows:
            reasons.append("LOW_BOARD_ROW_SPAN")
        if col_span < self.config.min_solve_tag_cols:
            reasons.append("LOW_BOARD_COL_SPAN")
        return reasons

    def holdout_gate_reasons(self, observation: Observation) -> List[str]:
        tag_count, row_span, col_span = _tag_grid_stats(observation)
        reasons: List[str] = []
        if tag_count < self.config.min_holdout_tags:
            reasons.append("LOW_HOLDOUT_TAG_COUNT")
        if row_span < self.config.min_holdout_tag_rows:
            reasons.append("LOW_HOLDOUT_ROW_SPAN")
        if col_span < self.config.min_holdout_tag_cols:
            reasons.append("LOW_HOLDOUT_COL_SPAN")
        points = np.asarray(observation.image_points, dtype=np.float64)
        if points.shape[0] >= 3 and np.isfinite(points).all():
            hull = cv2.convexHull(points.astype(np.float32))
            hull_area_ratio = float(cv2.contourArea(hull)) / float(self.image_size[0] * self.image_size[1])
            x, y, width, height = cv2.boundingRect(hull)
            margin_px = min(float(x), float(y), float(self.image_size[0] - (x + width)), float(self.image_size[1] - (y + height)))
            if hull_area_ratio < self.config.min_holdout_hull_area_ratio:
                reasons.append("LOW_HOLDOUT_HULL_AREA")
            if margin_px < self.config.min_holdout_margin_px:
                reasons.append("LOW_HOLDOUT_BOUNDARY_MARGIN")
        else:
            reasons.append("LOW_HOLDOUT_GEOMETRY")
        return reasons

    def _store(self, observation: Observation, signature: _PoseSignature, solve_eligible: bool) -> None:
        if observation.index in self._observations:
            return
        self._observations[observation.index] = observation
        self._pose_signatures.append(signature)
        self._signatures_by_index[observation.index] = signature
        if solve_eligible:
            self._solve_eligible_indices.add(observation.index)
        else:
            self._coverage_only_indices.add(observation.index)

    def readiness(self) -> CaptureSnapshot:
        return self.snapshot()

    def snapshot(self) -> CaptureSnapshot:
        grid = self._grid_counts()
        zones = self._zone_counts()
        radial = self._radial_counts()
        scale = self._scale_counts()
        pose = self._pose_counts()
        missing: List[str] = []
        if self.solve_eligible_count < self.config.min_observations:
            missing.append("SOLVE_ELIGIBLE_COUNT")
        for col in range(self.config.grid_cols):
            if int(grid[:, col].max(initial=0)) < self.config.min_grid_frames:
                missing.append(f"GRID_COL_{col}")
        for row in range(self.config.grid_rows):
            for col in range(self.config.grid_cols):
                if int(grid[row, col]) < self.config.min_grid_frames:
                    missing.append(f"GRID_R{row + 1}C{col + 1}")
        for name, count in zones.items():
            if not (name.startswith("CORNER_") or name.startswith("EDGE_")):
                continue
            threshold = self.config.min_corner_frames if name.startswith("CORNER_") else self.config.min_edge_frames
            if count < threshold:
                missing.append(name)
        for name, count in radial.items():
            threshold = self.config.min_outer_quadrant_frames if name.startswith("OUTER_") else self.config.min_radial_bucket_frames
            if count < threshold:
                missing.append(f"RADIAL_{name}")
        for name, count in scale.items():
            if count < self.config.min_scale_bucket_frames:
                missing.append(f"SCALE_{name}")
        for name, count in pose.items():
            if count < self.config.min_pose_bucket_frames:
                missing.append(f"POSE_{name}")
        next_tokens = [f"NEXT:{token}" for token in missing[:6]] or ["NEXT:READY"]
        return CaptureSnapshot(
            accepted_count=self.accepted_count,
            ready=not missing,
            missing=missing,
            next=next_tokens,
            grid_frame_counts=grid,
            zones=zones,
            radial=radial,
            scale=scale,
            pose=pose,
            solve_eligible_count=self.solve_eligible_count,
            coverage_only_count=self.coverage_only_count,
        )

    def _pose_signature(self, observation: Observation) -> _PoseSignature:
        points = np.asarray(observation.image_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] == 0:
            raise ValueError("observation image_points must have shape (N, 2) and be non-empty")
        if not np.isfinite(points).all():
            raise ValueError("observation image_points must be finite")
        width, height = self.image_size
        if np.any(points[:, 0] < 0.0) or np.any(points[:, 0] >= width) or np.any(points[:, 1] < 0.0) or np.any(points[:, 1] >= height):
            raise ValueError("observation image_points must lie inside image_size")
        center = points.mean(axis=0)
        rect_size = cv2.minAreaRect(points.astype(np.float32))[1] if points.shape[0] >= 2 else (1.0, 1.0)
        short_side = max(min(float(rect_size[0]), float(rect_size[1])), 1.0)
        long_side = max(float(rect_size[0]), float(rect_size[1]), 1.0)
        aspect = long_side / short_side
        scale = _convex_hull_area_ratio(points, self.image_size)
        proxy = _target_pose_proxy(observation, points)
        if proxy is None:
            return _PoseSignature(center, scale, aspect, 0.0, 0.0, 0.0, False)
        return _PoseSignature(center, scale, aspect, proxy[0], proxy[1], proxy[2], True)

    def _is_duplicate_pose(self, signature: _PoseSignature, solve_eligible_only: bool = False) -> bool:
        if solve_eligible_only:
            previous_signatures = (self._signatures_by_index[index] for index in self._solve_eligible_indices)
        else:
            previous_signatures = iter(self._pose_signatures)
        for previous in previous_signatures:
            center_distance_px = float(np.linalg.norm(signature.center - previous.center))
            if center_distance_px >= self.config.min_center_distance_px:
                continue
            scale_delta = abs(signature.scale - previous.scale) / max(signature.scale, previous.scale, 1e-6)
            aspect_delta = abs(signature.aspect - previous.aspect) / max(signature.aspect, previous.aspect, 1e-6)
            pose_delta_small = True
            if signature.pose_valid and previous.pose_valid:
                pose_delta_small = max(
                    abs(signature.roll_slope - previous.roll_slope),
                    abs(signature.tilt_x - previous.tilt_x),
                    abs(signature.tilt_y - previous.tilt_y),
                ) < self.config.min_aspect_change_ratio
            if scale_delta < self.config.min_scale_change_ratio and aspect_delta < self.config.min_aspect_change_ratio and pose_delta_small:
                return True
        return False

    def _grid_counts(self) -> np.ndarray:
        frame_sets = [[set() for _ in range(self.config.grid_cols)] for _ in range(self.config.grid_rows)]
        for observation in self._observations.values():
            rows, cols = coverage_cell_indices(
                observation.image_points,
                self.image_size,
                self.config.grid_cols,
                self.config.grid_rows,
            )
            for row, col in zip(rows, cols):
                frame_sets[int(row)][int(col)].add(observation.index)
        return np.array([[len(frame_sets[row][col]) for col in range(self.config.grid_cols)] for row in range(self.config.grid_rows)], dtype=np.int64)

    def _zone_counts(self) -> Dict[str, int]:
        zone_to_token = {
            "left_20": "EDGE_LEFT",
            "right_20": "EDGE_RIGHT",
            "top_20": "EDGE_TOP",
            "bottom_20": "EDGE_BOTTOM",
            "top_left_20": "CORNER_TOP_LEFT",
            "top_right_20": "CORNER_TOP_RIGHT",
            "bottom_left_20": "CORNER_BOTTOM_LEFT",
            "bottom_right_20": "CORNER_BOTTOM_RIGHT",
        }
        counts = {token: 0 for token in zone_to_token.values()}
        for observation in self._observations.values():
            masks = coverage_zone_masks(observation.image_points, self.image_size)
            for zone, token in zone_to_token.items():
                counts[token] += int(np.any(masks[zone]))
        counts["TOP_LEFT"] = counts["CORNER_TOP_LEFT"]
        counts["TOP_RIGHT"] = counts["CORNER_TOP_RIGHT"]
        counts["BOTTOM_LEFT"] = counts["CORNER_BOTTOM_LEFT"]
        counts["BOTTOM_RIGHT"] = counts["CORNER_BOTTOM_RIGHT"]
        return counts

    def _radial_counts(self) -> Dict[str, int]:
        counts = {name: 0 for name in ("CENTER", "MID", "OUTER", "OUTER_TL", "OUTER_TR", "OUTER_BL", "OUTER_BR")}
        width, height = self.image_size
        for observation in self._observations.values():
            points = np.asarray(observation.image_points, dtype=np.float64)
            radius = _normalized_radius(points, self.image_size)
            counts["CENTER"] += int(np.any(radius < 0.25))
            counts["MID"] += int(np.any((radius >= 0.25) & (radius < self.config.outer_radius_start)))
            outer = radius >= self.config.outer_radius_start
            counts["OUTER"] += int(np.any(outer))
            if np.any(outer):
                outer_points = points[outer]
                counts["OUTER_TL"] += int(np.any((outer_points[:, 0] < width * 0.5) & (outer_points[:, 1] < height * 0.5)))
                counts["OUTER_TR"] += int(np.any((outer_points[:, 0] >= width * 0.5) & (outer_points[:, 1] < height * 0.5)))
                counts["OUTER_BL"] += int(np.any((outer_points[:, 0] < width * 0.5) & (outer_points[:, 1] >= height * 0.5)))
                counts["OUTER_BR"] += int(np.any((outer_points[:, 0] >= width * 0.5) & (outer_points[:, 1] >= height * 0.5)))
        return counts

    def _scale_counts(self) -> Dict[str, int]:
        counts = {"SMALL": 0, "MEDIUM": 0, "LARGE": 0}
        for index in self._solve_eligible_indices:
            signature = self._signatures_by_index[index]
            if signature.scale < self.config.scale_small_max:
                counts["SMALL"] += 1
            elif signature.scale >= self.config.scale_large_min:
                counts["LARGE"] += 1
            else:
                counts["MEDIUM"] += 1
        return counts

    def _pose_counts(self) -> Dict[str, int]:
        counts = {"ROLL_NEG": 0, "ROLL_NEUTRAL": 0, "ROLL_POS": 0, "TILT_X_STRONG": 0, "TILT_Y_STRONG": 0, "OBLIQUE": 0}
        for index in self._solve_eligible_indices:
            signature = self._signatures_by_index[index]
            if not signature.pose_valid:
                continue
            if signature.roll_slope < -self.config.roll_strong_ratio:
                counts["ROLL_NEG"] += 1
            elif signature.roll_slope > self.config.roll_strong_ratio:
                counts["ROLL_POS"] += 1
            else:
                counts["ROLL_NEUTRAL"] += 1
            if signature.tilt_x >= self.config.tilt_strong_ratio:
                counts["TILT_X_STRONG"] += 1
            if signature.tilt_y >= self.config.tilt_strong_ratio:
                counts["TILT_Y_STRONG"] += 1
            if min(signature.tilt_x, signature.tilt_y) >= self.config.oblique_ratio:
                counts["OBLIQUE"] += 1
        return counts



def classify_frame_quality(gray: np.ndarray, thresholds: QualityThresholds = QualityThresholds()) -> FrameQuality:
    if gray.ndim != 2:
        raise ValueError("quality input must be mono8")
    gray_u8 = np.asarray(gray, dtype=np.uint8)
    laplacian_var = float(cv2.Laplacian(gray_u8, cv2.CV_64F).var())
    p5, p95 = np.percentile(gray_u8, [5.0, 95.0])
    contrast = float(p95 - p5)
    low_clip = float(np.mean(gray_u8 <= 2))
    high_clip = float(np.mean(gray_u8 >= 253))
    saturated_fraction = low_clip + high_clip
    reasons = []
    if laplacian_var < thresholds.min_laplacian_var:
        reasons.append("LOW_TEXTURE")
    if contrast < thresholds.min_contrast:
        reasons.append("LOW_CONTRAST")
    # 合成棋盘会两端饱和但仍有有效边缘,真实曝光异常通常表现为单侧大面积削顶。
    if max(low_clip, high_clip) > thresholds.max_saturated_fraction and min(low_clip, high_clip) < 0.05:
        reasons.append("SATURATED")
    return FrameQuality(
        accepted=not reasons,
        reasons=reasons,
        laplacian_var=laplacian_var,
        contrast=contrast,
        saturated_fraction=saturated_fraction,
    )


class CoverageSelector:
    def __init__(
        self,
        image_size: Tuple[int, int],
        grid: Tuple[int, int] = (4, 3),
        min_new_bins: int = 1,
        min_center_distance_px: float = 35.0,
        min_scale_change_ratio: float = 0.08,
        min_aspect_change_ratio: float = 0.10,
    ) -> None:
        self.image_size = image_size
        self.grid = grid
        self.min_new_bins = min_new_bins
        self.min_center_distance_px = min_center_distance_px
        self.min_scale_change_ratio = min_scale_change_ratio
        self.min_aspect_change_ratio = min_aspect_change_ratio
        self._covered_bins = set()
        self._pose_signatures: List[_PoseSignature] = []

    @property
    def coverage_percent(self) -> float:
        return 100.0 * len(self._covered_bins) / float(self.grid[0] * self.grid[1])

    def consider(self, image_points: np.ndarray) -> CoverageDecision:
        points = np.asarray(image_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] == 0:
            raise ValueError("image_points must have shape (N, 2) and be non-empty")
        signature = self._pose_signature(points)
        reasons = []
        if self._is_duplicate_pose(signature):
            reasons.append("DUPLICATE_POSE")
        bins = self._bins_for_points(points)
        new_bins = len(bins - self._covered_bins)
        coverage_already_full = len(self._covered_bins) >= self.grid[0] * self.grid[1]
        # ROI覆盖满后不能继续要求新增bucket，否则大视场AprilGrid会卡在少量accepted帧。
        if new_bins < self.min_new_bins and not coverage_already_full:
            reasons.append("NO_NEW_ROI")
        accepted = not reasons
        if accepted:
            self._pose_signatures.append(signature)
            self._covered_bins.update(bins)
        return CoverageDecision(
            accepted=accepted,
            reasons=reasons,
            new_bins=new_bins,
            covered_bins=len(self._covered_bins),
            total_bins=self.grid[0] * self.grid[1],
        )

    def _pose_signature(self, points: np.ndarray) -> _PoseSignature:
        center = points.mean(axis=0)
        span = np.maximum(np.ptp(points, axis=0), 1.0)
        width, height = self.image_size
        scale = float(np.sqrt(span[0] * span[1]) / max(min(width, height), 1))
        aspect = float(span[0] / span[1])
        return _PoseSignature(center=center, scale=scale, aspect=aspect)

    def _is_duplicate_pose(self, signature: _PoseSignature) -> bool:
        for previous in self._pose_signatures:
            center_distance_px = float(np.linalg.norm(signature.center - previous.center))
            if center_distance_px >= self.min_center_distance_px:
                continue
            scale_delta = abs(signature.scale - previous.scale) / max(signature.scale, previous.scale, 1e-6)
            aspect_delta = abs(signature.aspect - previous.aspect) / max(signature.aspect, previous.aspect, 1e-6)
            # 同中心但明显变焦距/倾角的AprilGrid帧仍提供独立标定约束，不能按中心距离一概判重。
            if scale_delta < self.min_scale_change_ratio and aspect_delta < self.min_aspect_change_ratio:
                return True
        return False

    def _bins_for_points(self, points: np.ndarray):
        width, height = self.image_size
        grid_x, grid_y = self.grid
        x_bins = np.clip((points[:, 0] / width * grid_x).astype(int), 0, grid_x - 1)
        y_bins = np.clip((points[:, 1] / height * grid_y).astype(int), 0, grid_y - 1)
        return {(int(x), int(y)) for x, y in zip(x_bins, y_bins)}


def _observation_bucket(observation: Observation, image_size: Tuple[int, int]) -> Tuple[int, int, int, int]:
    points = np.asarray(observation.image_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] == 0:
        return (0, 0, 0, 0)
    width, height = image_size
    center = points.mean(axis=0)
    x_bin = int(np.clip(center[0] / max(width, 1) * 4.0, 0, 3))
    y_bin = int(np.clip(center[1] / max(height, 1) * 3.0, 0, 2))
    span = np.ptp(points, axis=0)
    scale = float(np.sqrt(max(span[0] * span[1], 0.0)) / max(min(width, height), 1))
    scale_bin = int(np.clip(scale * 4.0, 0, 3))
    normalized = (center - np.array([width * 0.5, height * 0.5], dtype=np.float64)) / max(min(width, height), 1)
    radial_bin = int(np.clip(np.linalg.norm(normalized) * 4.0, 0, 3))
    return (x_bin, y_bin, scale_bin, radial_bin)


def _observation_feature_vector(observation: Observation, image_size: Tuple[int, int]) -> np.ndarray:
    points = np.asarray(observation.image_points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] == 0:
        return np.zeros(4, dtype=np.float64)
    width, height = image_size
    center = points.mean(axis=0) / np.array([max(width, 1), max(height, 1)], dtype=np.float64)
    span = np.maximum(np.ptp(points, axis=0), 1.0)
    scale = float(np.sqrt(span[0] * span[1]) / max(min(width, height), 1))
    aspect = float(np.log(span[0] / span[1]))
    return np.array([center[0], center[1], scale, aspect], dtype=np.float64)


def select_representative_observations(
    observations: Sequence[Observation],
    max_count: int,
    image_size: Tuple[int, int] = (1280, 1088),
) -> List[Observation]:
    """求解帧数上限必须保留中心/尺度/透视多样性，不能简单截断早期帧。"""
    observations = list(observations)
    # 0/负数配置表示不裁剪，少量观测直接保留原始采集顺序。
    if max_count <= 0 or len(observations) <= max_count:
        return observations
    if max_count == 1:
        return [observations[0]]

    # 特征向量只使用像面中心、尺度和形状，不依赖当前模型初值。
    features = np.vstack([_observation_feature_vector(observation, image_size) for observation in observations])
    selected_indices = [0]
    min_distances = np.linalg.norm(features - features[0], axis=1)
    # 最远点采样用当前已选集合的最小距离，优先保留覆盖/姿态差异最大的帧。
    while len(selected_indices) < max_count:
        min_distances[selected_indices] = -1.0
        next_index = int(np.argmax(min_distances))
        selected_indices.append(next_index)
        candidate_distances = np.linalg.norm(features - features[next_index], axis=1)
        min_distances = np.minimum(min_distances, candidate_distances)

    return [observations[index] for index in sorted(selected_indices)]


def split_train_holdout(
    observations: Sequence[Observation],
    holdout_fraction: float = 0.2,
    seed: int = 20260806,
    image_size: Tuple[int, int] = (1280, 1088),
) -> Tuple[List[Observation], List[Observation]]:
    if not 0.0 < holdout_fraction < 1.0:
        raise ValueError("holdout_fraction must be in (0, 1)")
    observations = list(observations)
    if len(observations) < 2:
        return observations, []
    holdout_count = max(1, int(round(len(observations) * holdout_fraction)))
    buckets: Dict[Tuple[int, int, int, int], List[int]] = {}
    for idx, observation in enumerate(observations):
        buckets.setdefault(_observation_bucket(observation, image_size), []).append(idx)
    holdout_indices = set()
    for _bucket, indices in sorted(buckets.items(), key=lambda item: (-len(item[1]), item[0])):
        if len(holdout_indices) >= holdout_count:
            break
        if len(indices) >= 2:
            holdout_indices.add(indices[-1])
    if len(holdout_indices) < holdout_count:
        for idx in range(len(observations) - 1, -1, -1):
            if idx not in holdout_indices:
                holdout_indices.add(idx)
                if len(holdout_indices) >= holdout_count:
                    break
    train = [obs for idx, obs in enumerate(observations) if idx not in holdout_indices]
    holdout = [obs for idx, obs in enumerate(observations) if idx in holdout_indices]
    return train, holdout
