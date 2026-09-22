"""提供在线标定结果的host-only离线审计核心函数。"""

from dataclasses import fields, is_dataclass, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import cv2
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .double_sphere import DSParameters, project_points as project_ds, unproject_pixels as unproject_ds
from .kb4 import KB4Parameters, project_points as project_kb4, unproject_pixels as unproject_kb4
from .quality import coverage_cell_indices, coverage_zone_masks
from .solver import Observation, estimate_fixed_intrinsics_pose, transform_points


@dataclass(frozen=True)
class AuditConfig:
    max_holdout_rms_px: float = 1.0
    max_holdout_p95_px: float = 1.0
    audit_grid_cols: int = 10
    audit_grid_rows: int = 8
    audit_min_grid_frames: int = 1
    audit_min_edge_frames: int = 5
    audit_min_corner_frames: int = 3
    audit_outer_radius_start: float = 0.75
    audit_max_outer_p95_px: float = 1.5
    audit_dense_grid_cols: int = 129
    audit_dense_grid_rows: int = 109
    audit_max_roundtrip_px: float = 1.0e-6
    audit_virtual_pinhole_fov_deg: float = 120.0


@dataclass(frozen=True)
class ZoneCoverage:
    corner_count: int
    frame_count: int


@dataclass(frozen=True)
class CoverageAudit:
    image_size: Tuple[int, int]
    corner_counts: np.ndarray
    frame_counts: np.ndarray
    zones: Dict[str, ZoneCoverage]
    failed_cells: List[Tuple[int, int]]
    failed_zones: List[str]
    corner_points_px: np.ndarray
    frame_centers_px: np.ndarray


@dataclass(frozen=True)
class CaptureReplaySummary:
    ready: bool
    accepted_count: int
    missing: List[str]
    next: List[str]
    solve_eligible_count: int = 0
    coverage_only_count: int = 0


@dataclass(frozen=True)
class ResidualRecord:
    frame_index: int
    observed_px: Tuple[float, float]
    residual_px: Tuple[float, float]
    model: str
    split: str


@dataclass(frozen=True)
class DenseDomainAudit:
    model: str
    grid_shape: Tuple[int, int]
    unproject_valid_count: int
    roundtrip_valid_count: int
    nonfinite_count: int
    max_roundtrip_px: float
    full_frame_roi_claimed: bool
    valid_mask: np.ndarray
    roundtrip_error_px: np.ndarray


@dataclass(frozen=True)
class RayAngleComparison:
    mutually_valid_count: int
    mean_angle_deg: float
    p95_angle_deg: float
    max_angle_deg: float
    angle_deg: np.ndarray
    valid_mask: np.ndarray


def _grid_pixels(image_size: Tuple[int, int], grid_cols: int, grid_rows: int) -> Tuple[np.ndarray, np.ndarray]:
    width, height = image_size
    xs = np.linspace(0.0, width - 1.0, grid_cols, dtype=np.float64)
    ys = np.linspace(0.0, height - 1.0, grid_rows, dtype=np.float64)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack((xx.reshape(-1), yy.reshape(-1))), xx


def _cell_indices(points: np.ndarray, image_size: Tuple[int, int], grid_cols: int, grid_rows: int) -> Tuple[np.ndarray, np.ndarray]:
    return coverage_cell_indices(points, image_size, grid_cols, grid_rows)


def _zone_masks(points: np.ndarray, image_size: Tuple[int, int]) -> Mapping[str, np.ndarray]:
    return coverage_zone_masks(points, image_size)


def compute_coverage(
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    grid_cols: int = 10,
    grid_rows: int = 8,
    min_grid_frames: int = 1,
    min_edge_frames: int = 5,
    min_corner_frames: int = 3,
) -> CoverageAudit:
    corner_counts = np.zeros((grid_rows, grid_cols), dtype=np.int64)
    frame_sets = [[set() for _ in range(grid_cols)] for _ in range(grid_rows)]
    zone_points: Dict[str, int] = {}
    zone_frames: Dict[str, set] = {}
    # coverage.png 需要同时展示真实角点散点和每帧中心，审计对象必须保留原始像素证据。
    corner_points_px: List[np.ndarray] = []
    frame_centers_px: List[np.ndarray] = []
    for observation in observations:
        points = np.asarray(observation.image_points, dtype=np.float64)
        if points.size == 0:
            continue
        corner_points_px.append(points)
        frame_centers_px.append(np.mean(points, axis=0))
        rows, cols = _cell_indices(points, image_size, grid_cols, grid_rows)
        for row, col in zip(rows, cols):
            corner_counts[row, col] += 1
            frame_sets[row][col].add(observation.index)
        for name, mask in _zone_masks(points, image_size).items():
            zone_points[name] = zone_points.get(name, 0) + int(mask.sum())
            zone_frames.setdefault(name, set())
            if np.any(mask):
                zone_frames[name].add(observation.index)
    frame_counts = np.array([[len(frame_sets[row][col]) for col in range(grid_cols)] for row in range(grid_rows)], dtype=np.int64)
    zones = {name: ZoneCoverage(zone_points.get(name, 0), len(zone_frames.get(name, set()))) for name in _zone_masks(np.zeros((1, 2)), image_size)}
    failed_cells = [(row, col) for row in range(grid_rows) for col in range(grid_cols) if frame_counts[row, col] < min_grid_frames]
    failed_zones = []
    for name, zone in zones.items():
        is_corner = ("left" in name or "right" in name) and ("top" in name or "bottom" in name)
        is_edge = ("left" in name) or ("right" in name) or ("top" in name) or ("bottom" in name)
        threshold = min_corner_frames if is_corner else min_edge_frames if is_edge else min_grid_frames
        if zone.frame_count < threshold:
            failed_zones.append(name)
    corners = np.vstack(corner_points_px) if corner_points_px else np.empty((0, 2), dtype=np.float64)
    centers = np.vstack(frame_centers_px) if frame_centers_px else np.empty((0, 2), dtype=np.float64)
    return CoverageAudit(image_size, corner_counts, frame_counts, zones, failed_cells, failed_zones, corners, centers)


def capture_config_from_mapping(data: Mapping[str, object]):
    from .quality import CaptureAssistantConfig

    return CaptureAssistantConfig(
        grid_cols=int(data.get("capture_grid_cols", data.get("audit_grid_cols", 10))),
        grid_rows=int(data.get("capture_grid_rows", data.get("audit_grid_rows", 8))),
        min_observations=int(data.get("capture_min_observations", data.get("min_observations", 12))),
        min_grid_frames=int(data.get("capture_min_grid_frames", data.get("audit_min_grid_frames", 1))),
        min_edge_frames=int(data.get("capture_min_edge_frames", data.get("audit_min_edge_frames", 5))),
        min_corner_frames=int(data.get("capture_min_corner_frames", data.get("audit_min_corner_frames", 3))),
        outer_radius_start=float(data.get("capture_outer_radius_start", data.get("audit_outer_radius_start", 0.75))),
        min_radial_bucket_frames=int(data.get("capture_min_radial_bucket_frames", 1)),
        min_outer_quadrant_frames=int(data.get("capture_min_outer_quadrant_frames", 1)),
        min_scale_bucket_frames=int(data.get("capture_min_scale_bucket_frames", 2)),
        min_coverage_tags=int(data.get("capture_min_coverage_tags", 6)),
        min_coverage_tag_rows=int(data.get("capture_min_coverage_tag_rows", 2)),
        min_coverage_tag_cols=int(data.get("capture_min_coverage_tag_cols", 2)),
        min_solve_tags=int(data.get("capture_min_solve_tags", 18)),
        min_solve_tag_rows=int(data.get("capture_min_solve_tag_rows", 4)),
        min_solve_tag_cols=int(data.get("capture_min_solve_tag_cols", 4)),
        min_holdout_tags=int(data.get("capture_min_holdout_tags", 28)),
        min_holdout_tag_rows=int(data.get("capture_min_holdout_tag_rows", 6)),
        min_holdout_tag_cols=int(data.get("capture_min_holdout_tag_cols", 6)),
        min_holdout_hull_area_ratio=float(data.get("capture_min_holdout_hull_area_ratio", 0.28)),
        min_holdout_margin_px=float(data.get("capture_min_holdout_margin_px", 24.0)),
        min_pose_bucket_frames=int(data.get("capture_min_pose_bucket_frames", 2)),
        min_center_distance_px=float(data.get("capture_min_center_distance_px", 35.0)),
        min_scale_change_ratio=float(data.get("capture_min_scale_change_ratio", 0.08)),
        min_aspect_change_ratio=float(data.get("capture_min_aspect_change_ratio", 0.10)),
        scale_small_max=float(data.get("capture_scale_small_max", 0.22)),
        scale_large_min=float(data.get("capture_scale_large_min", 0.38)),
        roll_strong_ratio=float(data.get("capture_roll_strong_ratio", 0.10)),
        tilt_strong_ratio=float(data.get("capture_tilt_strong_ratio", 0.18)),
        oblique_ratio=float(data.get("capture_oblique_ratio", 0.22)),
    )


def capture_replay_summary(path: Path, image_size: Tuple[int, int], config: Optional[object] = None) -> CaptureReplaySummary:
    from .quality import CaptureAssistant

    assistant = CaptureAssistant(image_size, config or capture_config_from_mapping({}))
    detected_count = 0
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("status", "DETECTED") != "DETECTED":
            continue
        detected_count += 1
        image_points = np.asarray(record["image_points"], dtype=np.float64)
        object_points = np.asarray(record.get("object_points", np.zeros((image_points.shape[0], 3))), dtype=np.float64)
        # 回放保留源accepted数量，同时重新执行solve eligibility门禁作为ready判定。
        assistant.consider_and_accept(Observation(object_points, image_points, index=int(record["frame_index"])))
    snapshot = assistant.snapshot()
    return CaptureReplaySummary(
        ready=snapshot.ready,
        accepted_count=detected_count,
        missing=snapshot.missing,
        next=snapshot.next,
        solve_eligible_count=snapshot.solve_eligible_count,
        coverage_only_count=max(0, detected_count - snapshot.solve_eligible_count),
    )


def _pose_is_valid(pose: object) -> bool:
    # holdout审计必须对失败pose fail-closed，禁止用非PASS或非有限pose继续投影。
    status = getattr(pose, "__dict__", {}).get("status", "PASS")
    if status != "PASS":
        return False
    rvec = np.asarray(getattr(pose, "rvec", []), dtype=np.float64)
    tvec = np.asarray(getattr(pose, "tvec", []), dtype=np.float64)
    return rvec.shape == (3,) and tvec.shape == (3,) and np.isfinite(rvec).all() and np.isfinite(tvec).all()


def radial_bucket_labels() -> Tuple[str, ...]:
    return ("[0,.25)", "[.25,.5)", "[.5,.75)", "[.75,1)", ">=1")


def radial_bucket_indices(points: np.ndarray, image_size: Tuple[int, int]) -> np.ndarray:
    width, height = image_size
    center = np.array([width * 0.5, height * 0.5], dtype=np.float64)
    radius = np.linalg.norm((np.asarray(points, dtype=np.float64) - center) / center, axis=1)
    return np.digitize(radius, np.array([0.25, 0.5, 0.75, 1.0], dtype=np.float64), right=False)


def evaluate_holdout_records(
    model: str,
    params: object,
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    split: str,
) -> Tuple[List[ResidualRecord], int]:
    projector = project_ds if model == "ds" else project_kb4
    records: List[ResidualRecord] = []
    invalid_count = 0
    for observation in observations:
        pose = estimate_fixed_intrinsics_pose(model, params, observation, image_size)
        if not _pose_is_valid(pose):
            invalid_count += int(observation.object_points.shape[0])
            continue
        points = transform_points(observation.object_points, pose.rvec, pose.tvec)
        projected, valid = projector(points, params)
        invalid_count += int((~valid).sum())
        for observed, residual in zip(observation.image_points[valid], projected[valid] - observation.image_points[valid]):
            records.append(ResidualRecord(observation.index, (float(observed[0]), float(observed[1])), (float(residual[0]), float(residual[1])), model, split))
    return records, invalid_count


def cross_tag_validation_records(
    model: str,
    params: object,
    observations: Sequence[Observation],
    image_size: Tuple[int, int],
    tag_ids_by_frame: Optional[Mapping[int, np.ndarray]] = None,
    first_tag_id: int = 0,
    target_cols: int = 1,
) -> Tuple[List[ResidualRecord], int]:
    projector = project_ds if model == "ds" else project_kb4
    records: List[ResidualRecord] = []
    invalid_count = 0
    for observation in observations:
        raw_tag_ids = None if tag_ids_by_frame is None else tag_ids_by_frame.get(observation.index)
        tag_ids = None if raw_tag_ids is None else np.asarray(raw_tag_ids, dtype=np.int64)
        if tag_ids is None or tag_ids.shape[0] != observation.object_points.shape[0]:
            invalid_count += int(observation.object_points.shape[0])
            continue
        # cross-tag 训练/评估拆分按目标板物理棋盘 parity，不能依赖检测返回顺序。
        tag_offsets = tag_ids - int(first_tag_id)
        tag_cols = np.maximum(1, int(target_cols))
        checker_parity = ((tag_offsets // tag_cols) + (tag_offsets % tag_cols)) % 2
        for parity in (0, 1):
            fit_mask = checker_parity == parity
            eval_mask = ~fit_mask
            if int(fit_mask.sum()) < 4 or int(eval_mask.sum()) == 0:
                invalid_count += int(observation.object_points.shape[0])
                continue
            fit_obs = Observation(observation.object_points[fit_mask], observation.image_points[fit_mask], observation.index)
            pose = estimate_fixed_intrinsics_pose(model, params, fit_obs, image_size)
            if not _pose_is_valid(pose):
                invalid_count += int(eval_mask.sum())
                continue
            eval_points = transform_points(observation.object_points[eval_mask], pose.rvec, pose.tvec)
            projected, valid = projector(eval_points, params)
            invalid_count += int((~valid).sum())
            for observed, residual in zip(observation.image_points[eval_mask][valid], projected[valid] - observation.image_points[eval_mask][valid]):
                records.append(ResidualRecord(observation.index, (float(observed[0]), float(observed[1])), (float(residual[0]), float(residual[1])), model, "cross_tag"))
    return records, invalid_count


def dense_domain_audit(model: str, params: object, image_size: Tuple[int, int], grid_cols: int = 129, grid_rows: int = 109, max_roundtrip_px: float = 1.0e-6) -> DenseDomainAudit:
    pixels, _ = _grid_pixels(image_size, grid_cols, grid_rows)
    unprojector = unproject_ds if model == "ds" else unproject_kb4
    projector = project_ds if model == "ds" else project_kb4
    rays, ray_valid = unprojector(pixels, params)
    roundtrip = np.full_like(pixels, np.nan)
    point_valid = np.zeros(pixels.shape[0], dtype=bool)
    if np.any(ray_valid):
        roundtrip[ray_valid], point_valid[ray_valid] = projector(rays[ray_valid], params)
    error = np.linalg.norm(roundtrip - pixels, axis=1)
    finite_error = np.isfinite(error)
    valid_mask = ray_valid & point_valid & finite_error & (error <= max_roundtrip_px)
    nonfinite_count = int((~np.isfinite(rays).all(axis=1)).sum() + (~np.isfinite(roundtrip).all(axis=1)).sum())
    max_error = float(np.max(error[finite_error])) if np.any(finite_error) else float("inf")
    return DenseDomainAudit(model, (grid_rows, grid_cols), int(ray_valid.sum()), int(point_valid.sum()), nonfinite_count, max_error, bool(valid_mask.all()), valid_mask.reshape(grid_rows, grid_cols), error.reshape(grid_rows, grid_cols))


def ray_angle_comparison(ds_params: DSParameters, kb4_params: KB4Parameters, image_size: Tuple[int, int], grid_cols: int = 129, grid_rows: int = 109, max_roundtrip_px: float = 1.0e-6) -> RayAngleComparison:
    pixels, _ = _grid_pixels(image_size, grid_cols, grid_rows)
    ds_rays, ds_valid = unproject_ds(pixels, ds_params)
    kb4_rays, kb4_valid = unproject_kb4(pixels, kb4_params)
    # ray angle 只比较两个模型都能稳定 unproject->project 回环的像素，避免把不可逆域混入角度统计。
    ds_roundtrip = np.full_like(pixels, np.nan)
    kb4_roundtrip = np.full_like(pixels, np.nan)
    ds_project_valid = np.zeros(pixels.shape[0], dtype=bool)
    kb4_project_valid = np.zeros(pixels.shape[0], dtype=bool)
    if np.any(ds_valid):
        ds_roundtrip[ds_valid], ds_project_valid[ds_valid] = project_ds(ds_rays[ds_valid], ds_params)
    if np.any(kb4_valid):
        kb4_roundtrip[kb4_valid], kb4_project_valid[kb4_valid] = project_kb4(kb4_rays[kb4_valid], kb4_params)
    ds_error = np.linalg.norm(ds_roundtrip - pixels, axis=1)
    kb4_error = np.linalg.norm(kb4_roundtrip - pixels, axis=1)
    valid = ds_valid & kb4_valid & ds_project_valid & kb4_project_valid & np.isfinite(ds_error) & np.isfinite(kb4_error) & (ds_error <= max_roundtrip_px) & (kb4_error <= max_roundtrip_px)
    angles = np.full(pixels.shape[0], np.nan, dtype=np.float64)
    if np.any(valid):
        dots = np.sum(ds_rays[valid] * kb4_rays[valid], axis=1)
        angles[valid] = np.rad2deg(np.arccos(np.clip(dots, -1.0, 1.0)))
    values = angles[valid]
    return RayAngleComparison(int(valid.sum()), float(np.mean(values)) if values.size else float("inf"), float(np.percentile(values, 95.0)) if values.size else float("inf"), float(np.max(values)) if values.size else float("inf"), angles.reshape(grid_rows, grid_cols), valid.reshape(grid_rows, grid_cols))


def write_validation_plots(output_dir: Path, coverage: CoverageAudit, ds_domain: DenseDomainAudit, kb4_domain: DenseDomainAudit, comparison: RayAngleComparison, residual_records: Sequence[ResidualRecord]) -> None:
    plots_dir = Path(output_dir) / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.imshow(coverage.frame_counts, origin="upper")
    # 独立帧 heatmap 之外叠加真实角点和中心散点，便于人工识别覆盖假阳性。
    if coverage.corner_points_px.size:
        height, width = coverage.frame_counts.shape
        image_width, image_height = coverage.image_size
        ax.scatter(
            coverage.corner_points_px[:, 0] / max(1.0, float(image_width)) * (width - 1),
            coverage.corner_points_px[:, 1] / max(1.0, float(image_height)) * (height - 1),
            s=4,
            c="white",
            alpha=0.55,
        )
    if coverage.frame_centers_px.size:
        height, width = coverage.frame_counts.shape
        image_width, image_height = coverage.image_size
        ax.scatter(
            coverage.frame_centers_px[:, 0] / max(1.0, float(image_width)) * (width - 1),
            coverage.frame_centers_px[:, 1] / max(1.0, float(image_height)) * (height - 1),
            s=24,
            c="red",
            marker="x",
        )
    ax.set_title("10x8 independent frame coverage")
    fig.tight_layout()
    fig.savefig(plots_dir / "coverage.png")
    plt.close(fig)
    _write_residual_plots(plots_dir, residual_records)
    for name, data in {"ray_validity.png": ds_domain.valid_mask.astype(float) + kb4_domain.valid_mask.astype(float), "model_ray_difference.png": comparison.angle_deg}.items():
        fig, ax = plt.subplots(figsize=(7, 4))
        ax.imshow(data, origin="upper")
        ax.set_title(name.replace(".png", ""))
        fig.tight_layout()
        fig.savefig(plots_dir / name)
        plt.close(fig)


def build_virtual_pinhole_maps(model: str, params: object, image_size: Tuple[int, int], pinhole_size: Tuple[int, int] = (640, 480), fov_deg: float = 120.0) -> Tuple[np.ndarray, np.ndarray]:
    if not np.isfinite(float(fov_deg)) or float(fov_deg) <= 0.0 or float(fov_deg) >= 180.0:
        raise ValueError("virtual pinhole FOV must be finite and within (0, 180) degrees")
    out_w, out_h = pinhole_size
    # 虚拟针孔预览使用固定水平FOV，焦距由输出宽度和FOV唯一确定，避免随输入分辨率漂移。
    focal = (out_w * 0.5) / np.tan(np.deg2rad(float(fov_deg)) * 0.5)
    grid_x, grid_y = np.meshgrid(np.arange(out_w, dtype=np.float64), np.arange(out_h, dtype=np.float64))
    ray_x = (grid_x - out_w * 0.5) / focal
    ray_y = (grid_y - out_h * 0.5) / focal
    rays = np.column_stack((ray_x.reshape(-1), ray_y.reshape(-1), np.ones(out_w * out_h, dtype=np.float64)))
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    projector = project_ds if model == "ds" else project_kb4
    pixels, valid = projector(rays, params)
    map_x = pixels[:, 0].reshape(out_h, out_w)
    map_y = pixels[:, 1].reshape(out_h, out_w)
    map_x[~valid.reshape(out_h, out_w)] = np.nan
    map_y[~valid.reshape(out_h, out_w)] = np.nan
    return map_x, map_y


def write_undistort_artifacts(output_dir: Path, image: np.ndarray, ds_params: DSParameters, kb4_params: KB4Parameters, image_size: Tuple[int, int], fov_deg: float = 120.0) -> None:
    plots_dir = Path(output_dir) / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    ds_map_x, ds_map_y = build_virtual_pinhole_maps("ds", ds_params, image_size, fov_deg=fov_deg)
    kb4_map_x, kb4_map_y = build_virtual_pinhole_maps("kb4", kb4_params, image_size, fov_deg=fov_deg)
    map_difference = np.sqrt((ds_map_x - kb4_map_x) ** 2 + (ds_map_y - kb4_map_y) ** 2)
    map_difference[~np.isfinite(map_difference)] = 0.0
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.imshow(map_difference, origin="upper")
    ax.set_title("undistort_map_difference")
    fig.tight_layout()
    fig.savefig(plots_dir / "undistort_map_difference.png")
    plt.close(fig)
    ds_preview = cv2.remap(image, ds_map_x.astype(np.float32), ds_map_y.astype(np.float32), interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    kb4_preview = cv2.remap(image, kb4_map_x.astype(np.float32), kb4_map_y.astype(np.float32), interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    preview = np.concatenate((ds_preview, kb4_preview), axis=1)
    cv2.imwrite(str(plots_dir / "undistort_preview.png"), preview)


def _write_residual_plots(plots_dir: Path, residual_records: Sequence[ResidualRecord]) -> None:
    for name in ("residual_heatmap.png", "residual_vector_field.png"):
        fig, axes = plt.subplots(2, 3, figsize=(12, 7))
        for row, model in enumerate(("ds", "kb4")):
            for col, split in enumerate(("holdout", "cross_tag", "accepted")):
                ax = axes[row][col]
                panel_records = [record for record in residual_records if record.model == model and record.split == split]
                points = np.array([record.observed_px for record in panel_records], dtype=np.float64) if panel_records else np.empty((0, 2))
                residuals = np.array([record.residual_px for record in panel_records], dtype=np.float64) if panel_records else np.empty((0, 2))
                ax.set_title(f"{model} {split} ({len(panel_records)})" if panel_records else f"{model} {split} EMPTY")
                if not panel_records:
                    ax.text(0.5, 0.5, "EMPTY", ha="center", va="center", transform=ax.transAxes)
                    continue
                if name == "residual_heatmap.png":
                    magnitudes = np.linalg.norm(residuals, axis=1)
                    ax.scatter(points[:, 0], points[:, 1], c=magnitudes, s=8)
                else:
                    ax.quiver(points[:, 0], points[:, 1], residuals[:, 0], residuals[:, 1])
        fig.suptitle(name.replace(".png", ""))
        fig.tight_layout()
        fig.savefig(plots_dir / name)
        plt.close(fig)


def residual_summary(records: Sequence[ResidualRecord], image_size: Optional[Tuple[int, int]] = None, min_normalized_radius: Optional[float] = None) -> dict:
    # outer gate 固定按归一化像面半径筛选，空样本必须 fail-closed 而不是退化为全域指标。
    filtered = list(records)
    if image_size is not None and min_normalized_radius is not None:
        width, height = image_size
        center = np.array([width * 0.5, height * 0.5], dtype=np.float64)
        scale = center
        filtered = [
            record
            for record in records
            if float(np.linalg.norm((np.asarray(record.observed_px, dtype=np.float64) - center) / scale)) >= float(min_normalized_radius)
        ]
    residuals = np.array([record.residual_px for record in filtered], dtype=np.float64) if filtered else np.empty((0, 2))
    norms = np.linalg.norm(residuals, axis=1) if residuals.size else np.empty((0,))
    return {"status": "OK" if norms.size else "EMPTY", "count": int(norms.size), "rms_px": float(np.sqrt(np.mean(norms * norms))) if norms.size else float("inf"), "p95_px": float(np.percentile(norms, 95.0)) if norms.size else float("inf")}


def _json_safe(value: object) -> object:
    # validation_audit.json 是机器证据，必须递归消除 ndarray、numpy scalar、tuple 和 NaN/Inf。
    if is_dataclass(value):
        return {field.name: _json_safe(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, tuple):
        return [_json_safe(item) for item in value]
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def audit_payload(status: str, reason_codes: Iterable[str], **sections: object) -> dict:
    payload = {"schema_version": 1, "status": status, "reason_codes": sorted(set(reason_codes))}
    for key, value in sections.items():
        payload[key] = _json_safe(value)
    return _json_safe(payload)
