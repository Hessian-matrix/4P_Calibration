"""输出双模型比较报告、选择状态和公开YAML合同。"""

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

from .double_sphere import DSParameters
from .kb4 import KB4Parameters


@dataclass(frozen=True)
class ModelMetrics:
    model: str
    train_rms_px: float
    holdout_rms_px: float
    holdout_p95_px: float
    invalid_projection_count: int


@dataclass(frozen=True)
class ModelSelection:
    selection_status: str
    selected_model: Optional[str]
    reason: str


@dataclass(frozen=True)
class ReportArtifacts:
    output_dir: Path
    selection: ModelSelection
    comparison_path: Path


def select_model(
    ds: ModelMetrics,
    kb4: ModelMetrics,
    equivalence_margin_px: float = 0.05,
    approved_policy: bool = False,
) -> ModelSelection:
    if ds.invalid_projection_count > 0 and kb4.invalid_projection_count > 0:
        return ModelSelection("NO_VALID_MODEL", None, "both models produced invalid holdout projections")
    if not approved_policy:
        return ModelSelection("UNDECIDED", None, "pre-pilot policy keeps both models as candidates")
    if abs(ds.holdout_rms_px - kb4.holdout_rms_px) <= equivalence_margin_px:
        return ModelSelection("UNDECIDED", None, "holdout RMS difference is inside the frozen equivalence band")
    if ds.holdout_rms_px < kb4.holdout_rms_px and ds.invalid_projection_count == 0:
        return ModelSelection("SELECTED", "ds", "DS holdout RMS is lower outside the equivalence band")
    if kb4.invalid_projection_count == 0:
        return ModelSelection("SELECTED", "kb4", "KB4 holdout RMS is lower outside the equivalence band")
    return ModelSelection("SELECTED", "ds", "KB4 has invalid holdout projections")


def _model_metadata(rig_id: str, camera_id: str, image_size: Tuple[int, int], metrics: ModelMetrics, source: str, max_holdout_rms_px: float, max_holdout_p95_px: float) -> dict:
    width, height = image_size
    return {
        "rig_id": rig_id,
        "camera_id": camera_id,
        "image_width": width,
        "image_height": height,
        "valid_roi": None,
        "valid_roi_status": "UNVERIFIED",
        "source": source,
        "validation": asdict(metrics),
        "validation_status": "PASS" if _metrics_pass(metrics, max_holdout_rms_px, max_holdout_p95_px) else "FAIL",
    }


def _ds_yaml(params: DSParameters, image_size: Tuple[int, int], metadata: dict = None) -> dict:
    width, height = image_size
    payload = {
        "schema_version": 1,
        "camera_model": "ds",
        "intrinsics": [params.xi, params.alpha, params.fx, params.fy, params.cx, params.cy],
        "distortion_model": "none",
        "distortion_coeffs": [],
        "resolution": [width, height],
    }
    payload.update(metadata or {})
    return payload


def _kb4_yaml(params: KB4Parameters, image_size: Tuple[int, int], metadata: dict = None) -> dict:
    width, height = image_size
    payload = {
        "schema_version": 1,
        "camera_model": "pinhole",
        "intrinsics": [params.fx, params.fy, params.cx, params.cy],
        "distortion_model": "equidistant",
        "distortion_coeffs": [params.k1, params.k2, params.k3, params.k4],
        "resolution": [width, height],
    }
    payload.update(metadata or {})
    return payload


def _metrics_pass(metrics: ModelMetrics, max_holdout_rms_px: float, max_holdout_p95_px: float) -> bool:
    return (
        metrics.invalid_projection_count == 0
        and math.isfinite(metrics.holdout_rms_px)
        and 0 <= metrics.holdout_rms_px <= max_holdout_rms_px
        and math.isfinite(metrics.holdout_p95_px)
        and 0 <= metrics.holdout_p95_px <= max_holdout_p95_px
    )


def write_reports(
    output_dir: Path,
    rig_id: str,
    camera_id: str,
    image_size: Tuple[int, int],
    ds_params: DSParameters,
    kb4_params: KB4Parameters,
    ds_metrics: ModelMetrics,
    kb4_metrics: ModelMetrics,
    decision: ModelSelection,
    max_holdout_rms_px: float = 1.0,
    max_holdout_p95_px: float = 1.0,
) -> ReportArtifacts:
    output_dir = Path(output_dir)
    models_dir = output_dir / "models"
    plots_dir = output_dir / "plots"
    models_dir.mkdir(parents=True, exist_ok=True)
    plots_dir.mkdir(parents=True, exist_ok=True)
    ds_payload = _ds_yaml(ds_params, image_size, _model_metadata(rig_id, camera_id, image_size, ds_metrics, "robobaton_calibrator", max_holdout_rms_px, max_holdout_p95_px))
    kb4_payload = _kb4_yaml(kb4_params, image_size, _model_metadata(rig_id, camera_id, image_size, kb4_metrics, "robobaton_calibrator", max_holdout_rms_px, max_holdout_p95_px))
    (models_dir / "ds.yaml").write_text(yaml.safe_dump(ds_payload, sort_keys=False), encoding="utf-8")
    (models_dir / "kb4.yaml").write_text(yaml.safe_dump(kb4_payload, sort_keys=False), encoding="utf-8")
    selected_model_path = output_dir / "selected_model.yaml"
    selected_payload = {"schema_version": 1, "policy_version": "pre-pilot-v1", "reason_codes": [decision.selection_status], **asdict(decision)}
    selected_model_path.write_text(yaml.safe_dump(selected_payload, sort_keys=False), encoding="utf-8")
    if decision.selection_status == "SELECTED" and decision.selected_model:
        selected_source = models_dir / f"{decision.selected_model}.yaml"
        (output_dir / "calibration.yaml").write_text(selected_source.read_text(encoding="utf-8"), encoding="utf-8")

    comparison = {
        "schema_version": 1,
        "rig_id": rig_id,
        "camera_id": camera_id,
        "image_size": list(image_size),
        "models": {"ds": asdict(ds_metrics), "kb4": asdict(kb4_metrics)},
        "selection": asdict(decision),
    }
    comparison_path = output_dir / "model_comparison.json"
    comparison_path.write_text(json.dumps(comparison, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    validation = {
        "schema_version": 1,
        "status": "PASS" if all(_metrics_pass(metric, max_holdout_rms_px, max_holdout_p95_px) for metric in (ds_metrics, kb4_metrics)) else "FAIL",
        "thresholds": {"max_holdout_rms_px": max_holdout_rms_px, "max_holdout_p95_px": max_holdout_p95_px},
        "models": {"ds": asdict(ds_metrics), "kb4": asdict(kb4_metrics)},
    }
    (output_dir / "validation.json").write_text(json.dumps(validation, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    _write_plot(plots_dir / "residual_summary.png", plots_dir / "residual_summary.pdf", ds_metrics, kb4_metrics)
    _write_report_pdf(output_dir)
    _write_markdown(output_dir / "report.md", comparison)
    _write_html(output_dir / "report.html", comparison)
    return ReportArtifacts(output_dir=output_dir, selection=decision, comparison_path=comparison_path)


def _write_plot(png_path: Path, pdf_path: Path, ds: ModelMetrics, kb4: ModelMetrics) -> None:
    labels = ["DS train", "DS holdout", "KB4 train", "KB4 holdout"]
    values = [ds.train_rms_px, ds.holdout_rms_px, kb4.train_rms_px, kb4.holdout_rms_px]
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.bar(labels, values, color=["#4c78a8", "#72b7b2", "#f58518", "#ffbf79"])
    ax.set_ylabel("RMS px")
    ax.set_title("Robobaton intrinsic calibration residual summary")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path)
    fig.savefig(pdf_path)
    plt.close(fig)


def _write_report_pdf(output_dir: Path) -> None:
    report_fig, report_ax = plt.subplots(figsize=(8.27, 11.69))
    report_ax.axis("off")
    report_ax.text(0.05, 0.95, "Robobaton calibration report\n\nSee report.md and validation.json for numeric metrics.", va="top")
    report_fig.savefig(output_dir / "report.pdf")
    plt.close(report_fig)


def _write_markdown(path: Path, comparison: dict) -> None:
    ds = comparison["models"]["ds"]
    kb4 = comparison["models"]["kb4"]
    selection = comparison["selection"]
    text = f"""# Robobaton Camera Calibration Report

- Rig: `{comparison['rig_id']}`
- Camera: `{comparison['camera_id']}`
- Image size: `{comparison['image_size'][0]}x{comparison['image_size'][1]}`
- Selection status: `{selection['selection_status']}`
- Selected model: `{selection['selected_model']}`
- Reason: {selection['reason']}

| Model | Train RMS px | Holdout RMS px | Holdout P95 px | Invalid projections |
|---|---:|---:|---:|---:|
| DS | {ds['train_rms_px']:.6f} | {ds['holdout_rms_px']:.6f} | {ds['holdout_p95_px']:.6f} | {ds['invalid_projection_count']} |
| KB4 | {kb4['train_rms_px']:.6f} | {kb4['holdout_rms_px']:.6f} | {kb4['holdout_p95_px']:.6f} | {kb4['invalid_projection_count']} |
"""
    path.write_text(text, encoding="utf-8")


def _write_html(path: Path, comparison: dict) -> None:
    markdown = (path.parent / "report.md").read_text(encoding="utf-8")
    escaped = markdown.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    html = f"<!doctype html><meta charset='utf-8'><title>Calibration Report</title><pre>{escaped}</pre>\n"
    path.write_text(html, encoding="utf-8")
