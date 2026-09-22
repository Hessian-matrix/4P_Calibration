#!/usr/bin/env python3
"""校准run目录的host-only离线审计入口，不执行重新标定。"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from robobaton_calibration.aprilgrid import detect_aprilgrid, load_target
from robobaton_calibration.dataset import verify_manifest
from robobaton_calibration.double_sphere import DSParameters
from robobaton_calibration.kb4 import KB4Parameters
from robobaton_calibration.solver import Observation
from robobaton_calibration.validation import AuditConfig, audit_payload, compute_coverage, cross_tag_validation_records, dense_domain_audit, evaluate_holdout_records, ray_angle_comparison, residual_summary, write_undistort_artifacts, write_validation_plots


def ensure_output_dir(output_dir: Path) -> Path:
    output_dir = Path(output_dir)
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output dir must be new or empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def load_audit_config(path: Path) -> AuditConfig:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    fields = {name: data[name] for name in AuditConfig.__dataclass_fields__ if name in data}
    return AuditConfig(**fields)


def load_ds_yaml(path: Path) -> DSParameters:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    intrinsics = data["intrinsics"]
    return DSParameters(fx=float(intrinsics[2]), fy=float(intrinsics[3]), cx=float(intrinsics[4]), cy=float(intrinsics[5]), xi=float(intrinsics[0]), alpha=float(intrinsics[1]))


def load_kb4_yaml(path: Path) -> KB4Parameters:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    fx, fy, cx, cy = [float(v) for v in data["intrinsics"]]
    coeffs = [float(v) for v in data.get("distortion_coeffs", [0.0, 0.0, 0.0, 0.0])]
    return KB4Parameters(fx=fx, fy=fy, cx=cx, cy=cy, k1=coeffs[0], k2=coeffs[1], k3=coeffs[2], k4=coeffs[3])


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def manifest_frame_split(frame: dict) -> str:
    """新manifest使用metadata.split，旧manifest按dataset目录回退，避免rejected误算accepted。"""
    metadata = frame.get("metadata") if isinstance(frame.get("metadata"), dict) else {}
    split = metadata.get("split")
    if split:
        return str(split)
    parts = Path(str(frame.get("path", ""))).parts
    if len(parts) >= 2 and parts[0] == "dataset":
        return parts[1]
    return "accepted"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate an existing Robobaton DS/KB4 calibration run. Final line: CALIBRATION_AUDIT_RESULT PASS|FAIL output_dir=...",
    )
    parser.add_argument("--run-dir", required=True, help="existing calibration run directory; read-only input")
    parser.add_argument("--target", required=True, help="AprilGrid target YAML used for re-detection")
    parser.add_argument("--session-config", required=True, help="online intrinsic session config with audit candidate gates")
    parser.add_argument("--output-dir", required=True, help="new or empty output directory for audit artifacts")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    run_dir = Path(args.run_dir)
    output_dir = ensure_output_dir(Path(args.output_dir))
    config = load_audit_config(Path(args.session_config))
    reason_codes = []
    try:
        if not verify_manifest(run_dir / "manifest.json"):
            raise ValueError("manifest hash verification failed")
        target = load_target(Path(args.target))
        ds = load_ds_yaml(run_dir / "models" / "ds.yaml")
        kb4 = load_kb4_yaml(run_dir / "models" / "kb4.yaml")
        manifest_path = run_dir / "manifest.json"
        target_path = Path(args.target)
        session_config_path = Path(args.session_config)
        ds_yaml_path = run_dir / "models" / "ds.yaml"
        kb4_yaml_path = run_dir / "models" / "kb4.yaml"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        detections = []
        observations_by_split = {"train": [], "holdout": [], "accepted": []}
        tag_ids_by_frame = {}
        seen_splits = set()
        split_frame_counts = {}
        image_size = (int(manifest.get("extra", {}).get("image_width", 1280)), int(manifest.get("extra", {}).get("image_height", 1088)))
        input_identity = {
            "source_run_path": str(run_dir),
            "manifest_sha256": sha256_file(manifest_path),
            "target_sha256": sha256_file(target_path),
            "session_config_sha256": sha256_file(session_config_path),
            "ds_yaml_sha256": sha256_file(ds_yaml_path) if ds_yaml_path.exists() else None,
            "kb4_yaml_sha256": sha256_file(kb4_yaml_path) if kb4_yaml_path.exists() else None,
            "image_size": image_size,
            "total_saved_frames": len(manifest.get("frames", [])),
            "split_frame_counts": {},
        }
        # 离线审计必须重新检测每张保存帧，并把 split/tag_id 证据固定到输出目录。
        for frame in manifest.get("frames", []):
            split = manifest_frame_split(frame)
            seen_splits.add(split)
            split_frame_counts[split] = split_frame_counts.get(split, 0) + 1
            path = run_dir / str(frame.get("path"))
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise ValueError(f"failed to reload saved frame: {path}")
            if tuple(gray.shape[:2]) != (image_size[1], image_size[0]):
                raise ValueError(f"saved frame shape mismatch: {path} shape={gray.shape[:2]} expected={(image_size[1], image_size[0])}")
            detection = detect_aprilgrid(gray, target)
            detections.append(
                {
                    "frame_index": int(frame["index"]),
                    "split": split,
                    "tag_ids": detection.tag_ids,
                    "status": detection.status,
                    "object_points": detection.object_points.tolist(),
                    "image_points": detection.image_points.tolist(),
                }
            )
            if detection.status != "DETECTED":
                reason_codes.append("REDETECTION_FAILED")
            # rejected/coverage-only保存帧只参与可追溯重检测，不进入train/holdout误差审计。
            elif split in observations_by_split:
                observations_by_split[split].append(Observation(detection.object_points, detection.image_points, index=int(frame["index"])))
                tag_ids_by_frame[int(frame["index"])] = np.repeat(np.asarray(detection.tag_ids, dtype=np.int64), 4)
        input_identity["split_frame_counts"] = dict(sorted(split_frame_counts.items()))
        (output_dir / "detections.jsonl").write_text("\n".join(json.dumps(item, ensure_ascii=False, sort_keys=True) for item in detections) + ("\n" if detections else ""), encoding="utf-8")
        first_frame = next((frame for frame in manifest.get("frames", []) if frame.get("path")), None)
        if first_frame is None:
            raise ValueError("manifest contains no saved frames")
        first_image = cv2.imread(str(run_dir / str(first_frame["path"])), cv2.IMREAD_GRAYSCALE)
        if first_image is None:
            raise ValueError("failed to load frame for undistort artifacts")
        coverage = compute_coverage(observations_by_split.get("train", []) + observations_by_split.get("holdout", []) + observations_by_split.get("accepted", []), image_size, config.audit_grid_cols, config.audit_grid_rows, config.audit_min_grid_frames, config.audit_min_edge_frames, config.audit_min_corner_frames)
        holdout_observations = observations_by_split.get("holdout", [])
        accepted_observations = observations_by_split.get("accepted", [])
        if "holdout" not in seen_splits:
            reason_codes.append("MISSING_HOLDOUT_SPLIT")
        if not holdout_observations:
            reason_codes.append("EMPTY_OFFICIAL_HOLDOUT")
        if not accepted_observations:
            reason_codes.append("EMPTY_ACCEPTED_EVIDENCE")
        holdout_records_ds, holdout_invalid_ds = evaluate_holdout_records("ds", ds, holdout_observations, image_size, "holdout")
        accepted_records_ds, accepted_invalid_ds = evaluate_holdout_records("ds", ds, accepted_observations, image_size, "accepted")
        cross_records_ds, cross_invalid_ds = cross_tag_validation_records("ds", ds, holdout_observations, image_size, tag_ids_by_frame=tag_ids_by_frame, first_tag_id=target.first_tag_id, target_cols=target.cols)
        holdout_records_kb4, holdout_invalid_kb4 = evaluate_holdout_records("kb4", kb4, holdout_observations, image_size, "holdout")
        accepted_records_kb4, accepted_invalid_kb4 = evaluate_holdout_records("kb4", kb4, accepted_observations, image_size, "accepted")
        cross_records_kb4, cross_invalid_kb4 = cross_tag_validation_records("kb4", kb4, holdout_observations, image_size, tag_ids_by_frame=tag_ids_by_frame, first_tag_id=target.first_tag_id, target_cols=target.cols)
        ds_domain = dense_domain_audit("ds", ds, image_size, config.audit_dense_grid_cols, config.audit_dense_grid_rows, config.audit_max_roundtrip_px)
        kb4_domain = dense_domain_audit("kb4", kb4, image_size, config.audit_dense_grid_cols, config.audit_dense_grid_rows, config.audit_max_roundtrip_px)
        comparison = ray_angle_comparison(ds, kb4, image_size, config.audit_dense_grid_cols, config.audit_dense_grid_rows, config.audit_max_roundtrip_px)
        residual_records = holdout_records_ds + accepted_records_ds + cross_records_ds + holdout_records_kb4 + accepted_records_kb4 + cross_records_kb4
        write_validation_plots(output_dir, coverage, ds_domain, kb4_domain, comparison, residual_records)
        write_undistort_artifacts(output_dir, first_image, ds, kb4, image_size, config.audit_virtual_pinhole_fov_deg)
        if coverage.failed_cells:
            reason_codes.append("COVERAGE_GAP")
        if coverage.failed_zones:
            reason_codes.append("ZONE_GAP")
        if not ds_domain.full_frame_roi_claimed:
            reason_codes.append("DS_DENSE_DOMAIN_FAIL")
        if not kb4_domain.full_frame_roi_claimed:
            reason_codes.append("KB4_DENSE_DOMAIN_FAIL")
        # 每个模型和 split 独立汇总，避免 holdout/cross-tag/accepted/outer 指标互相污染。
        model_summaries = {
            "ds": {
                "official_holdout": residual_summary(holdout_records_ds),
                "holdout_cross_tag": residual_summary(cross_records_ds),
                "accepted_evidence": residual_summary(accepted_records_ds),
                "accepted_outer_evidence": residual_summary(accepted_records_ds, image_size, config.audit_outer_radius_start),
                "invalid_counts": {"holdout": holdout_invalid_ds, "holdout_cross_tag": cross_invalid_ds, "accepted": accepted_invalid_ds},
            },
            "kb4": {
                "official_holdout": residual_summary(holdout_records_kb4),
                "holdout_cross_tag": residual_summary(cross_records_kb4),
                "accepted_evidence": residual_summary(accepted_records_kb4),
                "accepted_outer_evidence": residual_summary(accepted_records_kb4, image_size, config.audit_outer_radius_start),
                "invalid_counts": {"holdout": holdout_invalid_kb4, "holdout_cross_tag": cross_invalid_kb4, "accepted": accepted_invalid_kb4},
            },
        }
        for model_name, summary in model_summaries.items():
            official = summary["official_holdout"]
            cross = summary["holdout_cross_tag"]
            if official["status"] == "EMPTY":
                reason_codes.append(f"{model_name.upper()}_EMPTY_OFFICIAL_HOLDOUT")
            else:
                if official["rms_px"] > config.max_holdout_rms_px:
                    reason_codes.append(f"{model_name.upper()}_OFFICIAL_HOLDOUT_RMS_FAIL")
                if official["p95_px"] > config.max_holdout_p95_px:
                    reason_codes.append(f"{model_name.upper()}_OFFICIAL_HOLDOUT_P95_FAIL")
            if cross["status"] == "EMPTY":
                reason_codes.append(f"{model_name.upper()}_EMPTY_HOLDOUT_CROSS_TAG")
            else:
                if cross["rms_px"] > config.max_holdout_rms_px:
                    reason_codes.append(f"{model_name.upper()}_HOLDOUT_CROSS_TAG_RMS_FAIL")
                if cross["p95_px"] > config.max_holdout_p95_px:
                    reason_codes.append(f"{model_name.upper()}_HOLDOUT_CROSS_TAG_P95_FAIL")
            outer = summary["accepted_outer_evidence"]
            if outer["status"] == "EMPTY":
                reason_codes.append(f"{model_name.upper()}_EMPTY_ACCEPTED_OUTER_EVIDENCE")
            elif outer["p95_px"] > config.audit_max_outer_p95_px:
                reason_codes.append(f"{model_name.upper()}_OUTER_P95_FAIL")
            if any(int(value) > 0 for value in summary["invalid_counts"].values()):
                reason_codes.append(f"{model_name.upper()}_INVALID_EVALUATION_POINT")
        status = "FAIL" if reason_codes else "PASS"
        payload = audit_payload(
            status,
            reason_codes,
            coverage=coverage,
            input_identity=input_identity,
            models=model_summaries,
            dense_domain={
                "ds": ds_domain,
                "kb4": kb4_domain,
            },
            ray_angle={
                "mutually_valid_count": comparison.mutually_valid_count,
                "mean_angle_deg": comparison.mean_angle_deg,
                "p95_angle_deg": comparison.p95_angle_deg,
                "max_angle_deg": comparison.max_angle_deg,
            },
            note="Existing model audit; quality checks do not approve production model selection.",
        )
    except Exception as exc:
        status = "FAIL"
        payload = audit_payload(status, ["AUDIT_EXCEPTION"], error=str(exc))
    (output_dir / "validation_audit.json").write_text(json.dumps(payload, ensure_ascii=False, allow_nan=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"CALIBRATION_AUDIT_RESULT {status} output_dir={output_dir}")
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
