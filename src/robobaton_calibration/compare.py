"""复核Robobaton与外部标定结果的文件级比较门禁。"""

import argparse
import json
import math
from pathlib import Path

import yaml


def _load_structured(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        return json.loads(text)
    return yaml.safe_load(text) or {}


def _camera_payload(payload: dict) -> dict:
    if "cam0" in payload and isinstance(payload["cam0"], dict):
        return payload["cam0"]
    return payload


def _compare_external(name: str, robobaton: dict, external_path: Path, failures: list[str]) -> None:
    if not external_path.exists():
        failures.append(f"missing external report: {external_path}")
        return
    external = _camera_payload(_load_structured(external_path))
    if not external or not external.get("resolution") or not external.get("camera_model"):
        failures.append(f"{name} external report lacks camera model or resolution")
        return
    expected_resolution = external.get("resolution")
    if expected_resolution and list(expected_resolution) != robobaton.get("resolution"):
        failures.append(f"{name} external resolution mismatch: {expected_resolution} != {robobaton.get('resolution')}")
    expected_model = "ds" if name == "ds" else "pinhole"
    if external.get("camera_model") and external.get("camera_model") != expected_model:
        failures.append(f"{name} external camera_model mismatch: {external.get('camera_model')} != {expected_model}")
    if "holdout_rms_px" in external:
        rms = float(external["holdout_rms_px"])
        if not math.isfinite(rms) or rms < 0 or rms > float(robobaton.get("validation", {}).get("holdout_rms_px", float("inf"))) + 1.0:
            failures.append(f"{name} external holdout_rms_px is invalid or diverges from Robobaton result")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Compare Robobaton calibration report with optional external results")
    parser.add_argument("--robobaton-report", required=True, type=Path)
    parser.add_argument("--max-holdout-rms-px", type=float, default=1.0)
    parser.add_argument("--max-holdout-p95-px", type=float, default=1.0)
    parser.add_argument("--kalibr-ds-report", type=Path)
    parser.add_argument("--kalibr-kb4-report", type=Path)
    args = parser.parse_args(argv)
    if any(not math.isfinite(value) or value <= 0 for value in (args.max_holdout_rms_px, args.max_holdout_p95_px)):
        parser.error("holdout thresholds must be finite and positive")
    comparison = json.loads(args.robobaton_report.read_text(encoding="utf-8"))
    run_dir = args.robobaton_report.parent
    failures = []
    model_payloads = {}
    models = comparison.get("models", {})
    if not isinstance(models, dict) or set(models) != {"ds", "kb4"}:
        print("CALIBRATION_COMPARISON_RESULT FAIL missing ds/kb4 models")
        return 1
    for name, metrics in models.items():
        for key, limit in (("holdout_rms_px", args.max_holdout_rms_px), ("holdout_p95_px", args.max_holdout_p95_px)):
            value = float(metrics.get(key, float("inf")))
            if not math.isfinite(value) or not 0 <= value <= limit:
                failures.append(f"{name} {key} is invalid or exceeds {limit}")
        if metrics.get("invalid_projection_count") != 0:
            failures.append(f"{name} has invalid or missing projection count")
        model_path = run_dir / "models" / f"{name}.yaml"
        if model_path.exists():
            model_payloads[name] = yaml.safe_load(model_path.read_text(encoding="utf-8")) or {}
        else:
            failures.append(f"missing model: {model_path}")
    if args.kalibr_ds_report is not None:
        _compare_external("ds", model_payloads.get("ds", {}), args.kalibr_ds_report, failures)
    if args.kalibr_kb4_report is not None:
        _compare_external("kb4", model_payloads.get("kb4", {}), args.kalibr_kb4_report, failures)
    if failures:
        print("CALIBRATION_COMPARISON_RESULT FAIL")
        for failure in failures:
            print(f"- {failure}")
        return 1
    print("CALIBRATION_COMPARISON_RESULT PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
