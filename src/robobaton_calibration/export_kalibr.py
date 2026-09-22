"""Export intrinsic-calibration images while keeping holdout out of optimization."""

import argparse
import json
import shutil
from pathlib import Path

import yaml

from .aprilgrid import load_target
from .dataset import verify_manifest


def _frame_split(frame: dict) -> str:
    metadata = frame.get("metadata") or {}
    if metadata.get("split"):
        return str(metadata["split"])
    parts = Path(str(frame.get("path", ""))).parts
    return parts[1] if len(parts) >= 2 and parts[0] == "dataset" else ""


def export_dataset(manifest_path: Path, target_path: Path, output_dir: Path) -> dict:
    if not verify_manifest(manifest_path):
        raise ValueError("manifest hash or saved frame verification failed")
    target = load_target(target_path)
    if target.dictionary != "DICT_APRILTAG_36h11" or target.first_tag_id != 0:
        raise ValueError("Kalibr export requires AprilTag 36h11 with IDs starting at zero")
    if output_dir.exists() and any(output_dir.iterdir()):
        raise FileExistsError(f"output directory must be new or empty: {output_dir}")
    run_dir = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    selected = {"train": [], "holdout": []}
    indices = set()
    for frame in manifest.get("frames", []):
        split = _frame_split(frame)
        metadata = frame.get("metadata") or {}
        if split not in selected or not frame.get("accepted", True) or metadata.get("classification") in {"COVERAGE_ONLY", "REJECTED"}:
            continue
        if not frame.get("path"):
            raise ValueError("export requires saved PNG frames; capture with --save-data accepted or all")
        index = int(frame["index"])
        if index < 0 or index in indices:
            raise ValueError("export requires unique nonnegative frame indices")
        indices.add(index)
        selected[split].append(frame)
    if not selected["train"]:
        raise ValueError("no saved training frames to export")
    models = {}
    for name in ("ds", "kb4"):
        payload = yaml.safe_load((run_dir / "models" / f"{name}.yaml").read_text(encoding="utf-8"))
        models[name] = {key: payload[key] for key in ("camera_model", "intrinsics", "distortion_model", "distortion_coeffs", "resolution")}
        models[name]["rostopic"] = "/cam0/image_raw"
    output_dir.mkdir(parents=True, exist_ok=True)
    for split, frames in selected.items():
        base = output_dir if split == "train" else output_dir / "holdout"
        data_dir = base / "cam0" / "data"
        data_dir.mkdir(parents=True, exist_ok=True)
        image_lines = []
        for frame in sorted(frames, key=lambda item: int(item["index"])):
            # Ordering only: these timestamps are not camera or IMU measurements.
            filename = f"{int(frame['index']) * 33_333_333:018d}.png"
            destination = data_dir / filename
            shutil.copyfile(run_dir / frame["path"], destination)
            image_lines.append(str(destination.relative_to(base)))
        (base / "images.txt").write_text("".join(line + "\n" for line in image_lines), encoding="utf-8")
    for name, filename in (("ds", "kalibr_ds_none.yaml"), ("kb4", "kalibr_pinhole_equi.yaml")):
        (output_dir / filename).write_text(yaml.safe_dump({"cam0": models[name]}, sort_keys=False), encoding="utf-8")
    kalibr_target = {
        "target_type": "aprilgrid", "tagRows": target.rows, "tagCols": target.cols,
        "tagSize": target.tag_size_m, "tagSpacing": target.tag_spacing_ratio,
    }
    (output_dir / "target.yaml").write_text(yaml.safe_dump(kalibr_target, sort_keys=False), encoding="utf-8")
    shutil.copyfile(target_path, output_dir / "source_target.yaml")
    metadata = {
        "source_manifest_sha256": manifest["manifest_sha256"],
        "train_frames": len(selected["train"]), "holdout_frames": len(selected["holdout"]),
        "timestamps": "synthetic frame-index ordering at 30 Hz; not acquisition timestamps",
        "scope": "single-camera intrinsics only; do not use for time or camera-IMU calibration",
        "holdout_directory": "holdout", "optimization_directory": "cam0",
        "target_note": "Check printed tag orientation against Kalibr; source_target.yaml retains the original corner order.",
    }
    (output_dir / "export_metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Export Kalibr training images, separate holdout and measured AprilGrid geometry")
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--target", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        result = export_dataset(args.manifest, args.target, args.output_dir)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"KALIBR_EXPORT_RESULT FAIL error={exc}")
        return 1
    print(f"KALIBR_EXPORT_RESULT PASS output_dir={args.output_dir} train_frames={result['train_frames']} holdout_frames={result['holdout_frames']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
