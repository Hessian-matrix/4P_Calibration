"""持久化标定会话帧、元数据和可复核hash。"""

import hashlib
import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class FrameRecord:
    index: int
    accepted: bool
    path: Optional[str]
    sha256: Optional[str]
    metadata: Dict[str, object]


class DatasetWriter:
    def __init__(self, output_dir: Path, save_mode: str = "accepted") -> None:
        if save_mode not in {"off", "accepted", "all"}:
            raise ValueError("save_mode must be off, accepted, or all")
        self.output_dir = Path(output_dir)
        if self.output_dir.exists() and any(self.output_dir.iterdir()):
            raise ValueError(f"output directory already exists and is not empty: {self.output_dir}")
        self.save_mode = save_mode
        self.records: List[FrameRecord] = []

    def record_frame(
        self,
        gray: np.ndarray,
        index: int,
        accepted: bool,
        metadata: Optional[Dict[str, object]] = None,
        split: str = "accepted",
    ) -> None:
        metadata = dict(metadata or {})
        safe_split = split if accepted else "rejected"
        if safe_split not in {"train", "holdout", "accepted", "rejected"}:
            raise ValueError("dataset split must be train, holdout, accepted, or rejected")
        # manifest split 是离线审计和外部导出的合同字段，不能只从路径推断。
        metadata["split"] = safe_split
        should_save = self.save_mode == "all" or (self.save_mode == "accepted" and accepted)
        relative_path = None
        digest = None
        if should_save:
            frame_dir = self.output_dir / "dataset" / safe_split
            frame_dir.mkdir(parents=True, exist_ok=True)
            relative_path = f"dataset/{safe_split}/frame_{index:06d}.png"
            absolute_path = self.output_dir / relative_path
            image = np.asarray(gray, dtype=np.uint8)
            if image.ndim != 2:
                raise ValueError("dataset frames must be mono8")
            if not cv2.imwrite(str(absolute_path), image):
                raise RuntimeError(f"failed to write frame: {absolute_path}")
            digest = hashlib.sha256(absolute_path.read_bytes()).hexdigest()
        self.records.append(
            FrameRecord(index=index, accepted=accepted, path=relative_path, sha256=digest, metadata=metadata)
        )

    def finalize(self, extra: Optional[Dict[str, object]] = None) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        manifest = {
            "schema_version": 1,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "save_mode": self.save_mode,
            "dataset_layout": "dataset/<train|holdout|accepted|rejected>/frame_XXXXXX.png",
            "extra": dict(extra or {}),
            "frames": [record.__dict__ for record in self.records if record.path is not None or self.save_mode == "off"],
            "manifest_sha256": "",
        }
        manifest["manifest_sha256"] = _manifest_digest(manifest)
        path = self.output_dir / "manifest.json"
        path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path


def _manifest_digest(manifest: Dict[str, object]) -> str:
    stable = dict(manifest)
    stable["manifest_sha256"] = ""
    payload = json.dumps(stable, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def verify_manifest(manifest_path: Path) -> bool:
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected = manifest.get("manifest_sha256")
    if expected != _manifest_digest(manifest):
        return False
    root = manifest_path.parent
    for frame in manifest.get("frames", []):
        relative_path = frame.get("path")
        digest = frame.get("sha256")
        if not relative_path:
            continue
        path = root / relative_path
        # manifest只允许相对会话目录,防止复查工具跟随路径逃逸。
        if path.resolve().is_relative_to(root.resolve()) is False:
            return False
        if not path.exists() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            return False
    return True
