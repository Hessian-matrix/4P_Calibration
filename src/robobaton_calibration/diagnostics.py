"""提供单帧标定板检测诊断，隔离RTSP、画质和字典配置问题。"""

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import cv2
import numpy as np
import yaml

from robobaton_calibration.aprilgrid import AprilGridConfig, detect_aprilgrid


AprilGridDiagnosticTarget = AprilGridConfig



DIAGNOSTIC_DICTIONARIES = (
    "DICT_APRILTAG_36h11",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_16h5",
    "DICT_ARUCO_ORIGINAL",
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
    "DICT_7X7_1000",
)


@dataclass(frozen=True)
class CandidateDetection:
    dictionary: str
    marker_count: int
    marker_ids: List[int]
    rejected_count: int


def load_diagnostic_target(path: Path) -> AprilGridDiagnosticTarget:
    """离线诊断允许模板 target 只提供版型和字典；缺省几何只用于构造 AprilGrid 检测合同。"""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if data.get("target_type") != "aprilgrid":
        raise ValueError("target file must describe an aprilgrid")
    if "tag_id_x_direction" in data:
        raise ValueError("tag_id_x_direction is deprecated; use tag_corner_order")
    rows = int(data["rows"])
    cols = int(data["cols"])
    if rows <= 0 or cols <= 0:
        raise ValueError("target rows and cols must be positive")
    first_tag_id = int(data.get("first_tag_id", 0))
    if first_tag_id < 0:
        raise ValueError("target first_tag_id must be non-negative")
    # template target 允许把尺寸留空；诊断路径只需非零几何来驱动 detect_aprilgrid。
    tag_size_raw = data.get("tag_size_m")
    tag_spacing_raw = data.get("tag_spacing_ratio")
    tag_size_m = float(tag_size_raw) if tag_size_raw is not None else 0.04
    tag_spacing_ratio = float(tag_spacing_raw) if tag_spacing_raw is not None else 0.3
    dictionary = str(data.get("dictionary", "DICT_APRILTAG_36h11"))
    if not hasattr(cv2.aruco, dictionary):
        raise ValueError(f"unsupported aruco dictionary: {dictionary}")
    return AprilGridConfig(
        rows=rows,
        cols=cols,
        tag_size_m=tag_size_m,
        tag_spacing_ratio=tag_spacing_ratio,
        dictionary=dictionary,
        first_tag_id=first_tag_id,
        tag_corner_order=data.get("tag_corner_order"),
        target_id=str(data.get("target_id", Path(path).stem)),
        measured=bool(data.get("measured", False)),
    )


def load_diagnostic_gray_image(path: Path) -> np.ndarray:
    """单帧诊断必须使用和在线链路一致的mono8输入，避免彩色通道差异干扰结论。"""
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"failed to read diagnostic image: {path}")
    return image


def _detector_parameters(refine_apriltag: bool, marker_border_bits: int):
    parameters = cv2.aruco.DetectorParameters()
    # 诊断扫字典要覆盖手机拍板、RTSP压缩和大幅面板边缘情况；正式AprilGrid检测仍走aprilgrid.py严格id合同。
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 61
    parameters.adaptiveThreshWinSizeStep = 8
    parameters.minMarkerPerimeterRate = 0.01
    parameters.maxMarkerPerimeterRate = 8.0
    parameters.markerBorderBits = marker_border_bits
    # AprilTag与普通ArUco的角点细化路径不同；按字典类型切换，避免扫字典时人为放大假阴性。
    if hasattr(parameters, "cornerRefinementMethod"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG if refine_apriltag else cv2.aruco.CORNER_REFINE_SUBPIX
    return parameters


def _marker_border_bit_candidates(dictionary_name: str):
    # 实拍AprilGrid使用2-bit外黑边；OpenCV drawMarker夹具使用1-bit，诊断必须同时覆盖。
    return (2, 1) if dictionary_name.startswith("DICT_APRILTAG_") else (1,)


def _diagnostic_variants(gray: np.ndarray):
    # 单帧诊断先跑原图、直方图均衡和CLAHE三种视图，避免光照和对比度把整板tag压成少数命中。
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return (
        gray,
        cv2.equalizeHist(gray),
        clahe.apply(gray),
    )


def _detect_dictionary(image: np.ndarray, dictionary_name: str) -> CandidateDetection:
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    best_candidate: Optional[CandidateDetection] = None
    for marker_border_bits in _marker_border_bit_candidates(dictionary_name):
        parameters = _detector_parameters(dictionary_name.startswith("DICT_APRILTAG_"), marker_border_bits)
        _, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
        marker_ids = [] if ids is None else [int(marker_id) for marker_id in ids.reshape(-1)]
        candidate = CandidateDetection(
            dictionary=dictionary_name,
            marker_count=len(marker_ids),
            marker_ids=marker_ids,
            rejected_count=len(rejected),
        )
        if best_candidate is None or candidate.marker_count > best_candidate.marker_count or (
            candidate.marker_count == best_candidate.marker_count and candidate.rejected_count < best_candidate.rejected_count
        ):
            best_candidate = candidate
    if best_candidate is None:
        return CandidateDetection(dictionary=dictionary_name, marker_count=0, marker_ids=[], rejected_count=0)
    return best_candidate


def scan_marker_dictionaries(gray: np.ndarray, names: Iterable[str] = DIAGNOSTIC_DICTIONARIES) -> List[CandidateDetection]:
    """NO_DETECTION时同时扫常见AprilTag/Aruco字典，判断是图像不可检还是目标字典不一致。"""
    best_by_dictionary: Dict[str, CandidateDetection] = {}
    # 同一字典在不同预处理视图里只保留命中更多的结果，避免原图漏检把更稳的候选覆盖掉。
    for variant in _diagnostic_variants(gray):
        for name in names:
            if not hasattr(cv2.aruco, name):
                continue
            candidate = _detect_dictionary(variant, name)
            existing = best_by_dictionary.get(name)
            if existing is None or candidate.marker_count > existing.marker_count or (
                candidate.marker_count == existing.marker_count and candidate.rejected_count < existing.rejected_count
            ):
                best_by_dictionary[name] = candidate
    return sorted(best_by_dictionary.values(), key=lambda item: (-item.marker_count, item.dictionary))


def diagnose_frame_image(path: Path, target: AprilGridDiagnosticTarget) -> Dict[str, object]:
    """离线单帧诊断统一输出图像统计、目标检测结果和候选字典命中情况。"""
    gray = load_diagnostic_gray_image(path)
    detection = detect_aprilgrid(gray, target)
    candidates = scan_marker_dictionaries(gray)
    # 最佳候选回答“图像最像什么板”；配置候选用于区分“字典错”与“同字典但版型/id合同错”。
    best_candidate: Optional[CandidateDetection] = next((candidate for candidate in candidates if candidate.marker_count > 0), None)
    configured_candidate: Optional[CandidateDetection] = next(
        (candidate for candidate in candidates if candidate.dictionary == target.dictionary and candidate.marker_count > 0), None
    )
    payload: Dict[str, object] = {
        "status": "PASS" if detection.status == "DETECTED" else "FAIL",
        "image": {
            "path": str(path),
            "width": int(gray.shape[1]),
            "height": int(gray.shape[0]),
            "mean": float(gray.mean()),
            "std": float(gray.std()),
            "min": int(gray.min()),
            "max": int(gray.max()),
        },
        "target": {
            "target_id": target.target_id,
            "rows": target.rows,
            "cols": target.cols,
            "dictionary": target.dictionary,
            "first_tag_id": target.first_tag_id,
            "expected_tag_count": target.rows * target.cols,
        },
        "target_detection": {
            "status": detection.status,
            "tag_count": len(detection.tag_ids),
            "tag_ids": detection.tag_ids,
            "rejected_count": detection.rejected_count,
        },
        "candidate_dictionaries": [candidate.__dict__ for candidate in candidates if candidate.marker_count > 0],
        "best_candidate": None if best_candidate is None else best_candidate.__dict__,
    }
    if detection.status != "DETECTED" and configured_candidate is not None:
        payload["diagnosis"] = "target_layout_or_id_mismatch"
        payload["mismatch"] = {
            "configured_dictionary": target.dictionary,
            "observed_best_dictionary": configured_candidate.dictionary,
            "observed_marker_count": configured_candidate.marker_count,
            "observed_marker_ids": configured_candidate.marker_ids,
        }
    elif detection.status != "DETECTED" and best_candidate is not None:
        payload["diagnosis"] = "target_dictionary_mismatch_or_false_positive_candidate"
        payload["mismatch"] = {
            "configured_dictionary": target.dictionary,
            "observed_best_dictionary": best_candidate.dictionary,
            "observed_marker_count": best_candidate.marker_count,
            "observed_marker_ids": best_candidate.marker_ids,
        }
    elif detection.status != "DETECTED":
        payload["diagnosis"] = "target_not_detected_by_configured_dictionary"
    else:
        payload["diagnosis"] = "target_detected"
    return payload
