"""固化AprilGrid目标几何与OpenCV AprilTag检测合同。"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

TAG_CORNER_POINTS = {
    "bottom_left": (0.0, 0.0),
    "bottom_right": (1.0, 0.0),
    "top_right": (1.0, 1.0),
    "top_left": (0.0, 1.0),
}
DEFAULT_TAG_CORNER_ORDER = ("bottom_left", "bottom_right", "top_right", "top_left")


def normalize_tag_corner_order(value: Optional[Sequence[str]]) -> Tuple[str, str, str, str]:
    if value is None:
        return DEFAULT_TAG_CORNER_ORDER
    order = tuple(str(item) for item in value)
    if len(order) != 4 or set(order) != set(TAG_CORNER_POINTS):
        raise ValueError("tag_corner_order must contain bottom_left, bottom_right, top_right, top_left exactly once")
    return order


@dataclass(frozen=True)
class AprilGridConfig:
    rows: int
    cols: int
    tag_size_m: float
    tag_spacing_ratio: float
    dictionary: str = "DICT_APRILTAG_36h11"
    first_tag_id: int = 0
    target_id: str = ""
    measured: bool = True
    tag_corner_order: Tuple[str, str, str, str] = DEFAULT_TAG_CORNER_ORDER

    def __post_init__(self) -> None:
        if self.rows <= 0 or self.cols <= 0:
            raise ValueError("rows and cols must be positive")
        if self.first_tag_id < 0:
            raise ValueError("first_tag_id must be non-negative")
        if not np.isfinite(self.tag_size_m) or self.tag_size_m <= 0.0:
            raise ValueError("tag_size_m must be a measured positive value")
        if not np.isfinite(self.tag_spacing_ratio) or self.tag_spacing_ratio < 0.0:
            raise ValueError("tag_spacing_ratio must be a measured non-negative value")
        object.__setattr__(self, "tag_corner_order", normalize_tag_corner_order(self.tag_corner_order))
        if not hasattr(cv2.aruco, self.dictionary):
            raise ValueError(f"unsupported aruco dictionary: {self.dictionary}")


def load_target(path: Path) -> AprilGridConfig:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("target_type") != "aprilgrid":
        raise ValueError("target file must describe an aprilgrid")
    if data.get("measured") is not True:
        raise ValueError("target file must be measured before calibration, audit or export")
    if "tag_id_x_direction" in data:
        raise ValueError("tag_id_x_direction is deprecated; use tag_corner_order")
    for key in ("rows", "cols", "tag_size_m", "tag_spacing_ratio", "tag_corner_order"):
        if data.get(key) is None:
            raise ValueError(f"target file is missing measured value: {key}")
    return AprilGridConfig(
        rows=int(data["rows"]),
        cols=int(data["cols"]),
        tag_size_m=float(data["tag_size_m"]),
        tag_spacing_ratio=float(data["tag_spacing_ratio"]),
        dictionary=str(data.get("dictionary", "DICT_APRILTAG_36h11")),
        first_tag_id=int(data.get("first_tag_id", 0)),
        tag_corner_order=data.get("tag_corner_order"),
        target_id=str(data.get("target_id", Path(path).stem)),
        measured=bool(data.get("measured", False)),
    )


@dataclass(frozen=True)
class AprilGridDetection:
    status: str
    object_points: np.ndarray
    image_points: np.ndarray
    tag_ids: List[int]
    rejected_count: int


def object_corners_for_tag(config: AprilGridConfig, tag_id: int) -> np.ndarray:
    local_id = tag_id - config.first_tag_id
    if local_id < 0 or local_id >= config.rows * config.cols:
        raise ValueError("tag_id is outside the configured grid")
    row = local_id // config.cols
    col = local_id % config.cols
    pitch = config.tag_size_m * (1.0 + config.tag_spacing_ratio)
    x0 = col * pitch
    y0 = row * pitch

    size = config.tag_size_m
    corner_points = {
        name: (x0 + scale_x * size, y0 + scale_y * size, 0.0)
        for name, (scale_x, scale_y) in TAG_CORNER_POINTS.items()
    }
    # detectMarkers的角点0取决于tag自身旋转；板坐标原点仍固定在左下角。
    return np.array([corner_points[name] for name in config.tag_corner_order], dtype=np.float64)


def _dictionary(config: AprilGridConfig):
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.dictionary))


def _detector_parameters(marker_border_bits: int):
    parameters = cv2.aruco.DetectorParameters()
    # 实拍AprilGrid板的tag外黑边按2个border bit解码；OpenCV默认1会找到四边形但解码失败。
    parameters.markerBorderBits = marker_border_bits
    if hasattr(parameters, "cornerRefinementMethod"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    return parameters


def _refine_marker_corners(gray: np.ndarray, corners) -> list:
    """Refine decoded AprilTag corners on the luma image before calibration."""
    criteria = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.01
    if corners is None:
        return []
    refined = []
    for marker_corners in corners:
        points = np.asarray(marker_corners, dtype=np.float32).reshape(-1, 1, 2).copy()
        original = points.copy()
        try:
            candidate = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1), criteria)
        except cv2.error:
            candidate = original
        if candidate is None or not np.isfinite(candidate).all():
            candidate = original
        refined.append(candidate.reshape(1, 4, 2))
    return refined


def _marker_border_bit_candidates(config: AprilGridConfig):
    # 先匹配Kalibr/实拍AprilGrid常见2-bit黑边，再回退OpenCV drawMarker默认1-bit，兼容单marker回归夹具。
    return (2, 1) if config.dictionary.startswith("DICT_APRILTAG_") else (1,)


def _empty_detection(status: str, rejected) -> AprilGridDetection:
    return AprilGridDetection(
        status=status,
        object_points=np.empty((0, 3), dtype=np.float64),
        image_points=np.empty((0, 2), dtype=np.float64),
        tag_ids=[],
        rejected_count=len(rejected),
    )


def _decode_detected_markers(gray: np.ndarray, config: AprilGridConfig, corners, ids, rejected) -> AprilGridDetection:
    if ids is None or len(ids) == 0:
        return _empty_detection("NO_DETECTION", rejected)

    detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
    if len(detected_ids) != len(set(detected_ids)):
        return _empty_detection("DUPLICATE_TAG_ID", rejected)

    object_chunks = []
    image_chunks = []
    tag_ids = []
    height, width = gray.shape[:2]
    for marker_corners, tag_id in zip(corners, detected_ids):
        # 检测器会返回同字典但不属于目标板的tag；必须在物点映射前fail closed。
        if tag_id < config.first_tag_id or tag_id >= config.first_tag_id + config.rows * config.cols:
            return _empty_detection("UNKNOWN_TAG_ID", rejected)
        image_corners = np.asarray(marker_corners, dtype=np.float64).reshape(4, 2)
        if not np.isfinite(image_corners).all():
            return _empty_detection("NONFINITE_CORNERS", rejected)
        if (
            np.any(image_corners[:, 0] < 0.0)
            or np.any(image_corners[:, 0] >= width)
            or np.any(image_corners[:, 1] < 0.0)
            or np.any(image_corners[:, 1] >= height)
        ):
            return _empty_detection("OUT_OF_BOUNDS_CORNERS", rejected)
        object_chunks.append(object_corners_for_tag(config, tag_id))
        image_chunks.append(image_corners)
        tag_ids.append(tag_id)
    if not object_chunks:
        return _empty_detection("NO_GRID_TAGS", rejected)
    return AprilGridDetection(
        status="DETECTED",
        object_points=np.vstack(object_chunks),
        image_points=np.vstack(image_chunks),
        tag_ids=tag_ids,
        rejected_count=len(rejected),
    )


def detect_aprilgrid(image: np.ndarray, config: AprilGridConfig) -> AprilGridDetection:
    if image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    elif image.ndim == 2:
        gray = image
    else:
        raise ValueError("image must be mono8 or BGR")

    last_failure: Optional[AprilGridDetection] = None
    for marker_border_bits in _marker_border_bit_candidates(config):
        parameters = _detector_parameters(marker_border_bits)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, _dictionary(config), parameters=parameters)
        detection = _decode_detected_markers(gray, config, _refine_marker_corners(gray, corners), ids, rejected)
        if detection.status == "DETECTED":
            return detection
        if last_failure is None or detection.status != "NO_DETECTION":
            last_failure = detection
    return last_failure if last_failure is not None else _empty_detection("NO_DETECTION", [])
