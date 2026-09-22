"""提供主机端 Robobaton 相机内参标定入口。"""

import argparse
import multiprocessing as mp
import importlib
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from importlib.resources import files
from types import SimpleNamespace
from typing import Callable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import yaml

from robobaton_calibration import __version__
from robobaton_calibration.aprilgrid import AprilGridDetection, detect_aprilgrid, load_target
from robobaton_calibration.dataset import DatasetWriter
from robobaton_calibration.diagnostics import diagnose_frame_image, load_diagnostic_target
from robobaton_calibration.double_sphere import DSParameters
from robobaton_calibration.frame_source import RTSPFrameSource, RawTcpFrameSource, redact_url
from robobaton_calibration.kb4 import KB4Parameters
from robobaton_calibration.quality import CaptureAssistant, QualityThresholds, classify_frame_quality, select_representative_observations
from robobaton_calibration.reporting import ModelMetrics, select_model, write_reports
from robobaton_calibration.session import CalibrationSession, SessionConfig
from robobaton_calibration.solver import Observation, evaluate_fixed_intrinsics_holdout, residual_metrics, solve_double_sphere, solve_kb4
from robobaton_calibration.validation import capture_config_from_mapping

CONFIG_DIR = files("robobaton_calibration").joinpath("configs")
DEFAULT_IMAGE_SIZE = (1280, 1088)


def _positive_int(text: str) -> int:
    """CLI节流参数必须提前拒绝0和负数，避免运行期取模崩溃。"""
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be a positive integer")
    return value


def _nonnegative_int(text: str) -> int:
    """交互RTSP采集允许0表示不限制帧数，负数必须在解析阶段拒绝。"""
    value = int(text)
    if value < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return value


def _positive_float(text: str) -> float:
    """预览缩放比例必须为正，避免生成0尺寸OpenCV窗口帧。"""
    value = float(text)
    if not np.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def _parse_camera_index(text: str) -> int:
    """raw TCP 协议需要数值 0..3 相机编号；接受 cam0..cam3 与 0..3。"""
    normalized = str(text).strip()
    if normalized.lower().startswith("cam"):
        normalized = normalized[3:]
    try:
        value = int(normalized)
    except ValueError:
        raise ValueError(f"camera-id must encode a 0..3 camera index, got {text!r}") from None
    if not 0 <= value <= 3:
        raise ValueError(f"camera-id must be 0..3, got {value}")
    return value


def _parse_raw_tcp(text: str) -> Tuple[str, int]:
    """HOST:PORT 解析只接受 IPv4/主机名与十进制端口，拒绝空主机与越界端口。"""
    host, sep, port_text = text.rpartition(":")
    if not sep or not host:
        raise ValueError(f"raw-tcp must be HOST:PORT, got {text!r}")
    if not port_text.isdigit():
        raise ValueError(f"raw-tcp port must be a decimal integer, got {port_text!r}")
    port = int(port_text)
    if not 1 <= port <= 65535:
        raise ValueError(f"raw-tcp port must be 1..65535, got {port}")
    return host, port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robobaton mono8 DS/KB4 intrinsic calibrator")
    parser.add_argument("--version", action="store_true", help="print calibration tool version")
    parser.add_argument("--self-check", action="store_true", help="check host dependencies without creating session output")
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON for self-check or image diagnostics")
    parser.add_argument("--rtsp-url", help="single camera RTSP URL")
    parser.add_argument("--raw-tcp", help="board raw TCP server address as HOST:PORT; mutually exclusive with --rtsp-url")
    parser.add_argument("--camera-id", default="cam0", help="camera identifier, for example cam0 or 0")
    parser.add_argument("--rig-id", default="robobaton_4p", help="rig identifier written to reports")
    parser.add_argument("--session-config", type=Path, default=CONFIG_DIR / "online_intrinsic_v1.yaml")
    parser.add_argument("--target", type=Path, default=CONFIG_DIR / "targets/aprilgrid_6x6_36h11_template.yaml")
    parser.add_argument("--save-data", choices=["off", "accepted", "all"], default="accepted")
    parser.add_argument("--preview-window", action="store_true", help="show an opt-in OpenCV preview window with AprilGrid overlay and s/h/f/r/q hotkeys")
    parser.add_argument("--preview-scale", type=_positive_float, default=1.0, help="scale factor for the preview window only; calibration and saved frames stay full resolution")
    parser.add_argument("--preview-detect-every-n", type=_positive_int, default=30, help="run AprilGrid detection once every N preview frames; capture mode detects each consumed latest frame")
    parser.add_argument("--save-debug-overlays", action="store_true", help="save opt-in annotated preview PNGs under output_dir/debug_overlays")
    parser.add_argument("--debug-overlay-every-n", type=_positive_int, default=1, help="save one debug overlay every N decoded frames when --save-debug-overlays is enabled")
    parser.add_argument("--output-root", type=Path, default=Path("calibration_runs"))
    parser.add_argument("--output-dir", type=Path, help="exact output directory for this run")
    parser.add_argument("--diagnose-image", type=Path, help="run one-frame detector diagnostics on a saved mono/BGR image and exit")
    parser.add_argument("--max-frames", type=_nonnegative_int, default=0, help="maximum processed RTSP frames before stopping; 0 disables the interactive frame cap")
    return parser


def main(argv: Sequence[str] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.version:
        print(f"robobaton-camera-calibrator {__version__}")
        return 0
    if args.self_check:
        return _cmd_self_check(args.json)
    if args.diagnose_image:
        return _cmd_diagnose_image(args)
    if args.raw_tcp and args.rtsp_url:
        print("--raw-tcp and --rtsp-url are mutually exclusive", file=sys.stderr)
        return 2
    if not args.raw_tcp and not args.rtsp_url:
        print("--rtsp-url or --raw-tcp is required for interactive capture", file=sys.stderr)
        return 2
    if args.raw_tcp:
        try:
            _parse_raw_tcp(args.raw_tcp)
            _parse_camera_index(args.camera_id)
        except ValueError as exc:
            print(f"invalid argument: {exc}", file=sys.stderr)
            return 2
    try:
        return run_interactive_session(args)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"CALIBRATION_RESULT FAIL error={exc}", file=sys.stderr)
        return 1


def _cmd_self_check(json_output: bool) -> int:
    dependency_names = ["cv2", "numpy", "scipy", "yaml", "matplotlib"]
    dependencies = {name: _module_status(name) for name in dependency_names}
    dependencies["ffmpeg"] = _binary_status("ffmpeg")
    dependencies["ffprobe"] = _binary_status("ffprobe")
    dependencies["cv2_aruco_apriltag_36h11"] = {
        "available": hasattr(cv2, "aruco") and hasattr(cv2.aruco, "DICT_APRILTAG_36h11"),
        "version": str(getattr(cv2, "__version__", "unknown")),
    }
    payload = {"status": "PASS", "dependencies": dependencies, "tool_version": __version__}
    if any(not item["available"] for item in dependencies.values()):
        payload["status"] = "FAIL"
    if json_output:
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    else:
        print(f"CALIBRATOR_SELF_CHECK {payload['status']}")
        for name, item in sorted(dependencies.items()):
            print(f"{name}: {'OK' if item['available'] else 'MISSING'} {item.get('version', '')}")
    return 0 if payload["status"] == "PASS" else 1

def _cmd_diagnose_image(args) -> int:
    """单帧诊断路径不打开RTSP、不创建session输出，用于快速定位标定板字典/画质问题。"""
    # 诊断模式只需要版型和字典信息；允许读取模板target，避免把离线排查绑死到实测尺寸。
    target = load_diagnostic_target(args.target)
    payload = diagnose_frame_image(args.diagnose_image, target)
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    else:
        detection = payload["target_detection"]
        best_candidate = payload.get("best_candidate")
        print(
            "CALIBRATION_IMAGE_DIAGNOSIS "
            f"{payload['status']} detection={detection['status']} "
            f"tags={detection['tag_count']} rejected={detection['rejected_count']}"
        )
        if best_candidate:
            print(
                "BEST_MARKER_CANDIDATE "
                f"dictionary={best_candidate['dictionary']} count={best_candidate['marker_count']} "
                f"ids={best_candidate['marker_ids'][:16]}"
            )
        print(f"diagnosis={payload['diagnosis']}")
    return 0


def _module_status(name: str) -> dict:
    try:
        module = importlib.import_module(name)
    except Exception as exc:
        return {"available": False, "error": str(exc)}
    return {"available": True, "version": str(getattr(module, "__version__", "present"))}


def _binary_status(name: str) -> dict:
    path = shutil.which(name)
    payload = {"available": path is not None, "path": path}
    if path is not None:
        completed = subprocess.run([path, "-version"], text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        first_line = (completed.stdout or completed.stderr).splitlines()
        payload["version"] = first_line[0] if first_line else "unknown"
    return payload




def _output_dir(args) -> Path:
    if args.output_dir:
        return args.output_dir
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    camera_id = str(args.camera_id).replace("/", "_")
    rig_id = str(args.rig_id).replace("/", "_")
    if camera_id in {"", ".", ".."} or rig_id in {"", ".", ".."}:
        raise ValueError("camera-id and rig-id must name nonempty path components")
    return args.output_root / rig_id / camera_id / stamp




def _evaluate_holdout(model: str, params: object, observations: Sequence[Observation], image_size: Tuple[int, int]) -> Tuple[float, float, int]:
    residuals, invalid, rms = evaluate_fixed_intrinsics_holdout(model, params, observations, image_size)
    _median, p95 = residual_metrics(residuals)
    return rms, p95, invalid


def _combined_status(
    ds_result,
    kb4_result,
    ds_metrics: ModelMetrics,
    kb4_metrics: ModelMetrics,
    max_holdout_rms_px: float = 1.0,
    max_holdout_p95_px: float = 1.0,
) -> str:
    metrics_ok = all(
        np.isfinite(metric.holdout_rms_px)
        and metric.holdout_rms_px <= max_holdout_rms_px
        and metric.holdout_p95_px <= max_holdout_p95_px
        and metric.invalid_projection_count == 0
        for metric in (ds_metrics, kb4_metrics)
    )
    return "PASS" if ds_result.status == "PASS" and kb4_result.status == "PASS" and metrics_ok else "FAIL"


def _solve_ds_candidate_task(payload):
    index, xi, alpha, train, image_size, kb4_params, kb4_poses = payload
    initial = DSParameters(kb4_params.fx, kb4_params.fy, kb4_params.cx, kb4_params.cy, float(xi), float(alpha))
    result = solve_double_sphere(train, image_size, initial=initial, optimize_poses=True, initial_poses=kb4_poses)
    return index, result


def _solve_ds_candidates_parallel(
    train: Sequence[Observation],
    image_size: Tuple[int, int],
    kb4_params: KB4Parameters,
    candidates: Sequence[Tuple[float, float]],
    max_workers: int,
    kb4_poses: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
) -> List[Tuple[int, object]]:
    """DS多初值互不依赖，用spawn进程并行绕开Python残差循环单核瓶颈。"""
    seed_poses = None if kb4_poses is None else tuple(kb4_poses)
    payloads = [(index, float(xi), float(alpha), list(train), image_size, kb4_params, seed_poses) for index, (xi, alpha) in enumerate(candidates, start=1)]
    finished: List[Tuple[int, object]] = []
    ctx = mp.get_context("spawn")
    pool = ctx.Pool(processes=max_workers)
    try:
        for item in pool.imap_unordered(_solve_ds_candidate_task, payloads):
            finished.append(item)
        pool.close()
        pool.join()
    except BaseException:
        # Ctrl+C或worker异常时不能等待长耗时候选自然退出，否则现场恢复路径会继续卡住。
        pool.terminate()
        pool.join()
        raise
    return finished


def _better_ds_result(best, candidate):
    if best is None:
        return candidate
    current_key = (candidate.status != "PASS", candidate.invalid_projection_count, candidate.rms_px)
    best_key = (best.status != "PASS", best.invalid_projection_count, best.rms_px)
    return candidate if current_key < best_key else best

def _kb4_pose_seeds(kb4_result, train: Sequence[Observation]) -> Optional[Tuple[Tuple[np.ndarray, np.ndarray], ...]]:
    if getattr(kb4_result, "status", "FAIL") != "PASS":
        return None
    raw_poses = getattr(kb4_result, "poses", None)
    if raw_poses is None:
        return None
    poses = tuple(raw_poses)
    if len(poses) != len(train):
        return None
    for rvec, tvec in poses:
        pose = np.concatenate([np.asarray(rvec, dtype=np.float64).reshape(3), np.asarray(tvec, dtype=np.float64).reshape(3)])
        if not np.isfinite(pose).all() or pose[5] <= 0.0:
            return None
    # KB4失败或姿态数不匹配时DS必须回退PnP初值，不能让现场求解崩溃。
    return poses


def _solve_ds_multistart(
    train: Sequence[Observation],
    image_size: Tuple[int, int],
    kb4_params: KB4Parameters,
    candidates: Sequence[Tuple[float, float]],
    progress_callback: Optional[Callable[[str], None]] = None,
    max_workers: int = 1,
    kb4_poses: Optional[Sequence[Tuple[np.ndarray, np.ndarray]]] = None,
):
    """DS多初值优化逐候选耗时不定，回调暴露当前候选避免现场误判卡死。"""
    candidates = list(candidates)
    if not candidates:
        raise ValueError("at least one DS initial candidate is required")
    best = None
    total = len(candidates)
    worker_count = min(max(1, int(max_workers)), total)
    if worker_count > 1:
        if progress_callback is not None:
            progress_callback(f"ds_parallel_start workers={worker_count}")
            for index, (xi, alpha) in enumerate(candidates, start=1):
                progress_callback(f"ds_candidate_start candidate={index}/{total} xi={float(xi):g} alpha={float(alpha):g}")
        for index, result in _solve_ds_candidates_parallel(train, image_size, kb4_params, candidates, worker_count, kb4_poses):
            if progress_callback is not None:
                progress_callback(
                    f"ds_candidate_done candidate={index}/{total} status={result.status} train_rms_px={result.rms_px:.4f} "
                    f"invalid={result.invalid_projection_count}"
                )
            best = _better_ds_result(best, result)
        return best

    for index, (xi, alpha) in enumerate(candidates, start=1):
        # 候选开始事件必须在least_squares前输出，长耗时卡点才可定位。
        if progress_callback is not None:
            progress_callback(f"ds_candidate_start candidate={index}/{total} xi={float(xi):g} alpha={float(alpha):g}")
        initial = DSParameters(kb4_params.fx, kb4_params.fy, kb4_params.cx, kb4_params.cy, float(xi), float(alpha))
        result = solve_double_sphere(train, image_size, initial=initial, optimize_poses=True, initial_poses=kb4_poses)
        if progress_callback is not None:
            progress_callback(
                f"ds_candidate_done candidate={index}/{total} status={result.status} train_rms_px={result.rms_px:.4f} "
                f"invalid={result.invalid_projection_count}"
            )
        best = _better_ds_result(best, result)
    return best


def _ds_candidate_pairs(session_cfg: dict = None) -> List[Tuple[float, float]]:
    configured = (session_cfg or {}).get("ds_initial_candidates")
    if configured:
        return [(float(item[0]), float(item[1])) for item in configured]
    return [(-0.2, 0.35), (0.0, 0.5), (0.2, 0.65)]




def _debug_overlay_path(output_dir: Path, frame_index: int) -> Path:
    """返回调试标注图路径并创建目录；副作用仅在显式保存开关路径发生。"""
    overlay_dir = output_dir / "debug_overlays"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    return overlay_dir / f"frame_{frame_index:06d}.png"


def _render_debug_overlay(
    gray: np.ndarray,
    detection: AprilGridDetection,
    accepted: Optional[bool],
    quality_reasons: List[str],
    coverage=None,
    state: str = "PREVIEW",
    coverage_percent: Optional[float] = None,
    classification: Optional[str] = None,
    official_holdout_count: int = 0,
    required_holdout_count: int = 0,
) -> np.ndarray:
    """生成现场预览/调试PNG使用的角点标注图；输入必须是mono8或BGR图。"""
    # overlay 只在显式调试开关启用时生成，复制和BGR转换不会影响默认headless热路径。
    if gray.ndim == 2:
        overlay = cv2.cvtColor(np.asarray(gray, dtype=np.uint8), cv2.COLOR_GRAY2BGR)
    elif gray.ndim == 3:
        overlay = np.asarray(gray, dtype=np.uint8).copy()
    else:
        raise ValueError("debug overlay input must be mono8 or BGR")

    if accepted is True:
        color = (0, 220, 0)
        accepted_text = "true"
    elif accepted is False:
        color = (0, 0, 255)
        accepted_text = "false"
    else:
        color = (0, 220, 255)
        accepted_text = "n/a"

    # AprilGridDetection按tag顺序每4个角点对应一个tag_id，直接保留检测器输出顺序便于追查误检。
    points = np.asarray(detection.image_points, dtype=np.float64)
    if detection.status == "DETECTED" and points.size:
        for tag_offset, tag_id in enumerate(detection.tag_ids):
            corners = points[tag_offset * 4 : tag_offset * 4 + 4]
            if corners.shape != (4, 2) or not np.isfinite(corners).all():
                continue
            pixel_corners = np.round(corners).astype(np.int32)
            cv2.polylines(overlay, [pixel_corners.reshape((-1, 1, 2))], True, color, 2)
            for corner_index, (x_px, y_px) in enumerate(pixel_corners):
                cv2.circle(overlay, (int(x_px), int(y_px)), 4, color, -1)
                cv2.putText(
                    overlay,
                    str(corner_index),
                    (int(x_px) + 5, max(12, int(y_px) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    color,
                    1,
                    cv2.LINE_AA,
                )
            center_x, center_y = np.round(corners.mean(axis=0)).astype(int)
            cv2.putText(overlay, f"id={tag_id}", (int(center_x), int(center_y)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    snapshot = coverage if hasattr(coverage, "grid_frame_counts") else None
    coverage_percent = float(coverage_percent if coverage_percent is not None else coverage if coverage is not None else 0.0) if snapshot is None else 100.0 * float(np.count_nonzero(snapshot.grid_frame_counts)) / float(snapshot.grid_frame_counts.size)
    # 调试图必须展示可操作的多维采集缺口，不能只输出一个覆盖百分比。
    reasons = ",".join(quality_reasons) if quality_reasons else "-"
    classification_text = classification or ("REJECTED" if accepted is False else "-")
    if snapshot is None:
        status_lines = (
            f"{state} detection={detection.status} accepted={accepted_text} class={classification_text} coverage={coverage_percent:.1f}%",
            f"reasons={reasons} rejected_candidates={detection.rejected_count}",
        )
    else:
        zones = snapshot.zones
        scale = snapshot.scale
        pose = snapshot.pose
        status_lines = (
            f"{state} detection={detection.status} accepted={accepted_text} class={classification_text} accepted={snapshot.accepted_count} solve={snapshot.solve_eligible_count} coverage_only={snapshot.coverage_only_count} grid={coverage_percent:.1f}%",
            f"edges L/R/T/B={zones['EDGE_LEFT']}/{zones['EDGE_RIGHT']}/{zones['EDGE_TOP']}/{zones['EDGE_BOTTOM']} corners TL/TR/BL/BR={zones['CORNER_TOP_LEFT']}/{zones['CORNER_TOP_RIGHT']}/{zones['CORNER_BOTTOM_LEFT']}/{zones['CORNER_BOTTOM_RIGHT']}",
            f"scale S/M/L={scale['SMALL']}/{scale['MEDIUM']}/{scale['LARGE']} pose roll-/0/+={pose['ROLL_NEG']}/{pose['ROLL_NEUTRAL']}/{pose['ROLL_POS']} tiltX/Y/obl={pose['TILT_X_STRONG']}/{pose['TILT_Y_STRONG']}/{pose['OBLIQUE']}",
            f"holdout={official_holdout_count}/{required_holdout_count}",
            f"MISSING={','.join(snapshot.missing[:4]) or '-'}",
            f"NEXT={','.join(snapshot.next[:3])}",
            f"reasons={reasons} rejected_candidates={detection.rejected_count}",
        )
        grid = snapshot.grid_frame_counts
        height, width = overlay.shape[:2]
        # 10x8 网格颜色直接反映独立帧覆盖，操作者能按缺口移动标定板。
        for row in range(grid.shape[0]):
            for col in range(grid.shape[1]):
                x0 = int(round(col * width / grid.shape[1]))
                x1 = int(round((col + 1) * width / grid.shape[1]))
                y0 = int(round(row * height / grid.shape[0]))
                y1 = int(round((row + 1) * height / grid.shape[0]))
                cell_color = (0, 160, 0) if int(grid[row, col]) > 0 else (0, 0, 180)
                cv2.rectangle(overlay, (x0, y0), (max(x0, x1 - 1), max(y0, y1 - 1)), cell_color, 1)
    for row, text in enumerate(status_lines):
        y_px = 22 + row * 22
        cv2.putText(overlay, text[:140], (8, y_px), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(overlay, text[:140], (8, y_px), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return overlay


def _format_capture_status_line(
    state: str,
    snapshot: object,
    min_observations: int,
    classification: Optional[str],
    reasons: Sequence[str],
    capture_phase: str = "TRAIN",
    official_holdout_count: int = 0,
    required_holdout_count: int = 0,
) -> str:
    """headless采集必须暴露训练/hold-out阶段、样本分类和最近拒绝原因。"""
    reason_text = ",".join(reasons) if reasons else "-"
    classification_text = classification or "-"
    return (
        f"state={state} phase={capture_phase} accepted={snapshot.accepted_count} solve_eligible={snapshot.solve_eligible_count}/{min_observations} "
        f"coverage_only={snapshot.coverage_only_count} official_holdout={official_holdout_count}/{required_holdout_count} "
        f"class={classification_text} reasons={reason_text} "
        f"missing={','.join(snapshot.missing[:6]) or '-'} next={','.join(snapshot.next[:3])}"
    )

def _render_preview_status_overlay(gray: np.ndarray, coverage_percent: float, state: str) -> np.ndarray:
    """预览待机帧可跳过重检测，只绘制轻量状态文字保持窗口刷新。"""
    # 跳检帧只服务操作者取景，不进入标定/证据数据，因此避免AprilGrid检测热路径开销。
    if gray.ndim == 2:
        overlay = cv2.cvtColor(np.asarray(gray, dtype=np.uint8), cv2.COLOR_GRAY2BGR)
    elif gray.ndim == 3:
        overlay = np.asarray(gray, dtype=np.uint8).copy()
    else:
        raise ValueError("preview overlay input must be mono8 or BGR")

    color = (0, 220, 255)
    header = f"{state} detection=SKIPPED accepted=n/a coverage={coverage_percent:.1f}%"
    detail = "reasons=preview_detect_throttled rejected_candidates=n/a"
    for row, text in enumerate((header, detail)):
        y_px = 22 + row * 22
        cv2.putText(overlay, text[:140], (8, y_px), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(overlay, text[:140], (8, y_px), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return overlay


def _should_detect_frame(frame_index: int, state: str, preview_detect_every_n: int, force_overlay: bool) -> bool:
    """PREVIEW只需周期性刷新角点状态；CAPTURING检测每个已消费的最新帧，旧RTSP帧由帧源层丢弃。"""
    if force_overlay or state != "PREVIEW":
        return True
    return frame_index % preview_detect_every_n == 0


def _write_debug_overlay(output_dir: Path, frame_index: int, overlay: np.ndarray) -> Path:
    """将显式开启的调试标注PNG落盘；OpenCV写入失败时fail closed。"""
    path = _debug_overlay_path(output_dir, frame_index)
    if not cv2.imwrite(str(path), overlay):
        raise RuntimeError(f"failed to write debug overlay: {path}")
    return path


def _preview_key_to_command(key_code: int) -> Optional[str]:
    """OpenCV窗口获得焦点时，终端按键不会进入stdin，需要把窗口单键映射到会话命令。"""
    if key_code < 0:
        return None
    key = key_code & 0xFF
    if key == 27:
        return "q"
    key_char = chr(key).lower()
    if key_char in {"s", "h", "f", "r", "q"}:
        return key_char
    return None

def _scale_preview_overlay(overlay: np.ndarray, scale: float) -> np.ndarray:
    """GUI预览可降采样显示，标定输入和落盘证据仍保持原始分辨率。"""
    if scale == 1.0:
        return overlay
    height, width = overlay.shape[:2]
    scaled_size = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(overlay, scaled_size, interpolation=interpolation)


def _should_write_debug_overlay(frame_index: int, every_n: int) -> bool:
    """PNG写盘是现场调试证据路径，按帧号节流避免拖慢实时预览。"""
    return frame_index % every_n == 0

_PREVIEW_WINDOW_FRAMES: dict[str, np.ndarray] = {}


def _prepare_preview_frame(window_name: str, overlay: np.ndarray, preview_scale: float) -> np.ndarray:
    """OpenCV GUI后端可能异步读取imshow入参，必须保留连续显示缓冲避免窗口花屏。"""
    # resize结果或切片输入不一定拥有稳定生命周期；ascontiguousarray统一成imshow可安全读取的BGR缓冲。
    frame = np.ascontiguousarray(_scale_preview_overlay(overlay, preview_scale))
    _PREVIEW_WINDOW_FRAMES[window_name] = frame
    return frame





def _show_preview_window(window_name: str, overlay: np.ndarray, preview_scale: float = 1.0) -> Tuple[bool, Optional[str]]:
    """显式预览窗口既显示降采样帧，也接收窗口焦点下的采集热键。"""
    try:
        cv2.imshow(window_name, _prepare_preview_frame(window_name, overlay, preview_scale))
        key_code = cv2.waitKey(1)
    except cv2.error as exc:
        _PREVIEW_WINDOW_FRAMES.pop(window_name, None)
        raise RuntimeError("--preview-window requires a GUI-capable OpenCV display; run without it on headless hosts") from exc
    return True, _preview_key_to_command(key_code)


def _write_terminal_log(output_dir: Path, status: str, source: str, args) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    rtsp_url = redact_url(args.rtsp_url) if getattr(args, "rtsp_url", None) else ""
    raw_tcp = getattr(args, "raw_tcp", None) or ""
    text = f"status={status}\nsource={source}\nrtsp_url={rtsp_url}\nraw_tcp={raw_tcp}\ncamera_id={args.camera_id}\n"
    (output_dir / "terminal.log").write_text(text, encoding="utf-8")


def _print_solver_event(message: str) -> None:
    """求解期间stdout可能被行缓冲/块缓冲，操作者进度事件必须强制flush。"""
    print(message, flush=True)


def _capture_readiness_with_holdout(training_snapshot: object, official_holdout_count: int, required_holdout_count: int):
    missing = list(getattr(training_snapshot, "missing", []))
    if official_holdout_count < required_holdout_count:
        missing.append("OFFICIAL_HOLDOUT_COUNT")
    return SimpleNamespace(ready=bool(getattr(training_snapshot, "ready", False)) and not missing, missing=missing)


def _select_training_and_holdout(
    training_observations: Sequence[Observation],
    holdout_observations: Sequence[Observation],
    max_solve_observations: int,
    official_holdout_count: int,
    min_train_observations: int,
    image_size: Tuple[int, int],
) -> Tuple[List[Observation], List[Observation]]:
    holdout = select_representative_observations(holdout_observations, official_holdout_count, image_size)
    if max_solve_observations <= 0:
        train_limit = 0
    else:
        train_limit = max(min_train_observations, max_solve_observations - official_holdout_count)
    train = select_representative_observations(training_observations, train_limit, image_size)
    return train, holdout


def run_interactive_session(args) -> int:
    target = load_target(args.target)
    session_cfg = _load_session_config(args.session_config)
    output_dir = _output_dir(args)
    writer = DatasetWriter(output_dir, save_mode=args.save_data)
    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(args.target, output_dir / "target.yaml")
    shutil.copyfile(args.session_config, output_dir / "session_config.yaml")
    capture_config = capture_config_from_mapping(session_cfg)
    assistant = CaptureAssistant(DEFAULT_IMAGE_SIZE, capture_config)
    holdout_assistant = CaptureAssistant(DEFAULT_IMAGE_SIZE, capture_config)
    thresholds = QualityThresholds(
        min_laplacian_var=float(session_cfg.get("min_laplacian_var", 15.0)),
        min_contrast=float(session_cfg.get("min_contrast", 20.0)),
        max_saturated_fraction=float(session_cfg.get("max_saturated_fraction", 0.35)),
    )
    observations: List[Observation] = []
    official_holdout_observations: List[Observation] = []
    accepted_frames: dict[int, np.ndarray] = {}
    accepted_metadata: dict[int, dict] = {}
    min_observations = int(session_cfg.get("min_observations", 12))
    required_holdout_count = int(session_cfg.get("official_holdout_frames", 8))
    capture_phase = "TRAIN"

    def reset_capture_state() -> None:
        nonlocal assistant, holdout_assistant, capture_phase
        observations.clear()
        official_holdout_observations.clear()
        accepted_frames.clear()
        accepted_metadata.clear()
        assistant = CaptureAssistant(DEFAULT_IMAGE_SIZE, capture_config)
        holdout_assistant = CaptureAssistant(DEFAULT_IMAGE_SIZE, capture_config)
        capture_phase = "TRAIN"

    def solve_current(_captured: List[Observation]):
        """交互求解在owner线程同步执行，逐阶段flush日志补足GUI冻结期可观测性。"""
        solve_started_at = time.monotonic()

        def log_progress(message: str) -> None:
            elapsed_s = time.monotonic() - solve_started_at
            _print_solver_event(f"SOLVER_PROGRESS elapsed_s={elapsed_s:.1f} {message}")

        captured_observations = list(_captured)
        configured_max_solve = int(session_cfg.get("max_solve_observations", 30))
        effective_max_solve = max(configured_max_solve, min_observations + required_holdout_count) if configured_max_solve > 0 else 0
        train, holdout = _select_training_and_holdout(
            captured_observations,
            official_holdout_observations,
            max_solve_observations=effective_max_solve,
            official_holdout_count=required_holdout_count,
            min_train_observations=min_observations,
            image_size=DEFAULT_IMAGE_SIZE,
        )
        max_solve_label = "unbounded" if effective_max_solve == 0 else str(effective_max_solve)
        log_progress(
            f"select_representative accepted={len(observations)} solve_eligible={len(captured_observations)} "
            f"train={len(train)} official_holdout={len(holdout)} max_total={max_solve_label}"
        )
        log_progress(f"split train={len(train)} holdout={len(holdout)} source=operator_phases")
        log_progress("write_dataset_start")
        selected_indices = {observation.index for observation in train}
        selected_indices.update(observation.index for observation in holdout)
        for split_name, split_observations in (("train", train), ("holdout", holdout)):
            for observation in split_observations:
                frame_image = accepted_frames.get(observation.index)
                if frame_image is not None:
                    metadata = dict(accepted_metadata.get(observation.index, {}))
                    metadata["split"] = split_name
                    writer.record_frame(frame_image, observation.index, True, metadata, split=split_name)
        # 未进入train/holdout的accepted帧仍作为采集证据保存，coverage_only和超出求解上限帧都不参与优化。
        for observation in observations:
            if observation.index in selected_indices:
                continue
            frame_image = accepted_frames.get(observation.index)
            if frame_image is not None:
                metadata = dict(accepted_metadata.get(observation.index, {}))
                metadata["split"] = "accepted"
                writer.record_frame(frame_image, observation.index, True, metadata, split="accepted")
        log_progress("write_dataset_done")
        # KB4是第一段重优化，先打印start再进入OpenCV fisheye.calibrate。
        log_progress("kb4_start")
        kb4_result = solve_kb4(train, DEFAULT_IMAGE_SIZE, optimize_poses=True)
        log_progress(f"kb4_done status={kb4_result.status} train_rms_px={kb4_result.rms_px:.4f} invalid={kb4_result.invalid_projection_count}")
        kb4_pose_seeds = _kb4_pose_seeds(kb4_result, train)
        # DS会按配置跑多个初值，逐候选进度比单一总耗时更利于现场判断。
        ds_candidates = _ds_candidate_pairs(session_cfg)
        ds_parallel_workers = int(session_cfg.get("ds_parallel_workers", min(len(ds_candidates), 3)))
        log_progress(f"ds_multistart_start candidates={len(ds_candidates)} workers={max(1, min(ds_parallel_workers, len(ds_candidates)))}")
        ds_result = _solve_ds_multistart(train, DEFAULT_IMAGE_SIZE, kb4_result.parameters, ds_candidates, progress_callback=log_progress, max_workers=ds_parallel_workers, kb4_poses=kb4_pose_seeds)
        log_progress(f"ds_multistart_done status={ds_result.status} train_rms_px={ds_result.rms_px:.4f} invalid={ds_result.invalid_projection_count}")
        # holdout和报告写盘通常较快，仍显式打点以闭合求解阶段流水线。
        log_progress("holdout_start")
        ds_holdout = _evaluate_holdout("ds", ds_result.parameters, holdout, DEFAULT_IMAGE_SIZE)
        kb4_holdout = _evaluate_holdout("kb4", kb4_result.parameters, holdout, DEFAULT_IMAGE_SIZE)
        log_progress(f"holdout_done ds_rms_px={ds_holdout[0]:.4f} ds_p95_px={ds_holdout[1]:.4f} kb4_rms_px={kb4_holdout[0]:.4f} kb4_p95_px={kb4_holdout[1]:.4f}")
        ds_metrics = ModelMetrics("ds", ds_result.rms_px, ds_holdout[0], ds_holdout[1], ds_result.invalid_projection_count + ds_holdout[2])
        kb4_metrics = ModelMetrics("kb4", kb4_result.rms_px, kb4_holdout[0], kb4_holdout[1], kb4_result.invalid_projection_count + kb4_holdout[2])
        decision = select_model(ds_metrics, kb4_metrics, equivalence_margin_px=float(session_cfg.get("equivalence_margin_px", 0.05)))
        log_progress(f"model_select selected={decision.selected_model} reason={decision.reason}")
        log_progress("write_reports_start")
        write_reports(output_dir, args.rig_id, str(args.camera_id), DEFAULT_IMAGE_SIZE, ds_result.parameters, kb4_result.parameters, ds_metrics, kb4_metrics, decision,
                      max_holdout_rms_px=float(session_cfg.get("max_holdout_rms_px", 1.0)),
                      max_holdout_p95_px=float(session_cfg.get("max_holdout_p95_px", 1.0)))
        log_progress("write_reports_done")
        return _combined_status(
            ds_result,
            kb4_result,
            ds_metrics,
            kb4_metrics,
            max_holdout_rms_px=float(session_cfg.get("max_holdout_rms_px", 1.0)),
            max_holdout_p95_px=float(session_cfg.get("max_holdout_p95_px", 1.0)),
        )

    def capture_readiness(_captured: List[Observation]):
        return _capture_readiness_with_holdout(assistant.readiness(), len(official_holdout_observations), required_holdout_count)

    session = CalibrationSession(
        SessionConfig(min_observations=min_observations),
        solver=solve_current,
        reset_callback=reset_capture_state,
        progress_callback=_print_solver_event,
        readiness_gate=capture_readiness,
    )
    print("PREVIEW: click preview window and press s/h/f/r/q, or type s/h/f/r/q + Enter in terminal")
    preview_window_name = f"Robobaton calibration {args.camera_id}"
    if args.preview_window:
        print(f"PREVIEW_WINDOW: enabled; preview hotkeys are active; scale={args.preview_scale:g}; detect_every_n={args.preview_detect_every_n}")
    if args.save_debug_overlays:
        print(f"DEBUG_OVERLAYS: writing {output_dir / 'debug_overlays'} every {args.debug_overlay_every_n} frame(s)")
    preview_window_open = False
    status = "FAIL"
    processed_frame_count = 0


    def apply_operator_command(command: str) -> bool:
        """训练采集与official hold-out必须由操作者显式分段，禁止自动从训练池抽验证帧。"""
        nonlocal status, capture_phase
        normalized = command.strip().lower()
        if normalized == "h":
            if session.state.value != "CAPTURING":
                print(f"cannot start holdout from {session.state.value}")
                return False
            training_snapshot = assistant.readiness()
            if not training_snapshot.ready:
                print(f"holdout not started; training missing={','.join(training_snapshot.missing)}")
                return False
            if capture_phase == "HOLDOUT":
                print(f"official holdout already active count={len(official_holdout_observations)}/{required_holdout_count}")
                return False
            capture_phase = "HOLDOUT"
            print(f"OFFICIAL_HOLDOUT_START required={required_holdout_count}; use new poses, then press f")
            return False
        if normalized == "f" and capture_phase == "TRAIN" and assistant.readiness().ready:
            print("official holdout not started; press h first, then collect 8 new holdout poses")
            return False
        result = session.handle_command(normalized)
        print(result.message)
        if result.status in {"PASS", "FAIL", "CANCELED"}:
            status = "PASS" if result.status == "PASS" else "FAIL"
            return True
        return False
    if args.raw_tcp:
        host, port = _parse_raw_tcp(args.raw_tcp)
        source = RawTcpFrameSource(host, port, _parse_camera_index(args.camera_id), DEFAULT_IMAGE_SIZE)
        source_kind = "raw-tcp"
    else:
        source = RTSPFrameSource(args.rtsp_url, DEFAULT_IMAGE_SIZE)
        source_kind = "rtsp"
    try:
        with source:
            for frame in source.frames():
                accepted: Optional[bool] = None
                overlay_reasons: List[str] = []
                classification: Optional[str] = None
                preview_command: Optional[str] = None
                should_write_overlay = args.save_debug_overlays and _should_write_debug_overlay(frame.index, args.debug_overlay_every_n)
                detection: Optional[AprilGridDetection] = None
                if _should_detect_frame(frame.index, session.state.value, args.preview_detect_every_n, should_write_overlay):
                    detection = detect_aprilgrid(frame.gray, target)

                if detection is not None and session.state.value == "CAPTURING" and detection.status == "DETECTED":
                    quality = classify_frame_quality(frame.gray, thresholds)
                    observation = Observation(detection.object_points, detection.image_points, index=frame.index)
                    overlay_reasons = list(quality.reasons)
                    classification = "REJECTED"
                    coverage = None
                    if capture_phase == "HOLDOUT":
                        holdout_reasons = assistant.holdout_gate_reasons(observation) if quality.accepted else []
                        overlay_reasons.extend(holdout_reasons)
                        coverage = holdout_assistant.consider_and_accept(observation) if quality.accepted and not holdout_reasons else None
                        if coverage is not None:
                            overlay_reasons.extend(coverage.reasons)
                        accepted = quality.accepted and not holdout_reasons and coverage is not None and coverage.accepted
                        if accepted:
                            classification = "OFFICIAL_HOLDOUT"
                            observations.append(observation)
                            official_holdout_observations.append(observation)
                            accepted_frames[frame.index] = frame.gray.copy()
                            accepted_metadata[frame.index] = {
                                "detection": detection.status,
                                "quality": quality.reasons,
                                "classification": classification,
                                "coverage": coverage.reasons,
                            }
                    else:
                        coverage = assistant.consider_and_accept(observation) if quality.accepted else None
                        accepted = quality.accepted and coverage is not None and coverage.accepted
                        if coverage is not None:
                            classification = coverage.classification
                            overlay_reasons.extend(coverage.reasons)
                        if accepted:
                            observations.append(observation)
                            accepted_frames[frame.index] = frame.gray.copy()
                            accepted_metadata[frame.index] = {
                                "detection": detection.status,
                                "quality": quality.reasons,
                                "classification": classification,
                                "coverage": coverage.reasons,
                            }
                            if coverage.solve_eligible:
                                session.ingest_observation(observation)
                    if not accepted and args.save_data == "all":
                        metadata = {"detection": detection.status, "quality": quality.reasons, "classification": classification}
                        if coverage is not None:
                            metadata["coverage"] = coverage.reasons
                        writer.record_frame(frame.gray, frame.index, False, metadata)
                elif detection is not None and session.state.value == "CAPTURING":
                    accepted = False
                    classification = "REJECTED"
                    overlay_reasons = [detection.status]
                elif detection is not None and detection.status != "DETECTED":
                    overlay_reasons = [detection.status]

                if args.preview_window or should_write_overlay:
                    if detection is not None:
                        overlay = _render_debug_overlay(
                            frame.gray,
                            detection,
                            accepted,
                            overlay_reasons,
                            assistant.snapshot(),
                            f"{session.state.value}/{capture_phase}",
                            classification=classification,
                            official_holdout_count=len(official_holdout_observations),
                            required_holdout_count=required_holdout_count,
                        )
                    else:
                        overlay = _render_preview_status_overlay(frame.gray, 100.0 * np.count_nonzero(assistant.snapshot().grid_frame_counts) / assistant.snapshot().grid_frame_counts.size, session.state.value)
                    if args.preview_window:
                        opened, preview_command = _show_preview_window(preview_window_name, overlay, args.preview_scale)
                        preview_window_open = opened or preview_window_open
                    if should_write_overlay:
                        _write_debug_overlay(output_dir, frame.index, overlay)

                if preview_command is not None and apply_operator_command(preview_command):
                    break
                if processed_frame_count % 30 == 0:
                    snapshot = assistant.snapshot()
                    print(
                        _format_capture_status_line(
                            session.state.value,
                            snapshot,
                            session.config.min_observations,
                            classification,
                            overlay_reasons,
                            capture_phase,
                            len(official_holdout_observations),
                            required_holdout_count,
                        )
                    )
                terminal_command = _poll_terminal_command()
                if terminal_command is not None and apply_operator_command(terminal_command):
                    break
                # 真实交互采集默认不设帧上限；显式max_frames只服务自动化冒烟和现场限时复现。
                processed_frame_count += 1
                if args.max_frames > 0 and processed_frame_count >= args.max_frames:
                    snapshot = assistant.snapshot()
                    print(
                        f"max frame limit reached after {processed_frame_count} processed frames; "
                        + _format_capture_status_line(
                            session.state.value,
                            snapshot,
                            session.config.min_observations,
                            classification,
                            overlay_reasons,
                            capture_phase,
                            len(official_holdout_observations),
                            required_holdout_count,
                        )
                    )
                    break
    finally:
        if args.preview_window and preview_window_open:
            # 只销毁已成功创建的预览窗口，避免headless或测试替身路径产生伪错误日志。
            try:
                cv2.destroyWindow(preview_window_name)
            except cv2.error:
                pass
            finally:
                _PREVIEW_WINDOW_FRAMES.pop(preview_window_name, None)
        manifest = writer.finalize(extra={"rig_id": args.rig_id, "camera_id": args.camera_id, "source": source_kind, "tool_version": __version__, "image_width": DEFAULT_IMAGE_SIZE[0], "image_height": DEFAULT_IMAGE_SIZE[1]})
        _write_terminal_log(output_dir, status=status, source=source_kind, args=args)
    print(f"CALIBRATION_RESULT {status} output_dir={output_dir} manifest={manifest}")
    return 0 if status == "PASS" else 1


def select_ready_stdin() -> list:
    import select

    ready, _, _ = select.select([sys.stdin], [], [], 0.0)
    return ready


def _poll_terminal_command() -> Optional[str]:
    """非阻塞读取终端命令；忽略空行和管道EOF，避免无人值守运行刷ignored日志。"""
    if sys.stdin not in select_ready_stdin():
        return None
    command = sys.stdin.readline()
    return command if command.strip() else None




def _load_session_config(path: Path) -> dict:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("session configuration must be a mapping")
    if (data.get("image_width"), data.get("image_height")) != DEFAULT_IMAGE_SIZE or data.get("encoding") != "mono8":
        raise ValueError("this capture profile requires 1280x1088 mono8")
    return data


if __name__ == "__main__":
    raise SystemExit(main())
