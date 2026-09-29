#!/usr/bin/env python3
"""DS（Double Sphere）求解器桥——给 Rust 侧当后端用。

模型和优化器位于 `tools/ds_py/robobaton_ds/`。冷启动先解 KB4，
再把内参与位姿传给 DS；本文件负责 JSON 输入输出。

协议：stdin 一行 JSON → stdout 一行 JSON
  入: {"observations":[{"object_points":[[x,y,z]…],"image_points":[[u,v]…]}],
       "image_size":[w,h], "initial":[fx,fy,cx,cy,xi,alpha]|null, "candidates":[[xi,alpha],…]|null,
       "initial_poses":[[[rx,ry,rz],[tx,ty,tz]],…]|null}
  出: {"parameters":[…6], "rms_px":f, "per_view_rms":[…], "invalid_projection_count":i,
       "elapsed_s":f, "candidate_index":i, "status":s,
       "poses":[[[rx,ry,rz],[tx,ty,tz]],…], "residuals_px":[[du,dv],…]}

`initial` 非空视为**热启动**：直接拿它（含 xi/alpha）当单一初值，不再套用 `candidates`
——候选列表只在冷启动（`initial` 为空）时生效。

`initial_poses` 非空时作为各视图位姿初值；冷启动时使用 KB4 联合求解位姿。

依赖：python3 + numpy + scipy + opencv（`RIGCAL_DS_PYTHON` 可指定解释器）。
"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "ds_py"))

import numpy as np  # noqa: E402

from robobaton_ds.double_sphere import DSParameters  # noqa: E402
from robobaton_ds.solver import Observation, solve_double_sphere, solve_kb4  # noqa: E402


def _initial_candidates(payload: dict, kb4) -> list:
    """热启动保留 DS 解；冷启动候选保持 KB4 的轴心角度斜率。"""
    warm = payload.get("initial")
    if warm is not None:
        values = [float(value) for value in warm]
        if len(values) != 6:
            raise ValueError("initial must have 6 values [fx,fy,cx,cy,xi,alpha]")
        return [DSParameters(*values)]
    xi = 0.2
    scale = 1.0 + xi
    alpha = float(np.clip(scale - 2.0 * scale**2 * (kb4.k1 + 1.0 / 6.0), 0.05, 0.95))
    candidates = payload.get("candidates") or [[xi, alpha]]
    return [
        DSParameters(fx=kb4.fx * (1.0 + xi), fy=kb4.fy * (1.0 + xi), cx=kb4.cx, cy=kb4.cy,
                     xi=float(xi), alpha=float(alpha))
        for xi, alpha in candidates
    ]


def main() -> int:
    payload = json.load(sys.stdin)
    width, height = payload["image_size"]
    observations = [
        Observation(
            object_points=np.asarray(item["object_points"], dtype=np.float64),
            image_points=np.asarray(item["image_points"], dtype=np.float64),
        )
        for item in payload["observations"]
    ]
    initial_poses = payload.get("initial_poses")
    if initial_poses is not None:
        initial_poses = [
            (np.asarray(rvec, dtype=np.float64).reshape(3),
             np.asarray(tvec, dtype=np.float64).reshape(3))
            for rvec, tvec in initial_poses
        ]
    kb4 = None
    if payload.get("initial") is None:
        bootstrap = solve_kb4(observations, (width, height), optimize_poses=True)
        if bootstrap.status != "PASS" or bootstrap.invalid_projection_count:
            raise RuntimeError("KB4 initialization failed")
        kb4 = bootstrap.parameters
        initial_poses = bootstrap.poses

    best = None
    for index, params in enumerate(_initial_candidates(payload, kb4)):
        started = time.time()
        result = solve_double_sphere(
            observations, (width, height),
            initial=params, optimize_poses=True, initial_poses=initial_poses,
        )
        elapsed = time.time() - started
        rms = float(getattr(result, "rms_px", float("inf")))
        rank = (result.status != "PASS" or result.invalid_projection_count != 0, rms)
        if best is None or rank < best[0]:
            best = (rank, result, elapsed, index)
    (_, rms), result, elapsed, index = best
    poses = [
        [[float(value) for value in np.asarray(rvec, dtype=np.float64).reshape(3)],
         [float(value) for value in np.asarray(tvec, dtype=np.float64).reshape(3)]]
        for rvec, tvec in getattr(result, "poses", ())
    ]
    residuals = [
        [float(delta[0]), float(delta[1])]
        for delta in np.asarray(getattr(result, "residuals_px", []), dtype=np.float64).reshape(-1, 2)
    ]
    json.dump({
        "parameters": [float(value) for value in result.parameters.as_vector()],
        "rms_px": rms,
        "per_view_rms": [float(value) for value in getattr(result, "per_view_rms_px", [])],
        "invalid_projection_count": int(getattr(result, "invalid_projection_count", 0)),
        "elapsed_s": elapsed,
        "candidate_index": index,
        "status": str(getattr(result, "status", "FAIL")),
        "poses": poses,
        "residuals_px": residuals,
    }, sys.stdout)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
