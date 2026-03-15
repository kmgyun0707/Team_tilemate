#!/usr/bin/env python3
import math
from typing import Dict, Any, List, Tuple


def _norm(v: List[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


def _normalize(v: List[float]) -> List[float]:
    n = _norm(v)
    if n < 1e-12:
        raise ValueError(f"zero-length vector: {v}")
    return [x / n for x in v]


def _dot(a: List[float], b: List[float]) -> float:
    return sum(x * y for x, y in zip(a, b))


def angle_between_normals_deg(n1: List[float], n2: List[float]) -> float:
    a = _normalize(n1)
    b = _normalize(n2)
    c = max(-1.0, min(1.0, _dot(a, b)))
    return math.degrees(math.acos(c))


def signed_distance_point_to_plane_mm(point_mm: List[float], plane_normal: List[float], plane_d: float) -> float:
    """
    plane equation: n.x + d = 0
    normal은 normalize해서 사용
    """
    n = _normalize(plane_normal)
    return _dot(n, point_mm) + plane_d


def extract_wall(wall_json: Dict[str, Any]) -> Dict[str, Any]:
    wall = wall_json.get("wall")
    if not isinstance(wall, dict):
        raise ValueError("wall_json['wall'] missing or invalid")

    required = ["plane_normal", "plane_d", "plane_centroid_mm"]
    for k in required:
        if k not in wall:
            raise ValueError(f"wall_json['wall'] missing key: {k}")

    return wall


def analyze_tiles_against_wall(
    tile_json: Dict[str, Any],
    wall_json: Dict[str, Any],
    press_threshold_mm: float,
    tilt_threshold_deg: float,
) -> Dict[str, Any]:
    wall = extract_wall(wall_json)
    wall_normal = wall["plane_normal"]
    wall_d = float(wall["plane_d"])

    tiles = tile_json.get("tiles", [])
    analyzed_tiles = []
    defects = []
    press_targets = []

    for idx, tile in enumerate(tiles):
        tile_name = tile.get("name", f"tile_{idx}")
        tile_normal = tile.get("plane_normal")
        tile_centroid = tile.get("plane_centroid_mm")

        if tile_normal is None or tile_centroid is None:
            analyzed = dict(tile)
            analyzed["analysis_error"] = "missing plane_normal or plane_centroid_mm"
            analyzed_tiles.append(analyzed)
            continue

        tilt_deg = angle_between_normals_deg(tile_normal, wall_normal)
        signed_step_mm = signed_distance_point_to_plane_mm(tile_centroid, wall_normal, wall_d)

        # 현재 optical frame 기준으로, signed_step_mm < 0 이면 벽보다 카메라 쪽으로 튀어나온 것으로 해석
        protrusion_mm = max(0.0, -signed_step_mm)
        recession_mm = max(0.0, signed_step_mm)

        is_step_defect = protrusion_mm > press_threshold_mm
        is_tilt_defect = tilt_deg > tilt_threshold_deg
        is_defect = is_step_defect or is_tilt_defect
        press_candidate = is_step_defect  # 실제 press는 튀어나온 경우만

        reasons = []
        if is_step_defect:
            reasons.append("protrusion")
        if is_tilt_defect:
            reasons.append("tilt")

        analyzed = dict(tile)
        analyzed["analysis"] = {
            "signed_step_mm": signed_step_mm,
            "protrusion_mm": protrusion_mm,
            "recession_mm": recession_mm,
            "tilt_deg": tilt_deg,
            "is_step_defect": is_step_defect,
            "is_tilt_defect": is_tilt_defect,
            "is_defect": is_defect,
            "press_candidate": press_candidate,
            "reasons": reasons,
        }
        analyzed_tiles.append(analyzed)

        if is_defect:
            defect = {
                "tile_index": idx,
                "name": tile_name,
                "plane_centroid_mm": tile_centroid,
                "plane_normal": tile_normal,
                "signed_step_mm": signed_step_mm,
                "protrusion_mm": protrusion_mm,
                "recession_mm": recession_mm,
                "tilt_deg": tilt_deg,
                "is_step_defect": is_step_defect,
                "is_tilt_defect": is_tilt_defect,
                "press_candidate": press_candidate,
                "reasons": reasons,
                "raw_tile": tile,
            }
            defects.append(defect)
            if press_candidate:
                press_targets.append(defect)

    return {
        "success": bool(tile_json.get("success", True)),
        "message": tile_json.get("message", "inspect_complete"),
        "frame_id": tile_json.get("frame_id", ""),
        "timestamp_sec": tile_json.get("timestamp_sec", 0.0),
        "wall": wall,
        "tiles": analyzed_tiles,
        "defects": defects,
        "press_targets": press_targets,
        "summary": {
            "total_tiles": len(tiles),
            "total_defects": len(defects),
            "total_press_targets": len(press_targets),
        },
    }