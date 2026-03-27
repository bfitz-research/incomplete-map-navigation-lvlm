#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import re
from typing import List, Tuple

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


# ----------------------------
# Map YAML parsing
# ----------------------------
def parse_map_yaml(map_yaml_path: str) -> dict:
    """
    Minimal parser for ROS map_server YAML:
      image: map.png
      resolution: 0.05
      origin: [x, y, yaw]
    """
    txt = open(map_yaml_path, "r", encoding="utf-8").read()

    m_img = re.search(r"^\s*image:\s*(.+)\s*$", txt, re.MULTILINE)
    m_res = re.search(r"^\s*resolution:\s*([0-9.]+)\s*$", txt, re.MULTILINE)
    m_org = re.search(r"^\s*origin:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\]\s*$", txt, re.MULTILINE)

    if not (m_img and m_res and m_org):
        raise ValueError(f"Could not parse required fields from {map_yaml_path}")

    image_field = m_img.group(1).strip().strip('"').strip("'")
    res = float(m_res.group(1))
    ox = float(m_org.group(1))
    oy = float(m_org.group(2))
    yaw = float(m_org.group(3))

    if os.path.isabs(image_field):
        map_png_path = image_field
    else:
        map_png_path = os.path.join(os.path.dirname(map_yaml_path), image_field)

    return {
        "map_yaml": map_yaml_path,
        "map_png": map_png_path,
        "resolution": res,
        "origin_x": ox,
        "origin_y": oy,
        "origin_yaw": yaw,
    }


# ----------------------------
# Coordinate transforms
# ----------------------------
def world_to_pixel(x: float, y: float, meta: dict, img_h: int) -> Tuple[float, float]:
    """
    ROS map convention:
      origin (ox, oy) is world coordinate of the image's bottom-left pixel.
    Image pixel coords:
      (0,0) is top-left.
    """
    res = meta["resolution"]
    ox = meta["origin_x"]
    oy = meta["origin_y"]

    px = (x - ox) / res
    py_from_bottom = (y - oy) / res
    py = (img_h - 1) - py_from_bottom

    return px, py


# ----------------------------
# Loaders
# ----------------------------
def load_oracle_json(path_json: str) -> List[Tuple[float, float]]:
    data = json.load(open(path_json, "r", encoding="utf-8"))
    return [(float(p["x"]), float(p["y"])) for p in data["path"]["poses"]]


def load_traj_csv(traj_csv: str) -> List[Tuple[float, float]]:
    pts = []
    with open(traj_csv, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            pts.append((float(row["x"]), float(row["y"])))
    return pts


# ----------------------------
# Metrics
# ----------------------------
def polyline_length(pts: List[Tuple[float, float]]) -> float:
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts)):
        dx = pts[i][0] - pts[i - 1][0]
        dy = pts[i][1] - pts[i - 1][1]
        total += math.hypot(dx, dy)
    return total


def point_to_segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    """
    Distance from point P to segment AB in 2D.
    """
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab_len2 = abx * abx + aby * aby

    if ab_len2 <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = (apx * abx + apy * aby) / ab_len2
    if t < 0.0:
        cx, cy = ax, ay
    elif t > 1.0:
        cx, cy = bx, by
    else:
        cx, cy = ax + t * abx, ay + t * aby

    return math.hypot(px - cx, py - cy)


def deviation_stats_point_to_segment(executed: List[Tuple[float, float]], oracle: List[Tuple[float, float]]) -> dict:
    """
    Robust deviation: for each executed point, compute distance to the nearest ORACLE SEGMENT.
    Returns mean and max in meters.
    """
    if len(executed) == 0 or len(oracle) < 2:
        return {"mean_m": None, "max_m": None}

    dists = []
    # Pre-build segments for oracle
    segs = list(zip(oracle[:-1], oracle[1:]))

    for (x, y) in executed:
        best = None
        for (a, b) in segs:
            d = point_to_segment_distance(x, y, a[0], a[1], b[0], b[1])
            if best is None or d < best:
                best = d
        dists.append(best)

    mean_d = sum(dists) / len(dists)
    max_d = max(dists)
    return {"mean_m": float(mean_d), "max_m": float(max_d)}


# ----------------------------
# Plot styling (explicit)
# ----------------------------
ORACLE_COLOR = "#7E3FF2"   # purple
EXEC_COLOR   = "#FF8C1A"   # orange
ORACLE_LW = 1.8
EXEC_LW   = 1.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-yaml", required=True, help="Path to map.yaml used for the run")
    ap.add_argument("--oracle-json", required=True, help="notes/<env>/oracle/oracle_path.json")
    ap.add_argument("--traj-csv", required=True, help="runs/<MODE>/<timestamp>/traj.csv")
    ap.add_argument("--out-png", required=True, help="Output overlay PNG path")
    ap.add_argument("--out-metrics", required=False, default="", help="Optional metrics.json output path")
    ap.add_argument("--legend", action="store_true", help="Include legend labels (recommended)")
    args = ap.parse_args()

    map_yaml = os.path.expanduser(args.map_yaml)
    oracle_json = os.path.expanduser(args.oracle_json)
    traj_csv = os.path.expanduser(args.traj_csv)
    out_png = os.path.expanduser(args.out_png)
    out_metrics = os.path.expanduser(args.out_metrics) if args.out_metrics else ""

    meta = parse_map_yaml(map_yaml)

    img = mpimg.imread(meta["map_png"])
    h = img.shape[0]

    oracle_pts = load_oracle_json(oracle_json)
    exec_pts = load_traj_csv(traj_csv)

    oracle_xy = [world_to_pixel(x, y, meta, h) for (x, y) in oracle_pts]
    exec_xy = [world_to_pixel(x, y, meta, h) for (x, y) in exec_pts]

    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    plt.figure(figsize=(10, 10))
    plt.imshow(img)

    # Explicit colors + thinner lines
    if oracle_xy:
        ox, oy = zip(*oracle_xy)
        plt.plot(ox, oy, linewidth=ORACLE_LW, color=ORACLE_COLOR, label="Oracle path")
    if exec_xy:
        ex, ey = zip(*exec_xy)
        plt.plot(ex, ey, linewidth=EXEC_LW, color=EXEC_COLOR, label="Executed path")

    if args.legend:
        # Put legend in a corner with a semi-transparent background
        plt.legend(loc="upper right", framealpha=0.75)

    plt.axis("off")
    plt.tight_layout(pad=0)
    plt.savefig(out_png, dpi=200, bbox_inches="tight", pad_inches=0)
    plt.close()

    oracle_len = polyline_length(oracle_pts)
    exec_len = polyline_length(exec_pts)
    ratio = (exec_len / oracle_len) if oracle_len > 1e-9 else None
    dev = deviation_stats_point_to_segment(exec_pts, oracle_pts)

    metrics = {
        "map_yaml": meta["map_yaml"],
        "map_png": meta["map_png"],
        "resolution": meta["resolution"],
        "origin": [meta["origin_x"], meta["origin_y"], meta["origin_yaw"]],
        "oracle": {"points": len(oracle_pts), "length_m": float(oracle_len)},
        "executed": {"points": len(exec_pts), "length_m": float(exec_len)},
        "length_ratio_executed_over_oracle": float(ratio) if ratio is not None else None,
        "deviation_to_oracle_m": dev,
        "deviation_method": "point-to-segment (each executed point to nearest oracle segment)",
    }

    if out_metrics:
        os.makedirs(os.path.dirname(out_metrics), exist_ok=True)
        with open(out_metrics, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2)

    print(f"[make_overlay] wrote: {out_png}")
    if out_metrics:
        print(f"[make_overlay] wrote: {out_metrics}")
    print(f"[make_overlay] oracle_len={oracle_len:.3f}m exec_len={exec_len:.3f}m ratio={ratio if ratio is not None else 'NA'}")
    if dev["mean_m"] is not None:
        print(f"[make_overlay] deviation mean={dev['mean_m']:.3f}m max={dev['max_m']:.3f}m")


if __name__ == "__main__":
    main()
