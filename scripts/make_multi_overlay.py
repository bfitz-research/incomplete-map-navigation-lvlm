#!/usr/bin/env python3
"""
make_multi_overlay.py

Create one combined overlay image for multiple run folders, drawing:
- the base map
- the oracle path (blue)
- multiple executed trajectories (different colors)

Each run folder must contain traj.csv.
Outputs:
- combined overlay PNG
- legend/report TXT

Example:
REPO=/path/to/incomplete-map-navigation-lvlm

python3 make_multi_overlay.py \
  --runs-dir "$REPO/notes/env_warehouse/runs/lvlm_global_updates_on" \
  --map-yaml "$REPO/maps/env_warehouse/test/map.yaml" \
  --oracle-json "$REPO/notes/env_warehouse/oracle/oracle_path.json" \
  --limit 5
"""
import argparse
import csv
import json
import math
from pathlib import Path

import yaml
from PIL import Image, ImageDraw


PALETTE = [
    (220, 20, 60),    # crimson
    (255, 140, 0),    # dark orange
    (34, 139, 34),    # forest green
    (138, 43, 226),   # blue violet
    (0, 206, 209),    # dark turquoise
    (255, 0, 255),    # magenta
    (128, 0, 0),      # maroon
    (0, 128, 128),    # teal
    (255, 105, 180),  # hot pink
    (139, 69, 19),    # saddle brown
]

ORACLE_COLOR = (0, 102, 255)


def load_yaml(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def resolve_map_image(map_yaml_path: Path, meta: dict) -> Path:
    img_path = Path(meta["image"])
    if not img_path.is_absolute():
        img_path = map_yaml_path.parent / img_path
    return img_path


def world_to_px(x: float, y: float, meta: dict, img_h: int):
    res = float(meta["resolution"])
    ox, oy, _ = meta["origin"]
    px = int(round((x - ox) / res))
    py = int(round(img_h - 1 - ((y - oy) / res)))
    return px, py


def parse_xy_row(row: dict):
    keys = {k.lower(): k for k in row.keys()}
    x_candidates = ["x", "map_x", "pos_x", "position_x"]
    y_candidates = ["y", "map_y", "pos_y", "position_y"]
    xk = next((keys[k] for k in x_candidates if k in keys), None)
    yk = next((keys[k] for k in y_candidates if k in keys), None)
    if xk is None or yk is None:
        return None
    try:
        x = float(row[xk])
        y = float(row[yk])
        if math.isfinite(x) and math.isfinite(y):
            return x, y
    except Exception:
        return None
    return None


def load_traj_csv(path: Path):
    pts = []
    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            xy = parse_xy_row(row)
            if xy is not None:
                pts.append(xy)
    return pts


def load_oracle(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    poses = data.get("path", {}).get("poses", [])
    pts = []

    for p in poses:
        pos = None

        # Common ROS-style PoseStamped-like structure
        if isinstance(p, dict):
            if "pose" in p and isinstance(p["pose"], dict):
                if "position" in p["pose"]:
                    pos = p["pose"]["position"]
                elif "pose" in p["pose"] and isinstance(p["pose"]["pose"], dict) and "position" in p["pose"]["pose"]:
                    pos = p["pose"]["pose"]["position"]

            # Sometimes position is directly present
            if pos is None and "position" in p:
                pos = p["position"]

            # Sometimes x/y are directly on the pose record
            if pos is None and "x" in p and "y" in p:
                pos = p

        if pos is None:
            continue

        try:
            x = float(pos["x"])
            y = float(pos["y"])
            if math.isfinite(x) and math.isfinite(y):
                pts.append((x, y))
        except Exception:
            continue

    return pts


def draw_polyline(draw: ImageDraw.ImageDraw, px_pts, color, width=2):
    if len(px_pts) >= 2:
        draw.line(px_pts, fill=color, width=width)
    elif len(px_pts) == 1:
        x, y = px_pts[0]
        r = max(1, width)
        draw.ellipse((x-r, y-r, x+r, y+r), fill=color)


def run_sort_key(path: Path):
    try:
        return path.stat().st_mtime
    except Exception:
        return 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs-dir", required=True, help="Folder containing run subfolders with traj.csv")
    ap.add_argument("--map-yaml", required=True)
    ap.add_argument("--oracle-json", required=True)
    ap.add_argument("--limit", type=int, default=5, help="How many newest run folders to include")
    ap.add_argument("--sort", choices=["newest", "oldest", "name"], default="newest")
    ap.add_argument("--out-png", default=None, help="Output PNG path")
    ap.add_argument("--out-legend", default=None, help="Output TXT legend path")
    args = ap.parse_args()

    runs_dir = Path(args.runs_dir).expanduser().resolve()
    map_yaml = Path(args.map_yaml).expanduser().resolve()
    oracle_json = Path(args.oracle_json).expanduser().resolve()

    if not runs_dir.is_dir():
        raise SystemExit(f"runs-dir not found: {runs_dir}")
    if not map_yaml.is_file():
        raise SystemExit(f"map-yaml not found: {map_yaml}")
    if not oracle_json.is_file():
        raise SystemExit(f"oracle-json not found: {oracle_json}")

    if args.out_png:
        out_png = Path(args.out_png).expanduser().resolve()
    else:
        out_png = runs_dir / "all_runs_overlay.png"

    if args.out_legend:
        out_legend = Path(args.out_legend).expanduser().resolve()
    else:
        out_legend = runs_dir / "all_runs_overlay_legend.txt"

    meta = load_yaml(map_yaml)
    map_img_path = resolve_map_image(map_yaml, meta)
    if not map_img_path.is_file():
        raise SystemExit(f"map image not found: {map_img_path}")

    base_img = Image.open(map_img_path).convert("RGB")
    img = base_img.copy()
    draw = ImageDraw.Draw(img)
    img_w, img_h = img.size


    run_dirs = [d for d in runs_dir.iterdir() if d.is_dir() and (d / "traj.csv").exists()]

    if args.sort == "newest":
        run_dirs = sorted(run_dirs, key=run_sort_key, reverse=True)
    elif args.sort == "oldest":
        run_dirs = sorted(run_dirs, key=run_sort_key)
    else:
        run_dirs = sorted(run_dirs, key=lambda p: p.name)

    if args.limit > 0:
        run_dirs = run_dirs[:args.limit]

    legend_lines = []
    legend_lines.append(f"Base map: {map_img_path}")
    legend_lines.append(f"Oracle:   {oracle_json} (color={ORACLE_COLOR})")
    legend_lines.append(f"Runs dir:  {runs_dir}")
    legend_lines.append(f"Included runs: {len(run_dirs)}")
    legend_lines.append("")

    color_idx = 0
    for run_dir in run_dirs:
        traj_path = run_dir / "traj.csv"
        pts = load_traj_csv(traj_path)
        if not pts:
            legend_lines.append(f"{run_dir.name}: NO_VALID_POINTS")
            continue

        px_pts = [world_to_px(x, y, meta, img_h) for x, y in pts]
        color = PALETTE[color_idx % len(PALETTE)]
        color_idx += 1

        draw_polyline(draw, px_pts, color=color, width=2)

        # start marker: circle
        sx, sy = px_pts[0]
        # end marker: square
        ex, ey = px_pts[-1]
        r = 3
        draw.ellipse((sx-r, sy-r, sx+r, sy+r), fill=color)
        draw.rectangle((ex-r, ey-r, ex+r, ey+r), fill=color)

        legend_lines.append(
            f"{run_dir.name}: color={color}, points={len(px_pts)}, start={pts[0]}, end={pts[-1]}"
        )
        
    oracle_pts = load_oracle(oracle_json)
    oracle_px = [world_to_px(x, y, meta, img_h) for x, y in oracle_pts]
    draw_polyline(draw, oracle_px, ORACLE_COLOR, width=3)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    out_legend.parent.mkdir(parents=True, exist_ok=True)

    img.save(out_png)
    with open(out_legend, "w", encoding="utf-8") as f:
        f.write("\n".join(legend_lines) + "\n")

    print(f"Saved overlay: {out_png}")
    print(f"Saved legend:  {out_legend}")


if __name__ == "__main__":
    main()
