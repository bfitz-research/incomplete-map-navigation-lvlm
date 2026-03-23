# Overlay images and metrics (how to)

This generates paper-ready artifacts per run folder:
`notes/<env>/runs/<MODE>/<timestamp>/`

It creates:
- `overlay.png`: static map + oracle path + executed path (labeled)
- `metrics.json`: lengths + ratio + deviation

## Colors / style (fixed, not from RViz)
- Oracle path: **purple**
- Executed path: **orange**
- Lines are thin so overlaps are readable.
- Legend is enabled with `--legend`.

## Single-run overlay command (office example)
Set the run directory first:
```
RUN_DIR=~/table_nav2/notes/env_office/runs/nav2_global_updates_off/<timestamp>
```

Then:
```
python3 ~/table_nav2/scripts/make_overlay.py \
  --map-yaml ~/table_nav2/maps/env_office/test/map.yaml \
  --oracle-json ~/table_nav2/notes/env_office/oracle/oracle_path.json \
  --traj-csv "$RUN_DIR/traj.csv" \
  --out-png "$RUN_DIR/overlay.png" \
  --out-metrics "$RUN_DIR/metrics.json" \
  --legend
```

## Batch overlays for the latest 10 runs in a MODE folder
Example:
```
MODE=nav2_global_updates_off
MAP=~/table_nav2/maps/env_office/test/map.yaml
ORACLE=~/table_nav2/notes/env_office/oracle/oracle_path.json

for d in $(ls -td ~/table_nav2/notes/env_office/runs/$MODE/* | head -10); do
  python3 ~/table_nav2/scripts/make_overlay.py \
    --map-yaml "$MAP" \
    --oracle-json "$ORACLE" \
    --traj-csv "$d/traj.csv" \
    --out-png "$d/overlay.png" \
    --out-metrics "$d/metrics.json" \
    --legend
done
```

## Metric definitions
### Path lengths
- `oracle.length_m`: sum of distances along the oracle polyline (ComputePathToPose result)
- `executed.length_m`: sum of distances along the logged executed trajectory polyline

### Length ratio
- `length_ratio_executed_over_oracle = executed_length / oracle_length`

### Deviation (robust)
- `deviation_to_oracle_m` is **point-to-segment**:
  - For each executed point, compute distance to the nearest segment of the oracle polyline.
  - Report mean and max deviation (meters).
