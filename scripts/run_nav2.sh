#!/usr/bin/env bash
set -e

# ========= CONFIG =========
# Resolve everything relative to this script so you can run it from any directory.
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(realpath "${SCRIPT_DIR}/..")"

MAP_YAML="$(realpath "${ROOT_DIR}/maps/current/map.yaml")"
NAV2_PARAMS="$(realpath "${ROOT_DIR}/config/nav2_params_global_updates_on.yaml")"
RVIZ_CONFIG="$(realpath "${ROOT_DIR}/rviz/nav2.rviz")"

echo "[run_nav2] Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Sanity checks
if [[ ! -f "${MAP_YAML}" ]]; then
  echo "[run_nav2] ERROR: Map YAML not found: ${MAP_YAML}"
  exit 1
fi
if [[ ! -f "${NAV2_PARAMS}" ]]; then
  echo "[run_nav2] ERROR: Nav2 params not found: ${NAV2_PARAMS}"
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "[run_nav2] ERROR: RViz config not found: ${RVIZ_CONFIG}"
  exit 1
fi

echo "[run_nav2] Using map YAML:    ${MAP_YAML}"
echo "[run_nav2] Using Nav2 params: ${NAV2_PARAMS}"
echo "[run_nav2] Using RViz config: ${RVIZ_CONFIG}"

# Start RViz first (helps with the /map QoS timing issue)
echo "[run_nav2] Launching RViz..."
rviz2 -d "${RVIZ_CONFIG}" &
RVIZ_PID=$!

cleanup() {
  echo "[run_nav2] Shutting down..."
  if [[ -n "${NAV2_PID:-}" ]]; then
    kill "${NAV2_PID}" 2>/dev/null || true
  fi
  if [[ -n "${RVIZ_PID:-}" ]]; then
    kill "${RVIZ_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

# Launch Nav2 bringup
echo "[run_nav2] Launching Nav2..."
ros2 launch "${ROOT_DIR}/launch/nav2_bringup.launch.py" \
  use_sim_time:=true \
  map:="${MAP_YAML}" \
  params_file:="${NAV2_PARAMS}" &
NAV2_PID=$!

# Wait for map_server load_map service, then republish the map once.
# This makes RViz reliably show /map even if it subscribes VOLATILE.
echo "[run_nav2] Waiting for /map_server/load_map service..."
for _ in {1..80}; do  # ~16s max (80 * 0.2s)
  if ros2 service list | grep -q '^/map_server/load_map$'; then
    break
  fi
  sleep 0.2
done

if ros2 service list | grep -q '^/map_server/load_map$'; then
  echo "[run_nav2] Republishing /map via /map_server/load_map..."
  # Temporarily disable 'exit on error' for this call only
  set +e
  ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '${MAP_YAML}'}" >/dev/null
  set -e
else
  echo "[run_nav2] WARNING: /map_server/load_map not available; skipping map republish."
fi

# Wait for Nav2 to exit (Ctrl+C will trigger cleanup() via trap)
wait "${NAV2_PID}"
NAV2_RC=$?

exit "${NAV2_RC}"
