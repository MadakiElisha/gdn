#!/usr/bin/env bash
# Replay bench + RViz (terrain mesh + estimated path). Same env knobs as bench.sh.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
# shellcheck disable=SC1091
source "$ROOT/env.sh" >/dev/null 2>&1 || true
BAG="${GDN_BAG:-data/flight001}"
export GDN_MAP="${GDN_MAP:-$ROOT/data/maps/terrain_db_sigma4.bin}"
LOG="${GDN_LOG_DIR:-/tmp/gdn_bench}"; mkdir -p "$LOG"
[ -d "$BAG" ] || { echo "bag not found: $BAG  (run tools/fetch_data.sh)"; exit 1; }
command -v rviz2 >/dev/null || { echo "rviz2 missing: sudo apt install ros-jazzy-rviz2"; exit 1; }

pkill -f eskf_node; pkill -f trn_sensor_sim; pkill -f replay_scorer; pkill -f map_viz
pkill -f mag_sim; pkill -f "ros2 bag play"; sleep 2

ros2 run gdn_fusion eskf_node --ros-args -p map_path:="$GDN_MAP" > "$LOG/node.log" 2>&1 & NPID=$!
python3 tools/trn_sensor_sim.py > "$LOG/sim.log" 2>&1 & SPID=$!
python3 tools/mag_sim.py > "$LOG/mag.log" 2>&1 & MPID=$!
python3 -u tools/replay_scorer.py > "$LOG/scorer.log" 2>&1 & CPID=$!
ros2 run gdn_fusion map_viz_node --ros-args -p map_path:="$GDN_MAP" > "$LOG/viz.log" 2>&1 & VPID=$!

for _ in $(seq 1 30); do
  [ "$(ros2 node list 2>/dev/null | grep -c '/gdn_')" -ge 5 ] && break
  sleep 1
done

echo "Nodes running, starting RViz..."
rviz2 -d "$ROOT/tools/gdn.rviz" & RVIZ_PID=$!

ros2 bag play "$BAG"

kill $RVIZ_PID 2>/dev/null
kill -INT $CPID 2>/dev/null; sleep 2
kill $NPID $SPID $VPID $MPID 2>/dev/null
pkill -f "ros2 bag play" 2>/dev/null

echo ""
echo "=== Scorer summary ==="
grep -h "RMS 60-300s" "$LOG/scorer.log" 2>/dev/null || echo "(no summary; see $LOG/scorer.log)"
echo "=== logs: $LOG/*.log ==="
