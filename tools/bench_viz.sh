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
if [ ! -f "$BAG/.px4_msgs_version" ] && [ "${GDN_SKIP_BAG_CHECK:-0}" != "1" ]; then
  echo "$BAG looks like a px4_msgs 1.15 recording: run tools/migrate_flight001.sh first"; exit 1
fi
command -v rviz2 >/dev/null || { echo "rviz2 missing: sudo apt install ros-jazzy-rviz2"; exit 1; }

# Stop GDN bench processes. Binaries match by exact process name (-x) and scripts by
# path, so this never hits the shell that launched the bench (pkill -f eskf_node did,
# whenever that shell's command line mentioned eskf_node). The C++ node runs as a child
# of the `ros2 run` wrapper; killing only the wrapper's PID left it running.
gdn_stop() {
  local sig="${1:--TERM}"
  pkill "$sig" -x eskf_node; pkill "$sig" -x map_viz_node
  pkill "$sig" -f "ros2 run gdn_fusion (eskf_node|map_viz_node)"
  pkill "$sig" -f "tools/(trn_sensor_sim|baro_sim|mag_sim|replay_scorer)\.py"
  pkill "$sig" -f "ros2 bag play"
  return 0
}
gdn_stop; sleep 2

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
gdn_stop; sleep 1; gdn_stop -9

echo ""
echo "=== Scorer summary ==="
grep -h "RMS 60-300s" "$LOG/scorer.log" 2>/dev/null || echo "(no summary; see $LOG/scorer.log)"
echo "=== logs: $LOG/*.log ==="
