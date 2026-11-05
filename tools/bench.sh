#!/usr/bin/env bash
# Replay bench: eskf_node + trn_sensor_sim + mag_sim + [baro_sim] + replay_scorer
# over a recorded PX4 bag, then print horizontal RMS/MAX in the 60-300 s window.
#
# Env knobs:
#   BENCH_NO_BARO=1   disable the baro sim (A/B isolation of OI-002 coupling)
#   GDN_BAG=path      bag to replay        (default data/flight001)
#   GDN_MAP=path      terrain DB for node + sim (default data/maps/terrain_db_sigma4.bin)
#   GDN_LOG_DIR=dir   where logs go        (default /tmp/gdn_bench)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
# shellcheck disable=SC1091
source "$ROOT/env.sh" >/dev/null 2>&1 || true
BAG="${GDN_BAG:-data/flight001}"
export GDN_MAP="${GDN_MAP:-$ROOT/data/maps/terrain_db_sigma4.bin}"
LOG="${GDN_LOG_DIR:-/tmp/gdn_bench}"; mkdir -p "$LOG"
[ -d "$BAG" ] || { echo "bag not found: $BAG  (run tools/fetch_data.sh)"; exit 1; }
# Bags recorded with px4_msgs 1.15 don't decode with the 1.16 build (VehicleLocalPosition
# changed layout). Migrated or newly generated bags carry a marker file.
if [ ! -f "$BAG/.px4_msgs_version" ] && [ "${GDN_SKIP_BAG_CHECK:-0}" != "1" ]; then
  echo "$BAG has no .px4_msgs_version marker, so it's probably a px4_msgs 1.15 recording."
  echo "Migrate it first: tools/migrate_flight001.sh (or tools/migrate_bag_px4_msgs.py IN OUT)."
  echo "Set GDN_SKIP_BAG_CHECK=1 if you know it's fine."
  exit 1
fi
[ -f "$GDN_MAP" ] || { echo "map not found: $GDN_MAP  (run tools/make_data.sh)"; exit 1; }
command -v ros2 >/dev/null || { echo "ros2 not on PATH - source env.sh"; exit 1; }

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
gdn_stop -9; sleep 2

ros2 run gdn_fusion eskf_node --ros-args -p map_path:="$GDN_MAP" > "$LOG/node.log" 2>&1 & NPID=$!
python3 tools/trn_sensor_sim.py > "$LOG/sim.log" 2>&1 & SPID=$!
BPID=
if [ "${BENCH_NO_BARO:-0}" != "1" ]; then
  python3 tools/baro_sim.py > "$LOG/baro.log" 2>&1 & BPID=$!
fi
python3 tools/mag_sim.py > "$LOG/mag.log" 2>&1 & MPID=$!
python3 -u tools/replay_scorer.py > "$LOG/scorer.log" 2>&1 & CPID=$!

for _ in $(seq 1 30); do
  [ "$(ros2 node list 2>/dev/null | grep -c '/gdn_')" -ge 4 ] && break
  sleep 1
done
ros2 node list

ros2 bag play "$BAG"

kill -INT $CPID 2>/dev/null; sleep 2
kill $NPID $SPID $BPID $MPID 2>/dev/null
gdn_stop; sleep 1; gdn_stop -9

echo ""
echo "=== Scorer summary ==="
awk '/horiz err/ {for(i=1;i<=NF;i++){if($i=="t=")t=$(i+1); if($i=="err")e=$(i+1)}
     if(t>=60&&t<=300){s+=e*e;n++; if(e>m)m=e}}
     END{if(n>0) printf "RMS 60-300s: %.1f m  MAX: %.1f m  (n=%d)\n", sqrt(s/n), m, n;
          else print "no scorer samples"}' "$LOG/scorer.log" 2>/dev/null \
  || echo "(scorer log not available)"
grep -h "RMS 60-300s" "$LOG/scorer.log" 2>/dev/null
echo "=== logs: $LOG/{node,sim,baro,mag,scorer}.log ==="
