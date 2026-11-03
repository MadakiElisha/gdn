#!/usr/bin/env bash
# Replay a SITL bag (recorded by tools/sim_run.sh) through a fresh eskf_node and score it
# against the truth in the same bag. For trying filter settings on the exact same flight.
#
#   tools/sim_replay.sh /tmp/gdn_sim/sitl_bag
#   tools/sim_replay.sh /tmp/gdn_sim/sitl_bag -p mag_fusion:=align -p r_agl:=50
#
# Runs on its own ROS domain, so it doesn't disturb a live simulation.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:?usage: tools/sim_replay.sh BAG [eskf_node args]}"; shift
MAP="${GDN_MAP:-$ROOT/data/maps/zermatt.bin}"
MAG_REF="${GDN_MAG_REF:-0.9 22.5 -41.8}"; read -r MAG_N MAG_E MAG_D <<< "$MAG_REF"
OUT="$(mktemp -d)/replay"
export ROS_DOMAIN_ID="${GDN_REPLAY_DOMAIN:-77}"
setsid ros2 run gdn_fusion eskf_node --ros-args -p map_path:="$MAP" -p mag_ref_north:="$MAG_N" \
  -p mag_ref_east:="$MAG_E" -p mag_ref_down:="$MAG_D" "$@" > "$OUT.node.log" 2>&1 &
NP=$!
setsid ros2 bag record -s mcap -o "$OUT" /gdn/odom /gdn/sim/truth_odom > /dev/null 2>&1 &
RP=$!
trap 'kill -9 -- -$NP -$RP 2>/dev/null' EXIT
sleep 5
ros2 bag play "$BAG" --topics /fmu/out/sensor_combined /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_attitude /gdn/sim/truth_odom /gdn/baro_meas /gdn/agl_meas /gdn/mag_meas > /dev/null 2>&1
kill -INT -- -"$RP"; sleep 5
[ -f "$OUT/metadata.yaml" ] || ros2 bag reindex "$OUT" -s mcap > /dev/null 2>&1
python3 "$ROOT/tools/score_sitl_bag.py" "$OUT"
echo "node log: $OUT.node.log"
