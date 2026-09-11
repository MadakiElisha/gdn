#!/usr/bin/env bash
# Deterministic replay bench: node + sensor-sim + scorer + full-bag playback.
set -u
cd ~/gdn_workspace
source env.sh >/dev/null; source install/setup.bash >/dev/null
pkill -f eskf_node; pkill -f trn_sensor_sim; pkill -f replay_scorer; pkill -f "ros2 bag"; sleep 2
ros2 run gdn_fusion eskf_node 2>&1 | tee /tmp/bench_node.log & NPID=$!
python3 tools/trn_sensor_sim.py 2>&1 | tee /tmp/bench_sim.log & SPID=$!
python3 -u tools/replay_scorer.py 2>&1 | tee /tmp/bench_scorer.log & CPID=$!
for i in $(seq 1 30); do
  [ "$(ros2 node list 2>/dev/null | grep -c '/gdn_')" -eq 3 ] && break; sleep 1
done
ros2 node list
ros2 bag play data/flight001
kill -INT $CPID 2>/dev/null; sleep 2
kill $NPID $SPID 2>/dev/null; pkill -f "ros2 bag" 2>/dev/null
echo "=== logs: /tmp/bench_node.log /tmp/bench_sim.log /tmp/bench_scorer.log ==="
