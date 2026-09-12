#!/usr/bin/env bash
# Bench with visualization
cd ~/gdn_workspace || exit 1
source env.sh >/dev/null
source install/setup.bash >/dev/null
pkill -f eskf_node; pkill -f trn_sensor_sim; pkill -f replay_scorer; pkill -f map_viz; pkill -f "ros2 bag"; sleep 2

ros2 run gdn_fusion eskf_node > /tmp/bench_node.log 2>&1 & NPID=$!
python3 tools/trn_sensor_sim.py > /tmp/bench_sim.log 2>&1 & SPID=$!
python3 -u tools/replay_scorer.py > /tmp/bench_scorer.log 2>&1 & CPID=$!
ros2 run gdn_fusion map_viz_node > /tmp/bench_viz.log 2>&1 & VPID=$!

for i in $(seq 1 30); do
  [ "$(ros2 node list 2>/dev/null | grep -c '/gdn_')" -eq 4 ] && break
  sleep 1
done

echo "Nodes running, starting RViz..."
rviz2 -d ~/gdn_workspace/tools/gdn.rviz &
RVIZ_PID=$!

ros2 bag play data/flight001

kill $RVIZ_PID 2>/dev/null
kill -INT $CPID 2>/dev/null; sleep 2
kill $NPID $SPID $VPID 2>/dev/null
pkill -f "ros2 bag" 2>/dev/null

echo ""
echo "=== Scorer summary ==="
awk '/horiz err/ {for(i=1;i<=NF;i++){if($i=="t=")t=$(i+1); if($i=="err")e=$(i+1)}
     if(t>=60&&t<=300){s+=e*e;n++; if(e>m)m=e}} 
     END{if(n>0) printf "RMS 60-300s: %.1f m  MAX: %.1f m  (n=%d)\\n", sqrt(s/n), m, n}' \
    /tmp/bench_scorer.log 2>/dev/null || echo "(scorer log not available)"
echo "=== logs: /tmp/bench_*.log ==="
