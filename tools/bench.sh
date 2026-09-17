#!/usr/bin/env bash
# Replay bench: eskf_node + trn_sensor_sim + [baro_sim] + replay_scorer over flight001.
# BENCH_NO_BARO=1 disables the baro sim (A/B isolation of OI-002 coupling).
cd ~/gdn_workspace || exit 1
source env.sh >/dev/null 2>&1 || true
source install/setup.bash >/dev/null 2>&1
pkill -f eskf_node; pkill -f trn_sensor_sim; pkill -f replay_scorer
pkill -f baro_sim; pkill -f "ros2 bag"; sleep 2

ros2 run gdn_fusion eskf_node > /tmp/bench_node.log 2>&1 & NPID=$!
python3 tools/trn_sensor_sim.py > /tmp/bench_sim.log 2>&1 & SPID=$!
BPID=
if [ "${BENCH_NO_BARO:-0}" != "1" ]; then
  python3 tools/baro_sim.py > /tmp/bench_baro.log 2>&1 & BPID=$!
fi
MPID=
python3 tools/mag_sim.py > /tmp/bench_mag.log 2>&1 & MPID=$!
python3 -u tools/replay_scorer.py > /tmp/bench_scorer.log 2>&1 & CPID=$!

for i in $(seq 1 30); do
  [ "$(ros2 node list 2>/dev/null | grep -c '/gdn_')" -ge 3 ] && break
  sleep 1
done
ros2 node list

ros2 bag play data/flight001

kill -INT $CPID 2>/dev/null; sleep 2
kill $NPID $SPID $BPID $MPID 2>/dev/null
pkill -f "ros2 bag" 2>/dev/null
pkill -f baro_sim.py 2>/dev/null

echo ""
echo "=== Scorer summary ==="
awk '/horiz err/ {for(i=1;i<=NF;i++){if($i=="t=")t=$(i+1); if($i=="err")e=$(i+1)}
     if(t>=60&&t<=300){s+=e*e;n++; if(e>m)m=e}}
     END{if(n>0) printf "RMS 60-300s: %.1f m  MAX: %.1f m  (n=%d)\n", sqrt(s/n), m, n;
          else print "no scorer samples"}' /tmp/bench_scorer.log 2>/dev/null \
  || echo "(scorer log not available)"
echo "=== logs: /tmp/bench_node.log /tmp/bench_sim.log /tmp/bench_baro.log /tmp/bench_scorer.log ==="
