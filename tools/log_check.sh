#!/usr/bin/env bash
# R1.11 run-log check (TEST-029), as CI runs it:
#   1. the bench flies the Zermatt bag through eskf_node, which writes its run log;
#   2. gdn_replay replays the log and must find every state, update and event identical;
#   3. it must also be able to fail: a copy of the log with one state value changed by
#      1 ulp must be reported DIFFERENT, and the wrong map refused;
#   4. a what-if replay (--set) writes a new log, which itself replays identically.
# Usage: tools/log_check.sh            (needs a colcon build and data/flight_zermatt)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2
# shellcheck disable=SC1091
source "$ROOT/env.sh" >/dev/null 2>&1
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
set -u   # after the ROS setup scripts, which read unset variables
OUT="${TMPDIR:-/tmp}/gdn_log_check"
rm -rf "$OUT"; mkdir -p "$OUT"
REPLAY="$ROOT/install/gdn_fusion/lib/gdn_fusion/gdn_replay"
[ -x "$REPLAY" ] || { echo "build first (gdn_replay not found)"; exit 2; }
rc=0
pass() { echo "PASS  log: $*"; }
fail() { echo "FAIL  log: $*"; rc=1; }

# 1. A bench run that writes its log into $OUT/runs.
GDN_BAG=data/flight_zermatt GDN_MAP="$ROOT/data/maps/zermatt.bin" GDN_LOG_DIR="$OUT/bench" GDN_BAG_RATE=2 \
  GDN_NODE_ARGS="-p log_dir:=$OUT/runs" tools/bench.sh > "$OUT/bench.txt" 2>&1
LOG=$(grep -ho "run log: [^ ]*" "$OUT/bench/node.log" | head -1 | cut -d' ' -f3)
if [ -n "$LOG" ] && [ -f "$LOG/metadata.yaml" ]; then pass "the node wrote $LOG ($(du -sh "$LOG" | cut -f1))"
else fail "no run log (see $OUT/bench/node.log)"; tail -20 "$OUT/bench/node.log"; exit 1; fi

# 2. Replay: identical.
"$REPLAY" "$LOG" > "$OUT/replay.txt" 2>&1; r=$?
cat "$OUT/replay.txt"
if [ $r -eq 0 ] && grep -q "^IDENTICAL" "$OUT/replay.txt"; then pass "replay matches the log bit for bit"
else fail "replay differs from the log (exit $r)"; fi

# 3a. One state value changed by 1 ulp: the replay must say DIFFERENT.
python3 - "$LOG" "$OUT/tampered" <<'EOF' > "$OUT/tamper.txt" 2>&1
import math, sys
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
src, dst = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=src, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=dst, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
for i, t in enumerate(r.get_all_topics_and_types()):
    w.create_topic(rosbag2_py.TopicMetadata(id=i, name=t.name, type=t.type, serialization_format="cdr"))
n, done = 0, None
while r.has_next():
    topic, data, ts = r.read_next()
    if topic == "/gdn/log/state":
        n += 1
        if n == 5000:
            m = deserialize_message(data, get_message(types[topic]))
            m.position[0] = math.nextafter(m.position[0], math.inf)
            data, done = serialize_message(m), m.seq
    w.write(topic, data, ts)
print(f"changed north position of the state after input {done} by 1 ulp")
EOF
cat "$OUT/tamper.txt"
"$REPLAY" "$OUT/tampered" > "$OUT/replay_tampered.txt" 2>&1; r=$?
grep -E "^DIFFERENT" "$OUT/replay_tampered.txt"
if [ $r -eq 1 ] && grep -q "pn logged" "$OUT/replay_tampered.txt"; then pass "a 1-ulp change in the log is found"
else fail "a 1-ulp change in the log wasn't found (exit $r)"; fi

# 3b. The wrong map is refused.
"$REPLAY" "$LOG" --map=data/maps/terrain_db_sigma4.bin > "$OUT/replay_map.txt" 2>&1; r=$?
grep -E "^ERROR" "$OUT/replay_map.txt"
if [ $r -eq 2 ] && grep -q "not the map this run used" "$OUT/replay_map.txt"; then pass "the wrong map is refused"
else fail "the wrong map wasn't refused (exit $r)"; fi

# 4. What-if replay, and its own log replays identically.
"$REPLAY" "$LOG" --set=r_agl=200 --out="$OUT/whatif" > "$OUT/whatif.txt" 2>&1; r1=$?
"$REPLAY" "$OUT/whatif" > "$OUT/whatif_replay.txt" 2>&1; r2=$?
grep -E "what if|^replayed" "$OUT/whatif.txt"
if [ $r1 -eq 0 ] && [ $r2 -eq 0 ] && grep -q "^IDENTICAL" "$OUT/whatif_replay.txt"; then
  pass "a what-if replay writes a log that itself replays identically"
else fail "what-if replay (exit $r1, then $r2)"; fi

[ $rc -eq 0 ] && echo "LOG CHECK PASSED" || echo "LOG CHECK FAILED"
exit $rc
