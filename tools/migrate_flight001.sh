#!/usr/bin/env bash
# One-time: convert data/flight001 (recorded with px4_msgs 1.15) so it replays with
# the px4_msgs 1.16 the workspace now builds. Keeps the original next to it.
#   source env.sh && tools/migrate_flight001.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
D="$ROOT/data"
if [ -f "$D/flight001/.px4_msgs_version" ]; then
  echo "data/flight001 is already migrated ($(cat "$D/flight001/.px4_msgs_version"))"; exit 0
fi
[ -f "$D/flight001/metadata.yaml" ] || { echo "data/flight001 not found: run tools/fetch_data.sh"; exit 1; }
python3 -c "import px4_msgs.msg" 2>/dev/null || { echo "px4_msgs not importable: build the workspace and source env.sh first"; exit 1; }
python3 "$ROOT/tools/migrate_bag_px4_msgs.py" "$D/flight001" "$D/flight001_v116" --force
rm -rf "$D/flight001_px4_1_15"
mv "$D/flight001" "$D/flight001_px4_1_15"
mv "$D/flight001_v116" "$D/flight001"
echo "OK: data/flight001 now uses px4_msgs 1.16 (original kept in data/flight001_px4_1_15)"
