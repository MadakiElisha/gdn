#!/usr/bin/env bash
# Download Livox's measured Mid-360 scan pattern (roadmap R2.4c) into data/livox/, pinned
# by commit and checksum: Livox-SDK/livox_laser_simulation scan_mode/mid360.csv (MIT),
# 800,000 beam directions in firing order, 16.8 MB. tools/sim_run.sh calls this when the
# file is missing; sim_sensors fires the simulated Mid-360S's points in this pattern.
#
#   tools/fetch_livox_pattern.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMMIT=cf764140fe9a3bf79650c6705f1ac60a99246cf3
SHA256=aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a
OUT="$ROOT/data/livox/mid360.csv"
if [ -f "$OUT" ] && echo "$SHA256  $OUT" | sha256sum -c --status; then
  echo "data/livox/mid360.csv already present"; exit 0
fi
mkdir -p "$(dirname "$OUT")"
URL="https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/$COMMIT/scan_mode/mid360.csv"
echo "Downloading $URL ..."
curl -fL --retry 3 -o "$OUT.part" "$URL"
echo "$SHA256  $OUT.part" | sha256sum -c --status || { echo "checksum mismatch for $URL" >&2; rm -f "$OUT.part"; exit 1; }
mv "$OUT.part" "$OUT"
echo "OK: data/livox/mid360.csv (Livox-SDK/livox_laser_simulation@${COMMIT:0:7}, MIT)"
