#!/usr/bin/env bash
# OWNER ONLY - run once on the machine that has data/flight001 to publish it as
# a GitHub release asset that tools/fetch_data.sh downloads.
#   tools/pack_data.sh            # builds data/flight001.tar.gz + .sha256
#   tools/pack_data.sh --publish  # ...and uploads with the GitHub CLI (gh auth login first)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="${GDN_DATA_TAG:-data-v1}"
cd "$ROOT/data"
[ -d flight001 ] || { echo "data/flight001 not found"; exit 1; }
[ -d flight001_csv ] || { echo "data/flight001_csv missing: python3 tools/export_flight_csv.py"; exit 1; }
tar -czf flight001.tar.gz flight001 flight001_csv
sha256sum flight001.tar.gz > flight001.tar.gz.sha256
ls -lh flight001.tar.gz; cat flight001.tar.gz.sha256
if [ "${1:-}" = "--publish" ]; then
  gh release create "$TAG" flight001.tar.gz flight001.tar.gz.sha256 \
    --repo MadakiElisha/gdn --title "GDN data $TAG" \
    --notes "PX4 SITL replay flight001 (rosbag2 MCAP + exported CSVs). Fetch with tools/fetch_data.sh."
fi
