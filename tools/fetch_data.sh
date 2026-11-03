#!/usr/bin/env bash
# Download the recorded PX4 SITL flight (flight001: rosbag + exported CSVs,
# ~100 MB) from the GitHub release and verify its checksum. This is the only
# data that cannot be regenerated from the repo.
#
#   tools/fetch_data.sh                      # default release tag
#   GDN_DATA_TAG=data-v1 tools/fetch_data.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="${GDN_DATA_TAG:-data-v1}"
BASE="https://github.com/MadakiElisha/gdn/releases/download/$TAG"
ASSET="flight001.tar.gz"
cd "$ROOT/data"
if [ -f flight001/metadata.yaml ] && [ -f flight001_csv/imu.csv ]; then
  echo "flight001 already present in data/ - nothing to do"; exit 0
fi
echo "Downloading $BASE/$ASSET ..."
curl -fL --retry 3 -o "$ASSET" "$BASE/$ASSET" || {
  echo "Download failed. Either the release isn't published yet (owner: run tools/pack_data.sh"
  echo "on the machine that has data/flight001), or copy it over directly, e.g.:"
  echo "  rsync -av OLD_PC:gdn_workspace/data/{flight001,flight001_csv} $ROOT/data/"
  exit 1; }
curl -fL --retry 3 -o "$ASSET.sha256" "$BASE/$ASSET.sha256"
sha256sum -c "$ASSET.sha256"
tar -xzf "$ASSET" && rm -f "$ASSET" "$ASSET.sha256"
echo "OK: data/flight001 and data/flight001_csv ready"
# The release bag was recorded with px4_msgs 1.15. Convert it for the 1.16 build if
# the workspace is built and sourced; otherwise say how (the CSVs need nothing).
if python3 -c "import px4_msgs.msg" 2>/dev/null; then
  "$ROOT/tools/migrate_flight001.sh"
else
  echo "note: before replaying the bag, build, source env.sh and run tools/migrate_flight001.sh"
fi
