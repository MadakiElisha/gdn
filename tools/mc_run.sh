#!/usr/bin/env bash
# TEST-012 Monte Carlo campaign.
# Usage: tools/mc_run.sh [nseeds] [csv_dir] [map.bin]
#   tools/mc_run.sh 100                                         # flight001 on sigma4 (needs fetch_data.sh)
#   tools/mc_run.sh 24 data/flight_zermatt_csv data/maps/zermatt.bin   # Zermatt (make_data.sh)
#   tools/mc_run.sh 100 data/flight001_csv data/maps/terrain_db_sigma4.bin --model=agl --baro-drift=15
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GDN_ROOT="${GDN_ROOT:-$ROOT}"
BIN="$ROOT/install/gdn_fusion/lib/gdn_fusion/mc_campaign"
[ -x "$BIN" ] || { echo "build first: colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release"; exit 1; }
# Anything after the third argument goes straight to mc_campaign (e.g. --model=agl
# --baro-drift=15 --window=60,600); run mc_campaign with a bad option to see the list.
"$BIN" "${1:-100}" "${2:-data/flight001_csv}" "${3:-data/maps/terrain_db_sigma4.bin}" "${@:4}" | tee "${TMPDIR:-/tmp}/mc_report.txt"
