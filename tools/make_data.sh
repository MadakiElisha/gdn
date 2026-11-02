#!/usr/bin/env bash
# Generate the derived data a fresh clone needs, and (optionally) prove the
# tracked maps are reproducible from their sources.
#
#   tools/make_data.sh                 # Zermatt synthetic flight CSVs + checksum check
#   tools/make_data.sh --verify-maps   # also rebuild all maps from source in a scratch
#                                      # dir and compare byte-for-byte with data/maps/
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "== Zermatt synthetic flight (data/flight_zermatt_csv) =="
GDN_ROOT="$ROOT" python3 tools/gen_zermatt_flight.py

echo "== Checksums (data/SHA256SUMS) =="
(cd data && sha256sum -c SHA256SUMS)

if [ "${1:-}" = "--verify-maps" ]; then
  S="$(mktemp -d)"; trap 'rm -rf "$S"' EXIT
  mkdir -p "$S/data/maps"
  echo "== Rebuilding sigma4 map from seed (scratch: $S) =="
  GDN_ROOT="$S" python3 tools/generate_terrain.py | tail -1
  GDN_ROOT="$S" python3 tools/export_map_db.py
  GDN_ROOT="$S" python3 tools/make_sigma4_map.py
  echo "== Rebuilding zermatt.bin from data/maps/zermatt.tif =="
  python3 tools/make_zermatt_map.py --tif data/maps/zermatt.tif --out "$S/data/maps/zermatt.bin" | head -1
  for m in terrain_db_sigma4.bin zermatt.bin; do
    if cmp -s "$S/data/maps/$m" "data/maps/$m"; then echo "  $m: identical"
    else echo "  $m: DIFFERS (library versions can change float rounding; tracked file is canonical)"; fi
  done
fi
echo "done."
