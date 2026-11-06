#!/usr/bin/env bash
# Generate the derived data a fresh clone needs, and (optionally) prove the
# tracked maps are reproducible from their sources.
#
#   tools/make_data.sh                 # Zermatt synthetic flight CSVs + checksum check
#   tools/make_data.sh --verify-maps   # also rebuild all maps from source in a scratch
#                                      # dir and compare byte-for-byte with data/maps/
#   tools/make_data.sh --db-maps       # also build the degraded onboard maps (R1.3):
#                                      # data/maps/{zermatt,sigma4}_db_*.bin
# Options combine: tools/make_data.sh --verify-maps --db-maps
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "== Zermatt synthetic flight (data/flight_zermatt_csv) =="
GDN_ROOT="$ROOT" python3 tools/gen_zermatt_flight.py

echo "== Checksums (data/SHA256SUMS) =="
(cd data && sha256sum -c SHA256SUMS)

VERIFY=0; DBMAPS=0
for arg in "$@"; do
  case "$arg" in
    --verify-maps) VERIFY=1 ;;
    --db-maps) DBMAPS=1 ;;
    *) echo "unknown option: $arg (use --verify-maps and/or --db-maps)"; exit 1 ;;
  esac
done

if [ "$VERIFY" = 1 ]; then
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
if [ "$DBMAPS" = 1 ]; then
  # Onboard maps that differ from the truth world (R1.3, REQ-DB-002). Fixed seeds, so
  # they're reproducible; generated, not tracked. "srtm" combines all three errors at
  # roughly a 3 arc-second SRTM level against a 1 arc-second truth: 3x coarser cells,
  # 6 m correlated noise over 200 m patches, 3 m bias. See TEST-020.
  echo "== Degraded onboard maps (data/maps/*_db_*.bin) =="
  for base in zermatt:data/maps/zermatt.bin sigma4:data/maps/terrain_db_sigma4.bin; do
    name="${base%%:*}"; src="${base#*:}"
    while read -r tag opts; do
      # shellcheck disable=SC2086
      GDN_ROOT="$ROOT" python3 tools/make_db_map.py --in "$src" --out "data/maps/${name}_db_${tag}.bin" $opts
    done <<'MAPS'
noise5 --noise 5 --corr 150 --seed 1
noise10 --noise 10 --corr 150 --seed 2
bias10 --bias 10
coarse3 --coarsen 3
srtm --coarsen 3 --noise 6 --corr 200 --bias 3 --seed 3
MAPS
  done
fi
echo "done."
