#!/usr/bin/env bash
# Compile + run the ROS-free core unit tests under -O2 and -O3 (dual-build gate,
# see TEST-011). Needs only g++ and Eigen; no ROS. Warnings are errors (R1.9).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EIGEN_INC="${EIGEN_INC:-$(pkg-config --cflags-only-I eigen3 2>/dev/null | sed 's/^-I//;s/ *$//')}"
EIGEN_INC="${EIGEN_INC:-/usr/include/eigen3}"
[ -f "$EIGEN_INC/Eigen/Dense" ] || { echo "Eigen not found (apt install libeigen3-dev, or set EIGEN_INC)"; exit 1; }
export GDN_ROOT="${GDN_ROOT:-$ROOT}"
OUT="${TMPDIR:-/tmp}/gdn_selftest"; mkdir -p "$OUT"
rc=0
for opt in -O2 -O3; do
  echo "=== selftest $opt ==="
  g++ -std=c++17 "$opt" -Wall -Wextra -Wpedantic -Werror -Wno-maybe-uninitialized \
      -I "$ROOT/src/gdn_fusion/include" -isystem "$EIGEN_INC" \
      "$ROOT/src/gdn_fusion/src/selftest.cpp" -o "$OUT/selftest$opt"
  "$OUT/selftest$opt" || rc=1
done
exit $rc
