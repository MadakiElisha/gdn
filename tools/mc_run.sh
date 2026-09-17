#!/usr/bin/env bash
# TEST-012 Monte Carlo campaign. Usage: mc_run.sh [nseeds]
cd ~/gdn_workspace || exit 1
N=${1:-100}
BIN=install/gdn_fusion/lib/gdn_fusion/mc_campaign
[ -x "$BIN" ] || { echo "build first: colcon build --packages-select gdn_fusion"; exit 1; }
"$BIN" "$N" | tee /tmp/mc_report.txt
