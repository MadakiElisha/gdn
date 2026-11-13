#!/bin/bash
# retry.sh [-n N] [-c DIR] CMD...  Run CMD up to N times (default 5), 20 s apart.
# -c DIR: remove DIR before each new attempt (a clone that died half way).
# The image downloads several GB from GitHub and apt mirrors; on a slow or unstable
# connection a long transfer can be cut (git: "RPC failed ... early EOF").
n=5; clean=""
while [ $# -gt 0 ]; do
  case "$1" in
    -n) n="$2"; shift 2 ;;
    -c) clean="$2"; shift 2 ;;
    *) break ;;
  esac
done
for i in $(seq 1 "$n"); do
  [ -n "$clean" ] && rm -rf "$clean"
  "$@" && exit 0
  echo "retry.sh: attempt $i of $n failed: $*" >&2
  [ "$i" -lt "$n" ] && sleep 20
done
exit 1
