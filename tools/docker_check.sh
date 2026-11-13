#!/usr/bin/env bash
# Check the GDN dev image (R2.1): is it the pinned toolchain, and does the whole
# simulation loop run in it on this machine?
#
#   tools/docker_check.sh            # CPU only: versions, build, selftest, world, a short SITL flight
#   tools/docker_check.sh --gpu      # also: the NVIDIA GPU in the container, the rendered radar altimeter
#   tools/docker_check.sh --quick    # skip the SITL flight (about 2 minutes instead of 8)
#
# Run on the host. It runs the checks inside the image through tools/docker_dev.sh, and
# prints one PASS or FAIL line per check. Exit 0 only if every check passed.
# Logs: log/docker_check/.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 2
GPU=""; QUICK=""; INSIDE=""
for a in "$@"; do
  case "$a" in
    --gpu) GPU=1 ;;
    --quick) QUICK=1 ;;
    --inside) INSIDE=1 ;;
    *) echo "unknown option $a"; exit 2 ;;
  esac
done
LOGD="$ROOT/log/docker_check"
mkdir -p "$LOGD"
FAILS=0
check() {   # check "what" command...
  local what="$1"; shift
  if "$@" >> "$LOGD/check.log" 2>&1; then echo "PASS  $what"; else echo "FAIL  $what"; FAILS=$((FAILS + 1)); fi
}
want() {    # want "what" expected actual
  if [ "$2" = "$3" ]; then echo "PASS  $1 ($3)"; else echo "FAIL  $1: expected $2, found ${3:-nothing}"; FAILS=$((FAILS + 1)); fi
}

# ------------------------------------------------------------------- host side ----
if [ -z "$INSIDE" ]; then
  : > "$LOGD/check.log"
  if [ ! -f docker/Dockerfile ] || [ ! -f docker/versions.env ] || [ ! -x tools/docker_dev.sh ]; then
    echo "FAIL  docker/Dockerfile, docker/versions.env and tools/docker_dev.sh exist"
    echo "DOCKER CHECK FAILED (1 check)"; exit 1
  fi
  # shellcheck disable=SC1091
  . docker/versions.env
  IMAGE="$(tools/docker_dev.sh image)"
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "FAIL  image $IMAGE is built (run: tools/docker_dev.sh build)"
    echo "DOCKER CHECK FAILED (1 check)"; exit 1
  fi
  echo "PASS  image $IMAGE is built"
  want "image is built from the current docker/ files" "$(tools/docker_dev.sh hash)" \
    "$(docker image inspect "$IMAGE" --format '{{index .Config.Labels "gdn.docker_hash"}}')"
  want "image base is the pinned digest" "$BASE_IMAGE" \
    "$(docker image inspect "$IMAGE" --format '{{index .Config.Labels "gdn.base_image"}}')"
  tools/docker_dev.sh run ${GPU:+--gpu} -- tools/docker_check.sh --inside ${GPU:+--gpu} ${QUICK:+--quick}
  rc=$?
  [ $rc -eq 0 ] && [ $FAILS -eq 0 ] && { echo "DOCKER CHECK PASSED"; exit 0; }
  echo "DOCKER CHECK FAILED (see $LOGD/)"; exit 1
fi

# ----------------------------------------------------------------- inside image ----
# shellcheck disable=SC1091
. "$ROOT/docker/versions.env"
# shellcheck disable=SC1091
. /etc/os-release
want "Ubuntu 24.04" "noble" "${VERSION_CODENAME:-}"
want "ROS 2 distribution" "jazzy" "${ROS_DISTRO:-}"
want "Gazebo Harmonic (gz sim 8)" "8" "$(gz sim --versions 2>/dev/null | head -1 | cut -d. -f1)"
want "PX4 commit ($PX4_TAG)" "$PX4_COMMIT" "$(git -C "$PX4_DIR" rev-parse HEAD 2>/dev/null)"
check "PX4 SITL is built" test -x "$PX4_DIR/build/px4_sitl_default/bin/px4"
want "Micro XRCE-DDS Agent ($XRCE_TAG)" "$XRCE_COMMIT" "$(cat /opt/gdn/xrce_commit 2>/dev/null)"
check "MicroXRCEAgent is installed" command -v MicroXRCEAgent
check "ros_gz_bridge is installed" ros2 pkg prefix ros_gz_bridge
check "Gazebo Python bindings import" python3 -c "import gz.sim8, gz.math7, gz.transport13"
check "the workspace is the host's (same path)" test -f "$ROOT/env.sh"
check "files written in the workspace belong to the host user" \
  bash -c "f=\$(mktemp -p '$LOGD'); [ \"\$(stat -c %u \$f)\" = \"\${GDN_HOST_UID:-x}\" ]; r=\$?; rm -f \$f; exit \$r"
check "the container's build is kept apart from the host's" \
  bash -c "grep -q '$ROOT/build ' /proc/self/mounts && grep -q '$ROOT/install ' /proc/self/mounts"

echo "      building GDN in the container (colcon, Release) ..."
check "GDN builds (colcon, Release)" \
  bash -c "cd '$ROOT' && colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release"
# shellcheck disable=SC1091
set +u; source "$ROOT/env.sh" >/dev/null 2>&1; set -u
check "selftest passes (tools/run_selftest.sh)" "$ROOT/tools/run_selftest.sh"
check "the Zermatt world is generated" python3 "$ROOT/tools/make_sim_world.py"
check "world physics check (tools/check_sim_world.py)" python3 "$ROOT/tools/check_sim_world.py"

if [ -n "$GPU" ]; then
  check "the NVIDIA GPU is visible in the container (nvidia-smi)" nvidia-smi
  check "the rendered radar altimeter agrees with the map (tools/check_sim_radar.py, GPU)" \
    python3 "$ROOT/tools/check_sim_radar.py"
fi

if [ -z "$QUICK" ]; then
  echo "      flying a short SITL route headless (PX4 + Gazebo + GDN, about 6 minutes) ..."
  SL="$LOGD/sitl"
  rm -rf "$SL"
  n_runs_before=$(find "$ROOT/log/gdn_runs" -maxdepth 1 -name 'gdn_*' 2>/dev/null | wc -l)
  HEADLESS=1 GDN_LOG_DIR="$SL" GDN_FLY_ARGS="--legs 1 --leg-len 300" \
    timeout 900 "$ROOT/tools/sim_run.sh" > "$LOGD/sim_run.log" 2>&1
  check "SITL: PX4 reached over DDS" grep -q "flying" "$LOGD/sim_run.log"
  check "SITL: GDN aligned and set its altitude datum" grep -q "altitude datum" "$SL/gdn.log"
  check "SITL: PX4 flew the route and landed" grep -q "route done, landing" "$SL/fly.log"
  check "SITL: GDN was scored against truth" grep -qE "RMS" "$SL/scorer.log"
  check "SITL: GDN wrote a run log" \
    test "$(find "$ROOT/log/gdn_runs" -maxdepth 1 -name 'gdn_*' 2>/dev/null | wc -l)" -gt "$n_runs_before"
  grep -hE "RMS|MAX" "$SL/scorer.log" 2>/dev/null | sed 's/^/      /'
fi

[ $FAILS -eq 0 ] && exit 0
echo "      $FAILS check(s) failed; details in $LOGD/check.log"
exit 1
