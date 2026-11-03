#!/usr/bin/env bash
# One-time setup for closed-loop SITL (roadmap Phase 2, Stage A) on Ubuntu 24.04.
# Installs PX4 v1.16.2 next to (not inside) the GDN workspace, Gazebo Harmonic,
# the ROS-Gazebo bridge and the Micro XRCE-DDS Agent, then builds PX4 SITL.
#
#   tools/setup_sim.sh                         # PX4 goes to ~/PX4-Autopilot-1.16
#   PX4_DIR=/somewhere/else tools/setup_sim.sh
#
# Safe to re-run. Takes 20 to 40 minutes the first time (PX4 clone and build).
# An older PX4 checkout (e.g. ~/PX4-Autopilot at v1.15) is left untouched.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.16}"
PX4_TAG="${PX4_TAG:-v1.16.2}"
XRCE_TAG="v2.4.2"      # the version PX4's uXRCE-DDS docs use
log() { printf '\n\033[1;34m[gdn-sim-setup] %s\033[0m\n' "$*"; }

# shellcheck disable=SC1091
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || { echo "Needs Ubuntu 24.04 (found ${PRETTY_NAME:-?})"; exit 1; }

log "1/6 PX4 $PX4_TAG source in $PX4_DIR"
if [ ! -d "$PX4_DIR/.git" ]; then
  git clone --branch "$PX4_TAG" --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
  echo "already there: $(git -C "$PX4_DIR" describe --tags 2>/dev/null)"
fi

log "2/6 PX4 dependencies and Gazebo Harmonic (PX4's own script; asks for sudo)"
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

log "3/6 ROS-Gazebo bridge and Gazebo Python bindings"
sudo apt-get install -y ros-jazzy-ros-gz-bridge python3-gz-sim8 python3-gz-math7 || \
  sudo apt-get install -y ros-jazzy-ros-gz-bridge

log "4/6 Micro XRCE-DDS Agent $XRCE_TAG"
if ! command -v MicroXRCEAgent >/dev/null; then
  T="$(mktemp -d)"
  git clone -b "$XRCE_TAG" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$T/agent"
  cmake -S "$T/agent" -B "$T/agent/build"
  cmake --build "$T/agent/build" -j"$(nproc)"
  sudo cmake --install "$T/agent/build"
  sudo ldconfig /usr/local/lib/
  rm -rf "$T"
else
  echo "already installed: $(command -v MicroXRCEAgent)"
fi

log "5/6 Build PX4 SITL (no simulator launch)"
( cd "$PX4_DIR" && make px4_sitl )

log "6/6 GDN world for Zermatt, and its physics check"
cd "$ROOT"
python3 tools/make_sim_world.py
if python3 -c "import gz.sim8, gz.math7" 2>/dev/null; then
  python3 tools/check_sim_world.py || echo "world check FAILED: see output above"
else
  echo "(gz Python bindings not available, skipping tools/check_sim_world.py)"
fi

log "Done. To fly:  cd $ROOT && GDN_GPU=nvidia source env.sh && tools/sim_run.sh"
echo "If the workspace isn't rebuilt since the px4_msgs 1.16 move, first run:"
echo "  colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release"
