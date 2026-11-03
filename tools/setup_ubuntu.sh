#!/usr/bin/env bash
# One-shot setup of a fresh Ubuntu 24.04 machine for GDN. Safe to re-run.
#
#   git clone --recurse-submodules https://github.com/MadakiElisha/gdn.git ~/gdn_workspace
#   cd ~/gdn_workspace && tools/setup_ubuntu.sh            # desktop (RViz included)
#   tools/setup_ubuntu.sh --headless                       # ros-base only, no GUI
#
# Steps: ROS 2 Jazzy apt repo + packages -> build/python deps -> rosdep ->
# submodules -> colcon build -> derived data -> selftest + colcon test.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_PKG=ros-jazzy-desktop
[ "${1:-}" = "--headless" ] && ROS_PKG=ros-jazzy-ros-base
log() { printf '\n\033[1;34m[gdn-setup] %s\033[0m\n' "$*"; }

# shellcheck disable=SC1091
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
  echo "This script targets Ubuntu 24.04 (noble); found ${PRETTY_NAME:-unknown}."
  echo "ROS 2 Jazzy binaries are only built for noble. Use a 24.04 machine/VM/container."
  exit 1
fi

log "1/7 ROS 2 Jazzy apt repository"
if [ ! -f /etc/apt/sources.list.d/ros2.sources ] && [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
else
  echo "already configured"
fi

log "2/7 ROS 2 Jazzy ($ROS_PKG) + build and Python dependencies"
sudo apt-get update
sudo apt-get install -y "$ROS_PKG" ros-dev-tools \
  ros-jazzy-rosbag2-storage-mcap ros-jazzy-rclcpp-components \
  build-essential cmake git libeigen3-dev pkg-config \
  python3-numpy python3-scipy python3-matplotlib python3-h5py \
  python3-yaml python3-pil python3-rasterio

log "3/7 rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro jazzy
# shellcheck disable=SC1091
set +u; source /opt/ros/jazzy/setup.bash; set -u
rosdep install --from-paths "$ROOT/src" --ignore-src -y --rosdistro jazzy

log "4/7 git submodules (px4_msgs)"
git -C "$ROOT" submodule update --init --recursive

log "5/7 colcon build (px4_msgs takes several minutes the first time)"
cd "$ROOT"
# Keep the mode of an existing build: switching an old copy-install tree to
# --symlink-install makes colcon fail ("existing path cannot be removed").
BUILD_MODE=(--symlink-install)
if [ -f install/px4_msgs/share/px4_msgs/package.xml ] && [ ! -L install/px4_msgs/share/px4_msgs/package.xml ]; then
  echo "existing non-symlink build found; building without --symlink-install"
  echo "(for symlink mode: rm -rf build install log, then re-run this script)"
  BUILD_MODE=()
fi
set +u
colcon build "${BUILD_MODE[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release
set -u

log "6/7 derived data"
tools/make_data.sh

log "7/7 tests"
tools/run_selftest.sh
set +u; source "$ROOT/env.sh"; set -u
colcon test --packages-select gdn_fusion --event-handlers console_direct- >/dev/null
colcon test-result --verbose
python3 tools/make_zermatt_bag.py      # synthetic PX4 bag for the bench smoke test

log "Done. In every new terminal:  cd $ROOT && source env.sh"
echo "Smoke-test the full ROS bench (about 3 min):"
echo "  BENCH_NO_BARO=1 GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin tools/bench.sh"
echo "Recorded PX4 flight for the main bench: tools/fetch_data.sh (see docs/SETUP.md)."
