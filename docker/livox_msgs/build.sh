#!/usr/bin/env bash
# Build livox_ros_driver2's messages (docker/livox_msgs, roadmap R2.4c) into PREFIX:
#   docker/livox_msgs/build.sh /opt/livox_msgs     # the dev image (Dockerfile)
#   docker/livox_msgs/build.sh "$PWD/install_livox" # CI, before colcon builds src/
# then: source PREFIX/setup.bash. On a machine with the real driver (~/ws_livox), source
# that instead: it has the same messages.
set -euo pipefail
PREFIX="${1:?usage: build.sh PREFIX}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
set +u   # ROS's setup scripts read unset variables
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u
colcon --log-base "$WORK/log" build --base-paths "$HERE/livox_ros_driver2" --build-base "$WORK/build" \
  --install-base "$PREFIX" --merge-install --event-handlers console_cohesion- >/dev/null
echo "livox_ros_driver2 messages in $PREFIX (source $PREFIX/setup.bash)"
