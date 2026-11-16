#!/bin/bash
# GDN dev image entry point: ROS 2, then the GDN workspace environment (env.sh) when the
# workspace is mounted, then the command (default: an interactive shell).
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ -n "${GDN_ROOT:-}" ] && [ -f "$GDN_ROOT/env.sh" ]; then
  cd "$GDN_ROOT" || exit 1
  source "$GDN_ROOT/env.sh" 2>/dev/null
  [ -f "$GDN_ROOT/install/setup.bash" ] || \
    echo "[gdn-dev] workspace not built in the container yet: colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release" >&2
fi
exec "$@"
