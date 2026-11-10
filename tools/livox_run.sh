#!/usr/bin/env bash
# Start livox_ros_driver2 for the Mid-360S with GDN's config (config/sensors/mid360s.json,
# written by tools/livox_setup.py). The driver lives in its own workspace, ~/livox_ws by
# default, because its build script deletes ../../build and ../../install.
#
#   tools/livox_run.sh                 # CustomMsg on /livox/lidar (what FAST-LIO and Point-LIO read)
#   XFER=0 tools/livox_run.sh          # sensor_msgs/PointCloud2 instead (what RViz shows directly)
#
# Topics: /livox/lidar (10 Hz), /livox/imu (about 200 Hz, acceleration in g, not m/s^2).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIVOX_WS="${LIVOX_WS:-$HOME/livox_ws}"
CFG="$ROOT/config/sensors/mid360s.json"
[ -f "$CFG" ] || { echo "No $CFG yet: run python3 tools/livox_setup.py --sn <serial> first"; exit 1; }
[ -f "$LIVOX_WS/install/setup.bash" ] || { echo "No driver in $LIVOX_WS: see docs/hardware/MID360S_SETUP.md"; exit 1; }
# shellcheck disable=SC1091
source "$LIVOX_WS/install/setup.bash"
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:="${XFER:-1}" -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 \
  -p output_data_type:=0 -p frame_id:=livox_frame -p user_config_path:="$CFG"
