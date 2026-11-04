#!/usr/bin/env bash
# Closed-loop SITL, roadmap Stage A (and B): PX4 flies an x500 over real terrain in
# Gazebo Harmonic while GDN runs alongside in shadow mode, ignoring GPS.
#
#   GDN_GPU=nvidia source env.sh          # hybrid laptops: render on the NVIDIA GPU
#   tools/sim_run.sh                      # Gazebo + RViz windows, fly the default route
#   GPS_OFF_AT=120 tools/sim_run.sh       # Stage B: cut GPS 120 s into the route
#   HEADLESS=1 tools/sim_run.sh           # no windows (sensors still render via EGL)
#   GDN_NODE_ARGS="-p mag_fusion:=align" tools/sim_run.sh   # extra eskf_node parameters
#
# One-time setup: tools/setup_sim.sh. Logs: $GDN_LOG_DIR (default /tmp/gdn_sim).
# Ctrl+C stops everything.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.16}"
WORLD="${GDN_SIM_WORLD:-zermatt}"
MAP="${GDN_MAP:-$ROOT/data/maps/zermatt.bin}"
MODEL_INSTANCE="x500_gdn_0"
LOG="${GDN_LOG_DIR:-/tmp/gdn_sim}"
HEADLESS="${HEADLESS:-}"
mkdir -p "$LOG"

die() { echo "sim_run: $*" >&2; exit 1; }
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || die "PX4 SITL not built in $PX4_DIR (run tools/setup_sim.sh)"
command -v gz >/dev/null || die "Gazebo (gz) not found (run tools/setup_sim.sh)"
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not found (run tools/setup_sim.sh)"
command -v ros2 >/dev/null || die "ROS 2 not sourced: source env.sh"
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || die "ros_gz_bridge missing: sudo apt install ros-jazzy-ros-gz-bridge"
ros2 pkg prefix gdn_fusion >/dev/null 2>&1 || die "GDN not built: colcon build --base-paths src ..."

# World (generated per machine because it holds absolute paths).
WDIR="$ROOT/data/sim/$WORLD"
if [ ! -f "$WDIR/$WORLD.sdf" ] || [ "$MAP" -nt "$WDIR/$WORLD.sdf" ]; then
  python3 tools/make_sim_world.py --map "$MAP" --name "$WORLD" || die "world generation failed"
fi

PIDS=()
cleanup() {
  echo; echo "sim_run: stopping"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  sleep 1
  pkill -f "px4_sitl_default/bin/px4" 2>/dev/null
  pkill -f "gz sim" 2>/dev/null
  pkill -f MicroXRCEAgent 2>/dev/null
  echo "sim_run: logs in $LOG"
}
trap cleanup EXIT INT TERM
pkill -f "px4_sitl_default/bin/px4" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
sleep 1

# Gazebo paths and plugins exactly as PX4 sets them, plus GDN's models.
# shellcheck disable=SC1091
# gz_env.sh appends to these without checking they exist, which fails under set -u.
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
set +u
source "$PX4_DIR/build/px4_sitl_default/rootfs/gz_env.sh"
set -u
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$ROOT/sim/models:$WDIR"

echo "sim_run: starting Gazebo ($WORLD)"
if [ -n "$HEADLESS" ]; then
  gz sim -s -r --headless-rendering "$WDIR/$WORLD.sdf" > "$LOG/gz.log" 2>&1 & PIDS+=($!)
else
  gz sim -r "$WDIR/$WORLD.sdf" > "$LOG/gz.log" 2>&1 & PIDS+=($!)
fi

MicroXRCEAgent udp4 -p 8888 > "$LOG/agent.log" 2>&1 & PIDS+=($!)

echo "sim_run: starting PX4 (attaching to $MODEL_INSTANCE)"
( cd "$PX4_DIR" && PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME="$MODEL_INSTANCE" \
    PX4_GZ_WORLD="$WORLD" ./build/px4_sitl_default/bin/px4 -d > "$LOG/px4.log" 2>&1 ) & PIDS+=($!)

echo -n "sim_run: waiting for PX4 over DDS "
for _ in $(seq 1 90); do
  timeout 2 ros2 topic echo --once /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1 && break
  echo -n "."
done
echo
timeout 2 ros2 topic echo --once /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1 \
  || die "no /fmu/out/vehicle_local_position after 3 minutes; see $LOG/px4.log and $LOG/agent.log"
# PX4 settings for this setup:
# - IMU_INTEG_RATE 100: uXRCE-DDS forwards sensor_combined at most at 100 Hz, while PX4
#   integrates the IMU at 250 Hz by default. Then each message GDN gets covers only
#   4 of every 10 ms and 60% of the motion is lost, which wrecks the attitude on a
#   multirotor. At 100 Hz every message covers its whole interval.
# - SYS_FAILURE_EN 1: allow the GPS-failure injection used by Stage B.
# - NAV_DLL_ACT 0: arm without QGroundControl (the x500 airframe requires a GCS link).
( cd "$PX4_DIR/build/px4_sitl_default/rootfs" && ../bin/px4-param set IMU_INTEG_RATE 100 \
    && ../bin/px4-param set SYS_FAILURE_EN 1 && ../bin/px4-param set NAV_DLL_ACT 0 ) >> "$LOG/px4.log" 2>&1

# Gazebo -> ROS for the sensors GDN needs (raw, not PX4's estimates).
SENS="/world/$WORLD/model/$MODEL_INSTANCE/link/base_link/sensor"
cat > "$LOG/bridge.yaml" <<YAML
- ros_topic_name: /gdn/sim/agl_scan
  gz_topic_name: /gdn/agl_radar
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
- ros_topic_name: /gdn/sim/mag
  gz_topic_name: $SENS/magnetometer_sensor/magnetometer
  ros_type_name: sensor_msgs/msg/MagneticField
  gz_type_name: gz.msgs.Magnetometer
  direction: GZ_TO_ROS
- ros_topic_name: /gdn/sim/air_pressure
  gz_topic_name: $SENS/air_pressure_sensor/air_pressure
  ros_type_name: sensor_msgs/msg/FluidPressure
  gz_type_name: gz.msgs.FluidPressure
  direction: GZ_TO_ROS
- ros_topic_name: /gdn/sim/truth_odom
  gz_topic_name: /gdn/truth_odom
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
YAML
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$LOG/bridge.yaml" > "$LOG/bridge.log" 2>&1 & PIDS+=($!)
python3 tools/sim_adapter.py --ros-args -p world_info:="$WDIR/world_info.txt" ${GDN_SIM_ADAPTER_ARGS:-} \
  > "$LOG/adapter.log" 2>&1 & PIDS+=($!)

# GDN in shadow mode. The mag reference is the field Gazebo's magnetometer actually
# simulates (its built-in model at the world's lat/lon, with odd axes: see on_mag in
# sim_adapter.py). For Zermatt that's north 0.9, east 22.5, down -41.8 uT;
# adapter.log prints it in NED for any world, so a new world can be checked.
MAG_REF="${GDN_MAG_REF:-0.9 22.5 -41.8}"
read -r MAG_N MAG_E MAG_D <<< "$MAG_REF"
ros2 run gdn_fusion eskf_node --ros-args -p map_path:="$MAP" \
  -p mag_ref_north:="$MAG_N" -p mag_ref_east:="$MAG_E" -p mag_ref_down:="$MAG_D" ${GDN_NODE_ARGS:-} \
  > "$LOG/gdn.log" 2>&1 & PIDS+=($!)
python3 -u tools/replay_scorer.py > "$LOG/scorer.log" 2>&1 & PIDS+=($!)

# Record everything GDN sees plus truth, so any run can be replayed offline through
# eskf_node with different settings (ros2 bag play $LOG/sitl_bag).
rm -rf "$LOG/sitl_bag"
ros2 bag record -s mcap -o "$LOG/sitl_bag" --qos-profile-overrides-path "$ROOT/tools/bag_qos.yaml" \
  /fmu/out/sensor_combined /fmu/out/vehicle_attitude /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_status_v1 /gdn/odom /gdn/truth /gdn/mag_meas /gdn/baro_meas /gdn/agl_meas \
  /gdn/sim/truth_odom > "$LOG/bag.log" 2>&1 & PIDS+=($!)

if [ -z "$HEADLESS" ]; then
  ros2 run gdn_fusion map_viz_node --ros-args -p map_path:="$MAP" > "$LOG/viz.log" 2>&1 & PIDS+=($!)
  rviz2 -d "$ROOT/sim/gdn_sim.rviz" > "$LOG/rviz.log" 2>&1 & PIDS+=($!)
fi

echo -n "sim_run: waiting for GDN to align and set its altitude datum "
for _ in $(seq 1 120); do
  grep -q "altitude datum" "$LOG/gdn.log" 2>/dev/null && break
  echo -n "."; sleep 1
done
echo
grep -h "altitude datum" "$LOG/gdn.log" | tail -1 || echo "sim_run: warning, GDN hasn't set its datum yet (see $LOG/gdn.log)"

echo "sim_run: flying (Ctrl+C to stop early)"
python3 tools/sim_fly.py --map "$MAP" ${GPS_OFF_AT:+--gps-off-at "$GPS_OFF_AT"} ${GDN_FLY_ARGS:-} 2>&1 | tee "$LOG/fly.log"
sleep 20     # let it land and the scorer catch the end
kill -INT "$(pgrep -f replay_scorer.py)" 2>/dev/null
kill -INT "$(pgrep -f "ros2 bag record")" 2>/dev/null; sleep 4
echo; echo "=== GDN vs truth (horizontal) ==="
grep -hE "RMS|MAX" "$LOG/scorer.log" || tail -3 "$LOG/scorer.log"
echo "=== altimeter check (radar vs map at the true position) ==="
grep -h "radar AGL" "$LOG/adapter.log" | tail -3 || grep -h "no radar" "$LOG/adapter.log" | tail -1
echo "=== recorded: $LOG/sitl_bag ==="
