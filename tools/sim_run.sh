#!/usr/bin/env bash
# Closed-loop SITL, roadmap Stage A (and B): PX4 flies an x500 over real terrain in
# Gazebo Harmonic while GDN runs alongside in shadow mode, ignoring GPS.
#
#   GDN_GPU=nvidia source env.sh          # hybrid laptops: render on the NVIDIA GPU
#   tools/sim_run.sh                      # Gazebo + RViz windows, fly the default route
#   GPS_OFF_AT=120 tools/sim_run.sh       # Stage B: cut GPS 120 s into the route
#   HEADLESS=1 tools/sim_run.sh           # no windows (sensors still render via EGL)
#   GDN_NODE_ARGS="-p mag_fusion:=align" tools/sim_run.sh   # extra eskf_node parameters
#   GDN_SITE=ch_zurich tools/sim_run.sh   # an R2.2 site (sim/sites.yaml): swisstopo world,
#                                         # GDN on its GLO-30 map (tools/make_site.py)
#
# One-time setup: tools/setup_sim.sh. Logs: $GDN_LOG_DIR (default /tmp/gdn_sim).
# Ctrl+C stops everything.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.16}"
SITE="${GDN_SITE:-}"
WORLD="${GDN_SIM_WORLD:-zermatt}"
MAP="${GDN_MAP:-$ROOT/data/maps/zermatt.bin}"
if [ -n "$SITE" ]; then
  WORLD="$SITE"
  MAP="$ROOT/data/maps/${SITE}_truth.bin"
fi
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
if [ -n "$SITE" ]; then
  # R2.2 site: the world from swisstopo, GDN's map from GLO-30 (downloads are cached).
  # Rebuilt only if its sites.yaml entry or the building code changed.
  python3 tools/make_site.py "$SITE" --if-stale || die "site generation failed (tools/make_site.py $SITE)"
elif [ ! -f "$WDIR/$WORLD.sdf" ] || [ "$MAP" -nt "$WDIR/$WORLD.sdf" ] || [ tools/make_sim_world.py -nt "$WDIR/$WORLD.sdf" ]; then
  python3 tools/make_sim_world.py --map "$MAP" --name "$WORLD" || die "world generation failed"
fi
winfo() { sed -n "s/^$1=//p" "$WDIR/world_info.txt"; }
# GDN's own map: on a site, GLO-30, in the world's frame (origin at the pad); otherwise
# the world's map itself.
NAV_MAP="$(winfo onboard_map)"; NAV_MAP="${NAV_MAP:-$MAP}"
NAV_ORIGIN=()
if [ "$NAV_MAP" != "$(winfo map)" ]; then
  NAV_ORIGIN=(-p origin_lat:="$(winfo origin_lat)" -p origin_lon:="$(winfo origin_lon)")
fi

PIDS=()
# Everything a run starts, by name. Killing the PIDs above isn't enough: for
# "ros2 run ..." the PID is the launcher, and the program it started survives it.
# A bridge or map_viz_node left from an earlier run doubles every Gazebo message
# and keeps redrawing the old path in RViz.
SIM_PROCS=("px4_sitl_default/bin/px4" "gz sim" "MicroXRCEAgent" "ros_gz_bridge/parameter_bridge"
           "tools/sim_adapter.py" "tools/replay_scorer.py" "tools/sim_fly.py" "gdn_fusion/map_viz_node"
           "rviz2 -d $ROOT/sim/gdn_sim.rviz" "ros2 bag record -s mcap -o $LOG/sitl_bag")
stop_sim() {
  # GDN first, with SIGINT (its clean shutdown), so its run log (R1.11) is closed and
  # indexed; after a hard kill it needs "ros2 bag reindex". Up to 15 s.
  pkill -INT -f "gdn_fusion/eskf_node" 2>/dev/null
  for _ in $(seq 1 30); do pgrep -f "gdn_fusion/eskf_node" >/dev/null || break; sleep 0.5; done
  pkill -INT -f "ros2 bag record -s mcap -o $LOG/sitl_bag" 2>/dev/null   # closes the bag
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null; done
  for n in "${SIM_PROCS[@]}"; do pkill -f "$n" 2>/dev/null; done
  sleep 1
  pkill -9 -f "gdn_fusion/eskf_node" 2>/dev/null
  for n in "${SIM_PROCS[@]}"; do pkill -9 -f "$n" 2>/dev/null; done
}
cleanup() {
  echo; echo "sim_run: stopping"
  stop_sim
  echo "sim_run: logs in $LOG"
}
trap cleanup EXIT INT TERM
stop_sim   # anything left from an earlier run
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

# Gazebo's messaging uses multicast discovery; with no network route (offline laptop,
# some VPNs) it fails with "Network is unreachable". Everything here is on one machine.
export GZ_IP="${GZ_IP:-127.0.0.1}"

# Render engine: ogre2 (default, needed for the OAK-D camera later) or ogre (Ogre1, a
# fallback if ogre2's shaders fail to compile on a driver). The terrain is a mesh, which
# both render the same; tools/check_sim_radar.py checks the radar against the map. It applies to both the
# server's sensors (the radar altimeter) and the GUI, via a copy of PX4's server config.
RENDER="${GDN_GZ_RENDER:-ogre2}"
sed "s#<render_engine>[a-z0-9]*</render_engine>#<render_engine>$RENDER</render_engine>#" \
  "$GZ_SIM_SERVER_CONFIG_PATH" > "$LOG/server.config"
export GZ_SIM_SERVER_CONFIG_PATH="$LOG/server.config"

# Server and GUI run as separate processes, so a GUI crash doesn't stop the simulation.
echo "sim_run: starting Gazebo ($WORLD, render engine $RENDER)"
if [ -n "$HEADLESS" ]; then
  gz sim -v 4 -s -r --headless-rendering "$WDIR/$WORLD.sdf" > "$LOG/gz.log" 2>&1 & GZ_PID=$!
else
  gz sim -v 4 -s -r "$WDIR/$WORLD.sdf" > "$LOG/gz.log" 2>&1 & GZ_PID=$!
fi
PIDS+=($GZ_PID)
sleep 5
gz_dead() {
  kill -0 "$GZ_PID" 2>/dev/null && return 1
  echo; echo "sim_run: the Gazebo server stopped. Last lines of $LOG/gz.log:"
  grep -iE "error|exception|failed|terminate" "$LOG/gz.log" | tail -8
  if [ "$RENDER" = "ogre2" ]; then
    echo "sim_run: if that's a shader or OGRE error, try:  GDN_GZ_RENDER=ogre tools/sim_run.sh"
  fi
  return 0
}
gz_dead && exit 1
if [ -z "$HEADLESS" ]; then
  # Gazebo's own start camera is inside our terrain; tools/gz_gui_config.py fixes it.
  python3 tools/gz_gui_config.py "$WDIR" "$LOG/gui.config"
  gz sim -v 3 -g --render-engine-gui "$RENDER" --gui-config "$LOG/gui.config" > "$LOG/gz_gui.log" 2>&1 & PIDS+=($!)
fi

MicroXRCEAgent udp4 -p 8888 > "$LOG/agent.log" 2>&1 & PIDS+=($!)

echo "sim_run: starting PX4 (attaching to $MODEL_INSTANCE)"
( cd "$PX4_DIR" && PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME="$MODEL_INSTANCE" \
    PX4_GZ_WORLD="$WORLD" ./build/px4_sitl_default/bin/px4 -d > "$LOG/px4.log" 2>&1 ) & PIDS+=($!)

echo -n "sim_run: waiting for PX4 over DDS "
for _ in $(seq 1 90); do
  timeout 2 ros2 topic echo --once /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1 && break
  gz_dead && exit 1
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
# - NAV_DLL_ACT 0: arm without QGroundControl (the x500 airframe requires a GCS link).
# - EKF2_GPS_CTRL 7 (PX4's default): GPS on. A Stage B cut sets it to 0 mid-flight
#   (sim_fly.py --gps-off-at); PX4 keeps parameters between runs, so set it back here.
( cd "$PX4_DIR/build/px4_sitl_default/rootfs" && ../bin/px4-param set IMU_INTEG_RATE 100 \
    && ../bin/px4-param set NAV_DLL_ACT 0 && ../bin/px4-param set EKF2_GPS_CTRL 7 ) >> "$LOG/px4.log" 2>&1

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
# sim_adapter.py). For the default Zermatt world that's north 0.9, east 22.5, down
# -41.8 uT. A site is elsewhere, so there the reference is read from the adapter,
# which logs the field in NED (from the truth attitude) every 10 s.
MAG_REF="${GDN_MAG_REF:-0.9 22.5 -41.8}"
if [ -n "$SITE" ] && [ -z "${GDN_MAG_REF:-}" ]; then
  echo -n "sim_run: reading the simulated magnetic field "
  for _ in $(seq 1 40); do
    F="$(grep -o 'field in NED [-0-9. ]*uT' "$LOG/adapter.log" 2>/dev/null | head -1 | sed 's/field in NED //; s/ uT//')"
    [ -n "$F" ] && break
    echo -n "."; sleep 1
  done
  echo
  if [ -n "$F" ]; then MAG_REF="$F"; echo "sim_run: magnetic reference (NED, uT): $MAG_REF"
  else echo "sim_run: warning, no field from the adapter; using the Zermatt reference"; fi
fi
read -r MAG_N MAG_E MAG_D <<< "$MAG_REF"
ros2 run gdn_fusion eskf_node --ros-args -p map_path:="$NAV_MAP" "${NAV_ORIGIN[@]}" \
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

# Wait for GDN's static alignment (and heading). Its altitude datum comes with the
# first radar reading, and parked the radar sits under its 0.2 m minimum range, so
# that's at lift-off (gdn.log: "altitude datum ... AGL 0.2"); waiting for it here only
# ever timed out.
echo -n "sim_run: waiting for GDN to align "
for _ in $(seq 1 120); do
  grep -q "heading from magnetometer" "$LOG/gdn.log" 2>/dev/null && break
  echo -n "."; sleep 1
done
echo
grep -h "static alignment" "$LOG/gdn.log" | tail -1 || echo "sim_run: warning, GDN hasn't aligned yet (see $LOG/gdn.log)"

echo "sim_run: flying (Ctrl+C to stop early)"
python3 tools/sim_fly.py --map "$MAP" --px4-dir "$PX4_DIR" ${GPS_OFF_AT:+--gps-off-at "$GPS_OFF_AT"} ${GDN_FLY_ARGS:-} 2>&1 | tee "$LOG/fly.log"
sleep 20     # let it land and the scorer catch the end
kill -INT "$(pgrep -f replay_scorer.py)" 2>/dev/null
kill -INT "$(pgrep -f "ros2 bag record")" 2>/dev/null; sleep 4
echo; echo "=== GDN vs truth (horizontal) ==="
grep -hE "RMS|MAX" "$LOG/scorer.log" || tail -3 "$LOG/scorer.log"
echo "=== altimeter check (radar vs map at the true position) ==="
grep -h "radar AGL" "$LOG/adapter.log" | tail -3 || grep -h "no radar" "$LOG/adapter.log" | tail -1
echo "=== recorded: $LOG/sitl_bag ==="
