#!/usr/bin/env bash
# Closed-loop SITL, roadmap Stage A (and B): PX4 flies a vehicle over real terrain in
# Gazebo Harmonic while GDN runs alongside in shadow mode, ignoring GPS.
#
#   GDN_GPU=nvidia source env.sh          # hybrid laptops: render on the NVIDIA GPU
#   tools/sim_run.sh                      # Gazebo + RViz windows, fly the default route
#   GPS_OFF_AT=120 tools/sim_run.sh       # Stage B: cut GPS 120 s into the route
#   HEADLESS=1 tools/sim_run.sh           # no windows (sensors still render via EGL)
#   GDN_NODE_ARGS="-p mag_fusion:=align" tools/sim_run.sh   # extra eskf_node parameters
#   GDN_SITE=ch_zurich tools/sim_run.sh   # an R2.2 site (sim/sites.yaml): swisstopo world,
#                                         # GDN on its GLO-30 map (tools/make_site.py)
#   GDN_VEHICLE=standard_vtol tools/sim_run.sh   # R2.3: x500 (default), standard_vtol or
#                                         # rc_cessna (tools/sim_vehicles.py --list)
#   GDN_VEHICLE=rc_cessna GDN_FW_LAUNCH=catapult tools/sim_run.sh   # the fixed-wing takes
#                                         # off from the runway unless launched by catapult
#   GDN_BAG_IMAGES=1 tools/sim_run.sh     # R2.4b: record the OAK-D Lite's images too, in
#                                         # $LOG/image_bag (about 18 MB/s; by default only
#                                         # their camera_info, in sitl_bag)
#   GDN_SIM_CPU=1 tools/sim_run.sh        # no GPU: leave out the OAK-D Lite's cameras and
#                                         # the Mid-360S lidar, which Gazebo can't render
#                                         # fast enough on the processor (the CPU
#                                         # tools/docker_check.sh; IMUs, TFmini and radar stay)
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
VEHICLE="${GDN_VEHICLE:-x500}"
LAUNCH="${GDN_FW_LAUNCH:-runway}"
VEHICLE_SH="$(python3 tools/sim_vehicles.py shell "$VEHICLE" "$LAUNCH")" || exit 1
eval "$VEHICLE_SH"
MODEL_INSTANCE="$VEHICLE_INSTANCE"
LOG="${GDN_LOG_DIR:-/tmp/gdn_sim}"
HEADLESS="${HEADLESS:-}"
mkdir -p "$LOG"

die() { echo "sim_run: $*" >&2; exit 1; }
[ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ] || die "PX4 SITL not built in $PX4_DIR (run tools/setup_sim.sh)"
command -v gz >/dev/null || die "Gazebo (gz) not found (run tools/setup_sim.sh)"
command -v MicroXRCEAgent >/dev/null || die "MicroXRCEAgent not found (run tools/setup_sim.sh)"
command -v ros2 >/dev/null || die "ROS 2 not sourced: source env.sh"
ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 || die "ros_gz_bridge missing: sudo apt install ros-jazzy-ros-gz-bridge"
# R2.4c: the simulated Mid-360S needs Livox's scan pattern and livox_ros_driver2's messages.
"$ROOT/tools/fetch_livox_pattern.sh" >/dev/null || die "couldn't fetch the Mid-360 scan pattern (tools/fetch_livox_pattern.sh)"
ros2 interface show livox_ros_driver2/msg/CustomMsg >/dev/null 2>&1 \
  || die "livox_ros_driver2 messages missing: use the dev image, or source a Livox driver workspace (env.sh looks in ~/ws_livox)"
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
# The world places the x500; this run's copy has the chosen vehicle in its place
# (a fixed-wing facing down the runway, or its catapult heading).
RUN_WORLD="$LOG/$WORLD.sdf"
python3 tools/sim_vehicles.py world "$VEHICLE" "$WDIR" "$RUN_WORLD" "$LAUNCH" || die "no world for $VEHICLE"
printf "vehicle=%s\nworld=%s\nworld_dir=%s\nsite=%s\nlaunch=%s\n" "$VEHICLE" "$WORLD" "$WDIR" "$SITE" "$LAUNCH" \
  > "$LOG/run_info.txt"
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
SIM_PROCS=("px4_sitl_default/bin/px4" "gz sim" "MicroXRCEAgent" "ros_gz_bridge/parameter_bridge" "gdn_fusion/sim_sensors"
           "tools/sim_adapter.py" "tools/replay_scorer.py" "tools/sim_fly.py" "gdn_fusion/map_viz_node"
           "rviz2 -d $ROOT/sim/gdn_sim.rviz" "ros2 bag record -s mcap -o $LOG/sitl_bag"
           "ros2 bag record -s mcap -o $LOG/image_bag")
stop_sim() {
  # GDN first, with SIGINT (its clean shutdown), so its run log (R1.11) is closed and
  # indexed; after a hard kill it needs "ros2 bag reindex". Up to 15 s.
  pkill -INT -f "gdn_fusion/eskf_node" 2>/dev/null
  for _ in $(seq 1 30); do pgrep -f "gdn_fusion/eskf_node" >/dev/null || break; sleep 0.5; done
  pkill -INT -f "ros2 bag record -s mcap -o $LOG/(sitl|image)_bag" 2>/dev/null   # closes the bags
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
# GDN_SIM_CPU=1: a copy of the payload without its cameras and lidar, found first. Rendered
# on the processor they slowed Gazebo to a tenth of real time, and the CPU docker check's
# x500 never finished its climb (R2.4c).
SIM_SENSOR_ARGS=()
if [ -n "${GDN_SIM_CPU:-}" ]; then
  rm -rf "$LOG/models"; mkdir -p "$LOG/models"
  cp -r "$ROOT/sim/models/gdn_payload" "$LOG/models/"
  python3 - "$LOG/models/gdn_payload/model.sdf" <<'PY' || die "couldn't strip the payload's rendered sensors"
import re, sys
p = sys.argv[1]
s, n = re.subn(r'\n\s*<sensor name="(gdn_oak_left|gdn_oak_right|gdn_mid360s_lidar)".*?</sensor>', "",
               open(p).read(), flags=re.S)
if n != 3:
    sys.exit(f"expected the 2 cameras and the lidar in {p}, found {n}")
open(p, "w").write(s)
PY
  export GZ_SIM_RESOURCE_PATH="$LOG/models:$GZ_SIM_RESOURCE_PATH"
  SIM_SENSOR_ARGS=(-p "sensors:=[h30wp, mid360s_imu, oakd_imu, tfmini]")
  echo "sim_run: GDN_SIM_CPU=1, no cameras or lidar (IMUs, TFmini and radar only)"
fi

# Gazebo's own programs (gz sim, the gz command line, PX4 and its gz_bridge) run on the
# system's Gazebo libraries, as they were built. ROS 2 puts its own copies (gz_*_vendor)
# first on LD_LIBRARY_PATH, and env.sh adds the system's gz commands after ROS's. Once the
# system's Gazebo is newer than ROS's copy (gz-transport 13.7.0 against 13.6.0, 18 Nov
# 2026), "gz service" fails with "symbol lookup error ... ResolveServiceTypes", and PX4,
# which asks it for the world, never starts (the R2.4c CI image). ROS programs (the
# bridge, sim_sensors) keep ROS's copies.
GZ_LD="$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/gz_[a-z]*_vendor/lib$' | paste -sd: -)"

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
echo "sim_run: starting Gazebo ($WORLD, $VEHICLE, render engine $RENDER)"
if [ -n "$HEADLESS" ]; then
  LD_LIBRARY_PATH="$GZ_LD" gz sim -v 4 -s -r --headless-rendering "$RUN_WORLD" > "$LOG/gz.log" 2>&1 & GZ_PID=$!
else
  LD_LIBRARY_PATH="$GZ_LD" gz sim -v 4 -s -r "$RUN_WORLD" > "$LOG/gz.log" 2>&1 & GZ_PID=$!
fi
PIDS+=($GZ_PID)
gz_dead() {
  kill -0 "$GZ_PID" 2>/dev/null && return 1
  echo; echo "sim_run: the Gazebo server stopped. Last lines of $LOG/gz.log:"
  grep -iE "error|exception|failed|terminate" "$LOG/gz.log" | tail -8
  if [ "$RENDER" = "ogre2" ]; then
    echo "sim_run: if that's a shader or OGRE error, try:  GDN_GZ_RENDER=ogre tools/sim_run.sh"
  fi
  return 0
}
# Wait for the world to answer before starting PX4, which gives up after about 30 s
# ("Timed out waiting for Gazebo world"). A fixed 5 s was enough on a desktop, but
# GitHub's CPU runner needed longer, and PX4 never came up (R2.4c).
# gz_discovery: what Gazebo's discovery shows, the way PX4's startup asks it, for a
# failed start (in the CI artifact).
gz_discovery() {
  { date; echo "GZ_IP=${GZ_IP:-} GZ_PARTITION=${GZ_PARTITION:-} world=$WORLD"
    echo "--- gz topic -l (PX4 takes the first /world/*/clock)"; LD_LIBRARY_PATH="$GZ_LD" timeout 30 gz topic -l 2>&1
    echo "--- gz service -i /world/$WORLD/scene/info"; LD_LIBRARY_PATH="$GZ_LD" timeout 30 gz service -i --service "/world/$WORLD/scene/info" 2>&1
    echo "--- LD_LIBRARY_PATH for Gazebo: $GZ_LD"
    echo "--- processes"; ps -eo pid,pcpu,etime,args --sort=-pcpu | head -25; } > "$LOG/gz_discovery.txt" 2>&1
  echo "sim_run: Gazebo's discovery written to $LOG/gz_discovery.txt"
}
T0=$SECONDS
echo -n "sim_run: waiting for Gazebo's world "
for i in $(seq 1 240); do
  LD_LIBRARY_PATH="$GZ_LD" gz service -i --service "/world/$WORLD/scene/info" 2>&1 | grep -q "Service providers" && break
  gz_dead && exit 1
  [ "$i" -eq 240 ] && { echo; gz_discovery; die "Gazebo's world didn't come up in 4 minutes; see $LOG/gz.log"; }
  echo -n "."; sleep 1
done
echo " up after $((SECONDS - T0)) s"
# PX4 attaches to the first world Gazebo lists, so another one running would take its place.
FIRST_WORLD="$(LD_LIBRARY_PATH="$GZ_LD" gz topic -l 2>/dev/null | grep -m 1 -e "^/world/.*/clock" | sed 's#/world/##; s#/clock##')"
[ "$FIRST_WORLD" = "$WORLD" ] || echo "sim_run: warning, Gazebo lists world '${FIRST_WORLD}' first, not '$WORLD': PX4 would attach to it"
if [ -z "$HEADLESS" ]; then
  # Gazebo's own start camera is inside our terrain; tools/gz_gui_config.py fixes it.
  python3 tools/gz_gui_config.py "$WDIR" "$LOG/gui.config"
  LD_LIBRARY_PATH="$GZ_LD" gz sim -v 3 -g --render-engine-gui "$RENDER" --gui-config "$LOG/gui.config" > "$LOG/gz_gui.log" 2>&1 & PIDS+=($!)
fi

MicroXRCEAgent udp4 -p 8888 > "$LOG/agent.log" 2>&1 & PIDS+=($!)

echo "sim_run: starting PX4 (airframe $VEHICLE_AUTOSTART, attaching to $MODEL_INSTANCE)"
( cd "$PX4_DIR" && LD_LIBRARY_PATH="$GZ_LD" PX4_SYS_AUTOSTART="$VEHICLE_AUTOSTART" PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME="$MODEL_INSTANCE" \
    PX4_GZ_WORLD="$WORLD" ./build/px4_sitl_default/bin/px4 -d > "$LOG/px4.log" 2>&1 ) & PIDS+=($!)

# Each try listens 5 s: on GitHub's CPU runner a fresh "ros2 topic echo" needed more than
# 2 s to start and find PX4. A second, confirming 2 s listen after the loop missed it
# there, and stopped a run whose PX4 was up (R2.4c); the loop's own success counts now.
T0=$SECONDS
PX4_UP=
echo -n "sim_run: waiting for PX4 over DDS "
for _ in $(seq 1 40); do
  if timeout 5 ros2 topic echo --once /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition >/dev/null 2>&1; then
    PX4_UP=1; break
  fi
  gz_dead && exit 1
  echo -n "."
done
echo
[ -n "$PX4_UP" ] && echo "sim_run: PX4 up over DDS after $((SECONDS - T0)) s"
[ -n "$PX4_UP" ] \
  || { gz_discovery; die "no /fmu/out/vehicle_local_position after $((SECONDS - T0)) s; see $LOG/px4.log and $LOG/agent.log"; }
# PX4 settings for this setup and vehicle (each one explained in tools/sim_vehicles.py):
# IMU_INTEG_RATE 100, NAV_DLL_ACT 0 and EKF2_GPS_CTRL 7 always, plus the fixed-wing's
# catapult launch. PX4 resets its parameters when the airframe changes.
for kv in $VEHICLE_PARAMS; do
  ( cd "$PX4_DIR/build/px4_sitl_default/rootfs" && ../bin/px4-param set "${kv%%=*}" "${kv#*=}" ) >> "$LOG/px4.log" 2>&1 \
    || die "PX4 didn't take ${kv%%=*} ${kv#*=} (see $LOG/px4.log)"
done

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
# The payload's error-free sensors for tools/sim_sensors (R2.4a, R2.4b), which publishes
# them as the real drivers do: /imu/data, /livox/imu, /oak/imu/data, /tfmini/range and
# the OAK-D Lite's /oak/left and /oak/right images, and the Mid-360S's /livox/lidar (R2.4c).
for s in h30wp mid360s oakd; do
  cat >> "$LOG/bridge.yaml" <<YAML
- ros_topic_name: /gdn/sim/raw/imu_$s
  gz_topic_name: /gdn/imu/$s
  ros_type_name: sensor_msgs/msg/Imu
  gz_type_name: gz.msgs.IMU
  direction: GZ_TO_ROS
YAML
done
cat >> "$LOG/bridge.yaml" <<YAML
- ros_topic_name: /gdn/sim/raw/tfmini
  gz_topic_name: /gdn/tfmini
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
YAML
cat >> "$LOG/bridge.yaml" <<YAML
- ros_topic_name: /gdn/sim/raw/mid360s_points
  gz_topic_name: /gdn/mid360s/points
  ros_type_name: sensor_msgs/msg/PointCloud2
  gz_type_name: gz.msgs.PointCloudPacked
  direction: GZ_TO_ROS
YAML
for s in left right; do
  cat >> "$LOG/bridge.yaml" <<YAML
- ros_topic_name: /gdn/sim/raw/oak_$s
  gz_topic_name: /gdn/oak/$s
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
YAML
done
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$LOG/bridge.yaml" > "$LOG/bridge.log" 2>&1 & PIDS+=($!)
ros2 run gdn_fusion sim_sensors --ros-args --params-file "$ROOT/config/sim/sensors.yaml" "${SIM_SENSOR_ARGS[@]}" \
  > "$LOG/sensors.log" 2>&1 & PIDS+=($!)
python3 tools/sim_adapter.py --ros-args -p world_info:="$WDIR/world_info.txt" -p gz_model:="$MODEL_INSTANCE" ${GDN_SIM_ADAPTER_ARGS:-} \
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
# eskf_node with different settings (ros2 bag play $LOG/sitl_bag). The raw radar scan
# too: it and the truth carry Gazebo's time, so tools/check_vehicles.py can compare
# them at the same instant. The OAK-D Lite's camera_info always (each marks an image's
# stamp and arrival), and Gazebo's raw H30WP (250 Hz, Gazebo stamps: the reference
# check_sim_sensors.py turns receive times into Gazebo time with). Its images, with
# GDN_BAG_IMAGES=1, go to a bag of their own
# ($LOG/image_bag): in the same recorder, 18 MB/s held every topic up by about 0.3 s.
rm -rf "$LOG/sitl_bag" "$LOG/image_bag"
if [ -n "${GDN_BAG_IMAGES:-}" ]; then
  ros2 bag record -s mcap -o "$LOG/image_bag" /oak/left/image_raw /oak/right/image_raw \
    > "$LOG/image_bag.log" 2>&1 & PIDS+=($!)
fi
ros2 bag record -s mcap -o "$LOG/sitl_bag" --qos-profile-overrides-path "$ROOT/tools/bag_qos.yaml" \
  /fmu/out/sensor_combined /fmu/out/vehicle_attitude /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_status_v1 /gdn/odom /gdn/truth /gdn/mag_meas /gdn/baro_meas /gdn/agl_meas \
  /gdn/sim/truth_odom /gdn/sim/agl_scan /imu/data /livox/imu /oak/imu/data /tfmini/range \
  /oak/left/camera_info /oak/right/camera_info /gdn/sim/raw/imu_h30wp /livox/lidar \
  > "$LOG/bag.log" 2>&1 & PIDS+=($!)

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
python3 tools/sim_fly.py --map "$MAP" --px4-dir "$PX4_DIR" --vehicle "$VEHICLE" --gz-world "$WORLD" \
  --world-info "$WDIR/world_info.txt" --launch "$LAUNCH" --route-out "$LOG/route.csv" ${GPS_OFF_AT:+--gps-off-at "$GPS_OFF_AT"} ${GDN_FLY_ARGS:-} 2>&1 | tee "$LOG/fly.log"
sleep 20     # let it land (or circle) and the scorer catch the end
kill -INT "$(pgrep -f replay_scorer.py)" 2>/dev/null
pkill -INT -f "ros2 bag record" 2>/dev/null; sleep 4
echo; echo "=== GDN vs truth (horizontal) ==="
grep -hE "RMS|MAX" "$LOG/scorer.log" || tail -3 "$LOG/scorer.log"
# The whole flight, not the last report: that one is the landing over the level pad,
# where it's always about 0.3 m. In cruise over slopes it's metres, because the
# adapter converts the slant range to a height as if the ground were flat (OI-016).
echo "=== GDN's altimeter input against the height straight below (adapter, whole flight) ==="
grep -o "map-predicted AGL: mean [-+0-9.]* m, rms [0-9.]* m, n=[0-9]*" "$LOG/adapter.log" | awk '
  { n = substr($9, 3); s2 += n * $7 * $7; sm += n * $4; N += n; if ($7 > mx) mx = $7 }
  END { if (N) printf "RMS %.2f m, mean %+.2f m over %d readings; worst 10 s window %.2f m RMS (tilt on slopes: OI-016)\n", sqrt(s2 / N), sm / N, N, mx }'
grep -h "no radar" "$LOG/adapter.log" | tail -1
echo "=== recorded: $LOG/sitl_bag ==="
