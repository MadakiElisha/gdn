#!/usr/bin/env python3
"""Turn Gazebo's raw sensors into GDN's inputs, and publish truth for scoring.

In SITL (tools/sim_run.sh), ros_gz_bridge brings these Gazebo topics into ROS:
  /gdn/sim/agl_scan      sensor_msgs/LaserScan      downward radar altimeter (slant range)
  /gdn/sim/mag           sensor_msgs/MagneticField  vehicle magnetometer (see on_mag for its units)
  /gdn/sim/air_pressure  sensor_msgs/FluidPressure  vehicle barometer, Pa
  /gdn/sim/truth_odom    nav_msgs/Odometry          true pose, world ENU, origin = map centre
plus PX4's /fmu/out/vehicle_attitude for removing tilt from the altimeter range.

It publishes exactly what eskf_node consumes, the same as the replay simulators:
  /gdn/agl_meas   Float64      height above ground, tilt removed (2 Hz)
  /gdn/baro_meas  Float64      NED z from pressure, relative to the start, + optional bias
  /gdn/mag_meas   Vector3      body FRD, microtesla
  /gdn/truth      PointStamped true north/east/down (for replay_scorer)
  /gdn/truth_path nav_msgs/Path in the RViz "map" frame (ENU, z = 0 at the map centre)

Altimeter check: every 10 s it logs how far the measured AGL is from the AGL the map
predicts at the true position (truth altitude minus map height). Level (hover, the
pad) that measures how well the rendered Gazebo terrain matches GDN's map, about 0.3 m.
Tilted over a slope it's metres: the range times the cosine of the tilt is the height
above the ground where the beam lands, not straight below (OI-016), and that is what
GDN gets. tools/sim_run.sh prints the whole flight's figure at the end.
If the radar isn't publishing (no GPU rendering), agl_source:=auto falls back to that
map prediction plus 0.3 m noise, and says so.

Sensor faults (R2.6b, tools/sim_faults.py): the "faults" parameter (GDN_FAULTS in
sim_run.sh) applies dropouts, stuck values and noise bursts to what this publishes for
GDN, timed from the route's start (/gdn/sim/route_start, from sim_fly.py). For an IMU
fault, imu_relay:=true relays PX4's /fmu/out/sensor_combined to /gdn/sim/imu_relay with
the fault applied, and sim_run.sh points eskf_node at it. Each fault's start and end
goes out on /gdn/sim/fault (std_msgs/String, "on agl dropout at 30 s for 20 s" and
"off ..."), recorded in the bag for tools/check_scenario.py.

Lighting (R2.6b): every second, the light the OAK-D Lite's left camera sees, on
/gdn/sim/scene_light (Float64: Gazebo's raw linear mono16 image, /gdn/sim/raw/oak_left,
its mean as a fraction of full scale), and the mean brightness (0-255) of the image it
sends after its auto-exposure (sim_sensors), on /gdn/sim/image_brightness.

Parameters: world_info (data/sim/<world>/world_info.txt), agl_source (auto|lidar|truth),
agl_rate_hz (2), baro_bias_m (0), baro_drift_m_per_min (0), max_tilt_deg (35),
gz_model (the vehicle's Gazebo name, for the window's follow camera; default the
world's x500), gz_follow (true), gz_follow_offset (-1.8,0,0.7), faults (""),
imu_relay (false), fault_seed (0).
"""
import math
import os
import subprocess
import threading
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.qos import DurabilityPolicy
from sensor_msgs.msg import FluidPressure, Image, LaserScan, MagneticField
from std_msgs.msg import Float64, String
from px4_msgs.msg import SensorCombined, VehicleAttitude
from tf2_ros import TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parent))
from gdn_paths import ROOT, load_map  # noqa: E402
import sim_faults  # noqa: E402


def pressure_altitude(p_pa):
    """ISA pressure altitude, m (the standard barometric formula)."""
    return 44330.77 * (1.0 - (p_pa / 101325.0) ** 0.190263)


class SimAdapter(Node):
    def __init__(self):
        super().__init__("gdn_sim_adapter")
        info_path = self.declare_parameter("world_info", str(ROOT / "data/sim/zermatt/world_info.txt")).value
        self.agl_source = self.declare_parameter("agl_source", "auto").value
        self.agl_period = 1.0 / float(self.declare_parameter("agl_rate_hz", 2.0).value)
        self.baro_bias = float(self.declare_parameter("baro_bias_m", 0.0).value)
        self.baro_drift = float(self.declare_parameter("baro_drift_m_per_min", 0.0).value) / 60.0
        self.max_tilt = math.radians(float(self.declare_parameter("max_tilt_deg", 35.0).value))
        # Views that follow the drone: RViz orbits the "drone" frame published here, and
        # Gazebo's window (when there is one) gets its follow camera at this offset
        # (metres behind, left, above the drone).
        follow = self.declare_parameter("gz_follow", True).value and not os.environ.get("HEADLESS")
        # Metres behind, left, above, as "x,y,z" (commas: GDN_SIM_ADAPTER_ARGS is split at
        # spaces, so "-p gz_follow_offset:=-4,0,1.5" is the way to pass it through sim_run.sh).
        off = str(self.declare_parameter("gz_follow_offset", "-1.8,0,0.7").value).replace(",", " ").split()
        self.follow_offset = [float(v) for v in off]
        if len(self.follow_offset) != 3:
            raise SystemExit(f"sim_adapter: gz_follow_offset needs 3 numbers (x,y,z), got {off}")

        try:
            self.faults = sim_faults.parse(str(self.declare_parameter("faults", "").value))
        except ValueError as e:
            raise SystemExit(f"sim_adapter: {e}") from e
        rng = np.random.default_rng(int(self.declare_parameter("fault_seed", 0).value))
        self.inj = {k: sim_faults.Injector(self.faults, k, rng) for k in sim_faults.SENSORS}
        self.t_route0 = None          # unix time of the route's start (sim_fly.py)
        imu_relay = bool(self.declare_parameter("imu_relay", False).value)
        if any(f.sensor == "imu" for f in self.faults) and not imu_relay:
            raise SystemExit("sim_adapter: an imu fault needs imu_relay:=true (and eskf_node on /gdn/sim/imu_relay)")

        info = dict(l.split("=", 1) for l in Path(info_path).read_text().split() if "=" in l)
        self.gz_model = str(self.declare_parameter("gz_model", info.get("model_instance", "x500_gdn_0")).value)
        self.z0_amsl = float(info["z0_amsl"])
        self.alt, self.hdr = load_map(info.get("world_map", info["map"]))
        self.alt = self.alt.astype(np.float64)
        from make_sim_world import sample_map
        self.sample_map = sample_map
        self.base_alt = float(sample_map(self.alt, self.hdr, np.array(0.0), np.array(0.0)))

        best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, "/gdn/sim/agl_scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(MagneticField, "/gdn/sim/mag", self.on_mag, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, "/gdn/sim/air_pressure", self.on_baro, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/gdn/sim/truth_odom", self.on_truth, 10)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.on_att, best)
        self.pub_agl = self.create_publisher(Float64, "/gdn/agl_meas", 10)
        self.pub_baro = self.create_publisher(Float64, "/gdn/baro_meas", 10)
        self.pub_mag = self.create_publisher(Vector3, "/gdn/mag_meas", 10)
        self.pub_truth = self.create_publisher(PointStamped, "/gdn/truth", 10)
        self.pub_tpath = self.create_publisher(PathMsg, "/gdn/truth_path", 10)
        self.pub_fault = self.create_publisher(String, "/gdn/sim/fault", 10)
        self.pub_bright = self.create_publisher(Float64, "/gdn/sim/image_brightness", 10)
        self.create_subscription(Float64, "/gdn/sim/route_start", self.on_route_start, QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(Image, "/oak/left/image_raw", self.on_image, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(Image, "/gdn/sim/raw/oak_left", self.on_raw_image, QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1))
        self.pub_light = self.create_publisher(Float64, "/gdn/sim/scene_light", 10)
        self.last_bright = self.last_light = 0.0
        if imu_relay:
            self.pub_imu = self.create_publisher(SensorCombined, "/gdn/sim/imu_relay", qos_profile_sensor_data)
            self.create_subscription(SensorCombined, "/fmu/out/sensor_combined", self.on_imu, qos_profile_sensor_data)
        self.faults_on = set()
        for f in self.faults:
            self.get_logger().info(f"fault asked: {f.text()} into the route")
        if self.faults:
            self.create_timer(0.05, self.fault_events)

        self.cos_tilt = None          # from PX4 attitude
        self.truth = None             # (north, east, z_world_up)
        self.q_truth = None           # true attitude, body FLU to world ENU
        self.mag_ned = []
        self.h_ref = None             # pressure altitude at start
        self.t_start = None
        self.last_agl_t = 0.0
        self.last_scan_t = None
        self.last_truth_pub = 0.0
        self.resid = []
        self.n_scan = self.n_mag = self.n_baro = 0
        self.fallback_warned = False
        self.t_node = self.now_s()      # the radar gets 5 s to start before the fallback
        self.tpath = PathMsg()
        self.tpath.header.frame_id = "map"
        self.tfb = TransformBroadcaster(self)
        self.last_tf = 0.0
        if follow:
            threading.Thread(target=self.gz_follow, daemon=True).start()
        self.create_timer(0.5, self.tick)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"sim adapter: map {info.get('world_map', info['map'])}, world z=0 at {self.z0_amsl:.1f} m AMSL, "
                               f"agl_source={self.agl_source}")

    # --- helpers -------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def map_h(self, north, east):
        return float(self.sample_map(self.alt, self.hdr, np.array(east), np.array(north)))

    def predicted_agl(self):
        """Vertical AGL the map predicts at the true position."""
        n, e, z_up = self.truth
        return (self.z0_amsl + z_up) - self.map_h(n, e)

    def r_ned_frd(self):
        """Body FRD to world NED rotation from the true ENU/FLU attitude."""
        w, x, y, z = self.q_truth
        r_enu_flu = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        enu_to_ned = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        frd_to_flu = np.diag([1, -1, -1])
        return enu_to_ned @ r_enu_flu @ frd_to_flu

    def publish_agl(self, value):
        t = self.now_s()
        if t - self.last_agl_t < self.agl_period:
            return
        self.last_agl_t = t
        value = self.fault("agl", value)
        if value is not None:
            self.pub_agl.publish(Float64(data=float(value)))

    def t_route(self):
        return None if self.t_route0 is None else time.time() - self.t_route0

    def fault(self, sensor, value):
        """The value with any fault in force applied (None: publish nothing)."""
        inj = self.inj[sensor]
        if not inj.faults:
            return value
        return inj.apply(self.t_route(), value)[0]

    def fault_events(self):
        """Each fault's start and end on /gdn/sim/fault, by the schedule (20 Hz), not by the
        sensor's samples: a sensor can be silent when its fault starts (the VTOL's altimeter
        just after its transition, TEST-041)."""
        t = self.t_route()
        for f in self.faults:
            now = f.active(t)
            if now != (f in self.faults_on):
                (self.faults_on.add if now else self.faults_on.discard)(f)
                text = f"{'on' if now else 'off'} {f.text()}"
                self.pub_fault.publish(String(data=text))
                self.get_logger().warn(f"fault {text} ({t:.2f} s into the route)")

    def on_route_start(self, msg):
        if self.t_route0 is None:
            self.t_route0 = float(msg.data)
            self.get_logger().info(f"route started ({time.time() - self.t_route0:.2f} s ago): fault times count from here")

    def on_image(self, msg):
        t = self.now_s()
        if t - self.last_bright < 1.0 or not msg.data:
            return
        self.last_bright = t
        px = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        self.pub_bright.publish(Float64(data=float(px[::7].mean())))

    def on_raw_image(self, msg):
        t = self.now_s()
        if t - self.last_light < 1.0 or not msg.data or msg.encoding != "mono16":
            return
        self.last_light = t
        px = np.frombuffer(bytes(msg.data), dtype="<u2" if not msg.is_bigendian else ">u2")
        self.pub_light.publish(Float64(data=float(px[::7].mean() / 65535.0)))

    def on_imu(self, msg):
        acc = tuple(float(v) for v in msg.accelerometer_m_s2)
        gyr = tuple(float(v) for v in msg.gyro_rad)
        out = self.fault("imu", (acc, gyr))
        if out is None:
            return
        msg.accelerometer_m_s2 = [float(v) for v in out[0]]
        msg.gyro_rad = [float(v) for v in out[1]]
        self.pub_imu.publish(msg)

    # --- callbacks -----------------------------------------------------------
    def on_att(self, msg):
        w, x, y, z = (float(v) for v in msg.q)
        self.cos_tilt = 1.0 - 2.0 * (x * x + y * y)       # R(2,2) of body->NED

    def on_scan(self, msg):
        if self.agl_source == "truth" or not msg.ranges:
            return
        r = float(msg.ranges[0])
        self.last_scan_t = self.now_s()
        self.n_scan += 1
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max or self.cos_tilt is None:
            return
        if self.cos_tilt < math.cos(self.max_tilt):
            return
        agl = r * self.cos_tilt
        if self.truth is not None:
            self.resid.append(agl - self.predicted_agl())
        self.publish_agl(agl)

    def on_mag(self, msg):
        # Gazebo Harmonic's magnetometer publishes gauss (not tesla, despite the field
        # name), body FLU, as a proper rotation of a fixed world vector. That vector
        # comes from Gazebo's own field model at the world's lat/lon (it ignores
        # <magnetic_field>) and holds the north/east/down values in the east/north/up
        # slots, so the simulated field points east and up. That's physically odd
        # but self-consistent, which is all GDN needs: we rotate FLU to FRD, and
        # sim_run.sh gives eskf_node the matching reference (this node logs it in NED
        # every 10 s). Checked against truth attitude over a +-90 degree yaw sweep.
        # PX4's gz_bridge maps it as (-y, -x, z) instead (gz-sim PR 2460), which is a
        # reflection and only right at one heading; PX4 still flies on GPS.
        f = msg.magnetic_field
        m = np.array([f.x, -f.y, -f.z]) * 100.0             # gauss -> uT, FLU -> FRD
        out = self.fault("mag", (float(m[0]), float(m[1]), float(m[2])))
        if out is not None:
            self.pub_mag.publish(Vector3(x=out[0], y=out[1], z=out[2]))
        self.n_mag += 1
        if self.q_truth is not None:                        # field in NED, for the mag_ref check
            self.mag_ned.append(self.r_ned_frd() @ m)

    def on_baro(self, msg):
        h = pressure_altitude(msg.fluid_pressure)
        t = self.now_s()
        if self.h_ref is None:
            self.h_ref, self.t_start = h, t
        bias = self.baro_bias + self.baro_drift * (t - self.t_start)
        out = self.fault("baro", -(h - self.h_ref) + bias)               # NED z + bias
        if out is not None:
            self.pub_baro.publish(Float64(data=float(out)))
        self.n_baro += 1

    def on_truth(self, msg):
        p = msg.pose.pose.position                          # world ENU
        self.truth = (p.y, p.x, p.z)
        o = msg.pose.pose.orientation
        self.q_truth = (o.w, o.x, o.y, o.z)
        t = self.now_s()
        if t - self.last_tf >= 0.05:                        # the "drone" frame for RViz, 20 Hz
            self.last_tf = t
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "map"
            tf.child_frame_id = "drone"
            tf.transform.translation.x, tf.transform.translation.y = float(p.x), float(p.y)
            tf.transform.translation.z = float(self.z0_amsl + p.z - self.base_alt)
            tf.transform.rotation = o
            self.tfb.sendTransform(tf)
        if t - self.last_truth_pub >= 0.5:
            self.last_truth_pub = t
            pt = PointStamped()
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.header.frame_id = "ned"
            pt.point.x, pt.point.y = float(p.y), float(p.x)
            pt.point.z = float(-(self.z0_amsl + p.z - self.base_alt))
            self.pub_truth.publish(pt)
            ps = PoseStamped()
            ps.header = pt.header
            ps.header.frame_id = "map"
            ps.pose.position.x, ps.pose.position.y = float(p.x), float(p.y)
            ps.pose.position.z = float(self.z0_amsl + p.z - self.base_alt)
            ps.pose.orientation.w = 1.0
            self.tpath.poses.append(ps)
            self.tpath.header.stamp = pt.header.stamp
            self.pub_tpath.publish(self.tpath)

    def gz_follow(self):
        """Point Gazebo's window camera at the drone and keep it there (its "Follow"),
        once the window's camera services exist. Runs in its own thread, since a gz
        service call blocks; gives up quietly after 3 minutes (no window)."""
        x, y, z = self.follow_offset
        t_end = time.time() + 180.0
        while time.time() < t_end:
            time.sleep(3.0)
            try:
                r = subprocess.run(["gz", "service", "-s", "/gui/follow/offset", "--reqtype", "gz.msgs.Vector3d",
                                    "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                                    "--req", f"x: {x}, y: {y}, z: {z}"], capture_output=True, text=True, timeout=10)
                if "true" not in r.stdout:
                    continue
                r = subprocess.run(["gz", "service", "-s", "/gui/follow", "--reqtype", "gz.msgs.StringMsg",
                                    "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                                    "--req", f'data: "{self.gz_model}"'], capture_output=True, text=True, timeout=10)
                if "true" in r.stdout:
                    self.get_logger().info(f"Gazebo window follows the drone ({x:g}, {y:g}, {z:g} m); "
                                           "right-click it in the entity tree to change the view")
                    return
            except (OSError, subprocess.TimeoutExpired):
                continue
        self.get_logger().info("no Gazebo window answered: its camera doesn't follow the drone")

    def tick(self):
        if self.truth is None:
            return
        use_truth = self.agl_source == "truth"
        no_radar = (self.now_s() - self.t_node > 5.0 if self.last_scan_t is None
                    else self.now_s() - self.last_scan_t > 3.0)
        if self.agl_source == "auto" and no_radar:
            use_truth = True
            if not self.fallback_warned:
                self.get_logger().warn("no radar altimeter data (GPU rendering off?): AGL now comes from the "
                                       "map at the true position + 0.3 m noise")
                self.fallback_warned = True
        if use_truth:
            self.publish_agl(self.predicted_agl() + np.random.normal(0.0, 0.3))

    def report(self):
        msg = f"rates over 10 s: radar {self.n_scan / 10:.0f} Hz, mag {self.n_mag / 10:.0f} Hz, baro {self.n_baro / 10:.0f} Hz"
        if self.resid:
            r = np.array(self.resid)
            msg += (f"; radar AGL minus map-predicted AGL: mean {r.mean():+.2f} m, rms {np.sqrt((r ** 2).mean()):.2f} m, "
                    f"n={len(r)}")
        if self.mag_ned:
            f = np.mean(self.mag_ned, axis=0)
            msg += f"; field in NED {f[0]:.1f} {f[1]:.1f} {f[2]:.1f} uT (eskf_node mag_ref_* should match)"
        self.get_logger().info(msg)
        self.resid.clear()
        self.mag_ned.clear()
        self.n_scan = self.n_mag = self.n_baro = 0


def main():
    rclpy.init()
    rclpy.spin(SimAdapter())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
