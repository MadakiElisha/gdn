#!/usr/bin/env python3
"""Turn Gazebo's raw sensors into GDN's inputs, and publish truth for scoring.

In SITL (tools/sim_run.sh), ros_gz_bridge brings these Gazebo topics into ROS:
  /gdn/sim/agl_scan      sensor_msgs/LaserScan      downward radar altimeter (slant range)
  /gdn/sim/mag           sensor_msgs/MagneticField  x500 magnetometer (see on_mag for its units)
  /gdn/sim/air_pressure  sensor_msgs/FluidPressure  x500 barometer, Pa
  /gdn/sim/truth_odom    nav_msgs/Odometry          true pose, world ENU, origin = map centre
plus PX4's /fmu/out/vehicle_attitude for removing tilt from the altimeter range.

It publishes exactly what eskf_node consumes, the same as the replay simulators:
  /gdn/agl_meas   Float64      height above ground, tilt removed (2 Hz)
  /gdn/baro_meas  Float64      NED z from pressure, relative to the start, + optional bias
  /gdn/mag_meas   Vector3      body FRD, microtesla
  /gdn/truth      PointStamped true north/east/down (for replay_scorer)
  /gdn/truth_path nav_msgs/Path in the RViz "map" frame (ENU, z = 0 at the map centre)

Altimeter check: every 10 s it logs how far the measured AGL is from the AGL the map
predicts at the true position (truth altitude minus map height). That measures how
well the rendered Gazebo terrain matches GDN's map, live, on the machine that runs it.
If the radar isn't publishing (no GPU rendering), agl_source:=auto falls back to that
map prediction plus 0.3 m noise, and says so.

Parameters: world_info (data/sim/<world>/world_info.txt), agl_source (auto|lidar|truth),
agl_rate_hz (2), baro_bias_m (0), baro_drift_m_per_min (0), max_tilt_deg (35).
"""
import math
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3
from nav_msgs.msg import Odometry, Path as PathMsg
from sensor_msgs.msg import FluidPressure, LaserScan, MagneticField
from std_msgs.msg import Float64
from px4_msgs.msg import VehicleAttitude

from gdn_paths import ROOT, load_map


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

        info = dict(l.split("=", 1) for l in Path(info_path).read_text().split() if "=" in l)
        self.z0_amsl = float(info["z0_amsl"])
        self.alt, self.hdr = load_map(info["map"])
        self.alt = self.alt.astype(np.float64)
        import sys
        sys.path.insert(0, str(Path(__file__).resolve().parent))
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
        self.tpath = PathMsg()
        self.tpath.header.frame_id = "map"
        self.create_timer(0.5, self.tick)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"sim adapter: map {info['map']}, world z=0 at {self.z0_amsl:.1f} m AMSL, "
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
        self.pub_agl.publish(Float64(data=float(value)))

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
        self.pub_mag.publish(Vector3(x=float(m[0]), y=float(m[1]), z=float(m[2])))
        self.n_mag += 1
        if self.q_truth is not None:                        # field in NED, for the mag_ref check
            self.mag_ned.append(self.r_ned_frd() @ m)

    def on_baro(self, msg):
        h = pressure_altitude(msg.fluid_pressure)
        t = self.now_s()
        if self.h_ref is None:
            self.h_ref, self.t_start = h, t
        bias = self.baro_bias + self.baro_drift * (t - self.t_start)
        self.pub_baro.publish(Float64(data=-(h - self.h_ref) + bias))   # NED z + bias
        self.n_baro += 1

    def on_truth(self, msg):
        p = msg.pose.pose.position                          # world ENU
        self.truth = (p.y, p.x, p.z)
        o = msg.pose.pose.orientation
        self.q_truth = (o.w, o.x, o.y, o.z)
        t = self.now_s()
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

    def tick(self):
        if self.truth is None:
            return
        use_truth = self.agl_source == "truth"
        if self.agl_source == "auto" and (self.last_scan_t is None or self.now_s() - self.last_scan_t > 3.0):
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
