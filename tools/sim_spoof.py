#!/usr/bin/env python3
"""GNSS spoofing in SITL (roadmap R2.6b): PX4's GPS from a relay that can lie.

    python3 tools/sim_spoof.py --world W --model M --at 60 --rate 1.0 --bearing 90

tools/sim_run.sh starts it for GDN_SPOOF="at_s rate_mps bearing_deg", with PX4 told at
boot (PX4_PARAM_*) to take MAVLink HIL_GPS outside HIL mode (MAV_USEHILGPS 1) and to give
its own Gazebo GPS no fix (SIM_GPS_USED 0). PX4 picks the receiver with the best fix, so
this relay is its only GPS.

The relay reads Gazebo's own GPS (the model's navsat sensor, the one PX4's gz_bridge
reads) and sends each fix to PX4 as HIL_GPS, with the accuracy gz_bridge reports
(eph 0.9 m, epv 1.78 m, 10 satellites). Until the spoof starts it's the truth.

From --at seconds after the route's start (/gdn/sim/route_start, as the GNSS cut) the
reported position walks away at --rate m/s towards --bearing (degrees clockwise from
north), and the reported velocity carries the same drift, so position and velocity
agree: a slow, consistent spoof, the kind an estimator's innovation checks don't catch.
PX4 then steers its spoofed position along the route, and the vehicle itself drifts the
other way.

The offset in force goes out on /gdn/sim/spoof_offset (geometry_msgs/Vector3: north,
east, metres; once a second) for the bag and tools/check_scenario.py.

PX4 1.16 has no handler for MAVLink GPS_INPUT (only HIL_GPS, and that one only from its
own system id, 1), which is why this is HIL_GPS.
"""
import argparse
import math
import threading
import time

try:                      # offset_ne is used without ROS (tools/check_scenario.py)
    import rclpy
    from geometry_msgs.msg import Vector3
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64
except ImportError:
    Node = object

R_EARTH = 6378137.0


def offset_ne(t_route, at_s, rate, bearing_deg):
    """The spoof's (north, east) offset, metres, t_route seconds into the route."""
    if t_route is None or t_route < at_s:
        return 0.0, 0.0
    d = rate * (t_route - at_s)
    b = math.radians(bearing_deg)
    return d * math.cos(b), d * math.sin(b)


class Spoofer(Node):
    def __init__(self, a):
        super().__init__("gdn_sim_spoof")
        from pymavlink import mavutil
        self.a = a
        self.mav = mavutil.mavlink_connection(f"udpout:127.0.0.1:{a.port}", source_system=1, source_component=220)
        self.t_route0 = None
        self.n = 0
        self.lock = threading.Lock()
        self.off = (0.0, 0.0)
        self.started = False
        self.create_subscription(Float64, "/gdn/sim/route_start", self.on_start, QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1))
        self.pub = self.create_publisher(Vector3, "/gdn/sim/spoof_offset", 10)
        self.create_timer(1.0, self.tick)
        from gz.transport13 import Node as GzNode
        from gz.msgs10.navsat_pb2 import NavSat
        self.gz = GzNode()
        topic = f"/world/{a.world}/model/{a.model}/link/base_link/sensor/navsat_sensor/navsat"
        if not self.gz.subscribe(NavSat, topic, self.on_navsat):
            raise SystemExit(f"sim_spoof: couldn't subscribe to {topic}")
        self.get_logger().info(f"relaying {topic} to PX4 as HIL_GPS (udp {a.port}); spoof from {a.at:g} s into the "
                               f"route, {a.rate:g} m/s towards {a.bearing:g} deg")

    def on_start(self, msg):
        if self.t_route0 is None:
            self.t_route0 = float(msg.data)
            self.get_logger().info("route started: the spoof's clock runs")

    def on_navsat(self, m):
        t_route = None if self.t_route0 is None else time.time() - self.t_route0
        dn, de = offset_ne(t_route, self.a.at, self.a.rate, self.a.bearing)
        spoofing = t_route is not None and t_route >= self.a.at
        lat = m.latitude_deg + math.degrees(dn / R_EARTH)
        lon = m.longitude_deg + math.degrees(de / (R_EARTH * math.cos(math.radians(m.latitude_deg))))
        b = math.radians(self.a.bearing)
        vn = m.velocity_north + (self.a.rate * math.cos(b) if spoofing else 0.0)
        ve = m.velocity_east + (self.a.rate * math.sin(b) if spoofing else 0.0)
        vd = -m.velocity_up
        vel = math.hypot(vn, ve)
        cog = (math.degrees(math.atan2(ve, vn)) % 360.0) if vel > 0.3 else None
        self.mav.mav.hil_gps_send(
            int(time.time() * 1e6), 3, int(round(lat * 1e7)), int(round(lon * 1e7)), int(round(m.altitude * 1e3)),
            90, 178, int(round(vel * 100)), int(round(vn * 100)), int(round(ve * 100)), int(round(vd * 100)),
            65535 if cog is None else int(round(cog * 100)), 10)
        with self.lock:
            self.off, self.n = (dn, de), self.n + 1
            if spoofing and not self.started:
                self.started = True
                self.get_logger().warn(f"spoof started {t_route:.1f} s into the route")

    def tick(self):
        with self.lock:
            (dn, de), n = self.off, self.n
            self.n = 0
        self.pub.publish(Vector3(x=float(dn), y=float(de), z=0.0))
        if self.started and int(time.time()) % 10 == 0:
            self.get_logger().info(f"spoof offset {math.hypot(dn, de):.1f} m (north {dn:+.1f}, east {de:+.1f}); "
                                   f"{n} fixes/s to PX4")
        elif n == 0:
            self.get_logger().warn("no fix from Gazebo's GPS in the last second")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--world", required=True)
    ap.add_argument("--model", required=True, help="the vehicle's Gazebo model instance")
    ap.add_argument("--at", type=float, required=True, help="seconds after the route's start")
    ap.add_argument("--rate", type=float, required=True, help="m/s")
    ap.add_argument("--bearing", type=float, required=True, help="deg clockwise from north")
    ap.add_argument("--port", type=int, default=14580, help="PX4's onboard MAVLink port")
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(Spoofer(a))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
