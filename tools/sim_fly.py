#!/usr/bin/env python3
"""Fly PX4 (SITL) on a terrain-following lawnmower over the map, in offboard mode.

PX4 still flies on its own GNSS-based estimator here: GDN runs alongside in shadow
mode (roadmap Stage A). --gps-off-at T injects a GPS failure T seconds after take-off
(Stage B). PX4 then has no position and will fail-safe, while GDN keeps estimating.
That's expected until PX4 can use GDN's output (R5.1).

The route is planned from the same terrain map GDN uses. A "carrot" setpoint moves
along the legs at --speed. Its height is the highest terrain within --lookahead
metres along the track, plus --clearance, so climbs start early. The carrot waits
whenever the vehicle falls behind it.

    python3 tools/sim_fly.py                         # about 10 minutes of flight
    python3 tools/sim_fly.py --legs 3 --gps-off-at 120

Needs the PX4 uXRCE-DDS link (MicroXRCEAgent) and px4_msgs 1.16.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from gdn_paths import ROOT, load_map


def plan_route(alt, hdr, legs, leg_len, spacing, centre, clearance, lookahead, step=10.0):
    """Lawnmower in local NED. Returns (N, 3) points every `step` m: north, east, z (NED)."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from make_sim_world import sample_map
    cn, ce = centre
    corners = [(0.0, 0.0)]
    for k in range(legs):
        e = ce + (k - (legs - 1) / 2.0) * spacing
        n0, n1 = (cn - leg_len / 2, cn + leg_len / 2) if k % 2 == 0 else (cn + leg_len / 2, cn - leg_len / 2)
        corners += [(n0, e), (n1, e)]
    corners.append((0.0, 0.0))
    pts = []
    for (a_n, a_e), (b_n, b_e) in zip(corners[:-1], corners[1:]):
        d = math.hypot(b_n - a_n, b_e - a_e)
        for s in np.arange(0.0, d, step):
            f = s / d
            pts.append((a_n + f * (b_n - a_n), a_e + f * (b_e - a_e)))
    pts.append(corners[-1])
    pts = np.array(pts)
    base = float(sample_map(alt, hdr, np.array(0.0), np.array(0.0)))
    ground = np.array([float(sample_map(alt, hdr, np.array(e), np.array(n))) for n, e in pts]) - base
    w = max(1, int(lookahead / step))
    env = np.array([ground[max(0, i - w):i + w + 1].max() for i in range(len(ground))])
    z_ned = -(env + clearance)
    return np.column_stack([pts, z_ned]), ground


class Flyer(Node):
    def __init__(self, a):
        super().__init__("gdn_sim_fly")
        self.a = a
        alt, hdr = load_map(a.map)
        self.route, self.ground = plan_route(alt.astype(np.float64), hdr, a.legs, a.leg_len, a.spacing,
                                             (a.centre_n, a.centre_e), a.clearance, a.lookahead)
        length = float(np.sum(np.linalg.norm(np.diff(self.route[:, :2], axis=0), axis=1)))
        self.get_logger().info(f"route: {len(self.route)} points, {length / 1000:.1f} km, about "
                               f"{length / a.speed / 60:.0f} min at {a.speed} m/s; height above pad "
                               f"{-self.route[:, 2].max():.0f} to {-self.route[:, 2].min():.0f} m")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_pos, sub_qos)
        # VehicleStatus is message version 1 in PX4 1.16, so its topic carries a _v1 suffix.
        for t in ("/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"):
            self.create_subscription(VehicleStatus, t, self.on_status, sub_qos)
        self.pos = None
        self.nav_state = None
        self.armed = False
        self.idx = 0
        self.s_along = 0.0
        self.k = 0
        self.t_takeoff = None
        self.gps_off_sent = False
        self.landing = False
        self.create_timer(0.1, self.step)

    def on_pos(self, m):
        if m.xy_valid and m.z_valid:
            self.pos = np.array([m.x, m.y, m.z])

    def on_status(self, m):
        self.nav_state = m.nav_state
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = cmd
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def step(self):
        self.k += 1
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.pub_mode.publish(mode)
        if self.landing:
            return
        target = self.route[self.idx]
        if self.pos is not None and self.t_takeoff is None:
            target = np.array([self.pos[0], self.pos[1], self.route[0, 2]])    # climb straight up first
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [float(target[0]), float(target[1]), float(target[2])]
        sp.velocity = [float("nan")] * 3
        sp.acceleration = [float("nan")] * 3
        if self.idx + 1 < len(self.route):
            d = self.route[self.idx + 1] - self.route[self.idx]
            sp.yaw = float(math.atan2(d[1], d[0]))
        else:
            sp.yaw = float("nan")
        self.pub_sp.publish(sp)

        if self.k == 15:     # setpoints have been streaming for 1.5 s: switch to offboard and arm
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            self.get_logger().info("offboard + arm requested")
        if self.k > 15 and self.k % 50 == 0 and not self.armed:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        if self.pos is None:
            return
        err = target - self.pos
        if self.t_takeoff is None:
            if abs(err[2]) < 3.0 and self.armed:
                self.t_takeoff = self.k / 10.0
                self.get_logger().info(f"at cruise height ({-self.pos[2]:.0f} m above pad), starting the route")
            return
        t_flight = self.k / 10.0 - self.t_takeoff
        if self.a.gps_off_at is not None and not self.gps_off_sent and t_flight >= self.a.gps_off_at:
            self.command(VehicleCommand.VEHICLE_CMD_INJECT_FAILURE,
                         VehicleCommand.FAILURE_UNIT_SENSOR_GPS, VehicleCommand.FAILURE_TYPE_OFF)
            self.gps_off_sent = True
            self.get_logger().warn(f"GPS failure injected at t={t_flight:.0f} s "
                                   "(PX4 will fail-safe; GDN keeps estimating)")
        # Advance the carrot at --speed (route points are 10 m apart, ticks 0.1 s),
        # but hold it while the vehicle is more than 25 m behind or 10 m off in height.
        if np.linalg.norm(err[:2]) < 25.0 and abs(err[2]) < 10.0:
            self.s_along = min(self.s_along + self.a.speed * 0.1, 10.0 * (len(self.route) - 1))
            self.idx = int(self.s_along / 10.0)
        if self.idx >= len(self.route) - 1 and np.linalg.norm(err) < 5.0:
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing = True
            self.get_logger().info("route done, landing")
        if self.k % 100 == 0:
            self.get_logger().info(f"t={t_flight:5.0f} s  point {self.idx}/{len(self.route) - 1}  "
                                   f"height above ground {-self.pos[2] - self.ground[self.idx]:5.0f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=str(ROOT / "data/maps/zermatt.bin"))
    ap.add_argument("--legs", type=int, default=4)
    ap.add_argument("--leg-len", type=float, default=1400.0, dest="leg_len")
    ap.add_argument("--spacing", type=float, default=250.0)
    ap.add_argument("--centre-n", type=float, default=0.0, dest="centre_n")
    ap.add_argument("--centre-e", type=float, default=0.0, dest="centre_e")
    ap.add_argument("--clearance", type=float, default=60.0, help="m above the terrain envelope")
    ap.add_argument("--lookahead", type=float, default=200.0, help="m of track used for the envelope")
    ap.add_argument("--speed", type=float, default=10.0, help="carrot speed, m/s")
    ap.add_argument("--gps-off-at", type=float, default=None, dest="gps_off_at",
                    help="inject a GPS failure this many seconds after reaching cruise height")
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(Flyer(a))


if __name__ == "__main__":
    main()
