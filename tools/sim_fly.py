#!/usr/bin/env python3
"""Fly PX4 (SITL) on a terrain-following lawnmower over the map, in offboard mode.

--vehicle picks one of tools/sim_vehicles.py (roadmap R2.3):
  x500           climbs straight up, flies the route on a moving "carrot", lands on the pad.
  standard_vtol  climbs straight up like the x500, changes to fixed-wing flight (PX4's
                 front transition) and flies the route as a plane, changes back 350 m
                 before the pad (back transition) and lands like the x500. Both
                 transitions happen in Hold, PX4's own mode, not in offboard: in PX4 1.16
                 the transition flight task publishes trajectory_setpoint itself, the same
                 topic offboard setpoints arrive on, so in offboard the plane controller
                 sees no valid setpoint, gives no forward thrust, and the VTOL hangs and
                 sinks until PX4 aborts the transition (its quad-chute, 20 m lost).
  rc_cessna      PX4's runway take-off (Takeoff mode) along the world's runway, the
                 route as a plane, then a landing mission onto the runway: an approach
                 point on PX4's 8 degree glide path, from whichever end has more terrain
                 clearance, and the touchdown point, uploaded over MAVLink (pymavlink;
                 PX4 lands a plane on a runway only from a mission). --launch catapult:
                 Takeoff mode with launch detection instead, and this script asks Gazebo
                 to push it forward at 4 g until it's at 16 m/s, as a catapult would.
                 A world without a runway: it circles over the pad (Hold) for 30 s.
As a plane, the setpoint is a point on the route 60 m ahead of the vehicle's place on it,
with the route's direction as velocity: PX4 then follows the route as a line
(FixedwingPositionControl's "offboard position with velocity setpoints").

PX4 still flies on its own GNSS-based estimator here: GDN runs alongside in shadow
mode (roadmap Stage A). --gps-off-at T cuts GPS T seconds after take-off (Stage B).
PX4 then loses its position and fails safe, while GDN keeps estimating. That's expected
until PX4 can use GDN's output (R5.1).

The cut sets PX4's EKF2_GPS_CTRL to 0 (through px4-param in --px4-dir), so PX4's
estimator stops using GPS. PX4's failure injection (VEHICLE_CMD_INJECT_FAILURE, the
`failure gps off` command) doesn't work here: in PX4 1.16 only the MAVLink simulator
(Gazebo Classic, jMAVSim) acts on it, and gz_bridge passes every GPS reading on. With
it, PX4 kept a 0.2 m position through every Stage B run until TEST-032.

At the route's start (cruise height; a fixed wing: climbed out) it publishes
/gdn/sim/route_start once (std_msgs/Float64, the unix time; reliable, transient local):
the spoofer and the sensor faults (R2.6b) count their times from it, as the cut does.

The route is planned from the same terrain map GDN uses. A "carrot" setpoint moves
along the legs at --speed. Its height is the highest terrain within --lookahead
metres along the track, plus --clearance, so climbs start early. The carrot waits
whenever the vehicle falls behind it.

    python3 tools/sim_fly.py                         # about 10 minutes of flight
    python3 tools/sim_fly.py --legs 3 --gps-off-at 120
    python3 tools/sim_fly.py --vehicle standard_vtol --gz-world zermatt

Needs the PX4 uXRCE-DDS link (MicroXRCEAgent) and px4_msgs 1.16.
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float64
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from gdn_paths import ROOT, load_map
import sim_vehicles


def plan_route(alt, hdr, legs, leg_len, spacing, centre, clearance, lookahead, step=10.0, climb_grad=0.0):
    """Lawnmower in local NED. Returns (N, 3) points every `step` m: north, east, z (NED).

    climb_grad > 0 (fixed-wing): the height never changes faster than that per metre
    of track, climbs starting early enough and descents waiting until they're clear."""
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
    if climb_grad > 0:
        dz = climb_grad * step
        for i in range(len(env) - 2, -1, -1):      # climbs: be high enough before the hill
            env[i] = max(env[i], env[i + 1] - dz)
        for i in range(1, len(env)):               # descents: only as fast as allowed
            env[i] = max(env[i], env[i - 1] - dz)
    z_ned = -(env + clearance)
    return np.column_stack([pts, z_ned]), ground


FW_LEAD = 6                    # route points (60 m) ahead of the vehicle's place on the route
VTOL_BACK_TRANSITION_M = 350.0 # track left when the VTOL changes back to hover
CATAPULT_ACCEL = 40.0          # m/s2 of push (PX4 detects a launch at 30)
CATAPULT_SPEED = 16.0          # m/s at which the push stops
CATAPULT_MASS = 1.6            # kg, the rc_cessna with the payload
FW_HOLD_S = 30.0               # fixed-wing circling over the pad at the end (no runway)
LAND_TOUCHDOWN_M = 50.0        # touchdown this far into the runway
LAND_APPROACH_M = 600.0        # approach point this far before the touchdown (at least)
LAND_CLEARANCE_M = 20.0        # terrain at least this far under the glide path
LAND_TIMEOUT_S = 300.0         # from the landing mission to the disarm
EARTH_R = 6371000.0            # PX4's MapProjection radius
FAILSAFE_WAIT_S = 180.0        # after a fail-safe, how long to wait for a disarm
TAKEOFF_TIMEOUT_S = 180.0      # from start to cruise height (the climb or the launch)


class Flyer(Node):
    def __init__(self, a):
        super().__init__("gdn_sim_fly")
        self.a = a
        self.v = sim_vehicles.get(a.vehicle)
        self.kind = self.v.kind
        self.speed = a.speed if a.speed is not None else self.v.speed
        self.px4_param = a.px4_param
        alt, hdr = load_map(a.map)
        self.route, self.ground = plan_route(alt.astype(np.float64), hdr, a.legs, a.leg_len, a.spacing,
                                             (a.centre_n, a.centre_e), a.clearance, a.lookahead,
                                             climb_grad=self.v.climb_grad)
        seg = np.linalg.norm(np.diff(self.route[:, :2], axis=0), axis=1)
        self.left = np.concatenate([np.cumsum(seg[::-1])[::-1], [0.0]])   # track left from each point, m
        self.get_logger().info(f"{a.vehicle} ({self.kind}); route: {len(self.route)} points, "
                               f"{self.left[0] / 1000:.1f} km, about {self.left[0] / self.speed / 60:.0f} min at "
                               f"{self.speed:g} m/s; height above pad {-self.route[:, 2].max():.0f} to "
                               f"{-self.route[:, 2].min():.0f} m")
        if a.route_out:
            np.savetxt(a.route_out, np.column_stack([self.route, self.ground]), delimiter=",", fmt="%.2f",
                       header="north_m,east_m,z_ned_m,ground_above_pad_m", comments="")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.pub_start = self.create_publisher(Float64, "/gdn/sim/route_start", QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1))
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_pos, sub_qos)
        # VehicleStatus is message version 1 in PX4 1.16, so its topic carries a _v1 suffix.
        for t in ("/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_status"):
            self.create_subscription(VehicleStatus, t, self.on_status, sub_qos)
        self.pos = None
        self.vel = np.zeros(3)
        self.nav_state = None
        self.armed = False
        self.vehicle_type = None
        self.in_transition = False
        # climb (take-off), route_mc (carrot), route_fw (as a plane), to_pad (VTOL after
        # its back transition), landing, hold (fixed-wing at the end); the VTOL's
        # to_fw and to_mc (transitions, in Hold)
        self.phase = "climb"
        self.phase_k = 0
        self.idx = 0
        self.s_along = 0.0
        self.k = 0
        self.t_takeoff = None
        self.gps_off_sent = False
        self.gps_off_proc = None
        self.failsafe = False
        self.failsafe_k = None
        self.launched = False
        self.hold_until = None
        self.ref = None                 # PX4's EKF origin (lat, lon, alt AMSL)
        self.landing_plan = None        # [(north, east, alt above the origin)] approach, touchdown
        self.landing_uploaded = None    # None: not tried; True/False: the upload's outcome
        self.landing_started = False
        if self.kind == "fixedwing" and a.world_info:
            self.plan_landing(alt.astype(np.float64), hdr, a.world_info)
        self.done = False
        self.create_timer(0.1, self.step)

    def on_pos(self, m):
        if m.xy_valid and m.z_valid:
            self.pos = np.array([m.x, m.y, m.z])
            self.vel = np.array([m.vx, m.vy, m.vz])
        if self.ref is None and m.xy_global and m.z_global:
            self.ref = (m.ref_lat, m.ref_lon, m.ref_alt)

    def on_status(self, m):
        self.nav_state = m.nav_state
        self.armed = m.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.vehicle_type = m.vehicle_type
        self.in_transition = m.in_transition_mode

    def check_gps_cut(self):
        """Report how the px4-param call that cuts GPS ended, once it has."""
        if self.gps_off_proc is None or self.gps_off_proc.poll() is None:
            return
        out = (self.gps_off_proc.stdout.read() or "").strip().replace("\n", " | ")
        if self.gps_off_proc.returncode == 0:
            self.get_logger().info(f"PX4 took EKF2_GPS_CTRL 0 ({out or 'no output'})")
        else:
            self.get_logger().error(f"px4-param failed (exit {self.gps_off_proc.returncode}): {out}; "
                                    "GPS is still on")
        self.gps_off_proc = None

    def maybe_cut_gps(self, t_flight):
        if self.a.gps_off_at is None or self.gps_off_sent or t_flight < self.a.gps_off_at:
            return
        # Not waited for here: the setpoint stream must keep going at 10 Hz.
        self.gps_off_proc = subprocess.Popen([str(self.px4_param), "set", "EKF2_GPS_CTRL", "0"],
                                             cwd=self.px4_param.parent.parent / "rootfs",
                                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.gps_off_sent = True
        self.get_logger().warn(f"GPS cut at t={t_flight:.0f} s: EKF2_GPS_CTRL 0 "
                               "(PX4 will fail-safe; GDN keeps estimating)")

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def command(self, cmd, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = cmd
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.param4 = m.param5 = m.param6 = m.param7 = float("nan")
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def setpoint(self, pos, vel=None, yaw=float("nan")):
        sp = TrajectorySetpoint()
        sp.timestamp = self.now_us()
        sp.position = [float(v) for v in pos]
        sp.velocity = [float(v) for v in vel] if vel is not None else [float("nan")] * 3
        sp.acceleration = [float("nan")] * 3
        sp.yaw = float(yaw)
        self.pub_sp.publish(sp)

    def t_flight(self):
        return self.k / 10.0 - self.t_takeoff

    def progress(self):
        if self.k % 100 == 0 and self.pos is not None and self.t_takeoff is not None:
            self.get_logger().info(f"t={self.t_flight():5.0f} s  point {self.idx}/{len(self.route) - 1}  "
                                   f"height above ground {-self.pos[2] - self.ground[self.idx]:5.0f} m")

    def step(self):
        self.k += 1
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.pub_mode.publish(mode)
        self.check_gps_cut()
        if (self.gps_off_sent and not self.failsafe and self.nav_state is not None
                and self.phase in ("route_mc", "route_fw", "to_pad")
                and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.failsafe, self.failsafe_k = True, self.k
            self.get_logger().warn(f"PX4 left offboard {self.t_flight():.0f} s into the route, "
                                   f"after the GPS cut (nav_state {self.nav_state}): its fail-safe flies now")
        if self.phase == "landing" or self.failsafe:
            # PX4 disarms by itself once it has landed (COM_DISARM_LAND). Then this
            # script is done, and tools/sim_run.sh goes on to score and stop the run.
            if not self.armed and not self.done:
                self.done = True
                self.get_logger().info("landed and disarmed, done")
            elif (self.kind == "fixedwing" and not self.failsafe and not self.done
                  and self.k - self.phase_k > LAND_TIMEOUT_S * 10):
                self.done = True
                self.get_logger().error(f"not landed {LAND_TIMEOUT_S:.0f} s after starting the landing, done")
            elif self.failsafe and self.k - self.failsafe_k > FAILSAFE_WAIT_S * 10 and not self.done:
                self.done = True
                self.get_logger().info(f"still armed {FAILSAFE_WAIT_S:.0f} s after the fail-safe, done")
            return
        if self.t_takeoff is None and self.k > TAKEOFF_TIMEOUT_S * 10:
            self.done = True
            self.get_logger().error(f"not at cruise height after {TAKEOFF_TIMEOUT_S:.0f} s (phase {self.phase}, "
                                    f"nav_state {self.nav_state}, armed {self.armed}): giving up")
            return
        if self.phase == "hold":
            if self.k >= self.hold_until:
                self.done = True
                self.get_logger().info(f"circled over the pad for {FW_HOLD_S:.0f} s, done "
                                       "(the fixed-wing doesn't land: no runway)")
            return
        if self.kind == "fixedwing" and self.phase == "climb":
            self.fw_takeoff()
        elif self.phase in ("to_fw", "to_mc"):
            self.vtol_transition()
        elif self.phase == "route_fw":
            self.fw_step()
        else:
            self.mc_step()

    # --- hover flight: x500, and the VTOL's climb and its last stretch to the pad ---
    def mc_step(self):
        target = self.route[self.idx]
        if self.phase == "climb" and self.pos is not None:
            target = np.array([self.pos[0], self.pos[1], self.route[0, 2]])    # climb straight up first
        if self.phase == "to_pad":
            target = self.route[-1]
        yaw = float("nan")
        if self.phase != "to_pad" and self.idx + 1 < len(self.route):
            d = self.route[self.idx + 1] - self.route[self.idx]
            yaw = math.atan2(d[1], d[0])
        self.setpoint(target, yaw=yaw)

        if self.k == 15:     # setpoints have been streaming for 1.5 s: switch to offboard and arm
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
            self.get_logger().info("offboard + arm requested")
        if self.k > 15 and self.k % 50 == 0 and not self.armed and self.t_takeoff is None:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
        if self.pos is None:
            return
        err = target - self.pos
        if self.phase == "climb":
            if abs(err[2]) < 3.0 and self.armed:
                self.start_route()
                self.get_logger().info(f"at cruise height ({-self.pos[2]:.0f} m above pad), starting the route")
                if self.kind == "vtol":
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3)    # AUTO / LOITER: Hold
                    self.command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 4)
                    self.set_phase("to_fw")
                    self.get_logger().info("front transition requested (to fixed-wing flight, in Hold)")
                else:
                    self.set_phase("route_mc")
            return
        self.maybe_cut_gps(self.t_flight())
        if self.phase == "to_pad":
            if self.over_pad(err):
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_phase("landing")
                self.get_logger().info("over the pad, landing")
            self.progress()
            return
        # Advance the carrot at the route speed (route points are 10 m apart, ticks 0.1 s),
        # but hold it while the vehicle is more than 25 m behind or 10 m off in height.
        if np.linalg.norm(err[:2]) < 25.0 and abs(err[2]) < 10.0:
            self.s_along = min(self.s_along + self.speed * 0.1, 10.0 * (len(self.route) - 1))
            self.idx = int(self.s_along / 10.0)
        if self.idx >= len(self.route) - 1 and self.over_pad(err):
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.set_phase("landing")
            self.get_logger().info("route done, landing")
        self.progress()

    def over_pad(self, err):
        """Settled over the last route point: PX4's Land mode descends where the vehicle
        is, so landing while still moving put the x500 10 m off the pad."""
        return (np.hypot(err[0], err[1]) < 1.5 and abs(err[2]) < 3.0
                and float(np.linalg.norm(self.vel)) < 0.5)

    # --- flight as a plane: the VTOL between its transitions, and the fixed-wing ---
    def fw_setpoint(self):
        """Point FW_LEAD ahead of the vehicle's place on the route, with the route's
        direction as velocity, so PX4 follows the route as a line."""
        if self.pos is not None:
            w = self.route[self.idx:self.idx + 40, :2]
            self.idx += int(np.argmin(np.linalg.norm(w - self.pos[:2], axis=1)))
        j = min(self.idx + FW_LEAD, len(self.route) - 1)
        d = self.route[j] - self.route[j - 1] if j > 0 else self.route[1] - self.route[0]
        vel = d / max(float(np.linalg.norm(d[:2])), 1e-6) * self.speed
        self.setpoint(self.route[j], vel=vel)

    def fw_step(self):
        self.fw_setpoint()
        if self.pos is None:
            return
        self.maybe_cut_gps(self.t_flight())
        if self.kind == "vtol":
            if self.left[self.idx] < VTOL_BACK_TRANSITION_M:
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3)        # AUTO / LOITER: Hold
                self.command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, 3)
                self.set_phase("to_mc")
                self.get_logger().info(f"back transition requested (to hover, in Hold), {self.left[self.idx]:.0f} m "
                                       "of route left")
        elif self.left[self.idx] < 2 * FW_LEAD * 10.0 and self.landing_uploaded:
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 4)     # AUTO / MISSION: the landing
            self.set_phase("landing")
            self.get_logger().info("route done, landing on the runway (PX4 mission: approach, glide, touchdown)")
        elif self.left[self.idx] < 2 * FW_LEAD * 10.0 and np.linalg.norm(self.route[-1, :2] - self.pos[:2]) < 80.0:
            if self.landing_plan:
                self.get_logger().error("the landing mission didn't upload: circling over the pad instead")
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 3)     # AUTO / LOITER: Hold
            self.set_phase("hold")
            self.hold_until = self.k + int(FW_HOLD_S * 10)
            self.get_logger().info(f"route done, circling over the pad (Hold) for {FW_HOLD_S:.0f} s")
        self.progress()

    def start_route(self):
        """The route's start: the time GPS cuts, spoofing and sensor faults count from."""
        self.t_takeoff = self.k / 10.0
        self.pub_start.publish(Float64(data=time.time()))

    def set_phase(self, phase):
        self.phase, self.phase_k = phase, self.k

    def vtol_transition(self):
        """In Hold while PX4 changes over; no setpoints (they'd clobber Hold's own). Back
        to offboard once it flies the other way."""
        fw = self.vehicle_type == VehicleStatus.VEHICLE_TYPE_FIXED_WING and not self.in_transition
        mc = self.vehicle_type == VehicleStatus.VEHICLE_TYPE_ROTARY_WING and not self.in_transition
        waited = (self.k - self.phase_k) / 10.0
        if self.phase == "to_fw" and fw:
            self.get_logger().info(f"front transition done after {waited:.0f} s, flying as a plane: offboard, "
                                   "following the route")
            self.fw_setpoint()
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.set_phase("route_fw")
        elif self.phase == "to_fw" and mc and waited > 5.0:
            self.get_logger().error(f"front transition failed (PX4's quad-chute?) after {waited:.0f} s: "
                                    "flying the route hovering instead")
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.set_phase("route_mc")
        elif self.phase == "to_mc" and mc and waited > 1.0:
            self.get_logger().info(f"back transition done after {waited:.0f} s, hovering: offboard, to the pad")
            self.setpoint(self.route[-1])
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.set_phase("to_pad")
        if self.pos is not None:
            self.progress()

    def fw_takeoff(self):
        """Fixed-wing: Takeoff mode, from the runway or (--launch catapult) with launch
        detection and a Gazebo push. No setpoints until offboard takes over."""
        if not self.launched and self.k >= 15 and self.k % 50 == 15:
            if not (self.armed and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 4, 2)     # AUTO / TAKEOFF
                self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1)
                if self.k == 15:
                    self.get_logger().info("Takeoff mode + arm requested" + (" (PX4 waits for the launch)"
                                           if self.a.launch == "catapult" else ""))
        if (not self.launched and self.armed and self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
            self.launched = True
            if self.a.launch == "catapult":
                threading.Thread(target=self.catapult, daemon=True).start()
            else:
                self.get_logger().info("runway take-off: PX4 rolls down the runway and climbs out")
        top = float(self.v.params.get("MIS_TAKEOFF_ALT", 50.0))
        if self.launched and self.pos is not None and -self.pos[2] >= top - 5.0:
            self.start_route()
            self.fw_setpoint()
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.set_phase("route_fw")
            # The landing goes up now, not before take-off: PX4 sets home when it arms,
            # and checked a mission uploaded earlier against the previous run's home
            # (112 km away); that landing then skipped its approach and circled down.
            if self.landing_plan and self.ref is not None and self.landing_uploaded is None:
                self.landing_uploaded = False
                threading.Thread(target=self.upload_landing, daemon=True).start()
            self.get_logger().info(f"climbed out ({-self.pos[2]:.0f} m above pad): offboard, starting the route")

    def plan_landing(self, alt, hdr, world_info):
        """Approach and touchdown on the world's runway (world_info.txt, from
        make_sim_world.py), from the end whose approach keeps the terrain furthest under
        PX4's glide path (FW_LND_ANG 8 degrees on the rc_cessna)."""
        info = dict(l.split("=", 1) for l in Path(world_info).read_text().splitlines() if "=" in l)
        rw = sim_vehicles.runway(info)
        if rw is None:
            return
        from gdn_paths import load_map as lm
        from make_sim_world import sample_map
        walt, whdr = lm(info.get("world_map", info["map"]))
        walt = walt.astype(np.float64)
        base = float(sample_map(walt, whdr, np.array(0.0), np.array(0.0)))
        h = math.radians(rw["heading"])
        tan_g = math.tan(math.radians(8.0))
        best = None
        for d in (1, -1):                      # land along the heading, or against it
            along_td = rw["from"] + LAND_TOUCHDOWN_M if d > 0 else rw["to"] - LAND_TOUCHDOWN_M
            alt_td = rw["grade"] * along_td
            s = np.arange(150.0, 2000.0, 10.0)                  # distance back from touchdown
            along = along_td - d * s
            ground = sample_map(walt, whdr, along * math.sin(h), along * math.cos(h)) - base
            glide = alt_td + s * tan_g
            # The approach point: at least LAND_APPROACH_M back, and far enough that the
            # glide path from it clears all terrain behind it by LAND_CLEARANCE_M.
            need = np.nonzero(ground + LAND_CLEARANCE_M > glide)[0]
            app = max(LAND_APPROACH_M, float(s[need[-1]]) + 50.0 if len(need) else 0.0)
            margin = float(np.min((glide - ground)[s <= app]))
            if app < 1500.0 and (best is None or margin > best[0]):
                a_app = along_td - d * app
                # 3% under the glide path: PX4's mission check rejects a steeper approach.
                best = (margin, d, [(a_app * math.cos(h), a_app * math.sin(h), alt_td + 0.97 * app * tan_g),
                                    (along_td * math.cos(h), along_td * math.sin(h), alt_td)])
        if best is None:
            self.get_logger().error("no landing approach clears the terrain: the plane will circle instead")
            return
        self.landing_plan = best[2]
        (an, ae, aa), (tn, te, ta) = self.landing_plan
        self.get_logger().info(f"landing plan: towards {(rw['heading'] if best[1] > 0 else rw['heading'] + 180) % 360:.0f} deg, "
                               f"approach point {math.hypot(an - tn, ae - te):.0f} m out at {aa:.0f} m, touchdown "
                               f"{math.hypot(tn, te):.0f} m from the pad; terrain at least {best[0]:.0f} m under the glide path")

    def upload_landing(self):
        """Upload the landing as a PX4 mission over MAVLink (PX4 SITL's onboard link,
        UDP 14540): the approach waypoint, then NAV_LAND."""
        try:
            from pymavlink import mavutil
            m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245, source_component=190)
            if m.wait_heartbeat(timeout=30) is None:
                raise RuntimeError("no MAVLink heartbeat from PX4 on UDP 14540")
            lat0, lon0, alt0 = self.ref

            def ll(n, e):
                return (int(round((lat0 + math.degrees(n / EARTH_R)) * 1e7)),
                        int(round((lon0 + math.degrees(e / (EARTH_R * math.cos(math.radians(lat0))))) * 1e7)))
            (an, ae, aa), (tn, te, ta) = self.landing_plan
            mv = mavutil.mavlink
            nan = float("nan")
            # No DO_LAND_START: PX4 rejects it in a position frame (MAV_MISSION_UNSUPPORTED),
            # and switching to Mission starts at the first item anyway.
            items = [(mv.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, nan, *ll(an, ae), alt0 + aa),
                     (mv.MAV_CMD_NAV_LAND, 0, 0, 0, nan, *ll(tn, te), alt0 + ta)]
            m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
            t_end = time.time() + 20.0
            while time.time() < t_end:
                r = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=2)
                if r is None:
                    continue
                if r.get_type() == "MISSION_ACK":
                    if r.type != mv.MAV_MISSION_ACCEPTED:
                        raise RuntimeError(f"PX4 rejected the mission (MAV_MISSION_RESULT {r.type})")
                    # Start at the approach point, whatever PX4 kept from an earlier mission.
                    m.mav.mission_set_current_send(m.target_system, m.target_component, 0)
                    texts = []
                    t_txt = time.time() + 2.0
                    while time.time() < t_txt:
                        st = m.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
                        if st is not None:
                            texts.append(st.text.strip())
                    self.landing_uploaded = True
                    self.get_logger().info("landing mission uploaded" + (f" (PX4: {'; '.join(texts)})" if texts else ""))
                    return
                cmd, p1, p2, p3, p4, x, y, z = items[r.seq]
                m.mav.mission_item_int_send(m.target_system, m.target_component, r.seq,
                                            mv.MAV_FRAME_GLOBAL_INT, cmd, 0, 1, p1, p2, p3, p4, x, y, z, 0)
            raise RuntimeError("the mission upload timed out")
        except Exception as e:                     # noqa: BLE001  (reported, then the plane circles)
            self.get_logger().error(f"landing mission: {e}")

    def catapult(self):
        """Push the plane forward at CATAPULT_ACCEL until it's at CATAPULT_SPEED, as a
        catapult would: Gazebo's ApplyLinkWrench system (in PX4's server config) applies
        the force until it's cleared."""
        from gz.transport13 import Node as GzNode
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.odometry_pb2 import Odometry
        gz = GzNode()
        st = {}

        def on_odom(m):
            v = m.twist.linear
            st["speed"] = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            q = m.pose.orientation
            st["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        gz.subscribe(Odometry, "/gdn/truth_odom", on_odom)
        push = gz.advertise(f"/world/{self.a.gz_world}/wrench/persistent", EntityWrench)
        clear = gz.advertise(f"/world/{self.a.gz_world}/wrench/clear", Entity)
        t0 = time.time()
        while time.time() - t0 < 10.0 and not ("yaw" in st and push.has_connections() and clear.has_connections()):
            time.sleep(0.05)
        if "yaw" not in st or not push.has_connections():
            self.get_logger().error(f"catapult: no Gazebo wrench topic or truth on world {self.a.gz_world!r} "
                                    "(is ApplyLinkWrench loaded?); the plane stays on the pad")
            return
        w = EntityWrench()
        w.entity.name = f"{self.v.instance}::base_link"
        w.entity.type = Entity.LINK
        f = CATAPULT_MASS * CATAPULT_ACCEL
        w.wrench.force.x, w.wrench.force.y = f * math.cos(st["yaw"]), f * math.sin(st["yaw"])
        c = Entity()
        c.name, c.type = w.entity.name, Entity.LINK
        hdg = (90.0 - math.degrees(st["yaw"])) % 360.0
        self.get_logger().info(f"catapult: pushing at {CATAPULT_ACCEL:.0f} m/s2 towards {hdg:.0f} deg")
        push.publish(w)
        t0 = time.time()
        while st["speed"] < CATAPULT_SPEED and time.time() - t0 < 3.0:
            time.sleep(0.01)
        clear.publish(c)
        self.get_logger().info(f"catapult: released at {st['speed']:.1f} m/s after {time.time() - t0:.2f} s")


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
    ap.add_argument("--vehicle", default="x500", choices=list(sim_vehicles.VEHICLES),
                    help="which simulated vehicle PX4 flies (tools/sim_vehicles.py)")
    ap.add_argument("--speed", type=float, default=None,
                    help="route speed, m/s (default the vehicle's: x500 10, planes 15)")
    ap.add_argument("--gz-world", default="zermatt", dest="gz_world",
                    help="Gazebo world name (the fixed-wing's catapult pushes through it)")
    ap.add_argument("--world-info", default=None, dest="world_info",
                    help="the world's world_info.txt: its runway, for the fixed-wing's landing")
    ap.add_argument("--launch", choices=("runway", "catapult"), default="runway",
                    help="fixed-wing take-off: PX4's runway take-off, or a catapult push")
    ap.add_argument("--route-out", default=None, dest="route_out",
                    help="write the planned route here (CSV, local NED, for tools/check_vehicles.py)")
    ap.add_argument("--gps-off-at", type=float, default=None, dest="gps_off_at",
                    help="cut GPS this many seconds after reaching cruise height")
    ap.add_argument("--px4-dir", default=os.environ.get("PX4_DIR", str(Path.home() / "PX4-Autopilot-1.16")),
                    dest="px4_dir", help="PX4 checkout with SITL built, for px4-param (default $PX4_DIR)")
    a = ap.parse_args()
    a.px4_param = Path(a.px4_dir) / "build/px4_sitl_default/bin/px4-param"
    if a.gps_off_at is not None and not a.px4_param.exists():
        sys.exit(f"sim_fly: --gps-off-at needs {a.px4_param} (set --px4-dir or PX4_DIR)")
    rclpy.init()
    node = Flyer(a)
    try:
        while rclpy.ok() and not node.done:   # until landed and disarmed (or Ctrl+C)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
