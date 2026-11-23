#!/usr/bin/env python3
"""Did a SITL run do what its scenario asked? (roadmap R2.6a, TEST-040)

    python3 tools/check_scenario.py --run RUN_DIR                    # the run's scenario.yaml
    python3 tools/check_scenario.py --run RUN_DIR --scenario FILE    # any sim_run.sh log directory
    python3 tools/check_scenario.py --selfcheck                      # the geometry, on made-up data

RUN_DIR is a tools/run_scenario.py run (or a tools/sim_run.sh log directory). Needs ROS 2
for the bag and GDN's run log (the dev image).

  - the run is the scenario's: vehicle and world (run_info.txt);
  - PX4 flew the route and landed (fly.log), and the runner's limits held (result.json);
  - the route is the scenario's: its planned length from the legs, leg length and spacing
    within 1%, and never lower than the clearance asked over the terrain (route.csv);
  - GNSS: with "cut", PX4 lost GPS no earlier than asked and within 30 s of it (its own
    position estimate in GDN's run log: invalid, or eph over 10 m); with "none", it never did;
  - start error: once GDN's heading was aligned its position was the asked distance (1 m)
    and bearing (2 deg) from the truth; without one, within 5 m;
  - wind: the run's world and server config carry it, and the vehicle leaned or crabbed
    into it (truth attitude: a multirotor's thrust axis tilts upwind on the route, a
    plane's or a VTOL's in cruise nose points upwind of its track); without wind, neither;
  - the runner's recording and teardown (result.json): the bag and GDN's run log written,
    the run log replays bit for bit, nothing of the simulation left running.
"""
import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
GNSS_EPH_LOST, GNSS_LATE_S = 10.0, 30.0
START_DIST_TOL, START_BEAR_TOL, START_NONE_TOL = 1.0, 2.0, 5.0
# Wind response, from TEST-040's flights: the x500 leaned 4.4 deg upwind in 6 m/s (0.46 deg,
# not upwind, over a whole route without wind); the VTOL crabbed 19.3 deg (about asin(6/17)).
# Rotor drag leans a multirotor along its travel, so "no wind" is judged only over a whole
# route, whose legs cancel.
WIND_MIN_DEG = {"multirotor": 1.5, "plane": 5.0}
CALM_MAX_DEG = {"multirotor": 1.0, "plane": 3.0}
WIND_DIR_TOL = 35.0


def ck(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
    return bool(ok)


def note(what):
    print(f"note  {what}")


# --- geometry (unit-checked by --selfcheck) ---
def planned_length(legs, leg_len, spacing):
    """tools/sim_fly.py's lawnmower: pad -> first leg's start -> legs joined by steps -> pad."""
    corners = [(0.0, 0.0)]
    for k in range(legs):
        e = (k - (legs - 1) / 2.0) * spacing
        n0, n1 = (-leg_len / 2, leg_len / 2) if k % 2 == 0 else (leg_len / 2, -leg_len / 2)
        corners += [(n0, e), (n1, e)]
    corners.append((0.0, 0.0))
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(corners[:-1], corners[1:])))


def bearing_deg(dn, de):
    return math.degrees(math.atan2(de, dn)) % 360.0


def angle_diff(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def upwind_unit(from_deg):
    """(north, east) unit vector pointing to where the wind comes from."""
    b = math.radians(from_deg)
    return np.array([math.cos(b), math.sin(b)])


def thrust_tilt_ne(q_wxyz):
    """Horizontal part (north, east) of the body's up axis, from Gazebo's world-ENU
    model attitude: where a rotorcraft's thrust leans."""
    w, x, y, z = q_wxyz
    # Third column of the rotation matrix: the body z axis in the world (ENU).
    zx = 2 * (x * z + w * y)
    zy = 2 * (y * z - w * x)
    return np.array([zy, zx])          # ENU (x east, y north) -> (north, east)


def heading_ne(q_wxyz):
    """The body's forward axis, horizontal (north, east), from a world-ENU attitude."""
    w, x, y, z = q_wxyz
    fx = 1 - 2 * (y * y + z * z)
    fy = 2 * (x * y + w * z)
    return np.array([fy, fx])


# --- reading a run ---
def read_truth(bag):
    import check_sim_sensors as cs
    _, _, tr = cs.read_bag(bag, [])
    return np.array(tr, dtype=float)        # t, x(E), y(N), z, qw, qx, qy, qz, |v|, gazebo stamp


def wind_response(truth, kind, t0, t1):
    """Mean lean (rotorcraft) or crab (plane) into the wind over [t0, t1] (steady flight):
    (deg, its direction as a (north, east) unit vector)."""
    tr = truth[(truth[:, 0] >= t0) & (truth[:, 0] <= t1)]
    if len(tr) < 50:
        return 0.0, np.zeros(2)
    tg = tr[:, 9]
    ok = np.concatenate([[True], np.diff(tg) > 0])
    tr, tg = tr[ok], tg[ok]
    # Smoothed over about 1 s (Gazebo's 20 Hz pose differentiates noisily).
    def smooth(x, n=21):
        return np.convolve(x, np.ones(n) / n, mode="same")
    vn, ve = np.gradient(smooth(tr[:, 2]), tg), np.gradient(smooth(tr[:, 1]), tg)
    an, ae = np.gradient(smooth(vn), tg), np.gradient(smooth(ve), tg)
    steady = np.hypot(an, ae) < 0.5                  # no turns, no speed changes
    steady[:15] = steady[-15:] = False               # the smoothing's edges
    if kind in ("fixedwing", "vtol"):        # a VTOL flies its route as a plane
        v = np.stack([vn, ve], 1)
        sp = np.linalg.norm(v, axis=1)
        h = np.array([heading_ne(q) for q in tr[:, 4:8]])
        h /= np.maximum(np.linalg.norm(h, axis=1, keepdims=True), 1e-9)
        use = steady & (sp > 5)
        if use.sum() < 20:
            return 0.0, np.zeros(2)
        # Crab: the nose's direction minus the track's, as a sideways vector (north, east).
        track = v[use] / sp[use, None]
        side = h[use] - track * np.sum(h[use] * track, axis=1, keepdims=True)
        m = side.mean(axis=0)
        return math.degrees(math.asin(min(1.0, float(np.linalg.norm(m))))), m / max(float(np.linalg.norm(m)), 1e-9)
    # The lean the vehicle's own acceleration explains (a/g) taken out: what's left is a
    # force such as wind (plus the airframe's own small offset).
    tilt = (np.array([thrust_tilt_ne(q) for q in tr[:, 4:8]]) - np.stack([an, ae], 1) / 9.81)[steady]
    if len(tilt) < 20:
        return 0.0, np.zeros(2)
    m = tilt.mean(axis=0)
    return math.degrees(math.asin(min(1.0, float(np.linalg.norm(m))))), m / max(float(np.linalg.norm(m)), 1e-9)


def fly_times(fly_log):
    """(start, end) unix times of the steady route flight, from fly.log: from the route's
    start (a VTOL: its front transition, so only its plane part) to the first of the
    route's end, the back transition, a GPS cut or PX4 leaving offboard. None if missing."""
    t_start = t_end = None
    vtol = False
    lines = Path(fly_log).read_text().splitlines()
    for line in lines:
        vtol = vtol or "front transition done" in line
    for line in lines:
        m = re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)
        if not m:
            continue
        t = float(m.group(1))
        if t_start is None and (("front transition done" in line) if vtol else
                                ("starting the route" in line or "climbed out" in line)):
            t_start = t
        if t_start is not None and t_end is None and any(k in line for k in (
                "route done", "back transition requested", "GPS cut", "left offboard")):
            t_end = t
    return t_start, t_end


def landed_time(fly_log):
    for line in Path(fly_log).read_text().splitlines() if Path(fly_log).exists() else []:
        m = re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)
        if m and "landed and disarmed" in line:
            return float(m.group(1))
    return None


def check_run(run, sc):
    import scenario as scn
    import sim_vehicles
    run = Path(run)
    ok = True
    info = dict(l.split("=", 1) for l in (run / "run_info.txt").read_text().splitlines() if "=" in l)
    world = info.get("site") or info.get("world")
    ok &= ck(info["vehicle"] == sc["vehicle"] and world == sc["world"], "the run is the scenario's vehicle and world",
             f"{info['vehicle']} over {world} (asked {sc['vehicle']} over {sc['world']})")
    kind = sim_vehicles.get(sc["vehicle"]).kind
    fly = (run / "fly.log").read_text() if (run / "fly.log").exists() else ""
    ok &= ck("landed and disarmed" in fly and "giving up" not in fly, "PX4 flew the route and landed",
             "landed" if "landed and disarmed" in fly else "no landing in fly.log")
    res = json.loads((run / "result.json").read_text()) if (run / "result.json").exists() else None
    if res is not None:
        ok &= ck(not res.get("timed_out"), "within the scenario's time limit",
                 f"{res.get('duration_s', 0):.0f} s of {sc['limits']['timeout_s']:g}")
    # Route.
    r = sc["route"]
    route = np.genfromtxt(run / "route.csv", delimiter=",", names=True)
    length = float(np.sum(np.hypot(np.diff(route["north_m"]), np.diff(route["east_m"]))))
    want = planned_length(r["legs"], r["leg_len_m"], r["spacing_m"])
    ok &= ck(abs(length - want) <= 0.01 * want, "the route's length is the scenario's legs, leg length and spacing",
             f"{length:.0f} m planned, {want:.0f} m asked")
    clear = float(np.min(-route["z_ned_m"] - route["ground_above_pad_m"]))
    ok &= ck(clear >= r["clearance_m"] - 1.0, f"the route never lower than {r['clearance_m']:g} m over the terrain",
             f"lowest {clear:.1f} m")
    # GNSS, from PX4's own estimate in GDN's run log.
    m = re.search(r"run log: (\S+)", (run / "gdn.log").read_text()) if (run / "gdn.log").exists() else None
    runlog = Path(m.group(1)) if m else None
    t_route0, t_route1 = fly_times(run / "fly.log") if (run / "fly.log").exists() else (None, None)
    if runlog is not None:
        import check_gps_cut as gc
        lp = gc.read_lpos(runlog)
        # The airborne part only, as tools/check_gps_cut.py (before take-off PX4's GPS is
        # still settling): from the route's start to the landing.
        t_land = landed_time(run / "fly.log")
        lost = [t for t, _z, _zv, xy_valid, eph in lp if ((not xy_valid) or eph > GNSS_EPH_LOST)
                and (t_route0 is None or t >= t_route0) and (t_land is None or t <= t_land)]
        if sc["gnss"]["loss"] == "cut":
            ask = (t_route0 or 0) + sc["gnss"]["at_s"]
            first = lost[0] if lost else None
            ok &= ck(first is not None and ask - 1.0 <= first <= ask + GNSS_LATE_S,
                     f"PX4 lost GPS as asked, {sc['gnss']['at_s']:g} s into the route",
                     "never" if first is None else f"at {first - (t_route0 or 0):.1f} s into the route")
        else:
            ok &= ck(not lost, "PX4 kept GPS throughout (no GNSS loss asked)",
                     f"{len(lost)} samples lost of {len(lp)}" if lost else f"{len(lp)} samples, eph under {GNSS_EPH_LOST:g} m")
    else:
        ok &= ck(False, "GDN's run log found (gdn.log 'run log:')")
    # Start error: GDN's position against the truth when its heading was aligned.
    truth = read_truth(run / "sitl_bag")
    gdn_log = (run / "gdn.log").read_text() if (run / "gdn.log").exists() else ""
    se = sc["start_error"]
    m0 = re.search(r"\[(\d+\.\d+)\] \[gdn_eskf_node\]: start error: ([-\d.]+) m north, ([-\d.]+) m east", gdn_log)
    m_head = re.search(r"\[(\d+\.\d+)\] \[gdn_eskf_node\]: heading from magnetometer", gdn_log)
    t_head = float((m0 or m_head).group(1)) if (m0 or m_head) else None
    import check_sim_sensors as cs
    odom = cs.read_bag(run / "sitl_bag", ["/gdn/odom"])[1]["/gdn/odom"] if t_head else []
    after = [(t, m) for t, m in odom if t >= t_head + 0.05][:1] if t_head else []
    if after and len(truth):
        t, mo = after[0]
        g = np.array([mo.pose.pose.position.x, mo.pose.pose.position.y])
        tn, te = np.interp(t, truth[:, 0], truth[:, 2]), np.interp(t, truth[:, 0], truth[:, 1])
        d = g - np.array([tn, te])
        dist, bear = float(np.hypot(*d)), bearing_deg(*d)
        if se["distance_m"] > 0:
            ok &= ck(abs(dist - se["distance_m"]) <= START_DIST_TOL and
                     abs(angle_diff(bear, se["bearing_deg"])) <= START_BEAR_TOL,
                     f"GDN started {se['distance_m']:g} m from the truth, bearing {se['bearing_deg']:g} deg",
                     f"{dist:.2f} m at {bear:.1f} deg")
        else:
            ok &= ck(dist <= START_NONE_TOL, "GDN started at the truth (no start error asked)", f"{dist:.2f} m off")
    else:
        ok &= ck(False, "GDN's heading alignment and output found", "no alignment line in gdn.log or no /gdn/odom")
    # Wind.
    wi = sc["wind"]
    wsdf = next(iter(run.glob("*.sdf")), None)
    world_wind = re.search(r"<wind>\s*<linear_velocity>([^<]+)</linear_velocity>", wsdf.read_text()) if wsdf else None
    sc_cfg = (run / "server.config").read_text() if (run / "server.config").exists() else ""
    have_window = bool(t_route0 and t_route1 and len(truth))
    deg, direction = wind_response(truth, kind, t_route0, t_route1) if have_window else (0.0, np.zeros(2))
    group = "plane" if kind in ("fixedwing", "vtol") else "multirotor"
    what = "crab (nose upwind of the track)" if group == "plane" else "lean (thrust axis upwind)"
    if wi["speed_mps"] > 0:
        v = [float(x) for x in world_wind.group(1).split()] if world_wind else [0, 0, 0]
        b = math.radians(wi["from_deg"] + 180.0)            # where it blows to
        ok &= ck(world_wind is not None and abs(v[0] - wi["speed_mps"] * math.sin(b)) < 1e-3 and
                 abs(v[1] - wi["speed_mps"] * math.cos(b)) < 1e-3 and
                 (kind != "multirotor" or "WindEffects" in sc_cfg),
                 f"the world carries {wi['speed_mps']:g} m/s from {wi['from_deg']:g} deg" +
                 (", with Gazebo's wind effects (a multirotor's drag)" if kind == "multirotor" else
                  " (the wings' lift-drag model reads it)"),
                 f"world wind {world_wind.group(1) if world_wind else 'none'} (ENU)")
        off = math.degrees(math.acos(max(-1.0, min(1.0, float(direction @ upwind_unit(wi["from_deg"]))))))
        along = deg * float(direction @ upwind_unit(wi["from_deg"]))
        ok &= ck(have_window and along >= WIND_MIN_DEG[group] and off <= WIND_DIR_TOL,
                 f"the vehicle {what} into the wind on the route (at least {WIND_MIN_DEG[group]:g} deg upwind)",
                 f"{deg:.2f} deg, {along:.2f} deg upwind, {off:.0f} deg from upwind" if have_window
                 else "no steady route flight in fly.log")
    else:
        ok &= ck(world_wind is None or all(abs(float(x)) < 1e-9 for x in world_wind.group(1).split()),
                 "no wind in the world (none asked)")
        whole = have_window and "route done" in fly
        if whole:
            ok &= ck(deg <= CALM_MAX_DEG[group], f"no steady {what.split(' (')[0]} on the route (no wind; at most "
                     f"{CALM_MAX_DEG[group]:g} deg)", f"{deg:.2f} deg")
        else:
            note(f"no-wind {what.split(' (')[0]} not judged: the route wasn't flown whole (legs that cancel)")
    # Recording and teardown, from the runner.
    if res is not None:
        ok &= ck(res.get("bag_ok") and res.get("run_log_ok"), "the bag and GDN's run log were written",
                 f"bag {res.get('bag_mb', 0):.0f} MB, run log {res.get('run_log', '?')}")
        ok &= ck(res.get("replay") == "IDENTICAL", "GDN's run log replays bit for bit", str(res.get("replay")))
        ok &= ck(res.get("teardown_clean"), "nothing of the simulation left running after the run",
                 ", ".join(res.get("left_running", [])) or "none")
    else:
        note("no result.json (not a tools/run_scenario.py run): recording, replay and teardown not checked")
    print("SCENARIO CHECK PASSED" if ok else "SCENARIO CHECK FAILED")
    return ok


def selfcheck():
    ok = True
    # 2 legs of 800 m, 250 m apart: pad -> (-400, -125) -> (400, -125) -> (400, 125) -> (-400, 125) -> pad.
    want = 2 * math.hypot(400, 125) + 800 + 250 + 800
    ok &= ck(abs(planned_length(2, 800, 250) - want) < 1e-6, "selfcheck: a lawnmower's planned length",
             f"{planned_length(2, 800, 250):.1f} m")
    ok &= ck(abs(bearing_deg(1, 0)) < 1e-9 and abs(bearing_deg(0, 1) - 90) < 1e-9 and abs(bearing_deg(-1, -1) - 225) < 1e-9,
             "selfcheck: bearings clockwise from north")
    ok &= ck(abs(angle_diff(359, 1) + 2) < 1e-9 and abs(angle_diff(10, 350) - 20) < 1e-9, "selfcheck: angle differences")
    ok &= ck(np.allclose(upwind_unit(270), [0, -1]), "selfcheck: wind from 270 comes from the west")
    # A body pitched 5 deg nose-down while facing north: Gazebo ENU, yaw 90 deg (facing north).
    def q_from(yaw, pitch, roll):
        cy, sy, cp, sp, cr, sr = (math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                                  math.cos(roll / 2), math.sin(roll / 2))
        return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy)
    t = thrust_tilt_ne(q_from(math.radians(90), math.radians(5), 0.0))
    ok &= ck(abs(math.degrees(math.asin(np.linalg.norm(t))) - 5) < 1e-6 and t[0] > 0.08,
             "selfcheck: a north-facing body pitched 5 deg nose down leans its thrust 5 deg north", str(np.round(t, 4)))
    h = heading_ne(q_from(math.radians(90), 0.0, 0.0))
    ok &= ck(np.allclose(h, [1, 0], atol=1e-9), "selfcheck: yaw 90 deg in ENU faces north")
    # Lean into a west wind: a run whose thrust axis leans 2 deg west while flying north and south.
    tt = np.arange(0, 100, 0.05)
    rows = []
    for k, t_ in enumerate(tt):
        north = 10 * t_ if t_ < 50 else 500 - 10 * (t_ - 50)
        q = q_from(math.radians(90), 0.0, math.radians(-2))   # roll left (west) facing north
        rows.append((t_, 0.0, north, 50.0, *q, 10.0, t_))
    deg, d = wind_response(np.array(rows), "multirotor", 1, 99)
    ok &= ck(abs(deg - 2) < 0.05 and d[1] < -0.99, "selfcheck: a 2 deg lean west is found, pointing west",
             f"{deg:.2f} deg, ({d[0]:.2f}, {d[1]:.2f})")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", help="a tools/run_scenario.py run directory (or a sim_run.sh log directory)")
    ap.add_argument("--scenario", help="the scenario file (default RUN/scenario.yaml)")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return selfcheck()
    if not a.run:
        ap.error("give --run or --selfcheck")
    import scenario as scn
    sc = scn.load(a.scenario) if a.scenario else scn.parse(__import__("yaml").safe_load(
        (Path(a.run) / "scenario.yaml").read_text()), str(Path(a.run) / "scenario.yaml"))
    return check_run(a.run, sc)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
