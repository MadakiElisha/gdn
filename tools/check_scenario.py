#!/usr/bin/env python3
"""Did a SITL run do what its scenario asked? (roadmap R2.6, TEST-040 and TEST-041)

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
    with "spoof" (R2.6b), PX4's position followed the truth until the spoof, then walked off
    from it at the asked rate and bearing (PX4's estimate minus the truth, less the frames'
    constant offset found before the spoof: within 5 m or 10% at the landing, 10 deg);
  - faults (R2.6b): each fault in GDN's own record of its inputs (the run log), switched
    on when asked (the adapter's /gdn/sim/fault, within 1 s): a dropout's window empty, a
    stuck value constant and equal to the last one before, a noise burst at least 0.6 of
    its sigma and twice the sensor's own; an IMU relay passed PX4's IMU on whole outside
    its faults; without faults, none switched on;
  - lighting (R2.6b): the run's world has the sun asked (or the world's own), and the
    light the OAK-D Lite sees (Gazebo's raw linear image, before its auto-exposure)
    answers it: darker by at least half the asked change against the x500's over
    Emmental in the world's own light (TEST-041);
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
# R2.6b. Spoofing: PX4's estimate minus the truth within SPOOF_PRE_MAX before the spoof;
# at the landing within max(SPOOF_TOL_M, SPOOF_TOL_FRAC x asked) of the asked offset and
# SPOOF_BEAR_TOL of its bearing. Faults: FAULT_ON_TOL between asked and switched on.
SPOOF_PRE_MAX, SPOOF_TOL_M, SPOOF_TOL_FRAC, SPOOF_BEAR_TOL = 3.0, 5.0, 0.10, 10.0
FAULT_ON_TOL = 1.0
FAULT_TOPICS = {"agl": "/gdn/agl_meas", "baro": "/gdn/baro_meas", "mag": "/gdn/mag_meas",
                "imu": "/fmu/out/sensor_combined"}
# Lighting: the light the OAK-D Lite's left camera sees (/gdn/sim/scene_light, its raw
# linear image's mean, fraction of full scale) on the x500 over Emmental in the world's
# own light, on x500_emmental_dusk's route (TEST-041's first flights: 0.2025; dusk at 0.3
# of the light saw 0.0139). A run asking intensity k must be at most ref x (1 - 0.5 (1 - k))
# (darker), or at least that much brighter for k > 1. The image the camera sends is held
# near mid-grey by its auto-exposure (sim_sensors), so it isn't the measure.
LIGHT_REF = {("x500", "ch_emmental"): 0.2025}


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


def noise_sigma(v):
    """A robust white-noise sigma of a series (columns: axes): from the first differences'
    median absolute deviation, so slow changes (the flight itself) hardly count."""
    v = np.asarray(v, dtype=float).reshape(len(v), -1)
    if len(v) < 5:
        return float("nan")
    d = np.diff(v, axis=0)
    mad = np.median(np.abs(d - np.median(d, axis=0)), axis=0)
    return float(np.mean(1.4826 * mad / math.sqrt(2.0)))


def judge_fault(kind, t, v, on, off, sigma=0.0):
    """(ok, detail) for one fault in a series t (s), v (rows), switched on at on, off at off."""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float).reshape(len(t), -1)
    period = float(np.median(np.diff(t))) if len(t) > 2 else 1.0
    m = max(0.3, 1.5 * period)
    inside = (t > on + m) & (t < off - m)
    before = (t >= on - 20.0) & (t < on)
    after = (t > off + m) & (t <= off + 20.0)
    if kind == "dropout":
        gap = t[(t >= on - 5 * period - 1) & (t <= off + 5 * period + 1)]
        longest = float(np.max(np.diff(gap))) if len(gap) > 1 else 0.0
        ok = inside.sum() == 0 and before.sum() > 0 and after.sum() > 0
        return ok, (f"no sample for {longest:.1f} s (asked {off - on:g} s); "
                    f"{before.sum()} before, {after.sum()} after")
    if inside.sum() < 2 or before.sum() < 3:
        return False, f"{inside.sum()} samples in the fault, {before.sum()} before it"
    if kind == "stuck":
        w = v[inside]
        last = v[before][-1]
        const = bool(np.all(w == w[0])) and bool(np.all(w[0] == last))
        varies = bool(np.any(np.ptp(v[before], axis=0) > 0)) and (after.sum() < 2 or
                                                                   bool(np.any(np.ptp(v[after], axis=0) > 0)))
        return const and varies, (f"{inside.sum()} samples all {np.round(w[0], 4).tolist()}, the last before it"
                                  if const else f"{len(np.unique(w, axis=0))} different values in the fault") + \
            ("" if varies else "; constant outside it too")
    # noise
    s_in = noise_sigma(v[inside])
    s_out = noise_sigma(np.concatenate([v[before], v[after]])) if after.sum() > 2 else noise_sigma(v[before])
    ok = s_in >= 0.6 * sigma and s_in >= 2.0 * s_out
    return ok, f"noise {s_in:.3g} in the fault (asked {sigma:g}), {s_out:.3g} outside"


def spoof_offsets(t_l, xy_l, truth, t_pre0, t_pre1):
    """PX4's position minus the truth (north, east) at PX4's sample times, less the frames'
    constant offset (its median over [t_pre0, t_pre1], before the spoof)."""
    tn = np.interp(t_l, truth[:, 0], truth[:, 2])
    te = np.interp(t_l, truth[:, 0], truth[:, 1])
    d = np.asarray(xy_l, dtype=float) - np.stack([tn, te], 1)
    pre = (t_l >= t_pre0) & (t_l <= t_pre1)
    if pre.sum() < 10:
        return None
    return d - np.median(d[pre], axis=0)


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


def route_start_time(run):
    """The unix time the route started, as the spoofer and the faults count it: sim_fly.py's
    /gdn/sim/route_start in the bag, else its "starting the route" (or "climbed out") line."""
    import check_sim_sensors as cs
    try:
        msgs = cs.read_bag(Path(run) / "sitl_bag", ["/gdn/sim/route_start"])[1]["/gdn/sim/route_start"]
        if msgs:
            return float(msgs[0][1].data)
    except Exception:  # noqa: BLE001 (a bag without the topic, or none)
        pass
    for line in (Path(run) / "fly.log").read_text().splitlines() if (Path(run) / "fly.log").exists() else []:
        m = re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)
        if m and ("starting the route" in line or "climbed out" in line):
            return float(m.group(1))
    return None


def touchdown_time(run_log, t_from):
    """When PX4's land detector first felt the ground after t_from (ground contact, maybe
    landed or landed; the run log's record time), or None. Touching down jolts PX4's
    estimator (eph briefly over 10 m on the x500, TEST-042), so airborne windows end here."""
    import check_sim_sensors as cs
    msgs = cs.read_bag(run_log, ["/fmu/out/vehicle_land_detected"])[1]["/fmu/out/vehicle_land_detected"]
    return next((t for t, m in msgs if t > t_from and (m.ground_contact or m.maybe_landed or m.landed)), None)


def landed_time(fly_log):
    for line in Path(fly_log).read_text().splitlines() if Path(fly_log).exists() else []:
        m = re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)
        if m and "landed and disarmed" in line:
            return float(m.group(1))
    return None


def read_lpos_xy(run_log):
    """PX4's position, timed by when GDN received it (the run log's record time, the host's
    clock, as the truth's bag times), not by PX4's own stamp: in SITL that's Gazebo's
    time, which falls behind the host's when the simulation runs slower than real time
    (at 10 m/s a 0.3 s lag is 3 m; TEST-042)."""
    import check_sim_sensors as cs
    lp = cs.read_bag(run_log, ["/fmu/out/vehicle_local_position"])[1]["/fmu/out/vehicle_local_position"]
    return (np.array([t for t, _ in lp]), np.array([(m.x, m.y) for _, m in lp]),
            np.array([bool(m.xy_valid) for _, m in lp]), np.array([float(m.eph) for _, m in lp]))


def fault_values(topic, msgs):
    if topic == "/fmu/out/sensor_combined":
        return np.array([list(m.accelerometer_m_s2) for _, m in msgs])
    if topic == "/gdn/mag_meas":
        return np.array([(m.x, m.y, m.z) for _, m in msgs])
    return np.array([m.data for _, m in msgs])


def check_spoof(sc, runlog, truth, t_route0, t_land):
    g = sc["gnss"]
    ok = True
    t_l, xy, valid, eph = read_lpos_xy(runlog)
    if not len(t_l) or not len(truth) or t_route0 is None:
        return ck(False, "PX4's position and the truth found for the spoof")
    t_on = t_route0 + g["at_s"]
    t_end = t_land if t_land is not None else t_l[-1]
    off = spoof_offsets(t_l, xy, truth, t_route0 + 2.0, t_on - 1.0)
    if off is None:
        return ck(False, "PX4's position before the spoof (to align its frame with the truth's)",
                  f"spoof {g['at_s']:g} s into the route: too early")
    air = (t_l >= t_route0) & (t_l <= t_end)
    pre = air & (t_l < t_on)
    dist = np.hypot(off[:, 0], off[:, 1])
    ok &= ck(pre.sum() > 0 and dist[pre].max() <= SPOOF_PRE_MAX, "PX4's position followed the truth before the spoof",
             f"within {dist[pre].max():.2f} m over {t_on - t_route0:.0f} s" if pre.sum() else "no samples")
    import sim_spoof
    at_end = air & (t_l >= t_end - 10.0)
    if at_end.sum() == 0:
        return ck(False, "PX4's position at the landing found")
    meas = off[at_end].mean(axis=0)
    want = np.array(sim_spoof.offset_ne(float(np.mean(t_l[at_end])) - t_route0, g["at_s"], g["rate_mps"],
                                        g["bearing_deg"]))
    err = float(np.hypot(*(meas - want)))
    bear = bearing_deg(*meas)
    ok &= ck(err <= max(SPOOF_TOL_M, SPOOF_TOL_FRAC * np.hypot(*want)) and
             abs(angle_diff(bear, g["bearing_deg"])) <= SPOOF_BEAR_TOL,
             f"PX4's position walked off at {g['rate_mps']:g} m/s towards {g['bearing_deg']:g} deg from "
             f"{g['at_s']:g} s into the route",
             f"{np.hypot(*meas):.1f} m at {bear:.1f} deg before the landing (asked {np.hypot(*want):.1f} m), "
             f"{err:.1f} m apart")
    walk = air & (t_l >= t_on) & (dist > SPOOF_PRE_MAX)
    if walk.any():
        note(f"PX4's position was {SPOOF_PRE_MAX:g} m off the truth {t_l[walk][0] - t_route0:.1f} s into the route "
             f"(spoof asked at {g['at_s']:g} s, {SPOOF_PRE_MAX / g['rate_mps']:.0f} s to walk that far)")
    lost = air & ((~valid) | (eph > GNSS_EPH_LOST))
    note("PX4 never noticed: its position valid and eph under {:g} m throughout".format(GNSS_EPH_LOST) if not lost.any()
         else f"PX4 flagged its position {lost.sum()} times, first {t_l[lost][0] - t_route0:.0f} s into the route")
    return ok


def check_faults(sc, run, runlog, t_route0):
    import check_sim_sensors as cs
    ok = True
    ev = cs.read_bag(run / "sitl_bag", ["/gdn/sim/fault"])[1]["/gdn/sim/fault"]
    events = [(t, m.data) for t, m in ev]
    if not sc["faults"]:
        return ck(not events, "no sensor fault switched on (none asked)", f"{len(events)} fault events" if events else "")
    if t_route0 is None:
        return ck(False, "the route's start found (fault times count from it)")
    topics = sorted({FAULT_TOPICS[f["sensor"]] for f in sc["faults"]})
    series = cs.read_bag(runlog, topics)[1]
    for f in sc["faults"]:
        import sim_faults
        fl = sim_faults.Fault(**f)
        on_ev = next((t for t, d in events if d == f"on {fl.text()}"), None)
        off_ev = next((t for t, d in events if d == f"off {fl.text()}"), None)
        ask = t_route0 + f["at_s"]
        good = on_ev is not None and abs(on_ev - ask) <= FAULT_ON_TOL
        ok &= ck(good, f"{fl.text()}: switched on as asked",
                 "never" if on_ev is None else f"{on_ev - t_route0:.2f} s into the route")
        if not good:
            continue
        msgs = series[FAULT_TOPICS[f["sensor"]]]
        t = np.array([x for x, _ in msgs])
        v = fault_values(FAULT_TOPICS[f["sensor"]], msgs)
        good, detail = judge_fault(f["kind"], t, v, on_ev, off_ev if off_ev is not None else on_ev + f["duration_s"],
                                   f["sigma"])
        ok &= ck(good, f"{fl.text()}: in GDN's inputs (run log)", detail)
    if any(f["sensor"] == "imu" for f in sc["faults"]):
        raw = cs.read_bag(run / "sitl_bag", ["/fmu/out/sensor_combined"])[1]["/fmu/out/sensor_combined"]
        t_raw = np.array([x for x, _ in raw])
        t_gdn = np.array([x for x, _ in series["/fmu/out/sensor_combined"]])
        lo, hi = max(t_raw[0], t_gdn[0]) + 5, min(t_raw[-1], t_gdn[-1]) - 5
        out = np.ones(len(t_raw), bool)
        for f in sc["faults"]:
            if f["sensor"] == "imu" and f["kind"] == "dropout":
                a = t_route0 + f["at_s"]
                out &= ~((t_raw >= a - 1) & (t_raw <= a + f["duration_s"] + 1))
        n_raw = int(((t_raw >= lo) & (t_raw <= hi) & out).sum())
        out_g = np.ones(len(t_gdn), bool)
        for f in sc["faults"]:
            if f["sensor"] == "imu" and f["kind"] == "dropout":
                a = t_route0 + f["at_s"]
                out_g &= ~((t_gdn >= a - 1) & (t_gdn <= a + f["duration_s"] + 1))
        n_gdn = int(((t_gdn >= lo) & (t_gdn <= hi) & out_g).sum())
        ok &= ck(n_raw > 0 and n_gdn >= 0.98 * n_raw, "the IMU relay passed PX4's IMU on to GDN outside its faults",
                 f"{n_gdn} of {n_raw} samples ({100.0 * n_gdn / max(n_raw, 1):.1f}%)")
    return ok


def check_lighting(sc, run):
    import check_sim_sensors as cs
    import sim_light
    ok = True
    li = sc["lighting"]
    wsdf = next((p for p in run.glob("*.sdf")), None)
    src = ROOT / "data/sim" / sc["world"] / f"{sc['world']}.sdf"
    have, base = (sim_light.read_lighting(wsdf) if wsdf else None), (sim_light.read_lighting(src) if src.exists() else None)
    if have is None or base is None:
        return ck(False, "the run's world and its sun found", f"{wsdf} / {src}")
    el, az, dif, _amb = have
    if li:
        k = li["intensity"]
        ok &= ck(abs(el - li["sun_elevation_deg"]) < 0.1 and abs(angle_diff(az, li["sun_azimuth_deg"])) < 0.1 and
                 np.allclose(dif, np.minimum(1.0, np.array(base[2]) * k), atol=1e-3),
                 f"the world's sun {li['sun_elevation_deg']:g} deg up at {li['sun_azimuth_deg']:g} deg, x{k:g} light",
                 f"{el:.1f} deg up at {az:.1f} deg, diffuse {dif[0]:.3f} (the world's {base[2][0]:g})")
    else:
        ok &= ck(abs(el - base[0]) < 0.1 and np.allclose(dif, base[2]), "the world's own sun (no lighting asked)",
                 f"{el:.1f} deg up at {az:.1f} deg")
    got = cs.read_bag(run / "sitl_bag", ["/gdn/sim/scene_light", "/gdn/sim/image_brightness"])[1]
    sl, br = got["/gdn/sim/scene_light"], got["/gdn/sim/image_brightness"]
    if not sl:
        note("no scene light recorded (no cameras: GDN_SIM_CPU?, or a run from before R2.6b)")
        return ok if not li else ck(False, "the light the OAK-D Lite sees recorded")
    b = float(np.mean([m.data for _, m in sl]))
    img = f"; the image it sends {np.mean([m.data for _, m in br]):.0f} of 255 after its auto-exposure" if br else ""
    ref = LIGHT_REF.get((sc["vehicle"], sc["world"]))
    if li and ref:
        k = li["intensity"]
        bound = ref * (1.0 - 0.5 * (1.0 - k))
        ok &= ck(b <= bound if k < 1 else b >= bound,
                 f"the OAK-D Lite saw the light asked ({'darker' if k < 1 else 'brighter'} than {bound:.4f})",
                 f"scene light {b:.4f} against {ref:g} in the world's own light{img}")
    else:
        note(f"OAK-D Lite scene light {b:.4f} of full scale over {len(sl)} s{img}" +
             ("" if not li else " (no reference for this vehicle and world)"))
    return ok


def check_run(run, sc):
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
        # still settling): from the route's start to touchdown (the land detector; landing
        # jolts PX4's estimator), else to "landed and disarmed".
        t_land = landed_time(run / "fly.log")
        t_touch = touchdown_time(runlog, t_route0) if t_route0 else None
        t_end = t_touch or t_land
        lost = [t for t, _z, _zv, xy_valid, eph in lp if ((not xy_valid) or eph > GNSS_EPH_LOST)
                and (t_route0 is None or t >= t_route0) and (t_end is None or t <= t_end)]
        if sc["gnss"]["loss"] == "spoof":
            ok &= check_spoof(sc, runlog, read_truth(run / "sitl_bag"), route_start_time(run), t_land)
        elif sc["gnss"]["loss"] == "cut":
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
    # Faults and lighting (R2.6b).
    if runlog is not None:
        ok &= check_faults(sc, run, runlog, route_start_time(run))
    ok &= check_lighting(sc, run)
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
    # R2.6b: faults judged on made-up series (a 50 Hz sensor that changes slowly, sd 0.1).
    rng = np.random.default_rng(3)
    tt = np.arange(0.0, 100.0, 0.02)
    base = np.sin(tt / 10.0) + rng.normal(0, 0.1, len(tt))
    keep = ~((tt >= 40) & (tt < 60))
    good, d = judge_fault("dropout", tt[keep], base[keep], 40.0, 60.0)
    ok &= ck(good, "selfcheck: a 20 s dropout is found", d)
    ok &= ck(not judge_fault("dropout", tt, base, 40.0, 60.0)[0], "selfcheck: no dropout where none was")
    st = base.copy()
    st[(tt >= 40) & (tt < 60)] = base[np.searchsorted(tt, 40.0) - 1]
    good, d = judge_fault("stuck", tt, st, 40.0, 60.0)
    ok &= ck(good, "selfcheck: a stuck value is found", d)
    ok &= ck(not judge_fault("stuck", tt, base, 40.0, 60.0)[0], "selfcheck: no stuck value where none was")
    nz = base.copy()
    w = (tt >= 40) & (tt < 60)
    nz[w] += rng.normal(0, 2.0, w.sum())
    good, d = judge_fault("noise", tt, nz, 40.0, 60.0, 2.0)
    ok &= ck(good, "selfcheck: a noise burst of sigma 2 is found", d)
    ok &= ck(not judge_fault("noise", tt, base, 40.0, 60.0, 2.0)[0], "selfcheck: no noise burst where none was")
    # The spoof: PX4's frame 30 m north of the truth's, walking off east at 1 m/s from t = 60.
    tl = np.arange(0.0, 200.0, 0.1)
    tr = np.column_stack([tl, 5 * np.sin(tl / 20), 2 * tl, np.zeros((len(tl), 7))])   # t, E, N, ...
    xy = np.stack([tr[:, 2] + 30.0, tr[:, 1] + np.where(tl > 60, tl - 60, 0.0)], 1)
    off = spoof_offsets(tl, xy, tr, 2.0, 59.0)
    ok &= ck(off is not None and np.allclose(off[-1], [0.0, 139.9], atol=1e-6) and np.abs(off[tl < 60]).max() < 1e-9,
             "selfcheck: a spoof's offset found against the truth, the frames' offset taken out",
             str(np.round(off[-1], 2)) if off is not None else "none")
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
