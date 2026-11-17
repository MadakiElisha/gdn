#!/usr/bin/env python3
"""Check the simulated vehicles (roadmap R2.3, TEST-034).

    python3 tools/check_vehicles.py                    # the models (no simulator needed)
    python3 tools/check_vehicles.py --run LOG_DIR      # judge a tools/sim_run.sh flight
    python3 tools/check_vehicles.py --selfcheck        # the judging, on made-up tracks

The models, for every vehicle in tools/sim_vehicles.py:
  - sim/models/<model>/model.sdf merges PX4's own model, includes model://gdn_payload
    exactly once and fixes it to base_link, and publishes truth on gdn/truth_odom;
  - no vehicle carries GDN sensors of its own: they all come from the payload;
  - PX4's airframe file exists and simulates the same PX4 model (if PX4 is installed:
    $PX4_DIR, default ~/PX4-Autopilot-1.16, or /opt/PX4-Autopilot in the dev image);
  - with Gazebo installed, `gz sdf -p` resolves each model (payload included).

A flight (sim_run.sh's log directory, holding run_info.txt, route.csv, fly.log,
adapter.log and sitl_bag), from the true track in the bag:
  - it went through its phases: take-off (the cessna's from the runway, or its catapult
    launch); the VTOL's two transitions; landed and disarmed (the cessna on the runway,
    by its landing mission; in a world without a runway it circles over the pad
    instead); no fail-safe;
  - it came down where it should: the x500 and the VTOL within 10 m of the pad, the
    cessna on the runway (within its length, and 5 m of its edges);
  - it flew the whole route: every route point had the vehicle within --reach m;
  - it followed it: cross-track error, 95th percentile, under --xtrack m (hover) or
    --xtrack-fw m (as a plane), judged while sim_fly.py steers it (PX4 in offboard, not
    during the VTOL's transitions or the cessna's last circles, which PX4 flies in Hold);
  - it kept clear of the ground: true height above the world's terrain at least
    --clearance m while following the route more than 150 m from the pad, and at least
    --clearance-other m the rest of the time (transitions, holds, the landing approach)
    except within 150 m of the pad or 200 m of the runway, where it comes down;
  - the radar altimeter saw the ground, not the airframe: each range (/gdn/sim/agl_scan,
    2 Hz of its 20) against the range predicted from the truth at the same simulated
    instant (both carry Gazebo's time; the pose is interpolated between truth samples),
    by casting the beam (body down at the true attitude) onto the world's terrain: RMS
    under --radar-rms m, over the readings GDN uses (tilt up to sim_adapter.py's 35
    degrees; beyond that the beam meets slopes at a grazing angle, where a tenth of a
    degree of attitude is metres of range). A blocked beam would read the airframe at 0.2 m, a wrong mount
    nothing. It's cast along the beam, not straight down: a banked plane points the
    beam off nadir, onto higher or lower ground on a slope (OI-016). Bags recorded
    before the scan was (no /gdn/sim/agl_scan) fall back to GDN's AGL input,
    /gdn/agl_meas, which has no time stamp of its own: paired by arrival time, a
    tilting x500 then shows a metre or so of timing error;
  - GDN ran: the scorer reported its error.

Exit 0 if everything passes, 1 if anything fails, 2 if something can't be read.
"""
import argparse
import math
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sim_vehicles  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
PAD_ZONE_M = 150.0
MAX_TILT_DEG = 35.0     # sim_adapter.py's max_tilt_deg: GDN gets no AGL beyond it


def ck(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
    return bool(ok)


def px4_dir():
    for p in (os.environ.get("PX4_DIR"), str(Path.home() / "PX4-Autopilot-1.16"), "/opt/PX4-Autopilot"):
        if p and (Path(p) / "ROMFS/px4fmu_common/init.d-posix/airframes").is_dir():
            return Path(p)
    return None


# --- the models ---

def check_models():
    ok = True
    payload = (ROOT / "sim/models/gdn_payload/model.sdf").read_text()
    ok &= ck('<topic>gdn/agl_radar</topic>' in payload and 'type="gpu_lidar"' in payload,
             "gdn_payload has the radar altimeter (gpu_lidar on gdn/agl_radar)")
    px4 = px4_dir()
    gz = shutil.which("gz")
    for name, v in sim_vehicles.VEHICLES.items():
        f = ROOT / "sim/models" / v.model / "model.sdf"
        if not f.exists():
            ok &= ck(False, f"{name}: {f.relative_to(ROOT)} exists")
            continue
        sdf = f.read_text()
        sdf_nc = re.sub(r"<!--.*?-->", "", sdf, flags=re.S)
        base = re.search(r'<include merge="true">\s*<uri>model://(\w+)</uri>', sdf_nc)
        n_payload = len(re.findall(r"<uri>model://gdn_payload</uri>", sdf_nc))
        joint = re.search(r'<joint name="\w+" type="fixed">\s*<parent>base_link</parent>\s*'
                          r'<child>gdn_payload::gdn_payload_link</child>', sdf_nc)
        own = re.findall(r"<topic>gdn/[^<]*</topic>", sdf_nc)
        ok &= ck(base is not None, f"{name}: merges a PX4 model", base.group(1) if base else "none")
        ok &= ck(n_payload == 1 and joint is not None, f"{name}: includes gdn_payload once, fixed to base_link",
                 f"{n_payload} include(s), joint {'found' if joint else 'missing'}")
        ok &= ck(not own, f"{name}: no GDN sensors outside the payload", ", ".join(own) if own else "")
        ok &= ck("<odom_topic>gdn/truth_odom</odom_topic>" in sdf_nc, f"{name}: truth on gdn/truth_odom")
        if px4 is not None:
            af = px4 / "ROMFS/px4fmu_common/init.d-posix/airframes" / v.airframe
            m = re.search(r"PX4_SIM_MODEL:=(\w+)", af.read_text()) if af.exists() else None
            ok &= ck(m is not None and base is not None and m.group(1) == base.group(1),
                     f"{name}: PX4 airframe {v.airframe} simulates the same model",
                     f"{m.group(1) if m else 'airframe missing'} vs {base.group(1) if base else '?'}")
        if gz and px4 is not None:
            paths = f"{ROOT / 'sim/models'}:{px4 / 'Tools/simulation/gz/models'}"
            env = dict(os.environ, GZ_SIM_RESOURCE_PATH=paths, SDF_PATH=paths)
            r = subprocess.run(["gz", "sdf", "-p", str(f)], capture_output=True, text=True, env=env, timeout=60)
            text = r.stdout
            ok &= ck(r.returncode == 0 and "gdn_payload_link" in text and "gdn_agl_radar" in text
                     and "magnetometer_sensor" in text and "air_pressure_sensor" in text,
                     f"{name}: gz sdf resolves it with the payload, magnetometer and barometer",
                     (r.stderr.strip().splitlines() or [""])[0][:160])
    if px4 is None:
        print("note  PX4 not installed here: airframe checks skipped")
    if not gz:
        print("note  Gazebo not installed here: gz sdf checks skipped")
    return ok


# --- a flight ---

def cross_track(route_ne, pts):
    """Horizontal distance from each point (N, 2) to the route polyline (M, 2)."""
    a, b = route_ne[:-1], route_ne[1:]
    ab = b - a
    L2 = np.maximum((ab ** 2).sum(1), 1e-9)
    best = np.full(len(pts), np.inf)
    for i in range(0, len(pts), 2000):
        p = pts[i:i + 2000, None, :]
        t = np.clip(((p - a) * ab).sum(2) / L2, 0.0, 1.0)
        d = np.linalg.norm(p - (a + t[..., None] * ab), axis=2)
        best[i:i + 2000] = d.min(1)
    return best


def judge_track(route, truth_ne, truth_h_agl, kind, reach, xtrack, xtrack_fw, clearance, fw_mask=None,
                follow_mask=None, clearance_other=15.0, landing_zone=None):
    """route (M, 2) north/east; truth_ne (N, 2); truth_h_agl (N,) height above the
    world's terrain. fw_mask: samples flown as a plane; follow_mask: samples where
    the route is being followed (default all). Returns [(ok, what, detail)]."""
    out = []
    far = np.linalg.norm(truth_ne, axis=1) > PAD_ZONE_M
    # Reach: each route point away from the pad has the vehicle within `reach` metres.
    rp = route[np.linalg.norm(route, axis=1) > PAD_ZONE_M]
    near = np.array([np.min(np.linalg.norm(truth_ne - p, axis=1)) for p in rp[::5]]) if len(rp) else np.array([0.0])
    out.append((bool(np.all(near <= reach)), f"whole route flown (every route point within {reach:.0f} m)",
                f"worst {near.max():.0f} m"))
    xt = cross_track(route, truth_ne)
    planes = fw_mask if fw_mask is not None else np.full(len(xt), kind == "fixedwing")
    follow = follow_mask if follow_mask is not None else np.full(len(xt), True)
    for label, sel, lim in (("hovering", far & follow & ~planes, xtrack), ("as a plane", far & follow & planes, xtrack_fw)):
        if sel.sum() < 20:
            continue
        p95 = float(np.percentile(xt[sel], 95))
        out.append((p95 <= lim, f"follows the route {label} (cross-track p95 under {lim:.0f} m)",
                    f"p95 {p95:.1f} m, max {xt[sel].max():.1f} m over {sel.sum()} samples"))
    h = truth_h_agl[far & follow]
    hmin = float(h.min()) if len(h) else float("nan")
    out.append((len(h) > 0 and hmin >= clearance,
                f"clear of the terrain on the route (at least {clearance:.0f} m away from the pad)",
                f"lowest {hmin:.1f} m above the ground"))
    rest = far & ~follow & (~landing_zone if landing_zone is not None else True)
    if rest.sum():
        h2 = float(truth_h_agl[rest].min())
        out.append((h2 >= clearance_other, f"clear of the terrain the rest of the time (at least {clearance_other:.0f} m, "
                    "away from the pad and the runway)", f"lowest {h2:.1f} m over {rest.sum()} samples"))
    return out


def judge_log(kind, fly, launch="runway", runway=True):
    """Phases from fly.log text. Returns [(ok, what, detail)]."""
    fw = (["catapult: released"] if launch == "catapult" else ["runway take-off"]) + ["climbed out"]
    fw += (["route done, landing on the runway", "landed and disarmed"] if runway
           else ["route done, circling", "circled over the pad"])
    want = {"multirotor": ["at cruise height", "route done, landing", "landed and disarmed"],
            "vtol": ["at cruise height", "front transition done", "back transition done",
                     "over the pad, landing", "landed and disarmed"],
            "fixedwing": fw}[kind]
    missing = [w for w in want if w not in fly]
    out = [(not missing, "went through its phases (" + "; ".join(want) + ")",
            "missing: " + "; ".join(missing) if missing else "")]
    fs = re.findall(r"fail-safe flies now|left offboard|transition failed|didn't upload|not landed", fly)
    out.append((not fs, "no fail-safe", fs[0] if fs else ""))
    return out


def judge_landing(kind, east, north, rw):
    """Where it came down: (ok, what, detail). The cessna on the runway (rw from
    sim_vehicles.runway), the others on the pad."""
    if kind != "fixedwing":
        d = math.hypot(east, north)
        return d <= 10.0, "came down on the pad (within 10 m)", f"{d:.1f} m from it"
    if rw is None:
        return True, "came down: no runway, it circles at the end", ""
    h = math.radians(rw["heading"])
    along = east * math.sin(h) + north * math.cos(h)
    cross = east * math.cos(h) - north * math.sin(h)
    ok = rw["from"] <= along <= rw["to"] and abs(cross) <= rw["width"] / 2 + 5.0
    return ok, "came down on the runway", f"{along:.0f} m along it ({rw['from']:.0f} to {rw['to']:.0f}), {cross:+.1f} m across"


def read_truth(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    truth, status, agl, scan = [], [], [], []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic == "/gdn/sim/truth_odom":
            m = deserialize_message(data, types[topic])
            p, q = m.pose.pose.position, m.pose.pose.orientation
            truth.append((t_ns * 1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z,
                          m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
        elif topic == "/gdn/agl_meas":
            agl.append((t_ns * 1e-9, deserialize_message(data, types[topic]).data))
        elif topic == "/gdn/sim/agl_scan":
            m = deserialize_message(data, types[topic])
            if len(m.ranges) and math.isfinite(m.ranges[0]) and m.range_min <= m.ranges[0] <= m.range_max:
                scan.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, float(m.ranges[0])))
        elif topic == "/fmu/out/vehicle_status_v1":
            m = deserialize_message(data, types[topic])
            status.append((t_ns * 1e-9, int(m.vehicle_type), bool(m.in_transition_mode), int(m.nav_state)))
    return np.array(truth), np.array(status), np.array(agl).reshape(-1, 2), np.array(scan).reshape(-1, 2)


def cast(alt, hdr, z0, x, y, z, q):
    """Range from (x, y, z) (world ENU) along body down at attitude q (w, x, y, z) to the
    map's terrain, the tilt in degrees and the height straight above the ground."""
    from make_sim_world import sample_map
    qw, qx, qy, qz = q
    down = -np.array([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)])
    nadir = z - (float(sample_map(alt, hdr, np.array(x), np.array(y))) - z0)
    if nadir < 5.0 or down[2] > -0.3:
        return None
    s = np.arange(0.0, 3.0 * (nadir + 50.0) / -down[2], 0.25)
    hit = np.nonzero(z + s * down[2] <= sample_map(alt, hdr, x + s * down[0], y + s * down[1]) - z0)[0]
    if not len(hit):
        return None
    return s[hit[0]], math.degrees(math.acos(-down[2])), nadir


def beam_scan(truth, scan, alt, hdr, z0, every=10):
    """Radar ranges against the truth at the same Gazebo time (truth column 8, scan
    column 0), pose interpolated. Returns (N, 4) like beam_agl: measured and predicted
    as heights (range times the cosine of the tilt), tilt in degrees, height straight
    above the ground."""
    th = truth[:, 8]
    out = []
    for t, rng in scan[::every]:
        k = np.searchsorted(th, t)
        if k <= 0 or k >= len(th) or th[k] - th[k - 1] > 0.2:
            continue
        f = (t - th[k - 1]) / max(th[k] - th[k - 1], 1e-9)
        x, y, z = truth[k - 1, 1:4] + f * (truth[k, 1:4] - truth[k - 1, 1:4])
        q0, q1 = truth[k - 1, 4:8], truth[k, 4:8]
        q = q0 + f * ((q1 if np.dot(q0, q1) >= 0 else -q1) - q0)
        c = cast(alt, hdr, z0, x, y, z, q / np.linalg.norm(q))
        if c is not None:
            cos_t = math.cos(math.radians(c[1]))
            out.append((rng * cos_t, c[0] * cos_t, c[1], c[2]))
    return np.array(out).reshape(-1, 4)


def beam_agl(truth, agl, alt, hdr, z0):
    """For each AGL measurement (t, agl), paired with the truth by bag arrival time: the
    truth's prediction of it, casting the beam onto the map. The fallback for bags
    without /gdn/sim/agl_scan; same columns as beam_scan."""
    out = []
    for t, a in agl:
        k = np.searchsorted(truth[:, 0], t)
        if k <= 0 or k >= len(truth):
            continue
        c = cast(alt, hdr, z0, *truth[k, 1:4], truth[k, 4:8])
        if c is not None:
            out.append((a, c[0] * math.cos(math.radians(c[1])), c[1], c[2]))
    return np.array(out).reshape(-1, 4)


def check_run(d, a):
    d = Path(d)
    need = ["run_info.txt", "route.csv", "fly.log", "adapter.log", "sitl_bag"]
    miss = [n for n in need if not (d / n).exists()]
    if miss:
        print(f"check_vehicles: {d} lacks {', '.join(miss)} (a tools/sim_run.sh log directory?)")
        return 2
    info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
    v = sim_vehicles.get(info["vehicle"])
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    print(f"flight: {info['vehicle']} ({v.kind}) over {info['world']}")
    route = np.loadtxt(d / "route.csv", delimiter=",", skiprows=1)
    truth, status, agl, scan = read_truth(d / "sitl_bag")
    if len(truth) < 100:
        print(f"check_vehicles: only {len(truth)} truth samples in the bag")
        return 2
    from gdn_paths import load_map
    from make_sim_world import sample_map
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    alt = alt.astype(np.float64)
    east, north, z = truth[:, 1], truth[:, 2], truth[:, 3]
    ground = sample_map(alt, hdr, east, north) - float(winfo["z0_amsl"])
    h_agl = z - ground
    # Airborne: more than 3 m above the ground.
    air = h_agl > 3.0
    ne = np.column_stack([north, east])
    fw_mask = follow_mask = None
    if len(status):
        # PX4's state at each truth sample (the latest status message before it).
        k = np.clip(np.searchsorted(status[:, 0], truth[:, 0], side="right") - 1, 0, len(status) - 1)
        st = status[k][air]
        fw_mask = st[:, 1] == 2                            # VEHICLE_TYPE_FIXED_WING
        follow_mask = (st[:, 3] == 14) & (st[:, 2] == 0)   # NAVIGATION_STATE_OFFBOARD, not in transition
    # Runs from before the runway (no launch= in run_info.txt) were catapult launches
    # that ended circling.
    launch = info.get("launch", "catapult")
    rw = sim_vehicles.runway(winfo) if "launch" in info else None
    results = judge_log(v.kind, (d / "fly.log").read_text(), launch, rw is not None)
    results.append(judge_landing(v.kind, truth[-1, 1], truth[-1, 2], rw))
    zone = None
    if rw is not None:
        # Within 200 m of the runway (its centreline segment), where a plane comes down.
        hd = math.radians(rw["heading"])
        al = east * math.sin(hd) + north * math.cos(hd)
        cr = east * math.cos(hd) - north * math.sin(hd)
        zone = np.hypot(np.maximum(np.maximum(rw["from"] - al, al - rw["to"]), 0.0), cr)[air] < 200.0
    results += judge_track(route[:, :2], ne[air], h_agl[air], v.kind, a.reach, a.xtrack, a.xtrack_fw,
                           a.clearance, fw_mask, follow_mask, a.clearance_other, zone)
    fallback = "AGL now comes from the map" in (d / "adapter.log").read_text()
    z0 = float(winfo["z0_amsl"])
    if len(scan):
        b, how = beam_scan(truth, scan, alt, hdr, z0), "radar ranges at Gazebo time"
    else:
        b, how = beam_agl(truth, agl, alt, hdr, z0), "GDN's AGL input by arrival time (no agl_scan in the bag)"
    steep = int(np.sum(b[:, 2] > MAX_TILT_DEG))
    b = b[b[:, 2] <= MAX_TILT_DEG]
    if len(b) >= 20 and not fallback:
        res = b[:, 0] - b[:, 1]
        rms = float(np.sqrt(np.mean(res ** 2)))
        results.append((rms <= a.radar_rms, f"radar altimeter sees the ground along its beam (RMS under {a.radar_rms:.1f} m)",
                        f"RMS {rms:.2f} m, p95 {np.percentile(np.abs(res), 95):.2f} m, max {np.abs(res).max():.1f} m "
                        f"over {len(b)} readings ({steep} tilted past {MAX_TILT_DEG:.0f} deg left out); {how}"))
    else:
        results.append((False, "radar altimeter sees the ground along its beam",
                        "no radar: the adapter fell back to the map" if fallback else f"only {len(b)} readings"))
    scorer = (d / "scorer.log").read_text() if (d / "scorer.log").exists() else ""
    results.append(("RMS" in scorer, "GDN ran alongside (scorer reported)",
                    next((l.strip() for l in scorer.splitlines() if "RMS" in l), "")))
    ok = True
    for r_ok, what, detail in results:
        ok &= ck(r_ok, what, detail)
    t_air = truth[air, 0]
    print(f"airborne {t_air[-1] - t_air[0]:.0f} s, {np.sum(np.linalg.norm(np.diff(ne[air], axis=0), axis=1)) / 1000:.1f} km")
    return 0 if ok else 1


# --- the judging, on made-up tracks ---

def selfcheck():
    ok = True
    route = np.array([[0.0, 0.0], [-400.0, 0.0], [-400.0, 250.0], [400.0, 250.0], [0.0, 0.0]])
    dense = np.vstack([np.linspace(route[i], route[i + 1], 200) for i in range(len(route) - 1)])
    rng = np.random.default_rng(1)
    good = dense + rng.normal(0.0, 3.0, dense.shape)
    h = np.full(len(good), 60.0)

    def passes(res):
        return all(r[0] for r in res)
    ok &= ck(passes(judge_track(dense, good, h, "multirotor", 30, 15, 40, 25)), "selfcheck: a good track passes")
    ok &= ck(not passes(judge_track(dense, good[: len(good) // 2], h[: len(h) // 2], "multirotor", 30, 15, 40, 25)),
             "selfcheck: half a route fails")
    off = good + np.array([0.0, 35.0])
    ok &= ck(not passes(judge_track(dense, off, h, "multirotor", 60, 15, 40, 25)), "selfcheck: 35 m off the route fails")
    ok &= ck(passes(judge_track(dense, off, h, "fixedwing", 60, 15, 40, 25)),
             "selfcheck: 35 m off is within the plane's limit")
    low = h.copy()
    low[300] = 10.0
    ok &= ck(not passes(judge_track(dense, good, low, "multirotor", 30, 15, 40, 25)), "selfcheck: 10 m above ground fails")
    ok &= ck(abs(cross_track(route, np.array([[-200.0, 10.0]]))[0] - 10.0) < 1e-9, "selfcheck: cross-track distance")
    fly = "at cruise height\nfront transition done\nback transition done\nover the pad, landing\nlanded and disarmed"
    ok &= ck(passes(judge_log("vtol", fly)), "selfcheck: VTOL phases found")
    ok &= ck(not passes(judge_log("vtol", fly.replace("front transition done", ""))),
             "selfcheck: a missing transition fails")
    rw = {"heading": 90.0, "grade": 0.0, "from": -60.0, "to": 240.0, "width": 20.0}
    ok &= ck(judge_landing("fixedwing", 120.0, 3.0, rw)[0] and not judge_landing("fixedwing", 120.0, 20.0, rw)[0]
             and not judge_landing("fixedwing", 300.0, 0.0, rw)[0], "selfcheck: on the runway or not")
    fwlog = "runway take-off\nclimbed out\nroute done, landing on the runway\nlanded and disarmed"
    ok &= ck(passes(judge_log("fixedwing", fwlog)) and not passes(judge_log("fixedwing", fwlog, "catapult")),
             "selfcheck: cessna phases, runway and catapult")
    ok &= ck(not passes(judge_log("multirotor", "at cruise height\nroute done, landing\nlanded and disarmed\n"
                                  "PX4 left offboard 5 s into the route")), "selfcheck: a fail-safe fails")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", help="a tools/sim_run.sh log directory to judge")
    ap.add_argument("--selfcheck", action="store_true")
    ap.add_argument("--reach", type=float, default=60.0, help="m: every route point passed this close")
    ap.add_argument("--xtrack", type=float, default=15.0, help="m: cross-track p95 limit hovering")
    ap.add_argument("--xtrack-fw", type=float, default=40.0, dest="xtrack_fw", help="m: cross-track p95 limit as a plane")
    ap.add_argument("--clearance", type=float, default=25.0, help="m above the terrain on the route, away from the pad")
    ap.add_argument("--clearance-other", type=float, default=15.0, dest="clearance_other",
                    help="m above the terrain off the route (transitions, holds, landing approach)")
    ap.add_argument("--radar-rms", type=float, default=1.0, dest="radar_rms",
                    help="m: radar AGL against the beam cast at the true attitude, RMS")
    a = ap.parse_args()
    if a.selfcheck:
        return 0 if selfcheck() else 1
    if a.run:
        return check_run(a.run, a)
    return 0 if check_models() else 1


if __name__ == "__main__":
    sys.exit(main())
