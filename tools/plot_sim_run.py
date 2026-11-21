#!/usr/bin/env python3
"""Draw a whole SITL flight: GDN's estimate against the true flight, and the error between
them, from take-off to landing.

    python3 tools/plot_sim_run.py log/demo_emmental_x500          # -> <run>/gdn_vs_truth.png
    python3 tools/plot_sim_run.py RUN_DIR --gdn-log log/gdn_runs/gdn_20261121_073545 --out fig.png

RUN_DIR is a tools/sim_run.sh log directory (GDN_LOG_DIR). The truth and the flight phases
come from its bag (sitl_bag: /gdn/sim/truth_odom, /gdn/truth, PX4's vehicle_status) and
fly.log; GDN's estimate, its uncertainty and every update decision from GDN's own run log
(R1.11, log/gdn_runs/gdn_*), found by time unless --gdn-log is given. Without a run log it
falls back to the bag's /gdn/odom (no uncertainty, no update panels). Needs ROS 2 to read
the bags: run it in the dev image (tools/docker_dev.sh shell).

The page:
  - the true track and GDN's over the world's terrain, the planned route, and a line from
    truth to GDN every 20 s;
  - the error vector (GDN minus truth, north and east) over the flight;
  - horizontal error with GDN's own 3 sigma, shaded by flight phase from PX4 (ground,
    hover, transition, cruise) with fly.log's events (take-off, route, GPS cut, landing);
  - north, east and vertical error with 3 sigma; height, speed and heading, truth and GDN;
  - every terrain altimeter reading: applied, gated (with the gate) or skipped and why;
  - every update decision per sensor;
  - a summary table.

Frames: GDN and /gdn/truth are the world's north/east/down with the origin at the world
centre (the pad), as tools/replay_scorer.py compares them. Truth is /gdn/sim/truth_odom
(Gazebo, about 20 Hz) at GDN's state times; its down axis is put on /gdn/truth's datum.
GDN's run log is on the filter's clock (the IMU's, Unix time in SITL) and the bag on
receive time; they agree to about 50 ms, which is under a metre at these speeds.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parent.parent

INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
TRUTH, GDN, ERR, GATED, SKIP = "#0b0b0b", "#2a78d6", "#eb6834", "#eb6834", "#a8a7a3"
PHASE = {"ground": ("#eeede9", "on the ground (disarmed)"), "hover": ("#e4eefa", "hover (rotary wing)"),
         "transition": ("#fbe7da", "VTOL transition"), "cruise": ("#e6f2e1", "cruise (fixed wing)")}
SENSORS = {2: "terrain altimeter (AGL)", 1: "terrain (TRN)", 3: "barometer", 4: "magnetometer (3-axis)",
           5: "magnetometer (heading)", 6: "zero-velocity (ZUPT)", 7: "GNSS"}
REASONS = {0: "applied", 1: "not aligned", 2: "no heading", 3: "no altitude datum", 4: "off the map",
           5: "terrain too flat", 6: "weak field", 7: "gated", 8: "numeric", 9: "bad input", 10: "initialise",
           11: "not used"}


# ------------------------------------------------------------------------- reading ----
def _reader(uri):
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    return r, types


def read_bag(bag):
    from rclpy.serialization import deserialize_message
    r, types = _reader(bag)
    want = {"/gdn/sim/truth_odom", "/gdn/truth", "/fmu/out/vehicle_status_v1", "/gdn/odom"}
    d = {k: [] for k in want}
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic not in want:
            continue
        m = deserialize_message(raw, types[topic])
        ts = t * 1e-9
        if topic == "/gdn/sim/truth_odom":
            p, q = m.pose.pose.position, m.pose.pose.orientation
            d[topic].append((ts, p.x, p.y, p.z, q.w, q.x, q.y, q.z,
                             m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))   # ENU, + Gazebo time
        elif topic == "/gdn/truth":
            d[topic].append((ts, m.point.x, m.point.y, m.point.z))           # NED
        elif topic == "/fmu/out/vehicle_status_v1":
            d[topic].append((ts, m.arming_state, m.vehicle_type, int(m.in_transition_mode), m.nav_state))
        else:
            p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
            d[topic].append((ts, p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z))
    return {k: np.array(v, dtype=float) for k, v in d.items()}


def bag_span(bag):
    meta = (Path(bag) / "metadata.yaml").read_text()
    t0 = int(re.search(r"starting_time:\s*nanoseconds_since_epoch:\s*(\d+)", meta).group(1)) * 1e-9
    dur = int(re.search(r"duration:\s*nanoseconds:\s*(\d+)", meta).group(1)) * 1e-9
    return t0, t0 + dur


def find_run_log(t_a, t_b):
    """The GDN run log (log/gdn_runs/gdn_*) that overlaps the bag the most."""
    best, best_ov = None, 0.0
    for p in sorted((ROOT / "log/gdn_runs").glob("gdn_*")):
        if not (p / "metadata.yaml").exists():
            continue
        try:
            a, b = bag_span(p)
        except (AttributeError, ValueError):
            continue
        ov = min(b, t_b) - max(a, t_a)
        if ov > best_ov:
            best, best_ov = p, ov
    return best


def read_run_log(path):
    from rclpy.serialization import deserialize_message
    r, types = _reader(path)
    states, updates, events, last = [], [], [], -1.0
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic not in ("/gdn/log/state", "/gdn/log/update", "/gdn/log/event"):
            continue
        m = deserialize_message(raw, types[topic])
        if topic == "/gdn/log/state":
            if m.flags & 1 and m.t > 0 and m.t >= last + 0.1:                  # aligned, 10 Hz
                last = m.t
                states.append([m.t, *m.position, *m.velocity, *m.orientation, *list(m.p_diag)[:9], m.p_ne,
                               m.flags])
        elif topic == "/gdn/log/update":
            updates.append([m.t, m.sensor, m.reason, m.innovation[0] if len(m.innovation) else math.nan,
                            m.innovation_var[0] if len(m.innovation_var) else math.nan])
        else:
            events.append((m.t, m.level, m.text))
    s, u = np.array(states, dtype=float), np.array(updates, dtype=float)
    if len(u) and len(s):
        u[u[:, 0] <= 0, 0] = s[0, 0]                     # records before alignment carry t = 0
    return s, u, events


def fly_events(path):
    """fly.log's milestones: (unix time, label)."""
    pats = (("offboard + arm requested", "take-off"), ("starting the route", "route starts"),
            ("front transition done", "transition to plane"), ("back transition done", "transition to hover"),
            ("GPS cut", "GPS cut"), ("route done", "route done"), ("landed", "landed"),
            ("giving up", "gave up"))
    out = []
    if not Path(path).exists():
        return out
    for line in Path(path).read_text().splitlines():
        m = re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)
        if not m:
            continue
        for key, lab in pats:
            if key in line:
                out.append((float(m.group(1)), lab))
                break
    return out


# ---------------------------------------------------------------------- arithmetic ----
def yaw_ned_from_q(w, x, y, z):
    """Heading (deg, 0 north, 90 east) from a body-to-NED quaternion (GDN)."""
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360


def yaw_ned_from_enu_q(w, x, y, z):
    """Heading (deg) from Gazebo's world-ENU model pose (x forward)."""
    return (90.0 - np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))) % 360


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def phases(status, t):
    """Flight phase at times t from PX4's vehicle_status: ground (disarmed), hover (rotary
    wing), transition, cruise (fixed wing)."""
    if not len(status):
        return np.array(["hover"] * len(t))
    k = np.clip(np.searchsorted(status[:, 0], t, side="right") - 1, 0, len(status) - 1)
    st = status[k]
    ph = np.where(st[:, 1] != 2, "ground", np.where(st[:, 3] == 1, "transition",
                                                    np.where(st[:, 2] == 2, "cruise", "hover")))
    return ph


def stats(e):
    return (float(np.sqrt(np.mean(e ** 2))), float(np.percentile(e, 95)), float(e.max())) if len(e) else (math.nan,) * 3


def build(run_dir, gdn_log):
    run_dir = Path(run_dir)
    info = dict(l.split("=", 1) for l in (run_dir / "run_info.txt").read_text().splitlines() if "=" in l)
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    bag = read_bag(run_dir / "sitl_bag")
    tr, tn = bag["/gdn/sim/truth_odom"], bag["/gdn/truth"]
    if not len(tr):
        sys.exit(f"{run_dir}/sitl_bag has no /gdn/sim/truth_odom")
    if gdn_log is None:
        gdn_log = find_run_log(*bag_span(run_dir / "sitl_bag"))
    if gdn_log is not None:
        s, u, events = read_run_log(gdn_log)
    else:
        o = bag["/gdn/odom"]
        s = np.full((len(o), 22), np.nan)
        s[:, 0:7] = o[:, 0:7]
        s[:, 7:11] = o[:, 7:11]
        u, events = np.zeros((0, 5)), []
    if not len(s):
        sys.exit("no GDN states (run log or /gdn/odom)")
    t = s[:, 0]
    keep = (t >= tr[0, 0]) & (t <= tr[-1, 0])
    s, t = s[keep], t[keep]
    # Truth at GDN's state times.
    n_t = np.interp(t, tr[:, 0], tr[:, 2])
    e_t = np.interp(t, tr[:, 0], tr[:, 1])
    up = np.interp(t, tr[:, 0], tr[:, 3])
    c = np.median(tn[:, 3] + np.interp(tn[:, 0], tr[:, 0], tr[:, 3])) if len(tn) else 0.0
    d_t = -up + c                                          # /gdn/truth's down datum
    # Velocity on Gazebo's own clock (receive times jitter), then to GDN's state times.
    ts_gz = tr[:, 8]
    ok = np.concatenate([[True], np.diff(ts_gz) > 0])
    trv = tr[ok]
    vel = np.stack([np.gradient(trv[:, k], trv[:, 8]) for k in (2, 1)] + [-np.gradient(trv[:, 3], trv[:, 8])], axis=1)
    v_t = np.stack([np.interp(t, trv[:, 0], vel[:, k]) for k in range(3)], axis=1)
    # Heading: from each truth sample, unwrapped, then interpolated (interpolating the
    # quaternion's components breaks where its sign flips).
    yaw_s = np.unwrap(np.radians(yaw_ned_from_enu_q(tr[:, 4], tr[:, 5], tr[:, 6], tr[:, 7])))
    yaw_t = np.degrees(np.interp(t, tr[:, 0], yaw_s)) % 360
    # Ground under the truth (the world's own terrain).
    from gdn_paths import load_map
    from make_sim_world import sample_map
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    alt = alt.astype(np.float64)
    z0 = float(winfo["z0_amsl"])
    ground_up = sample_map(alt, hdr, e_t, n_t) - z0        # world z of the ground
    agl_t = up - ground_up
    g = {"t": t, "n": s[:, 1], "e": s[:, 2], "d": s[:, 3], "v": s[:, 4:7],
         "yaw": yaw_ned_from_q(s[:, 7], s[:, 8], s[:, 9], s[:, 10]),
         "pnn": s[:, 11], "pee": s[:, 12], "pdd": s[:, 13], "pvv": s[:, 14:17], "pyaw": s[:, 19], "pne": s[:, 20],
         "flags": s[:, 21]}
    truth = {"n": n_t, "e": e_t, "d": d_t, "v": v_t, "yaw": yaw_t, "agl": agl_t}
    # Horizontal 3 sigma: the larger axis of the position ellipse.
    a, b, cc = g["pnn"], g["pee"], g["pne"]
    lam = 0.5 * (a + b) + np.sqrt(0.25 * (a - b) ** 2 + cc ** 2)
    g["sig3_h"] = 3 * np.sqrt(lam)
    status = bag["/fmu/out/vehicle_status_v1"]
    rf = run_dir / "route.csv"
    route = np.genfromtxt(rf, delimiter=",", names=True) if rf.exists() else None
    return dict(info=info, winfo=winfo, gdn=g, truth=truth, updates=u, events=events, status=status,
                fly=fly_events(run_dir / "fly.log"), route=route, alt=alt, hdr=hdr, z0=z0, gdn_log=gdn_log,
                full_truth=tr, d_datum=c, run_dir=run_dir)


# -------------------------------------------------------------------------- drawing ----
def draw(D, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource
    from make_sim_world import sample_map
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.fontsize": 8, "legend.frameon": False})
    g, tr, info = D["gdn"], D["truth"], D["info"]
    t0 = g["t"][0]
    T = g["t"] - t0
    dn, de, dd = g["n"] - tr["n"], g["e"] - tr["e"], g["d"] - tr["d"]
    eh = np.hypot(dn, de)
    ph = phases(D["status"], g["t"])
    airborne = ph != "ground"
    have_sig = np.isfinite(g["sig3_h"]).any()
    ev = [(tt - t0, lab) for tt, lab in D["fly"] if tt - t0 >= T[0] - 5]

    fig = plt.figure(figsize=(16, 30))
    gs = fig.add_gridspec(9, 3, height_ratios=[2.5, 1.25, 1.0, 1.0, 1.0, 1.0, 1.2, 0.9, 0.9], hspace=0.55,
                          wspace=0.28, top=0.975, bottom=0.02)

    def shade(ax, legend=False):
        k0 = 0
        for k in range(1, len(T) + 1):
            if k == len(T) or ph[k] != ph[k0]:
                ax.axvspan(T[k0], T[min(k, len(T) - 1)], color=PHASE[ph[k0]][0], lw=0, zorder=0)
                k0 = k
        for x, lab in ev:
            ax.axvline(x, color=MUTED, lw=0.8, ls=(0, (2, 2)), zorder=1)
        ax.set_xlim(T[0], T[-1])
        ax.grid(color=GRID, lw=0.5)
        ax.set_axisbelow(True)
        if legend:
            from matplotlib.patches import Patch
            used = [p for p in PHASE if (ph == p).any()]
            return [Patch(color=PHASE[p][0], label=PHASE[p][1]) for p in used]
        return []

    def label_events(ax):
        ymax = ax.get_ylim()[1]
        for x, lab in ev:
            ax.text(x, ymax, f" {lab}", rotation=90, va="top", ha="left", fontsize=7.5, color=INK2)

    # 1. The map.
    ax = fig.add_subplot(gs[0, 0:2])
    ee = np.concatenate([g["e"], tr["e"]])
    nn = np.concatenate([g["n"], tr["n"]])
    pad = max(150.0, 0.08 * max(np.ptp(ee), np.ptp(nn)))
    e0, e1, n0, n1 = ee.min() - pad, ee.max() + pad, nn.min() - pad, nn.max() + pad
    span = max(e1 - e0, n1 - n0)
    ce, cn = 0.5 * (e0 + e1), 0.5 * (n0 + n1)
    e0, e1, n0, n1 = ce - span / 2, ce + span / 2, cn - span / 2, cn + span / 2
    step = span / 400
    E, N = np.meshgrid(np.arange(e0, e1, step), np.arange(n0, n1, step))
    H = sample_map(D["alt"], D["hdr"], E.ravel(), N.ravel()).reshape(E.shape)
    # Only where the world's terrain is: sample_map clamps at its edge, which drew stripes
    # when GDN's estimate ran far outside it.
    ny, nx = D["alt"].shape
    half_e = 0.5 * nx * abs(D["hdr"]["dlon"]) * D["hdr"]["m_per_deg_lon"]
    half_n = 0.5 * ny * abs(D["hdr"]["dlat"]) * D["hdr"]["m_per_deg_lat"]
    inside = (np.abs(E) <= half_e) & (np.abs(N) <= half_n)
    shadeimg = LightSource(azdeg=315, altdeg=40).hillshade(H[::-1], vert_exag=1.0, dx=step, dy=step)
    shadeimg = np.where(inside[::-1], shadeimg, np.nan)
    cmap = plt.get_cmap("gray").copy()
    cmap.set_bad("#f4f3ef")
    ax.imshow(shadeimg, extent=(e0, e1, n0, n1), cmap=cmap, vmin=-0.3, vmax=1.4, origin="upper",
              interpolation="bilinear", zorder=0)
    if inside.any():
        cs = ax.contour(E, N, np.where(inside, H, np.nan), levels=14, colors="#ffffff", linewidths=0.4, alpha=0.6,
                        zorder=1)
        ax.clabel(cs, fontsize=6, fmt="%.0f m", colors="#ffffff")
    if not inside.all():
        ax.plot([-half_e, half_e, half_e, -half_e, -half_e], [-half_n, -half_n, half_n, half_n, -half_n],
                color=MUTED, lw=0.8, zorder=2, label="edge of the world's terrain (outside: none)")
    half = float(D["winfo"].get("size_e", 4000)) / 2
    ax.plot([-half, half, half, -half, -half], [-half, -half, half, half, -half], color=INK2, lw=1.0,
            ls=(0, (6, 3)), zorder=2, label="edge of the simulated world")
    if D["route"] is not None:
        ax.plot(D["route"]["east_m"], D["route"]["north_m"], color="#ffffff", lw=3.5, alpha=0.8, zorder=2)
        ax.plot(D["route"]["east_m"], D["route"]["north_m"], color=INK2, lw=0.8, ls=(0, (3, 2)), zorder=3,
                label="planned route")
    every = max(1, int(round(20.0 / np.median(np.diff(T)))))
    for i in range(0, len(T), every):
        ax.plot([tr["e"][i], g["e"][i]], [tr["n"][i], g["n"][i]], color=ERR, lw=0.7, alpha=0.8, zorder=3)
    ax.plot([], [], color=ERR, lw=0.7, label="truth to GDN, every 20 s")
    ax.plot(tr["e"], tr["n"], color="#ffffff", lw=4.0, zorder=4)
    ax.plot(tr["e"], tr["n"], color=TRUTH, lw=1.8, zorder=5, label="true flight path (Gazebo)")
    ax.plot(g["e"], g["n"], color="#ffffff", lw=4.0, zorder=4)
    ax.plot(g["e"], g["n"], color=GDN, lw=1.8, zorder=6, label="GDN's estimate")
    ax.scatter([tr["e"][0]], [tr["n"][0]], s=90, marker="*", color=INK, edgecolor="#ffffff", zorder=8,
               label="take-off")
    ax.scatter([tr["e"][-1]], [tr["n"][-1]], s=55, marker="s", color=INK, edgecolor="#ffffff", zorder=8,
               label="true end")
    ax.scatter([g["e"][-1]], [g["n"][-1]], s=55, marker="s", color=GDN, edgecolor="#ffffff", zorder=8,
               label="GDN's end")
    ax.annotate(f"{eh[-1]:.0f} m", ((tr["e"][-1] + g["e"][-1]) / 2, (tr["n"][-1] + g["n"][-1]) / 2), color=INK,
                fontsize=9, fontweight="bold", ha="center", va="bottom", zorder=9,
                bbox=dict(boxstyle="round,pad=0.2", fc="#ffffff", ec="none", alpha=0.85))
    ax.set_xlim(e0, e1)
    ax.set_ylim(n0, n1)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m, from the world's centre)")
    ax.set_ylabel("north (m)")
    ax.set_title(f"The whole flight over {info['world']}: true path and GDN's estimate")
    ax.legend(loc="lower left", fontsize=8, frameon=True, framealpha=0.9)

    # 2. The error vector.
    ax = fig.add_subplot(gs[0, 2])
    sc = ax.scatter(de, dn, c=T, s=3, cmap="viridis", rasterized=True)
    fig.colorbar(sc, ax=ax, pad=0.02, shrink=0.8).set_label("time since alignment (s)")
    ax.axhline(0, color=INK2, lw=0.6)
    ax.axvline(0, color=INK2, lw=0.6)
    m = 1.1 * max(np.abs(de).max(), np.abs(dn).max(), 1.0)
    ax.set_xlim(-m, m)
    ax.set_ylim(-m, m)
    ax.set_aspect("equal")
    ax.grid(color=GRID, lw=0.5)
    ax.set_xlabel("east error, GDN minus truth (m)")
    ax.set_ylabel("north error (m)")
    ax.set_title("Where GDN's error points, over time")

    # 3. Horizontal error.
    ax = fig.add_subplot(gs[1, :])
    handles = shade(ax, legend=True)
    ax.plot(T, eh, color=ERR, lw=1.6, label="horizontal error, GDN against truth")
    if have_sig:
        ax.plot(T, g["sig3_h"], color=GDN, lw=1.1, ls="--", label="GDN's own 3 sigma (largest axis)")
        over = airborne & (eh > g["sig3_h"])
        ax.fill_between(T, 0, eh, where=over, color=ERR, alpha=0.12, lw=0, label="error beyond GDN's 3 sigma")
    ax.set_ylim(0, 1.08 * max(eh.max(), np.nanmax(g["sig3_h"]) if have_sig else 0, 1))
    ax.set_ylabel("m")
    ax.set_xlabel("time since GDN aligned (s)")
    ax.set_title("Horizontal error over the whole flight")
    label_events(ax)
    ax.legend(handles=ax.get_legend_handles_labels()[0] + handles, loc="upper left", ncol=3,
              bbox_to_anchor=(0.07, 1.0), frameon=True, framealpha=0.9)

    # 4. North, east, heading.
    for k, (err, var, lab) in enumerate(((dn, g["pnn"], "North error"), (de, g["pee"], "East error"))):
        ax = fig.add_subplot(gs[2, k])
        shade(ax)
        if have_sig:
            s3 = 3 * np.sqrt(var)
            ax.fill_between(T, -s3, s3, color=GDN, alpha=0.12, lw=0, label="GDN's +-3 sigma")
        ax.plot(T, err, color=ERR, lw=1.3, label="GDN minus truth")
        ax.axhline(0, color=INK2, lw=0.6)
        ax.set_ylabel("m")
        ax.set_title(lab)
        ax.legend(loc="upper left")
    ax = fig.add_subplot(gs[2, 2])
    shade(ax)
    eyaw = wrap180(g["yaw"] - tr["yaw"])
    if have_sig:
        s3 = 3 * np.degrees(np.sqrt(g["pyaw"]))
        ax.fill_between(T, -s3, s3, color=GDN, alpha=0.12, lw=0, label="GDN's +-3 sigma")
    ax.plot(T, eyaw, color=ERR, lw=1.0, label="GDN minus truth")
    ax.axhline(0, color=INK2, lw=0.6)
    lim = max(5.0, 1.2 * np.percentile(np.abs(eyaw), 99.5))
    ax.set_ylim(-lim, lim)
    ax.set_ylabel("deg")
    ax.set_title("Heading error")
    ax.legend(loc="upper left")

    # 5. Height and vertical error.
    ax = fig.add_subplot(gs[3, 0:2])
    shade(ax)
    ax.plot(T, -tr["d"], color=TRUTH, lw=1.6, label="true height (above the take-off datum)")
    ax.plot(T, -g["d"], color=GDN, lw=1.3, label="GDN's height")
    ax.plot(T, tr["agl"], color=MUTED, lw=1.0, ls=(0, (4, 2)), label="true height above the ground below")
    ax.set_ylabel("m")
    ax.set_title("Height")
    ax.legend(loc="upper left", ncol=3)
    ax = fig.add_subplot(gs[3, 2])
    shade(ax)
    if have_sig:
        s3 = 3 * np.sqrt(g["pdd"])
        ax.fill_between(T, -s3, s3, color=GDN, alpha=0.12, lw=0, label="GDN's +-3 sigma")
    ax.plot(T, -dd, color=ERR, lw=1.2, label="GDN minus truth (up)")
    ax.axhline(0, color=INK2, lw=0.6)
    ax.set_ylabel("m")
    ax.set_title("Vertical error")
    ax.legend(loc="upper left")

    # 6. Speed and velocity error.
    ax = fig.add_subplot(gs[4, 0:2])
    shade(ax)
    sp_t = np.hypot(tr["v"][:, 0], tr["v"][:, 1])
    sp_g = np.hypot(g["v"][:, 0], g["v"][:, 1])
    ax.plot(T, sp_t, color=TRUTH, lw=1.4, label="true ground speed")
    ax.plot(T, sp_g, color=GDN, lw=1.1, label="GDN's ground speed")
    ax.set_ylabel("m/s")
    ax.set_title("Ground speed")
    ax.legend(loc="upper left")
    ax = fig.add_subplot(gs[4, 2])
    shade(ax)
    dv = np.hypot(g["v"][:, 0] - tr["v"][:, 0], g["v"][:, 1] - tr["v"][:, 1])
    ax.plot(T, dv, color=ERR, lw=1.0, label="horizontal velocity error")
    if have_sig:
        ax.plot(T, 3 * np.sqrt(np.maximum(g["pvv"][:, 0], g["pvv"][:, 1])), color=GDN, lw=1.0, ls="--",
                label="GDN's 3 sigma")
    ax.set_ylim(0, max(1.0, 1.1 * np.percentile(dv, 99.5)))
    ax.set_ylabel("m/s")
    ax.set_title("Velocity error")
    ax.legend(loc="upper left")

    # 7. Terrain altimeter readings.
    u = D["updates"]
    ax = fig.add_subplot(gs[5, :])
    shade(ax)
    if len(u):
        U = u[(u[:, 1] == 2) | (u[:, 1] == 1)]
        tu = U[:, 0] - t0
        app, gat = U[:, 2] == 0, U[:, 2] == 7
        skp = ~(app | gat)
        fin = np.isfinite(U[:, 4])
        o = np.argsort(tu)
        ax.fill_between(tu[o], -3 * np.sqrt(np.where(fin, U[:, 4], np.nan)[o]),
                        3 * np.sqrt(np.where(fin, U[:, 4], np.nan)[o]), color=GDN, alpha=0.12, lw=0,
                        label="gate: 3 x predicted sigma")
        ax.scatter(tu[app], U[app, 3], s=6, color=GDN, label=f"applied ({app.sum()})", zorder=3)
        ax.scatter(tu[gat], U[gat, 3], s=14, marker="x", color=GATED, label=f"gated ({gat.sum()})", zorder=3)
        lim = np.nanpercentile(np.abs(U[app | gat, 3]), 99) * 1.3 if (app | gat).any() else 10
        ax.set_ylim(-lim, lim)
        yb = -lim * 0.93
        for r in sorted(set(U[skp, 2].astype(int))):
            w = skp & (U[:, 2] == r)
            ax.scatter(tu[w], np.full(w.sum(), yb), marker="|", s=40, color=SKIP if r != 4 else INK2,
                       label=f"skipped: {REASONS.get(r, r)} ({w.sum()})", zorder=3)
            yb += lim * 0.07
        ax.axhline(0, color=INK2, lw=0.6)
    else:
        ax.text(0.5, 0.5, "no GDN run log: update decisions unavailable", transform=ax.transAxes, ha="center")
    ax.set_ylabel("innovation (m)")
    ax.set_title("Every terrain altimeter reading: measured minus GDN's map prediction")
    ax.legend(loc="upper left", ncol=4)

    # 8. Update decisions per sensor.
    ax = fig.add_subplot(gs[6, :])
    shade(ax)
    if len(u):
        rows = [k for k in (2, 1, 3, 4, 5, 6, 7) if (u[:, 1] == k).any()]
        for y, k in enumerate(rows):
            w = u[:, 1] == k
            for cat, colr, sel in (("applied", GDN, u[:, 2] == 0), ("gated", GATED, u[:, 2] == 7),
                                   ("skipped", SKIP, (u[:, 2] != 0) & (u[:, 2] != 7))):
                ww = w & sel
                if ww.any():
                    ax.scatter(u[ww, 0] - t0, np.full(ww.sum(), y), marker="|", s=60, color=colr,
                               rasterized=True, label=cat if y == 0 or k == rows[-1] else None)
        ax.set_yticks(range(len(rows)))
        ax.set_yticklabels([SENSORS[k] for k in rows])
        ax.set_ylim(-0.6, len(rows) - 0.4)
        h, l = ax.get_legend_handles_labels()
        uniq = dict(zip(l, h))
        ax.legend(uniq.values(), uniq.keys(), loc="upper right", ncol=3, frameon=True, framealpha=0.9)
    ax.set_xlabel("time since GDN aligned (s)")
    ax.set_title("What GDN did with each input: applied, gated or skipped")

    # 9. Summary.
    ax = fig.add_subplot(gs[7:, :])
    ax.axis("off")
    dist = float(np.sum(np.hypot(np.diff(D["full_truth"][:, 1]), np.diff(D["full_truth"][:, 2]))))
    air = airborne
    win = []
    for name, hi in (("first 5 minutes", 300), ("first 10 minutes", 600), ("whole flight", 1e9)):
        rms, p95, mx = stats(eh[T <= hi])
        win.append((f"Horizontal error, {name} (RMS / 95% / max)", f"{rms:.1f} / {p95:.1f} / {mx:.1f} m"))
    site = f" (site {info['site']})" if info.get("site") else ""
    rows = [("Vehicle, world", f"{info['vehicle']} over {info['world']}{site}"),
            ("Flight", f"{T[-1]:.0f} s from alignment, {air.sum() / max(len(T), 1) * T[-1]:.0f} s in the air, "
                       f"{dist / 1000:.2f} km flown, true height {(-tr['d']).min():.0f} to {(-tr['d']).max():.0f} m "
                       f"(above ground {np.nanmin(tr['agl'][air]) if air.any() else 0:.0f} to "
                       f"{np.nanmax(tr['agl'][air]) if air.any() else 0:.0f} m)"),
            *win,
            ("Horizontal error at the end",
             f"{eh[-1]:.1f} m ({100 * eh[-1] / max(dist, 1):.1f}% of the distance flown)"),
            ("Vertical error (RMS / max)", f"{np.sqrt(np.mean(dd ** 2)):.1f} / {np.abs(dd).max():.1f} m"),
            ("Heading error (RMS / max)", f"{np.sqrt(np.mean(eyaw ** 2)):.1f} / {np.abs(eyaw).max():.1f} deg"),
            ("Speed error (RMS / max)", f"{np.sqrt(np.mean(dv ** 2)):.2f} / {dv.max():.2f} m/s")]
    if have_sig:
        rows.append(("In the air, error beyond GDN's own 3 sigma",
                     f"{100 * np.mean(eh[air] > g['sig3_h'][air]):.1f}% of the time (an honest filter: under 1%)"))
    if len(u):
        U = u[u[:, 1] == 2]
        parts = [f"{REASONS.get(int(r), r)} {int((U[:, 2] == r).sum())}" for r in sorted(set(U[:, 2]))]
        rows.append(("Terrain altimeter readings", f"{len(U)}: " + ", ".join(parts)))
        off = U[U[:, 2] == 4]
        if len(off):
            rows.append(("First reading skipped as off the map", f"{off[0, 0] - t0:.0f} s"))
    rows.append(("Sources", f"truth: {D['run_dir']}/sitl_bag; GDN: log/gdn_runs/"
                            f"{Path(D['gdn_log']).name if D['gdn_log'] else '/gdn/odom (no run log)'}"))
    tb = ax.table(cellText=[[r[0], r[1]] for r in rows], cellLoc="left", loc="upper center",
                  colWidths=[0.3, 0.66])
    tb.auto_set_font_size(False)
    tb.set_fontsize(9)
    tb.scale(1, 1.45)
    for (r, c), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        if c == 0:
            cell.get_text().set_color(INK2)
    ax.set_title("Summary", pad=8)
    fig.suptitle(f"GDN against the true flight: {info['vehicle']} over {info['world']}",
                 x=0.06, y=0.993, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.savefig(out, dpi=110, facecolor="white", bbox_inches="tight")
    print(f"wrote {out}")
    for r in rows:
        print(f"  {r[0]}: {r[1]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir", help="a tools/sim_run.sh log directory")
    ap.add_argument("--gdn-log", help="GDN's run log (log/gdn_runs/gdn_*); found by time if left out")
    ap.add_argument("--out", help="the PNG (default RUN_DIR/gdn_vs_truth.png)")
    a = ap.parse_args()
    D = build(a.run_dir, a.gdn_log)
    draw(D, a.out or str(Path(a.run_dir) / "gdn_vs_truth.png"))


if __name__ == "__main__":
    main()
