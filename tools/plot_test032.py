#!/usr/bin/env python3
"""TEST-032 figure: why GDN runs away on the x500 in SITL (OI-012), and the Stage B GPS cut.

    # from run logs and their what-if replays (gdn_replay --set ... --out DIR):
    python3 tools/plot_test032.py --export docs/test_reports/test032_data \\
        "081427:as flown=log/gdn_runs/gdn_20261116_081427" "081427:no altimeter=/path/w_noagl" ... \\
        --gps-kept log/gdn_runs/gdn_20261116_084914 --gps-cut log/gdn_runs/gdn_20261116_112059
    # from the exported CSVs (what's committed):
    python3 tools/plot_test032.py docs/test_reports/test032_data --out docs/test_reports/test032_oi012.png

Each run is "FLIGHT:VARIANT=RUN_LOG"; every flight needs an "as flown" one. All variants
of a flight are scored against PX4's position (GPS-aided, eph 0.2 m: GPS was on in these
runs, see tools/check_gps_cut.py) over one fixed window: that flight's airborne latch to
its last applied altimeter update as flown. The first flight also gets a 1 Hz error
timeline (as flown and "no altimeter").
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

C1, C2, C3, GREY = "#2a78d6", "#eb6834", "#52514e", "#a3a29d"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
MARK = ("o", "s", "^")


def read(run_log):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(run_log), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    st, lp, eph, agl, latch, over = [], [], [], [], None, ""
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in ("/gdn/log/state", "/fmu/out/vehicle_local_position", "/gdn/log/update",
                         "/gdn/log/event", "/gdn/log/run_info"):
            continue
        m = deserialize_message(data, types[topic])
        if topic == "/gdn/log/state":
            if m.t > 0:
                st.append([m.t, m.position[0], m.position[1], m.velocity[0], m.velocity[1],
                           max(m.p_diag[0], m.p_diag[1])])
        elif topic == "/fmu/out/vehicle_local_position":
            eph.append([m.timestamp * 1e-6, m.eph, float(m.xy_valid)])
            if m.xy_valid:
                lp.append([m.timestamp * 1e-6, m.x, m.y, m.vx, m.vy])
        elif topic == "/gdn/log/update":
            if m.sensor == 2:
                agl.append([m.t, m.reason])
        elif topic == "/gdn/log/event":
            if "airborne latch" in m.text:
                latch = m.t
        else:
            over = m.config_overrides
    return dict(st=np.array(st), lp=np.array(lp), eph=np.array(eph), agl=np.array(agl), latch=latch, over=over)


def errors(d):
    s, lp = d["st"], d["lp"]
    ref = [np.interp(s[:, 0], lp[:, 0], lp[:, k]) for k in (1, 2, 3, 4)]
    return (np.hypot(s[:, 1] - ref[0], s[:, 2] - ref[1]), np.hypot(s[:, 3] - ref[2], s[:, 4] - ref[3]),
            3.0 * np.sqrt(s[:, 5]))


def write_csv(path, head, rows):
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([head] + rows)


def export(runs, kept, cut, out):
    out.mkdir(parents=True, exist_ok=True)
    flights = list(dict.fromkeys(f for f, _, _ in runs))
    rows = []
    for fl in flights:
        mine = [(v, p) for f, v, p in runs if f == fl]
        base_path = dict(mine).get("as flown")
        if base_path is None:
            sys.exit(f"flight {fl} has no 'as flown' run")
        base = read(base_path)
        applied = base["agl"][base["agl"][:, 1] == 0, 0]
        t0, t1 = base["latch"], applied[-1]
        t_ref0 = base["lp"][0, 0]
        for v, p in mine:
            d = base if p == base_path else read(p)
            e, ev, s3 = errors(d)
            w = (d["st"][:, 0] >= t0) & (d["st"][:, 0] <= t1)
            settings = "the run's own" if p == base_path else (d["over"] or "none: a scratch code change")
            rows.append([fl, v, settings, f"{t1 - t0:.0f}", f"{np.sqrt(np.mean(e[w] ** 2)):.1f}",
                         f"{e[w].max():.1f}", f"{100 * np.mean(e[w] > s3[w]):.1f}",
                         f"{np.sqrt(np.mean(ev[w] ** 2)):.2f}"])
            print(" | ".join(rows[-1]))
            d["e"], d["s3"] = e, s3
            if v == "no altimeter":
                base["noalt"] = d
        if fl != flights[0]:
            continue
        grid = np.arange(np.ceil(base["st"][0, 0]), base["st"][-1, 0], 1.0)
        cols, head = [grid - t_ref0], ["t_s"]
        for key, d in (("flown", base), ("noalt", base.get("noalt"))):
            if d is not None:
                cols += [np.interp(grid, d["st"][:, 0], d["e"]), np.interp(grid, d["st"][:, 0], d["s3"])]
                head += [f"err_{key}_m", f"sigma3_{key}_m"]
        write_csv(out / "timeline.csv", head, [[f"{x:.1f}" for x in r] for r in np.column_stack(cols)])
        off = base["agl"][(base["agl"][:, 0] > t1) & (base["agl"][:, 1] == 4), 0]
        land = base["lp"][np.hypot(base["lp"][:, 3], base["lp"][:, 4]) > 0.5, 0][-1]
        marks = [["airborne", t0], ["estimate leaves the map", off[0] if len(off) else t1], ["landed", land]]
        write_csv(out / "marks.csv", ["event", "t_s"], [[m, f"{t - t_ref0:.1f}"] for m, t in marks])
    write_csv(out / "scores.csv", ["flight", "variant", "settings_changed", "window_s", "rms_m", "max_m",
                                   "over_3sigma_pct", "vel_rms_m_s"], rows)
    eph_rows = []
    for label, path in (("GPS on (Stage A, gdn_20261116_084914)", kept), ("GPS cut (Stage B, gdn_20261116_112059)", cut)):
        if path is None:
            continue
        e = read(path)["eph"]
        t = e[:, 0] - e[0, 0]
        keep = np.r_[True, np.diff(np.floor(t)) > 0]        # 1 Hz
        eph_rows += [[label, f"{a:.0f}", f"{b:.2f}", f"{c:.0f}"] for a, b, c in zip(t[keep], e[keep, 1], e[keep, 2])]
    if eph_rows:
        write_csv(out / "px4_eph.csv", ["run", "t_s", "eph_m", "xy_valid"], eph_rows)
    print(f"exported to {out}")


def style(ax):
    ax.set_facecolor(SURF)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


def draw(data, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rd = lambda n: list(csv.DictReader(open(data / n)))
    tl, sc, mk = rd("timeline.csv"), rd("scores.csv"), rd("marks.csv")
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "text.color": INK})
    fig, ax = plt.subplots(2, 2, figsize=(13, 8.6), facecolor=SURF,
                           gridspec_kw={"width_ratios": [1.35, 1], "height_ratios": [1, 1.05]})
    for a in ax.flat:
        style(a)

    # A: one flight's error against its own 3 sigma.
    a = ax[0, 0]
    col = lambda k: np.array([float(r[k]) for r in tl])
    t = col("t_s")
    flight0 = sc[0]["flight"]
    for key, c, name in (("flown", C2, "as flown"), ("noalt", C1, "no altimeter (replay)")):
        if f"err_{key}_m" in tl[0]:
            a.semilogy(t, np.maximum(col(f"err_{key}_m"), 1), color=c, lw=2, label=f"error, {name}")
            a.semilogy(t, np.maximum(col(f"sigma3_{key}_m"), 1), color=c, lw=1.2, ls="--", label="its 3σ")
    for r in mk:
        x = float(r["t_s"])
        a.axvline(x, color=GREY, lw=1)
        a.text(x + 6, 1.3, r["event"], rotation=90, va="bottom", fontsize=7.5, color=INK2)
    a.set_xlim(t[0], t[-1])
    a.set_ylim(1, 3e4)
    a.set_xlabel("time since PX4 start, s")
    a.set_ylabel("horizontal error against PX4, m")
    a.set_title(f"A. Flight {flight0}: GDN's error and its own 3σ", loc="left", fontsize=10)
    a.legend(loc="upper left", frameon=False, fontsize=8, ncol=2)

    # B: PX4's horizontal uncertainty, GPS on and GPS cut.
    a = ax[0, 1]
    if (data / "px4_eph.csv").exists():
        ep = rd("px4_eph.csv")
        for (name, c) in zip(dict.fromkeys(r["run"] for r in ep), (C1, C2)):
            rr = [r for r in ep if r["run"] == name and float(r["eph_m"]) > 0]   # 0 = not set (disarmed)
            tt, ee = np.array([float(r["t_s"]) for r in rr]), np.array([float(r["eph_m"]) for r in rr])
            if "cut" in name:            # ends at landing: PX4 resets eph once disarmed
                tt, ee = tt[:np.argmax(ee) + 1], ee[:np.argmax(ee) + 1]
            a.semilogy(tt, ee, color=c, lw=2, label=name)
            if "cut" in name and (ee > 0.5).any():
                x = tt[np.argmax(ee > 0.5)]
                a.axvline(x, color=GREY, lw=1)
                a.text(x + 12, 0.12, "the cut takes effect", rotation=90, va="bottom", fontsize=7.5, color=INK2)
        a.legend(loc="upper right", frameon=False, fontsize=8)
    a.set_ylim(0.1, 2e3)
    a.set_xlabel("time since PX4 start, s")
    a.set_ylabel("PX4's eph, m")
    a.set_title("B. PX4's own uncertainty: the GPS cut now works", loc="left", fontsize=10)

    # C, D: every variant on every flight.
    flights = list(dict.fromkeys(r["flight"] for r in sc))
    variants = list(dict.fromkeys(r["variant"] for r in sc))[::-1]
    y = {v: i for i, v in enumerate(variants)}
    for a, key, xlabel, title, logx in (
            (ax[1, 0], "rms_m", "RMS horizontal error in flight, m", "C. Accuracy: no setting wins on every flight",
             True),
            (ax[1, 1], "over_3sigma_pct", "time with error over its own 3σ, %",
             "D. Honesty: without the altimeter, never over 3σ", False)):
        for k, fl in enumerate(flights):
            rr = [r for r in sc if r["flight"] == fl]
            a.scatter([float(r[key]) for r in rr], [y[r["variant"]] + (k - 1) * 0.18 for r in rr], s=36,
                      marker=MARK[k % 3], color=(C1, C2, C3)[k % 3], label=f"flight {fl}", zorder=3,
                      edgecolor=SURF, linewidth=0.8)
        a.set_yticks(range(len(variants)), variants)
        a.set_xlabel(xlabel)
        a.set_title(title, loc="left", fontsize=10)
        a.grid(axis="y", visible=False)
        if logx:
            a.set_xscale("log")
        else:
            a.set_xlim(-3, 100)
            a.set_yticklabels([])
        for i in range(len(variants)):
            a.axhline(i, color=GRID, lw=0.5, zorder=1)
    ax[1, 0].legend(loc="upper left", frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=130, facecolor=SURF)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("src", nargs="+", help='"FLIGHT:VARIANT=RUN_LOG" runs (with --export), or an exported data dir')
    ap.add_argument("--gps-kept", help="a Stage A run log, for PX4's eph with GPS on")
    ap.add_argument("--gps-cut", help="a Stage B run log, for PX4's eph after the cut")
    ap.add_argument("--export", type=Path, help="write the CSVs here")
    ap.add_argument("--out", type=Path, help="write the figure here")
    a = ap.parse_args()
    if a.export:
        runs = []
        for s in a.src:
            head, path = s.split("=", 1)
            fl, v = head.split(":", 1)
            runs.append((fl, v, path))
        export(runs, a.gps_kept, a.gps_cut, a.export)
    if a.out:
        draw(a.export or Path(a.src[0]), a.out)
    if not (a.export or a.out):
        sys.exit("give --export and/or --out")


if __name__ == "__main__":
    main()
