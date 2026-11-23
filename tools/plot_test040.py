#!/usr/bin/env python3
"""TEST-040 figure: scenarios and the runner (R2.6a).

    # from the runner's run directories (needs ROS: the dev image), once:
    python3 tools/plot_test040.py --export docs/test_reports/test040_data log/r26a/final
    # from the exported files (what's committed):
    python3 tools/plot_test040.py docs/test_reports/test040_data --out docs/test_reports/test040_scenarios.png

Export, from each scenario's latest run: result.json and check.txt; PX4's horizontal
position validity and uncertainty (the GNSS cut); GDN's horizontal error against the truth
(the start error); the lean or crab towards upwind on steady route flight, per sample
(the wind, and the no-wind runs for comparison).
"""
import argparse
import csv
import json
import math
import re
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN = "#2a78d6", "#eb6834", "#1f9e6a"
SCEN = ("x500_emmental_smoke", "x500_emmental_gnss_cut", "x500_emmental_wind", "standard_vtol_emmental_wind",
        "rc_cessna_emmental_start_error")


def latest(batch, name):
    runs = sorted((Path(batch) / name).glob("*/"))
    return runs[-1] if runs else None


def export(out, batch):
    import check_gps_cut as gc
    import check_scenario as c
    import check_sim_sensors as cs
    import scenario as scn
    import sim_vehicles
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for name in SCEN:
        run = latest(batch, name)
        if run is None:
            print(f"no run of {name}")
            continue
        for f in ("result.json", "check.txt", "scenario.yaml"):
            if (run / f).exists():
                shutil.copy(run / f, out / f"{name}_{f.replace('.', '_', f.count('.') - 1)}")
        sc = scn.parse(__import__("yaml").safe_load((run / "scenario.yaml").read_text()))
        kind = sim_vehicles.get(sc["vehicle"]).kind
        t0, t1 = c.fly_times(run / "fly.log")
        truth = c.read_truth(run / "sitl_bag")
        res = json.loads((run / "result.json").read_text())
        if sc["gnss"]["loss"] == "cut":
            lp = gc.read_lpos(Path(res["run_log"]))
            route0 = next(float(m.group(1)) for line in (run / "fly.log").read_text().splitlines()
                          if (m := re.match(r"\[\w+\] \[(\d+\.\d+)\]", line)) and "starting the route" in line)
            with open(out / f"{name}_px4_eph.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_route_s", "xy_valid", "eph_m"])
                for t, _z, _zv, xyv, eph in lp[::5]:
                    w.writerow([f"{t - route0:.2f}", int(xyv), f"{eph:.2f}"])
        if sc["start_error"]["distance_m"] > 0:
            odom = cs.read_bag(run / "sitl_bag", ["/gdn/odom"])[1]["/gdn/odom"]
            g = np.array([(t, m.pose.pose.position.x, m.pose.pose.position.y) for t, m in odom[::10]])
            tn = np.interp(g[:, 0], truth[:, 0], truth[:, 2])
            te = np.interp(g[:, 0], truth[:, 0], truth[:, 1])
            err = np.hypot(g[:, 1] - tn, g[:, 2] - te)
            m0 = re.search(r"\[(\d+\.\d+)\] \[gdn_eskf_node\]: start error", (run / "gdn.log").read_text())
            ts = float(m0.group(1)) if m0 else g[0, 0]
            np.savetxt(out / f"{name}_gdn_error.csv", np.column_stack([g[:, 0] - ts, err]), fmt="%.2f", delimiter=",",
                       header="t_from_start_error_s,horizontal_error_m", comments="")
        if t0 and t1:
            # Per steady sample: the lean (multirotor) or crab (plane) towards upwind (from 270 deg).
            tr = truth[(truth[:, 0] >= t0) & (truth[:, 0] <= t1)]
            tg = tr[:, 9]
            keep = np.concatenate([[True], np.diff(tg) > 0])
            tr, tg = tr[keep], tg[keep]
            sm = (lambda x: np.convolve(x, np.ones(21) / 21, mode="same"))
            vn, ve = np.gradient(sm(tr[:, 2]), tg), np.gradient(sm(tr[:, 1]), tg)
            an, ae = np.gradient(sm(vn), tg), np.gradient(sm(ve), tg)
            steady = np.hypot(an, ae) < 0.5
            steady[:15] = steady[-15:] = False
            up = c.upwind_unit(270)
            if kind in ("fixedwing", "vtol"):
                v = np.stack([vn, ve], 1)
                sp = np.linalg.norm(v, axis=1)
                h = np.array([c.heading_ne(q) for q in tr[:, 4:8]])
                h /= np.maximum(np.linalg.norm(h, axis=1, keepdims=True), 1e-9)
                use = steady & (sp > 5)
                track = v[use] / sp[use, None]
                side = h[use] - track * np.sum(h[use] * track, axis=1, keepdims=True)
                vals = np.degrees(np.arcsin(np.clip(side @ up, -1, 1)))
            else:
                tilt = np.array([c.thrust_tilt_ne(q) for q in tr[:, 4:8]]) - np.stack([an, ae], 1) / 9.81
                vals = np.degrees(np.arcsin(np.clip(tilt[steady] @ up, -1, 1)))
            np.savetxt(out / f"{name}_upwind_deg.csv", vals, fmt="%.3f", header="upwind_deg", comments="")
        print(f"exported {name} ({run.name})")


def draw(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    d = Path(d)
    fig = plt.figure(figsize=(15, 12.5))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.15], hspace=0.45, wspace=0.28, top=0.93, bottom=0.03)
    # GNSS cut.
    ax = fig.add_subplot(gs[0, 0])
    p = d / "x500_emmental_gnss_cut_px4_eph.csv"
    if p.exists():
        e = np.genfromtxt(p, delimiter=",", names=True)
        ax.plot(e["t_route_s"], e["eph_m"], color=BLUE, lw=1.3, label="PX4's horizontal uncertainty (eph)")
        inval = e["t_route_s"][e["xy_valid"] == 0]
        if len(inval):
            ax.axvspan(inval.min(), inval.max(), color=ORANGE, alpha=0.15, lw=0, label="PX4's position invalid")
        ax.axvline(60, color=INK2, lw=1, ls=(0, (3, 2)), label="GNSS cut asked (60 s)")
        ax.set_yscale("log")
        ax.set_xlim(-5, e["t_route_s"].max())
    ax.set_xlabel("time into the route (s)")
    ax.set_ylabel("m")
    ax.set_title("GNSS cut (x500): PX4 loses its position")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Start error.
    ax = fig.add_subplot(gs[0, 1])
    p = d / "rc_cessna_emmental_start_error_gdn_error.csv"
    if p.exists():
        e = np.genfromtxt(p, delimiter=",", names=True)
        ax.plot(e["t_from_start_error_s"], e["horizontal_error_m"], color=BLUE, lw=1.3, label="GDN's horizontal error")
        ax.axhline(150, color=INK2, lw=1, ls=(0, (3, 2)), label="start error asked (150 m)")
        ax.set_xlim(-5, min(400, e["t_from_start_error_s"].max()))
    ax.set_xlabel("time since GDN was told its start error (s)")
    ax.set_ylabel("m")
    ax.set_title("Start error (cessna): GDN starts 150 m off")
    ax.legend(loc="upper right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Wind: lean and crab towards upwind, per steady sample.
    for col, (title, pairs, unit) in enumerate((
            ("Wind (x500): lean towards upwind", (("x500_emmental_wind", "6 m/s from the west", BLUE),
                                                   ("x500_emmental_smoke", "no wind", MUTED)), "lean"),
            ("Wind (planes): crab towards upwind", (("standard_vtol_emmental_wind", "VTOL in cruise, 6 m/s from the west",
                                                     BLUE),
                                                    ("rc_cessna_emmental_start_error", "cessna, no wind", MUTED)), "crab"))):
        ax = fig.add_subplot(gs[1, col])
        for name, lab, colr in pairs:
            p = d / f"{name}_upwind_deg.csv"
            if not p.exists():
                continue
            v = np.atleast_1d(np.loadtxt(p, skiprows=1))
            ax.hist(v, bins=np.linspace(-15 if unit == "lean" else -10, 15 if unit == "lean" else 35, 81), density=True,
                    histtype="stepfilled" if colr == MUTED else "step", color=colr, alpha=0.35 if colr == MUTED else 1,
                    lw=1.8, label=f"{lab}: mean {v.mean():+.2f} deg")
        ax.axvline(0, color=INK2, lw=0.6)
        ax.set_xlabel(f"{unit} towards upwind, per steady sample (deg)")
        ax.set_ylabel("density")
        ax.set_title(title)
        ax.legend(loc="upper left", fontsize=7.5)
        ax.grid(color=GRID, lw=0.5)
    # Runner timeline: duration per scenario.
    ax = fig.add_subplot(gs[0:2, 2])
    names, durs, mbs = [], [], []
    for name in SCEN:
        p = d / f"{name}_result.json"
        if p.exists():
            r = json.loads(p.read_text())
            names.append(name.replace("_emmental", "").replace("_", " "))
            durs.append(r["duration_s"] / 60)
            mbs.append(r["bag_mb"] / 1000)
    y = np.arange(len(names))
    ax.barh(y, durs, color=BLUE, height=0.55)
    for yy, dd, mb in zip(y, durs, mbs):
        ax.text(dd + 0.1, yy, f"{dd:.1f} min, {mb:.1f} GB", va="center", fontsize=8, color=INK2)
    ax.set_yticks(y)
    ax.set_yticklabels(names)
    ax.invert_yaxis()
    ax.set_xlim(0, max(durs + [1]) * 1.55)
    ax.set_xlabel("start to teardown (min)")
    ax.set_title("The runner: each scenario, headless")
    ax.grid(color=GRID, lw=0.5, axis="x")
    # Results table.
    ax = fig.add_subplot(gs[2, :])
    ax.axis("off")
    rows = []
    for name in SCEN:
        p, c = d / f"{name}_result.json", d / f"{name}_check.txt"
        if not p.exists():
            continue
        r = json.loads(p.read_text())
        chk = c.read_text() if c.exists() else ""
        n_pass, n_fail = chk.count("\nPASS") + chk.startswith("PASS"), chk.count("\nFAIL") + chk.startswith("FAIL")
        key = next((l.split(": ", 1)[1] for l in chk.splitlines() if l.startswith("PASS") and any(
            k in l for k in ("lost GPS as asked", "GDN started 150", "into the wind", "no steady lean"))), "")
        rows.append([name, "PASSED" if r.get("passed") else "FAILED", f"{n_pass} pass, {n_fail} fail",
                     "yes" if r.get("teardown_clean") else "NO", str(r.get("replay")), key[:70]])
    tb = ax.table(cellText=rows, colLabels=["scenario", "verdict", "scenario checks", "torn down clean",
                                            "run log replay", "what it asked, found"],
                  cellLoc="left", loc="upper center", colWidths=[0.2, 0.07, 0.11, 0.09, 0.09, 0.44])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.5)
    for (rr, cc), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        if rr == 0:
            cell.get_text().set_fontweight("bold")
        if cc == 1 and rr > 0:
            cell.get_text().set_color(GREEN if cell.get_text().get_text() == "PASSED" else "#b3261e")
    ax.set_title("tools/run_scenario.py, then tools/check_scenario.py, on each scenario (flown fresh)", pad=6)
    fig.suptitle("TEST-040: scenarios and the runner (R2.6a): GNSS cut, start error and wind, flown headless",
                 x=0.04, ha="left", fontsize=13, fontweight="bold", color=INK)
    fig.savefig(out, dpi=110, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test040_data")
    ap.add_argument("--export", metavar="BATCH_DIR", help="export the runs in this --out directory into DATA")
    ap.add_argument("--out", default="docs/test_reports/test040_scenarios.png")
    a = ap.parse_args()
    if a.export:
        export(a.data, a.export)
    else:
        draw(a.data, a.out)


if __name__ == "__main__":
    main()
