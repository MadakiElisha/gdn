#!/usr/bin/env python3
"""TEST-041 figure: GNSS spoofing, sensor faults and lighting (R2.6b).

    # from the runner's run directories (needs ROS: the dev image), once:
    python3 tools/plot_test041.py --export docs/test_reports/test041_data log/r26b/final
    # from the exported files (what's committed):
    python3 tools/plot_test041.py docs/test_reports/test041_data --out docs/test_reports/test041_faults_spoofing.png

Export, from each scenario's latest run: result.json and check.txt for all ten; the
spoof (PX4's position minus the truth, frames aligned before the spoof; the asked offset;
GDN's own error; the tracks); each fault's input series as GDN received it (run log),
around the fault; the OAK-D Lite's scene light and image brightness (the dusk run and the
smoke run). And gdn_response.txt: what GDN did with each faulted sensor before, during
and after its fault (its update decisions and events from the run log), and its whole-run
error from the scorer.
"""
import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN, RED = "#2a78d6", "#eb6834", "#1f9e6a", "#b3261e"
NEW = ("x500_emmental_spoof", "x500_emmental_faults", "standard_vtol_emmental_faults", "rc_cessna_emmental_faults",
       "x500_emmental_dusk")
OLD = ("x500_emmental_smoke", "x500_emmental_gnss_cut", "x500_emmental_wind", "standard_vtol_emmental_wind",
       "rc_cessna_emmental_start_error")
FAULTED = ("x500_emmental_faults", "standard_vtol_emmental_faults", "rc_cessna_emmental_faults")
REASONS = {0: "applied", 7: "gated", 4: "off map", 5: "flat", 6: "weak field", 9: "bad input", 11: "not used"}
UPDATE_SENSOR = {"agl": (2,), "baro": (3,), "mag": (4, 5)}
UNITS = {"agl": "AGL (m)", "baro": "baro, NED z (m)", "mag": "mag x (uT)", "imu": "accel x (m/s^2)"}


def latest(batch, name):
    runs = sorted((Path(batch) / name).glob("*/"))
    return runs[-1] if runs else None


def gdn_response(run, sc, runlog, t0):
    """Lines: per fault, GDN's decisions on that sensor before, during and after it."""
    import re
    import check_sim_sensors as cs
    import sim_faults
    d = cs.read_bag(runlog, ["/gdn/log/update", "/gdn/log/event"])[1]
    score = re.findall(r"RMS whole run: .*", (run / "scorer.log").read_text()) if (run / "scorer.log").exists() else []
    lines = [f"== {sc['name']}: GDN's horizontal error over the whole run: {score[-1] if score else '?'}"]
    for f in sc["faults"]:
        a, b, dur = t0 + f["at_s"], t0 + f["at_s"] + f["duration_s"], f["duration_s"]
        for lab, lo, hi in (("before", a - dur, a), ("during", a, b), ("after", b, b + dur), ("after, to the end", b, 1e12)):
            parts = []
            if f["sensor"] in UPDATE_SENSOR:
                cnt = {}
                for t, m in d["/gdn/log/update"]:
                    if lo <= t < hi and m.sensor in UPDATE_SENSOR[f["sensor"]]:
                        cnt[m.reason] = cnt.get(m.reason, 0) + 1
                parts.append("updates " + (", ".join(f"{n} {REASONS.get(r, r)}" for r, n in sorted(cnt.items()))
                                           or "none"))
            ev = sorted({re.sub(r" \(count \d+\)", "", m.text) for t, m in d["/gdn/log/event"] if lo <= t < hi
                         and "IMU" in m.text and "airborne latch" not in m.text})
            if ev or f["sensor"] == "imu":
                parts.append("IMU events: " + ("; ".join(ev) if ev else "none"))
            lines.append(f"  {sim_faults.Fault(**f).text():52s} {lab:18s} {' | '.join(parts)}")
    return lines


def export(out, batch):
    import check_scenario as c
    import check_sim_sensors as cs
    import scenario as scn
    import sim_faults
    import sim_spoof
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    response = []
    for name in NEW + OLD:
        run = latest(batch, name)
        if run is None:
            print(f"no run of {name}")
            continue
        for f in ("result.json", "check.txt", "scenario.yaml"):
            if (run / f).exists():
                shutil.copy(run / f, out / f"{name}_{f.replace('.', '_', f.count('.') - 1)}")
        sc = scn.parse(__import__("yaml").safe_load((run / "scenario.yaml").read_text()))
        res = json.loads((run / "result.json").read_text())
        runlog = Path(res["run_log"])
        t0 = c.route_start_time(run)
        t_land = c.landed_time(run / "fly.log")
        if sc["gnss"]["loss"] == "spoof":
            g = sc["gnss"]
            truth = c.read_truth(run / "sitl_bag")
            t_l, xy, valid, eph = c.read_lpos_xy(runlog)
            off = c.spoof_offsets(t_l, xy, truth, t0 + 2.0, t0 + g["at_s"] - 1.0)
            odom = cs.read_bag(run / "sitl_bag", ["/gdn/odom"])[1]["/gdn/odom"]
            gt = np.array([t for t, _ in odom])
            gxy = np.array([(m.pose.pose.position.x, m.pose.pose.position.y) for _, m in odom])
            tn, te = np.interp(t_l, truth[:, 0], truth[:, 2]), np.interp(t_l, truth[:, 0], truth[:, 1])
            gdn_err = np.hypot(np.interp(t_l, gt, gxy[:, 0]) - tn, np.interp(t_l, gt, gxy[:, 1]) - te)
            frame = (xy - np.stack([tn, te], 1)) - off           # the constant frame offset
            keep = (t_l >= t0 - 30) & (t_l <= (t_land or t_l[-1]) + 5)
            with open(out / f"{name}_spoof.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_route_s", "px4_off_n_m", "px4_off_e_m", "asked_n_m", "asked_e_m", "gdn_error_m",
                            "truth_n_m", "truth_e_m", "px4_n_m", "px4_e_m", "xy_valid", "eph_m"])
                for k in np.flatnonzero(keep)[::5]:
                    an, ae = sim_spoof.offset_ne(t_l[k] - t0, g["at_s"], g["rate_mps"], g["bearing_deg"])
                    w.writerow([f"{t_l[k] - t0:.2f}", f"{off[k, 0]:.2f}", f"{off[k, 1]:.2f}", f"{an:.2f}", f"{ae:.2f}",
                                f"{gdn_err[k]:.1f}", f"{tn[k]:.1f}", f"{te[k]:.1f}", f"{xy[k, 0] - frame[k, 0]:.1f}",
                                f"{xy[k, 1] - frame[k, 1]:.1f}", int(valid[k]), f"{eph[k]:.2f}"])
            shutil.copy(run / "route.csv", out / f"{name}_route.csv")
        if sc["faults"] or name == "x500_emmental_smoke":
            response += gdn_response(run, sc, runlog, t0)
        if sc["faults"]:
            ev = cs.read_bag(run / "sitl_bag", ["/gdn/sim/fault"])[1]["/gdn/sim/fault"]
            events = [(t, m.data) for t, m in ev]
            series = cs.read_bag(runlog, sorted(set(c.FAULT_TOPICS.values())))[1]
            for f in sc["faults"]:
                fl = sim_faults.Fault(**f)
                on = next((t for t, d in events if d == f"on {fl.text()}"), t0 + f["at_s"])
                off_t = next((t for t, d in events if d == f"off {fl.text()}"), on + f["duration_s"])
                topic = c.FAULT_TOPICS[f["sensor"]]
                msgs = series[topic]
                t = np.array([x for x, _ in msgs])
                v = c.fault_values(topic, msgs).reshape(len(t), -1)[:, 0]
                pad = max(10.0, f["duration_s"])
                w = (t >= on - pad) & (t <= off_t + pad)
                step = max(1, int(w.sum() / 3000))
                np.savetxt(out / f"{name}_fault_{f['sensor']}.csv",
                           np.column_stack([t[w][::step] - on, v[w][::step]]), fmt="%.4f", delimiter=",",
                           header=f"t_from_fault_s,value,{f['kind']},{off_t - on:.3f},{f['sigma']:g}", comments="")
        if name in ("x500_emmental_dusk", "x500_emmental_smoke"):
            got = cs.read_bag(run / "sitl_bag", ["/gdn/sim/scene_light", "/gdn/sim/image_brightness"])[1]
            sl = got["/gdn/sim/scene_light"]
            bt = np.array([t for t, _ in got["/gdn/sim/image_brightness"]])
            bv = np.array([m.data for _, m in got["/gdn/sim/image_brightness"]])
            np.savetxt(out / f"{name}_light.csv",
                       np.array([(t - t0, m.data, np.interp(t, bt, bv)) for t, m in sl]), fmt="%.5f",
                       delimiter=",", header="t_route_s,scene_light,image_brightness", comments="")
        print(f"exported {name} ({run.name})")
    (out / "gdn_response.txt").write_text("\n".join(response) + "\n")


def draw(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    d = Path(d)
    fig = plt.figure(figsize=(16, 20))
    gs = fig.add_gridspec(5, 4, height_ratios=[1.15, 0.8, 0.8, 0.8, 1.25], hspace=0.55, wspace=0.3, top=0.95,
                          bottom=0.02, left=0.05, right=0.98)
    # The spoof: offset against time.
    p = d / "x500_emmental_spoof_spoof.csv"
    ax = fig.add_subplot(gs[0, 0:2])
    if p.exists():
        e = np.genfromtxt(p, delimiter=",", names=True)
        ax.plot(e["t_route_s"], np.hypot(e["px4_off_n_m"], e["px4_off_e_m"]), color=ORANGE, lw=2.4,
                label="PX4's position minus the truth (its eph stayed under 1 m: it never noticed)")
        ax.plot(e["t_route_s"], np.hypot(e["asked_n_m"], e["asked_e_m"]), color=INK, lw=1.1, ls=(0, (3, 2)),
                label="spoof asked: 0.5 m/s east from 60 s")
        ax.plot(e["t_route_s"], e["gdn_error_m"], color=BLUE, lw=1.0, alpha=0.8,
                label="GDN's own error (uses no GNSS: not fooled; the x500's OI-012 drift is its own)")
        ax.set_yscale("log")
        ax.set_ylim(0.3, 5000)
        ax.set_xlim(e["t_route_s"].min(), e["t_route_s"].max())
    ax.axvline(60, color=MUTED, lw=0.8)
    ax.set_xlabel("time into the route (s)")
    ax.set_ylabel("horizontal distance (m, log)")
    ax.set_title("GNSS spoofing (x500): PX4's position walks off as asked")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(color=GRID, lw=0.5)
    # The spoof: tracks.
    ax = fig.add_subplot(gs[0, 2])
    if p.exists():
        r = np.genfromtxt(d / "x500_emmental_spoof_route.csv", delimiter=",", names=True)
        ax.plot(r["east_m"], r["north_m"], color=GRID, lw=6, solid_capstyle="round", label="the route (PX4's frame)")
        ax.plot(e["px4_e_m"], e["px4_n_m"], color=ORANGE, lw=1.4, label="where PX4 thinks it is")
        ax.plot(e["truth_e_m"], e["truth_n_m"], color=INK, lw=1.4, label="where it really is (truth)")
        ax.set_xlim(-350, 250)
        ax.set_ylim(-470, 720)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Spoofed: PX4 flies its route,\nthe x500 itself drifts west")
    ax.legend(loc="upper center", fontsize=7, ncol=1)
    ax.grid(color=GRID, lw=0.5)
    # Lighting.
    ax = fig.add_subplot(gs[0, 3])
    for name, lab, colr in (("x500_emmental_smoke", "the world's own sun, 61 deg up", MUTED),
                            ("x500_emmental_dusk", "dusk: 8 deg up, x0.3", BLUE)):
        q = d / f"{name}_light.csv"
        if q.exists():
            b = np.atleast_2d(np.genfromtxt(q, delimiter=",", skip_header=1))
            ax.plot(b[:, 0], b[:, 1], color=colr, lw=1.4,
                    label=f"{lab}: light {b[:, 1].mean():.3f};\n  image sent {b[:, 2].mean():.0f} of 255 (auto-exposure)")
    ax.set_ylim(bottom=0)
    ax.set_xlabel("time into the route (s)")
    ax.set_ylabel("scene light (raw linear image mean)")
    ax.set_title("Lighting (x500):\nthe light the OAK-D Lite sees")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Faults: one row per vehicle, one column per sensor.
    for row, name in enumerate(FAULTED):
        for col, sensor in enumerate(("agl", "baro", "mag", "imu")):
            ax = fig.add_subplot(gs[1 + row, col])
            q = d / f"{name}_fault_{sensor}.csv"
            if not q.exists():
                ax.axis("off")
                continue
            hdr = q.read_text().splitlines()[0].split(",")
            kind, dur, sigma = hdr[2], float(hdr[3]), float(hdr[4])
            v = np.atleast_2d(np.genfromtxt(q, delimiter=",", skip_header=1))
            gap = np.flatnonzero(np.diff(v[:, 0]) > 5 * np.median(np.diff(v[:, 0])) + 0.2)
            v = np.insert(v, gap + 1, np.nan, axis=0)       # a dropout drawn as a gap, not a line
            ax.axvspan(0, dur, color=ORANGE, alpha=0.13, lw=0)
            ax.plot(v[:, 0], v[:, 1], color=BLUE, lw=0.6 if sensor in ("imu", "mag") else 1.0,
                    marker="." if sensor == "agl" else None, ms=2.5)
            veh = name.split("_emmental")[0].replace("_", " ")
            what = kind + (f" (sigma {sigma:g})" if kind == "noise" else "")
            ax.set_title(f"{veh}: {sensor} {what}", fontsize=9)
            ax.set_xlabel("time from the fault's start (s)", fontsize=8)
            ax.set_ylabel(UNITS[sensor], fontsize=8)
            ax.grid(color=GRID, lw=0.5)
    # Results table: the five new scenarios, then R2.6a's five flown again.
    ax = fig.add_subplot(gs[4, :])
    ax.axis("off")
    rows = []
    keys = ("walked off", "PX4 lost GPS as asked", "the OAK-D Lite saw", "GDN started 150", "into the wind",
            "no steady lean", "no steady crab")
    for name in NEW + OLD:
        rp, cp = d / f"{name}_result.json", d / f"{name}_check.txt"
        if not rp.exists():
            continue
        r = json.loads(rp.read_text())
        chk = cp.read_text().splitlines() if cp.exists() else []
        n_pass = sum(l.startswith("PASS") for l in chk)
        n_fail = sum(l.startswith("FAIL") for l in chk)
        faults = [l for l in chk if l.startswith("PASS") and "in GDN's inputs" in l]
        found = [l.split(": ", 2)[-1] for k in keys for l in chk if l.startswith("PASS") and k in l]
        key = f"{len(faults)} faults found in GDN's inputs" if faults else (found[0] if found else "")
        rows.append([name + ("" if name in NEW else " (R2.6a)"), "PASSED" if r.get("passed") else "FAILED",
                     f"{n_pass} pass, {n_fail} fail", f"{r['duration_s'] / 60:.1f} min",
                     "yes" if r.get("teardown_clean") else "NO", str(r.get("replay")), key.split(";")[0][:78]])
    tb = ax.table(cellText=rows, colLabels=["scenario", "verdict", "scenario checks", "run", "torn down clean",
                                            "run log replay", "what it asked, found"],
                  cellLoc="left", loc="upper center", colWidths=[0.22, 0.06, 0.09, 0.05, 0.08, 0.08, 0.42])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.45)
    for (rr, cc), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        if rr == 0:
            cell.get_text().set_fontweight("bold")
        if cc == 1 and rr > 0:
            cell.get_text().set_color(GREEN if cell.get_text().get_text() == "PASSED" else RED)
    ax.set_title("tools/run_scenario.py, then tools/check_scenario.py, on all ten scenarios (flown fresh)", pad=6)
    fig.text(0.05, 0.765, "Sensor faults on GDN's inputs, as GDN received them (its run log); the fault shaded",
             fontsize=10, fontweight="bold", color=INK)
    fig.suptitle("TEST-041: GNSS spoofing, sensor faults and lighting (R2.6b), flown headless",
                 x=0.05, ha="left", fontsize=13, fontweight="bold", color=INK)
    fig.savefig(out, dpi=100, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test041_data")
    ap.add_argument("--export", metavar="BATCH_DIR", help="export the runs in this --out directory into DATA")
    ap.add_argument("--out", default="docs/test_reports/test041_faults_spoofing.png")
    a = ap.parse_args()
    if a.export:
        export(a.data, a.export)
    else:
        draw(a.data, a.out)


if __name__ == "__main__":
    main()
