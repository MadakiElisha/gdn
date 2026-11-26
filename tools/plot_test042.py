#!/usr/bin/env python3
"""TEST-042 figure: every scenario scored (R2.7a).

    # from the runner's run directories (needs ROS: the dev image), once:
    python3 tools/plot_test042.py --export docs/test_reports/test042_data log/r27a/final
    # from the exported files (what's committed):
    python3 tools/plot_test042.py docs/test_reports/test042_data --out docs/test_reports/test042_scores.png

Export, from each scenario's latest run: metrics.json and report.md (tools/score_run.py),
result.json, and GDN's horizontal error and 3 sigma at 1 Hz from the route's start.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN, RED, PURPLE = "#2a78d6", "#eb6834", "#1f9e6a", "#b3261e", "#7b4fc9"
SCEN = ("x500_emmental_smoke", "x500_emmental_gnss_cut", "x500_emmental_wind", "x500_emmental_spoof",
        "x500_emmental_faults", "x500_emmental_dusk", "standard_vtol_emmental_wind", "standard_vtol_emmental_faults",
        "rc_cessna_emmental_start_error", "rc_cessna_emmental_faults")
VCOL = {"x500": BLUE, "standard_vtol": PURPLE, "rc_cessna": ORANGE}


def latest(batch, name):
    runs = sorted((Path(batch) / name).glob("*/"))
    return runs[-1] if runs else None


def export(out, batch):
    import score_run as sr
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for name in SCEN:
        run = latest(batch, name)
        if run is None or not (run / "metrics.json").exists():
            print(f"no scored run of {name}")
            continue
        for f in ("metrics.json", "report.md", "result.json", "cpu.csv"):
            if (run / f).exists():
                shutil.copy(run / f, out / f"{name}_{f.replace('.', '_', f.count('.') - 1)}")
        d = sr.read_run(run)
        _m, s = sr.score(d)
        t, err, sig = s["t"], s["err"], s["sig"]
        keep = np.concatenate([[True], np.diff(np.floor(t)) > 0])       # 1 Hz
        np.savetxt(out / f"{name}_error.csv", np.column_stack([t[keep], err[keep], 3 * sig[keep]]), fmt="%.2f",
                   delimiter=",", header="t_route_s,horizontal_error_m,three_sigma_m", comments="")
        print(f"exported {name} ({run.name})")


def draw(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    d = Path(d)
    ms = {n: json.loads((d / f"{n}_metrics.json").read_text()) for n in SCEN if (d / f"{n}_metrics.json").exists()}
    names = list(ms)
    short = [n.replace("_emmental", "").replace("standard_vtol", "vtol").replace("rc_cessna", "cessna").replace("_", " ")
             for n in names]
    fig = plt.figure(figsize=(16, 17))
    gs = fig.add_gridspec(3, 3, height_ratios=[1, 1, 1.05], hspace=0.42, wspace=0.32, top=0.94, bottom=0.03,
                          left=0.09, right=0.98)
    # Error against time, every run.
    ax = fig.add_subplot(gs[0, 0:2])
    for n in names:
        p = d / f"{n}_error.csv"
        if p.exists():
            e = np.genfromtxt(p, delimiter=",", names=True)
            sel = (e["t_route_s"] >= 0) & (e["t_route_s"] <= ms[n]["route_s"])
            ax.plot(e["t_route_s"][sel], e["horizontal_error_m"][sel], color=VCOL[ms[n]["vehicle"]], lw=1.1,
                    alpha=0.85, ls="-" if "faults" not in n else (0, (4, 1.5)))
    for v, c in VCOL.items():
        ax.plot([], [], color=c, lw=1.5, label=v)
    ax.plot([], [], color=MUTED, lw=1.2, ls=(0, (4, 1.5)), label="with sensor faults (dashed)")
    for lim, lab in ((30, "x500 max 30 m"), (80, "VTOL max 80 m"), (100, "cessna max 100 m")):
        ax.axhline(lim, color=GRID, lw=1)
        ax.text(415, lim * (1.08 if lim != 80 else 0.72), lab, fontsize=7.5, color=INK2, ha="right")
    ax.set_yscale("log")
    ax.set_ylim(0.3, 50000)
    ax.set_xlim(0, 420)
    ax.set_xlabel("GNSS-free time: from the route's start (s)")
    ax.set_ylabel("horizontal error (m, log)")
    ax.set_title("GDN's horizontal error on every scenario (5-minute max limits as lines)")
    ax.legend(loc="upper left", fontsize=8, ncol=4)
    ax.grid(color=GRID, lw=0.5)
    # 5-minute CEP95 and max against their limits.
    ax = fig.add_subplot(gs[0, 2])
    y = np.arange(len(names))
    c95 = [ms[n]["windows"]["300"]["cep95_m"] for n in names]
    mx = [ms[n]["windows"]["300"]["max_m"] for n in names]
    lim95 = [ms[n]["windows"]["300"]["cep95_limit_m"] for n in names]
    ax.scatter(mx, y, color=MUTED, s=22, label="max", zorder=3)
    ax.scatter(c95, y, color=[VCOL[ms[n]["vehicle"]] for n in names], s=30, label="CEP95", zorder=4)
    ax.scatter(lim95, y, marker="|", color=RED, s=160, label="CEP95 limit", zorder=5)
    ax.set_xscale("log")
    ax.set_yticks(y)
    ax.set_yticklabels(short)
    ax.invert_yaxis()
    ax.set_xlabel("m (log)")
    ax.set_title("First 5 minutes: CEP95 and max")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Consistency.
    ax = fig.add_subplot(gs[1, 0])
    o3 = [ms[n]["consistency"]["over3sigma_pct"] for n in names]
    ax.barh(y, o3, color=[VCOL[ms[n]["vehicle"]] for n in names], height=0.55)
    ax.axvline(1, color=RED, lw=1, ls=(0, (3, 2)))
    ax.text(1.5, len(names) - 0.4, "REQ-INT-001: at most 1%", fontsize=7.5, color=RED)
    for yy, v, n in zip(y, o3, names):
        ax.text(v + 1, yy, f"{v:.0f}%  (NEES median {ms[n]['consistency']['nees_median']:.3g})", va="center",
                fontsize=7.5, color=INK2)
    ax.set_yticks(y)
    ax.set_yticklabels(short)
    ax.invert_yaxis()
    ax.set_xlim(0, 135)
    ax.set_xlabel("route samples with the error over GDN's 3 sigma (%)")
    ax.set_title("Consistency: over 3 sigma (and NEES)")
    ax.grid(color=GRID, lw=0.5, axis="x")
    # NIS per sensor.
    ax = fig.add_subplot(gs[1, 1])
    sensors = ["AGL", "baro", "mag (3-axis)", "ZUPT"]
    for k, sn in enumerate(sensors):
        vals = [(i, ms[n]["nis"][sn]["mean"] / ms[n]["nis"][sn]["expected"]) for i, n in enumerate(names)
                if sn in ms[n]["nis"]]
        if vals:
            ax.scatter([v for _, v in vals], [i + (k - 1.5) * 0.15 for i, _ in vals], s=20,
                       color=(BLUE, GREEN, ORANGE, MUTED)[k], label=sn, zorder=3)
    ax.axvline(1, color=INK2, lw=1)
    ax.set_xscale("log")
    ax.set_xlim(0.003, 100)
    ax.set_yticks(y)
    ax.set_yticklabels(short)
    ax.invert_yaxis()
    ax.set_xlabel("NIS mean / expected (1 = consistent; log)")
    ax.set_title("NIS per aiding sensor")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # CPU and latency.
    ax = fig.add_subplot(gs[1, 2])
    cm = [ms[n].get("cpu", {}).get("mean_pct", np.nan) for n in names]
    cp = [ms[n].get("cpu", {}).get("p95_pct", np.nan) for n in names]
    ax.barh(y, cp, color=GRID, height=0.55, label="p95")
    ax.barh(y, cm, color=[VCOL[ms[n]["vehicle"]] for n in names], height=0.3, label="mean")
    for yy, n in zip(y, names):
        la = ms[n].get("latency_ms", {})
        sm = ms[n].get("sim", {})
        s2o = (f"{la.get('sample_to_output_p95', np.nan):.0f} ms" if sm.get("timing_reliable") else "n/a")
        ax.text(max([c for c in cp if np.isfinite(c)] + [1]) * 1.03, yy,
                f"RTF {sm.get('real_time_factor', np.nan):.3f} | {la.get('processing_p95', np.nan):.3f} ms | {s2o}",
                va="center", fontsize=7.5, color=INK2)
    ax.set_yticks(y)
    ax.set_yticklabels(short)
    ax.invert_yaxis()
    ax.set_xlim(0, max([c for c in cp if np.isfinite(c)] + [1]) * 3.2)
    ax.set_xlabel("eskf_node CPU (% of one core)\ntext: RTF | processing p95 | PX4 sample to output p95\n"
                  "(n/a: the sim's slowest 30 s ran under RTF 0.98)", fontsize=8)
    ax.set_title("CPU, real-time factor (RTF) and latency")
    ax.legend(loc="lower right", fontsize=7.5)
    ax.grid(color=GRID, lw=0.5, axis="x")
    # Scorecard.
    ax = fig.add_subplot(gs[2, :])
    ax.axis("off")

    def v(ok, complete=True):
        return ("PASS" if ok else "FAIL") + ("" if complete else "*")
    rows = []
    for n in names:
        m = ms[n]
        w5, w10 = m["windows"]["300"], m["windows"]["600"]
        vert = m.get("vertical", {})
        acq = m["acquisition"]
        rows.append([n, m["requirement"], f"{m['route_s']:.0f} s",
                     f"{w5['cep95_m']:.0f} / {w5['cep95_limit_m']:g} {v(w5['cep95_ok'], w5['complete'])}",
                     f"{w5['max_m']:.0f} / {w5['max_limit_m']:g} {v(w5['max_ok'], w5['complete'])}",
                     f"{w10['cep95_m']:.0f} / {w10['cep95_limit_m']:g} {v(w10['cep95_ok'], w10['complete'])}",
                     f"{vert.get('p95_m', np.nan):.1f} / {vert.get('limit_m', np.nan):g} {v(vert.get('ok'))}",
                     f"{m['consistency']['over3sigma_pct']:.0f}% {v(m['consistency']['ok'])}",
                     ("-" if not acq["judged"] else f"{acq['error_m']:.0f} m {v(acq['ok'])}"),
                     "never" if acq["time_to_acquire_s"] is None else f"{acq['time_to_acquire_s']:.0f} s"])
    tb = ax.table(cellText=rows, colLabels=["scenario", "class", "route", "CEP95 5 min (m)", "max 5 min (m)",
                                            "CEP95 10 min (m)", "altitude p95 (m)", "over 3 sigma",
                                            "REQ-ACQ-001 at 120 s", "time to acquire"],
                  cellLoc="left", loc="upper center",
                  colWidths=[0.2, 0.08, 0.05, 0.11, 0.1, 0.11, 0.1, 0.08, 0.1, 0.07])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8)
    tb.scale(1, 1.55)
    for (rr, cc), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        txt = cell.get_text().get_text()
        if rr == 0:
            cell.get_text().set_fontweight("bold")
        elif "FAIL" in txt:
            cell.get_text().set_color(RED)
        elif "PASS" in txt:
            cell.get_text().set_color(GREEN)
    ax.set_title("tools/score_run.py on each scenario's run (one run each; * the window isn't complete: the route "
                 "ended first; the requirement pools sets of runs, R2.7b)", pad=6)
    fig.suptitle("TEST-042: every scenario scored against the requirements (R2.7a): GDN today, one run each",
                 x=0.04, ha="left", fontsize=13, fontweight="bold", color=INK)
    fig.savefig(out, dpi=100, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test042_data")
    ap.add_argument("--export", metavar="BATCH_DIR", help="export the runs in this --out directory into DATA")
    ap.add_argument("--out", default="docs/test_reports/test042_scores.png")
    a = ap.parse_args()
    if a.export:
        export(a.data, a.export)
    else:
        draw(a.data, a.out)


if __name__ == "__main__":
    main()
