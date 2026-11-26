#!/usr/bin/env python3
"""TEST-043 figure: the first accepted SITL baseline, three runs of each scenario (R2.7b).

    # from the scored run directories (numpy only), once:
    python3 tools/plot_test043.py --export docs/test_reports/test043_data log/r27a/final2 log/r27b/batch
    # from the exported files (what's committed):
    python3 tools/plot_test043.py docs/test_reports/test043_data --out docs/test_reports/test043_baseline.png

Export: runs.csv (per run: the compared metrics, the real-time factor, the integrity
proxy's cases and false alarms), loo.csv (each run against the worst of its scenario's
other runs: tools/score_set.py's leave-one-out spread), each run's metrics.json, and the
set's report (set.md, set.json, set.png from tools/score_set.py).
"""
import argparse
import csv
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import score_set as S  # noqa: E402

INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
BLUE, ORANGE, GREEN, RED, PURPLE = "#2a78d6", "#eb6834", "#1f9e6a", "#b3261e", "#7b4fc9"
SCEN = ("x500_emmental_smoke", "x500_emmental_gnss_cut", "x500_emmental_wind", "x500_emmental_spoof",
        "x500_emmental_faults", "x500_emmental_dusk", "standard_vtol_emmental_wind", "standard_vtol_emmental_faults",
        "rc_cessna_emmental_start_error", "rc_cessna_emmental_faults")
VCOL = {"x500": BLUE, "standard_vtol": PURPLE, "rc_cessna": ORANGE}
KEYS = [m[0] for m in S.METRICS]


def export(out, paths):
    out = Path(out)
    (out / "runs").mkdir(parents=True, exist_ok=True)
    runs = S.load(paths)
    p = S.pool(runs)
    rows = []
    for r in runs:
        m = r["metrics"]
        ig = m["integrity"]
        name = f"{m['scenario']}_{r['started']}"
        shutil.copy(Path(S.ROOT / r["dir"]) / "metrics.json", out / "runs" / f"{name}_metrics.json")
        v = S.values(m)
        rows.append({"scenario": m["scenario"], "vehicle": m["vehicle"], "started_utc": r["started"],
                     "run_dir": r["dir"], "gdn_version": r["version"], "check_passed": r["passed"],
                     "route_s": round(m["route_s"], 1), "rtf": round(m["sim"]["real_time_factor"], 4),
                     "rtf_min_30s": round(m["sim"]["rtf_min_30s"], 4), "max_limit_5min_m": m["windows"]["300"]["max_limit_m"],
                     **{k: (round(x, 6) if math.isfinite(x) else "") for k, x in v.items()},
                     "int_cases": ig["judged"], "int_detected": ig["detected"], "int_false": ig["false_alarms"],
                     "int_nominal": ig["nominal"], "int_flag_pct": round(ig["flag_pct"], 2),
                     "int_flight_h": round(ig["flight_h"], 4)})
    with open(out / "runs.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(out / "loo.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["metric", "scenario", "started_utc", "x_worst_other"])
        for k, label, unit, _, better, rel, abs_, gated in S.METRICS:
            for name, rs in p["by_run"].items():
                vals = [S.values(x["metrics"])[k] for x in rs]
                for i, n in enumerate(vals):
                    b = [x for j, x in enumerate(vals) if j != i and math.isfinite(x)]
                    if not b or not math.isfinite(n):
                        continue
                    worst = max(b) if better == "lower" else min(b)
                    if better == "lower" and worst > 0:
                        w.writerow([k, name, rs[i]["started"], round(n / worst, 6)])
                    elif better == "higher" and n > 0:
                        w.writerow([k, name, rs[i]["started"], round(worst / n, 6)])
    print(f"exported {len(rows)} runs to {out}")


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(x):
    return float(x) if x not in ("", None) else float("nan")


def draw(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    d = Path(d)
    runs = read_csv(d / "runs.csv")
    loo = read_csv(d / "loo.csv")
    from matplotlib.patches import Patch
    fig, axs = plt.subplots(1, 3, figsize=(18, 6.4), gridspec_kw={"width_ratios": [1.1, 1.0, 1.05]})
    fig.subplots_adjust(left=0.135, right=0.99, wspace=0.55, top=0.84, bottom=0.14)
    # (a) the same flight three times: the 5-minute max error per run.
    ax = axs[0]
    y = {s: k for k, s in enumerate(SCEN[::-1])}
    for r in runs:
        s = r["scenario"]
        ax.plot(fnum(r["max_5min"]), y[s], "o", ms=6, color=VCOL[r["vehicle"]], mec="white", mew=1, zorder=3)
    for s in SCEN:
        rs = [fnum(r["max_5min"]) for r in runs if r["scenario"] == s]
        ax.plot([min(rs), max(rs)], [y[s]] * 2, color=MUTED, lw=1, zorder=1)
        ax.text(max(rs) * 1.25, y[s], f"{max(rs) / min(rs):.0f} x", va="center", fontsize=8, color=INK2)
        lim = fnum(next(r for r in runs if r["scenario"] == s)["max_limit_5min_m"])
        ax.plot([lim], [y[s]], "|", ms=12, mew=2, color=RED, zorder=2)
    ax.set_xscale("log")
    ax.set_xlim(20, 1.5e5)
    ax.set_yticks(list(y.values()))
    ax.set_yticklabels(list(y.keys()))
    ax.set_xlabel("5-minute max horizontal error (m, log); red: the class's limit")
    ax.set_title("(a) The same scenario, three runs:\nthe max error spans up to 33 x")
    ax.grid(color=GRID, lw=0.5, axis="x")
    for v, c in VCOL.items():
        ax.plot([], [], "o", color=c, label=v)
    ax.legend(loc="lower right", fontsize=8)
    # (b) leave one out: how far an unchanged GDN lands past the worst of the others.
    ax = axs[1]
    order = [m for m in S.METRICS if any(r["metric"] == m[0] for r in loo) and not m[0].startswith("int_")]
    yy = {m[0]: k for k, m in enumerate(order[::-1])}
    rng = np.random.default_rng(1)
    for r in loo:
        k = r["metric"]
        if k not in yy:
            continue
        g = next(m[7] for m in S.METRICS if m[0] == k)
        x = max(fnum(r["x_worst_other"]), 0.01)
        ax.plot(x, yy[k] + rng.uniform(-0.18, 0.18), "o", ms=4, color=GREEN if g else BLUE, alpha=0.75, mec="none")
    for m in order:
        if m[7]:
            ax.plot([1 + m[5]], [yy[m[0]]], "|", ms=14, mew=2, color=RED)
    ax.axvline(1, color=INK2, lw=0.8)
    ax.set_xscale("log")
    ax.set_xlim(0.01, 60)
    ax.set_yticks(list(yy.values()))
    ax.set_yticklabels([m[1] for m in order[::-1]], fontsize=8)
    ax.set_xlabel("each run / the worst of its scenario's other runs (log)")
    ax.set_title("(b) Unchanged GDN, leave one out: navigation\nlands up to 25 x past; cost within 1.5 x")
    ax.plot([], [], "o", color=GREEN, label="gated")
    ax.plot([], [], "o", color=BLUE, label="reported only")
    ax.plot([], [], "|", ms=10, mew=2, color=RED, label="gated tolerance")
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(color=GRID, lw=0.5, axis="x")
    # (c) the integrity proxy per scenario: lost-lock cases flagged within 30 s, missed.
    ax = axs[2]
    for s in SCEN:
        rs = [r for r in runs if r["scenario"] == s]
        det = sum(int(r["int_detected"]) for r in rs)
        cas = sum(int(r["int_cases"]) for r in rs)
        fa = sum(int(r["int_false"]) for r in rs if r["int_nominal"] == "True")
        flag = np.median([fnum(r["int_flag_pct"]) for r in rs])
        ax.barh(y[s], det, color=GREEN, height=0.55)
        ax.barh(y[s], cas - det, left=det, color="white", edgecolor=RED, hatch="////", lw=1, height=0.55)
        txt = f"flagged {flag:.0f}%"
        if rs[0]["int_nominal"] == "True":
            txt += f", {fa} false alarm{'s' if fa != 1 else ''}"
        ax.text(4.25, y[s], txt, va="center", fontsize=8, color=INK2)
    ax.set_xlim(0, 7.6)
    ax.set_xticks(range(5))
    ax.set_yticks(list(y.values()))
    ax.set_yticklabels([])
    ax.legend(handles=[Patch(color=GREEN, label="flagged within 30 s"),
                       Patch(facecolor="white", edgecolor=RED, hatch="////", label="missed")],
              loc="upper center", bbox_to_anchor=(0.4, -0.09), ncol=2, fontsize=8)
    ax.set_xlabel("")
    ax.set_title("(c) Integrity proxy, lost-lock cases (error past\n3 x the max limit): 22 of 31 flagged")
    ax.grid(color=GRID, lw=0.5, axis="x")
    fig.suptitle("TEST-043: the first accepted SITL baseline, 30 runs (three of each R2.6 scenario), GDN as of R2.7a",
                 x=0.02, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.savefig(out, dpi=90, facecolor="white")
    plt.close(fig)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test043_data")
    ap.add_argument("--export", nargs="+", metavar="RUNS", help="export these scored runs into DATA")
    ap.add_argument("--out", default="docs/test_reports/test043_baseline.png")
    a = ap.parse_args()
    if a.export:
        export(a.data, a.export)
        return
    draw(a.data, a.out)


if __name__ == "__main__":
    main()
