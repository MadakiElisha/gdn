#!/usr/bin/env python3
"""Figure for TEST-027 (R1.9: code-quality gates in CI).

  docs/test_reports/test027_gates.png   what each gate found on main (all fixed), and its run time

Drawn from docs/test_reports/test027_data/{findings,gates}.csv, which list the findings of
the first run of tools/quality.sh on main and the local run times (see the report).

    python3 tools/plot_test027.py
"""
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

C1, C2 = "#2a78d6", "#eb6834"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
DATA = ROOT / "docs/test_reports/test027_data"
OUT = ROOT / "docs/test_reports/test027_gates.png"


def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8.5)
    ax.grid(True, axis="x", color=GRID, lw=0.5)
    ax.set_axisbelow(True)


def main():
    f = list(csv.DictReader(open(DATA / "findings.csv")))[::-1]
    g = list(csv.DictReader(open(DATA / "gates.csv")))[::-1]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12.5, 4.6), facecolor=SURF,
                                 gridspec_kw={"width_ratios": [1.25, 1], "wspace": 0.75})
    style(a1)
    y = range(len(f))
    cols = [C1 if r["gate"] == "clang-tidy" else C2 for r in f]
    a1.barh(list(y), [int(r["count"]) for r in f], color=cols, height=0.62)
    for yi, r in zip(y, f):
        a1.text(int(r["count"]) + 0.2, yi, r["count"], va="center", fontsize=8.5, color=INK2)
    a1.set_yticks(list(y))
    a1.set_yticklabels([r["check"] for r in f], fontsize=8.5, color=INK2)
    a1.set_xlabel("findings on main before this PR", color=INK2, fontsize=9)
    a1.set_title("What the gates found: 27 clang-tidy (blue), 3 cppcheck (orange)", loc="left", fontsize=10,
                 color=INK)
    style(a2)
    y = range(len(g))
    a2.barh(list(y), [float(r["local time s"]) / 60 for r in g], color=C1, height=0.62)
    for yi, r in zip(y, g):
        t = float(r["local time s"])
        lab = f"{t / 60:.1f} min" if t >= 60 else f"{t:.0f} s" if t >= 1 else "<1 s"
        a2.text(t / 60 + 0.05, yi, lab, va="center", fontsize=8.5, color=INK2)
    a2.set_yticks(list(y))
    a2.set_yticklabels([r["gate"] for r in g], fontsize=9, color=INK2)
    a2.set_xlabel("run time on the dev PC [minutes]", color=INK2, fontsize=9)
    a2.set_title("What they cost (one CI run)", loc="left", fontsize=10, color=INK)
    fig.suptitle("TEST-027: code-quality gates. No sanitizer or compiler-warning findings; 30 static-analysis "
                 "findings, none a crash: 29 fixed, 1 suppressed with its reason", x=0.01, ha="left", color=INK2, fontsize=10.5)
    fig.savefig(OUT, dpi=120, facecolor=SURF, bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
