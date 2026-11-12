#!/usr/bin/env python3
"""Figure for TEST-026 (R1.8: today's GDN scored against the agreed requirements).

  docs/test_reports/test026_scorecard.png   every scored line, as measured / limit

Drawn from docs/test_reports/test026_data/scorecard.csv, which tools/score_requirements.py
writes (--csv) from the Monte Carlo outputs listed in the TEST-026 report.

    python3 tools/plot_test026.py
"""
import csv
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
GOOD, BAD = "#1baf7a", "#d43d3d"
DATA = ROOT / "docs/test_reports/test026_data/scorecard.csv"
OUT = ROOT / "docs/test_reports/test026_scorecard.png"
FLOOR = 0.02   # where a measured 0 is drawn on the log axis


def ratio(meas, lim):
    """Measured over limit; for the 99% rules, the share of flights over the limit over 1%."""
    m = re.match(r"(\d+)/(\d+) flights", meas)
    if m:
        ok, n = int(m.group(1)), int(m.group(2))
        return (n - ok) / n / 0.01, f"{n - ok} of {n} flights over"
    v = float(re.match(r"[\d.]+", meas).group(0))
    lv = float(re.match(r"[\d.]+", lim).group(0))
    return v / lv, f"{meas} (limit {lim})"


def main():
    rows = [r for r in csv.DictReader(open(DATA)) if r["verdict"] in ("PASS", "FAIL")]
    labels, xs, notes, ok = [], [], [], []
    groups = []
    for r in rows:
        if not groups or groups[-1][0] != r["scenario"]:
            groups.append((r["scenario"], len(labels)))
        x, note = ratio(r["measured"], r["limit"])
        labels.append(f"{r['requirement']}  {r['what']}")
        xs.append(max(x, FLOOR))
        notes.append(note)
        ok.append(r["verdict"] == "PASS")
    # one gap row per scenario, holding its heading
    pos, heads, k = [], [], 0
    gstarts = {start: name for name, start in groups}
    for i in range(len(labels)):
        if i in gstarts:
            heads.append((gstarts[i], k)); k += 1
        pos.append(k); k += 1
    y = -np.array(pos, dtype=float)
    h = 0.31 * k + 1.2
    fig, ax = plt.subplots(figsize=(11.5, h), facecolor=SURF)
    fig.subplots_adjust(top=1.0 - 0.55 / h)
    ax.set_facecolor(SURF)
    lo, hi = 0.3, 400
    ax.axvspan(lo, 1.0, color=GOOD, alpha=0.07, lw=0)
    ax.axvline(1.0, color=INK2, lw=1.2)
    for yi, x, good, note in zip(y, xs, ok, notes):
        ax.plot([1.0, x], [yi, yi], "-", color=GRID, lw=2, zorder=1)
        ax.plot(max(x, lo * 1.05), yi, "o", ms=8, color=GOOD if good else BAD, mec=SURF, mew=1.5, zorder=3)
        ax.text(max(x, 1.0) * 1.25, yi, ("✓ " if good else "✗ ") + note, va="center", ha="left",
                fontsize=8, color=INK2)
    for name, kk in heads:
        ax.text(-0.01, -kk, name, transform=ax.get_yaxis_transform(), fontsize=9.5, color=INK, ha="right",
                va="center", fontweight="bold")
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8.5, color=INK2)
    ax.set_xscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(-k + 0.4, 0.8)
    ax.text(1.0, 0.8, " the limit", va="bottom", ha="left", fontsize=8.5, color=INK2)
    ax.set_xlabel("measured ÷ limit (for the 99% rules: share of flights over the limit ÷ 1%).  "
                  "Left of the line passes", color=INK2, fontsize=9)
    ax.grid(True, axis="x", which="major", color=GRID, lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(axis="x", colors=INK2, labelsize=8.5)
    fig.suptitle("TEST-026: today's GDN against the agreed fixed-wing requirements (realistic maps, 100 flights each)",
                 x=0.01, ha="left", color=INK2, fontsize=10.5)
    fig.savefig(OUT, dpi=120, facecolor=SURF, bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
