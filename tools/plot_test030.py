#!/usr/bin/env python3
"""Figure for TEST-030 (the Phase 1 report: R1.1 to R1.7 all on, scored against R1.8).

  docs/test_reports/test030_scorecard.png   every scored line, as measured / limit, with
                                            TEST-026's value where it differs

Drawn from docs/test_reports/test030_data/scorecard.csv and test026_data/scorecard.csv,
which tools/score_requirements.py writes (--csv) from the MC outputs in those folders.

    python3 tools/plot_test030.py
"""
import csv
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
GOOD, BAD, OLD = "#1baf7a", "#d43d3d", "#8a8984"
DATA = ROOT / "docs/test_reports/test030_data/scorecard.csv"
BEFORE = ROOT / "docs/test_reports/test026_data/scorecard.csv"
OUT = ROOT / "docs/test_reports/test030_scorecard.png"
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


def read(path):
    return [r for r in csv.DictReader(open(path)) if r["verdict"] in ("PASS", "FAIL")]


def keyed(rows):
    """(scenario, requirement, what, n-th such row): a scenario has one "whole run"
    integrity line per start error, so lines are matched in order."""
    seen, out = {}, {}
    for r in rows:
        k = (r["scenario"], r["requirement"], r["what"])
        seen[k] = seen.get(k, -1) + 1
        out[k + (seen[k],)] = r
    return out


def main():
    rows = read(DATA)
    now, before = keyed(rows), keyed(read(BEFORE))
    rows = list(now.items())
    labels, xs, notes, ok, olds = [], [], [], [], []
    groups = []
    for key, r in rows:
        if not groups or groups[-1][0] != r["scenario"]:
            groups.append((r["scenario"], len(labels)))
        x, note = ratio(r["measured"], r["limit"])
        b = before.get(key)
        old = None
        if b and b["measured"] != r["measured"]:
            ox, onote = ratio(b["measured"], b["limit"])
            old = max(ox, FLOOR)
            note += "   (TEST-026: " + onote.split(" (limit")[0] + ")"
        labels.append(f"{r['requirement']}  {r['what']}")
        xs.append(max(x, FLOOR))
        notes.append(note)
        ok.append(r["verdict"] == "PASS")
        olds.append(old)
    pos, heads, k = [], [], 0
    gstarts = {start: name for name, start in groups}
    for i in range(len(labels)):
        if i in gstarts:
            heads.append((gstarts[i], k)); k += 1
        pos.append(k); k += 1
    y = -np.array(pos, dtype=float)
    h = 0.31 * k + 1.0
    fig, ax = plt.subplots(figsize=(12.5, h), facecolor=SURF)
    ax.set_facecolor(SURF)
    lo, hi = 0.3, 600
    ax.axvspan(lo, 1.0, color=GOOD, alpha=0.07, lw=0)
    ax.axvline(1.0, color=INK2, lw=1.2)
    for yi, x, good, note, old in zip(y, xs, ok, notes, olds):
        ax.plot([1.0, x], [yi, yi], "-", color=GRID, lw=2, zorder=1)
        if old is not None:
            ax.plot(max(old, lo * 1.05), yi, "o", ms=8, mfc="none", mec=OLD, mew=1.4, zorder=2)
        ax.plot(max(x, lo * 1.05), yi, "o", ms=8, color=GOOD if good else BAD, mec=SURF, mew=1.5, zorder=3)
        ax.text(max(x, old or 0, 1.0) * 1.25, yi, ("✓ " if good else "✗ ") + note, va="center", ha="left",
                fontsize=8, color=INK2)
    for name, kk in heads:
        ax.text(-0.01, -kk, name, transform=ax.get_yaxis_transform(), fontsize=9.5, color=INK, ha="right",
                va="center", fontweight="bold")
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8.5, color=INK2)
    ax.set_xscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(-k + 0.4, 0.8)
    ax.text(1.0, -k + 0.5, " the limit", va="bottom", ha="left", fontsize=8.5, color=INK2)
    ax.plot([], [], "o", ms=7, color=BAD, label="Phase 1, R1.1 to R1.7 all on (fail)")
    ax.plot([], [], "o", ms=7, color=GOOD, label="(pass)")
    ax.plot([], [], "o", ms=7, mfc="none", mec=OLD, mew=1.4, label="TEST-026, where it differs (flight001's truth start-up)")
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.01), ncol=3, fontsize=8.5, frameon=False,
              labelcolor=INK2)
    ax.set_xlabel("measured ÷ limit (for the 99% rules: share of flights over the limit ÷ 1%).  "
                  "Left of the line passes", color=INK2, fontsize=9)
    ax.grid(True, axis="x", which="major", color=GRID, lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(axis="x", colors=INK2, labelsize=8.5)
    n_pass, n = sum(ok), len(ok)
    fig.suptitle(f"TEST-030: Phase 1 closed. Fixed-wing requirements, realistic maps, 100 flights each: "
                 f"{n_pass} pass, {n - n_pass} fail", x=0.01, y=1.0, ha="left", va="bottom", color=INK2, fontsize=10.5)
    fig.tight_layout()
    fig.savefig(OUT, dpi=120, facecolor=SURF, bbox_inches="tight")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
