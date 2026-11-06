#!/usr/bin/env python3
"""Figures for TEST-021 (R1.4, covariance consistency).

  docs/test_reports/test021_fixes.png  what each fix did, step by step
  docs/test_reports/test021_mag.png    the magnetometer trade-off per fusion mode

Inputs: docs/test_reports/test021_data/{stages,mag_modes}.csv (the Monte Carlo and
selftest numbers in the report).

    python3 tools/plot_covariance_fixes.py
"""
import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

# Default palette: categorical slots 1-3, text inks, grid, surface.
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"

ap = argparse.ArgumentParser()
ap.add_argument("--data", default=str(ROOT / "docs/test_reports/test021_data"))
ap.add_argument("--out-dir", default=str(ROOT / "docs/test_reports"))
a = ap.parse_args()
data, out = Path(a.data), Path(a.out_dir)
plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})

rows = list(csv.DictReader(open(data / "stages.csv")))
labels = []
for r in rows:
    if r["label"] not in labels:
        labels.append(r["label"])
series = (("zermatt_truth_map", C1, "Zermatt, perfect onboard map"),
          ("zermatt_srtm_maps", C2, "Zermatt, SRTM-like onboard maps"),
          ("flight001", C3, "flight001 (gentle terrain)"))


def col(scen, key):
    return [float(r[key]) if r[key] else np.nan for r in rows if r["scenario"] == scen]


SHORT = ["before\nR1.4", "+ fix 1", "+ fix 2", "+ fix 3", "+ fix 4"]


def stage_axis(ax, full=False):
    ax.set_xticks(range(len(labels)))
    if full:
        ax.set_xticklabels([f"fix {i}: " + l[2:] if i else l for i, l in enumerate(labels)], fontsize=8.5)
        ax.set_xticklabels([t.get_text().replace(" (", "\n(") for t in ax.get_xticklabels()])
    else:
        ax.set_xticklabels(SHORT[:len(labels)], fontsize=9)
    ax.grid(True, axis="y", color=GRID, lw=0.6); ax.set_axisbelow(True)


fig = plt.figure(figsize=(14, 10.5), facecolor=SURF)
gs = fig.add_gridspec(2, 2, hspace=0.5, wspace=0.22)
x = np.arange(len(labels))

ax = fig.add_subplot(gs[0, 0])
for scen, c, name in series:
    y = col(scen, "passing")
    ax.plot(x, y, "-o", color=c, lw=2, ms=8, mec=SURF, mew=1.2, label=name, zorder=3)
    ax.text(x[0] - 0.12, y[0], f"{y[0]:.0f}", ha="right", va="center", fontsize=8.5, color=INK2)
    ax.text(x[-1] + 0.12, y[-1], f"{y[-1]:.0f}", ha="left", va="center", fontsize=8.5, color=INK2)
stage_axis(ax); ax.set_xlim(-0.6, len(labels) - 0.4); ax.set_ylim(80, 102); ax.set_yticks(range(80, 101, 5))
ax.set_ylabel("flights under 100 m, of 100")
ax.set_title("Accuracy: flights staying under 100 m", loc="left", fontsize=11)
ax.legend(loc="lower right", fontsize=8.5, framealpha=0.9)

ax = fig.add_subplot(gs[0, 1])
for scen, c, name in series:
    y = np.maximum(col(scen, "over3sigma_pct"), 0.01)
    ax.plot(x, y, "-o", color=c, lw=2, ms=8, mec=SURF, mew=1.2, label=name, zorder=3)
ax.axhline(1.0, color=INK2, ls="--", lw=1)
ax.text(len(labels) - 0.5, 1.2, "REQ-INT-001 limit: 1%", ha="right", fontsize=9, color=INK2)
ax.set_yscale("log"); ax.set_ylim(0.008, 40)
stage_axis(ax)
ax.set_ylabel("% of time truth is outside the\nclaimed 3-sigma circle (log scale)")
ax.set_title("Honesty: below the dashed line passes\n(flight001 at 0.00% is drawn at 0.01%)", loc="left", fontsize=11)

ax = fig.add_subplot(gs[1, :])
for scen, c, name in series:
    y = col(scen, "cep95")
    ax.plot(x, y, "-o", color=c, lw=2, ms=8, mec=SURF, mew=1.2, label=name, zorder=3)
    for xi, yi in zip(x, y):
        ax.text(xi, yi * 1.12, f"{yi:.0f}", ha="center", fontsize=8.5, color=INK2)
ax.set_yscale("log"); ax.set_ylim(15, 600)
stage_axis(ax, full=True)
ax.set_ylabel("CEP95: error that 95% of\nsamples stay under [m] (log)")
ax.set_title("Worst case: the error that 95% of samples stay under, after each fix (100 flights per point)",
             loc="left", fontsize=11)
ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9)
fig.savefig(out / "test021_fixes.png", dpi=120, facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ---- magnetometer modes ----
m = list(csv.DictReader(open(data / "mag_modes.csv")))
names = [r["label"] for r in m]
fig, axs = plt.subplots(1, 2, figsize=(14, 4.8), facecolor=SURF, gridspec_kw={"wspace": 0.35})
ax = axs[0]
tilt = [float(r["tilt_6uT_deg"]) if r["tilt_6uT_deg"] else np.nan for r in m]
yy = np.arange(len(names))[::-1]
for yi, t, n in zip(yy, tilt, names):
    if np.isnan(t):
        ax.text(0.05, yi, "no updates in flight, so nothing to tilt", va="center", fontsize=9, color=INK2)
        continue
    ax.barh(yi, t, height=0.5, color=C1, zorder=3)
    ax.text(t + 0.05, yi, f"{t:.2f} deg", va="center", fontsize=9, color=INK)
ax.set_yticks(yy); ax.set_yticklabels(names)
ax.set_xlim(0, 3.2); ax.set_xlabel("attitude tilt caused [deg] (lower is better)")
ax.grid(True, axis="x", color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.set_title("Robustness: tilt caused by a 6 uT motor-current field\n(selftest TEST-021, level and still for 60 s)",
             loc="left", fontsize=11)

ax = axs[1]
for key, c, name, off in (("zermatt_truth_map_passing", C1, "perfect onboard map", -0.12),
                          ("srtm_maps_passing", C2, "SRTM-like onboard maps", 0.12)):
    vals = [float(r[key]) for r in m]
    ax.plot(np.array(yy) + off, vals, "o", color=c, ms=9, mec=SURF, mew=1.2, label=name, zorder=3)
    for yi, v in zip(np.array(yy) + off, vals):
        ax.text(yi, v + 1.2, f"{v:.0f}", ha="center", fontsize=8.5, color=INK2)
ax.set_xticks(yy); ax.set_xticklabels(names)
ax.set_ylim(70, 105); ax.set_ylabel("flights under 100 m, of 100")
ax.grid(True, axis="y", color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.set_title("Accuracy: Zermatt, calibrated compass, clean simulated field\n(the simulation has no disturbance, so robustness doesn't show here)",
             loc="left", fontsize=11)
ax.legend(loc="lower left", fontsize=8.5, framealpha=0.9)
fig.savefig(out / "test021_mag.png", dpi=120, facecolor=SURF, bbox_inches="tight")
print(f"wrote {out / 'test021_fixes.png'} and {out / 'test021_mag.png'}")
