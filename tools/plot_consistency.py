#!/usr/bin/env python3
"""Show whether GDN's uncertainty is honest (R1.5, TEST-019).

For one good and one bad Monte Carlo flight: the estimated track with the 3-sigma
circle the filter claimed, against the true track, and the error against that
circle's radius over time. Then every flight on one chart: how far off it was
against how often the truth fell outside its claimed circle.

    install/gdn_fusion/lib/gdn_fusion/mc_campaign 100 data/flight_zermatt_csv \
        data/maps/zermatt.bin --model=agl --startup=node --trace=trace.csv
    python3 tools/plot_consistency.py --trace trace.csv
"""
import argparse
import csv
import struct
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from gdn_paths import ROOT

# Default palette (categorical slots 1 and 2, text inks).
GOOD, BAD = "#2a78d6", "#eb6834"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#d9d8d4"

ap = argparse.ArgumentParser()
ap.add_argument("--trace", default="trace.csv", help="mc_campaign --trace output")
ap.add_argument("--map", default=str(ROOT / "data/maps/zermatt.bin"))
ap.add_argument("--out", default="consistency.png")
ap.add_argument("--good", type=int, help="seed to show as the good flight (default: a typical one)")
ap.add_argument("--bad", type=int, help="seed to show as the bad flight (default: a typical one)")
ap.add_argument("--window", default="60,300", help="scoring window, s")
ap.add_argument("--every", type=float, default=10.0, help="seconds between drawn circles")
a = ap.parse_args()
w0, w1 = (float(v) for v in a.window.split(","))

rows = defaultdict(lambda: defaultdict(list))
with open(a.trace) as f:
    for r in csv.DictReader(f):
        d = rows[int(r["seed"])]
        for k in ("t", "x", "y", "tx", "ty", "err", "sigma"):
            d[k].append(float(r[k]))
seeds = {s: {k: np.array(v) for k, v in d.items()} for s, d in rows.items()}

# Per-seed scores in the window: RMS error and % of time outside 3 x sigma.
score = {}
for s, d in seeds.items():
    w = (d["t"] >= w0) & (d["t"] <= w1)
    if w.any():
        score[s] = (float(np.sqrt(np.mean(d["err"][w] ** 2))),
                    float(100.0 * np.mean(d["err"][w] > 3.0 * d["sigma"][w])))
passing = sorted((s for s in score if score[s][0] < 100.0), key=lambda s: score[s][0])
failing = sorted((s for s in score if score[s][0] >= 100.0), key=lambda s: score[s][0])
good = a.good if a.good is not None else (passing[len(passing) // 2] if passing else None)
bad = a.bad if a.bad is not None else (failing[len(failing) // 2] if failing else None)

with open(a.map, "rb") as f:
    nx, ny = struct.unpack("<ii", f.read(8))
    lat0, lon0, dlat, dlon = struct.unpack("<dddd", f.read(32))
    alt = np.frombuffer(f.read(4 * nx * ny), dtype="<f4").reshape(ny, nx).astype(np.float64)
dn_m = dlat * 111320.0
de_m = dlon * 111320.0 * np.cos(np.radians(lat0 + 0.5 * ny * dlat))
n_ax = (np.arange(ny) - 0.5 * ny) * dn_m
e_ax = (np.arange(nx) - 0.5 * nx) * de_m
gz_n, gz_e = np.gradient(alt, abs(dn_m), abs(de_m))
shade = np.clip((-gz_e * -0.5 + gz_n * 0.5 + 1.0) / np.sqrt(gz_e ** 2 + gz_n ** 2 + 1.0), 0, 1)
extent = [e_ax[0] - de_m / 2, e_ax[-1] + de_m / 2, n_ax[-1] - abs(dn_m) / 2, n_ax[0] + abs(dn_m) / 2]

plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})
fig = plt.figure(figsize=(13, 13), facecolor="#fcfcfb")
gs = fig.add_gridspec(3, 2, height_ratios=[1.35, 0.8, 1.0], hspace=0.38, wspace=0.18)


def map_panel(ax, s, color, label):
    d = seeds[s]
    rms, out = score[s]
    ax.imshow(shade, extent=extent, cmap="gray", vmin=-0.3, vmax=1.2, alpha=0.55)
    ax.plot(d["ty"], d["tx"], "-", color=INK, lw=1.6, label="where it really was")
    ax.plot(d["y"], d["x"], "-", color=color, lw=1.6, label="where GDN thought it was")
    nxt = d["t"][0]
    for i, t in enumerate(d["t"]):
        if t < nxt:
            continue
        nxt = t + a.every
        ax.add_patch(Circle((d["y"][i], d["x"][i]), 3 * d["sigma"][i], fill=False, ec=color, lw=1.1, alpha=0.9))
        ax.plot([d["y"][i], d["ty"][i]], [d["x"][i], d["tx"][i]], "-", color=INK2, lw=0.6)
        ax.plot(d["ty"][i], d["tx"][i], "o", color=INK, ms=3)
    ax.plot([], [], "o", mfc="none", mec=color, ms=9, label=f"its 3-sigma circle (every {a.every:.0f} s)")
    xs = np.concatenate([d["y"], d["ty"]]); ys = np.concatenate([d["x"], d["tx"]])
    pad = 250.0
    ax.set_xlim(xs.min() - pad, xs.max() + pad); ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("east [m]"); ax.set_ylabel("north [m]")
    ax.set_title(f"{label} (seed {s}): RMS error {rms:.0f} m,\ntruth outside its circle {out:.0f}% of the time",
                 loc="left", fontsize=11)
    ax.legend(loc="lower left", fontsize=8, framealpha=0.9)


def error_panel(ax, s, color):
    d = seeds[s]
    ax.fill_between(d["t"], 0, 3 * d["sigma"], color=color, alpha=0.18, lw=0, label="claimed 3-sigma radius")
    ax.plot(d["t"], 3 * d["sigma"], "-", color=color, lw=0.8, alpha=0.6)
    ax.plot(d["t"], d["err"], "-", color=INK, lw=1.6, label="actual error")
    ax.axvspan(w0, min(w1, d["t"].max()), color=GRID, alpha=0.25, lw=0)
    ax.set_xlabel("time [s]"); ax.set_ylabel("metres")
    ax.set_ylim(0, max(3 * d["sigma"].max(), d["err"].max()) * 1.08)
    ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    ax.set_title("error against the circle it claimed (shaded: scoring window)", loc="left", fontsize=10)


if good is not None:
    map_panel(fig.add_subplot(gs[0, 0]), good, GOOD, "A good flight")
    error_panel(fig.add_subplot(gs[1, 0]), good, GOOD)
if bad is not None:
    map_panel(fig.add_subplot(gs[0, 1]), bad, BAD, "A bad flight")
    error_panel(fig.add_subplot(gs[1, 1]), bad, BAD)

ax = fig.add_subplot(gs[2, :])
for group, color, name in ((passing, GOOD, "under 100 m RMS"), (failing, BAD, "over 100 m RMS")):
    if group:
        ax.scatter([score[s][0] for s in group], [score[s][1] for s in group], s=46, color=color,
                   edgecolor="#fcfcfb", linewidth=1.2, zorder=3, label=f"{len(group)} flights {name}")
ax.axvline(100, color=INK2, ls="--", lw=1)
ax.axhline(1, color=INK2, ls=":", lw=1)
ax.text(104, 96, "100 m limit", color=INK2, fontsize=9, va="top")
ax.text(2900, 2.5, "REQ-INT-001: at most 1% outside", color=INK2, fontsize=9, ha="right")
for s, label, off in ((good, "good flight above", (-20, 45)), (bad, "bad flight above", (14, -28))):
    if s is not None:
        ax.annotate(label, score[s], xytext=off, textcoords="offset points", fontsize=9, color=INK,
                    arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
ax.set_xscale("log"); ax.set_xlim(2, 3000); ax.set_ylim(-3, 103)
ax.set_xlabel("RMS horizontal error over the scoring window [m] (log scale)")
ax.set_ylabel("% of time truth was outside\nthe claimed 3-sigma circle")
ax.grid(True, color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.legend(loc="center left", fontsize=9, framealpha=0.9)
ax.set_title("Every flight: honest flights sit bottom left; confidently wrong flights sit top right",
             loc="left", fontsize=11)

fig.savefig(a.out, dpi=130, facecolor=fig.get_facecolor(), bbox_inches="tight")
print(f"wrote {a.out}: {len(score)} flights, {len(passing)} under 100 m, {len(failing)} over; "
      f"good seed {good}, bad seed {bad}")
