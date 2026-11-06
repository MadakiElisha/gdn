#!/usr/bin/env python3
"""Figures for TEST-020 (R1.3): truth world map against the onboard map, and what map
error does to GDN.

  docs/test_reports/test020_maps.png     truth map, onboard map, onboard minus truth
  docs/test_reports/test020_results.png  flights passing and REQ-INT-001 per onboard map,
                                         and the spread across map error patterns

Inputs: the maps (tools/make_data.sh --db-maps) and the ensemble outputs in
docs/test_reports/test020_data/ ({z,f1}_<recipe>.txt from tools/mc_map_ensemble.py).

    python3 tools/plot_map_error.py
"""
import argparse
import csv
import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from scipy import ndimage

from gdn_paths import ROOT, load_map

# Default palette: categorical slots 1-2, diverging blue <-> red with a grey midpoint,
# text inks and grid.
ZER, F1 = "#2a78d6", "#eb6834"
DIV = LinearSegmentedColormap.from_list("div", ["#2a78d6", "#f0efec", "#e34948"])
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
RECIPES = [("none", "truth map\n(no error)"), ("noise2", "noise\n2.5 m"), ("noise5", "noise\n5 m"),
           ("noise10", "noise\n10 m"), ("coarse3", "3x coarser\ncells"), ("srtm", "SRTM-like\n(all three)")]

ap = argparse.ArgumentParser()
ap.add_argument("--data", default=str(ROOT / "docs/test_reports/test020_data"))
ap.add_argument("--world", default=str(ROOT / "data/maps/zermatt.bin"))
ap.add_argument("--db", default=str(ROOT / "data/maps/zermatt_db_srtm.bin"))
ap.add_argument("--track", default=str(ROOT / "data/flight_zermatt_csv/lpos.csv"))
ap.add_argument("--out-dir", default=str(ROOT / "docs/test_reports"))
a = ap.parse_args()
out = Path(a.out_dir)
plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})


def grid_ne(h, ny, nx):
    """North/east (m) of every grid sample, in the world map's frame (origin = world centre)."""
    n = (h["lat0"] + np.arange(ny) * h["dlat"] - W["lat_c"]) * 111320.0
    e = (h["lon0"] + np.arange(nx) * h["dlon"] - W["lon_c"]) * Wm
    return n, e


def shade(alt, dn, de):
    gn, ge = np.gradient(alt, abs(dn), abs(de))
    return np.clip((ge * 0.5 + gn * 0.5 + 1.0) / np.sqrt(ge ** 2 + gn ** 2 + 1.0), 0, 1)


# ---------- Figure 1: the maps ----------
walt, W = load_map(a.world)
dalt, D = load_map(a.db)
walt = walt.astype(float); dalt = dalt.astype(float)
Wm = W["m_per_deg_lon"]
wn, we = grid_ne(W, *walt.shape)
dn_, de_ = grid_ne(D, *dalt.shape)
# Onboard map sampled at the truth grid points (bilinear), as the filter sees it.
fi = (W["lat0"] + np.arange(walt.shape[0]) * W["dlat"] - D["lat0"]) / D["dlat"]
fj = (W["lon0"] + np.arange(walt.shape[1]) * W["dlon"] - D["lon0"]) / D["dlon"]
gi, gj = np.meshgrid(fi, fj, indexing="ij")
diff = ndimage.map_coordinates(dalt, [gi, gj], order=1, mode="nearest") - walt
m = 6
inner = diff[m:-m, m:-m]

tn, te = [], []
with open(a.track) as f:
    for r in csv.DictReader(f):
        tn.append(float(r["x"])); te.append(float(r["y"]))


def extent(n, e, dn, de):
    return [e.min() - abs(de) / 2, e.max() + abs(de) / 2, n.min() - abs(dn) / 2, n.max() + abs(dn) / 2]


cw = (abs(W["dlat"]) * 111320.0, abs(W["dlon"]) * Wm)
cd = (abs(D["dlat"]) * 111320.0, abs(D["dlon"]) * D["m_per_deg_lon"])
fig, ax = plt.subplots(1, 3, figsize=(15, 6.2), facecolor=SURF)
vmin, vmax = min(walt.min(), dalt.min()), max(walt.max(), dalt.max())
for k, (alt, n, e, c, title) in enumerate((
        (walt, wn, we, cw, f"Truth world: what the aircraft flies over\n{walt.shape[1]} x {walt.shape[0]} cells of {cw[0]:.0f} x {cw[1]:.0f} m"),
        (dalt, dn_, de_, cd, f"Onboard map: what GDN navigates with\n{dalt.shape[1]} x {dalt.shape[0]} cells of {cd[0]:.0f} x {cd[1]:.0f} m (SRTM-like)"))):
    s = shade(alt, c[0], c[1])
    norm = (alt - vmin) / (vmax - vmin)
    rgb = plt.cm.gray(0.35 + 0.55 * norm)[:, :, :3] * (0.45 + 0.55 * s[:, :, None])
    ax[k].imshow(np.clip(rgb, 0, 1), extent=extent(n, e, c[0], c[1]), origin="upper"
                 if W["dlat"] < 0 else "lower", interpolation="nearest")
    ax[k].plot(te, tn, "-", color=INK, lw=1.3, label="flight path")
    ax[k].set_title(title, loc="left", fontsize=10.5)
im = ax[2].imshow(diff, extent=extent(wn, we, *cw), origin="upper" if W["dlat"] < 0 else "lower",
                  cmap=DIV, vmin=-30, vmax=30, interpolation="nearest")
ax[2].plot(te, tn, "-", color=INK, lw=1.3, label="flight path")
ax[2].set_title(f"Onboard minus truth: the map error GDN doesn't know about\n"
                f"RMS {np.sqrt((inner ** 2).mean()):.1f} m, mean {inner.mean():+.1f} m, "
                f"worst {np.abs(inner).max():.0f} m", loc="left", fontsize=10.5)
cb = fig.colorbar(im, ax=ax[2], fraction=0.046, pad=0.03)
cb.set_label("height error [m]  (red: map too high, blue: too low)")
for x in ax:
    x.set_xlabel("east [m]"); x.set_ylabel("north [m]"); x.set_aspect("equal")
    x.legend(loc="lower left", fontsize=8, framealpha=0.9)
fig.tight_layout()
fig.savefig(out / "test020_maps.png", dpi=120, facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ---------- Figure 2: the results ----------
def parse(path):
    t = Path(path).read_text()
    per_map = [int(x) for x in re.findall(r"^map\s+\d+:.*\(<100 m\):\s*(\d+)/\d+", t, re.M)]
    pas = re.search(r"passing\(<100 m\):\s*(\d+)/(\d+)", t)
    over = re.search(r"error > 3 x published sigma\s*([0-9.]+)%", t)
    return per_map, int(pas.group(1)), int(pas.group(2)), float(over.group(1))


data = {}
for flight in ("z", "f1"):
    for tag, _ in RECIPES:
        p = Path(a.data) / f"{flight}_{tag}.txt"
        if p.exists():
            data[(flight, tag)] = parse(p)

fig = plt.figure(figsize=(14, 10.5), facecolor=SURF)
gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.05], hspace=0.45, wspace=0.22)
x = np.arange(len(RECIPES))
labels = [l for _, l in RECIPES]
series = (("z", ZER, "Zermatt (rough mountains)", -0.12), ("f1", F1, "flight001 (gentle, smoothed map)", 0.12))

ax = fig.add_subplot(gs[0, 0])
for fl, col, name, off in series:
    ys = [100.0 * data[(fl, t)][1] / data[(fl, t)][2] for t, _ in RECIPES]
    ax.plot(x + off, ys, "o", color=col, ms=9, mec=SURF, mew=1.2, label=name, zorder=3)
    for xi, y in zip(x + off, ys):
        ax.text(xi, y + 3.2, f"{y:.0f}", ha="center", fontsize=8.5, color=INK2)
ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
ax.set_ylim(0, 110); ax.set_ylabel("% of flights under 100 m RMS")
ax.set_title("Accuracy: how many flights stay under 100 m\n(200 flights per point: 10 maps x 20 flights)",
             loc="left", fontsize=10.5)
ax.grid(True, axis="y", color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.legend(loc="lower left", fontsize=9, framealpha=0.9)

ax = fig.add_subplot(gs[0, 1])
for fl, col, name, off in series:
    ys = [max(data[(fl, t)][3], 0.01) for t, _ in RECIPES]
    ax.plot(x + off, ys, "o", color=col, ms=9, mec=SURF, mew=1.2, label=name, zorder=3)
ax.axhline(1.0, color=INK2, ls="--", lw=1)
ax.text(len(RECIPES) - 0.5, 1.15, "REQ-INT-001 limit: 1%", ha="right", fontsize=9, color=INK2)
ax.set_yscale("log"); ax.set_ylim(0.008, 150)
ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
ax.set_ylabel("% of time truth is outside the claimed\n3-sigma circle (log scale; lower is honest)")
ax.set_title("Honesty: is GDN's circle still true when its map is wrong?\n(below the dashed line passes)",
             loc="left", fontsize=10.5)
ax.grid(True, axis="y", color=GRID, lw=0.6, which="major"); ax.set_axisbelow(True)
ax.legend(loc="lower right", fontsize=9, framealpha=0.9)

ax = fig.add_subplot(gs[1, :])
rng = np.random.default_rng(0)
for fl, col, name, off in series:
    for xi, (t, _) in enumerate(RECIPES):
        pm = data[(fl, t)][0]
        jit = rng.uniform(-0.07, 0.07, len(pm))
        ax.scatter(xi + off * 1.6 + jit, pm, s=34, color=col, edgecolor=SURF, linewidth=1, zorder=3,
                   label=name if xi == 0 else None)
ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
ax.set_ylim(-1, 21.5); ax.set_yticks(range(0, 21, 5)); ax.set_ylabel("flights under 100 m, out of 20")
ax.set_title("One map is one draw of map error: each dot is one onboard map (20 flights over it).\n"
             "With 10 m of noise one map lets every flight through and another none, so results are pooled over 10 maps.",
             loc="left", fontsize=10.5)
ax.grid(True, axis="y", color=GRID, lw=0.6); ax.set_axisbelow(True)
ax.legend(loc="lower left", fontsize=9, framealpha=0.9)
fig.savefig(out / "test020_results.png", dpi=120, facecolor=SURF, bbox_inches="tight")
print(f"wrote {out / 'test020_maps.png'} and {out / 'test020_results.png'}")
