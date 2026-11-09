#!/usr/bin/env python3
"""Figures for TEST-023 (R1.7: GNSS loss with a covariance handover, start errors).

  docs/test_reports/test023_flights.png    the 15 min Zermatt flight over the map
  docs/test_reports/test023_gnss_loss.png  error and honesty against time since GNSS loss
  docs/test_reports/test023_start_error.png  acquisition against start error

The figures are drawn from small CSVs in docs/test_reports/test023_data/. Those are
binned from the Monte Carlo traces (about 10 MB each, not committed) with --aggregate:

    python3 tools/plot_gnss_loss.py --aggregate DIR   # DIR: *.trace and *.out of the runs
    python3 tools/plot_gnss_loss.py                   # figures from test023_data/

The runs, their names and options are listed in the TEST-023 report ("Reproduce").
"""
import argparse
import csv
import re
import struct
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
BLUES = ["#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]          # sequential: start error size
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--aggregate", metavar="DIR", help="bin the traces and summaries in DIR into test023_data/")
ap.add_argument("--data", default=str(ROOT / "docs/test_reports/test023_data"))
ap.add_argument("--out-dir", default=str(ROOT / "docs/test_reports"))
a = ap.parse_args()
data, out = Path(a.data), Path(a.out_dir)

LOSS = 150.0
GNSS_SCEN = [  # name, label, colour, trace files (relative to the aggregate dir or absolute)
    ("zl_gnss", "Zermatt 15 min, perfect map", C1, ["zl_gnss.trace"]),
    ("zl_gnss_srtm", "Zermatt 15 min, SRTM-like maps (5 x 20)", C2,
     [str(ROOT / f"data/maps/ensemble/r17_srtm_{k:02d}.csv") for k in range(5)]),
    ("f1_gnss", "flight001 (real IMU), filter as today", C3, ["f1_gnss.trace"]),
    ("f1_gnss_qan0.1", "flight001, accel noise setting 0.1 (not adopted)", C4, ["f1_gnss_qan0.1.trace"]),
]
START_SCEN = [("zl", "Zermatt 15 min", 31.5), ("f1", "flight001", 22.6)]
START_ERR = [0, 60, 150, 500]
LOSS_EDGES = [0, 0.5, 1, 2, 3, 5, 7, 10, 15, 20, 30, 45, 60, 90, 120, 180, 240, 300, 360, 420, 480, 540, 600.01]


def read_trace(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d["seed"], d["t"], d["err"], d["sigma"]


def aggregate(src):
    src = Path(src)
    data.mkdir(parents=True, exist_ok=True)
    # 1. Error and honesty against time since GNSS loss.
    with open(data / "gnss_loss_profile.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "label", "t_lo", "t_hi", "n", "err_p50", "err_p95", "sigma3_p50", "over3sigma_pct"])
        for name, label, _, files in GNSS_SCEN:
            parts = [read_trace(p if Path(p).is_absolute() else src / p) for p in files]
            t = np.concatenate([p[1] for p in parts]) - LOSS
            e = np.concatenate([p[2] for p in parts]); s = np.concatenate([p[3] for p in parts])
            for lo, hi in zip(LOSS_EDGES[:-1], LOSS_EDGES[1:]):
                m = (t >= lo) & (t < hi)
                if m.any():
                    w.writerow([name, label, lo, min(hi, 600), int(m.sum()), f"{np.median(e[m]):.3f}",
                                f"{np.percentile(e[m], 95):.3f}", f"{3 * np.median(s[m]):.3f}",
                                f"{100 * np.mean(e[m] > 3 * s[m]):.3f}"])
    # 2. Error against time since take-off, per start error.
    with open(data / "start_error_profile.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "start_error", "t_lo", "t_hi", "n", "err_p50", "err_p95", "under100_pct"])
        for fl, _, t_to in START_SCEN:
            for se in START_ERR:
                _, t, e, _ = read_trace(src / f"{fl}_s{se}.trace")
                t = t - t_to
                for lo in np.arange(0, 960, 15.0):
                    m = (t >= lo) & (t < lo + 15)
                    if m.any():
                        w.writerow([fl, se, lo, lo + 15, int(m.sum()), f"{np.median(e[m]):.2f}",
                                    f"{np.percentile(e[m], 95):.2f}", f"{100 * np.mean(e[m] < 100):.1f}"])
    # 3. Summaries from the mc_campaign reports.
    gf = re.compile(r"^GNSS-free ([0-9.]+) s: CEP50 ([0-9.]+) m\s+CEP95 ([0-9.]+) m\s+max ([0-9.]+) m\s+"
                    r"over 3 sigma ([0-9.]+)%\s+flights never over 100 m: (\d+)/(\d+)", re.M)
    rows = []
    for name, label, _, _ in GNSS_SCEN:
        txt = (src / f"{name}.out").read_text()
        rows += [[name, label, *m.groups()] for m in gf.finditer(txt)]
    for name in ("f1_gnss_qan0.05", "f1_gnss_qan0.2", "f1_gnss_qan0.1_qba1e-2",
                 "f1_gnss_qba2e-3", "f1_gnss_qba4e-3", "f1_gnss_qba1e-2", "f1_gnss_qba2e-2",
                 "f1_gnss_base_short", "zl_gnss_qan0.1", "zl_gnss_qba4e-3", "zl_gnss_qba1e-2"):
        txt = (src / f"{name}.out").read_text()
        rows += [[name, name, *m.groups()] for m in gf.finditer(txt)]
    with open(data / "gnss_free_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "label", "gnss_free_s", "cep50", "cep95", "max", "over3sigma_pct",
                    "never_over_100", "flights"])
        w.writerows(rows)
    acq = re.compile(r"start error (\d+) m: under 100 m at 120 s after take-off: (\d+)/(\d+) flights "
                     r"\(error there median ([0-9.]+) m\); stayed under to the end \(([0-9.]+) s\): (\d+)/")
    o3 = re.compile(r"error > 3 x published sigma ([0-9.]+)%")
    with open(data / "start_error_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "start_error", "acquired", "flights", "err_at_120s_p50", "end_s", "held",
                    "over3sigma_pct_60_300s"])
        for fl in ("zs", "zl", "f1"):
            for se in START_ERR:
                txt = (src / f"{fl}_s{se}.out").read_text()
                m = acq.search(txt)
                w.writerow([fl, se, m.group(2), m.group(3), m.group(4), m.group(5), m.group(6), o3.search(txt).group(1)])
    # 4. Tracks, 1 s.
    with open(data / "tracks.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "t", "x", "y"])
        for fl, d in (("zermatt_15min", "flight_zermatt_long_csv"), ("zermatt_177s", "flight_zermatt_csv")):
            r = list(csv.DictReader(open(ROOT / "data" / d / "lpos.csv")))
            t0 = float(r[0]["t"])
            for x in r[::10]:
                w.writerow([fl, f"{(float(x['t']) - t0) * 1e-6:.1f}", f"{float(x['x']):.1f}", f"{float(x['y']):.1f}"])
    print(f"wrote {data}/gnss_loss_profile.csv, start_error_profile.csv, gnss_free_summary.csv, "
          f"start_error_summary.csv, tracks.csv")


if a.aggregate:
    aggregate(a.aggregate)

plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})


def style(ax):
    ax.grid(True, which="major", color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


# ------------------------------------------------------------- flights on the map ----
with open(ROOT / "data/maps/zermatt.bin", "rb") as f:
    nx, ny = struct.unpack("<ii", f.read(8))
    lat0, lon0, dlat, dlon = struct.unpack("<dddd", f.read(32))
    alt = np.frombuffer(f.read(4 * nx * ny), dtype="<f4").reshape(ny, nx)
lat_c = lat0 + 0.5 * ny * dlat
dn, de = dlat * 111320.0, dlon * 111320.0 * np.cos(np.radians(lat_c))
north = (np.arange(ny) - ny / 2) * dn
east = (np.arange(nx) - nx / 2) * de
if dn < 0:                     # rows run north to south: flip so north increases with row
    north, alt = north[::-1], alt[::-1]
tr = defaultdict(lambda: {"t": [], "x": [], "y": []})
for r in csv.DictReader(open(data / "tracks.csv")):
    d = tr[r["flight"]]
    d["t"].append(float(r["t"])); d["x"].append(float(r["x"])); d["y"].append(float(r["y"]))
fig, ax = plt.subplots(figsize=(7.2, 8.6), facecolor=SURF)
ext = [east.min(), east.max(), north.min(), north.max()]
im = ax.imshow(alt, extent=ext, origin="lower", cmap="Greys", alpha=0.55)
ax.contour(east, north, alt, levels=np.arange(2200, 3500, 100), colors=INK2, linewidths=0.3, alpha=0.5)
L = tr["zermatt_15min"]; S = tr["zermatt_177s"]
t = np.array(L["t"]); x = np.array(L["x"]); y = np.array(L["y"])
on = t <= LOSS
ax.plot(y[on], x[on], color=C1, lw=1.2, ls=(0, (2, 1.5)), label="15 min flight, GNSS on (first 150 s)")
ax.plot(y[~on], x[~on], color=C1, lw=1.8, label="15 min flight, GNSS lost: 10 min scored")
ax.plot(S["y"], S["x"], color=C2, lw=1.8, label="original 177 s flight")
k = np.searchsorted(t, LOSS)
ax.plot(y[k], x[k], "o", ms=9, color=C1, mec=SURF, mew=1.5, zorder=5)
ax.annotate("GNSS lost (t = 150 s)", (y[k], x[k]), xytext=(12, -14), textcoords="offset points",
            fontsize=9, color=INK)
ax.plot(0, 0, "s", ms=8, color=INK, mec=SURF, mew=1.2, zorder=5)
ax.annotate("take-off", (0, 0), xytext=(8, 6), textcoords="offset points", fontsize=9, color=INK)
ax.set_xlabel("east [m]"); ax.set_ylabel("north [m]")
ax.set_title("Zermatt flights over the map (1 arc-second DEM, darker = higher)", loc="left", fontsize=11)
cb = fig.colorbar(im, ax=ax, shrink=0.6, pad=0.02); cb.set_label("terrain [m]", color=INK2)
cb.outline.set_edgecolor(GRID)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=1, fontsize=8.5, frameon=False)
ax.set_aspect("equal")
fig.savefig(out / "test023_flights.png", dpi=120, facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ------------------------------------------------------------ after GNSS loss ----
# Time axis: logarithmic up to 1 minute (the handover), linear from 1 to 10 minutes.
T_SPLIT, T_LIN = 60.0, 230.0      # seconds per log-decade-equivalent in the linear part


def _fwd(t):
    t = np.maximum(np.asarray(t, float), 1e-3)
    return np.where(t < T_SPLIT, np.log10(t), np.log10(T_SPLIT) + (t - T_SPLIT) / T_LIN)


def _inv(u):
    u = np.asarray(u, float)
    return np.where(u < np.log10(T_SPLIT), 10 ** u, T_SPLIT + (u - np.log10(T_SPLIT)) * T_LIN)


prof = defaultdict(list)
for r in csv.DictReader(open(data / "gnss_loss_profile.csv")):
    prof[r["scenario"]].append(r)
fig, axs = plt.subplots(2, 4, figsize=(16, 7.6), facecolor=SURF, sharex=True,
                        gridspec_kw={"height_ratios": [1.5, 1], "hspace": 0.12, "wspace": 0.1})
for col, (name, label, c, _) in enumerate(GNSS_SCEN):
    rr = prof[name]
    tc = np.array([np.sqrt(max(float(r["t_lo"]), 0.25) * float(r["t_hi"])) if float(r["t_hi"]) <= T_SPLIT
                   else (float(r["t_lo"]) + float(r["t_hi"])) / 2 for r in rr])
    p50 = np.array([float(r["err_p50"]) for r in rr]); p95 = np.array([float(r["err_p95"]) for r in rr])
    s3 = np.array([float(r["sigma3_p50"]) for r in rr]); o3 = np.array([float(r["over3sigma_pct"]) for r in rr])
    ax = axs[0, col]
    ax.fill_between(tc, p50, p95, color=c, alpha=0.18, lw=0, label="median to 95th percentile")
    ax.plot(tc, p50, color=c, lw=2, label="median error")
    ax.plot(tc, s3, color=INK, lw=1.1, ls="--", label="claimed 3 sigma (median)")
    ax.axhline(100, color=INK2, lw=0.8, ls=":")
    ax.set_yscale("log"); ax.set_ylim(0.1, 3000)
    ax.set_title(label, loc="left", fontsize=10)
    style(ax)
    if col == 0:
        ax.set_ylabel("horizontal error [m] (log)")
        ax.text(0.35, 115, "100 m: fixed-wing max (REQ-NAV-003)", fontsize=8, color=INK2)
    else:
        ax.tick_params(labelleft=False)
    ax2 = axs[1, col]
    ax2.plot(tc, np.maximum(o3, 0.01), "o-", color=c, ms=4.5, lw=1.4, mec=SURF, mew=0.8)
    ax2.axhline(1.0, color=INK2, lw=1, ls="--")
    ax2.set_yscale("log"); ax2.set_ylim(0.008, 150)
    style(ax2)
    if col == 0:
        ax2.set_ylabel("% outside the claimed\n3 sigma (log; 0 drawn at 0.01)")
        ax2.text(0.35, 1.25, "1%: REQ-INT-001", fontsize=8, color=INK2)
    else:
        ax2.tick_params(labelleft=False)
    for axx in (ax, ax2):
        axx.set_xscale("function", functions=(_fwd, _inv)); axx.set_xlim(0.25, 600)
        axx.axvline(T_SPLIT, color=GRID, lw=2.5, zorder=0)
        for v in (300, 600):
            axx.axvline(v, color=INK2, lw=0.7, alpha=0.6)
        axx.grid(False, which="minor")
    ax2.set_xticks([1, 10, 60, 180, 300, 420, 600])
    ax2.set_xticklabels(["1 s", "10 s", "1 min", "3", "5", "7", "10 min"])
    ax2.set_xlabel("time since GNSS was lost")
h, l = axs[0, 0].get_legend_handles_labels()
h = [plt.Line2D([], [], color=INK2, lw=2), plt.Rectangle((0, 0), 1, 1, color=INK2, alpha=0.18),
     plt.Line2D([], [], color=INK, lw=1.1, ls="--")]
fig.legend(h, ["median error (colour: scenario)", "median to 95th percentile",
               "what the filter claims: 3 sigma (median)"],
           loc="lower center", ncol=3, fontsize=9, frameon=False, bbox_to_anchor=(0.5, -0.03))
fig.text(0.01, 0.955, "GNSS lost at t = 150 s, 100 flights per scenario, binned by time since the loss. "
         "Time axis: log up to 1 min, linear after. Above the dashed line the error is bigger than the filter says.", fontsize=9.5, color=INK2)
fig.savefig(out / "test023_gnss_loss.png", dpi=120, facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ------------------------------------------------------------- start errors ----
summ = list(csv.DictReader(open(data / "start_error_summary.csv")))
sp = defaultdict(list)
for r in csv.DictReader(open(data / "start_error_profile.csv")):
    sp[(r["flight"], int(r["start_error"]))].append(r)
fig, axs = plt.subplots(1, 3, figsize=(16, 5.2), facecolor=SURF, gridspec_kw={"wspace": 0.22,
                                                                                 "width_ratios": [1, 1.25, 1.25]})
ax = axs[0]
for fl, label, c in (("zs", "Zermatt 177 s", C2), ("zl", "Zermatt 15 min", C1), ("f1", "flight001", C3)):
    rr = [r for r in summ if r["flight"] == fl]
    xs = [int(r["start_error"]) for r in rr]; ys = [int(r["acquired"]) for r in rr]
    ax.plot(xs, ys, "o-", color=c, lw=1.8, ms=7, mec=SURF, mew=1.2, label=label)
ax.set_xscale("symlog", linthresh=60); ax.set_xticks(START_ERR); ax.set_xticklabels([f"{v} m" for v in START_ERR])
ax.set_xlim(-5, 700); ax.set_ylim(-3, 105)
ax.set_xlabel("start error (filter told sigma = max(error, 10 m))")
ax.set_ylabel("flights under 100 m at 120 s after take-off, of 100")
ax.set_title("REQ-ACQ-001 asks for all of them at 500 m", loc="left", fontsize=10.5)
ax.legend(loc="lower left", fontsize=8.5, frameon=False)
style(ax)
for k, (fl, label, _) in enumerate(START_SCEN):
    ax = axs[k + 1]
    for se, c in zip(START_ERR, BLUES):
        rr = sp[(fl, se)]
        tc = np.array([(float(r["t_lo"]) + float(r["t_hi"])) / 2 for r in rr])
        ax.plot(tc / 60, [float(r["err_p50"]) for r in rr], color=c, lw=1.9, label=f"start error {se} m")
    ax.axhline(100, color=INK2, lw=0.8, ls=":")
    ax.axvline(2.0, color=INK2, lw=0.8, ls="--")
    ax.text(2.1, 0.03, "120 s after take-off", fontsize=8, color=INK2, transform=ax.get_xaxis_transform())
    ax.set_yscale("log"); ax.set_ylim(1, 5000)
    ax.set_xlabel("minutes since take-off")
    ax.set_ylabel("median horizontal error [m] (log)")
    ax.set_title(f"{label}: median error per start error (no GNSS)", loc="left", fontsize=10.5)
    style(ax)
h_se, l_se = axs[1].get_legend_handles_labels()
fig.legend(h_se, l_se, loc="lower center", ncol=4, fontsize=9, frameon=False, bbox_to_anchor=(0.62, -0.06))
fig.savefig(out / "test023_start_error.png", dpi=120, facecolor=SURF, bbox_inches="tight")
print(f"wrote {out}/test023_flights.png, test023_gnss_loss.png, test023_start_error.png")
