#!/usr/bin/env python3
"""Figures for TEST-022 (R1.6, IMU error model).

  docs/test_reports/test022_allan.png  Allan deviation: simulated 2 h static logs, the
                                       fit, the true model, and what the filter assumes
  docs/test_reports/test022_mc.png     Monte Carlo per IMU model, and the filter's
                                       accel-bias random walk sweep

Inputs: docs/test_reports/test022_data/ (allan_*.csv and fitted_*.yaml from
tools/allan_fit.py, mc_imu_models.csv and q_accel_bias_sweep.csv from the MC runs in the
report).

    python3 tools/plot_imu_model.py
"""
import argparse
import csv
import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT
from allan_fit import model_adev

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"

ap = argparse.ArgumentParser()
ap.add_argument("--data", default=str(ROOT / "docs/test_reports/test022_data"))
ap.add_argument("--out-dir", default=str(ROOT / "docs/test_reports"))
a = ap.parse_args()
data, out = Path(a.data), Path(a.out_dir)
plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.titlecolor": INK})


def load_model(path):
    m = {}
    for line in open(path):
        line = line.split("#")[0].strip()
        if ":" not in line:
            continue
        k, v = (s.strip() for s in line.split(":", 1))
        nums = re.findall(r"[-+0-9.eE]+", v)
        m[k] = [float(x) for x in nums] if v.startswith("[") else (float(nums[0]) if k not in ("name", "source") else v)
    return m


def load_adev(path):
    rows = list(csv.DictReader(open(path)))
    out = {}
    for s in ("accel", "gyro"):
        r = [x for x in rows if x["sensor"] == s]
        out[s] = (np.array([float(x["tau"]) for x in r]),
                  np.array([np.sqrt(np.mean([float(x[f"adev_{c}"]) ** 2 for c in "xyz"])) for x in r]))
    return out


cfg = ROOT / "config/imu"
true_m = {"placeholder": load_model(cfg / "placeholder_px4_default.yaml"),
          "drift": load_model(cfg / "sensitivity_bias_drift.yaml")}
fit_m = {"placeholder": load_model(data / "fitted_placeholder.yaml"),
         "drift": load_model(data / "fitted_bias_drift.yaml")}
adev = {"placeholder": load_adev(data / "allan_placeholder.csv"),
        "drift": load_adev(data / "allan_bias_drift.csv")}
# What the filter assumes today (EskfConfig): white noise plus a pure bias random walk.
FILTER = {"accel": (0.02, 5e-4), "gyro": (0.02 * np.pi / 180, 0.01 * np.pi / 180)}

# ---------------------------------------------------------------- Allan figure ----
fig, axs = plt.subplots(1, 2, figsize=(14, 5.4), facecolor=SURF, gridspec_kw={"wspace": 0.25})
for ax, s, unit in ((axs[0], "accel", "m/s$^2$"), (axs[1], "gyro", "rad/s")):
    for key, c, name in (("placeholder", C1, "placeholder (PX4 default: white noise only)"),
                         ("drift", C2, "placeholder + PX4's pre-2025 bias drift")):
        t, y = adev[key][s]
        ax.loglog(t, y, "o", ms=4, color=c, mec=SURF, mew=0.6, zorder=3, label=f"{name}: simulated 2 h log")
        tt = np.logspace(np.log10(t[0]), np.log10(t[-1]), 300)
        f, tr = fit_m[key], true_m[key]
        ax.loglog(tt, model_adev(np.sqrt(np.mean(np.square(f[f"{s}_noise_density"]))), f[f"{s}_bias_rw"],
                                 f[f"{s}_bias_tau"], tt), "-", color=c, lw=1.8, zorder=2,
                  label="its Allan fit (tools/allan_fit.py)")
        ax.loglog(tt, model_adev(np.sqrt(np.mean(np.square(tr[f"{s}_noise_density"]))), tr[f"{s}_bias_rw"],
                                 tr[f"{s}_bias_tau"], tt), "--", color=INK, lw=0.9, zorder=4,
                  label="true model" if key == "drift" else None)
    t = adev["placeholder"][s][0]
    tt = np.logspace(np.log10(t[0]), np.log10(t[-1]), 300)
    qn, qk = FILTER[s]
    ax.loglog(tt, np.sqrt(qn ** 2 / tt + qk ** 2 * tt / 3.0), "-", color=C3, lw=2.2, zorder=1,
              label="what the filter assumes today (EskfConfig)")
    ax.set_xlabel("averaging time [s]")
    ax.set_ylabel(f"Allan deviation [{unit}]")
    ax.grid(True, which="major", color=GRID, lw=0.6)
    ax.grid(True, which="minor", color=GRID, lw=0.3, alpha=0.6)
    ax.set_axisbelow(True)
for ax in axs:
    ax.set_xlim(0.008, 1000)
axs[0].set_title("Accelerometer: above the green line, the filter underestimates the error", loc="left", fontsize=11)
axs[1].set_title("Gyro: the filter's assumption covers both models", loc="left", fontsize=11)
axs[0].text(150, 0.0012, "past about 6 s the drift\noutgrows the filter's\nbias allowance", color=INK2, fontsize=8.5, ha="center")
h, l = axs[0].get_legend_handles_labels()
fig.legend(h, l, loc="lower center", ncol=3, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, -0.07))
fig.savefig(out / "test022_allan.png", dpi=120, facecolor=SURF, bbox_inches="tight")
plt.close(fig)

# ------------------------------------------------------------------- MC figure ----
rows = list(csv.DictReader(open(data / "mc_imu_models.csv")))
cases = []
for r in rows:
    if r["case"] not in [c[0] for c in cases]:
        q = "" if r["filter_q"] == "tuned" else "; filter Q matched"
        cases.append((r["case"], r["label"] + q))
fig = plt.figure(figsize=(14, 10), facecolor=SURF)
gs = fig.add_gridspec(2, 2, hspace=0.42, wspace=0.08, height_ratios=[1.1, 1], bottom=0.1)
yy = np.arange(len(cases))[::-1]
for col, key, xlabel, title in ((0, "passing", "flights under 100 m RMS, of 100",
                                 "Accuracy: flights staying under 100 m"),
                                (1, "over3sigma_pct", "% of time truth is outside the claimed 3-sigma circle (log)",
                                 "Honesty: left of the dashed line passes REQ-INT-001")):
    ax = fig.add_subplot(gs[0, col])
    for scen, c, name, off in (("zermatt_truth_map", C1, "perfect onboard map", 0.13),
                               ("zermatt_srtm_maps", C2, "SRTM-like onboard maps (5 x 20)", -0.13)):
        for yi, (k, _) in zip(yy, cases):
            r = next((x for x in rows if x["case"] == k and x["scenario"] == scen), None)
            if not r:
                continue
            v = max(float(r[key]), 0.01)
            ax.plot(v, yi + off, "o", color=c, ms=8, mec=SURF, mew=1.2, zorder=3,
                    label=name if k == "placeholder" else None)
            ax.text(v * (1.12 if key != "passing" else 1) + (0.6 if key == "passing" else 0), yi + off,
                    f"{float(r[key]):.0f}" if key == "passing" else f"{float(r[key]):.2f}%",
                    va="center", fontsize=8, color=INK2)
    ax.set_yticks(yy)
    ax.set_yticklabels([lab for _, lab in cases] if col == 0 else [])
    ax.grid(True, axis="x", color=GRID, lw=0.6); ax.set_axisbelow(True)
    ax.set_xlabel(xlabel); ax.set_title(title, loc="left", fontsize=11)
    if key == "passing":
        ax.set_xlim(84, 102); ax.set_xticks(range(85, 101, 5))
        ax.legend(loc="lower left", fontsize=8.5, framealpha=0.9)
    else:
        ax.set_xscale("log"); ax.set_xlim(0.2, 40)
        ax.axvline(1.0, color=INK2, ls="--", lw=1)
    ax.axhline(yy[3] - 0.5, color=INK2, lw=0.8)
    if col == 1:
        ax.text(38, yy[3] - 0.38, "no bias drift above, drift below", ha="right", va="bottom", fontsize=8, color=INK2)

sw = list(csv.DictReader(open(data / "q_accel_bias_sweep.csv")))
for col, key, ylab, title, logy in ((0, "cep95", "CEP95 [m] (log)", "Worst case: CEP95 against the filter's accel-bias allowance", True),
                                    (1, "over3sigma_pct", "% outside 3-sigma (log)", "Honesty against the same setting", True)):
    ax = fig.add_subplot(gs[1, col])
    for scen, c, name, mk in (("zermatt_placeholder", C1, "Zermatt, placeholder IMU", "o-"),
                              ("zermatt_bias_drift", C2, "Zermatt, placeholder + bias drift", "o-"),
                              ("flight001_recorded", C3, "flight001, real recorded IMU", "o-"),
                              ("srtm_placeholder", C1, "Zermatt SRTM-like maps, placeholder", "s--"),
                              ("srtm_bias_drift", C2, "Zermatt SRTM-like maps, + drift", "s--")):
        r = [x for x in sw if x["scenario"] == scen]
        q = np.array([float(x["q_accel_bias_rw"]) for x in r])
        v = np.maximum([float(x[key]) for x in r], 0.01)
        ax.plot(q, v, mk, color=c, lw=1.8 if "-" == mk[-1] else 1.2, ms=6.5 if mk[0] == "o" else 6,
                mfc=c if mk[0] == "o" else SURF, mec=c if mk[0] == "s" else SURF, mew=1.2, zorder=3, label=name)
    ax.set_xscale("log")
    if logy:
        ax.set_yscale("log")
    ax.axvline(5e-4, color=INK2, ls=":", lw=1)
    ax.text(4.6e-4, 0.5, "today's default", fontsize=8, color=INK2, rotation=90, ha="right", va="center",
            transform=ax.get_xaxis_transform())
    ax.set_xlim(2.5e-4, 2.8e-2)
    if key == "over3sigma_pct":
        ax.axhline(1.0, color=INK2, ls="--", lw=1)
        ax.set_ylim(0.008, 40)
        ax.text(2.7e-4, 1.15, "1% limit", ha="left", fontsize=8.5, color=INK2)
    ax.set_xlabel("filter accel-bias random walk q_accel_bias_rw [m/s$^3$/$\\sqrt{Hz}$] (log)")
    ax.set_ylabel(ylab)
    ax.grid(True, which="major", color=GRID, lw=0.6); ax.set_axisbelow(True)
    ax.set_title(title, loc="left", fontsize=11)
    if col == 0:
        h_sw, l_sw = ax.get_legend_handles_labels()
fig.legend(h_sw, l_sw, loc="lower center", ncol=3, fontsize=8.5, frameon=False, bbox_to_anchor=(0.5, 0.0))
fig.text(0.01, 0.925, "Zermatt node start-up, 100 flights per point (flight001: CI configuration). "
         "flight001 at 0.00% is drawn at 0.01%.", fontsize=9, color=INK2)
fig.savefig(out / "test022_mc.png", dpi=120, facecolor=SURF, bbox_inches="tight")
print(f"wrote {out / 'test022_allan.png'} and {out / 'test022_mc.png'}")
