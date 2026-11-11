#!/usr/bin/env python3
"""Figures for TEST-025 (R1.6 hardware: the Mid-360S and OAK-D Lite IMU fits).

  docs/test_reports/test025_allan.png  Allan deviation of the three IMUs with their models
  docs/test_reports/test025_bench.png  the two bench logs over time: warm-up, cut, break

The figures are drawn from small CSVs in docs/test_reports/test025_data/. Those come from
the static logs (log/mid360s_static, 525 MB; log/oakd_lite_static5, 160 MB; not committed,
checksums in log_sha256.txt) and the Monte Carlo outputs, with --aggregate:

    python3 tools/plot_test025.py --aggregate MCDIR    # MCDIR: mc_{loss,srtml}_{h30wp,mid360s,oakd_lite}.txt
    python3 tools/plot_test025.py                      # figures from test025_data/

The runs and their options are listed in the TEST-025 report ("Reproduce").
"""
import argparse
import csv
import datetime
import hashlib
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

sys.path.insert(0, str(Path(__file__).resolve().parent))
import allan_fit as af  # noqa: E402

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
DATA = ROOT / "docs/test_reports/test025_data"
OUT = ROOT / "docs/test_reports"
# name, label, colour, log, how to load it, warm-up dropped (s), clock time of the first sample
SENSORS = (
    ("h30wp", "H30WP", C1, None, None, 900.0, None),
    ("mid360s", "Mid-360S", C2, "log/mid360s_static", ("bag", "/livox/imu", 9.80665), 2400.0, "11:55:20"),
    ("oakd_lite", "OAK-D Lite", C3, "log/oakd_lite_static5/imu.csv", ("csv",), 1200.0, "10:18:13"),
)


def load(spec, path):
    if spec[0] == "bag":
        t, a, g = af.load_bag(str(ROOT / path), spec[1])
        return t, a * spec[2], g
    return af.load_csv(str(ROOT / path))


def aggregate(mc_dir):
    DATA.mkdir(parents=True, exist_ok=True)
    fits = [["sensor", "kind", "N_x", "N_y", "N_z", "K", "tau_s", "drift_fit", "hours", "rate_hz", "dt_spread",
             "cut_at_s", "cut_what"]]
    sha = []
    for name, _, _, log, spec, skip, clock in SENSORS:
        if log is None:
            continue
        t, a, g = load(spec, log)
        end, cut = af.first_segment(t)
        res = af.fit_log(t, a, g, skip)
        for kind in ("accel", "gyro"):
            r = res[kind]
            fits.append([name, kind, *(f"{v:.5g}" for v in r["N"]), f"{r['K']:.5g}", f"{r['tc']:.4g}",
                         "envelope" if r["envelope"] else "least squares", f"{res['hours']:.3f}",
                         f"{res['fs']:.2f}", f"{res['dt_spread']:.3f}", f"{cut['at']:.0f}", cut["what"]])
        af.write_adev_csv(DATA / f"adev_{name}.csv", res)
        # one-minute means over the whole first segment, to show the warm-up and the drift
        t, a, g = t[:end], a[:end], g[:end]
        m = ((t - t[0]) // 60).astype(int)
        t0 = datetime.datetime.strptime(clock, "%H:%M:%S")
        with open(DATA / f"bench_minute_means_{name}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["minute", "clock", "ax_mps2", "ay_mps2", "az_mps2", "gx_mdegps", "gy_mdegps", "gz_mdegps",
                        "gyro_spread_mdegps"])
            for k in np.unique(m):
                s = m == k
                gd = np.degrees(g[s]) * 1e3
                w.writerow([int(k), (t0 + datetime.timedelta(minutes=int(k))).strftime("%H:%M"),
                            *(f"{v:.6f}" for v in a[s].mean(0)), *(f"{v:.3f}" for v in gd.mean(0)),
                            f"{np.max(gd.std(0)):.2f}"])
        for p in sorted((ROOT / log).parent.glob("*") if log.endswith(".csv") else (ROOT / log).glob("*")):
            sha.append(f"{hashlib.sha256(p.read_bytes()).hexdigest()}  {p.relative_to(ROOT)}")
        print(f"aggregated {name}")
    with open(DATA / "fits.csv", "w", newline="") as f:
        csv.writer(f).writerows(fits)
    (DATA / "log_sha256.txt").write_text("\n".join(sha) + "\n")
    # Monte Carlo: Zermatt 15 min, GNSS lost at 150 s, perfect and SRTM-like maps
    rx = re.compile(r"GNSS-free (\d+) s: CEP50 ([\d.]+) m\s+CEP95 ([\d.]+) m\s+max ([\d.]+) m\s+over 3 sigma "
                    r"([\d.]+)%\s+flights never over 100 m: (\d+)/(\d+)")
    rows = [["scenario", "imu", "gnss_free_s", "cep50", "cep95", "max", "over3sigma_pct", "never_over_100", "flights"]]
    for tag, scen in (("loss", "Zermatt 15 min, GNSS lost at 150 s"),
                      ("srtml", "Zermatt 15 min, GNSS lost at 150 s, SRTM-like maps")):
        for name, *_ in SENSORS:
            for mm in rx.finditer((Path(mc_dir) / f"mc_{tag}_{name}.txt").read_text()):
                rows.append([scen, name, *mm.groups()])
    with open(DATA / "mc_imu_compare.csv", "w", newline="") as f:
        csv.writer(f).writerows(rows)
    print(f"wrote {DATA}")


def model(name):
    import yaml
    m = yaml.safe_load(open(ROOT / f"config/imu/{name}.yaml"))
    v = lambda k: float(m[k])
    return ({"N": np.mean([float(x) for x in m["accel_noise_density"]]), "K": v("accel_bias_rw"),
             "tc": v("accel_bias_tau")},
            {"N": np.mean([float(x) for x in m["gyro_noise_density"]]), "K": v("gyro_bias_rw"),
             "tc": v("gyro_bias_tau")})


def read_adev(path):
    rows = list(csv.DictReader(open(path)))
    out = {}
    for kind in ("accel", "gyro"):
        r = [x for x in rows if x["sensor"] == kind]
        tau = np.array([float(x.get("tau_s") or x["tau"]) for x in r])
        ad = np.sqrt(np.mean([[float(x[f"adev_{a}"]) ** 2 for a in "xyz"] for x in r], axis=1))
        out[kind] = (tau, ad)
    return out


def style(ax):
    ax.set_facecolor(SURF)
    ax.grid(True, which="both", color=GRID, lw=0.5)
    for s in ax.spines.values():
        s.set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8.5)


def plot_allan():
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8), facecolor=SURF)
    for ax, kind, unit, scale in ((axs[0], "accel", "µg", 1 / 9.80665e-6), (axs[1], "gyro", "°/h", 180 / np.pi * 3600)):
        style(ax)
        ax.axvspan(*af.MISSION_BAND, color=GRID, alpha=0.45, lw=0)
        ax.text(np.sqrt(af.MISSION_BAND[0] * af.MISSION_BAND[1]), 0.97, "30 s to 10 min\n(GNSS-free windows)",
                transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=8, color=INK2)
        for name, label, col, *_ in SENSORS:
            src = (ROOT / "docs/test_reports/test024_data/adev_h30wp.csv") if name == "h30wp" else DATA / f"adev_{name}.csv"
            tau, ad = read_adev(src)[kind]
            ax.loglog(tau, ad * scale, "o", ms=3.2, color=col, mec=SURF, mew=0.3, label=f"{label} (measured, axis rms)")
            p = model(name)[0 if kind == "accel" else 1]
            tt = np.logspace(np.log10(tau[0]), np.log10(max(tau[-1], af.MISSION_BAND[1])), 200)
            ax.loglog(tt, af.model_adev(p["N"], p["K"], p["tc"], tt) * scale, "-", color=col, lw=1.6,
                      label=f"{label} model")
        ax.set_xlabel("averaging time τ [s]", color=INK2, fontsize=9)
        ax.set_ylabel(f"Allan deviation [{unit}]", color=INK2, fontsize=9)
        ax.set_title(f"{kind}", loc="left", fontsize=10, color=INK)
    h, lab = axs[0].get_legend_handles_labels()
    fig.legend(h, lab, ncol=3, loc="lower center", bbox_to_anchor=(0.5, -0.1), fontsize=8.5, frameon=False)
    fig.suptitle("TEST-025: Allan deviation of the three IMUs. Lower is better; the models (lines) are what the "
                 "Monte Carlo and the Gazebo sensors use", x=0.01, ha="left", color=INK2, fontsize=10.5)
    fig.savefig(OUT / "test025_allan.png", dpi=120, facecolor=SURF, bbox_inches="tight")


def plot_bench():
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4), facecolor=SURF, gridspec_kw={"wspace": 0.18})
    for ax, (name, label, _, log, _, skip, clock) in zip(axs, SENSORS[1:]):
        style(ax)
        rows = list(csv.DictReader(open(DATA / f"bench_minute_means_{name}.csv")))
        m = np.array([int(r["minute"]) for r in rows])
        g = np.array([[float(r[f"g{a}_mdegps"]) for a in "xyz"] for r in rows])
        ref = g[m >= skip / 60].mean(0)
        ax.axvspan(0, skip / 60, color=GRID, alpha=0.55, lw=0)
        ax.text(skip / 120, 0.97, f"warm-up,\ndropped ({skip / 60:.0f} min)", transform=ax.get_xaxis_transform(),
                ha="center", va="top", fontsize=8, color=INK2)
        for i, (a, col) in enumerate(zip("xyz", (C1, C2, C3))):
            ax.plot(m, g[:, i] - ref[i], "-", color=col, lw=1.6, label=f"gyro {a}")
        ax.axvline(m[-1] + 1, color=INK2, lw=1.0, ls="--")
        cut = [r for r in csv.DictReader(open(DATA / "fits.csv")) if r["sensor"] == name][0]["cut_what"]
        ax.text(m[-1] - 1.0, 0.97, f"log breaks here:\n{cut}", transform=ax.get_xaxis_transform(), ha="right",
                va="top", fontsize=8, color=INK2)
        ax.set_xlabel(f"minutes from {clock[:5]}", color=INK2, fontsize=9)
        ax.set_ylabel("gyro offset minus its fitted-part mean\n[millidegrees/s, 1-minute means]", color=INK2,
                      fontsize=9)
        ax.set_title(label, loc="left", fontsize=10, color=INK)
        ax.legend(fontsize=8, frameon=False, loc="lower center", ncol=3)
    fig.suptitle("TEST-025: the bench logs. The warm-up is dropped; each fit stops at the log's first break",
                 x=0.01, ha="left", color=INK2, fontsize=10.5)
    fig.savefig(OUT / "test025_bench.png", dpi=120, facecolor=SURF, bbox_inches="tight")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aggregate", metavar="MCDIR", help="rebuild test025_data/ from the logs and MC outputs")
    a = ap.parse_args()
    if a.aggregate:
        aggregate(a.aggregate)
    plot_allan()
    plot_bench()
    print(f"wrote {OUT / 'test025_allan.png'} and {OUT / 'test025_bench.png'}")


if __name__ == "__main__":
    main()
