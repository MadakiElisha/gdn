#!/usr/bin/env python3
"""Figures for TEST-024 (R1.6 hardware: the H30WP's Allan fit and the filter tuned to it).

  docs/test_reports/test024_bench.png  Allan deviation with both fits, and the bias over the recording
  docs/test_reports/test024_mc.png     Monte Carlo before and after, per scenario

The figures are drawn from small CSVs in docs/test_reports/test024_data/. Those come from
the static log (log/h30wp_static, 280 MB, not committed; checksums in log_sha256.txt) and
the Monte Carlo outputs, with --aggregate:

    python3 tools/plot_h30wp.py --aggregate log/h30wp_static MCDIR   # MCDIR: the runs' *.out
    python3 tools/plot_h30wp.py                                      # figures from test024_data/

The runs, their names and options are listed in the TEST-024 report ("Reproduce").
"""
import argparse
import csv
import hashlib
import re
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker

from gdn_paths import ROOT

sys.path.insert(0, str(Path(__file__).resolve().parent))
import allan_fit as af  # noqa: E402

C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
BEFORE = "#86b6ef"
DATA = ROOT / "docs/test_reports/test024_data"
OUT = ROOT / "docs/test_reports"
SKIP_S = 900.0

# Monte Carlo rows: label, before (main: placeholder IMU, gyro-bias random walk 0.01),
# after (this PR: H30WP model, 1e-3). "ref:" takes a row from an earlier report's data.
MC_ROWS = [
    ("Zermatt 177 s, from take-off", "z_ph", None, "fin_z", None),
    ("Zermatt 177 s, SRTM-like maps", "ref:test022:placeholder:zermatt_srtm_maps", None, "fin_srtm", None),
    ("Zermatt 15 min, 5 min after GNSS loss", "zl_ph", 300, "fin_zl", 300),
    ("Zermatt 15 min, 10 min after GNSS loss", "zl_ph", 600, "fin_zl", 600),
    ("Zermatt 15 min, SRTM-like, 10 min after loss", "ref:test023:zl_gnss_srtm:600", 600, "fin_srtml", 600),
    ("flight001 (real IMU), from take-off", "f1_def", None, "fin_f1", None),
    ("flight001, 5 min after GNSS loss", "f1g_def", 300, "fin_f1g", 300),
    ("flight001, 10 min after GNSS loss", "f1g_def", 600, "fin_f1g", 600),
]
# The sweep behind the choice: one setting at a time on the H30WP model, then together.
SWEEP = [
    ("defaults before (gyro bias rw 0.01)", ""), ("all four matched to the H30WP (--q-imu=match)", "m"),
    ("gyro white noise matched (--q-gn=5.4e-3)", "_qgn"), ("accel bias rw 1e-4", "_qba1e-4"),
    ("accel bias rw 2e-3", "_qba2e-3"), ("accel white noise 0.005", "_qan0.005"), ("accel white noise 0.1", "_qan0.1"),
    ("gyro bias rw 3e-3", "_qbg3e-3"), ("gyro bias rw 1e-3 (adopted)", "_qbg1e-3"),
    ("gyro bias rw 2.26e-4 (= the fit)", "_qbg2.26e-4"),
    ("gyro bias rw 1e-3 + accel white 0.1", "_bg1e-3_an0.1"), ("gyro bias rw 1e-3 + accel white 0.2", "_bg1e-3_an0.2"),
]


# ------------------------------------------------------------------ aggregate ----
def parse_out(path, window=None):
    txt = Path(path).read_text()
    if window is None:
        m = re.findall(r"CEP50: ([\d.]+) m\s+CEP95: ([\d.]+) m\s+MAX: ([\d.]+) m", txt)[-1]
        p = re.findall(r"passing\(<100 m\): (\d+)/(\d+)", txt)[-1]
        s = re.findall(r"error > 3 x published sigma ([\d.]+)%", txt)[-1]
        return {"cep50": m[0], "cep95": m[1], "max": m[2], "over3sigma_pct": s, "ok": p[0], "flights": p[1]}
    m = re.search(rf"GNSS-free {window} s: CEP50 ([\d.]+) m\s+CEP95 ([\d.]+) m\s+max ([\d.]+) m\s+over 3 sigma "
                  rf"([\d.]+)%\s+flights never over 100 m: (\d+)/(\d+)", txt)
    return {"cep50": m[1], "cep95": m[2], "max": m[3], "over3sigma_pct": m[4], "ok": m[5], "flights": m[6]}


def ref_row(spec):
    _, test, key, extra = spec.split(":", 3) if spec.count(":") == 3 else (*spec.split(":", 2), None)
    if test == "test022":
        for r in csv.DictReader(open(ROOT / "docs/test_reports/test022_data/mc_imu_models.csv")):
            if r["case"] == key and r["scenario"] == extra:
                return {"cep50": r["cep50"], "cep95": r["cep95"], "max": "", "over3sigma_pct": r["over3sigma_pct"],
                        "ok": r["passing"], "flights": "100"}
    for r in csv.DictReader(open(ROOT / "docs/test_reports/test023_data/gnss_free_summary.csv")):
        if r["scenario"] == key and r["gnss_free_s"] == extra:
            return {"cep50": r["cep50"], "cep95": r["cep95"], "max": r["max"], "over3sigma_pct": r["over3sigma_pct"],
                    "ok": r["never_over_100"], "flights": r["flights"]}
    raise KeyError(spec)


def aggregate(log_dir, mc_dir):
    log_dir, mc_dir = Path(log_dir), Path(mc_dir)
    DATA.mkdir(parents=True, exist_ok=True)
    with open(DATA / "log_sha256.txt", "w") as f:
        for name in ("raw.csv", "allan.csv"):
            h = hashlib.sha256()
            with open(log_dir / name, "rb") as g:
                for chunk in iter(lambda: g.read(1 << 20), b""):
                    h.update(chunk)
            f.write(f"{h.hexdigest()}  log/h30wp_static/{name}\n")
    # Bias over the recording: 1-minute means, clock time from the first frame's host time.
    with open(log_dir / "raw.csv") as f:
        f.readline()
        t0_host = float(f.readline().split(",")[0])
    t, acc, gyr = af.load_csv(str(log_dir / "allan.csv"))
    n = int(round(60.0 / np.median(np.diff(t))))
    k = t.size // n
    am = acc[:k * n].reshape(k, n, 3).mean(1)
    gm = gyr[:k * n].reshape(k, n, 3).mean(1)
    with open(DATA / "bench_minute_means.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["minute", "clock", "ax_mps2", "ay_mps2", "az_mps2", "gx_mdegps", "gy_mdegps", "gz_mdegps"])
        for i in range(k):
            clock = time.strftime("%H:%M", time.localtime(t0_host + 60.0 * i))
            w.writerow([i, clock, *(f"{v:.6f}" for v in am[i]), *(f"{v:.3f}" for v in np.rad2deg(gm[i]) * 1e3)])
    # Allan curves and both fits (least squares; the envelope over 30 s to 10 min).
    res = af.fit_log(t, acc, gyr, SKIP_S)
    with open(DATA / "adev_h30wp.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sensor", "tau_s", "adev_x", "adev_y", "adev_z"])
        for key in ("accel", "gyro"):
            r = res[key]
            for i, tau in enumerate(r["tau"]):
                w.writerow([key, f"{tau:.6g}", *(f"{r['adev'][a][i]:.6g}" for a in range(3))])
    with open(DATA / "fits.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sensor", "fit", "N_mean", "K", "tau_s", "worst_ratio_in_band", "at_tau_s"])
        for key in ("accel", "gyro"):
            r = res[key]
            tau, nclust = r["tau"], res["n"] / (r["tau"] * res["fs"])
            avar = np.mean([a ** 2 for a in r["adev"]], axis=0)
            sel = tau >= 10.0 / res["fs"]
            N_ls, K_ls, tc_ls = af.fit_white_gm(tau[sel], avar[sel], nclust[sel])
            e = r["envelope"] or {}
            w.writerow([key, "least squares", f"{N_ls:.5g}", f"{K_ls:.5g}", f"{tc_ls:.5g}",
                        f"{e.get('was', float('nan')):.3f}", f"{e.get('at', float('nan')):.0f}"])
            w.writerow([key, "envelope (model file)", f"{np.mean(r['N']):.5g}", f"{r['K']:.5g}", f"{r['tc']:.5g}", "", ""])
    # Monte Carlo.
    cols = ["cep50", "cep95", "max", "over3sigma_pct", "ok", "flights"]
    with open(DATA / "mc_before_after.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "case", "imu", "gyro_bias_rw_degps_rtHz", "gnss_free_s", *cols])
        for label, b, bw, a, aw in MC_ROWS:
            rb = ref_row(b) if b.startswith("ref:") else parse_out(mc_dir / f"{b}.out", bw)
            ra = parse_out(mc_dir / f"{a}.out", aw)
            imu_b = "recorded" if label.startswith("flight001") else "placeholder-px4-default"
            imu_a = "recorded" if label.startswith("flight001") else "h30wp"
            w.writerow([label, "before", imu_b, "0.01", bw or "", *(rb[c] for c in cols)])
            w.writerow([label, "after", imu_a, "0.001", aw or "", *(ra[c] for c in cols)])
    with open(DATA / "process_noise_sweep.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["setting", "scenario", "gnss_free_s", *cols])
        for label, suf in SWEEP:
            # run names: PREFIX + suffix (z_h_qbg1e-3, f1g_qbg1e-3), BASE for the defaults
            for scen, pre, base, win in (("Zermatt 177 s, from take-off", "z_h", "z_h", None),
                                         ("Zermatt 15 min, 10 min after GNSS loss", "zl_h", "zl_h", 600),
                                         ("flight001, from take-off", "f1", "f1_def", None),
                                         ("flight001, 5 min after GNSS loss", "f1g", "f1g_def", 300)):
                p = mc_dir / f"{pre + suf if suf else base}.out"
                if p.exists():   # not every setting was run on every scenario
                    w.writerow([label, scen, win or "", *(parse_out(p, win)[c] for c in cols)])
    with open(DATA / "start_error_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["flight", "start_error_m", "under_100m_at_120s", "flights"])
        for flight, pre in (("Zermatt 177 s", "fin_zs"), ("Zermatt 15 min", "fin_zls"), ("flight001", "fin_f1s")):
            for s in (0, 60, 150, 500):
                m = re.search(r"under 100 m at 120 s after take-off: (\d+)/(\d+)", (mc_dir / f"{pre}_s{s}.out").read_text())
                w.writerow([flight, s, m[1], m[2]])
    print(f"wrote {DATA}")


# ---------------------------------------------------------------------- plots ----
def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8.5)
    ax.grid(True, which="major", color=GRID, lw=0.6)


def plot_bench():
    rows = list(csv.DictReader(open(DATA / "adev_h30wp.csv")))
    fits = {(r["sensor"], r["fit"].split(" ")[0]): r for r in csv.DictReader(open(DATA / "fits.csv"))}
    mins = list(csv.DictReader(open(DATA / "bench_minute_means.csv")))
    fig = plt.figure(figsize=(12, 8.2), facecolor=SURF)
    gs = fig.add_gridspec(2, 2, height_ratios=(1.15, 1), hspace=0.42, wspace=0.22)
    for col, (key, unit) in enumerate((("accel", "m/s²"), ("gyro", "rad/s"))):
        ax = fig.add_subplot(gs[0, col]); style(ax)
        rr = [r for r in rows if r["sensor"] == key]
        tau = np.array([float(r["tau_s"]) for r in rr])
        ax.axvspan(af.MISSION_BAND[0], af.MISSION_BAND[1], color=GRID, alpha=0.5, lw=0)
        ax.text(np.sqrt(af.MISSION_BAND[0] * af.MISSION_BAND[1]), 0.97, "30 s to 10 min\n(scored windows)",
                transform=ax.get_xaxis_transform(), ha="center", va="top", fontsize=8, color=INK2)
        for i, (a, c) in enumerate(zip("xyz", (C1, C2, C3))):
            ax.loglog(tau, [float(r[f"adev_{a}"]) for r in rr], "o", ms=3.6, color=c, label=f"{a} axis, measured",
                      mec=SURF, mew=0.4)
        tt = np.logspace(np.log10(tau[0]), np.log10(tau[-1]), 200)
        ls, en = fits[(key, "least")], fits[(key, "envelope")]
        ax.loglog(tt, af.model_adev(float(ls["N_mean"]), float(ls["K"]), float(ls["tau_s"]), tt), "--", color=INK2,
                  lw=1.4, label="least-squares fit (too low)")
        ax.loglog(tt, af.model_adev(float(en["N_mean"]), float(en["K"]), float(en["tau_s"]), tt), "-", color=INK,
                  lw=2, label="model file (envelope)")
        ax.set_xlabel("averaging time τ [s]", color=INK2, fontsize=9)
        ax.set_ylabel(f"Allan deviation [{unit}]", color=INK2, fontsize=9)
        ax.set_title(f"{key}: least squares {1 / float(ls['worst_ratio_in_band']):.1f}× too low at "
                     f"{float(ls['at_tau_s']):.0f} s", loc="left", fontsize=10, color=INK)
        ax.legend(fontsize=7.8, frameon=False, loc="lower left")
    ax = fig.add_subplot(gs[1, :]); style(ax)
    m = np.array([int(r["minute"]) for r in mins])
    g = np.array([[float(r[f"g{a}_mdegps"]) for a in "xyz"] for r in mins])
    ax.axvspan(0, SKIP_S / 60, color=GRID, alpha=0.5, lw=0)
    ax.text(SKIP_S / 120, 0.96, "warm-up\n(not fitted)", transform=ax.get_xaxis_transform(), ha="center", va="top",
            fontsize=8, color=INK2)
    for i, (a, c) in enumerate(zip("xyz", (C1, C2, C3))):
        ax.plot(m, g[:, i] - g[0, i], "-", color=c, lw=2, label=f"gyro {a}")
        ax.text(m[-1] + 1.5, g[-1, i] - g[0, i], f"{a}", color=INK2, fontsize=9, va="center")
    tmin = int(np.argmin(g[:60, 2]))
    ax.annotate(f"turns at {mins[tmin]['clock']}: the AC went off\nand the room warmed", xy=(m[tmin], g[tmin, 2] - g[0, 2]),
                xytext=(m[tmin] + 10, -40), fontsize=8.5, color=INK2, va="center",
                arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.set_ylim(-46, None)
    ticks = list(range(0, len(mins), 15))
    ax.set_xticks(ticks, [f"{t} min\n{mins[t]['clock']}" for t in ticks])
    ax.set_xlim(0, m[-1] + 5)
    ax.set_ylabel("gyro bias change since start\n[millidegrees/s, 1-minute means]", color=INK2, fontsize=9)
    ax.set_title("The bias follows the room: 0.06 °/s over the recording, 50× the sensor's own instability",
                 loc="left", fontsize=10, color=INK)
    ax.legend(fontsize=8, frameon=False, loc="upper left", ncol=3, bbox_to_anchor=(0.12, 1.0))
    fig.suptitle("TEST-024  H30WP static bench log, 2.25 h at 200 Hz, 0 frames lost", x=0.01, ha="left",
                 color=INK2, fontsize=10.5)
    fig.savefig(OUT / "test024_bench.png", dpi=120, facecolor=SURF, bbox_inches="tight")


def plot_mc():
    rows = list(csv.DictReader(open(DATA / "mc_before_after.csv")))
    labels = list(dict.fromkeys(r["scenario"] for r in rows))
    get = {(r["scenario"], r["case"]): r for r in rows}
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.9), facecolor=SURF, gridspec_kw={"wspace": 0.06})
    y = np.arange(len(labels))[::-1]
    for ax, key, xl in ((axs[0], "cep95", "CEP95 [m], log scale"), (axs[1], "over3sigma_pct",
                                                                    "samples with error over 3 × stated sigma [%]")):
        style(ax); ax.grid(True, axis="x", color=GRID, lw=0.6); ax.grid(False, axis="y")
        for yi, lab in zip(y, labels):
            b, a = float(get[(lab, "before")][key]), float(get[(lab, "after")][key])
            ax.plot([b, a], [yi, yi], "-", color=GRID, lw=3, solid_capstyle="round", zorder=1)
            ax.plot(b, yi, "o", ms=8, color=BEFORE, mec=SURF, mew=1.5, zorder=2)
            ax.plot(a, yi, "o", ms=8, color=C1 if a <= b else C2, mec=SURF, mew=1.5,
                    zorder=3)
            fmt = "{:.0f}" if key == "cep95" else "{:.2f}"
            ax.text(max(a, b) * (1.12 if key == "cep95" else 1) + (0 if key == "cep95" else 0.35), yi,
                    f"{fmt.format(b)} → {fmt.format(a)}", va="center", fontsize=8, color=INK2)
        ax.set_xlabel(xl, color=INK2, fontsize=9)
        ax.set_yticks(y, labels if ax is axs[0] else [""] * len(labels), fontsize=8.5, color=INK)
        ax.set_ylim(-0.7, len(labels) - 0.3)
    axs[0].set_xscale("log"); axs[0].set_xlim(12, 330)
    axs[0].set_xticks([20, 50, 100, 200], ["20", "50", "100", "200"])
    axs[0].xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
    axs[1].axvline(1.0, color=INK2, lw=1, ls=":")
    axs[1].text(1.1, len(labels) - 0.45, "REQ-INT-001: at most 1%", fontsize=8, color=INK2, va="center")
    axs[1].set_xlim(-0.5, 17)
    from matplotlib.lines import Line2D
    h = [Line2D([], [], marker="o", ls="", ms=8, color=BEFORE, label="before: placeholder IMU, gyro bias rw 0.01"),
         Line2D([], [], marker="o", ls="", ms=8, color=C1, label="after: H30WP model, gyro bias rw 1e-3"),
         Line2D([], [], marker="o", ls="", ms=8, color=C2, label="after, and worse")]
    fig.legend(handles=h, loc="upper left", bbox_to_anchor=(0.01, 0.99), ncol=3, frameon=False, fontsize=8.5)
    fig.suptitle("TEST-024  Monte Carlo, 100 flights per row (flight001 keeps its recorded IMU)", x=0.01, y=1.03,
                 ha="left", color=INK2, fontsize=10.5)
    fig.savefig(OUT / "test024_mc.png", dpi=120, facecolor=SURF, bbox_inches="tight")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aggregate", nargs=2, metavar=("LOG_DIR", "MC_DIR"))
    a = ap.parse_args()
    if a.aggregate:
        aggregate(*a.aggregate)
    plot_bench()
    plot_mc()
    print(f"wrote {OUT / 'test024_bench.png'} and {OUT / 'test024_mc.png'}")
