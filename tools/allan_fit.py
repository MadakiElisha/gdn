#!/usr/bin/env python3
"""Allan-variance fit of a static IMU log into a GDN IMU model file (R1.6).

The model is the one gdn::ImuErrorSim simulates (src/gdn_fusion/include/gdn/imu_model.hpp):
white noise N per axis plus a first-order Gauss-Markov bias drift (driving density K,
correlation time tau). For each sensor the overlapping Allan deviation is computed per
axis, N is fitted per axis and (K, tau) on the mean of the three axes, by weighted least
squares on the exact Allan variance of that model. The bias instability B (minimum of
the curve / 0.664) is also printed, to compare with a datasheet.

One drift term can't follow every sensor: a flat stretch followed by a rise (usually
temperature) gets fitted to the flat part, and the model then falls below the measured
curve over minutes. Where that is more than 25% below anywhere from 30 s to 10 min (the
GNSS-free windows, MISSION_BAND), the drift is refitted as the smallest one that stays
at or above the curve there, and the output says so (TEST-024).

    # a static log (at least 2 h, see config/imu/README.md):
    python3 tools/allan_fit.py --csv log/h30wp_static/allan.csv --skip 900 \\
        --name h30wp --out config/imu/h30wp.yaml --plot h30wp_allan.png    # tools/h30wp_log.py
    python3 tools/allan_fit.py --bag log/px4_static --topic /fmu/out/sensor_combined \\
        --name px4 --out config/imu/px4.yaml
    python3 tools/allan_fit.py --bag log/mid360s_static --topic /livox/imu --accel-unit g \\
        --skip 2400 --name mid360s --out config/imu/mid360s.yaml
    python3 tools/allan_fit.py --csv log/oakd_lite_static/imu.csv --skip 1200 \\
        --name oakd_lite --out config/imu/oakd_lite.yaml                 # tools/oak_imu_log.py
    python3 tools/allan_fit.py --csv log.csv ...       # t[s or us],ax,ay,az,gx,gy,gz
    python3 tools/allan_fit.py --selfcheck             # recovers known parameters (CI)

Sample times needn't be even (TEST-025): the Mid-360S's are jittered and the OAK-D Lite
drops about 40% of its gyro ticks. Each sample is held until the next, as GDN
integrates it, and that is resampled evenly at the mean rate (even_samples). A log that
breaks off (a gap over BREAK_S, or the clock going back when the sensor restarts) is cut
there, because what follows is a fresh power-up (first_segment).

One static log can't show the turn-on bias (it is one draw); the output keeps the
--turn-on-* values (default: the placeholder's) and says so.
"""
import argparse
import csv
import datetime
import os
import sys

import numpy as np

AXES = ("x", "y", "z")
# Averaging times the drift must be right over: the 5 and 10 minute GNSS-free windows of
# REQ-NAV-003, from 30 s up. Shorter logs cap the top at a tenth of their length.
MISSION_BAND = (30.0, 600.0)
# The least-squares fit is replaced by an upper envelope over that band when it is below
# the measured curve by more than this anywhere in it (25% is the scatter of a 2 h log's
# Allan deviation near 10 min, so a model that is right but unlucky doesn't trigger).
ENVELOPE_TRIGGER = 0.75


# ---------------------------------------------------------------- Allan variance ----
def adev_curve(x, fs, n_points=60, max_frac=0.1):
    """Overlapping Allan deviation of samples x at rate fs. Cluster sizes m are log-spaced
    up to max_frac of the record. Returns tau, adev, number of independent clusters."""
    x = np.asarray(x, dtype=np.float64)
    n = x.size
    c = np.concatenate([[0.0], np.cumsum(x - x.mean())])
    ms = np.unique(np.logspace(0, np.log10(max(1, int(n * max_frac))), n_points).astype(int))
    out = []
    for m in ms:
        d = c[2 * m:] - 2.0 * c[m:-m] + c[:-2 * m]
        out.append(np.sqrt(np.mean(d * d) / (2.0 * m * m)))
    return ms / fs, np.array(out), n / ms


def gm_avar(K, tau, t):
    """Allan variance of a first-order Gauss-Markov process (driving density K)."""
    t = np.asarray(t, dtype=np.float64)
    x = t / tau
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        big = K * K * tau * tau / t * (1.0 - (3.0 - 4.0 * np.exp(-x) + np.exp(-2.0 * x)) / (2.0 * x))
    return np.where(x < 1e-3, K * K * t / 3.0, big)


def model_adev(N, K, tau, t):
    return np.sqrt(N * N / np.asarray(t) + gm_avar(K, tau, t))


def fit_white_gm(t, avar, nclust, n_axes_white=None):
    """Weighted LS of avar ~ N^2/t + K^2 g(t, tau) over a tau grid. Relative residuals,
    weighted by sqrt(clusters). Returns N, K, tau (K = 0 when drift isn't resolved)."""
    w = np.sqrt(np.maximum(nclust - 1.0, 1.0))
    best = None
    for tau in np.logspace(0, 5, 151):
        A = np.stack([1.0 / t, gm_avar(1.0, tau, t)], axis=1) / avar[:, None] * w[:, None]
        b = w
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        if sol[1] < 0:
            sol = np.array([np.linalg.lstsq(A[:, :1], b, rcond=None)[0][0], 0.0])
        if sol[0] < 0:
            sol = np.array([0.0, np.linalg.lstsq(A[:, 1:], b, rcond=None)[0][0]])
        r = np.linalg.norm(A @ sol - b)
        if best is None or r < best[0]:
            best = (r, sol, tau)
    _, (n2, k2), tau = best
    N, K = np.sqrt(max(n2, 0.0)), np.sqrt(max(k2, 0.0))
    # Drift resolved only if it is a visible part of the curve at the longest tau.
    tm = t[-1]
    if K == 0.0 or gm_avar(K, tau, tm) < 0.25 * N * N / tm:
        K = 0.0
    return N, K, tau


def envelope_drift(t, avar, N):
    """The drift (K, tau) whose model never falls below the measured Allan variance over
    t (white noise N held), and exceeds it least (sum of log ratios). Used when the least-
    squares fit is too optimistic there, e.g. a flat floor followed by a temperature-driven
    rise that one Gauss-Markov term can't follow: the model then errs on the safe side."""
    best = None
    need = np.maximum(avar - N * N / t, 0.0)
    for tau in np.logspace(0, 5, 151):
        g = gm_avar(1.0, tau, t)
        k2 = float(np.max(need / g))
        excess = float(np.sum(np.log((N * N / t + k2 * g) / avar)))
        if best is None or excess < best[0]:
            best = (excess, np.sqrt(k2), tau)
    return best[1], best[2]


def bias_instability(t, adev):
    i = int(np.argmin(adev))
    return adev[i] / 0.664, t[i]


# --------------------------------------------------------------------- loading ----
def load_csv(path):
    with open(path) as f:
        r = csv.DictReader(f)
        cols = {k: [] for k in ("t", "ax", "ay", "az", "gx", "gy", "gz")}
        bad = 0
        for row in r:
            try:
                vals = [float(row[k]) for k in cols]
            except (TypeError, ValueError):   # e.g. a half-written last line after a power cut
                bad += 1
                continue
            for k, v in zip(cols, vals):
                cols[k].append(v)
        if bad:
            print(f"note: skipped {bad} unreadable line(s) in {path}")
    t = np.array(cols["t"])
    if np.median(np.diff(t)) > 100.0:   # microseconds (GDN CSVs, PX4)
        t = t * 1e-6
    return t, np.stack([cols[k] for k in ("ax", "ay", "az")], 1), np.stack([cols[k] for k in ("gx", "gy", "gz")], 1)


def load_bag(path, topic):
    import yaml
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    sid = "mcap"
    meta = os.path.join(path, "metadata.yaml")
    if os.path.exists(meta):
        info = yaml.safe_load(open(meta)).get("rosbag2_bag_information", {})
        sid = info.get("storage_identifier", sid) or sid
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=sid), rosbag2_py.ConverterOptions("", ""))
    types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    if topic not in types:
        sys.exit(f"topic {topic} not in bag (has: {', '.join(sorted(types))})")
    typ = types[topic]
    cls = get_message(typ)
    t, a, g = [], [], []
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, cls)
        if typ == "sensor_msgs/msg/Imu":
            hs = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
            t.append(hs if hs > 0 else stamp * 1e-9)
            a.append((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
            g.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
        elif typ == "px4_msgs/msg/SensorCombined":
            t.append(m.timestamp * 1e-6)
            a.append(tuple(m.accelerometer_m_s2))
            g.append(tuple(m.gyro_rad))
        else:
            sys.exit(f"{topic} is {typ}; supported: sensor_msgs/msg/Imu, px4_msgs/msg/SensorCombined")
    return np.array(t), np.array(a), np.array(g)


# ------------------------------------------------------------------------- fit ----
BREAK_S = 1.0   # a step longer than this (or backwards) ends the log: what follows is a restart


def first_segment(t):
    """Index where the log first breaks off (a gap over BREAK_S, or time going backwards
    when a sensor restarts and its clock begins again), and a description; (len, None) if
    it never does. After a break the sensor has powered up again, with a new bias and a
    new warm-up, so it is not the same static log (TEST-025)."""
    dt = np.diff(t)
    bad = np.flatnonzero((dt < 0.0) | (dt > BREAK_S))
    if bad.size == 0:
        return t.size, None
    i = int(bad[0])
    what = f"the clock went back {-dt[i]:.1f} s" if dt[i] < 0 else f"a {dt[i]:.1f} s gap"
    return i + 1, {"at": float(t[i] - t[0]), "what": what, "dropped": int(t.size - i - 1)}


def even_samples(t, x):
    """Resample a stream with uneven sample times onto an even grid at its mean rate. Each
    sample is held until the next one, as GDN's integration uses it; the integral of that is
    interpolated onto the grid and differenced. Evenly timed logs come back unchanged (the
    last sample has no hold time and is dropped). The time-weighted mean is removed first,
    so timestamp jitter times a large mean (gravity, a gyro offset) doesn't add noise."""
    dt = np.diff(t)
    xs = x[:-1]
    xs = xs - (xs * dt[:, None]).sum(axis=0) / dt.sum()
    C = np.concatenate([np.zeros((1, x.shape[1])), np.cumsum(xs * dt[:, None], axis=0)])
    fs = (t.size - 1) / (t[-1] - t[0])
    tg = t[0] + np.arange(int(np.floor((t[-1] - t[0]) * fs + 1e-6)) + 1) / fs
    Cg = np.column_stack([np.interp(tg, t, C[:, i]) for i in range(x.shape[1])])
    return fs, np.diff(Cg, axis=0) * fs


def fit_log(t, acc, gyr, skip_s=0.0):
    end, cut = first_segment(t)
    t, acc, gyr = t[:end], acc[:end], gyr[:end]
    keep = t >= t[0] + skip_s
    t, acc, gyr = t[keep], acc[keep], gyr[keep]
    dt = np.diff(t)
    gaps = int(np.sum(dt > 3.0 * np.median(dt)))
    fs, acc = even_samples(t, acc)
    _, gyr = even_samples(t, gyr)
    res = {"fs": fs, "hours": (t[-1] - t[0]) / 3600.0, "gaps": gaps, "n": t.size, "cut": cut,
           "dt_spread": float(np.std(dt) / np.mean(dt))}
    for key, data in (("accel", acc), ("gyro", gyr)):
        curves = [adev_curve(data[:, i], fs) for i in range(3)]
        tau = curves[0][0]
        nclust = curves[0][2]
        # Fit from 10 samples up: below that, sensor anti-alias filters bend the curve.
        sel = tau >= 10.0 / fs
        Ns = []
        for c in curves:
            N_i, _, _ = fit_white_gm(tau[sel], c[1][sel] ** 2, nclust[sel])
            Ns.append(N_i)
        mean_avar = np.mean([c[1] ** 2 for c in curves], axis=0)
        N_m, K, tc = fit_white_gm(tau[sel], mean_avar[sel], nclust[sel])
        band = (MISSION_BAND[0], min(MISSION_BAND[1], tau[-1]))
        inb = (tau >= band[0]) & (tau <= band[1])
        low = np.sqrt(N_m * N_m / tau + gm_avar(K, tc, tau)) / np.sqrt(mean_avar) if K > 0 \
            else N_m / np.sqrt(tau * mean_avar)
        envelope = None
        if inb.sum() >= 3 and low[inb].min() < ENVELOPE_TRIGGER:
            i = np.flatnonzero(inb)[np.argmin(low[inb])]
            K, tc = envelope_drift(tau[inb], mean_avar[inb], N_m)
            envelope = {"band": band, "was": float(low[i]), "at": float(tau[i])}
        if K > 0:   # refit each axis's N with the common drift held fixed
            G = gm_avar(K, tc, tau[sel])
            Ns = [_white_given_drift(tau[sel], c[1][sel] ** 2, G, nclust[sel]) for c in curves]
        B, tB = bias_instability(tau, np.sqrt(mean_avar))
        res[key] = {"tau": tau, "adev": [c[1] for c in curves], "N": np.array(Ns), "K": K, "tc": tc,
                    "B": B, "tB": tB, "envelope": envelope}
    return res


def _white_given_drift(t, avar, G, nclust):
    """N for one axis with the drift part G of the Allan variance fixed: relative-weighted
    LS of avar = N^2 / t + G (linear in N^2)."""
    w = np.sqrt(np.maximum(nclust - 1.0, 1.0))
    A = (1.0 / t) / avar * w
    b = (avar - G) / avar * w
    return float(np.sqrt(max(np.dot(A, b) / np.dot(A, A), 0.0)))


def write_model(path, name, res, src, turn_on_a, turn_on_g, source=None):
    a, g = res["accel"], res["gyro"]
    with open(path, "w") as f:
        f.write(f"# IMU model fitted by tools/allan_fit.py on {datetime.date.today().isoformat()}\n")
        f.write(f"# from {os.path.basename(src.rstrip('/'))}: {_timing(res)}.\n")
        f.write("# White noise and bias drift from the Allan deviation; turn-on bias NOT from this log\n")
        f.write("# (one static log is one power-up), kept from --turn-on-accel / --turn-on-gyro.\n")
        f.write(f"name: {name}\n")
        f.write(f"source: {source or 'Allan fit of ' + os.path.basename(src.rstrip('/'))}, {res['hours']:.1f} h static\n")
        f.write("accel_noise_density: [" + ", ".join(f"{v:.5g}" for v in a["N"]) + "]   # m/s^2/sqrt(Hz)\n")
        f.write("gyro_noise_density: [" + ", ".join(f"{v:.5g}" for v in g["N"]) + "]    # rad/s/sqrt(Hz)\n")
        f.write(f"accel_bias_rw: {a['K']:.5g}     # m/s^3/sqrt(Hz){'  (drift not resolved by this log)' if a['K'] == 0 else ''}\n")
        f.write(f"accel_bias_tau: {a['tc']:.4g}\n")
        f.write(f"gyro_bias_rw: {g['K']:.5g}      # rad/s^2/sqrt(Hz){'  (drift not resolved by this log)' if g['K'] == 0 else ''}\n")
        f.write(f"gyro_bias_tau: {g['tc']:.4g}\n")
        f.write(f"accel_turn_on_sigma: {turn_on_a:.5g}\n")
        f.write(f"gyro_turn_on_sigma: {turn_on_g:.5g}\n")
        f.write(f"fit_hours: {res['hours']:.3f}\n")
        f.write(f"fit_rate_hz: {res['fs']:.2f}\n")
        f.write(f"fit_accel_bias_instability: {a['B']:.4g}   # m/s^2, at tau {a['tB']:.3g} s\n")
        f.write(f"fit_gyro_bias_instability: {g['B']:.4g}    # rad/s, at tau {g['tB']:.3g} s\n")
        for key, r in (("accel", a), ("gyro", g)):
            e = r["envelope"]
            f.write(f"fit_{key}_drift: " + (f"upper envelope over {e['band'][0]:.0f}-{e['band'][1]:.0f} s "
                    f"(least squares was {1 / e['was']:.1f}x low at {e['at']:.0f} s)" if e else "least squares") + "\n")


def write_adev_csv(path, res):
    with open(path, "w") as f:
        f.write("sensor,tau,adev_x,adev_y,adev_z\n")
        for key in ("accel", "gyro"):
            r = res[key]
            for i, t in enumerate(r["tau"]):
                f.write(f"{key},{t:.6g},{r['adev'][0][i]:.6g},{r['adev'][1][i]:.6g},{r['adev'][2][i]:.6g}\n")


def _timing(res):
    s = f"{res['hours']:.2f} h at {res['fs']:.1f} Hz (mean), {res['gaps']} gaps"
    if res.get("dt_spread", 0.0) > 0.05:
        s += f", uneven sample times (spread {100 * res['dt_spread']:.0f}%: resampled, see even_samples)"
    if res.get("cut"):
        s += f", cut where it broke off at {res['cut']['at']:.0f} s ({res['cut']['what']})"
    return s


def report(res):
    print(f"log: {_timing(res)}, {res['n']} samples")
    for key, unit, rw in (("accel", "m/s^2/sqrt(Hz)", "m/s^3/sqrt(Hz)"), ("gyro", "rad/s/sqrt(Hz)", "rad/s^2/sqrt(Hz)")):
        r = res[key]
        drift = (f"drift K {r['K']:.3g} {rw}, tau {r['tc']:.0f} s (settles at {r['K'] * np.sqrt(r['tc'] / 2):.3g})"
                 if r["K"] > 0 else "drift not resolved")
        print(f"{key:5s}: N = " + ", ".join(f"{v:.3g}" for v in r["N"]) + f" {unit}; {drift}; "
              f"bias instability {r['B']:.3g} at {r['tB']:.3g} s")
        e = r["envelope"]
        if e:
            print(f"       {key} drift: the least-squares fit was {1 / e['was']:.1f}x below the measured curve at "
                  f"{e['at']:.0f} s, so the drift is the upper envelope over {e['band'][0]:.0f}-{e['band'][1]:.0f} s")
    if res["hours"] < 1.99:
        print("WARNING: shorter than the 2 h R1.6 asks for; drift terms are poorly constrained")


def plot(path, res, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    C = ("#2a78d6", "#eb6834", "#1baf7a")
    INK2, GRID, SURF = "#52514e", "#d9d8d4", "#fcfcfb"
    fig, axs = plt.subplots(1, 2, figsize=(12, 4.6), facecolor=SURF)
    for ax, key, unit in ((axs[0], "accel", "m/s^2"), (axs[1], "gyro", "rad/s")):
        r = res[key]
        for i in range(3):
            ax.loglog(r["tau"], r["adev"][i], "o", ms=3.5, color=C[i], label=f"{AXES[i]} axis")
        tt = np.logspace(np.log10(r["tau"][0]), np.log10(r["tau"][-1]), 200)
        ax.loglog(tt, model_adev(np.mean(r["N"]), r["K"], r["tc"], tt), "-", color="#0b0b0b", lw=1.5,
                  label="fitted model")
        ax.set_xlabel("averaging time tau [s]"); ax.set_ylabel(f"Allan deviation [{unit}]")
        ax.axvspan(MISSION_BAND[0], min(MISSION_BAND[1], r["tau"][-1]), color=GRID, alpha=0.45, lw=0,
                   label="30 s to 10 min" + (" (envelope fit)" if r["envelope"] else ""))
        ax.grid(True, which="both", color=GRID, lw=0.5); ax.set_title(key, loc="left")
        ax.legend(fontsize=8.5)
    fig.suptitle(title, x=0.01, ha="left", color=INK2)
    fig.savefig(path, dpi=120, facecolor=SURF, bbox_inches="tight")


# ------------------------------------------------------------------- selfcheck ----
def selfcheck():
    """Synthetic 2 h logs with known parameters (the placeholder and the bias-drift
    sensitivity case of config/imu/): the fit must recover them."""
    from scipy.signal import lfilter
    rng = np.random.default_rng(7)
    fs, hours = 100.0, 2.0
    n = int(hours * 3600 * fs)
    t = np.arange(n) / fs
    cases = (("placeholder", 4.03e-4, 0.0, 300.0, 5.52e-5, 0.0, 1000.0),
             ("bias drift", 4.03e-4, 0.006, 300.0, 5.52e-5, 3.8785e-5, 1000.0))
    ok = True

    def sim(N, K, tc):
        x = N * np.sqrt(fs) * rng.standard_normal((n, 3))
        if K > 0:
            dt = 1.0 / fs
            phi = np.exp(-dt / tc)
            sd = np.sqrt(-K * K * tc / 2.0 * np.expm1(-2.0 * dt / tc))
            x += lfilter([sd], [1.0, -phi], rng.standard_normal((n, 3)), axis=0)
        return x

    for name, Na, Ka, ta, Ng, Kg, tg in cases:
        res = fit_log(t, sim(Na, Ka, ta), sim(Ng, Kg, tg))
        for key, N, K, tc in (("accel", Na, Ka, ta), ("gyro", Ng, Kg, tg)):
            r = res[key]
            eN = np.max(np.abs(r["N"] / N - 1.0))
            if K > 0:
                s_true, s_fit = K * np.sqrt(tc / 2), r["K"] * np.sqrt(r["tc"] / 2)
                good = eN < 0.05 and r["K"] > 0 and abs(s_fit / s_true - 1.0) < 0.35
                msg = f"drift settles at {s_fit:.3g} (true {s_true:.3g}), tau {r['tc']:.0f} s (true {tc:.0f})"
            else:
                good = eN < 0.05 and r["K"] == 0.0
                msg = "no drift found" if r["K"] == 0 else f"found drift K={r['K']:.3g} that isn't there"
            print(f"{'PASS' if good else 'FAIL'}  allan: {name:11s} {key:5s} N off by {100 * eN:.1f}%, {msg}")
            ok &= good
    # A sensor the single-drift model can't describe (like the H30WP, TEST-024): a short
    # Gauss-Markov drift (the flat floor near a minute) plus a slow random walk (the rise
    # after it). However it fits, the model must not claim the sensor is better than it
    # is over the 30 s to 10 min the GNSS-free windows are scored on.
    Kw, tw, Kr = 2.7e-4, 6.0, 3.3e-5
    acc = sim(5.3e-4, Kw, tw) + sim(0.0, Kr, 1e9)
    res = fit_log(t, acc, sim(5.52e-5, 0.0, 1.0))
    r = res["accel"]
    tb = np.logspace(np.log10(MISSION_BAND[0]), np.log10(MISSION_BAND[1]), 25)
    truth = np.sqrt(5.3e-4 ** 2 / tb + gm_avar(Kw, tw, tb) + gm_avar(Kr, 1e9, tb))
    ratio = model_adev(np.mean(r["N"]), r["K"], r["tc"], tb) / truth
    good = ratio.min() > 0.75 and ratio.max() < 2.5
    print(f"{'PASS' if good else 'FAIL'}  allan: two drifts  accel model / true Allan deviation over "
          f"{MISSION_BAND[0]:.0f}-{MISSION_BAND[1]:.0f} s: {ratio.min():.2f} to {ratio.max():.2f} "
          "(must stay within 0.75 to 2.5)")
    ok &= good
    ok &= _selfcheck_uneven(rng)
    ok &= _selfcheck_breaks(rng)
    print("ALLAN SELFCHECK PASSED" if ok else "ALLAN SELFCHECK FAILED")
    return ok


def _selfcheck_uneven(rng):
    """Uneven sample times, like the OAK-D Lite's stream (TEST-025): the gyro ticks every
    2.53 ms but 62% of the steps skip a tick. Each sample is still one tick's reading, so
    the stream GDN integrates (each sample held until the next) has more angle noise than
    an even one: N_stream^2 = N^2 E[dt^2] / (E[dt] dt0). The fit must give that, not
    pretend the samples were evenly spaced at some rate."""
    dt0, N, hours = 1.0 / 395.0, 1.5e-3, 2.0
    steps = np.where(rng.random(int(hours * 3600 / dt0 / 1.62)) < 0.62, 2, 1)
    t = np.concatenate([[0.0], np.cumsum(steps)]) * dt0
    x = N / np.sqrt(dt0) * rng.standard_normal((t.size, 3))
    dt = np.diff(t)
    want = N * np.sqrt(np.mean(dt * dt) / (np.mean(dt) * dt0))
    res = fit_log(t, x * 0.1, x)
    ok = True
    for key, scale in (("accel", 0.1), ("gyro", 1.0)):
        r = res[key]
        e = np.max(np.abs(r["N"] / (want * scale) - 1.0))
        lo, hi = _band_ratio(r, want * scale)
        good = e < 0.05 and lo > 0.75 and hi < 1.25
        print(f"{'PASS' if good else 'FAIL'}  allan: uneven times {key:5s} N off by {100 * e:.1f}% from the stream's "
              f"{want * scale:.3g} (sensor {N * scale:.3g}); model / true over 30-600 s {lo:.2f} to {hi:.2f}")
        ok &= good
    return ok


def _band_ratio(r, N_true):
    """Fitted model over the true (white noise only) Allan deviation, min and max over
    the mission band. A small drift found in pure noise (it happens: the drift test is
    statistical) is harmless if the model stays close there; a spurious one isn't."""
    tb = np.logspace(np.log10(MISSION_BAND[0]), np.log10(MISSION_BAND[1]), 25)
    ratio = model_adev(np.mean(r["N"]), r["K"], r["tc"], tb) / (N_true / np.sqrt(tb))
    return ratio.min(), ratio.max()


def _selfcheck_breaks(rng):
    """A log that breaks off and resumes (TEST-025): the Mid-360S lost power for 128 s and
    came back, the OAK-D Lite restarted and its clock began again at zero. What follows is
    a fresh power-up (here, a different bias), so the fit must stop at the break."""
    fs, hours, N = 200.0, 2.0, 1.0e-3
    n = int(hours * 3600 * fs)
    ok = True
    for name, t2_start in (("gap 128 s", n / fs + 128.0), ("clock reset", 3.0)):
        t = np.concatenate([np.arange(n) / fs, t2_start + np.arange(n // 10) / fs])
        x = N * np.sqrt(fs) * rng.standard_normal((t.size, 3))
        x[n:] += 0.02                                   # the bias after the restart
        res = fit_log(t, x, x)
        r = res["gyro"]
        e = np.max(np.abs(r["N"] / N - 1.0))
        lo, hi = _band_ratio(r, N)
        good = e < 0.05 and lo > 0.75 and hi < 1.25 and abs(res["hours"] - hours) < 0.01
        print(f"{'PASS' if good else 'FAIL'}  allan: {name:11s} fitted {res['hours']:.2f} h (the {hours:.0f} h before "
              f"the break), N off by {100 * e:.1f}%; model / true over 30-600 s {lo:.2f} to {hi:.2f}")
        ok &= good
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--csv", help="CSV with t (s or us), ax, ay, az, gx, gy, gz")
    src.add_argument("--bag", help="rosbag2 directory (needs --topic)")
    src.add_argument("--selfcheck", action="store_true", help="fit synthetic logs with known parameters")
    ap.add_argument("--topic", help="sensor_msgs/msg/Imu or px4_msgs/msg/SensorCombined topic in --bag")
    ap.add_argument("--accel-unit", choices=("mps2", "g"), default="mps2", help="Livox drivers publish g")
    ap.add_argument("--gyro-unit", choices=("rads", "dps"), default="rads")
    ap.add_argument("--skip", type=float, default=0.0, help="seconds to drop at the start (warm-up)")
    ap.add_argument("--name", default="fitted-imu", help="model name, printed with every result that uses it")
    ap.add_argument("--out", help="write the model file (config/imu/NAME.yaml)")
    ap.add_argument("--adev-csv", help="write the Allan deviation curves")
    ap.add_argument("--plot", help="write a PNG of the curves and the fit")
    ap.add_argument("--source", help="source: line of the model file (default: the log's name)")
    ap.add_argument("--turn-on-accel", type=float, default=0.05, help="m/s^2 (not observable in one log)")
    ap.add_argument("--turn-on-gyro", type=float, default=0.02 * np.pi / 180, help="rad/s (not observable in one log)")
    a = ap.parse_args()
    if a.selfcheck:
        sys.exit(0 if selfcheck() else 1)
    if a.bag and not a.topic:
        ap.error("--bag needs --topic")
    t, acc, gyr = load_csv(a.csv) if a.csv else load_bag(a.bag, a.topic)
    end, cut = first_segment(t)
    if cut:
        print(f"note: the log breaks off {cut['at']:.0f} s in ({cut['what']}); the {cut['dropped']} samples "
              "after it are a fresh power-up and are left out")
        t, acc, gyr = t[:end], acc[:end], gyr[:end]
    if a.accel_unit == "g":
        acc = acc * 9.80665
    if a.gyro_unit == "dps":
        gyr = np.deg2rad(gyr)
    g = np.linalg.norm(acc.mean(axis=0))
    if not 9.0 < g < 10.6:
        print(f"WARNING: mean specific force {g:.2f} m/s^2, not about 9.81: check --accel-unit")
    spread_a = np.std(np.linalg.norm(acc, axis=1))
    dw = np.linalg.norm(gyr - np.median(gyr, axis=0), axis=1)
    peak_w = np.percentile(dw, 99.9)   # sustained motion, not one knock
    if spread_a > 0.2 or peak_w > 0.1:
        print(f"WARNING: not a static log (|a| spread {spread_a:.2f} m/s^2, gyro 99.9th percentile "
              f"{peak_w:.2f} rad/s): the fit describes motion, not sensor noise")
    knocks = np.flatnonzero(dw > 0.02)
    if knocks.size:
        ev = np.split(knocks, np.flatnonzero(np.diff(t[knocks]) > 1.0) + 1)   # 1 s apart = separate
        print(f"note: {len(ev)} short knock(s) over 0.02 rad/s, at " +
              ", ".join(f"{t[e[0]] - t[0]:.0f} s ({e.size} samples)" for e in ev[:5]) +
              ("" if len(ev) <= 5 else ", ...") + "; check the fit doesn't change without them")
    res = fit_log(t, acc, gyr, a.skip)
    res["cut"] = cut
    report(res)
    src_name = a.csv or a.bag
    if a.out:
        write_model(a.out, a.name, res, src_name, a.turn_on_accel, a.turn_on_gyro, a.source)
        print(f"wrote {a.out}")
    if a.adev_csv:
        write_adev_csv(a.adev_csv, res)
    if a.plot:
        plot(a.plot, res, f"{a.name}: Allan deviation of {os.path.basename(src_name.rstrip('/'))}")
        print(f"wrote {a.plot}")


if __name__ == "__main__":
    main()
