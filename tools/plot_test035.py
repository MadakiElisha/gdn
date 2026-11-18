#!/usr/bin/env python3
"""TEST-035 figure: the simulated IMUs and TFmini-S (R2.4a) against the real ones.

    # from a tools/sim_run.sh log directory:
    python3 tools/plot_test035.py --export docs/test_reports/test035_data log/r24/t035/zermatt_x500
    # from the exported CSVs (what's committed):
    python3 tools/plot_test035.py docs/test_reports/test035_data --out docs/test_reports/test035_sensors.png

Export writes, from one run: each IMU's stamp spacing, its white noise at rest per axis
against its R1.6 model, every sensor's arrival minus stamp, and the TFmini-S's readings
with the true range along its beam.
"""
import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

IMUS = {"h30wp": ("#2a78d6", "H30WP (/imu/data)"),
        "mid360s_imu": ("#eb6834", "Mid-360S IMU (/livox/imu)"),
        "oakd_imu": ("#2b2a28", "OAK-D Lite IMU (/oak/imu/data)")}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#d9d8d4"


def export(out, run):
    import check_sim_sensors as cs
    from check_vehicles import cast
    from gdn_paths import load_map
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    d = Path(run)
    info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    cfg = cs.load_config()
    types, msgs, truth = cs.read_bag(d / "sitl_bag", [r["topic"] for r in cs.REAL.values()])
    t_rest = cs.rest_window(truth)
    noise_rows, lat_rows = [], []
    with open(out / "imu_spacing.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sensor", "dt_ms"])
        for name in IMUS:
            c, m = cfg[name], msgs[cs.REAL[name]["topic"]]
            arr = np.array([t for t, _ in m])
            st = np.array([cs.stamp(x) for _, x in m])
            for v in np.diff(st)[:5000] * 1000.0:
                w.writerow([name, f"{v:.4f}"])
            rest = (arr >= t_rest[0]) & (arr <= t_rest[1])
            scale = cs.G_STD if c["accel_in_g"] else 1.0
            model = cs.load_model(c["model"])
            for what, attr, dens in (("accel", "linear_acceleration", model["accel_noise_density"]),
                                     ("gyro", "angular_velocity", model["gyro_noise_density"])):
                x = np.array([[getattr(getattr(mm, attr), k) for k in "xyz"] for _, mm in m])[rest]
                if what == "accel":
                    x = x * scale
                sig = np.std(x - x.mean(axis=0), axis=0)
                exp = cs.expected_sigma(dens, st[rest])
                for axis, s_, e_ in zip("xyz", sig, exp):
                    noise_rows.append([name, what, axis, f"{s_:.4e}", f"{e_:.4e}"])
            for v in ((arr - st) * 1000.0)[::20]:
                lat_rows.append([name, f"{v:.3f}"])
    with open(out / "imu_noise_at_rest.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sensor", "kind", "axis", "sigma", "model_sigma"])
        w.writerows(noise_rows)
    # TFmini-S: readings against the true range along the beam.
    c = cfg["tfmini"]
    m = msgs[cs.REAL["tfmini"]["topic"]]
    arr = np.array([t for t, _ in m])
    st = np.array([cs.stamp(x) for _, x in m])
    r = np.array([x.range for _, x in m])
    for v in ((arr - st) * 1000.0)[::20]:
        lat_rows.append(["tfmini", f"{v:.3f}"])
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    alt = alt.astype(np.float64)
    z0 = float(winfo["z0_amsl"])
    mount = cs.payload_mount("gdn/tfmini")
    t0 = truth[0, 0]
    with open(out / "tfmini.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "range_m", "true_range_m"])
        for i in range(0, len(r), 2):                           # 50 Hz
            tr = cs.truth_at(truth, arr[i] - c["latency_ms"] * 1e-3 + 0.001)
            qw, qx, qy, qz = tr[4:8]
            R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                          [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
                          [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])
            p = tr[1:4] + R @ mount
            hit = cast(alt, hdr, z0, p[0], p[1], p[2], tr[4:8], min_height=0.0)
            pred = hit[0] if hit is not None else float("nan")
            if np.isfinite(pred) and pred > 20.0:
                continue                                        # high in cruise: nothing to see
            w.writerow([f"{arr[i] - t0:.3f}", f"{r[i]:.3f}", f"{pred:.3f}"])
    with open(out / "latency.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sensor", "arrival_minus_stamp_ms"])
        w.writerows(lat_rows)
    (out / "run.txt").write_text(f"{info['vehicle']} over {info['world']}, at rest {t_rest[1] - t_rest[0]:.0f} s\n")
    print(f"exported {d.name}")


def plot(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = Path(d)
    sp = list(csv.DictReader(open(d / "imu_spacing.csv")))
    nz = list(csv.DictReader(open(d / "imu_noise_at_rest.csv")))
    lat = list(csv.DictReader(open(d / "latency.csv")))
    tf = np.genfromtxt(d / "tfmini.csv", delimiter=",", names=True)
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold"})
    fig, axs = plt.subplots(2, 2, figsize=(13, 8.6))
    # Stamp spacing.
    ax = axs[0, 0]
    bins = np.arange(0, 9.01, 0.1)
    for name, (c, label) in IMUS.items():
        v = np.array([float(r["dt_ms"]) for r in sp if r["sensor"] == name])
        spread = np.std(v) / np.mean(v)
        ax.hist(v, bins=bins, histtype="step", color=c, lw=1.6, density=True,
                label=f"{label}: {1000 / np.mean(v):.0f} Hz, spread {100 * spread:.0f}%")
    ax.set_xlabel("time between stamps (ms)")
    ax.set_ylabel("density")
    ax.set_title("IMU stamp spacing (bench: Mid-360S spread 15%; OAK-D 239 Hz, 29%)", loc="left")
    ax.legend(fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Noise at rest against the model.
    ax = axs[0, 1]
    names = list(IMUS)
    for j, name in enumerate(names):
        c, label = IMUS[name]
        for kk, kind in enumerate(("accel", "gyro")):
            rows = [r for r in nz if r["sensor"] == name and r["kind"] == kind]
            ratio = [float(r["sigma"]) / float(r["model_sigma"]) for r in rows]
            x = j * 2.5 + kk + np.array([-0.15, 0.0, 0.15])
            ax.plot(x, ratio, "o" if kind == "accel" else "s", color=c, ms=6, ls="none")
    ax.axhspan(0.75, 1.25, color=GRID, alpha=0.5, lw=0)
    ax.axhline(1.0, color=INK2, lw=0.8)
    ax.set_xticks([j * 2.5 + kk for j in range(3) for kk in range(2)])
    ax.set_xticklabels([f"{n.split('_')[0]}\n{k}" for n in names for k in ("accel", "gyro")], fontsize=8)
    ax.set_ylim(0.5, 1.5)
    ax.set_ylabel("white noise at rest / R1.6 model (x, y, z)")
    ax.set_title("IMU noise at rest against its R1.6 model (band: the check's 25%)", loc="left")
    ax.grid(axis="y", color=GRID, lw=0.5)
    # Latency.
    ax = axs[1, 0]
    groups = list(IMUS) + ["tfmini"]
    data = [np.array([float(r["arrival_minus_stamp_ms"]) for r in lat if r["sensor"] == g]) for g in groups]
    data = [x[x < 30] for x in data]
    bp = ax.boxplot(data, vert=False, showfliers=False, widths=0.5, patch_artist=True)
    for patch, g in zip(bp["boxes"], groups):
        patch.set_facecolor(IMUS.get(g, ("#8a8a85", ""))[0])
        patch.set_alpha(0.35)
    ax.set_yticks(range(1, len(groups) + 1))
    ax.set_yticklabels(["H30WP\n(stamped on arrival)", "Mid-360S IMU\n(sensor stamps, 1 ms)",
                        "OAK-D Lite IMU\n(device stamps, 4 ms)", "TFmini-S\n(stamped on arrival)"], fontsize=8)
    ax.set_xlabel("arrival minus stamp (ms)")
    ax.set_title("Timing: what a subscriber sees", loc="left")
    ax.grid(axis="x", color=GRID, lw=0.5)
    # TFmini-S.
    ax = axs[1, 1]
    t, r, p = tf["t_s"], tf["range_m"], tf["true_range_m"]
    fin = np.isfinite(r)
    ax.plot(t[fin], r[fin], ".", ms=2, color="#2a78d6", label="TFmini-S reading")
    ax.plot(t, np.where(p <= 12.0, p, np.nan), "-", lw=1.0, color=INK, label="true range along the beam")
    up = np.isposinf(r) & (t > 0)
    ax.plot(t[up], np.full(up.sum(), 12.3), "|", ms=4, color="#eb6834", label="+inf (beyond 12 m)")
    ax.axhline(12.0, color=INK2, lw=0.6, ls=":")
    ax.set_xlabel("seconds into the recording")
    ax.set_ylabel("range (m)")
    near = fin & np.isfinite(p) & (p > 0.3) & (p < 11.5) & (t > 30)
    e = r[near] - p[near]
    ax.set_title(f"TFmini-S at take-off and landing (error RMS {np.sqrt(np.mean(e ** 2)) * 100:.1f} cm)", loc="left")
    ax.legend(fontsize=7.5, loc="upper right")
    ax.grid(color=GRID, lw=0.5)
    run = (d / "run.txt").read_text().strip() if (d / "run.txt").exists() else ""
    fig.suptitle(f"TEST-035: simulated IMUs and TFmini-S as their real drivers publish them (R2.4a); {run}",
                 x=0.04, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=130, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test035_data")
    ap.add_argument("--export", metavar="OUT_DIR", help="export this run directory into OUT_DIR")
    ap.add_argument("--out", default="docs/test_reports/test035_sensors.png")
    a = ap.parse_args()
    if a.export:
        export(a.export, a.data)
    else:
        plot(a.data, a.out)


if __name__ == "__main__":
    main()
