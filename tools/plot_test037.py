#!/usr/bin/env python3
"""TEST-037 figure: the simulated Mid-360S lidar (R2.4c) against our real unit.

    # from a tools/sim_run.sh log directory, and the real recording (once):
    python3 tools/plot_test037.py --export docs/test_reports/test037_data log/r24c/t037/ch_emmental_x500 \
        --real log/mid360s_lidar_real
    # from the exported files (what's committed):
    python3 tools/plot_test037.py docs/test_reports/test037_data --out docs/test_reports/test037_lidar.png

Export writes, from one run: one frame's returns climbing out (azimuth, elevation, range
in the sensor frame), every return's height above the world's terrain in moving frames
at its own time and 50 ms either side (a sample), and every frame's arrival minus stamp
on Gazebo's clock; from the real recording, one frame's returns and every frame's
arrival minus stamp. The stream's numbers come from the check (sim_stats.json, and the
reference mid360s_real.json).
"""
import argparse
import csv
import json
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, GRID, BLUE, ORANGE = "#0b0b0b", "#52514e", "#d9d8d4", "#2a78d6", "#eb6834"


def write_frame(path, p):
    import check_sim_sensors as cs
    az, el, hit, r = cs.livox_directions(p)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["azimuth_deg", "elevation_deg", "range_m", "line"])
        for a, e, rr, ln in zip(az[hit], el[hit], r[hit], p["line"][hit]):
            w.writerow([f"{a:.3f}", f"{e:.3f}", f"{rr:.3f}", int(ln)])


def export(out, run, real):
    import check_sim_sensors as cs
    from gdn_paths import load_map
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    d = Path(run)
    info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    world = (alt.astype(np.float64), hdr, float(winfo["z0_amsl"]))
    types, msgs, truth = cs.read_bag(d / "sitl_bag", [cs.RAW_REF])
    clock = cs.RunClock(truth, cs.RunClock.offset_from_log(d / "sensors.log"),
                        [(t, cs.stamp(m)) for t, m in msgs[cs.RAW_REF]])
    t_rest = cs.rest_window(truth)
    frames, pts = cs.read_livox(d / "sitl_bag", points_every=5)
    arr = np.array([f[0] for f in frames])
    st = np.array([f[1] for f in frames])
    lat = (clock.sim_of_wall(arr) - clock.sample(st)) * 1e3
    stats = cs.livox_stats(frames, pts, lat)
    (out / "sim_stats.json").write_text(json.dumps(stats, indent=1) + "\n")
    np.savetxt(out / "sim_latency.csv", lat, fmt="%.3f", header="arrival_minus_stamp_ms (Gazebo clock)", comments="")
    sdf = re.sub(r"<!--.*?-->", "", (cs.ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    mp = cs.payload_sensor(sdf, cs.LIVOX["gz"])[1]
    mount = (mp[:3], mp[3:])
    rng = np.random.default_rng(1)
    best, best_n = None, 0
    with open(out / "terrain_residual.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["shift_ms", "height_above_terrain_m"])
        for i, p in pts.items():
            if arr[i] <= t_rest[1]:
                continue
            t_gz = clock.sample(st[i]) + p["offset_time"].astype(float) * 1e-9
            if cs.truth_at(clock.truth, t_gz[len(t_gz) // 2])[8] <= 1.0:
                continue
            for sh in (-0.05, 0.0, 0.05):
                dz, r = cs.livox_ground_residual(p, t_gz, clock, mount, world, sh)
                dz = dz[r > 1.0]
                for v in dz[rng.random(len(dz)) < 0.02]:            # a 2% sample
                    w.writerow([int(round(sh * 1e3)), f"{v:.4f}"])
            n = int((cs.livox_directions(p)[2]).sum())
            if n > best_n:
                best, best_n = p, n
    if best is not None:
        write_frame(out / "sim_frame.csv", best)
    (out / "run.txt").write_text(f"{info['vehicle']} over {info['world']}\n")
    if real:
        rf, rp = cs.read_livox(Path(real), points_every=100)
        write_frame(out / "real_frame.csv", rp[sorted(rp)[3]])
        np.savetxt(out / "real_latency.csv", [(f[0] - f[1]) * 1e3 for f in rf], fmt="%.3f",
                   header="arrival_minus_stamp_ms (host clock)", comments="")
    print(f"exported {d.name}")


def plot(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = Path(d)
    sim = json.loads((d / "sim_stats.json").read_text())
    real = json.loads((d / "mid360s_real.json").read_text())
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(13, 10))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 1.0])
    for k, (name, title) in enumerate((("real_frame.csv", "Our real Mid-360S: one 100 ms frame (a room)"),
                                       ("sim_frame.csv", "Simulated: one 100 ms frame (climbing out)"))):
        ax = fig.add_subplot(gs[0, k])
        if (d / name).exists():
            f = np.genfromtxt(d / name, delimiter=",", names=True)
            sc = ax.scatter(f["azimuth_deg"], f["elevation_deg"], c=np.minimum(f["range_m"], 40), s=0.4, cmap="viridis",
                            vmin=0, vmax=40 if k else 10, rasterized=True)
            fig.colorbar(sc, ax=ax, pad=0.01).set_label("range (m)")
            ax.set_title(f"{title}: {len(f)} returns", loc="left")
        ax.set_xlim(-180, 180)
        ax.set_ylim(-10, 55)
        ax.set_xlabel("azimuth in the sensor frame (deg)")
        ax.set_ylabel("elevation (deg)")
        ax.grid(color=GRID, lw=0.5)
    # Terrain residual.
    ax = fig.add_subplot(gs[1, 0])
    tr = np.genfromtxt(d / "terrain_residual.csv", delimiter=",", names=True)
    bins = np.linspace(-1.0, 1.0, 81)
    for sh, c, lab in ((-50, ORANGE, "pose 50 ms before"), (50, INK2, "pose 50 ms after"), (0, BLUE, "pose at the point's own time")):
        v = tr["height_above_terrain_m"][tr["shift_ms"] == sh]
        ax.hist(v, bins=bins, histtype="step", lw=1.8 if sh == 0 else 1.2, color=c, density=True,
                label=f"{lab}: median |h| {np.median(np.abs(v)) * 100:.1f} cm")
    ax.set_xlabel("return's height above the world's terrain (m)")
    ax.set_ylabel("density")
    ax.set_title("Every return on the terrain, while moving (sample)", loc="left")
    ax.legend(fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # Timing.
    ax = fig.add_subplot(gs[1, 1])
    ls = np.loadtxt(d / "sim_latency.csv", skiprows=1)
    lr = np.loadtxt(d / "real_latency.csv", skiprows=1) if (d / "real_latency.csv").exists() else np.zeros(0)
    b = np.arange(95, 125.5, 0.5)
    ax.hist(lr, bins=b, density=True, color=INK2, alpha=0.45, label=f"real: median {np.median(lr):.1f} ms")
    ax.hist(ls, bins=b, density=True, histtype="step", lw=1.8, color=BLUE, label=f"simulated: median {np.median(ls):.1f} ms")
    ax.set_xlabel("arrival minus stamp (ms): the 100 ms frame, then the driver")
    ax.set_ylabel("density")
    ax.set_title("When a frame arrives", loc="left")
    ax.legend(fontsize=7.5)
    ax.grid(color=GRID, lw=0.5)
    # The stream's numbers.
    ax = fig.add_subplot(gs[2, :])
    ax.axis("off")
    rows = [("frame_id, lidar_id", f"{real['frame_ids'][0]}, {real['lidar_ids'][0]}", f"{sim['frame_ids'][0]}, {sim['lidar_ids'][0]}"),
            ("frame rate", f"{real['rate_hz']:.3f} Hz", f"{sim['rate_hz']:.3f} Hz"),
            ("points a frame (min, median, max)", "{} / {:.0f} / {}".format(*real["points_per_frame"]),
             "{} / {:.0f} / {}".format(*sim["points_per_frame"])),
            ("stamp = timebase (first point)", f"{100 * real['stamp_is_timebase']:.0f}%", f"{100 * sim['stamp_is_timebase']:.0f}%"),
            ("point spacing in a packet", f"{real['point_spacing_us']:.3f} us", f"{sim['point_spacing_us']:.3f} us"),
            ("offset_time to", f"{real['offset_time_max_ms']:.1f} ms", f"{sim['offset_time_max_ms']:.1f} ms"),
            ("lines 0..3", " / ".join(f"{100 * x:.0f}%" for x in real["line_shares"]),
             " / ".join(f"{100 * x:.0f}%" for x in sim["line_shares"])),
            ("pattern: azimuth step a group of 4", f"{real['azimuth_step_deg']:.2f} deg", f"{sim['azimuth_step_deg']:.2f} deg"),
            ("pattern: line spread in a group (IQR)", f"{real['line_spread_iqr_deg']:.2f} deg", f"{sim['line_spread_iqr_deg']:.2f} deg"),
            ("no return: (0,0,0), reflectivity 0", f"{real['no_return_reflectivity_0']} ({100 * real['no_return_share']:.0f}% in a room)",
             f"{sim['no_return_reflectivity_0']} ({100 * sim['no_return_share']:.0f}% in flight, beyond 40 m)"),
            ("nearest return", f"{real['nearest_return_m']:.3f} m", f"{sim['nearest_return_m']:.3f} m")]
    t = ax.table(cellText=[r[1:] for r in rows], rowLabels=[r[0] for r in rows], colLabels=["our real unit", "simulated"],
                 cellLoc="center", loc="center", colWidths=[0.3, 0.3])
    t.auto_set_font_size(False)
    t.set_fontsize(8.5)
    t.scale(1, 1.25)
    ax.set_title("The /livox/lidar stream, as livox_ros_driver2 1.2.8 publishes it", loc="left", pad=22)
    run = (d / "run.txt").read_text().strip() if (d / "run.txt").exists() else ""
    fig.suptitle(f"TEST-037: simulated Mid-360S lidar against our real unit (R2.4c); {run}",
                 x=0.04, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=120, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test037_data")
    ap.add_argument("--export", metavar="OUT_DIR", help="export this run directory into OUT_DIR")
    ap.add_argument("--real", help="with --export: the real recording, for its frame and arrival times")
    ap.add_argument("--out", default="docs/test_reports/test037_lidar.png")
    a = ap.parse_args()
    if a.export:
        export(a.export, a.data, a.real)
    else:
        plot(a.data, a.out)


if __name__ == "__main__":
    main()
