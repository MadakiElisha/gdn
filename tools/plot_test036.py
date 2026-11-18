#!/usr/bin/env python3
"""TEST-036 figure: the simulated OAK-D Lite cameras (R2.4b) against the real ones.

    # from a tools/sim_run.sh log directory recorded with GDN_BAG_IMAGES=1:
    python3 tools/plot_test036.py --export docs/test_reports/test036_data log/r24b/t036/ch_emmental_x500
    # from the exported files (what's committed):
    python3 tools/plot_test036.py docs/test_reports/test036_data --out docs/test_reports/test036_cameras.png

Export writes, from one run: a stereo pair in flight and one near the ground (PNG), the
stereo warp test for each pair 1.5 to 15 m up (how well the left image shifted by a
scale of the truth's disparity reproduces the right one), every pair's arrival
minus stamp (on Gazebo's clock), and the mean grey of an image every 2 s with the height above the ground.
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, GRID, BLUE, ORANGE = "#0b0b0b", "#52514e", "#d9d8d4", "#2a78d6", "#eb6834"


def export(out, run):
    import cv2
    import check_sim_sensors as cs
    from gdn_paths import load_map
    from make_sim_world import sample_map
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    d = Path(run)
    info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    st = cs.STEREO
    c = cs.load_config()[st["name"]]
    types, msgs, truth = cs.read_bag(d / "sitl_bag", [st["left"]["info"], st["right"]["info"]])
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    world = (alt.astype(np.float64), hdr, float(winfo["z0_amsl"]))
    t_rest = cs.rest_window(truth)
    clock = cs.RunClock(truth, cs.RunClock.offset_from_log(d / "sensors.log"))
    left = msgs[st["left"]["info"]]
    arr = np.array([t for t, _ in left])
    stl = np.array([cs.stamp(m) for _, m in left])
    t_sample = clock.sample(stl)                            # Gazebo time of each pair
    t0 = clock.truth[0, 0]
    with open(out / "latency.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["arrival_minus_stamp_ms"])
        for v in (clock.sim_of_wall(arr) - t_sample) * 1000.0:
            w.writerow([f"{v:.3f}"])

    def height(t):
        tr = cs.truth_at(clock.truth, t)
        return tr[3] - (float(sample_map(*world[:2], np.array(tr[1]), np.array(tr[2]))) - world[2])

    expo, low, last_e, last_l = [], [], -1e9, -1e9
    for i, t in enumerate(t_sample):
        if t - last_e >= 2.0:
            expo.append(i)
            last_e = t
        if arr[i] > t_rest[1] and t - last_l >= 0.5 and 1.5 < height(t) < 15.0:
            low.append(i)
            last_l = t
    stl_ns = [cs.stamp_ns(m) for _, m in left]
    want = {stl_ns[i] for i in expo + low}
    imgs = cs.read_images(d / "image_bag", (st["left"]["topic"], st["right"]["topic"]), want)

    def pair(i):
        k = stl_ns[i]
        a, b = imgs.get((st["left"]["topic"], k)), imgs.get((st["right"]["topic"], k))
        return (a[1], b[1]) if a and b else None
    with open(out / "exposure.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "height_m", "mean_grey", "p5", "p95"])
        for i in expo:
            p = pair(i)
            if p is not None:
                w.writerow([f"{t_sample[i] - t0:.2f}", f"{height(t_sample[i]):.1f}", f"{p[0].mean():.1f}",
                            f"{np.percentile(p[0], 5):.0f}", f"{np.percentile(p[0], 95):.0f}"])
    sdf = re.sub(r"<!--.*?-->", "", (cs.ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    lm = cs.payload_sensor(sdf, st["left"]["gz"])[1]
    best, best_h = None, 1e9                                 # the pair nearest 8 m up, for the picture
    with open(out / "warp.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "height_m", "best_scale", "pixels", "holds_disparity"] + [f"err_{x:.2f}" for x in cs.SCALES])
        k0 = int(np.argmin(np.abs(cs.SCALES)))
        for i in low:
            p = pair(i)
            if p is None:
                continue
            disp = cs.dense_disparity(world, cs.truth_at(clock.truth, t_sample[i]), (lm[:3], lm[3:]), c)
            err = cs.warp_errors(p[0], p[1], disp)
            if err is None:
                continue
            npx = int((disp >= 2.0).sum())
            h = height(t_sample[i])
            holds = int(err[k0] / err.min() >= 1.10)            # as check_sim_sensors.check_stereo
            w.writerow([f"{t_sample[i] - t0:.2f}", f"{h:.1f}", f"{cs.best_scale(err):.3f}", npx, holds]
                       + [f"{e:.4f}" for e in err])
            if holds and abs(h - 8.0) < abs(best_h - 8.0):
                best, best_h = p, h
    if best is not None:
        cv2.imwrite(str(out / "pair_low.png"), np.hstack([best[0], best[1]]))
    hi = [i for i in expo if height(t_sample[i]) > 50.0]
    if hi:
        p = pair(hi[len(hi) // 2])
        if p is not None:
            cv2.imwrite(str(out / "pair_cruise.png"), np.hstack([p[0], p[1]]))
    (out / "run.txt").write_text(f"{info['vehicle']} over {info['world']}\n")
    print(f"exported {d.name}: {len(low)} pairs below 15 m, {len(expo)} for the exposure")


def plot(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = Path(d)
    lat = np.genfromtxt(d / "latency.csv", delimiter=",", names=True)["arrival_minus_stamp_ms"]
    ex = np.genfromtxt(d / "exposure.csv", delimiter=",", names=True)
    rows = list(csv.reader(open(d / "warp.csv")))
    head, body = rows[0], np.array(rows[1:], float)
    body = body[body[:, head.index("holds_disparity")] > 0]   # the pairs the check judges by
    k0 = head.index("best_scale")
    scales = np.array([float(h[4:]) for h in head if h.startswith("err_")])
    err = body[:, [i for i, h in enumerate(head) if h.startswith("err_")]]
    best_s = body[:, k0]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(13, 9.4))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.0, 1.0, 1.15])
    for k, (name, title) in enumerate((("pair_cruise.png", "In cruise (left | right)"),
                                       ("pair_low.png", "8 m up, in the 1.5 to 15 m the stereo geometry is checked in"))):
        ax = fig.add_subplot(gs[0, k])
        if (d / name).exists():
            img = plt.imread(d / name)
            ax.imshow(img, cmap="gray", vmin=0, vmax=1 if img.max() <= 1 else 255)
        ax.set_title(title, loc="left")
        ax.axis("off")
    # Stereo geometry: the warp test.
    ax = fig.add_subplot(gs[1:, 0])
    norm = err / err.min(axis=1, keepdims=True)
    for row in norm:
        ax.plot(scales, row, "-", color=BLUE, lw=0.6, alpha=0.25)
    pooled = norm.mean(axis=0)
    ax.plot(scales, pooled, "-", color=INK, lw=2.0, label=f"mean of {len(norm)} pairs 1.5 to 15 m up")
    ax.axvline(1.0, color=INK2, lw=0.8, ls="--")
    ax.axvspan(0.95, 1.05, color=GRID, alpha=0.6, lw=0, label="the check's 0.95 to 1.05")
    ax.annotate("a swapped pair\nwould sit here", xy=(-1.0, pooled[0]), xytext=(-0.8, pooled.max() * 0.98),
                fontsize=8, color=ORANGE, arrowprops=dict(arrowstyle="->", color=ORANGE))
    ax.set_xlabel("scale of the truth's disparity used to shift the left image")
    ax.set_ylabel("difference from the right image (relative to each pair's best)")
    ax.set_title(f"Stereo geometry: best scale {np.median(best_s):.2f} (median of pairs; "
                 f"{best_s.min():.2f} to {best_s.max():.2f})", loc="left")
    ax.legend(fontsize=7.5, loc="upper center")
    ax.grid(color=GRID, lw=0.5)
    # Exposure.
    ax = fig.add_subplot(gs[1, 1])
    ax.fill_between(ex["t_s"], ex["p5"], ex["p95"], color=BLUE, alpha=0.15, lw=0, label="5th to 95th percentile")
    ax.plot(ex["t_s"], ex["mean_grey"], "-", color=BLUE, lw=1.4, label="mean grey (left image)")
    ax.axhspan(70, 170, color=GRID, alpha=0.4, lw=0)
    ax2 = ax.twinx()
    ax2.plot(ex["t_s"], ex["height_m"], "-", color=INK2, lw=0.8)
    ax2.set_ylabel("height above ground (m)", color=INK2)
    ax.set_ylim(0, 255)
    ax.set_xlabel("seconds into the recording (Gazebo time)")
    ax.set_ylabel("grey level")
    ax.set_title("Auto-exposure, 8-bit sRGB (band: the check's 70 to 170)", loc="left")
    ax.legend(fontsize=7.5, loc="lower right")
    ax.grid(color=GRID, lw=0.5)
    # Timing.
    ax = fig.add_subplot(gs[2, 1])
    ax.hist(lat, bins=np.arange(0, 60.5, 0.5), color=BLUE, alpha=0.8)
    ax.axvline(25.0, color=INK, lw=1.0)
    ax.set_xlabel("arrival minus stamp (ms); line: the configured 25 ms")
    ax.set_ylabel("pairs")
    ax.set_title(f"Timing: median {np.median(lat):.1f} ms, both images one stamp", loc="left")
    ax.grid(color=GRID, lw=0.5)
    run = (d / "run.txt").read_text().strip() if (d / "run.txt").exists() else ""
    fig.suptitle(f"TEST-036: simulated OAK-D Lite stereo cameras as depthai-ros publishes them (R2.4b); {run}",
                 x=0.04, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=120, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test036_data")
    ap.add_argument("--export", metavar="OUT_DIR", help="export this run directory into OUT_DIR")
    ap.add_argument("--out", default="docs/test_reports/test036_cameras.png")
    a = ap.parse_args()
    if a.export:
        export(a.export, a.data)
    else:
        plot(a.data, a.out)


if __name__ == "__main__":
    main()
