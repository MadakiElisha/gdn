#!/usr/bin/env python3
"""TEST-034 figure: the three simulated vehicles (R2.3) flying the same route.

    # from tools/sim_run.sh log directories (one per vehicle and world):
    python3 tools/plot_test034.py --export docs/test_reports/test034_data log/r23/t034/zermatt_x500 ...
    # from the exported CSVs (what's committed):
    python3 tools/plot_test034.py docs/test_reports/test034_data --out docs/test_reports/test034_vehicles.png

Export writes, per run, the true track at 2 Hz (with height above the world's terrain
and what PX4 was doing), the planned route and tools/check_vehicles.py's verdict; per
world, the terrain at 40 m; and, per run, the radar altimeter against the terrain
straight below and along its beam, by tilt (OI-016).
"""
import argparse
import csv
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

VEH = {"x500": ("#2a78d6", "-", "x500 (quadrotor)"),
       "standard_vtol": ("#eb6834", "--", "standard VTOL"),
       "rc_cessna": ("#2b2a28", ":", "rc_cessna (fixed-wing)")}
WORLD = {"zermatt": "Zermatt (SRTM, mountain)", "ch_emmental": "Emmental (swissALTI3D, rolling hills)"}
INK, INK2, GRID = "#0b0b0b", "#52514e", "#d9d8d4"
TILTS = ((0, 5), (5, 15), (15, 25), (25, 35.01))     # up to sim_adapter.py's 35 deg


def export(out, runs):
    import check_vehicles as cv
    from gdn_paths import load_map
    from make_sim_world import sample_map
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    radar_rows = []
    for d in map(Path, runs):
        info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
        winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                     if "=" in l)
        tag = f"{info['world']}_{info['vehicle']}"
        alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
        alt = alt.astype(np.float64)
        z0 = float(winfo["z0_amsl"])
        truth, status, agl, scan = cv.read_truth(d / "sitl_bag")
        k = np.clip(np.searchsorted(status[:, 0], truth[:, 0], side="right") - 1, 0, len(status) - 1)
        st = status[k]
        ground = sample_map(alt, hdr, truth[:, 1], truth[:, 2]) - z0
        keep = np.arange(0, len(truth), 10)                  # 20 Hz -> 2 Hz
        with open(out / f"{tag}_track.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "east_m", "north_m", "z_world_m", "h_agl_m", "plane", "following"])
            for i in keep:
                w.writerow([f"{truth[i, 0] - truth[0, 0]:.1f}", f"{truth[i, 1]:.1f}", f"{truth[i, 2]:.1f}",
                            f"{truth[i, 3]:.1f}", f"{truth[i, 3] - ground[i]:.1f}", int(st[i, 1] == 2),
                            int(st[i, 3] == 14 and st[i, 2] == 0)])
        (out / f"{tag}_route.csv").write_text((d / "route.csv").read_text())
        r = subprocess.run([sys.executable, str(Path(__file__).with_name("check_vehicles.py")), "--run", str(d)],
                           capture_output=True, text=True)
        (out / f"{tag}_check.txt").write_text(r.stdout)
        # measured, along the beam, tilt, straight below (all as heights)
        b = cv.beam_scan(truth, scan, alt, hdr, z0) if len(scan) else cv.beam_agl(truth, agl, alt, hdr, z0)
        for lo, hi in TILTS:
            sel = b[(b[:, 2] >= lo) & (b[:, 2] < hi)]
            if len(sel) < 5:
                continue
            radar_rows.append([info["world"], info["vehicle"], lo, hi, len(sel),
                               f"{np.sqrt(np.mean((sel[:, 0] - sel[:, 3]) ** 2)):.2f}",
                               f"{np.sqrt(np.mean((sel[:, 0] - sel[:, 1]) ** 2)):.2f}"])
        rwf = out / f"{info['world']}_runway.txt"
        if "runway_heading" in winfo and not rwf.exists():
            rwf.write_text("".join(f"{k}={winfo[k]}\n" for k in winfo if k.startswith("runway_")))
        tf = out / f"{info['world']}_terrain.csv"
        if not tf.exists():
            half_e, half_n = float(winfo["size_e"]) / 2, float(winfo["size_n"]) / 2
            ee = np.arange(-half_e, half_e, 40.0)
            nn = np.arange(-half_n, half_n, 40.0)
            E, N = np.meshgrid(ee, nn)
            H = sample_map(alt, hdr, E, N) - z0
            with open(tf, "w") as f:
                f.write(f"# terrain above world z=0, 40 m grid; east {ee[0]:.0f} to {ee[-1]:.0f} m, "
                        f"north {nn[0]:.0f} to {nn[-1]:.0f} m (rows south to north)\n")
                np.savetxt(f, H, fmt="%.0f", delimiter=",")
        print(f"exported {tag}")
    with open(out / "radar_by_tilt.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["world", "vehicle", "tilt_lo_deg", "tilt_hi_deg", "n", "rms_vs_nadir_m", "rms_vs_beam_m"])
        w.writerows(radar_rows)


def load(d):
    d = Path(d)
    runs = {}
    for f in sorted(d.glob("*_track.csv")):
        tag = f.name[:-len("_track.csv")]
        world, veh = next((w, tag[len(w) + 1:]) for w in WORLD if tag.startswith(w + "_"))
        tr = np.genfromtxt(f, delimiter=",", names=True)
        route = np.loadtxt(d / f"{tag}_route.csv", delimiter=",", skiprows=1)
        check = (d / f"{tag}_check.txt").read_text() if (d / f"{tag}_check.txt").exists() else ""
        runs[(world, veh)] = dict(tr=tr, route=route, check=check)
    terrain = {}
    for w in WORLD:
        f = d / f"{w}_terrain.csv"
        if f.exists():
            head = f.read_text().splitlines()[0]
            nums = [float(x) for x in head.replace(",", " ").split() if x.lstrip("-").replace(".", "").isdigit()]
            terrain[w] = (np.loadtxt(f, delimiter=",", comments="#"), nums)   # nums: grid, e0, e1, n0, n1
    radar = list(csv.DictReader(open(d / "radar_by_tilt.csv"))) if (d / "radar_by_tilt.csv").exists() else []
    runway = {}
    for w in WORLD:
        f = d / f"{w}_runway.txt"
        if f.exists():
            runway[w] = {k[len("runway_"):]: float(v) for k, v in (l.split("=", 1) for l in f.read_text().split())}
    return runs, terrain, radar, runway


def xtrack(route, e, n):
    import check_vehicles as cv
    return cv.cross_track(route[:, :2], np.column_stack([n, e]))


def plot(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource
    runs, terrain, radar, runway = load(d)
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold"})
    fig = plt.figure(figsize=(13, 10.2))
    gs = fig.add_gridspec(2, 6, height_ratios=[1.3, 1], hspace=0.3, wspace=0.9)
    worlds = [w for w in WORLD if any(k[0] == w for k in runs)]
    # Top: the tracks over each world.
    for i, w in enumerate(worlds):
        ax = fig.add_subplot(gs[0, 3 * i:3 * i + 3] if len(worlds) == 2 else gs[0, :])
        if w in terrain:
            H, nums = terrain[w]
            _, e0, e1, n0, n1 = nums[:5]
            ls = LightSource(azdeg=315, altdeg=40)
            shade = ls.hillshade(H[::-1], vert_exag=1.0, dx=40, dy=40)
            ax.imshow(shade, extent=(e0, e1 + 40, n0, n1 + 40), cmap="gray", vmin=-0.3, vmax=1.4, origin="upper",
                      interpolation="bilinear")
            ax.contour(np.arange(e0, e1 + 1, 40), np.arange(n0, n1 + 1, 40), H, levels=12, colors="#ffffff",
                       linewidths=0.4, alpha=0.6)
        route = next(r["route"] for k, r in runs.items() if k[0] == w)
        ax.plot(route[:, 1], route[:, 0], color="#ffffff", lw=4.0, alpha=0.9, zorder=2)
        ax.plot(route[:, 1], route[:, 0], color=INK2, lw=0.8, ls=(0, (3, 2)), zorder=3, label="planned route")
        for veh, (c, lsy, label) in VEH.items():
            if (w, veh) not in runs:
                continue
            tr = runs[(w, veh)]["tr"]
            air = tr["h_agl_m"] > 3
            ok = "check exit 0" in runs[(w, veh)]["check"] or "FAIL" not in runs[(w, veh)]["check"]
            ax.plot(tr["east_m"][air], tr["north_m"][air], color=c, ls=lsy, lw=1.6, zorder=4,
                    label=label + ("" if ok else " (check FAILED)"))
        if w in runway:
            r = runway[w]
            h = np.radians(r["heading"])
            u = np.array([np.sin(h), np.cos(h)])          # along (east, north)
            v = np.array([np.cos(h), -np.sin(h)])         # across
            c = [u * a + v * b for a, b in ((r["from"], -r["width"] / 2), (r["to"], -r["width"] / 2),
                                           (r["to"], r["width"] / 2), (r["from"], r["width"] / 2))]
            ax.fill([p[0] for p in c], [p[1] for p in c], color="#ffd23f", ec=INK, lw=0.6, zorder=3.5,
                    label="runway" if i == 0 else None)
        ax.plot(0, 0, marker="s", ms=7, color="#ffffff", mec=INK, zorder=5)
        ax.annotate("pad", (0, 0), (8, -14), textcoords="offset points", color=INK, fontsize=8)
        span = np.abs(route[:, :2]).max() + 250
        ax.set_xlim(-span, span)
        ax.set_ylim(-span, span)
        ax.set_aspect("equal")
        ax.set_xlabel("east of the pad (m)")
        ax.set_ylabel("north of the pad (m)")
        ax.set_title(WORLD[w], loc="left")
        if i == 0:
            fig.legend(*ax.get_legend_handles_labels(), loc="upper center", ncol=4, fontsize=8.5,
                       frameon=False, bbox_to_anchor=(0.5, 0.955))
    # Bottom left: height above the ground along the flight, Zermatt.
    ax = fig.add_subplot(gs[1, 0:2])
    w0 = worlds[0]
    for veh, (c, lsy, label) in VEH.items():
        if (w0, veh) not in runs:
            continue
        tr = runs[(w0, veh)]["tr"]
        air = np.nonzero(tr["h_agl_m"] > 3)[0]
        t = tr["t_s"][air] - tr["t_s"][air[0]]
        ax.plot(t / 60, tr["h_agl_m"][air], color=c, ls=lsy, lw=1.2, label=label)
    ax.axhline(25, color=INK2, lw=0.8)
    ax.annotate("check: at least 25 m (away from the pad)", (0.2, 27), color=INK2, fontsize=7.5)
    ax.set_xlabel("minutes airborne")
    ax.set_ylabel("height above the ground (m)")
    ax.set_title(f"Terrain clearance, {WORLD[w0].split(' (')[0]}", loc="left")
    ax.grid(color=GRID, lw=0.5)
    # Bottom middle: cross-track while following the route.
    ax = fig.add_subplot(gs[1, 2:4])
    labels, vals, cols, lims = [], [], [], []
    for w in worlds:
        for veh, (c, _, _) in VEH.items():
            if (w, veh) not in runs:
                continue
            r = runs[(w, veh)]
            tr = r["tr"]
            far = np.hypot(tr["east_m"], tr["north_m"]) > 150
            sel = far & (tr["following"] == 1) & (tr["h_agl_m"] > 3)
            xt = xtrack(r["route"], tr["east_m"][sel], tr["north_m"][sel])
            plane = tr["plane"][sel] == 1
            for name, m, lim in (("hover", ~plane, 15), ("plane", plane, 40)):
                if m.sum() < 20:
                    continue
                labels.append(f"{w.replace('ch_', '')}\n{veh.replace('standard_', '').replace('rc_', '')} {name}")
                vals.append((np.median(xt[m]), np.percentile(xt[m], 95), xt[m].max()))
                cols.append(c)
                lims.append(lim)
    y = np.arange(len(labels))[::-1]
    for yi, (med, p95, mx), c, lim in zip(y, vals, cols, lims):
        ax.plot([0, mx], [yi, yi], color=GRID, lw=1, zorder=1)
        ax.plot(med, yi, "o", color=c, ms=5, zorder=3)
        ax.plot(p95, yi, "|", color=c, ms=14, mew=2.5, zorder=3)
        ax.plot(lim, yi, "x", color=INK2, ms=5, zorder=2)
        ax.annotate(f"{p95:.0f}", (p95, yi), (5, 3), textcoords="offset points", fontsize=7, color=INK)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=7)
    ax.set_xlabel("cross-track from the route (m): dot median, bar p95, x check limit")
    ax.set_title("Route following (while offboard)", loc="left")
    ax.grid(axis="x", color=GRID, lw=0.5)
    # Bottom right: radar against the ground straight below and along the beam (OI-016).
    ax = fig.add_subplot(gs[1, 4:6])
    top = 0.0
    for veh, (c, lsy, label) in VEH.items():
        rows = [r for r in radar if r["vehicle"] == veh and r["world"] == worlds[0]]
        if not rows:
            continue
        mid = [(float(r["tilt_lo_deg"]) + float(r["tilt_hi_deg"])) / 2 for r in rows]
        ax.plot(mid, [float(r["rms_vs_nadir_m"]) for r in rows], color=c, ls=lsy, marker="o", ms=4, lw=1.2,
                label=label)
        top = max(top, max(float(r["rms_vs_nadir_m"]) for r in rows))
        ax.plot(mid, [float(r["rms_vs_beam_m"]) for r in rows], color=c, ls=lsy, marker="s", ms=4, lw=0.8,
                alpha=0.6)
    from matplotlib.lines import Line2D
    handles, _ = ax.get_legend_handles_labels()
    handles += [Line2D([], [], color=INK2, marker="o", ls="none", ms=4, label="dots: against the ground straight below"),
                Line2D([], [], color=INK2, marker="s", ls="none", ms=4, alpha=0.6,
                       label="squares: along the beam (the sensor's 0.3 m noise)")]
    ax.set_xlabel("tilt of the vehicle (deg)")
    ax.set_ylabel("radar height minus true height, RMS (m)")
    ax.set_ylim(0, top * 1.7)
    ax.set_title(f"Altimeter by tilt, {WORLD[worlds[0]].split(' (')[0]} (OI-016)", loc="left")
    ax.grid(color=GRID, lw=0.5)
    ax.legend(handles=handles, fontsize=7, loc="upper left")
    fig.suptitle("TEST-034: x500, standard VTOL and rc_cessna, each with the GDN payload, on the same route (R2.3)",
                 x=0.06, y=0.99, ha="left", fontsize=12, fontweight="bold", color=INK)
    fig.savefig(out, dpi=130, bbox_inches="tight", facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test034_data")
    ap.add_argument("--export", metavar="OUT_DIR", help="export these run directories into OUT_DIR")
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--out", default="docs/test_reports/test034_vehicles.png")
    a = ap.parse_args()
    if a.export:
        export(a.export, ([a.data] if a.data else []) + a.runs)
    else:
        plot(a.data, a.out)


if __name__ == "__main__":
    main()
