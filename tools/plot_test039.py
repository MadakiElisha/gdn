#!/usr/bin/env python3
"""TEST-039 figure: the flight regime from PX4 (R2.5b).

    # from the batch's run directories (needs ROS: the dev image), once:
    python3 tools/plot_test039.py --export docs/test_reports/test039_data log/r25b/t039
    # from the exported files (what's committed):
    python3 tools/plot_test039.py docs/test_reports/test039_data --out docs/test_reports/test039_regime.png

Export, per airframe: GDN's states (10 Hz: time, horizontal speed, regime flags) and its
applied zero-velocity updates from the flight's run log and from its --no-regime replay
(the same inputs as GDN ran before R2.5b), PX4's regime track, and the true ground speed
from the bag. The figure: per airframe, the whole flight shaded by PX4's regime with the
true speed and GDN's with and without the regime, and the time after landing close up
with each applied ZUPT; then the check results (sitl_<airframe>.txt).
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
WITH, WITHOUT = "#2a78d6", "#eb6834"
PHASE = {"ground": "#eeede9", "hover": "#e4eefa", "transition": "#fbe7da", "cruise": "#e6f2e1", "unknown": "#ffffff"}
VEHICLES = ("x500", "standard_vtol", "rc_cessna")
TITLE = {"x500": "x500 (multirotor)", "standard_vtol": "standard VTOL", "rc_cessna": "cessna (fixed wing)"}


def export(out, batch):
    import check_regime as cr
    import check_sim_sensors as cs
    out, batch = Path(out), Path(batch)
    out.mkdir(parents=True, exist_ok=True)
    for v in VEHICLES:
        run = batch / v
        log = cr.run_log_of(run)
        for tag, path in (("with", log), ("noregime", Path(f"{run}.noregime"))):
            d = cr.read_log(path)
            st = np.array(d["states"], dtype=float)
            up = np.array(d["updates"], dtype=float)
            z = up[(up[:, 1] == cr.S_ZUPT) & (up[:, 2] == cr.APPLIED), 0] if len(up) else np.zeros(0)
            with open(out / f"{v}_{tag}_states.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_s", "speed_mps", "regime_code"])
                for r in st[::2]:
                    w.writerow([f"{r[0]:.2f}", f"{np.hypot(r[1], r[2]):.4f}", int(r[4]) >> 4])
            np.savetxt(out / f"{v}_{tag}_zupt.csv", z, fmt="%.3f", header="t_s", comments="")
            if tag == "with":
                track = cr.px4_track(d["status"], d["land"])
                with open(out / f"{v}_px4_regime.csv", "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["t_s", "regime"])
                    w.writerows([(f"{t:.3f}", g) for t, g in track])
        # The true ground speed, from the bag (receive time; GDN's log is on PX4's clock,
        # which agrees with it to about 50 ms in SITL).
        _, msgs, truth = cs.read_bag(run / "sitl_bag", [])
        tr = np.array(truth, dtype=float)
        if len(tr):
            # Differentiated on Gazebo's own clock (column 9; receive times jitter), then put
            # on the receive clock.
            t, e, n, tg = tr[:, 0], tr[:, 1], tr[:, 2], tr[:, 9]
            keep = np.concatenate([[True], np.diff(tg) > 0])
            t, e, n, tg = t[keep], e[keep], n[keep], tg[keep]
            sp_g = np.hypot(np.gradient(e, tg), np.gradient(n, tg))
            grid = np.arange(t[0], t[-1], 0.2)
            sp = np.interp(grid, t, sp_g)
            np.savetxt(out / f"{v}_truth_speed.csv", np.column_stack([grid, sp]), fmt="%.3f", delimiter=",",
                       header="t_s,speed_mps", comments="")
        print(f"exported {v}")


def load_csv(p):
    return np.genfromtxt(p, delimiter=",", names=True, dtype=None, encoding=None) if Path(p).exists() else None


def results(d):
    rows = []
    texts = {v: (Path(d) / f"sitl_{v}.txt").read_text() if (Path(d) / f"sitl_{v}.txt").exists() else "" for v in VEHICLES}

    def section(t, which):
        parts = t.split("# tools/check_regime.py")
        return parts[1] if which == "with" and len(parts) > 1 else (parts[2] if len(parts) > 2 else "")

    labels = [("GDN went through the regimes", r"went through"), ("each PX4 change followed within 0.5 s", r"followed by GDN"),
              ("no ZUPT applied in the air", r"no zero-velocity update applied"),
              ("ZUPT re-armed after landing", r"re-armed after landing"),
              ("speed on the ground after landing < 0.3 m/s", r"speed on the ground"),
              ("altimeter not applied on the ground", r"terrain altimeter not applied")]
    for lab, pat in labels:
        for which, name in (("with", "with the regime"), ("without", "without (replay)")):
            row = {}
            for v in VEHICLES:
                line = next((l for l in section(texts[v], which).splitlines() if re.search(pat, l)), "")
                if not line:
                    row[v] = "n/a"
                    continue
                det = line.split(": ", 2)[-1] if line.count(": ") >= 2 else ""
                row[v] = ("pass" if line.startswith("PASS") else "FAIL") + (f" ({det[:38]})" if det else "")
            rows.append((f"{lab}: {name}", row))
    rep = {v: ((m := re.search(r"IDENTICAL: (\d+) states", texts[v])) and f"identical, {int(m.group(1)):,} states")
           or "n/a" for v in VEHICLES}
    rows.append(("Run log replays bit for bit, with the regime", rep))
    return rows


def draw(d, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    d = Path(d)
    fig = plt.figure(figsize=(15, 15.5))
    gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1, 1.35], hspace=0.42, wspace=0.22, top=0.94, bottom=0.02)
    for row, v in enumerate(VEHICLES):
        px4 = load_csv(d / f"{v}_px4_regime.csv")
        w = load_csv(d / f"{v}_with_states.csv")
        wo = load_csv(d / f"{v}_noregime_states.csv")
        tru = load_csv(d / f"{v}_truth_speed.csv")
        if w is None:
            continue
        t0 = float(w["t_s"][0])
        zw = np.atleast_1d(np.loadtxt(d / f"{v}_with_zupt.csv", skiprows=1)) - t0
        zo = np.atleast_1d(np.loadtxt(d / f"{v}_noregime_zupt.csv", skiprows=1)) - t0
        tracks = [(float(t) - t0, str(g)) for t, g in zip(np.atleast_1d(px4["t_s"]), np.atleast_1d(px4["regime"]))]
        t_end = float(w["t_s"][-1]) - t0
        land = [t for (t, g), (tp, gp) in zip(tracks[1:], tracks[:-1]) if g == "ground" and gp != "ground"]
        t_land = land[-1] if land else t_end - 30
        for col, (lo, hi, title) in enumerate(((None, None, "the whole flight"), (t_land - 15, t_end, "after landing"))):
            ax = fig.add_subplot(gs[row, 0:2] if col == 0 else gs[row, 2])
            for k, (t, g) in enumerate(tracks):
                ax.axvspan(t, tracks[k + 1][0] if k + 1 < len(tracks) else t_end, color=PHASE.get(g, "#ffffff"), lw=0)
            if tru is not None:
                ax.plot(tru["t_s"] - t0, tru["speed_mps"], color=INK, lw=1.2, label="true ground speed")
            ax.plot(wo["t_s"] - t0, wo["speed_mps"], color=WITHOUT, lw=1.1, label="GDN without the regime (replay)")
            ax.plot(w["t_s"] - t0, w["speed_mps"], color=WITH, lw=1.3, label="GDN with the regime from PX4")
            if col == 1:
                top = ax.get_ylim()[1]
                ax.scatter(zw[zw > lo], np.full((zw > lo).sum(), 0.04 * top), marker="|", s=60, color=WITH,
                           label=f"ZUPT applied with the regime ({(zw > t_land).sum()} after landing)")
                ax.scatter(zo[zo > lo], np.full((zo > lo).sum(), 0.1 * top), marker="|", s=60, color=WITHOUT,
                           label=f"ZUPT applied without ({(zo > t_land).sum()} after landing)")
                ax.axvline(t_land, color=MUTED, lw=0.8, ls=(0, (2, 2)))
                ax.set_xlim(lo, hi)
                ws = w["speed_mps"][(w["t_s"] - t0) > t_end - 5]
                os_ = wo["speed_mps"][(wo["t_s"] - t0) > t_end - 5]
                ax.set_title(f"{TITLE[v]}: {title} (last 5 s: {np.median(ws):.2f} vs {np.median(os_):.2f} m/s)")
                ax.legend(loc="upper right", fontsize=7.5)
            else:
                ax.set_xlim(0, t_end)
                ax.set_title(f"{TITLE[v]}: {title}, shaded by PX4's regime")
                ax.legend(loc="upper left", fontsize=7.5, ncol=3)
            ax.set_ylim(bottom=0)
            ax.grid(color=GRID, lw=0.5)
            ax.set_xlabel("time since GDN aligned (s)")
            ax.set_ylabel("ground speed (m/s)")
    fig.legend(handles=[Patch(color=PHASE[g], label=g) for g in ("ground", "hover", "transition", "cruise")],
               loc="upper right", ncol=4, bbox_to_anchor=(0.98, 0.985), title="PX4's regime", title_fontsize=8)
    ax = fig.add_subplot(gs[3, :])
    ax.axis("off")
    rows = results(d)
    tb = ax.table(cellText=[[r[0]] + [r[1].get(v, "") for v in VEHICLES] for r in rows],
                  colLabels=["", "x500", "standard VTOL", "cessna"], cellLoc="left", loc="upper center",
                  colWidths=[0.37, 0.21, 0.21, 0.21])
    tb.auto_set_font_size(False)
    tb.set_fontsize(7.8)
    tb.scale(1, 1.3)
    for (r, c), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        if r == 0:
            cell.get_text().set_fontweight("bold")
        if c == 0:
            cell.get_text().set_color(INK2)
        if c > 0 and r > 0 and cell.get_text().get_text().startswith("FAIL"):
            cell.get_text().set_color("#b3261e")
    ax.set_title("tools/check_regime.py on each flight, and on its --no-regime replay (the same inputs)", pad=6)
    fig.suptitle("TEST-039: the flight regime from PX4 (R2.5b): ZUPT only on the ground, re-armed after landing",
                 x=0.04, ha="left", fontsize=13, fontweight="bold", color=INK)
    fig.savefig(out, dpi=110, facecolor="white")
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test039_data")
    ap.add_argument("--export", metavar="BATCH_DIR", help="export from this batch directory into DATA")
    ap.add_argument("--out", default="docs/test_reports/test039_regime.png")
    a = ap.parse_args()
    if a.export:
        export(a.data, a.export)
    else:
        draw(a.data, a.out)


if __name__ == "__main__":
    main()
