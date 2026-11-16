#!/usr/bin/env python3
"""Draw a GDN run log (R1.11) for diagnosis, or export it as small CSVs.

    python3 tools/plot_run_log.py log/gdn_runs/gdn_20261112_145510 --out run.png
    python3 tools/plot_run_log.py RUN_LOG WHATIF_LOG --label default --label "r_agl 200" --out cmp.png
    python3 tools/plot_run_log.py RUN_LOG --export DIR          # CSVs: states (10 Hz), reference,
                                                                # updates, events
    python3 tools/plot_run_log.py DIR --out run.png              # an exported DIR draws the same

Panels, on one time axis, with the start-up events marked:
  1. horizontal error against the PX4 reference (vehicle_local_position, which is the truth
     on the bench and in SITL, and PX4's GNSS-aided estimate on a real flight; only
     meaningful when GDN's map frame is PX4's, as on the bench), with the filter's own
     3 sigma;
  2. the altimeter (AGL) innovations, applied or gated, against the 3 sigma gate;
  3. every update decision per sensor: applied, gated, or skipped (with why).
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

SENSORS = {1: "TRN", 2: "AGL", 3: "baro", 4: "mag 3-axis", 5: "mag heading", 6: "ZUPT", 7: "GNSS"}
REASONS = {0: "applied", 1: "not aligned", 2: "no heading", 3: "no datum", 4: "off map", 5: "flat", 6: "weak field",
           7: "gated", 8: "numeric", 9: "bad input", 10: "init", 11: "not used"}
C1, C2, GREY = "#2a78d6", "#eb6834", "#a3a29d"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"


def read_mcap(path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    d = {"info": {}, "states": [], "ref": [], "updates": [], "events": []}
    last_t = -1.0
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in ("/gdn/log/run_info", "/gdn/log/state", "/gdn/log/update", "/gdn/log/event",
                         "/fmu/out/vehicle_local_position"):
            continue
        m = deserialize_message(data, types[topic])
        if topic == "/gdn/log/run_info":
            d["info"] = {"version": m.gdn_version, "config": m.config_name, "fingerprint": m.config_fingerprint,
                         "overrides": m.config_overrides, "map": Path(m.map_path).name}
        elif topic == "/gdn/log/state":
            if m.t >= last_t + 0.1 and m.flags & 1:      # 10 Hz is plenty to draw
                last_t = m.t
                d["states"].append([m.t, m.position[0], m.position[1], m.position[2], m.p_diag[0], m.p_diag[1],
                                    m.p_diag[2], m.p_ne, m.flags])
        elif topic == "/gdn/log/update":
            d["updates"].append([m.seq, m.t, m.sensor, m.reason, m.innovation[0] if len(m.innovation) else math.nan,
                                 m.innovation_var[0] if len(m.innovation_var) else math.nan, m.nis,
                                 int(m.gain_limited)])
        elif topic == "/gdn/log/event":
            d["events"].append([m.seq, m.t, m.level, m.text])
        elif m.xy_valid:
            d["ref"].append([m.timestamp * 1e-6, m.x, m.y])
    return d


HEAD = {"states": ["t", "pn", "pe", "pd", "p_nn", "p_ee", "p_dd", "p_ne", "flags"], "ref": ["t", "x", "y"],
        "updates": ["seq", "t", "sensor", "reason", "innovation", "innovation_var", "nis", "gain_limited"],
        "events": ["seq", "t", "level", "text"]}


def export(d, out):
    out.mkdir(parents=True, exist_ok=True)
    for k, cols in HEAD.items():
        with open(out / f"{k}.csv", "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            w.writerows(d[k])
    with open(out / "info.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        w.writerows(d["info"].items())


def read_export(path):
    d = {}
    for k in HEAD:
        with open(path / f"{k}.csv") as f:
            rows = list(csv.reader(f))[1:]
        d[k] = [[float(x) for x in r] for r in rows] if k != "events" else \
            [[float(r[0]), float(r[1]), float(r[2]), r[3]] for r in rows]
    with open(path / "info.csv") as f:
        d["info"] = {r[0]: r[1] for r in list(csv.reader(f))[1:]}
    return d


def load(path):
    """A run log directory or an --export directory. Exits with a clear message when it
    is neither, instead of rosbag2's "No storage could be initialized"."""
    p = Path(path)
    if not p.is_dir():
        hint = (" (a what-if log is written by gdn_replay LOG --set=KEY=VALUE --out=DIR: run that first)"
                if not p.exists() else "")
        sys.exit(f"ERROR: {path}: no such run log directory{hint}")
    if (p / "states.csv").exists():
        return read_export(p)
    if not any(p.glob("*.mcap")):
        sys.exit(f"ERROR: {path}: not a run log (no .mcap file) nor an --export directory (no states.csv)")
    if not (p / "metadata.yaml").exists():
        sys.exit(f"ERROR: {path}: the log has no index (the node didn't close cleanly). "
                 f"Run: ros2 bag reindex {path} -s mcap")
    return read_mcap(p)


def start_of_clock(d):
    """The filter's time is 0 until its first propagation after static alignment, so
    the records before that (skipped "not aligned" updates, the alignment events) carry
    t = 0. On a bag that starts near 0 that's harmless; in SITL the IMU clock is Unix
    time, and t = 0 put them 1.8e9 s before the rest. Put them at the first real time."""
    ts = [r[0] for r in d["states"] if r[0] > 0]
    if not ts:
        return
    first = min(ts)
    for r in d["states"]:
        if r[0] <= 0:
            r[0] = first
    for k in ("updates", "events"):
        for r in d[k]:
            if r[1] <= 0:
                r[1] = first


def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8.5)
    ax.grid(True, color=GRID, lw=0.5)
    ax.set_axisbelow(True)


def draw(runs, labels, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 1, figsize=(12.5, 10), facecolor=SURF, sharex=True,
                            gridspec_kw={"height_ratios": [1.2, 1.0, 1.0], "hspace": 0.18})
    for ax in axs:
        style(ax)
    for d in runs:   # records from before the filter's clock started carry t = 0
        start_of_clock(d)
    t0 = min(np.array(d["states"])[:, 0].min() for d in runs)
    cols = [C1, C2]
    summary = []
    for i, (d, lab) in enumerate(zip(runs, labels)):
        c = cols[i % 2]
        s = np.array(d["states"])
        ref = np.array(d["ref"])
        t = s[:, 0] - t0
        if len(ref):
            rx = np.interp(s[:, 0], ref[:, 0], ref[:, 1])
            ry = np.interp(s[:, 0], ref[:, 0], ref[:, 2])
            err = np.hypot(s[:, 1] - rx, s[:, 2] - ry)
            sig3 = 3.0 * np.sqrt(np.maximum(s[:, 4], s[:, 5]))
            airborne = (s[:, 8].astype(int) & 8) > 0
            axs[0].plot(t, err, color=c, lw=1.6, label=f"{lab}: error")
            axs[0].plot(t, sig3, color=c, lw=1.0, ls="--", label=f"{lab}: filter's 3 sigma")
            if airborne.any():
                ea = err[airborne]
                summary.append(f"{lab}: airborne horizontal error RMS {np.sqrt(np.mean(ea ** 2)):.1f} m, "
                               f"max {ea.max():.1f} m, over 3 sigma {100 * np.mean(ea > sig3[airborne]):.1f}%")
        u = np.array(d["updates"])
        agl = u[u[:, 2] == 2]
        formed = agl[~np.isnan(agl[:, 4])]
        ok, gated = formed[formed[:, 3] == 0], formed[formed[:, 3] == 7]
        if i == 0 or len(runs) == 1:
            axs[1].fill_between(formed[:, 1] - t0, -3 * np.sqrt(formed[:, 5]), 3 * np.sqrt(formed[:, 5]),
                                color=c, alpha=0.12, lw=0, label=f"{lab}: 3 sigma gate")
        axs[1].plot(ok[:, 1] - t0, ok[:, 4], "o", ms=2.2, color=c, alpha=0.8, label=f"{lab}: applied")
        axs[1].plot(gated[:, 1] - t0, gated[:, 4], "x", ms=5, color=c, mew=1.4, label=f"{lab}: gated ({len(gated)})")
        if i == 0:
            # Decision strip: one row per sensor, applied / gated / skipped.
            rows = [k for k in (6, 5, 4, 3, 2, 1) if (u[:, 2] == k).any()]
            for yi, k in enumerate(rows):
                uk = u[u[:, 2] == k]
                for reason_set, colr, name in (((0,), C1, "applied"), ((7, 8), C2, "gated"),
                                               (tuple(range(1, 7)) + (9, 10, 11), GREY, "skipped (not aligned, "
                                                                                       "no heading/datum, init...)")):
                    sel = uk[np.isin(uk[:, 3], reason_set)]
                    axs[2].plot(sel[:, 1] - t0, np.full(len(sel), yi), "|", ms=9, color=colr,
                                label=name if yi == 0 else None)
            axs[2].set_yticks(range(len(rows)))
            axs[2].set_yticklabels([SENSORS[k] for k in rows], fontsize=8.5, color=INK2)
            axs[2].set_ylim(-0.6, len(rows) - 0.4)
            # Start-up events (info level), merged when under 2 s apart so labels don't overlap.
            groups = []
            for ev in d["events"]:
                if ev[2] != 0:
                    continue
                name = re.match(r"[A-Za-z ]+", ev[3].split(":")[0].split(" at ")[0]).group(0).strip()
                if groups and ev[1] - groups[-1][0] < 2.0:
                    groups[-1][1].append(name)
                else:
                    groups.append([ev[1], [name]])
            for te, names in groups:
                for ax in axs:
                    ax.axvline(te - t0, color=INK2, lw=0.7, ls=":", zorder=0)
                axs[0].text(te - t0, 0.98, " " + " + ".join(names), rotation=90, va="top", ha="left", fontsize=7.5,
                            color=INK2, transform=axs[0].get_xaxis_transform())
    axs[0].set_ylabel("horizontal error, m", color=INK2, fontsize=9)
    axs[0].set_ylim(bottom=0)
    axs[0].legend(fontsize=8, frameon=False, loc="upper right", ncol=2, labelcolor=INK2)
    axs[1].set_ylabel("AGL innovation, m", color=INK2, fontsize=9)
    axs[1].legend(fontsize=8, frameon=False, loc="upper right", ncol=3, labelcolor=INK2)
    axs[2].set_ylabel("update decisions", color=INK2, fontsize=9)
    axs[2].legend(fontsize=8, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.18), ncol=3, labelcolor=INK2)
    axs[2].set_xlabel("time since the first aligned state, s", color=INK2, fontsize=9)
    info = runs[0]["info"]
    fig.suptitle(f"Run log: {info.get('map', '')}, filter {info.get('config', '')} ({info.get('fingerprint', '')}), "
                 f"version {info.get('version', '')}", x=0.01, ha="left", fontsize=11, color=INK)
    if summary:
        fig.text(0.01, 0.945, "; ".join(summary), fontsize=8.5, color=INK2)
    fig.subplots_adjust(left=0.08, right=0.985, top=0.925, bottom=0.09)
    fig.savefig(out, dpi=150, facecolor=SURF)
    print(f"wrote {out}")
    for line in summary:
        print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="run log directories (or exported CSV directories); at most two")
    ap.add_argument("--label", action="append", default=[], help="name for each log, in order")
    ap.add_argument("--out", help="figure to write (PNG)")
    ap.add_argument("--export", help="write the first log's data as CSVs to this directory")
    a = ap.parse_args()
    if len(a.logs) > 2:
        sys.exit("at most two logs")
    runs = [load(p) for p in a.logs]
    for d, p in zip(runs, a.logs):
        u = np.array(d["updates"])
        print(f"{p}: {len(d['states'])} states at 10 Hz, {len(u)} update records, {len(d['events'])} events")
        for k, name in SENSORS.items():
            uk = u[u[:, 2] == k] if len(u) else u
            if len(uk):
                counts = ", ".join(f"{REASONS[int(r)]} {int((uk[:, 3] == r).sum())}" for r in np.unique(uk[:, 3]))
                print(f"  {name}: {counts}")
    if a.export:
        export(runs[0], Path(a.export))
        print(f"exported {a.export}")
    if a.out:
        labels = a.label + [Path(p).name for p in a.logs[len(a.label):]]
        draw(runs, labels, a.out)


if __name__ == "__main__":
    main()
