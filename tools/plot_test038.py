#!/usr/bin/env python3
"""TEST-038 figure: the vehicle profiles (R2.5a).

    python3 tools/plot_test038.py --out docs/test_reports/test038_profiles.png

Top: each simulated airframe's sensors from its profile (config/vehicles/), side view
(x forward, z up) and from above (x forward, y left), with the direction each one looks
or measures along: a camera's optical axis, the lidar's dome axis, a range sensor's beam,
an IMU's x axis. Points
are labelled; mounts are placeholders until R6.4. Bottom: what the check, the selftest
and the SITL flights found (docs/test_reports/test038_data/).
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicle_profile as vp  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#8a8984", "#d9d8d4"
KIND_COLOUR = {"imu": "#2a78d6", "camera": "#eb6834", "lidar": "#1f9e6a", "range": "#8e5bd0",
               "radar_altimeter": "#c23b5c", "baro": "#52514e", "mag": "#52514e"}
VEHICLES = ("x500", "standard_vtol", "rc_cessna")


def look_axis(s):
    """The unit vector a sensor looks or measures along, in base_link."""
    R = vp.rot(np.radians(s["rpy_deg"]))
    if s["kind"] in ("camera", "lidar"):
        return R @ np.array([0, 0, 1.0])        # a camera's optical z; the lidar's dome axis
    return R @ np.array([1.0, 0, 0])            # x: a gpu_lidar's beam, an IMU's x axis


def draw(out, data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.titlesize": 10, "axes.titleweight": "bold",
                         "axes.titlelocation": "left", "legend.frameon": False})
    profiles = {v: vp.load(ROOT / "config/vehicles" / f"{v}.yaml") for v in VEHICLES}
    fig = plt.figure(figsize=(15, 10.6))
    gs = fig.add_gridspec(3, 3, height_ratios=[1.0, 0.82, 1.15], hspace=0.32, wspace=0.25, top=0.92, bottom=0.02)
    for col, name in enumerate(VEHICLES):
        p = profiles[name]
        for row, (ia, ib, xl, yl, title) in enumerate(((0, 2, "x forward (m)", "z up (m)", "side view"),
                                                       (0, 1, "x forward (m)", "y left (m)", "from above"))):
            ax = fig.add_subplot(gs[row, col])
            ax.axhline(0, color=GRID, lw=0.8)
            ax.axvline(0, color=GRID, lw=0.8)
            ax.scatter([0], [0], marker="P", s=110, color=INK, zorder=5)
            ax.annotate("base_link\n(autopilot: IMU, baro, mag)", (0, 0), xytext=(-0.135, 0.03 if row == 0 else 0.07),
                        fontsize=7, color=INK2)
            seen = {}
            for sid, s in p["sensors"].items():
                if s["kind"] in ("baro", "mag") or s.get("sim", "").startswith("autopilot:"):
                    continue                     # the autopilot's sensors: at base_link (marked)
                a, b = s["xyz"][ia], s["xyz"][ib]
                key = (round(a, 4), round(b, 4))
                k = seen.get(key, 0)
                seen[key] = k + 1
                c = KIND_COLOUR[s["kind"]]
                d = look_axis(s)
                ax.annotate("", (a + 0.045 * d[ia], b + 0.045 * d[ib]), (a, b),
                            arrowprops=dict(arrowstyle="-|>", color=c, lw=1.3), zorder=4)
                ax.scatter([a], [b], s=34, color=c, edgecolor="#ffffff", zorder=6)
                ax.annotate(sid, (a, b), xytext=(5, -10 - 9 * k if row == 0 else 4 + 9 * k), textcoords="offset points",
                            fontsize=7, color=INK)
            ax.set_xlim(-0.15, 0.17)
            ax.set_ylim(-0.17, 0.06) if row == 0 else ax.set_ylim(-0.09, 0.09)
            ax.set_aspect("equal")
            ax.grid(color=GRID, lw=0.4)
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            t = p["top"]
            ax.set_title(f"{name} ({t['kind']}): {title}" if row == 0 else f"{name}: {title}")
            if row == 0:
                ax.text(0.99, 0.03, f"profile {p['fingerprint']}\npayload {-p['sensors']['agl_radar']['xyz'][2]:.2f} m "
                        "under base_link", transform=ax.transAxes, ha="right", va="bottom", fontsize=7, color=INK2)
    # Legend for the kinds.
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], marker="o", ls="", color=c, label={"radar_altimeter": "radar altimeter"}.get(k, k))
               for k, c in KIND_COLOUR.items() if k not in ("baro", "mag")]
    fig.legend(handles=handles, loc="upper right", ncol=5, fontsize=8, bbox_to_anchor=(0.98, 0.985),
               title="arrow: where it looks or measures (camera: optical axis; lidar: its dome's axis; IMU, range: x)", title_fontsize=8)
    # Results.
    ax = fig.add_subplot(gs[2, :])
    ax.axis("off")
    cols = ["", "x500", "standard VTOL", "cessna"]
    rows = [[r[0]] + [r[1].get(v, "") for v in VEHICLES] for r in data]
    tb = ax.table(cellText=rows, colLabels=cols, cellLoc="left", loc="upper center", colWidths=[0.4, 0.2, 0.2, 0.2])
    tb.auto_set_font_size(False)
    tb.set_fontsize(8.5)
    tb.scale(1, 1.42)
    for (r, c), cell in tb.get_celld().items():
        cell.set_edgecolor(GRID)
        if r == 0:
            cell.get_text().set_fontweight("bold")
        if c == 0:
            cell.get_text().set_color(INK2)
    ax.set_title("What was checked", pad=6)
    fig.suptitle("TEST-038: a vehicle profile per airframe (R2.5a): sensors, mounts, noise and aiding",
                 x=0.04, ha="left", fontsize=13, fontweight="bold", color=INK)
    fig.savefig(out, dpi=115, facecolor="white")
    print(f"wrote {out}")


def results(d):
    """Table rows from test038_data: [(label, {vehicle: text})]."""
    d = Path(d)
    chk = (d / "check_profiles.txt").read_text() if (d / "check_profiles.txt").exists() else ""
    rows = []

    def per(label, fn):
        rows.append((label, {v: fn(v) for v in VEHICLES}))

    def count(v, pat):
        lines = [l for l in chk.splitlines() if l.split("  ", 1)[-1].startswith(f"{v}: ") and re.search(pat, l)]
        good = sum(l.startswith("PASS") for l in lines)
        return f"{good} of {len(lines)} pass" if lines else "n/a"

    per("Profile reads cleanly; fingerprint", lambda v: (re.search(rf"{v}: profile reads cleanly: fingerprint (\w+)", chk)
                                                       or [None, "?"])[1])
    per("Kind, PX4 airframe, Gazebo model as simulated", lambda v: count(v, r"kind, PX4 airframe"))
    per("Payload mounts where Gazebo has them (1 mm, 0.05 deg)", lambda v: count(v, r"mount matches Gazebo"))
    per("Autopilot sensors at base_link, as PX4's model", lambda v: count(v, r"defines base_link|where PX4's model"))
    per("Topics, frames, noise files as the simulated drivers", lambda v: count(v, r"as simulated"))
    sitl = {v: (d / f"sitl_{v}.txt").read_text() if (d / f"sitl_{v}.txt").exists() else "" for v in VEHICLES}
    rows.append(("SITL: the node ran with it (fingerprint in its log)",
                 {v: ((m := re.search(r"vehicle profile: \S+ \(.*?\), fingerprint (\w+)", sitl[v])) and m.group(1)) or "n/a"
                  for v in VEHICLES}))
    rows.append(("SITL: its run log replays bit for bit, with the profile",
                 {v: ((m := re.search(r"IDENTICAL: (\d+) states", sitl[v])) and f"identical, {int(m.group(1)):,} states")
                  or ("n/a" if not sitl[v] else "DIFFERENT") for v in VEHICLES}))
    rows.append(("SITL: flew the route and landed",
                 {v: ("yes" if "landed and disarmed" in sitl[v] and "giving up" not in sitl[v]
                      else ("n/a" if not sitl[v] else "no")) for v in VEHICLES}))
    st = (d / "selftest_test038.txt").read_text() if (d / "selftest_test038.txt").exists() else ""
    n_pass, n = st.count("PASS  profile: TEST-038"), st.count("profile: TEST-038")
    rows.append(("Selftest TEST-038 (loader, fingerprint as Python's, aiding off)",
                 {"x500": f"{n_pass} of {n} pass", "standard_vtol": "(not per airframe)", "rc_cessna": ""}))
    lc = (d / "log_check.txt").read_text() if (d / "log_check.txt").exists() else ""
    rows.append(("Run-log check: a profile with baro and ZUPT off replays",
                 {"x500": "bit for bit" if "a run with a vehicle profile replays bit for bit" in lc else "n/a",
                  "standard_vtol": "(bench, x500's profile)", "rc_cessna": ""}))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", nargs="?", default="docs/test_reports/test038_data")
    ap.add_argument("--out", default="docs/test_reports/test038_profiles.png")
    a = ap.parse_args()
    draw(a.out, results(a.data))


if __name__ == "__main__":
    main()
