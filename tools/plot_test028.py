#!/usr/bin/env python3
"""Figure for TEST-028 (R1.10: configuration files).

  docs/test_reports/test028_sensitivity.png   which filter settings matter: each one halved
                                              and doubled, 10 minutes after GNSS loss

Drawn from docs/test_reports/test028_data/sensitivity.csv, written by
tools/config_sensitivity.py (see the report for the scenario).

    python3 tools/plot_test028.py
"""
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from gdn_paths import ROOT

C1, C2 = "#2a78d6", "#eb6834"      # halved / lower, doubled / higher
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#d9d8d4", "#fcfcfb"
DATA = ROOT / "docs/test_reports/test028_data/sensitivity.csv"
OUT = ROOT / "docs/test_reports/test028_sensitivity.png"
TOP = 18           # settings drawn, by largest effect
EPS_M, EPS_PP = 0.05, 0.005   # below both: "no effect"


def style(ax):
    ax.set_facecolor(SURF)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=8.5, length=0)
    ax.grid(True, axis="x", color=GRID, lw=0.5)
    ax.set_axisbelow(True)
    ax.axvline(0, color=INK2, lw=0.8, zorder=1)


def main():
    rows = list(csv.DictReader(open(DATA)))
    base = next(r for r in rows if r["key"] == "(default)")
    b95, bo3 = float(base["cep95_m"]), float(base["over3_pct"])
    eff = {}
    for r in rows[1:]:
        k = r["key"]
        d = eff.setdefault(k, [])
        if r["status"] != "ok":
            d.append((r["change"], None, None))
        else:
            d.append((r["change"], float(r["cep95_m"]) - b95, float(r["over3_pct"]) - bo3))

    def size(k):
        return max((abs(x[1]) if x[1] is not None else 1e9) for x in eff[k])

    ranked = sorted(eff, key=size, reverse=True)
    none = [k for k in ranked if all(x[1] is not None and abs(x[1]) < EPS_M and abs(x[2]) < EPS_PP for x in eff[k])]
    shown = [k for k in ranked if k not in none][:TOP][::-1]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 0.34 * len(shown) + 2.6), facecolor=SURF, sharey=True,
                                 gridspec_kw={"wspace": 0.08})
    for ax in (a1, a2):
        style(ax)
    for yi, k in enumerate(shown):
        for j, (lab, d95, do3) in enumerate(eff[k]):
            c = C1 if j == 0 else C2
            if d95 is None:
                a1.text(0, yi + (0.18 if j == 0 else -0.18), f"  {lab}: run failed", fontsize=7.5, color=c, va="center")
                continue
            off = 0.15 if j == 0 else -0.15
            for ax, v in ((a1, d95), (a2, do3)):
                ax.plot([0, v], [yi + off] * 2, color=c, lw=2.2, solid_capstyle="round", zorder=2)
                ax.plot([v], [yi + off], "o", ms=5.5, color=c, mec=SURF, mew=1.0, zorder=3)
    labels = []
    for k in shown:
        alts = [x[0] for x in eff[k]]
        labels.append(k if alts == ["x0.5", "x2"] else f"{k} ({' / '.join(alts)})")
    a1.set_yticks(range(len(shown)))
    a1.set_yticklabels(labels, fontsize=8.5, color=INK)
    a1.set_xlabel(f"change in CEP95 at 10 min, m (default {b95:.1f} m)", color=INK2, fontsize=9)
    a2.set_xlabel(f"change in samples over 3 sigma, percentage points (default {bo3:.2f}%)", color=INK2, fontsize=9)
    a1.set_title("Accuracy: right is worse", loc="left", fontsize=10, color=INK)
    a2.set_title("Integrity (REQ-INT-001): right is worse", loc="left", fontsize=10, color=INK)
    a1.plot([], [], "o-", color=C1, label="halved (or first alternative)")
    a1.plot([], [], "o-", color=C2, label="doubled (or second alternative)")
    a1.legend(loc="lower right", fontsize=8, frameon=False, labelcolor=INK2)
    n = int(base["flights"])
    fig.suptitle(f"Which filter settings matter: each halved and doubled, {n} flights on SRTM-like maps, GNSS lost "
                 f"at 150 s (largest {len(shown)} effects).\nNearly every change makes it worse; doubling r_agl "
                 f"improves both accuracy and integrity.", x=0.01, ha="left", fontsize=11, color=INK)
    note = (f"No effect in this scenario ({len(none)}): " + ", ".join(sorted(none))) if none else ""
    if note:
        import textwrap
        fig.text(0.01, 0.005, textwrap.fill(note, 190), fontsize=7.5, color=INK2, va="bottom")
    fig.subplots_adjust(left=0.2, right=0.985, top=1 - 1.0 / fig.get_figheight(),
                        bottom=(1.15 if note else 0.6) / fig.get_figheight())
    fig.savefig(OUT, dpi=150, facecolor=SURF)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
