#!/usr/bin/env python3
"""Overview of the R2.2 simulation sites (sim/sites.yaml), one row per site:

  1. the orthophoto draped on the world (SWISSIMAGE 2 m), with the default Stage A
     route (tools/sim_fly.py, planned on the truth terrain) and the pad;
  2. the world's terrain (the truth map: swissALTI3D, or swissSURFACE3D in town),
     shaded, with its height range;
  3. GDN's onboard map (Copernicus GLO-30) minus the truth, in the world's frame: what
     the filter's map gets wrong. Trees and roofs show as GLO-30 higher (red).

    python3 tools/plot_sites.py --out docs/test_reports/test033_sites.png
    python3 tools/plot_sites.py ch_zurich --out zurich.png
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import swiss_terrain as st                                     # noqa: E402
from check_site import sample_map_frame                        # noqa: E402
from gdn_paths import load_map                                 # noqa: E402

INK, INK2, SURF = "#0b0b0b", "#52514e", "#fcfcfb"


def route(truth_path):
    """The default sim_fly.py route on this map: (north, east) points."""
    import argparse as ap_
    from sim_fly import plan_route
    alt, hdr = load_map(truth_path)
    d = ap_.Namespace(legs=4, leg_len=1400.0, spacing=250.0, centre_n=0.0, centre_e=0.0, clearance=60.0,
                      lookahead=200.0)
    r, _ = plan_route(alt.astype(np.float64), hdr, d.legs, d.leg_len, d.spacing, (d.centre_n, d.centre_e),
                      d.clearance, d.lookahead)
    return r[:, 0], r[:, 1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sites", nargs="*")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource
    from PIL import Image
    sites = st.load_sites()
    names = a.sites or list(sites)
    plt.rcParams.update({"font.size": 9, "text.color": INK, "axes.labelcolor": INK2, "xtick.color": INK2,
                         "ytick.color": INK2, "axes.edgecolor": "#d9d8d4"})
    fig, axs = plt.subplots(len(names), 3, figsize=(13, 4.25 * len(names)), facecolor=SURF, squeeze=False)
    for row, nm in zip(axs, names):
        s = sites[nm]
        p = st.site_paths(nm)
        alt, h = load_map(p["truth"])
        cell = abs(h["dlat"]) * st.M_PER_DEG_LAT
        n_ = h["nx"]
        ext = [-(n_ / 2) * cell / 1000, (n_ / 2 - 1) * cell / 1000, -(n_ / 2 - 1) * cell / 1000, (n_ / 2) * cell / 1000]
        photo = np.asarray(Image.open(p["world"] / "photo.png").convert("RGB"))
        rn, re = route(p["truth"])
        ax = row[0]
        ax.imshow(photo, extent=ext)
        ax.plot(re / 1000, rn / 1000, color="#ffd23f", lw=1.3)
        ax.plot(0, 0, marker="o", ms=7, mfc="none", mec="#ffd23f", mew=2)
        ax.set_title(f"{nm}: {s['class']}\n{s['place']}", loc="left", fontsize=9.5)
        ax.set_ylabel("north, km")
        ax = row[1]
        ls = LightSource(azdeg=315, altdeg=40)
        ax.imshow(ls.shade(alt.astype(float), cmap=plt.get_cmap("terrain"), vert_exag=1.0, dx=cell, dy=cell,
                           blend_mode="soft"), extent=ext)
        ax.set_title(f"world terrain ({'swissSURFACE3D, buildings and trees' if s['truth'] == 'surface' else 'swissALTI3D, bare earth'})\n"
                     f"{alt.min():.0f} to {alt.max():.0f} m, {cell:.1f} m grid", loc="left", fontsize=9.5)
        ax = row[2]
        ob, oh = load_map(p["onboard"])
        lat_c, lon_c = s["centre"]
        oh = dict(oh, lat_c=lat_c, lon_c=lon_c, m_per_deg_lon=st.M_PER_DEG_LAT * math.cos(math.radians(lat_c)))
        k = np.arange(0, n_, 2)
        north = (h["lat0"] + k * h["dlat"] - lat_c) * st.M_PER_DEG_LAT
        east = (h["lon0"] + k * h["dlon"] - lon_c) * h["m_per_deg_lon"]
        E, N = np.meshgrid(east, north)
        d = sample_map_frame(ob, oh, E, N) - alt[np.ix_(k, k)]
        im = ax.imshow(d, extent=ext, cmap="RdBu_r", vmin=-20, vmax=20)
        ax.set_title(f"GDN's map (GLO-30, 30 m) minus the world\nRMS {np.sqrt(np.mean(d ** 2)):.1f} m, "
                     f"mean {np.mean(d):+.1f} m", loc="left", fontsize=9.5)
        cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.03)
        cb.set_label("m (red: GDN's map higher)")
        for ax in row:
            ax.set_xlabel("east, km")
    fig.tight_layout()
    fig.savefig(a.out, dpi=80, facecolor=SURF)
    # A 256-colour palette keeps the five photos' figure under 1 MB (from 4 MB) for git.
    Image.open(a.out).convert("RGB").quantize(256, method=Image.Quantize.MEDIANCUT).save(a.out, optimize=True)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
