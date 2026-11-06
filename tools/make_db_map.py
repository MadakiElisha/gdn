#!/usr/bin/env python3
"""Build an onboard (database) terrain map that differs from the truth world map (R1.3).

A real onboard map is never the ground the aircraft flies over: it comes from a
satellite DEM with its own errors and resolution (REQ-DB-002). This tool makes such a
map from a truth map, so the Monte Carlo and bench can fly over one map and navigate
with another. Errors, applied in this order:

  --coarsen N   resample to N times the cell size (box average over N x N cells, then
                sampled at the coarse grid points). Same geographic centre, so both maps
                stay registered; MapDb::SetOrigin puts them in one frame anyway.
  --noise S     spatially correlated height error, S metres (1 sigma), with
                correlation length --corr metres (default 150). DEM errors come in
                patches (vegetation, radar shadow, interpolation), not per cell.
  --bias B      constant height offset, metres (datum or geoid error).
  --smooth S    Gaussian smoothing, S cells of the output grid (scalar TRN maps are
                smoothed, REQ-DB-001).

    python3 tools/make_db_map.py --in data/maps/zermatt.bin --out data/maps/zermatt_db_srtm.bin \\
        --coarsen 3 --noise 6 --corr 200 --bias 3 --seed 1

Prints the RMS and max height difference against the truth map at the truth grid
points, which is what the filter's r_agl / r_trn has to cover.
"""
import argparse
import struct
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

from gdn_paths import load_map


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--in", dest="src", required=True, help="truth map (.bin)")
    ap.add_argument("--out", required=True, help="onboard map to write (.bin)")
    ap.add_argument("--coarsen", type=int, default=1, help="cell-size factor (integer, default 1)")
    ap.add_argument("--noise", type=float, default=0.0, help="correlated height error, m (1 sigma)")
    ap.add_argument("--corr", type=float, default=150.0, help="noise correlation length, m")
    ap.add_argument("--bias", type=float, default=0.0, help="constant height offset, m")
    ap.add_argument("--smooth", type=float, default=0.0, help="Gaussian smoothing, output cells")
    ap.add_argument("--seed", type=int, default=1, help="noise seed (maps are reproducible)")
    a = ap.parse_args()
    if a.coarsen < 1:
        sys.exit("--coarsen must be 1 or more")

    alt, h = load_map(a.src)
    alt = alt.astype(np.float64)
    ny, nx = alt.shape
    cell_n = abs(h["dlat"]) * h["m_per_deg_lat"]
    cell_e = abs(h["dlon"]) * h["m_per_deg_lon"]

    # Resample about the same geographic centre.
    N = a.coarsen
    ny2, nx2 = max(4, ny // N), max(4, nx // N)
    dlat2, dlon2 = h["dlat"] * N, h["dlon"] * N
    lat02 = h["lat_c"] - 0.5 * ny2 * dlat2
    lon02 = h["lon_c"] - 0.5 * nx2 * dlon2
    if N > 1:
        box = ndimage.uniform_filter(alt, size=N, mode="nearest")
        fi = (lat02 + np.arange(ny2) * dlat2 - h["lat0"]) / h["dlat"]
        fj = (lon02 + np.arange(nx2) * dlon2 - h["lon0"]) / h["dlon"]
        gi, gj = np.meshgrid(fi, fj, indexing="ij")
        db = ndimage.map_coordinates(box, [gi, gj], order=1, mode="nearest")
    else:
        db = alt.copy()

    if a.noise > 0:
        rng = np.random.default_rng(a.seed)
        white = rng.standard_normal(db.shape)
        sig = (a.corr / (cell_n * N), a.corr / (cell_e * N)) if a.corr > 0 else (0, 0)
        field = ndimage.gaussian_filter(white, sigma=sig, mode="wrap") if a.corr > 0 else white
        db += a.noise * field / field.std()
    db += a.bias
    if a.smooth > 0:
        db = ndimage.gaussian_filter(db, sigma=a.smooth, mode="nearest")

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "wb") as f:
        f.write(struct.pack("<iidddd", nx2, ny2, lat02, lon02, dlat2, dlon2))
        f.write(db.astype("<f4").tobytes())

    # Difference against the truth at the truth grid points (inner region, away from edges).
    ti = (h["lat0"] + np.arange(ny) * h["dlat"] - lat02) / dlat2
    tj = (h["lon0"] + np.arange(nx) * h["dlon"] - lon02) / dlon2
    gi, gj = np.meshgrid(ti, tj, indexing="ij")
    back = ndimage.map_coordinates(db, [gi, gj], order=1, mode="nearest")
    m = max(2, 2 * N)
    d = (back - alt)[m:-m, m:-m]
    print(f"wrote {out}: {nx2}x{ny2}, cell {cell_n * N:.1f} x {cell_e * N:.1f} m, "
          f"vs truth: mean {d.mean():+.1f} m, RMS {np.sqrt((d ** 2).mean()):.1f} m, "
          f"max {np.abs(d).max():.1f} m")


if __name__ == "__main__":
    main()
