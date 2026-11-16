#!/usr/bin/env python3
"""Check an R2.2 simulation site (sim/sites.yaml) built by tools/make_site.py.

    python3 tools/check_site.py ch_zurich          # one site
    python3 tools/check_site.py --all              # every site in sim/sites.yaml

One PASS or FAIL line per check; exit 0 only if all pass. What it checks:
  - the WGS84 -> LV95 transform reproduces swisstopo's published example (1 m);
  - every downloaded tile matches its sha256 in sim/sites.lock;
  - the truth map (the world's terrain) is centred on the site, and matches swisstopo
    at 400 random grid points, recomputed here from the source tiles (5 cm);
  - the onboard map is GLO-30 itself (exact pixels), covers map_m, and differs from the
    truth like a real onboard map: not zero (the filter must not see the truth), RMS
    under 30 m, mean under 10 m; and it's registered: no shift of half or one GLO-30
    cell (15 or 30 m) in any direction fits more than 15% better. A registration slip
    fits about 2x better shifted (Zermatt: 11.2 m shifted against 5.8 m in place); the
    15% allows for GLO-30 seeing treetops and roofs that bare-earth truth doesn't.
    Skipped on flat sites, where a shift changes nothing;
  - the take-off pad is level (heights within 15 m of the centre span under 2 m);
  - the world was built from these maps (world_info.txt), with its files and a draped
    photo that isn't blank.
The physics and radar checks run separately on the world:
  python3 tools/check_sim_world.py --world-dir data/sim/<site>
  python3 tools/check_sim_radar.py --world-dir data/sim/<site>     (GPU)
"""
import argparse
import math
import sys

import numpy as np

import swiss_terrain as st
from gdn_paths import load_map


class Checks:
    def __init__(self):
        self.fails = 0

    def __call__(self, ok, what, detail=""):
        print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
        self.fails += 0 if ok else 1
        return ok


def block_mean(a, k, nodata=-9999.0):
    """k x k box average (the source resampled to about the world's cell), nodata-aware."""
    if k <= 1:
        return np.where(a <= nodata + 1, np.nan, a.astype(np.float64))
    h, w = a.shape[0] // k * k, a.shape[1] // k * k
    b = np.where(a[:h, :w] <= nodata + 1, np.nan, a[:h, :w].astype(np.float64))
    with np.errstate(all="ignore"):
        return np.nanmean(b.reshape(h // k, k, w // k, k), axis=(1, 3))


def truth_at(site, lock, lat, lon):
    """Heights at (lat, lon) recomputed from the source tiles, independently of make_site."""
    prod = site["truth"]
    gsd = st.PRODUCTS[prod][1]
    k = max(1, round(site["size_m"] / (site["grid"] - 1) / gsd))
    e, n = st.lv95_from_wgs84(lat, lon)
    out = np.full(np.shape(e), np.nan)
    tiles = {(t["e_km"], t["n_km"]): t for t in lock[site["name"]][prod]}
    cache = {}
    for idx in np.ndindex(np.shape(e)):
        key = (int(e[idx] // 1000), int(n[idx] // 1000))
        if key not in tiles:
            continue
        if key not in cache:
            a, x0, y0, dx, dy = st.read_geotiff(st.CACHE / prod / tiles[key]["file"])
            cache[key] = (block_mean(a, k), x0, y0, dx * k, dy * k)
        a, x0, y0, dx, dy = cache[key]
        fc = min(max((e[idx] - x0) / dx - 0.5, 0), a.shape[1] - 1.000001)
        fr = min(max((y0 - n[idx]) / dy - 0.5, 0), a.shape[0] - 1.000001)
        c0, r0 = int(fc), int(fr)
        tc, tr = fc - c0, fr - r0
        out[idx] = ((1 - tr) * ((1 - tc) * a[r0, c0] + tc * a[r0, c0 + 1]) +
                    tr * ((1 - tc) * a[r0 + 1, c0] + tc * a[r0 + 1, c0 + 1]))
    return out


def check_site(site, ck, rng):
    name = site["name"]
    paths = st.site_paths(name)
    lat_c, lon_c = site["centre"]
    print(f"== {name} ({site['class']}: {site['place']})")
    lock = st.read_lock()
    if not ck(name in lock, "tiles are pinned in sim/sites.lock", "" if name in lock else "run tools/make_site.py"):
        return
    bad, n_tiles = [], 0
    for prod, tiles in lock[name].items():
        for t in tiles:
            n_tiles += 1
            p = st.CACHE / prod / t["file"]
            if not p.exists() or st.sha_file(p) != t["sha256"]:
                bad.append(t["file"])
    ck(not bad, f"all {n_tiles} source tiles are downloaded and match their sha256",
       f"{len(bad)} missing or changed, e.g. {bad[:2]}" if bad else "")

    # Truth map.
    if not ck(paths["truth"].exists(), "truth map exists", str(paths["truth"])):
        return
    alt, hdr = load_map(paths["truth"])
    ck(abs(hdr["lat_c"] - lat_c) < 1e-9 and abs(hdr["lon_c"] - lon_c) < 1e-9 and hdr["nx"] == site["grid"],
       "truth map is centred on the site, at the site's grid",
       f"centre {hdr['lat_c']:.7f} {hdr['lon_c']:.7f}, {hdr['nx']}x{hdr['ny']}")
    ck(not np.isnan(alt).any(), "truth map has no holes", f"{int(np.isnan(alt).sum())} NaN cells")
    i = rng.integers(0, hdr["ny"], 400)
    j = rng.integers(0, hdr["nx"], 400)
    ref = truth_at(site, lock, hdr["lat0"] + i * hdr["dlat"], hdr["lon0"] + j * hdr["dlon"])
    err = np.abs(alt[i, j] - ref)
    ck(np.nanmax(err) < 0.05 and not np.isnan(ref).any(),
       f"truth map matches swisstopo {site['truth']} at 400 random points (5 cm)",
       f"max |diff| {np.nanmax(err):.3f} m")

    # Onboard map.
    if not ck(paths["onboard"].exists(), "onboard map exists", str(paths["onboard"])):
        return
    ob, ohdr = load_map(paths["onboard"])
    span_n = ohdr["ny"] * abs(ohdr["dlat"]) * st.M_PER_DEG_LAT
    span_e = ohdr["nx"] * abs(ohdr["dlon"]) * st.M_PER_DEG_LAT * math.cos(math.radians(lat_c))
    ck(abs(ohdr["dlat"]) == 1 / 3600 and abs(ohdr["dlon"]) == 1 / 3600 and min(span_n, span_e) >= site["map_m"],
       f"onboard map is on GLO-30's 1 arc-second grid and covers {site['map_m'] / 1000:.0f} km",
       f"{span_e / 1000:.1f} x {span_n / 1000:.1f} km")
    glo = {}
    ii, jj = rng.integers(0, ohdr["ny"], 200), rng.integers(0, ohdr["nx"], 200)
    lat, lon = ohdr["lat0"] + ii * ohdr["dlat"], ohdr["lon0"] + jj * ohdr["dlon"]
    worst = 0.0
    for t in lock[name]["glo30"]:
        a, x0, y0, dx, dy = st.read_geotiff(st.CACHE / "glo30" / t["file"])
        glo[t["file"]] = (a, x0, y0)
    for la, lo, v in zip(lat, lon, ob[ii, jj]):
        for a, x0, y0 in glo.values():
            c, r = (lo - x0) * 3600 - 0.5, (y0 - la) * 3600 - 0.5
            if 0 <= round(r) < a.shape[0] and 0 <= round(c) < a.shape[1] and abs(c - round(c)) < 1e-4:
                worst = max(worst, abs(float(a[round(r), round(c)]) - v))
                break
        else:
            worst = math.inf
    ck(worst < 1e-3, "onboard map holds GLO-30's own pixels (200 random points)", f"max |diff| {worst:.4f} m")

    # The onboard map in the world's frame (MapDb::SetOrigin to the site centre).
    oh = dict(ohdr, lat_c=lat_c, lon_c=lon_c, m_per_deg_lon=st.M_PER_DEG_LAT * math.cos(math.radians(lat_c)))
    sub = slice(0, None, 8)
    north = (hdr["lat0"] + np.arange(hdr["ny"])[sub] * hdr["dlat"] - lat_c) * st.M_PER_DEG_LAT
    east = (hdr["lon0"] + np.arange(hdr["nx"])[sub] * hdr["dlon"] - lon_c) * hdr["m_per_deg_lon"]
    E, N = np.meshgrid(east, north)
    tru = alt[sub, sub].astype(np.float64)

    def diff(dn=0.0, de=0.0):
        return sample_map_frame(ob, oh, E + de, N + dn) - tru

    d = diff()
    rms, mean = float(np.sqrt(np.mean(d ** 2))), float(np.mean(d))
    ck(0.5 < rms < 30.0 and abs(mean) < 10.0, "onboard map differs from the truth like a real one",
       f"RMS {rms:.1f} m, mean {mean:+.1f} m, 95th |diff| {np.percentile(np.abs(d), 95):.1f} m")
    relief = float(np.std(tru))
    if relief < 5.0:
        print(f"skip  registration: terrain too flat to tell (std {relief:.1f} m)")
    else:
        shifted = {(dn, de): float(np.sqrt(np.mean(diff(dn, de) ** 2)))
                   for dn in (-30, -15, 0, 15, 30) for de in (-30, -15, 0, 15, 30) if (dn, de) != (0, 0)}
        best = min(shifted, key=shifted.get)
        ck(shifted[best] > 0.85 * rms, "onboard map is registered to the world (no 15 or 30 m shift fits 15% better)",
           f"RMS {rms:.1f} m in place, best shift {best} m gives {shifted[best]:.1f} m")

    # Pad.
    r = np.hypot(*np.meshgrid((np.arange(hdr["nx"]) - hdr["nx"] / 2) * abs(hdr["dlon"]) * hdr["m_per_deg_lon"],
                              (np.arange(hdr["ny"]) - hdr["ny"] / 2) * abs(hdr["dlat"]) * st.M_PER_DEG_LAT))
    pad = alt[r <= 15.0]
    if not ck(pad.size > 0 and float(pad.max() - pad.min()) < 2.0, "take-off pad is level (within 15 m)",
              f"heights span {float(pad.max() - pad.min()):.2f} m over {pad.size} cells"):
        print("      " + level_spot_hint(alt, hdr, r))

    # World.
    w = paths["world"]
    info_p = w / "world_info.txt"
    if not ck(info_p.exists(), "world is built", str(w)):
        return
    info = dict(l.split("=", 1) for l in info_p.read_text().split("\n") if "=" in l)
    ck(info.get("map") == str(paths["truth"].resolve()) and info.get("onboard_map") == str(paths["onboard"].resolve())
       and abs(float(info.get("origin_lat", "nan")) - lat_c) < 1e-9
       and abs(float(info.get("origin_lon", "nan")) - lon_c) < 1e-9,
       "world was built from these maps, origin at the pad")
    files = [w / f"{name}.sdf", w / "heightmap.png", w / "terrain.obj", w / "terrain.mtl", w / "photo.png"]
    missing = [f.name for f in files if not f.exists()]
    ck(not missing, "world files exist (sdf, heightmap, mesh, material, photo)", ", ".join(missing))
    if (w / "photo.png").exists():
        from PIL import Image
        img = np.asarray(Image.open(w / "photo.png").convert("RGB"), dtype=np.float64)
        black = float(np.mean(img.sum(axis=2) < 3))
        ck(img.std() > 10 and black < 0.02, "the draped photo isn't blank",
           f"{img.shape[1]}x{img.shape[0]} px, std {img.std():.0f}, {100 * black:.1f}% black")


def level_spot_hint(alt, hdr, r):
    """The nearest place where heights within 15 m span under 1 m, to move the site to."""
    from scipy import ndimage
    cell = abs(hdr["dlat"]) * st.M_PER_DEG_LAT
    k = int(math.ceil(15.0 / cell))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    disc = np.hypot(yy, xx) * cell <= 15.0
    span = ndimage.maximum_filter(alt, footprint=disc) - ndimage.minimum_filter(alt, footprint=disc)
    ok = (span < 1.0) & (r < 1500.0)
    if not ok.any():
        return "no level spot within 1.5 km"
    i, j = np.unravel_index(np.argmin(np.where(ok, r, np.inf)), r.shape)
    return (f"nearest level spot: centre: [{hdr['lat0'] + i * hdr['dlat']:.5f}, {hdr['lon0'] + j * hdr['dlon']:.5f}] "
            f"({r[i, j]:.0f} m away, span {span[i, j]:.2f} m, {alt[i, j]:.0f} m)")


def sample_map_frame(alt, hdr, east, north):
    """MapDb::Query with an origin other than the grid centre (SetOrigin), bilinear."""
    fi = (hdr["lat_c"] + north / st.M_PER_DEG_LAT - hdr["lat0"]) / hdr["dlat"]
    fj = (hdr["lon_c"] + east / hdr["m_per_deg_lon"] - hdr["lon0"]) / hdr["dlon"]
    ny, nx = alt.shape
    fi = np.clip(fi, 0, ny - 1.000001)
    fj = np.clip(fj, 0, nx - 1.000001)
    i0, j0 = np.floor(fi).astype(int), np.floor(fj).astype(int)
    ti, tj = fi - i0, fj - j0
    a = alt.astype(np.float64)
    return ((1 - ti) * ((1 - tj) * a[i0, j0] + tj * a[i0, j0 + 1]) +
            ti * ((1 - tj) * a[i0 + 1, j0] + tj * a[i0 + 1, j0 + 1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", nargs="?")
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    ck = Checks()
    e, n = st.lv95_from_wgs84(46 + 2 / 60 + 38.87 / 3600, 8 + 43 / 60 + 49.79 / 3600)
    ck(math.hypot(e - 2699999.76, n - 1099999.97) < 1.0, "WGS84 -> LV95 reproduces swisstopo's example",
       f"E {e:.2f}, N {n:.2f}")
    sites = st.load_sites()
    names = list(sites) if a.all else [a.site]
    if not a.all and a.site not in sites:
        ck(False, f"site {a.site} is in sim/sites.yaml", f"sites: {', '.join(sites)}")
        names = []
    rng = np.random.default_rng(1)
    for nm in names:
        check_site(sites[nm], ck, rng)
    print(f"{'SITE CHECK PASSED' if ck.fails == 0 else f'SITE CHECK FAILED ({ck.fails})'}")
    return 0 if ck.fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
