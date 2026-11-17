#!/usr/bin/env python3
"""Build an R2.2 simulation site from real terrain (sites in sim/sites.yaml).

    python3 tools/make_site.py ch_zurich          # fetch (cached), build maps and world
    python3 tools/make_site.py --all
    python3 tools/make_site.py ch_zurich --fetch-only
    python3 tools/make_site.py ch_zurich --repin   # re-pick the newest tiles, rewrite sites.lock
    python3 tools/make_site.py ch_zurich --if-stale  # only if its sites.yaml entry or this code changed

For each site:
  1. Fetch and pin the source tiles (tools/swiss_terrain.py; cache data/terrain/, pins
     in sim/sites.lock).
  2. Truth map, data/maps/<site>_truth.bin: the world's terrain on a square grid
     (size_m, grid points per side) centred on the pad, from swissALTI3D (or
     swissSURFACE3D) box-averaged to about the grid cell, then sampled bilinearly. The
     world, the radar check and the scorer's terrain all come from this file.
  3. Onboard map, data/maps/<site>_glo30.bin: Copernicus GLO-30's own 1 arc-second
     pixels over map_m, untouched. GDN flies on this; eskf_node gets the pad as its
     origin (origin_lat/lon), so both maps share one local frame.
  4. Photo, data/sim/<site>/photo.png: SWISSIMAGE 2 m over the world, for the mesh.
  5. The world: tools/make_sim_world.py with the truth map, the photo and the origin.

Check the result with tools/check_site.py.
"""
import argparse
import json
import math
import sys

import numpy as np

import swiss_terrain as st
from gdn_paths import ROOT


def lv95_box(lat_c, lon_c, half_m, margin=60.0):
    """LV95 bounds of the square of half-side half_m around the centre (plus margin)."""
    s = np.linspace(-half_m, half_m, 41)
    n = np.r_[s, s, np.full(41, half_m), np.full(41, -half_m)]
    e = np.r_[np.full(41, half_m), np.full(41, -half_m), s, s]
    lat, lon = st.latlon_of_local(lat_c, lon_c, n, e)
    E, N = st.lv95_from_wgs84(lat, lon)
    return E.min() - margin, N.min() - margin, E.max() + margin, N.max() + margin


def fetch(site, repin=False):
    name = site["name"]
    lat_c, lon_c = site["centre"]
    lock = st.read_lock()
    pins = {} if repin else dict(lock.get(name, {}))
    half = site["size_m"] / 2.0
    box = lv95_box(lat_c, lon_c, half)
    lat_lo, lon_lo = st.latlon_of_local(lat_c, lon_c, -half - 200, -half - 200)
    lat_hi, lon_hi = st.latlon_of_local(lat_c, lon_c, half + 200, half + 200)
    for prod in (site["truth"], "photo"):
        need = {(e, n) for e in range(int(box[0] // 1000), int(box[2] // 1000) + 1)
                for n in range(int(box[1] // 1000), int(box[3] // 1000) + 1)}
        have = {(t["e_km"], t["n_km"]): t for t in pins.get(prod, [])}
        if not need <= set(have):              # new site, or it moved: pin the tiles it lacks
            tiles = st.stac_tiles(prod, *box, (lon_lo, lat_lo, lon_hi, lat_hi))
            missing = sorted(need - set(tiles) - set(have))
            if missing:
                sys.exit(f"{name}: swisstopo has no {prod} tile for {missing} (outside Switzerland?); "
                         "move the site in sim/sites.yaml")
            for k in need - set(have):
                v = tiles[k]
                have[k] = dict(e_km=k[0], n_km=k[1], item=v[0], url=v[1], file=v[1].rsplit("/", 1)[1], sha256=None)
        pins[prod] = [have[k] for k in sorted(need)]
        print(f"{name}: {len(pins[prod])} {prod} tiles")
        for k, t in enumerate(pins[prod]):
            t["sha256"] = st.download(t["url"], st.CACHE / prod / t["file"], t["sha256"])
            print(f"  {k + 1}/{len(pins[prod])} {t['file']}", flush=True)
    h = site["map_m"] / 2.0 + 500.0
    la0, lo0 = st.latlon_of_local(lat_c, lon_c, -h, -h)
    la1, lo1 = st.latlon_of_local(lat_c, lon_c, h, h)
    have = {(t["lat"], t["lon"]): t for t in pins.get("glo30", [])}
    for la in range(math.floor(la0), math.floor(la1) + 1):
        for lo in range(math.floor(lo0), math.floor(lo1) + 1):
            if (la, lo) not in have:
                n, url = st.glo30_name(la, lo)
                have[(la, lo)] = dict(lat=la, lon=lo, item=n, url=url, file=n + ".tif", sha256=None)
    pins["glo30"] = [have[k] for k in sorted(have)
                     if math.floor(la0) <= k[0] <= math.floor(la1) and math.floor(lo0) <= k[1] <= math.floor(lo1)]
    print(f"{name}: {len(pins['glo30'])} GLO-30 tiles (42 MB each)")
    for t in pins["glo30"]:
        t["sha256"] = st.download(t["url"], st.CACHE / "glo30" / t["file"], t["sha256"])
        print(f"  {t['file']}", flush=True)
    lock = st.read_lock()                      # another site may have been pinned meanwhile
    lock[name] = pins
    st.write_lock(lock)
    return pins


def block_tiles(paths, k):
    """Source tiles box-averaged by k (nodata-aware), as (array, x0, y0, dx, dy)."""
    out = []
    for p in paths:
        a, x0, y0, dx, dy = st.read_geotiff(p)
        a = np.where(a <= -9998, np.nan, a.astype(np.float64))
        if k > 1:
            h, w = a.shape[0] // k * k, a.shape[1] // k * k
            with np.errstate(all="ignore"):
                a = np.nanmean(a[:h, :w].reshape(h // k, k, w // k, k), axis=(1, 3))
        out.append((a, x0, y0, dx * k, dy * k))
    return out


def build_truth(site, pins):
    lat_c, lon_c = site["centre"]
    n = site["grid"]
    lat0, lon0, dlat, dlon = st.grid_for(lat_c, lon_c, site["size_m"], n)
    prod = site["truth"]
    k = max(1, round(site["size_m"] / (n - 1) / st.PRODUCTS[prod][1]))
    mos = st.Mosaic([])
    mos.tiles = block_tiles([st.CACHE / prod / t["file"] for t in pins[prod]], k)
    lat = lat0 + np.arange(n) * dlat
    lon = lon0 + np.arange(n) * dlon
    LON, LAT = np.meshgrid(lon, lat)
    E, N = st.lv95_from_wgs84(LAT, LON)
    alt = mos.sample(E, N)
    holes = int(np.isnan(alt).sum())
    if holes:
        print(f"warning: {holes} truth cells without data (filled from neighbours)")
        alt = fill_nan(alt)
    path = st.site_paths(site["name"])["truth"]
    st.write_map(path, alt.astype(np.float32), lat0, lon0, dlat, dlon)
    print(f"truth map {path.name}: {n}x{n}, cell {site['size_m'] / (n - 1):.2f} m, from {prod} "
          f"(box {k}x{k}), {np.nanmin(alt):.0f} to {np.nanmax(alt):.0f} m")
    return path


def fill_nan(a):
    from scipy import ndimage
    idx = ndimage.distance_transform_edt(np.isnan(a), return_distances=False, return_indices=True)
    return a[tuple(idx)]


def build_onboard(site, pins):
    """GLO-30's own pixels over map_m around the pad: rows and columns on the exact
    arc-second grid (pixel centres at whole seconds, PixelIsPoint)."""
    lat_c, lon_c = site["centre"]
    h = site["map_m"] / 2.0
    s_lat0 = math.ceil((lat_c + h / st.M_PER_DEG_LAT) * 3600)         # north edge, seconds
    s_lat1 = math.floor((lat_c - h / st.M_PER_DEG_LAT) * 3600)
    m_lon = st.M_PER_DEG_LAT * math.cos(math.radians(lat_c))
    s_lon0 = math.floor((lon_c - h / m_lon) * 3600)
    s_lon1 = math.ceil((lon_c + h / m_lon) * 3600)
    ny, nx = s_lat0 - s_lat1 + 1, s_lon1 - s_lon0 + 1
    out = np.full((ny, nx), np.nan)
    for t in pins["glo30"]:
        a, x0, y0, dx, dy = st.read_geotiff(st.CACHE / "glo30" / t["file"])
        top = round((y0 - dy / 2) * 3600)          # seconds of row 0's centre
        left = round((x0 + dx / 2) * 3600)
        rows = np.arange(ny)
        cols = np.arange(nx)
        r = top - (s_lat0 - rows)                  # source row of each output row
        c = (s_lon0 + cols) - left
        rok = (r >= 0) & (r < a.shape[0])
        cok = (c >= 0) & (c < a.shape[1])
        out[np.ix_(rows[rok], cols[cok])] = a[np.ix_(r[rok], c[cok])]
    if np.isnan(out).any():
        sys.exit(f"onboard map: {int(np.isnan(out).sum())} cells not covered by the GLO-30 tiles")
    path = st.site_paths(site["name"])["onboard"]
    st.write_map(path, out.astype(np.float32), s_lat0 / 3600.0, s_lon0 / 3600.0, -1.0 / 3600.0, 1.0 / 3600.0)
    print(f"onboard map {path.name}: GLO-30 {nx}x{ny} ({nx * m_lon / 3600 / 1000:.1f} x "
          f"{ny * st.M_PER_DEG_LAT / 3600 / 1000:.1f} km)")
    return path


def build_photo(site, pins, px=2048):
    """The orthophoto over the world square, as the mesh's texture: pixel (r, c) at the
    same east/north extent the mesh spans (make_sim_world.heightmap_grid with
    oversample 1 = the truth grid's first to last sample)."""
    from PIL import Image
    lat_c, lon_c = site["centre"]
    n = site["grid"]
    cell = site["size_m"] / (n - 1)
    n_max, e_min = (n / 2.0) * cell, -(n / 2.0) * cell
    span = (n - 1) * cell
    u = (np.arange(px) + 0.5) / px
    east = e_min + u * span
    north = n_max - u * span
    E_, N_ = np.meshgrid(east, north)
    lat, lon = st.latlon_of_local(lat_c, lon_c, N_, E_)
    E, N = st.lv95_from_wgs84(lat, lon)
    mos = st.Mosaic([st.CACHE / "photo" / t["file"] for t in pins["photo"]])
    rgb = mos.sample(E, N)
    rgb = np.nan_to_num(rgb, nan=0.0)
    path = st.site_paths(site["name"])["world"] / "photo.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.clip(np.round(rgb), 0, 255).astype(np.uint8)).save(path)
    print(f"photo {path.name}: {px}x{px} px, {span / px:.2f} m per pixel")
    return path


CODE = [ROOT / "tools" / f for f in ("make_site.py", "make_sim_world.py", "swiss_terrain.py")]


BUILD_KEYS = ("centre", "size_m", "grid", "map_m", "truth")    # what shapes the world (not its name)


def site_def(site):
    return json.dumps({k: site[k] for k in BUILD_KEYS}, sort_keys=True, separators=(",", ":"))


def is_current(site):
    """The world was built from this exact sites.yaml entry, by the current code."""
    w = st.site_paths(site["name"])["world"]
    info, sdf = w / "world_info.txt", w / f"{site['name']}.sdf"
    if not info.exists() or not sdf.exists():
        return False
    built = dict(l.split("=", 1) for l in info.read_text().split("\n") if "=" in l).get("site_def")
    return built == site_def(site) and all(sdf.stat().st_mtime > c.stat().st_mtime for c in CODE)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("site", nargs="?")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--fetch-only", action="store_true", dest="fetch_only")
    ap.add_argument("--repin", action="store_true", help="re-pick the newest tiles (rewrites sim/sites.lock)")
    ap.add_argument("--if-stale", action="store_true", dest="if_stale",
                    help="skip a site whose world is built from its current sites.yaml entry by the current code")
    a = ap.parse_args()
    sites = st.load_sites()
    names = list(sites) if a.all else [a.site]
    for nm in names:
        if nm not in sites:
            sys.exit(f"no site {nm} in sim/sites.yaml ({', '.join(sites)})")
    built = 0
    for nm in names:
        site = sites[nm]
        if a.if_stale and not a.repin and is_current(site):
            print(f"{nm}: world is current")
            continue
        print(f"== {nm} ({site['class']}: {site['place']})")
        built += 1
        pins = fetch(site, a.repin)
        if a.fetch_only:
            continue
        truth = build_truth(site, pins)
        onboard = build_onboard(site, pins)
        photo = build_photo(site, pins)
        import make_sim_world
        lat_c, lon_c = site["centre"]
        make_sim_world.main(["--map", str(truth), "--name", nm, "--oversample", "1", "--texture", str(photo),
                             "--onboard-map", str(onboard), "--origin", f"{lat_c}", f"{lon_c}"])
        info = st.site_paths(nm)["world"] / "world_info.txt"
        info.write_text(info.read_text() + f"site_def={site_def(site)}\n")
        (st.site_paths(nm)["world"] / f"{nm}.sdf").touch()
    if built:
        print(f"done; check with: python3 tools/check_site.py {'--all' if a.all else a.site}")


if __name__ == "__main__":
    main()
