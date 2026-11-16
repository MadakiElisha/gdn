"""Real terrain for the R2.2 simulation sites (sim/sites.yaml): swisstopo for the world,
Copernicus GLO-30 for GDN's onboard map. Shared by tools/make_site.py and
tools/check_site.py. Needs only numpy, PIL and PyYAML (all in the dev image).

Sources (open data, no account):
  swissALTI3D      2 m bare-earth terrain, LV95 (EPSG:2056), heights LN02, 1 km tiles
  swissSURFACE3D   0.5 m surface model (buildings, trees), same tiling
  SWISSIMAGE       2 m orthophoto (RGB JPEG-in-TIFF), same tiling
  Copernicus GLO-30  1 arc-second DSM, WGS84, heights EGM2008, 1 degree tiles (AWS)

Downloads go to data/terrain/ (not in git). The first fetch of a site pins every
tile (STAC item, file, sha256) in sim/sites.lock (in git), and later fetches get
exactly those files and check them, so a world is rebuilt from the same data even
after swisstopo publishes a newer survey.
"""
import hashlib
import json
import math
import time
import urllib.request
from pathlib import Path

import numpy as np

from gdn_paths import ROOT

SITES_FILE = ROOT / "sim/sites.yaml"
LOCK_FILE = ROOT / "sim/sites.lock"
CACHE = ROOT / "data/terrain"
STAC = "https://data.geo.admin.ch/api/stac/v0.9/collections"
GLO30 = "https://copernicus-dem-30m.s3.amazonaws.com"
M_PER_DEG_LAT = 111320.0          # gdn::MapDb's local frame (equirectangular)

# product key -> (STAC collection, asset ground sample distance in m, asset kind)
PRODUCTS = {
    "terrain": ("ch.swisstopo.swissalti3d", 2.0, "tif"),
    "surface": ("ch.swisstopo.swisssurface3d-raster", 0.5, "tif"),
    "photo": ("ch.swisstopo.swissimage-dop10", 2.0, "tif"),
}


# ------------------------------------------------------------------------- sites ----
def load_sites():
    import yaml
    d = yaml.safe_load(SITES_FILE.read_text())
    out = {}
    for name, s in d["sites"].items():
        site = dict(d.get("defaults", {}))
        site.update(s)
        site["name"] = name
        out[name] = site
    return out


def site_paths(name):
    return dict(truth=ROOT / f"data/maps/{name}_truth.bin", onboard=ROOT / f"data/maps/{name}_glo30.bin",
                world=ROOT / f"data/sim/{name}")


# ------------------------------------------------------------------ coordinates ----
def lv95_from_wgs84(lat, lon):
    """WGS84 degrees -> Swiss LV95 (E, N) metres, swisstopo's approximate formulas
    ("Approximate formulas for the transformation between Swiss projection coordinates
    and WGS84"), good to about 1 m. Works on numpy arrays."""
    p = (np.asarray(lat, dtype=np.float64) * 3600.0 - 169028.66) / 10000.0
    l = (np.asarray(lon, dtype=np.float64) * 3600.0 - 26782.5) / 10000.0
    e = 2600072.37 + 211455.93 * l - 10938.51 * l * p - 0.36 * l * p ** 2 - 44.54 * l ** 3
    n = 1200147.07 + 308807.95 * p + 3745.25 * l ** 2 + 76.63 * p ** 2 - 194.56 * l ** 2 * p + 119.79 * p ** 3
    return e, n


def latlon_of_local(lat_c, lon_c, north, east):
    """GDN's local frame (MapDb): north/east metres around (lat_c, lon_c), equirectangular."""
    return (lat_c + np.asarray(north) / M_PER_DEG_LAT,
            lon_c + np.asarray(east) / (M_PER_DEG_LAT * math.cos(math.radians(lat_c))))


# --------------------------------------------------------------------- download ----
def _get(url, timeout=60):
    req = urllib.request.Request(url, headers={"User-Agent": "gdn-terrain-fetch"})
    return urllib.request.urlopen(req, timeout=timeout)


def download(url, dest, sha256=None, tries=6):
    """Fetch url to dest (cached), retrying; check sha256 if given. Returns sha256."""
    dest = Path(dest)
    if dest.exists():
        h = sha_file(dest)
        if sha256 is None or h == sha256:
            return h
        dest.unlink()
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    for k in range(tries):
        try:
            with _get(url, timeout=120) as r, open(tmp, "wb") as f:
                while True:
                    b = r.read(1 << 20)
                    if not b:
                        break
                    f.write(b)
            break
        except Exception as e:                           # slow or cut connections: retry
            if k == tries - 1:
                raise RuntimeError(f"download failed after {tries} tries: {url}: {e}")
            time.sleep(5 * (k + 1))
    h = sha_file(tmp)
    if sha256 is not None and h != sha256:
        tmp.unlink()
        raise RuntimeError(f"{url}: sha256 {h}, sites.lock says {sha256}")
    tmp.rename(dest)
    return h


def sha_file(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def read_lock():
    return json.loads(LOCK_FILE.read_text()) if LOCK_FILE.exists() else {}


def write_lock(lock):
    LOCK_FILE.write_text(json.dumps(lock, indent=1, sort_keys=True) + "\n")


def stac_tiles(product, e_min, n_min, e_max, n_max, bbox_wgs84):
    """Newest STAC item per 1 km tile covering [e_min, e_max] x [n_min, n_max] (LV95).
    Returns {(e_km, n_km): (item_id, asset_url)}."""
    coll, gsd, kind = PRODUCTS[product]
    url = f"{STAC}/{coll}/items?bbox={','.join(f'{v:.6f}' for v in bbox_wgs84)}&limit=100"
    best = {}
    while url:
        d = json.load(_get(url))
        for f in d["features"]:
            parts = f["id"].split("_")              # e.g. swissalti3d_2019_2623-1093
            year = int(parts[1])
            ek, nk = (int(v) for v in parts[2].split("-"))
            if not (e_min - 1000 < ek * 1000 <= e_max and n_min - 1000 < nk * 1000 <= n_max):
                continue
            asset = next((a["href"] for a in f["assets"].values()
                          if abs(float(a.get("eo:gsd", -1)) - gsd) < 1e-6 and a["href"].endswith(kind)), None)
            if asset and ((ek, nk) not in best or year > best[(ek, nk)][0]):
                best[(ek, nk)] = (year, f["id"], asset)
        url = next((l["href"] for l in d.get("links", []) if l["rel"] == "next"), None)
    return {k: (v[1], v[2]) for k, v in best.items()}


def glo30_name(lat_floor, lon_floor):
    ns = f"N{lat_floor:02d}" if lat_floor >= 0 else f"S{-lat_floor:02d}"
    ew = f"E{lon_floor:03d}" if lon_floor >= 0 else f"W{-lon_floor:03d}"
    n = f"Copernicus_DSM_COG_10_{ns}_00_{ew}_00_DEM"
    return n, f"{GLO30}/{n}/{n}.tif"


# ------------------------------------------------------------------------ rasters ----
def read_geotiff(path):
    """(array, x0, y0, dx, dy) for a north-up GeoTIFF: pixel (r, c) centre is at
    x = x0 + (c + 0.5) dx, y = y0 - (r + 0.5) dy.

    The tie point is a pixel corner for PixelIsArea rasters (swisstopo) and a pixel
    centre for PixelIsPoint ones (Copernicus GLO-30): GTRasterTypeGeoKey decides, and
    getting it wrong shifts a GLO-30 map by half a cell (15 m)."""
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    im = Image.open(path)
    scale = im.tag_v2.get(33550)
    tie = im.tag_v2.get(33922)
    k = im.tag_v2.get(34735) or ()
    keys = {k[i]: k[i + 3] for i in range(4, len(k), 4)}
    half = 0.5 if keys.get(1025) == 2 else 0.0            # PixelIsPoint
    a = np.array(im)
    return (a, tie[3] - (tie[0] + half) * scale[0], tie[4] + (tie[1] + half) * scale[1], scale[0], scale[1])


class Mosaic:
    """Bilinear sampling across a set of same-resolution north-up tiles."""

    def __init__(self, paths, nodata=-9999.0):
        self.tiles = [read_geotiff(p) for p in paths]
        self.nodata = nodata

    def sample(self, x, y):
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        shape = x.shape
        x, y = x.ravel(), y.ravel()
        out = None
        for a, x0, y0, dx, dy in self.tiles:
            fc = (x - x0) / dx - 0.5
            fr = (y0 - y) / dy - 0.5
            h, w = a.shape[:2]
            # A point sits in the tile whose pixel area holds it; the bilinear 2x2 then
            # needs the neighbour pixel, clamped at the tile edge (a half-pixel strip,
            # 1 m at 2 m tiles, where the edge value is held).
            inside = (x >= x0) & (x < x0 + w * dx) & (y <= y0) & (y > y0 - h * dy)
            if not inside.any():
                continue
            fc_i, fr_i = np.clip(fc[inside], 0, w - 1.000001), np.clip(fr[inside], 0, h - 1.000001)
            c0, r0 = np.floor(fc_i).astype(int), np.floor(fr_i).astype(int)
            tc, tr = fc_i - c0, fr_i - r0
            if a.ndim == 3:
                tc, tr = tc[:, None], tr[:, None]
            v = ((1 - tr) * ((1 - tc) * a[r0, c0] + tc * a[r0, c0 + 1]) +
                 tr * ((1 - tc) * a[r0 + 1, c0] + tc * a[r0 + 1, c0 + 1]))
            if out is None:
                out = np.full((x.size,) + a.shape[2:], np.nan)
            out[inside] = v
        if out is None:
            out = np.full(x.size, np.nan)
        if out.ndim == 1:
            out[out <= self.nodata + 1] = np.nan
        return out.reshape(shape + out.shape[1:])


# ------------------------------------------------------------------------ GDN map ----
def write_map(path, alt, lat0, lon0, dlat, dlon):
    """GDN terrain DB (gdn::MapDb): <iidddd header, then row-major float32 heights."""
    import struct
    ny, nx = alt.shape
    with open(path, "wb") as f:
        f.write(struct.pack("<iidddd", nx, ny, lat0, lon0, dlat, dlon))
        f.write(np.ascontiguousarray(alt, dtype="<f4").tobytes())


def grid_for(lat_c, lon_c, size_m, n):
    """lat0, lon0, dlat, dlon of an n x n MapDb grid of side size_m whose grid centre
    (MapDb: lat0 + n/2 dlat) is exactly (lat_c, lon_c)."""
    cell = size_m / (n - 1)
    dlat = -cell / M_PER_DEG_LAT
    dlon = cell / (M_PER_DEG_LAT * math.cos(math.radians(lat_c)))
    return lat_c - 0.5 * n * dlat, lon_c - 0.5 * n * dlon, dlat, dlon
