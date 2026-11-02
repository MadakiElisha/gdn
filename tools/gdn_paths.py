"""Shared path + terrain-map helpers for the Python tools.

GDN_ROOT is the repo root. It comes from the environment (set by env.sh)
and falls back to this file's parent directory, so tools work from any
clone location without editing.
"""
import math
import os
import struct
from pathlib import Path

ROOT = Path(os.environ.get("GDN_ROOT", Path(__file__).resolve().parents[1]))
DATA = ROOT / "data"
MAPS = DATA / "maps"
DEFAULT_MAP = MAPS / "terrain_db_sigma4.bin"


def load_map(path=None):
    """Read a GDN terrain DB (.bin). Returns (alt[ny,nx], header dict).

    Local NED origin = grid centre, identical to gdn::MapDb::Load().
    """
    import numpy as np  # local import: keep the module light for shell use
    p = Path(path) if path else DEFAULT_MAP
    b = p.read_bytes()
    nx, ny, lat0, lon0, dlat, dlon = struct.unpack("<iidddd", b[:40])
    expect = 40 + 4 * nx * ny
    if len(b) != expect:
        raise ValueError(f"{p}: {len(b)} bytes, header implies {expect}")
    alt = np.frombuffer(b[40:], dtype="<f4").reshape(ny, nx)
    lat_c = lat0 + 0.5 * ny * dlat
    lon_c = lon0 + 0.5 * nx * dlon
    hdr = dict(nx=nx, ny=ny, lat0=lat0, lon0=lon0, dlat=dlat, dlon=dlon,
               lat_c=lat_c, lon_c=lon_c,
               m_per_deg_lat=111320.0,
               m_per_deg_lon=111320.0 * math.cos(math.radians(lat_c)))
    return alt, hdr
