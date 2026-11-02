from gdn_paths import ROOT  # repo root (GDN_ROOT or this clone)
import h5py, numpy as np, struct
from scipy.ndimage import gaussian_filter
from pathlib import Path
src = ROOT/'data/maps/terrain.h5'
dst = ROOT/'data/maps/terrain_db.bin'
with h5py.File(src,'r') as f:
    lats, lons, alt = f['lats'][:], f['lons'][:], f['alt'][:]
alt_sm = gaussian_filter(alt, 1.0)   # sigma = 1 cell ~ 27 m: TEST-008 sweet spot
hdr = struct.pack('<iidddd', alt_sm.shape[1], alt_sm.shape[0],
                  lats[0], lons[0], lats[1]-lats[0], lons[1]-lons[0])
dst.write_bytes(hdr + alt_sm.astype('<f4').tobytes())
print('wrote', dst, dst.stat().st_size, 'bytes')
