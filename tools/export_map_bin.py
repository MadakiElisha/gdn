import h5py, numpy as np, struct
from pathlib import Path
src = Path.home()/'gdn_workspace/data/maps/terrain.h5'
dst = Path.home()/'gdn_workspace/data/maps/terrain.bin'
with h5py.File(src,'r') as f:
    lats, lons, alt = f['lats'][:], f['lons'][:], f['alt'][:]
hdr = struct.pack('<iidddd', alt.shape[1], alt.shape[0],
                  lats[0], lons[0], lats[1]-lats[0], lons[1]-lons[0])
dst.write_bytes(hdr + alt.astype('<f4').tobytes())   # row-major alt[ilat, ilon]
print(f"wrote {dst} ({dst.stat().st_size} bytes), grid {alt.shape}")
