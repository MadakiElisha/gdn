#!/usr/bin/env python3
"""Regenerate terrain_db_sigma4.bin (REQ-DB-001) from terrain_db.bin (sigma1).
sigma_add = sqrt(4^2 - 1^2) so total smoothing == sigma4 cells."""
import numpy as np
from pathlib import Path
src = Path.home()/'gdn_workspace/data/maps/terrain_db.bin'
dst = Path.home()/'gdn_workspace/data/maps/terrain_db_sigma4.bin'
with open(src,'rb') as f:
    nx, ny = np.frombuffer(f.read(8), dtype=np.int32)
    lat0, lon0, dlat, dlon = np.frombuffer(f.read(32), dtype=np.float64)
    alt = np.frombuffer(f.read(), dtype=np.float32).reshape((ny,nx)).astype(np.float64)
sigma_add = np.sqrt(16.0 - 1.0)
r = int(np.ceil(3*sigma_add))
k = np.exp(-0.5*(np.arange(-r,r+1)/sigma_add)**2); k /= k.sum()
sm = np.apply_along_axis(lambda m: np.convolve(m, k, mode='same'), 0, alt)
sm = np.apply_along_axis(lambda m: np.convolve(m, k, mode='same'), 1, sm)
with open(dst,'wb') as f:
    f.write(np.array([nx,ny],dtype=np.int32).tobytes())
    f.write(np.array([lat0,lon0,dlat,dlon],dtype=np.float64).tobytes())
    f.write(sm.astype(np.float32).tobytes())
print(f"wrote {dst}")
