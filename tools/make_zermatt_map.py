#!/usr/bin/env python3
import os, argparse, struct
import numpy as np
import rasterio

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tif", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--flat-crop", type=int, default=129, dest="flat_crop")
    ap.add_argument("--pad-r", type=int, default=4, dest="pad_r")
    ap.add_argument("--pad-fade", type=int, default=3, dest="pad_fade")
    a = ap.parse_args()
    
    with rasterio.open(a.tif) as src:
        dem = src.read(1); bounds = src.bounds; res = src.res
    dem = np.nan_to_num(dem, nan=-9999.0)
    mask = dem > -9990
    
    n = a.flat_crop
    filled = np.where(mask, dem, np.nanmedian(dem[mask]))
    gy, gx = np.gradient(filled)
    g = np.abs(gy) + np.abs(gx)
    cs = np.cumsum(np.cumsum(g, 0), 1)
    H, W = g.shape
    best = None
    for r in range(0, H-n+1, 4):
        for c in range(0, W-n+1, 4):
            s = cs[r+n-1, c+n-1]
            if r > 0: s -= cs[r-1, c+n-1]
            if c > 0: s -= cs[r+n-1, c-1]
            if r > 0 and c > 0: s += cs[r-1, c-1]
            v = s/(n*n)
            if best is None or v < best[0]: best = (v, r, c)
    _, r0, c0 = best
    crop = dem[r0:r0+n, c0:c0+n]
    
    west = bounds.left + c0*res[0]; north = bounds.top - r0*res[1]
    east = west + n*res[0]; south = north - n*res[1]
    clat = (north+south)/2; clon = (west+east)/2
    
    cy = cx = n//2
    yy, xx = np.mgrid[0:n, 0:n]
    d = np.sqrt((xx-cx)**2 + (yy-cy)**2)
    pad_amsl = float(crop[cy, cx])
    blend = np.clip((d - a.pad_r)/float(a.pad_fade), 0.0, 1.0)
    crop = blend*crop + (1.0-blend)*pad_amsl
    
    ny = n; nx = n
    lat0 = north; lon0 = west
    dlat = -res[1]; dlon = res[0]
    
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'wb') as f:
        f.write(struct.pack('<ii', nx, ny))
        f.write(struct.pack('<dddd', lat0, lon0, dlat, dlon))
        f.write(crop.astype(np.float32).tobytes())
    
    print(f"wrote {a.out}: {nx}x{ny} grid, origin ({lat0:.6f},{lon0:.6f}), step ({dlat:.8f},{dlon:.8f})")
    print(f"pad at ({clat:.6f},{clon:.6f}) elev {pad_amsl:.1f} m AMSL")

if __name__ == "__main__":
    main()
