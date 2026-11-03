#!/usr/bin/env python3
import os, argparse
import numpy as np
import rasterio
from PIL import Image

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tif", required=True)
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--name", default="zermatt")
    ap.add_argument("--flat-crop", type=int, default=129, dest="flat_crop")
    ap.add_argument("--pad-r", type=int, default=4, dest="pad_r")
    ap.add_argument("--pad-fade", type=int, default=3, dest="pad_fade")
    a = ap.parse_args()
    os.makedirs(a.out_dir, exist_ok=True)
    with rasterio.open(a.tif) as src:
        dem = src.read(1); bounds = src.bounds; res = src.res
    dem = np.nan_to_num(dem, nan=-9999.0)
    mask = dem > -9990
    n = a.flat_crop
    filled = np.where(mask, dem, np.nanmedian(dem[mask]))
    gy, gx = np.gradient(filled)
    g = np.abs(gy) + np.abs(gx)
    cs = np.cumsum(np.cumsum(g, axis=0), axis=1)
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
    dem = dem[r0:r0+n, c0:c0+n]
    west = bounds.left + c0*res[0]; north = bounds.top - r0*res[1]
    east = west + n*res[0]; south = north - n*res[1]
    clat = (north+south)/2; clon = (west+east)/2
    print(f"flat-crop {n}x{n} row {r0} col {c0}; center {clat:.6f},{clon:.6f}")
    cy = cx = n//2
    yy, xx = np.mgrid[0:n, 0:n]
    d = np.sqrt((xx-cx)**2 + (yy-cy)**2)
    pad_elev = float(dem[cy, cx])
    blend = np.clip((d - a.pad_r)/float(a.pad_fade), 0.0, 1.0)
    dem = blend*dem + (1.0-blend)*pad_elev
    print(f"pad stamped at {pad_elev:.1f} m AMSL")
    dx = res[0]*111320*np.cos(np.radians(clat)); dy = res[1]*111320
    xm = (np.arange(n) - cx)*dx
    ym = (cy - np.arange(n))*dy
    z = dem - pad_elev
    gz_i, gz_j = np.gradient(z)
    nx = -gz_j/dx; ny = gz_i/dy; nz = np.ones_like(z)
    ln = np.sqrt(nx*nx + ny*ny + 1.0)
    nx, ny, nz = nx/ln, ny/ln, nz/ln
    obj = os.path.abspath(os.path.join(a.out_dir, "terrain.obj"))
    with open(obj, "w") as f:
        for i in range(n):
            for j in range(n):
                f.write(f"v {xm[j]:.2f} {z[i, j]:.2f} {-ym[i]:.2f}\n")
        for i in range(n):
            for j in range(n):
                f.write(f"vn {nx[i, j]:.4f} {nz[i, j]:.4f} {-ny[i, j]:.4f}\n")
        for i in range(n-1):
            for j in range(n-1):
                aa = i*n + j + 1; bb = aa + 1; cc = aa + n; dd = cc + 1
                f.write(f"f {aa}//{aa} {bb}//{bb} {dd}//{dd}\n")
                f.write(f"f {aa}//{aa} {dd}//{dd} {cc}//{cc}\n")
    print(f"wrote {obj} ({n*n} verts+normals)")
    mnz = float(z.min()); mxz = float(z.max()); h16 = max(mxz-mnz, 1.0)
    arr16 = np.clip((z-mnz)/h16*65535, 0, 65535).astype(np.uint16)
    png = os.path.abspath(os.path.join(a.out_dir, "heightmap16.png"))
    Image.fromarray(arr16).save(png)
    wm = xm[-1]-xm[0]; dm = ym[0]-ym[-1]
    with open(f"/tmp/{a.name}_sz.txt", "w") as f: f.write("0.5")
    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{a.name}">
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>{clat:.9f}</latitude_deg>
      <longitude_deg>{clon:.9f}</longitude_deg>
      <elevation>{pad_elev:.2f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{png}</uri>
              <size>{wm:.2f} {dm:.2f} {h16:.2f}</size>
              <pos>0 0 {h16/2 + mnz:.2f}</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>file://{obj}</uri></mesh></geometry>
          <material>
            <ambient>0.6 0.55 0.45 1</ambient>
            <diffuse>0.6 0.55 0.45 1</diffuse>
            <double_sided>true</double_sided>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""
    sdf_path = os.path.abspath(os.path.join(a.out_dir, a.name + ".sdf"))
    with open(sdf_path, "w") as f: f.write(sdf)
    print(f"saved world: {sdf_path} (collision=heightmap, visual=mesh)")
    print("SPAWN_Z=0.5")

if __name__ == "__main__":
    main()
