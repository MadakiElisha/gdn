import numpy as np, h5py, matplotlib.pyplot as plt
from scipy.ndimage import zoom
from scipy.signal import fftconvolve
from pathlib import Path

SEED = 42
rng = np.random.default_rng(SEED)

lat0, lon0 = 47.3977, 8.5456
grid_size = 400
lat_extent, lon_extent = 0.1, 0.1
lats = np.linspace(lat0 - lat_extent/2, lat0 + lat_extent/2, grid_size)
lons = np.linspace(lon0 - lon_extent/2, lon0 + lon_extent/2, grid_size)

def octave(cells, amp):
    g = rng.standard_normal((cells, cells))
    z = zoom(g, (grid_size/cells, grid_size/cells), order=3)[:grid_size, :grid_size]
    return amp * (z - z.mean()) / z.std()

ALT = np.full((grid_size, grid_size), 400.0)
# Increased high-frequency amplitudes for better matching uniqueness
for cells, amp in [(4, 100), (8, 80), (16, 60), (32, 40), (64, 30), (128, 20)]:
    ALT += octave(cells, amp)
ALT -= ALT.min() - 100.0

gy, gx = np.gradient(ALT, np.mean(np.diff(lats))*111320, np.mean(np.diff(lons))*74000)
slope = np.degrees(np.arctan(np.hypot(gx, gy)))
print(f"Alt range: [{ALT.min():.0f}, {ALT.max():.0f}] m | std: {ALT.std():.0f} m")
print(f"Slope: mean {slope.mean():.1f} deg, max {slope.max():.1f} deg")

p = ALT[180:220, 180:220]
corr = fftconvolve(ALT, p[::-1, ::-1], mode='valid')
msq  = fftconvolve(ALT**2, np.ones_like(p), mode='valid')
ssd  = msq - 2*corr + (p**2).sum()
iy, ix = np.unravel_index(np.argmin(ssd), ssd.shape)
mask = np.ones_like(ssd, bool); r = 25
mask[max(iy-r,0):iy+r, max(ix-r,0):ix+r] = False
jy, jx = np.unravel_index(np.argmin(np.where(mask, ssd, np.inf)), ssd.shape)
rms2 = np.sqrt(ssd[jy, jx]/p.size)
print(f"True-match SSD ~ {ssd[iy,ix]:.1f} | 2nd-best RMS mismatch: {rms2:.1f} m")
print("PASS: terrain unique" if rms2 > 25 else "FAIL: terrain too repetitive")

out_dir = Path(__file__).resolve().parent.parent / 'data' / 'maps'
out_dir.mkdir(parents=True, exist_ok=True)
with h5py.File(out_dir/'terrain.h5', 'w') as f:
    f.create_dataset('lats', data=lats); f.create_dataset('lons', data=lons)
    f.create_dataset('alt', data=ALT)

plt.figure(figsize=(10, 8))
plt.contourf(lons, lats, ALT, levels=25, cmap='terrain')
plt.colorbar(label='Altitude (m)'); plt.xlabel('Longitude'); plt.ylabel('Latitude')
plt.title('Synthetic Terrain Map (fractal, high uniqueness)')
plt.tight_layout(); plt.savefig(out_dir/'terrain_preview.png', dpi=150)
print(f"Saved {out_dir/'terrain.h5'} and terrain_preview.png")
