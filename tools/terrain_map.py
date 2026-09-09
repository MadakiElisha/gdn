import h5py
import numpy as np
from scipy.interpolate import RectBivariateSpline
from pathlib import Path

class TerrainMap:
    def __init__(self, map_path=None):
        if map_path is None:
            map_path = Path(__file__).resolve().parent.parent / 'data' / 'maps' / 'terrain.h5'
        with h5py.File(map_path, 'r') as f:
            self.lats = f['lats'][:]
            self.lons = f['lons'][:]
            self.alt = f['alt'][:]
        # Build interpolator for fast queries
        self.interp = RectBivariateSpline(self.lats, self.lons, self.alt, kx=1, ky=1)

    def get_altitude(self, lat, lon):
        """Query terrain altitude at (lat, lon). Returns scalar."""
        return float(self.interp(lat, lon, grid=False))

    def get_altitude_grid(self, lats, lons):
        """Query terrain altitude at arrays of (lat, lon). Returns array."""
        return self.interp(lats, lons, grid=False)

if __name__ == '__main__':
    # Quick test
    tm = TerrainMap()
    test_lat, test_lon = 47.3977, 8.5456
    print(f"Altitude at ({test_lat}, {test_lon}): {tm.get_altitude(test_lat, test_lon):.1f} m")
