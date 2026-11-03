#!/usr/bin/env python3
"""Build a Gazebo (Harmonic) world from a GDN terrain map, for closed-loop SITL.

The terrain comes straight from the onboard map file (default zermatt.bin), so the
simulated ground and GDN's map share one geometry: same centre, same scale, same
orientation. (R1.3 / R2.2 will later build the world from a finer DEM, so the filter
sees realistic map error.)

Frames:
  - World origin (x=0, y=0) is the map centre, which is also GDN's local NED origin.
    Gazebo is ENU: x = east, y = north, z = up.
  - World z = 0 is the lowest map point; the AMSL height of z = 0 goes into
    <spherical_coordinates><elevation>, so PX4's GPS altitude is AMSL.
  - Each heightmap pixel is sampled with MapDb's bilinear interpolation at exactly the
    position Gazebo will put it (row 0 = north), so terrain and map agree point for
    point. tools/check_sim_world.py proves it with a physics drop test.

    python3 tools/make_sim_world.py                       # zermatt -> data/sim/zermatt/
    python3 tools/make_sim_world.py --map data/maps/terrain_db_sigma4.bin --name sigma4

The world has no <plugin> entries on purpose: PX4 loads the system plugins from its
server.config (GZ_SIM_SERVER_CONFIG_PATH), as with its own worlds.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image

from gdn_paths import ROOT, load_map

# Earth field for the world, ENU, tesla. Same value as PX4's own Gazebo worlds
# (north 23 uT, east 6 uT, down 42 uT). GDN's mag reference must match:
# eskf_node mag_ref_north/east/down = 23.0 / 6.0 / 42.0 uT (sim_run.sh sets this).
MAG_ENU_T = (6e-06, 2.3e-05, -4.2e-05)


def sample_map(alt, hdr, east, north):
    """Bilinear map height at local east/north metres, exactly as gdn::MapDb::Query.

    MapDb puts sample (i, j) at north = (ny/2 - i) * cell (when dlat < 0, row 0 at
    the north edge) and east = (j - nx/2) * cell. Out-of-range points are clamped.
    """
    ny, nx = alt.shape
    c_e = abs(hdr["dlon"]) * hdr["m_per_deg_lon"]
    c_n = abs(hdr["dlat"]) * hdr["m_per_deg_lat"]
    fj = east / c_e + nx / 2.0
    fi = (ny / 2.0 - north / c_n) if hdr["dlat"] < 0 else (ny / 2.0 + north / c_n)
    fi = np.clip(fi, 0.0, ny - 1.000001)
    fj = np.clip(fj, 0.0, nx - 1.000001)
    i0 = np.floor(fi).astype(int); j0 = np.floor(fj).astype(int)
    ti = fi - i0; tj = fj - j0
    a00 = alt[i0, j0]; a01 = alt[i0, j0 + 1]; a10 = alt[i0 + 1, j0]; a11 = alt[i0 + 1, j0 + 1]
    return (1 - ti) * ((1 - tj) * a00 + tj * a01) + ti * ((1 - tj) * a10 + tj * a11)


def heightmap_grid(alt, hdr, oversample=4):
    """Gazebo heightmap covering exactly the span of the map's samples.

    Gazebo puts image pixel (r, k) of a side x side heightmap at
      x = pos_x - size_e/2 + k * size_e/(side-1),  y = pos_y + size_n/2 - r * size_n/(side-1)
    (row 0 = +y = north). The size and pos chosen here line the first and last pixels
    up with the map's first and last samples, and every pixel is sampled with MapDb's
    own bilinear interpolation, so the world and the filter's map agree everywhere.

    Gazebo's physics turns the heightmap into two flat triangles per pixel, while the
    map is bilinear. On steep terrain with 20 to 30 m cells the two differ by several
    metres, so the image is sampled `oversample` times finer than the map: the
    difference shrinks with the square of the pixel size.
    """
    ny, nx = alt.shape
    c_e = abs(hdr["dlon"]) * hdr["m_per_deg_lon"]
    c_n = abs(hdr["dlat"]) * hdr["m_per_deg_lat"]
    e_min, e_max = (0 - nx / 2.0) * c_e, (nx - 1 - nx / 2.0) * c_e
    if hdr["dlat"] < 0:
        n_max, n_min = (ny / 2.0) * c_n, (ny / 2.0 - (ny - 1)) * c_n
    else:
        n_min, n_max = (0 - ny / 2.0) * c_n, (ny - 1 - ny / 2.0) * c_n
    size_e, size_n = e_max - e_min, n_max - n_min
    pos_e, pos_n = (e_max + e_min) / 2.0, (n_max + n_min) / 2.0
    side = min(2 ** math.ceil(math.log2((max(nx, ny) - 1) * oversample)) + 1, 2049)
    k = np.arange(side)
    east = pos_e - size_e / 2.0 + k * size_e / (side - 1)
    north = pos_n + size_n / 2.0 - k * size_n / (side - 1)
    E, N = np.meshgrid(east, north)            # row r <-> north[r], col k <-> east[k]
    img = sample_map(alt, hdr, E, N)
    return img, side, size_e, size_n, pos_e, pos_n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default=str(ROOT / "data/maps/zermatt.bin"))
    ap.add_argument("--name", default="zermatt")
    ap.add_argument("--out-dir", default=None, help="default data/sim/<name>")
    ap.add_argument("--model", default="x500_gdn", help="model to place on the pad (model://NAME)")
    ap.add_argument("--model-instance", default="x500_gdn_0")
    ap.add_argument("--oversample", type=int, default=4,
                    help="heightmap pixels per map cell (finer = closer to bilinear)")
    a = ap.parse_args()

    alt, hdr = load_map(a.map)
    alt = alt.astype(np.float64)
    img, side, size_e, size_n, pos_e, pos_n = heightmap_grid(alt, hdr, a.oversample)

    mn, mx = float(img.min()), float(img.max())
    relief = max(mx - mn, 1.0)
    u16 = np.clip(np.round((img - mn) / relief * 65535), 0, 65535).astype(np.uint16)

    out = Path(a.out_dir) if a.out_dir else ROOT / "data/sim" / a.name
    out.mkdir(parents=True, exist_ok=True)
    png = (out / "heightmap.png").resolve()
    Image.fromarray(u16).save(png)

    # Terrain height at the map centre (the take-off pad), relative to world z = 0.
    pad_amsl = float(sample_map(alt, hdr, np.array(0.0), np.array(0.0)))
    pad_z = pad_amsl - mn
    spawn_z = pad_z + 0.3

    sdf = f"""<?xml version="1.0" ?>
<!-- Generated by tools/make_sim_world.py from {Path(a.map).name}. Don't edit by hand. -->
<sdf version="1.9">
  <world name="{a.name}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>{MAG_ENU_T[0]} {MAG_ENU_T[1]} {MAG_ENU_T[2]}</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.8 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 2000 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-60 -60 {pad_z + 40:.1f} 0 0.4 0.785</pose>
      </camera>
    </gui>
    <model name="terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://{png}</uri>
              <size>{size_e:.3f} {size_n:.3f} {relief:.3f}</size>
              <pos>{pos_e:.3f} {pos_n:.3f} 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>file://{png}</uri>
              <size>{size_e:.3f} {size_n:.3f} {relief:.3f}</size>
              <pos>{pos_e:.3f} {pos_n:.3f} 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.45 0.42 0.36 1</ambient>
            <diffuse>0.55 0.52 0.45 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <include>
      <uri>model://{a.model}</uri>
      <name>{a.model_instance}</name>
      <pose>0 0 {spawn_z:.3f} 0 0 0</pose>
    </include>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{hdr["lat_c"]:.9f}</latitude_deg>
      <longitude_deg>{hdr["lon_c"]:.9f}</longitude_deg>
      <elevation>{mn:.3f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
  </world>
</sdf>
"""
    world = out / f"{a.name}.sdf"
    world.write_text(sdf)
    meta = out / "world_info.txt"
    meta.write_text(
        f"world={a.name}\nmap={Path(a.map).resolve()}\nsize_e={size_e:.3f}\nsize_n={size_n:.3f}\n"
        f"pos_e={pos_e:.3f}\npos_n={pos_n:.3f}\nrelief={relief:.3f}\nz0_amsl={mn:.3f}\npad_amsl={pad_amsl:.3f}\npad_z={pad_z:.3f}\n"
        f"spawn_z={spawn_z:.3f}\nmodel_instance={a.model_instance}\nheightmap_side={side}\n")
    print(f"wrote {world}")
    print(f"  heightmap {side}x{side}, {size_e:.0f} m east x {size_n:.0f} m north, relief {relief:.0f} m")
    print(f"  world z=0 is {mn:.1f} m AMSL; pad at z={pad_z:.1f} m ({pad_amsl:.1f} m AMSL)")


if __name__ == "__main__":
    main()
