# Closed-loop simulation (SITL)

This is roadmap Phase 2, Stage A. PX4 flies an x500 quadcopter over real terrain
(Zermatt, from SRTM) in Gazebo Harmonic, and GDN runs alongside in **shadow mode**:
it never sees GPS, and its estimate is drawn live against the true path.

| Stage | What you see | Status |
|---|---|---|
| A. Shadow mode | The drone flies a terrain-following lawnmower route. RViz shows GDN's estimated path (red) against the true path (white) over the terrain. | This page |
| B. GPS cut | Same flight, GPS failure injected mid-route (`GPS_OFF_AT`). PX4 on its own loses position and fails safe. GDN keeps estimating. | This page |
| C. Flying on GDN | PX4 uses GDN's position and finishes the route with GPS off. | Needs R5.1 |

## What runs

```
 Gazebo Harmonic  ──IMU, baro, mag, GPS──►  PX4 v1.16 SITL  ──uXRCE-DDS──►  /fmu/out/*  ──►  GDN eskf_node
   x500_gdn model                            (flies the                     sensor_combined,      (shadow mode,
   + radar altimeter                          route, offboard)              attitude, local pos    no GPS input)
   + truth odometry                                                                                   │
        │                                                                                             ▼
        └──ros_gz_bridge──► sim_adapter.py ──► /gdn/agl_meas, /gdn/baro_meas, /gdn/mag_meas ──────────┘
                                         └──► /gdn/truth, /gdn/truth_path ──► replay_scorer, RViz
```

- **World:** `tools/make_sim_world.py` builds it straight from GDN's map file, so terrain
  and map share one geometry. The heightmap is oversampled 4x, so Gazebo's
  triangulated surface stays close to the map's bilinear one.
- **Drone:** `sim/models/x500_gdn` is PX4's x500 plus a downward radar altimeter
  (1-beam lidar, 0.2 to 1500 m, 20 Hz, 0.3 m noise) and a truth-pose publisher. The
  truth is used for scoring only.
- **Adapter:** `tools/sim_adapter.py` turns raw Gazebo sensors into GDN's inputs. It
  removes tilt from the altimeter using PX4's attitude, converts pressure to height,
  and converts the magnetometer to body FRD in microtesla. Every 10 s it also logs
  how far the radar AGL is from what the map predicts at the true position. That's
  the live check that the rendered terrain matches GDN's map.
- **Flight:** `tools/sim_fly.py` flies offboard. The route is planned from the same
  map, at 60 m above the terrain envelope with a 200 m lookahead. The default route
  is 7.9 km, about 13 minutes.
- **GDN:** `eskf_node` with the real altimeter model (`trn_model:=agl`), the
  Zermatt map, and the world's magnetic field (north 23, east 6, down 42 uT).

## One-time setup

```bash
cd ~/gdn_workspace
source env.sh
tools/setup_sim.sh        # PX4 v1.16.2 in ~/PX4-Autopilot-1.16, Gazebo Harmonic,
                          # ros_gz_bridge, Micro XRCE-DDS Agent; 20 to 40 min
colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
tools/migrate_flight001.sh    # once: px4_msgs is now 1.16, and the old bag needs converting
```

On a laptop with hybrid graphics, check that rendering uses the NVIDIA GPU (see
`docs/SETUP.md`, "GPU for simulation").

## Fly

```bash
GDN_GPU=nvidia source env.sh
tools/sim_run.sh                    # Stage A
GPS_OFF_AT=120 tools/sim_run.sh     # Stage B: GPS cut 2 minutes into the route
```

Two windows open:
- **Gazebo:** the x500 on its pad in the Alps. The camera can be moved with the mouse.
  Right-click the drone and choose "Follow" to track it.
- **RViz:** GDN's map as a 3D terrain, the true path in white and GDN's estimate in
  red.

The terminal shows the route plan, flight progress and, at the end, GDN's error
against truth for the first 5 minutes, the first 10 minutes and the whole run.
Everything is logged to `/tmp/gdn_sim/`:

| Log | What to look for |
|---|---|
| `gdn.log` | `altitude datum ... AMSL` at start-up, then `DBG` lines once a second |
| `adapter.log` | `radar AGL minus map-predicted AGL: mean ..., rms ...` every 10 s. Small (about 1 m) means the rendered terrain matches the map. If it says `no radar altimeter data`, GPU rendering isn't working, and AGL falls back to the map at the true position. |
| `scorer.log` | horizontal error every 20 s, and the summary |
| `fly.log` | route progress, GPS failure injection |
| `px4.log`, `gz.log`, `agent.log`, `bridge.log` | if something doesn't start |

Useful variations:

| Variable | Effect |
|---|---|
| `HEADLESS=1` | no windows; the sensors still render through EGL |
| `GDN_FLY_ARGS="--legs 2 --clearance 80"` | shorter route, more clearance (`python3 tools/sim_fly.py -h`) |
| `GDN_SIM_ADAPTER_ARGS="-p baro_drift_m_per_min:=2.0"` | add a barometer drift |
| `GDN_SIM_ADAPTER_ARGS="-p agl_source:=truth"` | skip the rendered radar (map at the true position instead) |
| `PX4_DIR=...` | PX4 checkout somewhere other than `~/PX4-Autopilot-1.16` |

## How the world is checked

`tools/check_sim_world.py` runs a headless physics drop test. It sets 30 boxes down
on the Gazebo terrain and compares their resting heights with the map. It also scores
the same points against a mirrored and a shifted map, to prove it would catch those
mistakes. Result on the Zermatt world:

| | Median error |
|---|---|
| This world | 0.38 m (pad 0.00 m) |
| Mirrored east-west | 172 m |
| Mirrored north-south | 189 m |
| Shifted 50 m east | 13.5 m |
| Shifted 50 m north | 6.0 m |

Single probes on the steepest cells can sit several metres off. That's because
Gazebo's physics surface is made of triangles. It only matters for touching the
ground, and the pad is flat. The radar altimeter measures the rendered terrain, and
the adapter's live check covers that part on the machine that renders it.

## Known limits

- **PX4 flies on GPS in Stages A and B.** Its own estimator uses Gazebo's simulated GPS.
  GDN is only compared against it. Stage C needs R5.1.
- **The world is built from the onboard map itself,** so there's no map error yet. R1.3
  and R2.2 build the world from a finer DEM, with real map error.
- **The node still reads PX4's `vehicle_local_position`** for its ground ZUPT and take-off
  latch (R1.2).
- **The terrain has no photo texture yet.** That matters once the OAK-D Lite cameras are
  simulated for VIO (R2.2, R2.4).
- **Only the radar altimeter is simulated so far.** The Mid-360S, TFmini-S and OAK-D
  Lite come next (R2.4).
