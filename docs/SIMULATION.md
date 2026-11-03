# Closed-loop simulation (SITL)

This is roadmap Phase 2, Stage A. PX4 flies an x500 quadcopter over real terrain
(Zermatt, from SRTM) in Gazebo Harmonic, and GDN runs alongside in **shadow mode**:
it never sees GPS, and its estimate is drawn live against the true path.

| Stage | What you see | Status |
|---|---|---|
| A. Shadow mode | The drone flies a terrain-following lawnmower route. RViz shows GDN's estimated path (red) against the true path (white) over the terrain. | This page. Runs end to end; GDN drifts on the multirotor (see "Current result") |
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
  Zermatt map, and the magnetic field the simulated sensor actually reports (see
  "Magnetometer" below).
- **Recording:** every run records a bag of all GDN inputs plus truth to
  `/tmp/gdn_sim/sitl_bag`, so it can be replayed offline (`tools/sim_replay.sh`).

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
| `gdn.log` | `altitude datum ... AMSL` at start-up, then `DBG` lines once a second. `rej=...(m..` counts rejected magnetometer updates; thousands means the attitude has gone wrong. |
| `adapter.log` | `field in NED ...` every 10 s should stay constant. `radar AGL minus map-predicted AGL: mean ..., rms ...` every 10 s. Small (about 1 m) means the rendered terrain matches the map. If it says `no radar altimeter data`, GPU rendering isn't working, and AGL falls back to the map at the true position. |
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
| `GDN_NODE_ARGS="-p mag_fusion:=align"` | extra `eskf_node` parameters |
| `GDN_MAG_REF="0.9 22.5 -41.8"` | magnetic reference for GDN (north east down, uT); see "Magnetometer" |
| `PX4_DIR=...` | PX4 checkout somewhere other than `~/PX4-Autopilot-1.16` |

## Current result (read this before judging the red line)

Checked in a headless run without a GPU (the altimeter came from the map at the
true position, not the rendered radar):
- PX4 v1.16.2 attaches to the generated world and the x500_gdn model, arms on
  GDN's offboard command and flies the whole route at 60 to 150 m above the terrain.
- All Gazebo topic names in the bridge match, and the magnetometer, baro, truth and
  PX4 topics reach GDN at the expected rates (IMU 83 Hz, mag 100 Hz, baro 50 Hz).
- GDN aligns, sets its altitude datum from the first AGL reading (within 0.3 m of
  the pad) and tracks attitude to within 1 degree through the climb and the first
  turn.
- **It then drifts once forward flight starts, to about 2 km after 4 minutes.**
  That's OI-012 in `docs/open_items.md`. In short: on a slow multirotor that pitches
  hard, nothing observes tilt well enough, and the 3-axis magnetometer update makes
  it worse. The fixed-wing replay (`flight001`, 21 m) doesn't stress this. So for
  now Stage A is the test bench for the estimator, not a demonstration that it
  works. `GDN_NODE_ARGS="-p mag_fusion:=align"` (magnetometer for initial yaw only)
  is about 5x better, still far from the R1.8 limits.

To try a change on the exact same flight without flying again:

```bash
tools/sim_replay.sh /tmp/gdn_sim/sitl_bag                          # as flown
tools/sim_replay.sh /tmp/gdn_sim/sitl_bag -p mag_fusion:=align      # any eskf_node parameter
```

Replays of one bag vary by up to about 40% between runs (playback timing isn't
deterministic), so compare a few runs before trusting a small difference.

## Things that were wrong and how they're handled

These bit during the first runs. They're fixed in the scripts, but worth knowing if
something looks off on a new world or a new PX4 version.

- **Magnetometer.** Gazebo Harmonic's magnetometer publishes gauss, not tesla, and
  ignores the world's `<magnetic_field>` when the world has spherical coordinates.
  It uses its own field model at that lat/lon, and puts the north, east and down
  values into the east, north and up slots. The result is a field that points east
  and up, rotated properly into the body. That's physically odd but self-consistent,
  so the adapter just rotates FLU to FRD, and `sim_run.sh` passes the matching
  reference (north 0.9, east 22.5, down -41.8 uT for Zermatt). `adapter.log` prints
  the field in NED every 10 s. If it changes as the drone turns, the mapping is
  wrong. Set `GDN_MAG_REF="n e d"` for another world. PX4's own bridge reads the
  same sensor as a reflection, which is only right at one heading. PX4 doesn't care,
  because it flies on GPS here.
- **IMU rate.** uXRCE-DDS forwards at most 100 Hz per topic, and PX4 integrates the
  IMU at 250 Hz by default. Each message GDN got covered 4 ms of every 10 ms.
  `sim_run.sh` sets `IMU_INTEG_RATE 100`, and `eskf_node` now logs an error if
  messages cover much less time than their spacing.
- **Lost IMU messages.** On a busy machine an IMU message occasionally goes missing
  (about 0.2 s). During a fast manoeuvre that's 15 to 20 degrees of rotation GDN
  never saw. The node now widens the attitude and velocity covariance across such a
  hole, so the magnetometer can correct it (`IMU hole` in `gdn.log`).
- **Arming.** The x500 airframe refuses to arm without a ground station link.
  `sim_run.sh` sets `NAV_DLL_ACT 0`, so QGroundControl isn't needed. Opening it
  anyway is fine.
- **Micro XRCE-DDS Agent v2.4.2 no longer builds from source,** because the Fast-DDS
  branch it pulls is gone. `setup_sim.sh` uses v2.4.3.

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
