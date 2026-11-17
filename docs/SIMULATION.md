# Closed-loop simulation (SITL)

This is roadmap Phase 2, Stage A. PX4 flies an x500 quadcopter over real terrain
(Zermatt, from SRTM) in Gazebo Harmonic, and GDN runs alongside in **shadow mode**:
it never sees GPS, and its estimate is drawn live against the true path.

| Stage | What you see | Status |
|---|---|---|
| A. Shadow mode | The drone flies a terrain-following lawnmower route. RViz shows GDN's estimated path (red) against the true path (white) over the terrain. | This page. Runs end to end; GDN drifts on the multirotor (see "Current result") |
| B. GPS cut | Same flight, GPS cut mid-route (`GPS_OFF_AT`). PX4 on its own loses position within about 5 s and fails safe (a blind landing). GDN keeps estimating. | This page. Works since TEST-032; before it, the cut never reached PX4 |
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

### Or: the dev image (Docker, R2.1)

The same toolchain, pinned, in a Docker image, so every machine runs the same PX4,
Gazebo and ROS 2. Nothing is installed on the host except Docker (and, for `--gpu`,
the NVIDIA Container Toolkit).

```bash
tools/docker_dev.sh build                  # once; downloads 3 to 4 GB, then builds PX4
tools/docker_check.sh                      # the image is the pinned one, and the loop flies in it
tools/docker_check.sh --gpu                # plus the GPU and the rendered radar altimeter
tools/docker_dev.sh shell --gpu --gui      # a shell, with Gazebo and RViz windows on your desktop
#   inside:  colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
#            tools/sim_run.sh
```

- **What's in it** (pins in `docker/versions.env`):
  - Ubuntu 24.04 and ROS 2 Jazzy desktop, the base image pinned by digest;
  - Gazebo Harmonic;
  - PX4 v1.16.2 with SITL built, in `/opt/PX4-Autopilot`;
  - the Micro XRCE-DDS Agent v2.4.3;
  - `ros_gz_bridge`, the Gazebo Python bindings, and every GDN build dependency.
- **The workspace is mounted, not copied,** at the same path as on the host, so you
  edit on the host and build and run in the container.
  - The container keeps its own `build/` and `install/` (stored in `.docker/`), so a
    host build and a container build never mix. A host build can point at a Python the
    container doesn't have.
  - `log/` is shared, so run logs land where the host sees them.
- **The user inside is you** (same name and UID), so files it writes stay yours.
- **The network is the host's.** ROS 2 topics and Gazebo pass between container and
  host, so RViz on the host sees a simulation running in the container.
- **The image name carries a hash of the `docker/` files.** After a change there,
  `docker_dev.sh` asks for a rebuild instead of running an old image.
- **CI** builds the image and runs `tools/docker_check.sh` (CPU) on PRs that change the
  Docker files (`.github/workflows/docker.yml`).
- **A slow connection:** the build retries interrupted downloads (git, apt, pip). On a
  very slow link it can still take hours; it resumes from the last finished step if
  run again.
- **Portainer** (or `docker ps`) shows the running container, its logs and a console.
  It can't show Gazebo or RViz; use `--gui`, or RViz on the host.

## Fly

```bash
GDN_GPU=nvidia source env.sh
tools/sim_run.sh                    # Stage A
GPS_OFF_AT=120 tools/sim_run.sh     # Stage B: GPS cut 2 minutes into the route
python3 tools/check_gps_cut.py log/gdn_runs/gdn_<time>   # did PX4 really lose GPS?
```

**How the GPS cut works.** `sim_fly.py` sets PX4's `EKF2_GPS_CTRL` to 0 (through
`px4-param`), so PX4's estimator stops using GPS. Within about 5 s PX4 marks its position
invalid, leaves offboard and lands blind, and its uncertainty (eph) climbs to hundreds of
metres. PX4's failure injection (`failure gps off`, `VEHICLE_CMD_INJECT_FAILURE`) isn't
used: in PX4 1.16 only the MAVLink simulator (Gazebo Classic, jMAVSim) acts on it. Under
Gazebo Harmonic it was acknowledged and ignored, so until TEST-032 every Stage B run
actually flew on GPS. `sim_run.sh` sets `EKF2_GPS_CTRL` back to 7 at start, since PX4
keeps its parameters between runs. After a cut, PX4's position is no reference any more:
the scorer and RViz use Gazebo's truth, and `plot_run_log.py` only draws the error while
PX4's position was valid.

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
| `fly.log` | route progress; on Stage B, the GPS cut, PX4 taking it, and PX4 leaving offboard |
| `px4.log`, `gz.log`, `agent.log`, `bridge.log` | if something doesn't start |

Useful variations:

| Variable | Effect |
|---|---|
| `HEADLESS=1` | no windows; the sensors still render through EGL |
| `GDN_GZ_RENDER=ogre` | older renderer, if Ogre2 crashes (see "If Gazebo crashes at start") |
| `GDN_FLY_ARGS="--legs 2 --clearance 80"` | shorter route, more clearance (`python3 tools/sim_fly.py -h`) |
| `GDN_SIM_ADAPTER_ARGS="-p baro_drift_m_per_min:=2.0"` | add a barometer drift |
| `GDN_SIM_ADAPTER_ARGS="-p agl_source:=truth"` | skip the rendered radar (map at the true position instead) |
| `GDN_NODE_ARGS="-p mag_fusion:=align"` | extra `eskf_node` parameters |
| `GDN_MAG_REF="0.9 22.5 -41.8"` | magnetic reference for GDN (north east down, uT); see "Magnetometer" |
| `PX4_DIR=...` | PX4 checkout somewhere other than `~/PX4-Autopilot-1.16` |

## Real-terrain sites (R2.2)

The default world above is built from GDN's own 30 m map, so GDN flies over exactly
the terrain it knows. The five R2.2 sites, by contrast, are built from swisstopo's survey
data. GDN flies them with a different, coarser map, as it would in reality (TEST-033):

| Site | Terrain class | Place | World terrain |
|---|---|---|---|
| `ch_zermatt` | mountain | Sunnegga above Zermatt (pad at 2243 m) | swissALTI3D 2 m |
| `ch_emmental` | rolling hills | near Langnau | swissALTI3D 2 m |
| `ch_seeland` | flat farmland | Grosses Moos (Mont Vully at the south edge) | swissALTI3D 2 m |
| `ch_zurich` | built-up | Zurich, Kaserne parade ground | swissSURFACE3D 0.5 m (buildings, trees) |
| `ch_neuchatel` | coast and open water | Lake Neuchatel at Estavayer-le-Lac (half the route over water) | swissALTI3D 2 m |

```bash
python3 tools/make_site.py --all          # once: downloads (cached in data/terrain/) and builds
python3 tools/check_site.py --all         # each site's maps, pad and world
tools/view_site.sh --all                  # look at them in Gazebo (dev image: shell --gpu --gui)
GDN_SITE=ch_zurich tools/sim_run.sh       # fly one, with GDN on its onboard map
python3 tools/plot_sites.py --out sites.png   # one overview image of all five
```

- **Each site is a 4 by 4 km world around a level take-off pad,** defined in
  `sim/sites.yaml`.
  - **Terrain:** the survey heights are averaged to a 3.9 m grid, which gives both the
    physics heightmap and the rendered mesh the radar sees.
  - **Photo:** SWISSIMAGE at 2 m is draped on the mesh. Finer photos for visual odometry
    come with R3.5, for the flown corridor only.
- **GDN's onboard map is Copernicus GLO-30:** a 30 m satellite surface model, 10 by
  10 km, a different sensor and year from the world.
  - It differs from the world by 2.3 m RMS (lake shore) to 9.5 m (Emmental's forests).
  - `sim_run.sh` passes the pad as GDN's origin, so both maps share one frame.
- **The magnetic reference** is read from the simulated magnetometer at start-up, since
  Gazebo's field depends on where the world is.
- **The first build downloads about 100 MB per site,** and 400 MB for Zurich's 0.5 m
  surface model. Every tile is pinned by checksum in `sim/sites.lock`, so the same
  worlds are rebuilt even after swisstopo publishes newer surveys.
  - Each world is about 210 MB on disk, mostly the mesh.
  - `make_site.py --if-stale` (which `sim_run.sh` and `view_site.sh` use) rebuilds a
    site only when its entry or the building code changes.
- **Both windows follow the drone** during `sim_run.sh`:
  - RViz orbits it about 500 m out, with a marker on the drone. Set the Views panel's
    Target Frame to "<Fixed Frame>" for the old fixed overview.
  - Gazebo's camera sits 3 m behind and 1.2 m above the x500, started by
    `sim_adapter.py` once the window is up.
    - Change it with `GDN_SIM_ADAPTER_ARGS="-p gz_follow_offset:=-6,0,2.5"` (metres
      behind, left, above; commas, no spaces).
    - Turn it off with `GDN_SIM_ADAPTER_ARGS="-p gz_follow:=false"`.
- **In the Gazebo window:**
  - scroll zooms towards the mouse, Shift + drag orbits, drag pans;
  - to find the drone, right-click `x500_gdn_0` in the entity tree, then "Move to"
    (or "Follow" while it flies);
  - scroll gently: zooming out far takes the camera past the 30 km drawing limit,
    and then you see only sky;
  - Gazebo's own start camera would sit inside the terrain, so `tools/gz_gui_config.py`
    gives `view_site.sh` and `sim_run.sh` a start view and that drawing limit.
- **A new site:** add an entry to `sim/sites.yaml`. If the pad isn't level,
  `check_site.py` suggests the nearest level spot; check it on the photo, since a
  lake is level too.
- **Data credits:**
  - World terrain and photos: © swisstopo (swissALTI3D, swissSURFACE3D, SWISSIMAGE;
    open data).
  - Onboard maps: Copernicus WorldDEM-30, © DLR e.V. 2010-2014 and © Airbus Defence
    and Space GmbH 2014-2018, provided under COPERNICUS by the European Union and ESA;
    all rights reserved.

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
- **Why, measured on three flights in the dev image (TEST-032):**
  - The altimeter readings are good: 0.32 m RMS against the map at the true position.
  - Velocity isn't observed. The estimate slows on the straight legs, and the error
    grows to hundreds of metres.
  - The filter is overconfident with altimeter updates on: 15 to 81% of the time over
    its own 3σ. With them off it never is.
  - Once the estimate leaves the map (1 to 2 km off), no altimeter update applies, and
    it runs away at 20 to 40 m/s. That's the "hallucination" at the end of a long flight.
  - No setting fixes it: each wins on one flight and loses on another. The fix is
    velocity aiding (Phase 3).

To try a change on the exact same flight without flying again:

```bash
tools/sim_replay.sh /tmp/gdn_sim/sitl_bag                          # as flown
tools/sim_replay.sh /tmp/gdn_sim/sitl_bag -p mag_fusion:=align      # any eskf_node parameter
```

Replays of one bag vary by up to about 40% between runs (playback timing isn't
deterministic), so compare a few runs before trusting a small difference.

Since R1.11 there is an exact way: every node run writes a run log
(`log/gdn_runs/gdn_<time>/`, named in `node.log`). `gdn_replay` replays it through the
same code with no timing in the way, so the same settings give the same result bit for
bit and a difference is the setting's:

```bash
ros2 run gdn_fusion gdn_replay log/gdn_runs/gdn_...                      # checks it reproduces
ros2 run gdn_fusion gdn_replay log/gdn_runs/gdn_... --set=mag_fusion=align --out=/tmp/align
python3 tools/plot_run_log.py log/gdn_runs/gdn_... /tmp/align --out cmp.png
```

The replay keeps the inputs the node received; it can't show what a change in the
flight itself (PX4 reacting to GDN, R5.1) would do. That still needs `sim_replay.sh`
or a new flight.

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

## If Gazebo crashes at start

`sim_run.sh` runs the Gazebo server and GUI as separate processes. If the server dies,
it stops and prints the error from `gz.log`. If only the window dies, the flight
carries on, and `gz_gui.log` says why.

- **`Fragment Program ... PixelShader_ps failed to compile` / `OGRE EXCEPTION`**: this
  was Ogre2's heightmap terrain shader, and it failed the same way on the dev PC's
  NVIDIA card and on plain Mesa. Worlds made by `make_sim_world.py` now draw the
  terrain as a mesh instead, so it shouldn't happen any more. Delete
  `data/sim/<world>` if an old world is still around (`sim_run.sh` regenerates it
  when the generator is newer). If some other Ogre2 error turns up,
  `GDN_GZ_RENDER=ogre` switches to the older renderer.
- **`Exception sending a multicast message: Network is unreachable`**: Gazebo's
  messaging had no network route. `sim_run.sh` now sets `GZ_IP=127.0.0.1`, so it stays
  on this machine.
- **Waiting for PX4 forever**: PX4 runs in lockstep with Gazebo, so it doesn't
  publish anything until the Gazebo server is running. Check `gz.log` first.

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
ground, and the pad is flat.

`tools/check_sim_radar.py` checks what the radar altimeter sees, which is the
rendered terrain rather than the physics one. It puts a probe carrying the x500_gdn
radar 100 m above the map at 26 points and compares the ranges:

| Terrain visual | Ogre1 | Ogre2 |
|---|---|---|
| Heightmap (before) | median +2 m, but up to 177 m off, and no return at 4 points | crashes (shader) |
| Mesh (now) | median +0.1 m, max 0.8 m | median +0.2 m, max 1.4 m |

During a run, `adapter.log` reports the same comparison every 10 s along the real
flight path (`radar AGL minus map-predicted AGL`).

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
