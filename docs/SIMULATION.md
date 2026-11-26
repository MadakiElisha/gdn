# Closed-loop simulation (SITL)

This is roadmap Phase 2, Stage A. PX4 flies a vehicle over real terrain (Zermatt, from
SRTM, or one of the R2.2 sites) in Gazebo Harmonic, and GDN runs alongside in **shadow
mode**: it never sees GPS, and its estimate is drawn live against the true path. The
vehicle is an x500 quadcopter by default, or a standard VTOL or a fixed-wing (R2.3,
"Vehicles" below), each carrying the same GDN payload.

| Stage | What you see | Status |
|---|---|---|
| A. Shadow mode | The drone flies a terrain-following lawnmower route. RViz shows GDN's estimated path (red) against the true path (white) over the terrain. | This page. Runs end to end; GDN drifts on the multirotor (see "Current result") |
| B. GPS cut | Same flight, GPS cut mid-route (`GPS_OFF_AT`). PX4 on its own loses position within about 5 s and fails safe (a blind landing). GDN keeps estimating. | This page. Works since TEST-032; before it, the cut never reached PX4 |
| C. Flying on GDN | PX4 uses GDN's position and finishes the route with GPS off. | Needs R5.1 |

## What runs

```
 Gazebo Harmonic  ──IMU, baro, mag, GPS──►  PX4 v1.16 SITL  ──uXRCE-DDS──►  /fmu/out/*  ──►  GDN eskf_node
   vehicle model                             (flies the                     sensor_combined,      (shadow mode,
   + GDN payload (radar altimeter,            route, offboard)              attitude, local pos    no GPS input)
     IMUs, TFmini-S, truth)                                                                           │
        │                                                                                             ▼
        ├─ros_gz_bridge──► sim_adapter.py ──► /gdn/agl_meas, /gdn/baro_meas, /gdn/mag_meas ──────────┘
        │                                └──► /gdn/truth, /gdn/truth_path ──► replay_scorer, RViz
        └─ros_gz_bridge──► sim_sensors ──► /imu/data, /livox/imu, /oak/imu/data, /tfmini/range
                                           (as the real drivers publish them; not used by GDN yet)
```

- **World:** `tools/make_sim_world.py` builds it straight from GDN's map file, so terrain
  and map share one geometry. The heightmap is oversampled 4x, so Gazebo's
  triangulated surface stays close to the map's bilinear one.
- **Vehicle:** `sim/models/x500_gdn` (default), `standard_vtol_gdn` or `rc_cessna_gdn`:
  PX4's own model with the GDN payload (`sim/models/gdn_payload`) included and fixed
  under the body. The payload holds a downward radar altimeter (1-beam lidar, 0.2 to
  1500 m, 20 Hz, 0.3 m noise), three IMUs and a TFmini-S (see "Simulated sensors"),
  and the truth publisher: the payload frame's true pose, used for scoring and checks
  only.
- **Adapter:** `tools/sim_adapter.py` turns raw Gazebo sensors into GDN's inputs. It
  removes tilt from the altimeter using PX4's attitude, converts pressure to height,
  and converts the magnetometer to body FRD in microtesla. Every 10 s it also logs
  how far the radar AGL is from what the map predicts at the true position. Level,
  that's the live check that the rendered terrain matches GDN's map (about 0.3 m);
  tilted over slopes it's metres, because the tilt removal assumes flat ground
  (OI-016). `sim_run.sh` prints the whole flight's figure at the end.
- **Flight:** `tools/sim_fly.py` flies offboard. The route is planned from the same
  map, at 60 m above the terrain envelope with a 200 m lookahead. The default route
  is 7.9 km, about 13 minutes for the x500 and 9 for the planes.
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
- **The CPU check flies without the cameras and lidar** (`GDN_SIM_CPU=1`, set by
  `docker_check.sh` without `--gpu`).
  - **Why:** rendered on the processor, they slowed Gazebo to about a tenth of real
    time, and the x500 never finished its climb (R2.4c).
  - **What stays:** the IMUs, TFmini-S and radar altimeter, which is all the loop
    needs.
  - **On a PC without a GPU,** `GDN_SIM_CPU=1 tools/sim_run.sh` does the same.
- **Gazebo's programs run on the system's Gazebo libraries.** ROS 2 brings its own copy of
  some of them (`/opt/ros/jazzy/opt/gz_*_vendor`), first on `LD_LIBRARY_PATH`. When the
  system's Gazebo is newer (gz-transport 13.7.0 against ROS's 13.6.0, November 2026), `gz
  service` fails with "symbol lookup error", and PX4 never finds the world. So
  `sim_run.sh` takes ROS's copies off the path for `gz sim`, the `gz` command line and
  PX4. `docker_check.sh` tests `gz service` the same way.
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
GDN_VEHICLE=standard_vtol tools/sim_run.sh   # another vehicle: x500, standard_vtol, rc_cessna
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

**Vehicle profiles (R2.5a).** GDN runs with the flown vehicle's profile,
`config/vehicles/<vehicle>.yaml` (`GDN_PROFILE` picks another):
- **What it holds:** the filter settings (a file plus overrides), which aiding GDN uses
  (AGL, baro, magnetometer, zero-velocity), and every sensor with its driver topic,
  frame, noise model and mount.
- **Mounts:** the pose of the sensor's frame in the airframe's `base_link` (x forward,
  y left, z up, degrees), placeholders until R6.4.
- **Where it shows:** the node logs the profile's fingerprint (`gdn.log`: "vehicle
  profile: ..."), and its run log carries the profile so `gdn_replay` repeats it.
- **The check:** `tools/check_profiles.py` checks every profile against the simulation:
  the payload's mounts, the topics, frames and noise files, and PX4's airframe and model.
- **The flight regime (R2.5b):** GDN takes it from PX4's land detector and status:
  - **The regimes:** ground (landed or disarmed), hover, transition (a VTOL changing
    over), cruise (flying as a plane).
  - **The rules:** the profile's `regime.*` rules say which aiding each one allows. The
    altimeter is off on the ground, and zero-velocity updates run only on the ground and
    resume after every landing.
  - **Where it shows:** `gdn.log` shows each change ("regime: ground -> hover ...").
  - **The check:** `tools/check_regime.py --run LOG` checks a flight against PX4's regime.

**Drawing a run.** `python3 tools/plot_sim_run.py LOG` (the run's `GDN_LOG_DIR`, in the
dev image) draws the whole flight on one page, `LOG/gdn_vs_truth.png`:
- GDN's track against Gazebo's truth over the world's terrain, with the planned route;
- the horizontal, north, east, vertical, heading and velocity error, each with GDN's own
  3 sigma, shaded by flight phase from PX4 (ground, hover, transition, cruise);
- every terrain altimeter reading (applied, gated, or skipped and why), every update
  decision per sensor, and a summary table.

It takes GDN's uncertainty and decisions from its run log (`log/gdn_runs`, found by time).
`GDN_LOG_DIR` may be relative (`log/run1`); the container's `/tmp` is gone when its shell
closes, so keep runs you want in the workspace.

Two windows open:
- **Gazebo:** the x500 on its pad in the Alps. The camera can be moved with the mouse.
  Right-click the drone and choose "Follow" to track it.
- **RViz:** the terrain as a brown surface and as points coloured by height, a 100 m
  grid at pad level, the true path in white and GDN's estimate in red. The view orbits
  the drone. There is no lidar point cloud yet: the only sensor simulated so far is the
  radar altimeter, a single beam (the Mid-360S comes with R2.4).

The terminal shows the route plan, flight progress and, at the end, GDN's error
against truth for the first 5 minutes, the first 10 minutes and the whole run.
Everything is logged to `/tmp/gdn_sim/`:

| Log | What to look for |
|---|---|
| `gdn.log` | `altitude datum ... AMSL` at start-up, then `DBG` lines once a second. `rej=...(m..` counts rejected magnetometer updates; thousands means the attitude has gone wrong. |
| `adapter.log` | `field in NED ...` every 10 s should stay constant. `radar AGL minus map-predicted AGL: mean ..., rms ...` every 10 s. About 0.3 m while level (hover, the pad) means the rendered terrain matches the map; in cruise over slopes it's metres, from the tilt (OI-016). If it says `no radar altimeter data`, GPU rendering isn't working, and AGL falls back to the map at the true position. |
| `scorer.log` | horizontal error every 20 s, and the summary |
| `fly.log` | route progress; on Stage B, the GPS cut, PX4 taking it, and PX4 leaving offboard |
| `sensors.log` | the simulated sensors (R2.4): each one's rate and lost messages every 10 s |
| `px4.log`, `gz.log`, `agent.log`, `bridge.log` | if something doesn't start |

Useful variations:

| Variable | Effect |
|---|---|
| `HEADLESS=1` | no windows; the sensors still render through EGL |
| `GDN_GZ_RENDER=ogre` | older renderer, if Ogre2 crashes (see "If Gazebo crashes at start") |
| `GDN_VEHICLE=rc_cessna` | the vehicle (`python3 tools/sim_vehicles.py --list`; see "Vehicles") |
| `GDN_FW_LAUNCH=catapult` | the fixed-wing's take-off: `runway` (default) or `catapult` |
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
  - Gazebo's camera sits 1.8 m behind and 0.7 m above the vehicle (the x500 fills
    about half the view), started by `sim_adapter.py` once the window is up. It keeps up in
    cruise because `tools/gz_gui_config.py` raises the follow gain; with Gazebo's
    default the camera trailed many metres behind, whatever the offset.
    - Change it with `GDN_SIM_ADAPTER_ARGS="-p gz_follow_offset:=-6,0,2.5"` (metres
      behind, left, above; commas, no spaces).
    - Turn it off with `GDN_SIM_ADAPTER_ARGS="-p gz_follow:=false"`.
- **In the Gazebo window:**
  - scroll zooms towards the mouse, Shift + drag orbits, drag pans;
  - to find the drone, right-click `x500_gdn_0` in the entity tree (or
    `standard_vtol_gdn_0`, `rc_cessna_gdn_0`), then "Move to" (or "Follow" while it
    flies);
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

## Vehicles (R2.3)

Three vehicles, one payload. Each is PX4's own Gazebo model and airframe, with the GDN
payload included through an SDF `<include>` and fixed to its body. Sensors are added to
`sim/models/gdn_payload` once and every vehicle carries them (see "Simulated sensors"). `tools/sim_vehicles.py` holds the table: model, PX4
airframe, PX4 settings and how the vehicle flies.

| `GDN_VEHICLE` | Model | PX4 airframe | How it flies the route |
|---|---|---|---|
| `x500` (default) | `x500_gdn` | 4001 `gz_x500` | climbs straight up, follows a moving setpoint ("carrot") at 10 m/s, lands on the pad |
| `standard_vtol` | `standard_vtol_gdn` | 4004 `gz_standard_vtol` | climbs like the x500, changes to plane flight, flies the route at about 15 m/s, changes back 350 m before the pad and lands on it |
| `rc_cessna` | `rc_cessna_gdn` | 4003 `gz_rc_cessna` | takes off along the runway, flies the route at about 15 m/s, lands back on the runway |

```bash
python3 tools/sim_vehicles.py --list
GDN_VEHICLE=rc_cessna GDN_SITE=ch_emmental tools/sim_run.sh
GDN_VEHICLE=rc_cessna GDN_FW_LAUNCH=catapult tools/sim_run.sh   # launched by catapult instead
python3 tools/check_vehicles.py                   # the models and their airframes
python3 tools/check_vehicles.py --run /tmp/gdn_sim # judge a flight (phases, route, terrain, radar)
```

- **As a plane, the route is followed as a line.** `sim_fly.py` sends a point 60 m
  ahead on the route with the route's direction as velocity. PX4's fixed-wing
  controller then follows the line through them. Its 90 degree corners are flown
  wide: the cross-track error has a median of about 1 m, a 95th percentile of 6 to
  8 m, and reaches 23 to 35 m at the corners (TEST-034).
- **The route never climbs or descends faster than a plane can.** For the planes, its
  height is limited to a 25% (VTOL) or 17% (cessna) gradient, half of what PX4's climb
  limits allow at cruise speed, so climbs start earlier.
- **The VTOL changes over in Hold, not in offboard.** In PX4 1.16 the transition flight
  task publishes its own `trajectory_setpoint`, the same topic offboard setpoints
  arrive on. In offboard the plane controller then saw no valid setpoint and gave no
  forward thrust; the VTOL hung in the air, sank 20 m and PX4 aborted the transition
  (its "quad-chute"). In Hold, PX4 does the transition itself; `sim_fly.py` goes back to
  offboard once it's done. Front transition takes 7 to 8 s, back transition about 5.
  This is about offboard only: in its own modes (Hold, missions, a pilot's switch) PX4
  1.16 transitions normally.
- **The VTOL's transition throttle is 0.6** (`VT_F_TRANS_THR`; the airframe's is 0.3).
  With 0.3 the pusher got it to only 7.7 to 8 m/s by the time PX4 checks (8 m/s after
  about 6.6 s), and one transition in five at Zermatt's 2600 m was aborted.
- **Every world has a runway through the pad,** 300 by 20 m (`make_sim_world.py`): the
  terrain is graded along the heading that needs the least earthworks, at a constant
  grade of at most 3%, and the strip is drawn grey with a white centreline. The
  ground moved 0.2 m (Seeland) to 12.6 m (Zurich, through buildings).
  - **Only the world gets it.** The graded terrain is the world's own map
    (`world_map.bin`, which the physics, radar and checks use); GDN's onboard map
    doesn't know the strip, as for an airfield built after the survey.
  - **The x500 and the VTOL** still take off from and land on the pad, which is on the
    runway.
- **The cessna takes off along the runway** with PX4's runway take-off (Takeoff mode:
  it rolls, rotates and climbs out to 50 m), facing down the runway, then flies the
  route.
- **It lands back on the runway with a PX4 landing mission.** PX4 lands a plane on a
  runway only from a mission, so `sim_fly.py` uploads one over MAVLink (pymavlink,
  UDP 14540) once the plane has taken off. Not before: PX4 sets home when it arms, and
  a mission uploaded earlier was checked against the previous run's home, 112 km
  away; that landing skipped its approach and circled down beside the runway. The
  mission:
  - **the approach point,** at least 600 m out on PX4's 8° glide path, from the runway
    end whose approach keeps the terrain furthest under that path (at least 20 m);
  - **the touchdown point,** 50 m into the runway.

  At the end of the route `sim_fly.py` switches PX4 to Mission. PX4 flies out to the
  approach point, turns, glides down, flares and rolls out on the runway, then
  disarms. Two PX4 details:
  - **`FW_LND_USETER 0`:** with PX4's default, the cessna (no rangefinder) found no
    terrain measurement on final approach and aborted the landing.
  - **No `DO_LAND_START` item:** PX4 rejects it in a position frame.
- **Or by catapult** (`GDN_FW_LAUNCH=catapult`): PX4 waits in Takeoff mode with its
  launch detection on; `sim_fly.py` asks Gazebo to push the plane forward at 40 m/s²
  (ApplyLinkWrench) until it's at 16 m/s, about half a second. PX4 detects the launch
  and climbs out; the landing is on the runway as above. In a world without a runway
  (`make_sim_world.py --runway 0`) the cessna circles over the pad at the end
  instead.
- **The worlds still place the x500.** `sim_run.sh` writes a copy of the world into the
  log directory with the chosen vehicle in its place, so the worlds and
  `view_site.sh` are unchanged.
- **PX4 resets its parameters when the airframe changes,** then `sim_run.sh` sets this
  setup's ones (`VEHICLE_PARAMS` in `sim_vehicles.py`).
- **The radar on a tilted vehicle** sees the ground beside the point straight below.
  `sim_adapter.py`'s conversion to height assumes flat ground, so over slopes GDN's
  altimeter input is metres off: 6 to 9.5 m RMS on the x500, which cruises tilted, and
  2 to 5 m on the planes (OI-016, TEST-034). `check_vehicles.py` checks the radar along
  its beam, where it matches the terrain to 0.4 to 0.7 m.

## Simulated sensors (R2.4)

The payload carries GDN's sensors, and `sim_sensors` (gdn_fusion; settings in
`config/sim/sensors.yaml`) publishes each the way its real driver does: the same topic,
message type, frame, units and rate, with the sensor's errors and timing. GDN doesn't
read them yet; LIO, VIO and the rangefinder come in Phases 3 and 4. R2.4 comes in three
parts: R2.4a the IMUs and the TFmini-S, R2.4b the OAK-D Lite's cameras, R2.4c the
Mid-360S lidar (now).

| Sensor | Topic | Type, frame | Rate | Errors | Timing |
|---|---|---|---|---|---|
| H30WP | `/imu/data` | `sensor_msgs/Imu`, `imu_link` | 200 Hz, even | R1.6 model (`config/imu/h30wp.yaml`) | stamped on arrival, as `tools/h30wp_node.py` does (no sample time in the factory output); 3 ms latency |
| Mid-360S IMU | `/livox/imu` | `sensor_msgs/Imu`, `livox_frame`; acceleration in g | 200 Hz | R1.6 model (`mid360s.yaml`) | stamp jitter 0.53 ms (the 15% spacing spread TEST-025 measured); 1 ms latency |
| OAK-D Lite IMU | `/oak/imu/data` | `sensor_msgs/Imu`, `oak_imu_frame` | about 239 Hz: 394.5 Hz ticks, 65% of steps skip one (TEST-025) | R1.6 model (`oakd_lite.yaml`) | device stamps; 4 ms latency |
| TFmini-S | `/tfmini/range` | `sensor_msgs/Range`, `tfmini_link`; infrared, 0.1 to 12 m, 2° | 100 Hz | 3 cm sigma to 6 m, 0.5% of the range beyond; 1 cm steps; +inf beyond 12 m, -inf under 0.1 m | stamped on arrival; 5 ms latency; 0.2% lost |
| Mid-360S lidar | `/livox/lidar` | `livox_ros_driver2/CustomMsg` (xfer_format 1), `livox_frame`, lidar_id 192 | 10 Hz frames of 208 packets of 96 points (19,968), 4.947 µs apart, lines 0 to 3 | Livox's measured scan pattern; 40 m limit (no return: 0, 0, 0, reflectivity 0); 2 cm range and 0.15° angle noise | stamp = timebase = the first point; `offset_time` per point; out 5 ms after the frame ends, so a bag gets it about 109 ms after the stamp, as our unit's |
| OAK-D Lite cameras | `/oak/left/image_raw`, `/oak/right/image_raw`, each with its `camera_info` | `sensor_msgs/Image` mono8, 640 x 480, `oak_left_camera_optical_frame`, `oak_right_camera_optical_frame` | 30 Hz, a synced pair | 73° pinhole, no distortion (raw is already rectified); 7.5 cm baseline in the right camera's projection; auto-exposure, 8-bit sRGB; read noise 2 grey levels | device stamps, the same on both images; 25 ms latency; 0.2% of pairs lost |

```bash
python3 tools/check_sim_sensors.py                  # the payload and config/sim/sensors.yaml
python3 tools/check_sim_sensors.py --run /tmp/gdn_sim   # judge a run against the real sensors
```

- **How it works.** The payload's Gazebo sensors are error-free, at the physics rate
  (IMUs 250 Hz). `sim_sensors`:
  - **puts each IMU on its own clock**, interpolating between Gazebo's samples: an even
    200 Hz, or the OAK-D Lite's tick clock with skipped ticks;
  - **adds each IMU's R1.6 errors** with `gdn::ImuErrorSim`, the code the Monte Carlo
    uses (white noise, bias drift, a turn-on bias per run);
  - **converts units and frames** as the driver does (the Mid-360S in g);
  - **for every message,** draws a latency, a stamp (the sample time plus jitter, or
    the arrival time) and whether it's lost, and publishes it when it would arrive.
  - **for the cameras (R2.4b),** pairs the left and right images Gazebo rendered at the
    same moment, gives the pair one stamp, one latency and one chance of being lost (the
    real pair is synced in hardware), and makes each image what the camera sends:
    - Gazebo's grey cameras give linear light, which in 8 bits looks about three times
      too dark (a mean of 20 where the colour camera shows 70). So they render 16-bit,
      and `sim_sensors` applies one auto-exposure gain to the pair (mean to mid-grey,
      0.2 s to settle), encodes sRGB to 8 bits and adds the read noise;
    - the `camera_info` is the distortion-free pinhole of the 73° field of view
      (fx 432.46, centre 319.5, 239.5), the right one with `Tx = -fx * 0.075`, as a
      rectified ROS pair.
- **All publishers are reliable,** as the real drivers' are (`h30wp_node.py` publishes
  `/imu/data` reliable). A reliable publisher serves best-effort subscribers too, but a
  reliable subscriber (VINS and OpenVINS subscribe that way) gets nothing from a
  best-effort one; R2.4a's IMUs and TFmini-S were best-effort.
- **Images aren't recorded by default** (about 18 MB/s, 3 GB for a short flight): the
  bag has both `camera_info` topics, which carry every image's stamp, and
  `GDN_BAG_IMAGES=1` adds the images.
- **The Mid-360S lidar (R2.4c)** follows a 69 s recording of our unit
  (`docs/test_reports/test037_data/mid360s_real.json`):
  - Gazebo renders a dense grid of rays (720 x 120, 0.5°, over the Mid-360S's 360° by
    -7.2 to 52.2°) 20 times a second, error-free;
  - `sim_sensors` fires Livox's measured Mid-360 pattern into it
    (`data/livox/mid360.csv`, Livox-SDK/livox_laser_simulation, MIT, fetched and
    checksum-pinned by `tools/fetch_livox_pattern.sh`, mirrored in azimuth to sweep as
    our unit does): each point takes the nearest ray, its range interpolated between
    the renders around the point's own time, so the cloud's motion matches its
    `offset_time` as LIO's deskewing assumes;
  - the message is the driver's `CustomMsg`. Its package comes messages-only from
    `docker/livox_msgs` (the driver's own `.msg` files), built into the dev image and in
    CI, and kept out of `src/` so it can't shadow the real driver on the vehicle. On a
    PC with the real driver, `env.sh` sources `~/ws_livox` instead;
  - `/livox/lidar` is recorded (about 4 MB/s).
- **Use a site for the cameras** (`GDN_SITE=...`): the default Zermatt world has no
  photo on its terrain, so the cameras see plain grey shading there.
- **Which numbers are measured.** The IMU rates, the Mid-360S's stamp jitter and the
  OAK-D Lite's skip pattern come from the bench logs (TEST-024, TEST-025), the IMU
  errors from R1.6. Every latency, and the TFmini-S's noise and dropouts (from its
  datasheet), are placeholders until Phase 6 measures them on the vehicle.
- **The clock (changed in R2.4b).** Every stamp is Gazebo's time plus one offset,
  fixed when the first raw message comes in and logged in `sensors.log` ("clock: stamps
  are Gazebo time + ... s"). Latencies run on Gazebo's clock too: a message goes out
  once Gazebo's time passes its arrival. So stamps start near wall time and keep
  simulated seconds, the same for every sensor.
  - **Why it changed:** R2.4a followed the wall clock (the bridge's delay floor). With
    the cameras rendering, this laptop runs the simulation at about 0.92 of real time,
    and every stamp interval then stretched by 1 / 0.92: the IMUs read 183 Hz instead
    of 200, so each sample's interval was 9% longer than the motion it carries.
  - **What it means for a subscriber:** in a run slower than real time, stamps fall
    behind `now()` by the time lost. Use the stamps, not the receive time, as a real
    driver's stamps would be used. `check_sim_sensors.py` judges timing in Gazebo
    seconds, turning the bag's receive times into Gazebo time through the truth's.
- **Mounts are placeholders.** The vehicle profiles (R2.5a, `config/vehicles/`) hold
  them, and `tools/check_profiles.py` checks the payload against them; R6.4 measures the
  real ones:
  - the H30WP level at the payload origin;
  - the Mid-360S upside down (so the lidar's -7 to 52° field of view looks at the
    ground);
  - the OAK-D Lite forward, pitched 30° down, its two cameras 3.75 cm either side of its
    IMU (left at +y), pointing the same way;
  - the TFmini-S pointing down.
- **Not simulated yet:**
  - the H30WP's own orientation estimate (the `orientation` field is marked empty, as
    for a driver without one);
  - its magnetometer and temperature (`/imu/mag`, `/imu/temperature`);
  - the TFmini-S's shorter range on dark ground (about 7 m): Gazebo has no reflectivity;
  - the cameras' lens distortion, motion blur, vignetting and the sensor's own response
    curve (the sRGB curve stands in for it); the colour camera (IMX214); the OAK-D's
    own depth, rectified-pair and feature-tracker topics;
  - the lidar's reflectivity (Gazebo has none: one value for every return), its range
    depending on it (40 m for all), multiple returns and the tag bits, and the dust or
    rain a real Mid-360S sees.
- **`check_sim_sensors.py --run`** judges each sensor against the real one: topic,
  type, frame, rate, measured spacing spread, gravity in the right units at rest,
  white noise against its model, the configured timing, and the TFmini-S against the
  true range. For the cameras: the `camera_info`, the synced pair, rate and timing, and
  with the images recorded the exposure, the read noise at rest and the stereo
  geometry: 1.5 to 15 m up, the left image shifted by the disparity the truth gives
  must reproduce the right one best at 0.95 to 1.05 times that disparity (TEST-036:
  1.013 to 1.017; a swapped pair gives -1). For the lidar: the real unit's frame,
  lidar_id, rate, points a frame, point clock, lines, no-return encoding, scan-pattern
  fingerprint and arrival time, then every return against the world's terrain from the
  true pose at the point's own `offset_time`. Timing is judged on Gazebo's clock.

## Scenarios and the runner (R2.6)

A scenario is one SITL flight described in a file, `config/scenarios/<name>.yaml`
(format: `tools/scenario.py`):
- **What it names:** vehicle and world; the route (legs, leg length, spacing, height
  over the terrain, speed, launch); GNSS (none, or a cut at a time into the route); a
  start error for GDN; a steady wind; and a time limit.
- **Checking:** every key is checked, and an unknown or missing one is an error.
  `python3 tools/scenario.py --list` lists them, and `--check` reads them all, as CI
  does.

```bash
tools/docker_dev.sh run --gpu -- tools/run_scenario.py config/scenarios/x500_emmental_gnss_cut.yaml
tools/docker_dev.sh run --gpu -- tools/run_scenario.py config/scenarios/*.yaml       # one after another
```

`tools/run_scenario.py` is the SITL counterpart of `tools/bench.sh`:
1. **The run:** `tools/sim_run.sh` with the scenario's settings, headless, under its
   time limit (Ctrl+C, then kill), in `log/scenarios/<name>/<UTC time>/` with a copy of
   the scenario.
2. **Teardown:** nothing of the simulation may be left running; anything found is
   stopped, and the run fails.
3. **The recording:** the bag and GDN's run log are written, and the run log replays bit
   for bit.
4. **The verdict:** `tools/check_scenario.py` judges whether the run did what was asked:
   - the vehicle and world;
   - the route's length and height;
   - GNSS lost when asked, and not before;
   - GDN's start error at the asked distance and bearing;
   - the vehicle leaning or crabbing into the wind.

   `result.json` has every step; the exit code says whether all passed.

How each condition is made:
- **GNSS cut:** `GPS_OFF_AT` (Stage B: PX4's `EKF2_GPS_CTRL` 0). PX4 gives up its
  position about 4 s later and its fail-safe lands it.
- **Start error:** `GDN_START_ERROR="north east"` (node parameters
  `start_error_north_m`, `start_error_east_m`).
  - **When:** once its heading is aligned, GDN's position moves by that much, and it's
    told sigma = max(distance, 10 m), as `mc_campaign --start-error` does.
  - **Replay:** the run log carries it, so a replay repeats it.
- **Wind:** `GDN_WIND="speed from_deg"` (`tools/sim_wind.py`), in the run's copies only:
  - **The world:** gets the wind, which the cessna's and the VTOL's wings feel through
    Gazebo's lift-drag model: the VTOL crabbed 19 deg in 6 m/s (TEST-040).
  - **The x500:** has no aero model, so Gazebo's wind effects push its body (a linear
    drag, `mass x 0.1/s x (wind - velocity)`): it leaned 4.4 deg into 6 m/s.
  - **The VTOL's hover phases** feel no wind this way.

### Spoofing, sensor faults and lighting (R2.6b)

The scenario file can also name these three (format: `tools/scenario.py`):

```yaml
gnss: {loss: spoof, at_s: 60, rate_mps: 0.5, bearing_deg: 90}
faults:
  - {sensor: agl, kind: dropout, at_s: 30, duration_s: 20}
  - {sensor: mag, kind: noise, at_s: 120, duration_s: 20, sigma: 20}
lighting: {sun_elevation_deg: 8, sun_azimuth_deg: 250, intensity: 0.3}
```

- **GNSS spoofing:** `GDN_SPOOF="at_s rate_mps bearing_deg"` (`tools/sim_spoof.py`).
  - **The relay:** PX4's GPS comes from a relay. It reads Gazebo's own GPS, the same
    sensor PX4's `gz_bridge` reads, and sends it to PX4 as MAVLink `HIL_GPS`.
  - **PX4's settings, at its boot:** `MAV_USEHILGPS` 1 takes `HIL_GPS` outside HIL
    mode, and `SIM_GPS_USED` 0 gives PX4's own Gazebo GPS no fix. PX4 picks the receiver
    with the best fix. Both are set on every run, both ways, since PX4 keeps its
    parameters between runs.
  - **The spoof:** from `at_s` into the route, the reported position walks off at
    `rate_mps` towards `bearing_deg`. The reported velocity carries the same drift, so
    the two agree. PX4 steers its spoofed position along the route, and the vehicle
    drifts the other way.
  - **GDN** uses no GNSS, so it isn't fooled.
  - **Why `HIL_GPS`:** PX4 1.16 has no handler for MAVLink `GPS_INPUT` (only
    `HIL_GPS`, and that one only from its own system id).
- **Sensor faults on GDN's inputs:** `GDN_FAULTS="sensor:kind:at_s:duration_s[:sigma[:sigma_gyro]];..."`
  (`tools/sim_faults.py`).
  - **Where:** the adapter applies them to what it publishes for GDN: the altimeter,
    baro and mag.
  - **The IMU:** an IMU fault relays PX4's `/fmu/out/sensor_combined` through the
    adapter (`/gdn/sim/imu_relay`), and `eskf_node` is pointed at the relay.
  - **The kinds:** dropout (nothing), stuck (the last value, repeated), noise (Gaussian,
    in the sensor's units).
  - **PX4 is untouched,** since it flies on its own sensors.
  - **The record:** each fault's start and end goes out on `/gdn/sim/fault`.
- **Lighting:** `GDN_LIGHTING="elevation azimuth intensity"` (`tools/sim_light.py`).
  - **In the run's world:** the sun's direction, and its light, the ambient and the
    sky scaled by the intensity.
  - **The camera:** the adapter reports the light the OAK-D Lite's left camera sees:
    `/gdn/sim/scene_light`, the mean of Gazebo's raw linear image. It also reports the
    image's brightness after the simulated auto-exposure, `/gdn/sim/image_brightness`,
    which stays near mid-grey (TEST-041: dusk at 0.3 of the light let 7% of the light
    through, and the image went from 108 to 95 of 255).
  - **GDN** uses no camera until Phase 3.

All three count their times from the route's start (`/gdn/sim/route_start`, from
`sim_fly.py`), as the GNSS cut does. `tools/check_scenario.py` checks each one:
- the spoof: PX4's position walked off the truth as asked;
- the faults: each one is in GDN's own record of its inputs;
- the lighting: the world's sun, and the images darker.

### Scoring a run (R2.7a)

`tools/run_scenario.py` scores every run once it's checked: `tools/score_run.py` writes
`metrics.json`, `report.md` and `report.png` into the run directory. A score is a
measurement, not part of the scenario's verdict.

```bash
tools/docker_dev.sh run -- python3 tools/score_run.py log/scenarios/x500_emmental_smoke/<UTC>/   # again, or an older run
```

- **The clock:** GNSS-free time counts from the route's start, as the scenario's cuts,
  spoofing and faults do. GDN uses no GNSS in SITL, so the whole flight is GNSS-free
  to it. The windows are the first 5 and 10 minutes, cut at the landing.
- **Accuracy:** CEP50, CEP95 and max against the vehicle class's REQ-NAV limits, which
  are read from `docs/requirements.md`: x500 REQ-NAV-001, VTOL REQ-NAV-002, cessna
  REQ-NAV-003. Also the altitude p95 (REQ-NAV-010).
- **Consistency:** the share of samples over GDN's own 3 sigma (REQ-INT-001), the 2-D
  NEES, and the NIS per aiding sensor from GDN's update records.
- **Time to acquire:** until the error stays under the 5-minute max limit for 30 s.
  REQ-ACQ-001 is judged when the scenario asks a start error.
- **CPU and memory:** `eskf_node`'s, sampled by the runner once a second (`cpu.csv`).
- **Latency, per IMU sample, from GDN's run log:** its processing time, and the time
  from PX4's sample to GDN's output.
- **Cross-check:** the whole-run numbers sit beside the run's own `replay_scorer`.

- **Integrity (REQ-INT-002, 003), a proxy until R4.4 gives GDN a status:** GDN "flags"
  while 3 x its sigma is over the class's 5-minute max limit.
  - **A lost-lock case:** the error passes 3 x that limit. It's detected if GDN is
    flagged within 30 s.
  - **A false alarm:** a flagged spell (gaps under 10 s merged) in which the error never
    passes the limit. Counted on nominal flights: no sensor faults and no start error.
    GDN uses no GNSS, so cuts and spoofing are nominal to it.

### Sets of runs and the baseline (R2.7b)

One run gives one run's CEP95 and max; the requirements pool a set of runs.
`tools/score_set.py` pools runs per scenario from each run's `metrics.json` and
`series.npz` (no ROS needed), and compares them with the accepted baseline,
`config/baselines/sitl.json`.

```bash
tools/docker_dev.sh run --gpu -- tools/run_scenario.py config/scenarios/*.yaml --runs 3 --out log/set   # scores the set at the end
python3 tools/score_set.py log/set --out log/set           # again: set.md, set.json, set.png
python3 tools/score_set.py log/set --accept --note "why"   # make it the baseline, then commit it in a PR
```

- **Pooled as the requirements count:**
  - CEP95 over every sample of the set's flights;
  - the max rule as the share of flights never over the limit;
  - over 3 sigma over every sample;
  - the integrity rates over every scenario.

  A few runs can't show a 99% rule. The report says how many flights each figure stands on.
- **The comparison:** the set's median per metric against the baseline's runs.
  - **Gated:** `eskf_node`'s CPU, memory and processing time, and the error at 120 s from
    a start error. WORSE past the worst baseline run by more than the tolerance gives
    exit code 1.
  - **Reported, not gated:** the navigation, consistency and integrity metrics. The report
    says whether the set is above all the baseline's runs, below all of them or among
    them, and how many times the worst.
  - **Why not gated:** while GDN's drift is unbounded (OI-012), one scenario's runs span
    up to 33 x (TEST-043). A few runs can't tell a regression from that. The report's
    leave-one-out spread table shows how far an unchanged GDN lands. The navigation
    regression guard stays the Monte Carlo in CI, which is repeatable.
- **Accepting:** `--accept` writes the set's scenarios into the baseline and keeps the
  others. It refuses a run that failed its scenario check, fewer than three runs of a
  scenario (`--min-runs`), or runs of different scenario files. The baseline keeps each
  scenario file's hash, so runs of a changed scenario aren't compared.

**Fly timed batches on mains power.** On a laptop on battery in power-saver mode,
Gazebo ran at 0.86 of real time (TEST-042). Even on mains power with the performance
profile, 9 of TEST-043's 30 runs ran at 0.93 to 0.955 (cause not known, no visible
effect on GDN's error). Each run's report states its real-time factor.
- **The effect:** PX4's clock, which is Gazebo's, kept falling out of sync with the PC's
  ("time jump detected" in `px4.log`). GDN dropped IMU samples, and the timing figures
  measured the drift.
- **Before a batch:** `powerprofilesctl get` should say `performance` (or `balanced`),
  on the charger.
- **The check:** each run's `report.md` gives its real-time factor.

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
  - The altimeter readings looked good: 0.32 m RMS against the map at the true position.
    *Correction (TEST-034): that was the landing over the level pad. In cruise GDN's
    altimeter input is 6 to 9.5 m RMS off the height straight below, because the x500
    flies tilted and the tilt removal assumes flat ground (OI-016). Whether that's part
    of the runaway isn't tested yet.*
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
- **Arming.** The x500 airframe refuses to arm without a ground station link, and
  the VTOL and cessna airframes fail safe without one. `sim_run.sh` sets
  `NAV_DLL_ACT 0`, so QGroundControl isn't needed. Opening it anyway is fine.
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
rendered terrain rather than the physics one. It puts a probe carrying the GDN
payload's radar 100 m above the map at 26 points and compares the ranges:

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
- **The default Zermatt world has no photo on its terrain;** the R2.2 sites have the
  2 m orthophoto, which the cameras see as a soft image: 2 m per photo pixel against
  about 0.2 to 0.5 m per camera pixel at 100 to 200 m. Finer photos for VIO are R3.5's.
- **GDN doesn't read the R2.4 sensors yet** (the three IMUs, the TFmini-S, the cameras and
  the lidar); LIO and VIO are Phases 3 and 4.
- **The Mid-360S sees little from cruise height:** its 40 m limit on dark ground means
  that above about 40 m most of its points come back empty, as on the real unit.
- **GDN itself is still tuned for the multirotor.** Each airframe has its own profile
  (R2.5a), but all three name the same filter settings and aiding for now: R2.5b adds
  the flight regime's rules, and tuning per airframe waits for SITL and Monte Carlo
  evidence.
- **GDN's altimeter input assumes flat ground under a tilted vehicle** (OI-016): metres
  off over slopes, on every vehicle.
- **The runway is a graded strip,** not a surveyed airfield: up to 12.6 m of ground moved
  (Zurich, through buildings), and the photo under it still shows what was there.
