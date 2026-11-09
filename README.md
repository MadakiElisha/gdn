# GDN: GNSS-Denied Navigation

GDN is a navigation stack for drones that have to keep flying when GNSS is jammed,
spoofed or simply not there. The goal is one estimator that works across multirotors,
quadplanes and conventional fixed-wing aircraft. It should give the autopilot a
position it can fly on, together with an honest statement of how good that position
is.

Navigation without GNSS comes down to two problems. The first is short-term motion:
how far and in which direction the vehicle has moved in the last few seconds. The
second is long-term position: where the vehicle actually is on the map, which you need
because every motion estimate drifts. GDN tackles them in layers:

1. **Inertial core.** An error-state Kalman filter (ESKF) integrates the IMU at high
   rate and keeps track of its own uncertainty.
2. **Relative odometry.** Lidar-inertial odometry (LIO) from a Livox Mid-360S and
   stereo visual-inertial odometry (VIO) from a Luxonis OAK-D Lite measure motion
   between fixes and slow down IMU drift. *Planned, next after the measurement-model
   work.*
3. **Absolute fixes.** Terrain-referenced navigation (TRN) compares the terrain under
   the vehicle with an onboard elevation map to pull the estimate back to the right
   place. Lidar map relocalisation and camera-to-satellite matching are planned for
   places where terrain alone isn't enough.
4. **Integrity.** The filter has to notice when it's lost, say so, and recover.
   *Planned.*

Today the inertial core and TRN work in simulation and replay. The rest is laid out in
[ROADMAP.md](ROADMAP.md).

## Contents

- [Project status](#project-status)
- [How it works](#how-it-works)
- [Vehicle-agnostic design](#vehicle-agnostic-design)
- [Reference hardware](#reference-hardware)
- [How we got here](#how-we-got-here)
- [Requirements](#requirements)
- [Repository layout](#repository-layout)
- [Tools](#tools)
- [Data](#data)
- [Getting started](#getting-started)
- [Running things](#running-things)
- [How we validate](#how-we-validate)
- [Validation so far](#validation-so-far)
- [Known limitations](#known-limitations)
- [Open items](#open-items)
- [Contributing](#contributing)
- [Documentation map](#documentation-map)
- [References](#references)

## Project status

GDN is a research prototype. It navigates well in simulation under friendly
assumptions, and it has not flown on real hardware yet.

| Part | State | Notes |
|---|---|---|
| ESKF core (19 states) | Working in sim | Position, velocity, attitude, accel/gyro bias, baro bias, mag bias. ROS-free C++17. |
| TRN with a real altimeter model | Working in sim | Height above ground from a rangefinder or radar, plus baro with a bias state (R1.1, TEST-017). The old idealised model is kept as `trn_model:=ideal` for comparison. |
| Barometer, magnetometer, ZUPT | Working in sim | Mag gives heading at start-up; ZUPT pins the filter on the ground. |
| Terrain database tools | Working | Synthetic terrain and real SRTM (Zermatt). The maps rebuild byte for byte. |
| ROS 2 node (PX4 topics) | Working on replay | Jazzy, `px4_msgs` 1.16, deterministic replay bench. |
| Unit tests and Monte Carlo | Working | 31 checks at `-O2` and `-O3`; seeded MC campaigns. |
| Gazebo world from a real DEM | Working (Zermatt) | `tools/make_sim_world.py` builds the world from the onboard map; a physics drop test checks it against the map. No photo texture yet. |
| Closed-loop SITL with GDN in the loop | First version (Stage A/B), estimator drifts | PX4 v1.16 flies an x500 over Zermatt in Gazebo Harmonic, GDN runs in shadow mode with a simulated radar altimeter, RViz shows GDN against truth. See `docs/SIMULATION.md`. On this multirotor GDN drifts to kilometres within minutes (OI-012); the fixed-wing replay is fine. Still to come: simulated Mid-360S, TFmini-S and OAK-D Lite, PX4 flying on GDN, scenario runner. |
| LIO odometry (Mid-360S) | Planned | FAST-LIO2 or Point-LIO feeding GDN as a velocity / delta-pose measurement. |
| Stereo VIO (OAK-D Lite) | Planned | Global-shutter stereo pair plus IMU, through the same odometry interface as LIO. |
| TRN acquisition from a large position error | Planned | Point-mass or particle filter that hands over to the ESKF. |
| Integrity monitoring | Planned | Innovation tests, lost-lock detection, quality flag. |
| Vehicle profiles and flight-regime switching | Planned | Hover, transition and cruise behaviour from one config. |
| Output to PX4 / ArduPilot | Planned | External vision / auxiliary global position, or ExternalNav. |

## How it works

```
                 sensors                              GDN                         autopilot
  ---------------------------------   -----------------------------------   ----------------
  IMU (high rate) ------------------> ESKF propagation
  baro, mag ------------------------> ESKF updates
  LIO (lidar) / VIO (stereo camera)-> velocity / delta-pose updates (planned)
  rangefinder / lidar AGL ----------> TRN: terrain height vs onboard map ---> position, velocity,
  lidar scans, camera images -------> map relocalisation / geo-registration   attitude + covariance
                                      integrity monitor (planned) ---------> quality / lost flag
```

All of the navigation maths sits in a small header-only core with no ROS dependency
(`src/gdn_fusion/include/gdn/`). The same code is compiled into the unit tests, the
Monte Carlo driver and the ROS 2 node, so a result in one is a result in all three.
The ROS node is deliberately thin. It subscribes to PX4 topics and the GDN measurement
topics, calls the core, and publishes odometry.

### The filter

It's an error-state Kalman filter following Solà's quaternion formulation. The nominal
state holds position, velocity and attitude in a local NED frame, plus sensor biases.
The filter estimates a 19-dimensional error state around it. The IMU drives
propagation with sub-stepping at 10 ms or less. Updates arrive as they become
available:

- **Static alignment** sets roll, pitch and the gyro and accel biases while the vehicle
  sits still.
- **ZUPT** (zero-velocity update) pins velocity on the ground until take-off.
- **Magnetometer** gives a one-shot heading, then continuous attitude and mag-bias
  updates.
- **TRN** is a scalar update: measured terrain height against map height, with the map
  gradient as the measurement Jacobian. It is gated on slope (flat ground tells you
  nothing) and on innovation size.
- **Barometer** observes altitude plus a slowly varying bias.

Updates use the Joseph form, the covariance is repaired whenever it loses positive
definiteness, and the attitude error is reset after every injection. The full
reference, with state indices and every parameter, is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

### Interfaces

The node (`eskf_node`) talks to the rest of the system over these topics today.

| Direction | Topic | Type | What it carries |
|---|---|---|---|
| in | `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | IMU, drives propagation |
| in | `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | GNSS-derived; still used for ZUPT and take-off detection (to be removed, R1.2) |
| in | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | truth heading, logging only |
| in | `/gdn/agl_meas` | `std_msgs/Float64` | height above ground, tilt-corrected, m (default `trn_model:=agl`) |
| in | `/gdn/trn_meas` | `std_msgs/Float64` | terrain height directly, m (legacy `trn_model:=ideal` only) |
| in | `/gdn/baro_meas` | `std_msgs/Float64` | baro altitude (NED z), m |
| in | `/gdn/mag_meas` | `geometry_msgs/Vector3` | magnetometer, uT, body frame |
| out | `/gdn/odom` | `nav_msgs/Odometry` | fused position, velocity, attitude |
| out | `/gdn/sigma` | `std_msgs/Float64` | horizontal 1-sigma, m |

Parameters (`map_path`, `r_trn`, gates, clamps) default to the values in `EskfConfig`,
so the node, the unit tests and the Monte Carlo always run the same configuration.

### Terrain database

Maps are a simple binary grid: a 40-byte header, then row-major `float32` heights. The
local NED origin is the grid centre. Queries return bilinear height and gradient. Two
maps ship with the repo:
- A synthetic fractal map centred on the PX4 SITL default location (Zurich), smoothed
  so scalar TRN behaves (REQ-DB-001).
- A 1 arc-second SRTM crop around Zermatt for testing on real relief.

`tools/fetch_terrain.py` pulls SRTM for any area, and `make_zermatt_map.py` shows how to
turn a GeoTIFF into a GDN map.

### Where LIO and VIO fit

Between terrain fixes the filter otherwise relies on the IMU alone, and a MEMS IMU
drifts by kilometres within minutes (TEST-002 measured 245 km over 17 minutes
unaided). The odometry layer fills that gap. GDN will accept velocity or delta-pose
from an external odometry source, with its covariance and a health flag, through one
interface (`ApplyOdom()`, roadmap R3.2).

- **LIO** (FAST-LIO2 or Point-LIO on the Mid-360S) works in the dark and in low
  texture. It loses lock above the lidar's useful range (about 30 m above ground when
  pointed down), though. It also slides sideways over flat ground with nothing to
  anchor it horizontally.
- **Stereo VIO on the OAK-D Lite** (OpenVINS, VINS-Fusion, or cuVSLAM on the Orin)
  uses the two global-shutter OV7251 cameras and the onboard IMU. It's good over
  flat textured ground, exactly where LIO is weak. It struggles at night and over
  water. The 7.5 cm stereo baseline only gives useful depth to about 12 m, so higher
  up it behaves like monocular VIO and gets its scale from the IMU. The OAK-D's own
  processor can run feature tracking on-device, which takes load off a Pi 5.

The two fail in different conditions. That is why GDN is the one that weighs them,
rather than trusting either blindly. The OAK-D Lite's 13 MP colour camera is also the
planned source of images for matching against satellite imagery (R8.6).

## Vehicle-agnostic design

The core has no airframe model in it: no rotor, thrust or control-surface dynamics.
What changes between vehicles is which sensors are fitted and trusted, the noise
tuning, and the flight regime. That will live in a vehicle profile (a YAML file) and in
a regime signal taken from the autopilot (ground, hover, transition, cruise).

| | Multirotor | Quadplane / VTOL | Fixed-wing |
|---|---|---|---|
| Typical speed and height | 0 to 20 m/s, low | Both regimes | 15 to 40+ m/s, high |
| Terrain matching | Weak when hovering (no new terrain) | Good in cruise | Strong |
| Best velocity aiding | LIO, VIO, optical flow, rotor drag | LIO/VIO in hover, airspeed and wind in cruise | Airspeed and wind, VIO |
| Height-above-ground sensor | Lidar, short-range rangefinder | Both, plus long-range altimeter | Radar or long-range laser altimeter |
| Start-up | Static on the ground | Static | Often launched, so it must align in motion |
| Things to watch | Motor magnetic interference, vibration | Mode transitions; tailsitters rotate the body 90 degrees | Long GNSS-free legs, wind |

All validation so far has been on a fixed-wing trajectory. The simulation
environment (roadmap Phase 2) brings in PX4's multirotor, standard VTOL and
fixed-wing models, so all three airframes get tested in closed loop before any of them
flies.

## Reference hardware

This is the first build we're bringing up. Anything that publishes the same ROS 2
messages should work.

| Role | Part | What it gives GDN |
|---|---|---|
| IMU | Yesense / WHEELTEC H30WP | 9-axis MEMS (gyro, accel, mag), up to 400 Hz over UART/RS485 |
| Lidar | Livox Mid-360S | 360 by 59 degree point cloud, about 40 m on dark surfaces, built-in IMU, PTP time sync; used for LIO and lidar AGL |
| Rangefinder | Benewake TFmini-S | Height above ground from 0.1 to 12 m (about 7 m on dark surfaces), for landing and low hover |
| Stereo camera | Luxonis OAK-D Lite | Global-shutter OV7251 stereo pair (7.5 cm baseline), 13 MP IMX214 colour camera, BMI270 IMU, onboard processor (4 TOPS); used for VIO and later geo-registration |
| Companion | Jetson Orin Nano (JetPack 7.2+) or Raspberry Pi 5 + Hailo HAT | Runs GDN, LIO and VIO. The Orin has more headroom; the Hailo NPU only helps neural-network parts |
| Autopilot | PX4 v1.16 (R2.1) | uXRCE-DDS link, matching the `px4_msgs` submodule. v1.16 is the first release with official Ubuntu 24.04 and Gazebo Harmonic support |

Between them, the lidar and rangefinder cover heights up to about 30 m above ground.
That suits a low-flying multirotor first. Quadplane cruise and fixed-wing need a
long-range altimeter, which isn't in this list (R8.4). Two things to check on your own
units:
- OAK-D Lite cameras from the original Kickstarter batch shipped without the IMU.
- The Mid-360S has its own IMU, so we still have to decide whether LIO uses that one
  or the H30WP (R6.4).

## How we got here

The project grew in phases, each closed by a test report in `docs/test_reports/`. The
phase numbers in those reports are the original development phases; the ROADMAP
numbers its phases separately.

| Step | What we learned | Report |
|---|---|---|
| Unaided INS on the PX4 replay flight | Pure strapdown drifted 245 km in 1049 s. Absolute aiding is mandatory. | TEST-002 |
| Coarse alignment and bias removal | Calibration slows the early drift but can't bound it. | TEST-003 (commit c6c4c54) |
| Batch TERCOM on a loiter | Most windows were ambiguous, and fixes went wrong once drift exceeded the search radius. We moved to recursive point-wise TRN (SITAN style) inside the ESKF. | TEST-005 |
| Recursive ESKF + TRN, Python | Closed-loop lock on the replay flight. | TEST-006 |
| Map resolution sweep | The filter is very sensitive to mismatch between world and onboard map. | TEST-008 |
| C++ node on the replay bench | The C++ node matched the Python prototype (30.4 m RMS at the time). Heading transfer has to be treated as a re-initialisation. | TEST-009/010 |
| Dual-build unit tests | `-O3` exposed uninitialised accumulators that `-O2` hid. Both builds are now a standing gate. | TEST-011 |
| Monte Carlo on the smoothed map | 100/100 seeds locked on the sigma-4 map versus 27/100 on sigma-1. The sigma-4 map became REQ-DB-001. | TEST-012 |
| Barometer and magnetometer states | Vertical channel observable; GNSS-free heading at start-up. | OI-002, OI-003 (TEST-013, TEST-014) |
| ZUPT on the ground | Pins position and velocity before take-off. | TEST-015 (selftest) |
| Real terrain | The filter navigates SRTM relief around Zermatt (CEP50 about 14 m). | OI-005, commits 85e2d52 and 742b5ca |
| Setup rework | A fresh clone builds anywhere. Three bench bugs found and fixed along the way. | PR #1 |
| Re-baseline after the fixes | flight001 MC: CEP50 13.4 m, 100/100. Bench: 31.5 m RMS with baro, 22.5 m without. | TEST-016 |
| Real altimeter model | The oracle hid how much altitude error costs. The real model (AGL plus a baro bias state) is honest and beats the naive approach under every baro drift tested. | TEST-017 |

## Requirements

The full set, with definitions and how each one is checked, is in
[docs/requirements.md](docs/requirements.md). It's a draft: the GNSS-free durations are
fixed, and the error limits are proposals to agree. The first targets are 5 and 10
minutes without GNSS:

| Vehicle | 5 min without GNSS | 10 min without GNSS |
|---|---|---|
| Multirotor, low | CEP95 15 m, max 30 m | CEP95 25 m, max 50 m |
| Quadplane, cruise | CEP95 40 m, max 80 m | CEP95 60 m, max 120 m |
| Fixed-wing, cruise | CEP95 50 m, max 100 m | CEP95 75 m, max 100 m |

On top of these: altitude within 5 m, the reported uncertainty has to be honest
(error over 3-sigma in at most 1% of samples), lost lock is flagged within 30 s, and
acquisition from a 500 m start error takes 120 s or less on hilly terrain. The older
REQ-P-001 (at most 100 m over a mission) and REQ-DB-001 (map resolution and smoothing)
still stand.

## Repository layout

```
.
├── README.md                 this file
├── ROADMAP.md                what's next, phase by phase
├── LICENSE                   MIT
├── env.sh                    source in every terminal (sets GDN_ROOT, ROS, RMW)
├── src/
│   ├── gdn_fusion/           main package
│   │   ├── include/gdn/      ROS-free core: eskf.hpp, map_db.hpp, paths.hpp
│   │   └── src/              eskf_node, map_viz_node, selftest, mc_campaign
│   ├── gdn_sensors/          sensor_monitor: prints IMU and local-position rates
│   └── px4_msgs/             git submodule, PX4 release/1.16
├── tools/                    scripts, simulators and analysis (see Tools)
├── data/                     maps in git; flights generated or downloaded (see Data)
└── docs/
    ├── SETUP.md              install, update, data, troubleshooting
    ├── ARCHITECTURE.md       filter maths, parameters, validation detail
    ├── design/eskf_design.md design record and the literature behind it
    ├── open_items.md         open-item ledger (OI-xxx)
    └── test_reports/         TEST-002 to TEST-012, with plots
```

### C++ programs (`gdn_fusion`)

| Program | What it does |
|---|---|
| `eskf_node` | The ROS 2 node. Also built as a component (`EskfNode`) for composition. |
| `map_viz_node` | Publishes the terrain as an RViz mesh, plus the estimated path. |
| `gdn_selftest` | Unit tests for the core. Run by `tools/run_selftest.sh` and `colcon test`. |
| `mc_campaign` | Monte Carlo driver. Replays IMU and truth CSVs through the core with seeded sensor errors. |

## Tools

Everything in `tools/` is run from the repo root after `source env.sh`.

| Purpose | Tools |
|---|---|
| Setup and data | `setup_ubuntu.sh` (machine setup), `make_data.sh` (derived data and checksum checks), `fetch_data.sh` / `pack_data.sh` (download or publish the flight001 recording) |
| Tests and benches | `run_selftest.sh` (unit tests at `-O2` and `-O3`), `mc_run.sh` (Monte Carlo), `bench.sh` (ROS replay bench with scoring), `bench_viz.sh` (the same with RViz) |
| Sensor simulators | `trn_sensor_sim.py` (terrain height under the vehicle), `baro_sim.py`, `mag_sim.py`; all fed by PX4 truth from the bag |
| Scoring and plots | `replay_scorer.py` (horizontal error against truth), `plot_zermatt_track.py` (track over hillshade, error and innovations), `ci_check_mc.py` (fails CI if a Monte Carlo report is worse than its thresholds) |
| Terrain maps | `generate_terrain.py` then `export_map_db.py` then `make_sigma4_map.py` (synthetic map); `fetch_terrain.py` and `make_zermatt_map.py` (SRTM); `export_map_bin.py`, `terrain_map.py` (older helpers) |
| Synthetic flights | `gen_zermatt_flight.py` (IMU and truth CSVs over Zermatt), `make_zermatt_bag.py` (the same as a PX4-style bag) |
| Closed-loop SITL | `setup_sim.sh` (PX4 v1.16, Gazebo, bridge, agent), `make_sim_world.py` (world from the map), `check_sim_world.py` (physics drop test), `sim_run.sh` (whole loop), `sim_adapter.py` (Gazebo sensors to GDN inputs), `sim_fly.py` (offboard route, GPS cut) |
| Older Gazebo world tools | `make_world_final.py` (heightmap world), `make_terrain_mesh.py` (mesh visual) |
| Bag handling | `migrate_bag_px4_msgs.py` and `migrate_flight001.sh` (convert a px4_msgs 1.15 bag to 1.16), `export_bag.py`, `export_flight_csv.py` (bag to CSV), `bag_qos.yaml` (QoS overrides for recording PX4 topics) |
| Python prototypes and analysis | `analyze_ins.py` (INS drift, TEST-002/003), `trn_match.py` (batch TERCOM, TEST-005), `eskf_trn.py` and `eskf_mismatch.py` (Python ESKF, TEST-006/008), `py_trace.py` (step-by-step comparison with the C++ core) |
| Shared helpers | `gdn_paths.py` (repo root and map reader for the Python tools), `gdn.rviz` (RViz layout) |

## Data

| Dataset | What it is | How you get it |
|---|---|---|
| `flight001` | PX4 SITL fixed-wing (Cessna) flight, 1049 s: IMU, attitude, local and global position, GPS, status. MCAP bag plus exported CSVs. | `tools/fetch_data.sh` (about 100 MB, GitHub release) |
| `terrain_db_sigma4.bin` | Synthetic 400 by 400 map, about 28 m cells, smoothed to sigma 4 | in git |
| `zermatt.bin`, `zermatt.tif` | 129 by 129 SRTM crop and its source GeoTIFF | in git |
| `flight_zermatt_csv/`, `flight_zermatt/` | Synthetic lawnmower flight over Zermatt, as CSVs and as a PX4-style bag | `tools/make_data.sh`, `python3 tools/make_zermatt_bag.py` |

`data/SHA256SUMS` pins every tracked or deterministic file. Flight logs from the real
hardware (roadmap R6.6) will go into releases or shared storage, not into git.

## Getting started

You need Ubuntu 24.04 (desktop, Pi 5 or Jetson on JetPack 7.2+), about 10 GB of disk
and sudo.

```bash
git clone --recurse-submodules https://github.com/MadakiElisha/gdn.git ~/gdn_workspace
cd ~/gdn_workspace
tools/setup_ubuntu.sh
source env.sh
```

Updating an older copy, doing the setup by hand, data downloads and troubleshooting
are all in [docs/SETUP.md](docs/SETUP.md).

## Running things

Source `env.sh` first in every terminal.

| Task | Command |
|---|---|
| Unit tests (no ROS needed) | `tools/run_selftest.sh` |
| Monte Carlo on the PX4 replay flight | `tools/mc_run.sh 100` |
| Monte Carlo on Zermatt | `tools/mc_run.sh 24 data/flight_zermatt_csv data/maps/zermatt.bin` |
| Monte Carlo with options | add `--model=agl\|naive\|ideal`, `--baro-drift=15`, `--window=60,600`, `--q-baro=`, `--r-agl=` |
| Replay bench on the PX4 flight | `tools/bench.sh` (add `BENCH_NO_BARO=1` for the baro-off run) |
| Replay bench on any bag and map | `GDN_BAG=... GDN_MAP=... tools/bench.sh` |
| Bench with RViz | `tools/bench_viz.sh` |
| Just the node | `ros2 run gdn_fusion eskf_node --ros-args -p map_path:=data/maps/zermatt.bin` |
| Check sensor rates from PX4 | `ros2 run gdn_sensors sensor_monitor` |

## How we validate

Nothing goes on a vehicle until it has passed in simulation. The simulation is
layered, and each layer catches different problems:

| Layer | What runs | Catches | Today |
|---|---|---|---|
| Unit tests | Core maths only | Equation and bookkeeping bugs | Working |
| Monte Carlo | Core on IMU and truth CSVs with seeded errors | Statistical robustness, filter consistency | Working |
| Replay bench | ROS node on recorded bags | ROS wiring, timing, message handling | Working |
| Closed-loop SITL | PX4 + Gazebo + simulated sensors + GDN, with the autopilot flying | Whole-system behaviour on all three airframes | First version, multirotor only (`docs/SIMULATION.md`) |
| Processor-in-the-loop | The same SITL, with GDN on the Orin or Pi 5 | Compute budget, latency, network timing | Planned (Phase 5) |

Scenarios (vehicle, terrain, height, when and how GNSS is lost, start error, wind,
sensor faults) are scored automatically against per-vehicle requirements. A
qualification run of the full scenario matrix at the end of roadmap Phase 5 is the
gate for hardware. Real flight logs then get replayed through the same scoring, to
check that the simulated sensors aren't flattering us.

## Validation so far

| Test | Result | Conditions |
|---|---|---|
| Unit tests | 31 / 31 at `-O2` and `-O3` | Map maths, alignment, TRN, AGL, baro, mag, ZUPT |
| Monte Carlo, Zermatt SRTM | CEP50 14.2 m, 19 of 24 seeds under 100 m | Synthetic flight, known start, map used as truth |
| ROS replay bench, Zermatt | 13 to 20 m RMS | Synthetic PX4-style bag, baro off |
| Monte Carlo, PX4 replay flight, real altimeter model | CEP50 26.4 m, CEP95 60.1 m, 100 of 100 seeds; altitude p95 2.9 m | 100 seeds, no baro drift (TEST-017) |
| Same, with 15 m baro drift over 5 min | CEP50 50.8 m, 97 of 100 seeds; altitude p95 9.9 m | TEST-017 |
| ROS replay bench, PX4 replay flight | 21.3 m RMS (max 35.6 m) with baro; 31.5 m with the old oracle | TEST-016, TEST-017 |
| Old oracle model, for reference | MC CEP50 13.4 m, 100/100 | TEST-016; optimistic, because altitude was assumed perfect |

The Zermatt failures are false-contour locks (OI-009).

## Known limitations

These are the honest gaps between this and something you could fly. They're the top of
the roadmap.

- **Terrain matching is limited by altitude accuracy.** With the real altimeter
  model (R1.1), each metre of altitude error looks like several metres of horizontal
  error on gentle slopes. On the recorded flight, CEP50 is 26 m, against 13 m for the
  old oracle that hid this. A 15 m baro drift over 5 minutes now degrades rather than
  diverges (97 of 100 seeds under 100 m), but it still costs accuracy.
- **The filter starts where the vehicle really is.** There's no acquisition mode, and
  start errors above about 60 m fail (TEST-023: half or more fail from 150 m, nearly all from 500 m).
- **The Monte Carlo uses one map as both world and onboard database,** so map error
  isn't modelled.
- **The node still reads GNSS-derived data** for ZUPT, take-off detection and the
  initial altitude.
- **There's no integrity monitoring.** A filter that has lost lock can still report a
  small uncertainty.
- **Some covariance bookkeeping is inconsistent.** The TRN update skips some state
  corrections but still shrinks their uncertainty, and the clamps do the same.
- **Late-flight and landing phases** have poor terrain signature on the replay flight,
  and error grows there (OI-004).

## Open items

The full ledger is [docs/open_items.md](docs/open_items.md). Status as of November
2026:

| ID | Item | Status |
|---|---|---|
| OI-002 | Vertical channel unobservable without a baro state | Done (16th state, then 19-state core) |
| OI-003 | Heading came from GNSS velocity | Done (magnetometer alignment and bias states) |
| OI-004 | Error growth late in the flight and near landing | Accepted limitation for now |
| OI-005 | Untested on fine or real terrain | Done (Zermatt SRTM) |
| OI-006 | Attitude/velocity oscillation under TRN updates | Done (velocity injection clamp) |
| OI-007 | Gazebo heightmap visuals broken under WSL2 software rendering | Deferred |
| OI-009 | False-contour lock in some Zermatt seeds | Open; fixed by acquisition (R4.1) |
| OI-010 | Bench left stray nodes between runs | Done |
| OI-011 | Earlier Zermatt map was mirrored north-south | Done (map regenerated from GeoTIFF) |

## Contributing

We work in small, checkable steps: state a hypothesis, run one decisive experiment,
decide on the pass rule before looking at the result, then commit. In practice:

- Branch from `main` and open a pull request. Don't push to `main` directly.
- CI (`.github/workflows/ci.yml`) runs on every PR: the build, the unit tests at `-O2`
  and `-O3`, `colcon test`, the map checks and a 24-seed Zermatt Monte Carlo with
  regression thresholds. It has to be green before merging. Changes to the filter core
  also need the Monte Carlo and bench numbers before and after, in the PR description.
- New behaviour comes with a test, and ideally one that fails on the old code.
- Keep the core ROS-free. Anything ROS-specific belongs in the node.
- Record open questions in `docs/open_items.md` with an OI number, and test write-ups
  in `docs/test_reports/`.
- Pick work from [ROADMAP.md](ROADMAP.md). Each item has an ID; put it in your branch
  name and PR title.

## Documentation map

| Document | Read it when you want to |
|---|---|
| [README.md](README.md) | understand what GDN is, where it stands, and how the pieces fit |
| [ROADMAP.md](ROADMAP.md) | pick up work, or see what comes next and why |
| [docs/SETUP.md](docs/SETUP.md) | install, update an older copy, get data, fix setup problems |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | work on the filter: state indices, update equations, parameters |
| [docs/design/eskf_design.md](docs/design/eskf_design.md) | see the design record and the literature behind each choice |
| [docs/requirements.md](docs/requirements.md) | see what "good enough" means for each vehicle, and how it's checked |
| [docs/open_items.md](docs/open_items.md) | check or log an open issue |
| [docs/test_reports/](docs/test_reports/) | see the evidence behind each design decision |

## References

- J. Solà, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508, 2017.
- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
- L. D. Hostetler, work on SITAN recursive terrain-aided navigation, IEEE, 1978 to 1985.
- J. P. Golden, *Terrain contour matching (TERCOM)*, SPIE, 1980.
- T. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017.
- W. Xu et al., *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*, IEEE T-RO, 2022.
- P. Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA, 2020.
- PX4 ROS 2 user guide; ROS 2 Jazzy documentation; Luxonis DepthAI documentation.

## License

MIT. See [LICENSE](LICENSE).
