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
2. **Relative odometry.** Lidar-inertial odometry (LIO), and later visual-inertial
   odometry (VIO), measure motion between fixes and slow down IMU drift. *Planned,
   next after the measurement-model work.*
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
- [Repository layout](#repository-layout)
- [Getting started](#getting-started)
- [Running things](#running-things)
- [Validation so far](#validation-so-far)
- [Known limitations](#known-limitations)
- [Contributing](#contributing)
- [References](#references)

## Project status

GDN is a research prototype. It navigates well in simulation under friendly
assumptions, and it has not flown on real hardware yet.

| Part | State | Notes |
|---|---|---|
| ESKF core (19 states) | Working in sim | Position, velocity, attitude, accel/gyro bias, baro bias, mag bias. ROS-free C++17. |
| TRN scalar elevation update | Working in sim | Uses the terrain-map gradient. The measurement model is still idealised (see limitations). |
| Barometer, magnetometer, ZUPT | Working in sim | Mag gives heading at start-up; ZUPT pins the filter on the ground. |
| Terrain database tools | Working | Synthetic terrain and real SRTM (Zermatt). The maps rebuild byte for byte. |
| ROS 2 node (PX4 topics) | Working on replay | Jazzy, `px4_msgs` 1.15, deterministic replay bench. |
| Unit tests and Monte Carlo | Working | 24 checks at `-O2` and `-O3`; seeded MC campaigns. |
| Real altimeter (AGL) measurement model | Next | Baro altitude minus measured height above ground, with an altitude-bias state. |
| LIO / VIO odometry input | Planned | LIO first (Livox lidar). VIO once a camera is fitted. |
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
  LIO / VIO odometry (planned) -----> velocity / delta-pose updates
  rangefinder / lidar AGL ----------> TRN: terrain height vs onboard map ---> position, velocity,
  lidar scans, camera (planned) ----> map relocalisation / geo-registration   attitude + covariance
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
reference, with state indices, parameters and topic contract, is in
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

### Terrain database

Maps are a simple binary grid: a 40-byte header, then row-major `float32` heights. The
local NED origin is the grid centre. Queries return bilinear height and gradient. Two
maps ship with the repo: a synthetic fractal map, smoothed so scalar TRN behaves
(REQ-DB-001), and a 1 arc-second SRTM crop around Zermatt for testing on real relief.

### Where LIO and VIO fit

Between terrain fixes the filter otherwise relies on the IMU alone, and a MEMS IMU
drifts by kilometres within minutes. The odometry layer fills that gap. GDN will
accept velocity or delta-pose from an external odometry source, with its covariance
and a health flag.

- **LIO** (FAST-LIO2 or Point-LIO on a Livox Mid-360 class lidar) works in the dark and
  in low texture. It loses lock above the lidar's useful range, though, and it slides
  sideways over flat ground with nothing to anchor it horizontally.
- **VIO** (OpenVINS or similar, once a camera is fitted) is good over flat textured
  ground. It struggles at night, over water, and when hovering high with little
  parallax.

The two fail in different conditions. That is why GDN is the one that weighs them,
rather than trusting either blindly.

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

All validation so far has been on fixed-wing-like trajectories. Multirotor and
quadplane work starts with hardware bring-up (see the roadmap).

## Repository layout

```
.
├── README.md                 this file
├── ROADMAP.md                what's next, phase by phase
├── env.sh                    source in every terminal (sets GDN_ROOT, ROS, RMW)
├── src/
│   ├── gdn_fusion/           main package
│   │   ├── include/gdn/      ROS-free core: eskf.hpp, map_db.hpp, paths.hpp
│   │   └── src/              eskf_node, map_viz_node, selftest, mc_campaign
│   ├── gdn_sensors/          sensor_monitor (IMU / local position rate check)
│   └── px4_msgs/             git submodule, PX4 release/1.15
├── tools/
│   ├── setup_ubuntu.sh       one-shot machine setup
│   ├── make_data.sh          derived data, checksum and map verification
│   ├── fetch_data.sh, pack_data.sh   download / publish the flight001 recording
│   ├── run_selftest.sh, mc_run.sh, bench.sh, bench_viz.sh
│   ├── trn_sensor_sim.py, baro_sim.py, mag_sim.py, replay_scorer.py
│   ├── make_zermatt_bag.py, gen_zermatt_flight.py, plot_zermatt_track.py
│   ├── generate_terrain.py, export_map_db.py, make_sigma4_map.py, make_zermatt_map.py
│   ├── fetch_terrain.py, make_world_final.py, make_terrain_mesh.py   SRTM to map / Gazebo
│   ├── gdn_paths.py          shared repo-root and map helpers for the Python tools
│   └── analysis scripts      analyze_ins, eskf_trn, eskf_mismatch, trn_match, py_trace
├── data/                     maps in git; flights generated or downloaded
└── docs/
    ├── SETUP.md              install, update, data, troubleshooting
    ├── ARCHITECTURE.md       filter maths, topics, parameters, validation detail
    ├── design/eskf_design.md design record
    ├── open_items.md         open-item ledger (OI-xxx)
    └── test_reports/         TEST-002 to TEST-012
```

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
| Replay bench on the PX4 flight | `tools/bench.sh` (add `BENCH_NO_BARO=1` for the baro-off run) |
| Replay bench on any bag and map | `GDN_BAG=... GDN_MAP=... tools/bench.sh` |
| Bench with RViz | `tools/bench_viz.sh` |
| Just the node | `ros2 run gdn_fusion eskf_node --ros-args -p map_path:=data/maps/zermatt.bin` |

The PX4 replay flight (`flight001`) is a download: `tools/fetch_data.sh`. Everything
else is in the repo or generated by `tools/make_data.sh`.

## Validation so far

| Test | Result | Conditions |
|---|---|---|
| Unit tests | 24 / 24 at `-O2` and `-O3` | Map maths, alignment, TRN, baro, mag, ZUPT |
| Monte Carlo, Zermatt SRTM | CEP50 14.2 m, 19 of 24 seeds under 100 m | Synthetic flight, known start, map used as truth |
| ROS replay bench, Zermatt | 13 to 20 m RMS | Synthetic PX4-style bag, baro off |
| Monte Carlo and bench, PX4 replay flight | Needs re-baselining | Three bench fixes in Nov 2026 changed the setup; the older figures were 15.7 m RMS on the bench and CEP50 14.9 m in MC |

The failures in Zermatt are false-contour locks (OI-009). Each test is described in
`docs/test_reports/`.

## Known limitations

These are the honest gaps between this and something you could fly. They're the top of
the roadmap.

- **The TRN measurement is idealised.** The simulator hands the filter terrain height
  directly, so altitude error never reaches it. A real system measures baro altitude
  minus height above ground. When we injected a 15 m altitude bias, every seed
  diverged.
- **The filter starts where the vehicle really is.** There's no acquisition mode, and
  start errors above about 60 m fail.
- **The Monte Carlo uses one map as both world and onboard database,** so map error
  isn't modelled.
- **The node still reads GNSS-derived data** for ZUPT, the airborne latch and the
  initial altitude.
- **There's no integrity monitoring.** A filter that has lost lock can still report a
  small uncertainty.
- **Some covariance bookkeeping is inconsistent.** The TRN update skips some state
  corrections but still shrinks their uncertainty, and the clamps do the same.

## Contributing

We work in small, checkable steps: state a hypothesis, run one decisive experiment,
decide on the pass rule before looking at the result, then commit. In practice:

- Branch from `main` and open a pull request. Don't push to `main` directly.
- `tools/run_selftest.sh` and `colcon test` must pass. Changes to the filter core also
  need the Monte Carlo and bench numbers before and after, in the PR description.
- New behaviour comes with a test, and ideally one that fails on the old code.
- Keep the core ROS-free. Anything ROS-specific belongs in the node.
- Record open questions in `docs/open_items.md` with an OI number, and test write-ups
  in `docs/test_reports/`.
- Pick work from [ROADMAP.md](ROADMAP.md). Each item has an ID; put it in your branch
  name and PR title.

## References

- J. Solà, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508, 2017.
- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
- L. D. Hostetler, work on SITAN recursive terrain-aided navigation, IEEE, 1978 to 1985.
- J. P. Golden, *Terrain contour matching (TERCOM)*, SPIE, 1980.
- T. Barfoot, *State Estimation for Robotics*, Cambridge University Press, 2017.
- W. Xu et al., *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*, IEEE T-RO, 2022.
- P. Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA, 2020.
- PX4 ROS 2 user guide; ROS 2 Jazzy documentation.

## License

MIT. See [LICENSE](LICENSE).
