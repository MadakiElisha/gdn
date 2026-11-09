# GDN roadmap

This is the plan from where the project stands now (November 2026) to a stack that can
fly a vehicle without GNSS. It's written for anyone picking up work, so each item says
what to build, where it goes, and how we'll know it's done.

Items have IDs (R1.2 and so on). Use the ID in your branch name and PR title, for
example `r1.2-agl-measurement`. Tick items off in the PR that finishes them. If an item
turns out to be wrong or too big, change this file in a PR rather than working around
it.

## The approach: simulation first, then hardware

Nothing goes on a vehicle until it has passed in simulation. The order is:

1. Make the estimator honest and the engineering solid (Phases 0 and 1).
2. Build a simulation environment good enough to stand in for flight (Phase 2).
3. Build each new capability in that environment: odometry, acquisition, integrity,
   autopilot output (Phases 3 to 5).
4. Pass a simulation qualification gate at the end of Phase 5.
5. Only then do hardware bring-up, real data and flight tests (Phases 6 and 7),
   followed by quadplane and fixed-wing work (Phase 8) and hardening (Phase 9).

Some hardware work can run in parallel because it only needs a bench, not a vehicle:
the IMU noise log (R1.6), driver smoke tests, and recording sensor data to tune the
simulated sensor models. Those items say so.

The simulation is layered. Each layer catches different problems, and each has a place
in CI.

| Layer | What runs | Catches | Speed |
|---|---|---|---|
| Unit tests | Core maths only | Equation and bookkeeping bugs | Seconds |
| Monte Carlo | Core on IMU and truth CSVs with seeded errors | Statistical robustness and filter consistency | Minutes, thousands of runs |
| Replay bench | ROS node on recorded bags | ROS wiring, timing, message handling | Minutes |
| Closed-loop SITL | PX4 + Gazebo + simulated sensors + GDN, autopilot flying | Whole-system behaviour, all three airframes | Real time |
| Processor-in-the-loop | The same SITL, with GDN running on the Orin or Pi 5 | Compute budget, latency, network timing | Real time |

## Where we are

- The ESKF core, the TRN update with the real altimeter model (R1.1), baro, mag and
  ZUPT all work in simulation and on replay.
- The setup was reworked in PRs #1 to #4:
  - A fresh clone builds and passes its tests on any machine.
  - The maps are in git.
  - Three bench bugs were fixed: the simulator/filter map-origin offset, the
    east-scale error, and the node overriding `r_trn`.
- The simulation is the Monte Carlo, the replay bench and, from the R2.1 branch, a
  first closed-loop SITL (Stage A below, `docs/SIMULATION.md`): PX4 v1.16 flies an
  x500 over the Zermatt terrain in Gazebo Harmonic, and GDN runs alongside in shadow
  mode, drawn live in RViz against the true path.
- The hardware is on order. Until it arrives, all work is in simulation, and the IMU
  noise in simulation is a generic MEMS placeholder (see R1.6).
- Nothing has flown on real hardware.
- The main weaknesses are listed under "Known limitations" in the README.

## Reference hardware for the first build

This is the kit we're bringing up first. Phase 2 builds simulated versions of these
sensors that publish the same ROS 2 messages as the real drivers.

| Role | Part | Notes |
|---|---|---|
| IMU | Yesense / WHEELTEC H30WP | 9-axis MEMS (gyro, accel, mag), up to 400 Hz, UART/RS485. Vendor ROS 2 SDK targets Humble, so it needs a Jazzy build. |
| Lidar | Livox Mid-360S | 360 by 59 degree FOV (-7 to 52 degrees), about 40 m on dark surfaces, 10 Hz, built-in IMU, Ethernet with PTP time sync. |
| Rangefinder | Benewake TFmini-S | 0.1 to 12 m (about 7 m on dark surfaces), 2 degree beam, UART/I2C. PX4 has a driver. |
| Stereo camera | Luxonis OAK-D Lite | Global-shutter OV7251 stereo pair, 7.5 cm baseline (useful depth to about 12 m), 73 degree HFOV. 13 MP rolling-shutter IMX214 colour camera, BMI270 IMU, onboard RVC2 processor, USB-C, 2.5 to 3 W. Kickstarter-batch units have no IMU. |
| Companion | Jetson Orin Nano (JetPack 7.2+, Ubuntu 24.04) or Raspberry Pi 5 + Hailo HAT (Ubuntu 24.04) | Orin is preferred: more CPU/GPU headroom for LIO and VIO. The Hailo NPU only helps neural-network parts, and the OAK-D can run feature tracking itself. |
| Autopilot | PX4 v1.16 (`px4_msgs` release/1.16, see R2.1); ArduPilot later | uXRCE-DDS link, with `px4_msgs` matching the PX4 release. |

What this kit can and can't do:
- The lidar sees the ground only up to about 30 m above ground when mounted pointing
  down, and the TFmini-S only to about 12 m. So the kit suits a **multirotor flying
  low**.
- The OAK-D Lite works at any height, but its stereo depth runs out at about 12 m.
  Higher up it acts as a monocular camera that gets its scale from the IMU.
- Quadplane cruise and fixed-wing work also need a long-range altimeter (R8.4).

---

## Phase 0: close out the setup work

Goal: everyone works from the same code and data, and the baselines are current.

- [x] **R0.1** Update the dev PC to `main` (steps in `docs/SETUP.md`, "Existing machine", including the copied-workspace note).
- [x] **R0.2** Publish the `flight001` recording as release `data-v1` (`tools/pack_data.sh --publish`) and check `tools/fetch_data.sh` on a second machine.
- [x] **R0.3** Re-baseline `flight001` after the PR #1 fixes. Run `tools/bench.sh`, `BENCH_NO_BARO=1 tools/bench.sh` and `tools/mc_run.sh 100`, and record the numbers in `docs/test_reports/` and the README. Done: TEST-016.
- [ ] **R0.4** Run `tools/setup_ubuntu.sh` once on a real Ubuntu 24.04 machine using the apt path, and fix anything it trips on.
- [x] **R0.5** Add GitHub Actions CI: build in a Jazzy container, `run_selftest.sh`, `colcon test`, `make_data.sh`, and a short Zermatt MC (24 seeds) with pass thresholds.
- [x] **R0.6** Bring `docs/open_items.md` up to date.
- [x] **R0.7** Move `PX4-Autopilot` out of the GDN workspace (for example to `~/PX4-Autopilot`), so colcon and git never see it.

Done when: CI is green on `main`, and the README validation table shows fresh
`flight001` numbers.

## Phase 1: honest estimator and solid engineering

Goal: test against the problems a real flight will have, before we tune for them.
Also set up the engineering practices that make the result trustworthy. Each estimator
item should first produce a test that fails, then the fix.

Estimator:

- [x] **R1.1 Real altimeter measurement model.** TRN measurement = baro altitude minus measured AGL, corrected for tilt. Add an altitude-bias state, and put z into the TRN Jacobian. Update `trn_sensor_sim.py` to publish AGL with realistic errors. Core: `eskf.hpp`, new `ApplyAgl()`. Test: a 15 m baro bias no longer diverges. Done: TEST-017. The node now defaults to `trn_model:=agl`, and the oracle stays available as `trn_model:=ideal` and `--model=ideal`. Follow-ups: the tilt correction belongs with the real rangefinder driver (R6.2), and the baro-bias model needs a rate state if fast pressure changes matter (see TEST-017).
- [x] **R1.2 Remove truth leaks from the node.** ZUPT and the airborne latch use IMU stillness or PX4's land detector, not GNSS-derived velocity. Initial altitude comes from baro, not `vehicle_local_position`. Done: TEST-018. `gdn/ground_monitor.hpp` (IMU only) drives ZUPT and the latch in both the node and the MC, and `vehicle_local_position` is now display truth only. The old rule ran ZUPT for the first 0.9 s of the take-off roll, which was most of audit finding A-01: through the node's start-up path (`mc_campaign --startup=node`) the AGL model goes from 46.0 m CEP50 and 59/100 seeds to 10.8 m and 87/100. Also fixed: ZUPT no longer moves heading or the baro and mag biases (8 deg of yaw walk in 60 s parked; part of R1.4). Follow-ups: landing and re-take-off need the autopilot's regime (R2.5), since the latch is one-way; the MC's default start-up is still `truth` for the CI baselines.
- [x] **R1.3 Separate world and database maps.** The MC and bench support a truth map that differs from the onboard map: DEM noise, a bias, a resolution change. Add `make_data.sh` options to build the degraded maps. Done: TEST-020. `MapDb::SetOrigin` puts both maps in one frame (node: `origin_lat`/`origin_lon`); `tools/make_db_map.py` (correlated noise, bias, coarsen, smooth); `make_data.sh --db-maps`; `mc_campaign --db-map` and `bench.sh GDN_DB_MAP`; `tools/mc_map_ensemble.py` pools several map error patterns, because one map is one sample (single maps gave 0 to 20 of 20 passing at 10 m noise). Results, 200 flights each: map noise costs steadily (Zermatt 171, 165, 142, 80 passing at 0, 2.5, 5, 10 m) and the filter turns overconfident, since correlated map error isn't modelled. A constant bias is harmless to the AGL model. The best onboard resolution depends on terrain: 3x coarser passes 200/200 on rough Zermatt but costs 24 flights on the already-smoothed flight001. Follow-ups: onboard map resolution/smoothing per terrain (R1.4 gradient work, Phase 4); an independent DEM source arrives with R2.2.
- [x] **R1.4 Covariance consistency fixes.** Raised in priority by OI-012: the 3-axis mag update corrupts tilt on the SITL multirotor.
  - Zero the gain rows for states an update doesn't correct.
  - Remove the per-update clamps, or inflate the covariance when one fires.
  - Fuse the magnetometer as heading only.
  - Make the map gradient consistent with bilinear interpolation.

  Done: TEST-021. Map slope from the bilinear surface (`MapDb::Query`); zeroed gain rows in `ApplyTrn` and `ApplyBaro`; per-update limits now scale the gain (`LimitGain`), so P matches the applied correction; `ApplyMagHeading` (heading only, can't tilt the attitude; assumes a calibrated compass, since a heading can't observe hard iron) and `ApplyMagAuto` (3-axis while the field's strength and dip look clean, else heading only), which is the default (`mag_fusion:=auto`). The simulated compass is now calibrated (0.35 uT residual; `GDN_MAG_HARDIRON=1` for the old 2.4 uT). Zermatt, node start-up, 100 flights: 87 to 100 passing, CEP95 288 to 27 m, REQ-INT-001 12.8% to 0.46% (that CI step is now blocking); the map-slope fix did most of it. Follow-ups: still overconfident on degraded onboard maps (5.5%: correlated map error isn't modelled); auto's tolerances need multirotor data (OI-012).
- [x] **R1.5 Consistency metrics in the MC.** Report NEES/NIS and "error over 3 sigma" percentages, and fail the run when they're out of bounds. Done: TEST-019. `gdn/consistency.hpp`; `ci_check_mc.py --max-over-3sigma/--max-nees-median/--max-nis-over9`. flight001 passes REQ-INT-001 (0.04%) and gates on it in CI. Zermatt fails it (12.8%): 87 passing seeds are honest (NEES median 0.8), the 13 failing seeds are confidently wrong (90% of samples outside 3 sigma). That CI step became blocking with R1.4 (TEST-021: 0.46%).
- [ ] **R1.6 IMU error model** (bench, waits for the H30WP). Until then the MC and Gazebo use a generic MEMS placeholder (PX4's default IMU noise), and results are labelled with it. Record a static H30WP log of 2 hours or more and fit an Allan variance. Do the same for the Mid-360S and OAK-D IMUs. The results feed the process noise, the MC's simulated IMU and the Gazebo IMU models (R2.4). Placeholder part done (TEST-022): `gdn/imu_model.hpp` and `config/imu/`. The MC applies the placeholder (PX4 v1.16's Gazebo x500 IMU, white noise only) to the Zermatt flight's new error-free `imu_truth.csv`, and every report carries an `imu:` line that CI checks (`--expect-imu`). `tools/allan_fit.py` turns a static log or bag into a model file, and has been checked on simulated logs (`imu_sim`). Finding: white noise hardly matters, but bias drift does. With PX4's pre-2025 drift the tuned filter falls to 94/100 and 8.7% over 3 sigma, because its accel-bias random walk is 12x too small; matching it restores 100/100 and 0.97%. That setting is left for the H30WP fit (OI-013). Still open: the three static logs and their fits, and the Gazebo models (R2.4).
- [x] **R1.7 Start-uncertainty and GNSS-loss scenarios.** MC options for a start error of 0, 60, 150 and 500 m, and for GNSS loss mid-flight with a covariance handover. Synthetic flights long enough to score 5 and 10 minutes without GNSS (the Zermatt flight is only 177 s), and an MC scoring window that starts when GNSS is lost rather than the fixed 60 to 300 s. Done: TEST-023. `Eskf::ApplyGnss` and `Eskf::SetHorizontalPosition`; `mc_campaign --gnss-loss=T --gnss-free=300,600` (simulated 5 Hz GNSS until T, then the filter keeps its covariance; scored from the loss) and `--start-error=M` (REQ-ACQ-001 line); a 15 minute Zermatt flight (`gen_zermatt_flight.py --minutes 15`); truth resets end a flight (flight001 at 967.7 s); CI steps for both. Results, 100 flights, fixed-wing limits: Zermatt with a perfect map meets CEP95 at 5 and 10 minutes (25.9 and 32.0 m) and REQ-INT-001, but not the max (false locks); SRTM-like maps reach 202 m CEP95 and 9.7% over 3 sigma at 10 minutes; flight001's real IMU is overconfident for the first minute after the handover (8.6% at 5 minutes), fixed by `q_accel_nd` 0.1 to 0.2 (left for OI-013). Start errors: 60 m works (72 to 99 of 100 under 100 m at 120 s), 150 m fails half or more, 500 m fails (1 to 4 of 100), as expected until R4.1. Follow-ups: correlated GNSS error and the node-side PX4 handover come with SITL (R2.8); IMU tuning with OI-013.

Engineering:

- [ ] **R1.8 Requirements per vehicle class.** Drafted in [docs/requirements.md](docs/requirements.md): 5 and 10 minutes without GNSS, with error limits per vehicle class, plus integrity, acquisition and timing requirements. Remaining: agree the error limits, then tick this off. Every later "done when" refers to this file.
- [ ] **R1.9 Code quality gates in CI.** clang-tidy, cppcheck, ASan/UBSan builds of the selftest and MC, and clang-format. Warnings fail the build for the core.
- [ ] **R1.10 Configuration files.** Move the filter tuning out of code defaults into versioned YAML loaded by the node and the MC, so a result can always be traced to the exact configuration.
- [ ] **R1.11 Full internal logging.** The node logs every input, innovation, gate decision and state to MCAP, so any run (sim or real) can be replayed and diagnosed offline.

Done when: MC results with R1.1 to R1.7 all switched on are written up as a new test
report, scored against R1.8. At this point we expect the large-start-error cases to
fail; Phase 4 fixes those.

## Phase 2: simulation environment

Goal: a closed-loop simulation good enough that passing it means something. PX4 flies
the vehicle in Gazebo, simulated sensors look like our real ones, and GDN runs on top.
Everything gets scored automatically against R1.8.

We bring the visible part forward in three stages, so the whole loop runs early and
each later item lands in a working system:

| Stage | What it shows | Needs | Status |
|---|---|---|---|
| A. Shadow mode | PX4 flies a terrain-following route on GPS; GDN estimates without GPS, drawn against truth in RViz | R2.1, a first world and vehicle | Runs end to end (checked headless, without a GPU). GDN drifts badly on the multirotor (OI-012), so this stage is now the estimator test bench. Still to check on the dev PC: the rendered radar altimeter |
| B. GPS cut | The same flight with a GPS failure injected mid-route; PX4 alone fails safe, GDN carries on | Stage A | Built (`GPS_OFF_AT`) |
| C. Flying on GDN | PX4 takes GDN's position and finishes the route with GPS off | R5.1 | Not started |

- [ ] **R2.1 Toolchain.** Done so far: `px4_msgs` on release/1.16, the `flight001` bag migrated with `tools/migrate_flight001.sh`, `tools/setup_sim.sh` for PX4 v1.16.2, Gazebo Harmonic, ros_gz_bridge and the XRCE agent. Left: the Docker image.
  - Move to **PX4 v1.16.x**. v1.15's setup only supports Ubuntu 22.04 with Gazebo Garden. From v1.16, PX4 officially supports Ubuntu 24.04 with Gazebo Harmonic, which is the pairing ROS 2 Jazzy is built for.
  - Bump the `px4_msgs` submodule to `release/1.16`, and adapt the node to any renamed or versioned topics (1.16 introduced message versioning).
  - Pin everything in a Docker dev image, with a GPU passthrough option, so every machine runs the same simulation.
- [ ] **R2.2 Worlds from real terrain.** Build Gazebo worlds from a high-resolution DEM with orthophoto texture draped on top; the texture matters for VIO. Cover at least these terrain classes:
  - mountain (Zermatt)
  - rolling hills
  - flat farmland
  - built-up area with structures
  - coast or open water

  Each world comes with an onboard GDN map built from a coarser source (SRTM or Copernicus GLO-30), so the filter never sees the exact truth terrain. Fix OI-007 by rendering on native Linux with a GPU.
- [ ] **R2.3 Vehicles.** PX4's Gazebo multirotor (x500), standard VTOL and fixed-wing models, each with a GDN sensor payload attached through an SDF include.
- [ ] **R2.4 Simulated sensors that match the real ones**, publishing the same message types and topic names as the real drivers:
  - **Mid-360S:** gpu_lidar with a 360 by 59 degree FOV (-7 to 52 degrees), 40 m range on low-reflectivity ground, 10 Hz, and a scan pattern that approximates the Livox non-repetitive pattern. The existing Livox simulation plugins target Gazebo Classic and ROS 1, so port one or approximate. The real pattern gets checked on real bags in Phase 6.
  - **TFmini-S:** a single-beam ranger, 12 m, 100 Hz, with range-dependent noise.
  - **OAK-D Lite:** two 640 by 480 global-shutter cameras, 7.5 cm apart, 73 degree HFOV, plus an IMU.
  - **IMUs** with bias instability and random walk from R1.6.
  - **Timing:** latency, timestamp jitter and dropouts on every sensor.
- [ ] **R2.5 Vehicle profiles and flight regime.** A YAML file per airframe: sensors, extrinsics, noise, enabled aiding, and regime rules (ground, hover, transition, cruise). The regime comes from the autopilot. The node loads one profile by parameter. All three simulated airframes have one.
- [ ] **R2.6 Scenario runner.** A scenario is a YAML file naming:
  - vehicle, world and mission route, height and speed
  - when GNSS is lost, and how: PX4 failure injection (`SYS_FAILURE_EN`, `failure gps off`), or slow spoofing drift
  - start error, wind and lighting
  - sensor faults (dropout, stuck values, noise bursts)

  The runner launches it headless, records everything to MCAP, and tears down cleanly. It's the SITL equivalent of `bench.sh`.
- [ ] **R2.7 Metrics and reports.** From each run, compute:
  - CEP50 and CEP95, maximum error, and error against requirement
  - NEES and NIS
  - time to acquire
  - integrity detection and false-alarm rates
  - CPU and latency

  Write a short report with plots, and compare against the last accepted baseline.
- [ ] **R2.8 Scenario matrix.** Vehicles by terrain class by height by GNSS-loss time by start error by faults. Tag each scenario "smoke" (a few minutes, runs on every PR), "nightly" or "full".
- [ ] **R2.9 Simulation in CI.** Smoke scenarios on every PR, headless. The nightly suite runs on a self-hosted runner with a GPU (the dev PC can do this to start with).

Done when: all three airframes fly a mission in SITL with GNSS on. GDN runs alongside
in shadow mode, and each run is scored automatically. Then the same mission with GNSS
cut mid-flight shows today's known weaknesses in the report.

## Phase 3: odometry layer (LIO and VIO)

Goal: bounded short-term drift between absolute fixes. Develop it in Phase 2 SITL,
then confirm it on real bags in Phase 6.

- [ ] **R3.1 LIO in SITL.** Run FAST-LIO2 or Point-LIO on the simulated Mid-360S. Measure drift by height and terrain class, and note where it degenerates (flat ground, out of range).
- [ ] **R3.2 GDN odometry input.** A new core update for body-frame velocity or delta-pose with covariance (`ApplyOdom()`), and a node subscription to `nav_msgs/Odometry`. Test: a unit test, an MC with simulated odometry noise, and SITL.
- [ ] **R3.3 Odometry health.** Reject or inflate odometry when LIO reports degeneracy, when the lidar sees too few points, or when its innovations stay too large.
- [ ] **R3.4 Compute budget.** Measure CPU, GPU and memory for LIO, VIO and GDN together on the Orin and on the Pi 5, using processor-in-the-loop. Decide which board we use.
- [ ] **R3.5 Stereo VIO.** Run OpenVINS or VINS-Fusion in stereo-inertial mode on the simulated OAK-D Lite. On the Orin, also try cuVSLAM; Isaac ROS support on JetPack 7.2 for Orin was still "coming soon" at last check. Feed it through the same `ApplyOdom()` interface as LIO.
- [ ] **R3.6 Cross-checking LIO and VIO.** Compare the two against each other and against truth by terrain class and height. GDN should down-weight whichever one is degenerate, for example LIO over a flat field or VIO over water or in low light.

Done when: in the SITL scenario matrix with GNSS cut, GDN fused with LIO and VIO stays
within the R1.8 numbers on the flat-ground scenarios. Those are the ones TRN alone
can't handle.

## Phase 4: absolute fixes, acquisition and integrity

Goal: GDN finds its own position from a large error, notices when it's lost, and
recovers.

- [ ] **R4.1 TRN acquisition.** A point-mass or particle filter over position, fed by a sequence of TRN measurements. Hand over to the ESKF once the solution has a single clear peak. Test: the R1.7 start-error cases now pass, and OI-009 is closed.
- [ ] **R4.2 Lidar terrain patch matching.** At low height, match a ground patch from the lidar against a high-resolution DSM. This needs 1 to 5 m data, since 30 m SRTM is too coarse for a 60 m wide patch.
- [ ] **R4.3 Map relocalisation.** Localise against a lidar map from an earlier GNSS-on flight over the same area, flown first in SITL. This is the most practical absolute fix for a low multirotor.
- [ ] **R4.4 Integrity monitor.**
  - Windowed innovation tests on each sensor.
  - Cross-checks between TRN, odometry and the IMU.
  - States: valid, degraded and lost, plus a published protection level.
- [ ] **R4.5 Lost-lock recovery.** On "lost", widen the covariance and go back to R4.1 acquisition.

Done when: MC and SITL runs pass the R1.8 numbers with random start errors up to
500 m and injected faults, including spoofing drift. Lost-lock detection rate and
false-alarm rate are within the requirement.

## Phase 5: autopilot integration and simulation qualification

Goal: PX4 flies on GDN with GNSS off, in simulation, on the real companion computer.
Passing this phase is the gate for hardware.

- [ ] **R5.1 Output to PX4.** PX4 v1.16 exposes both `/fmu/in/vehicle_visual_odometry` and `/fmu/in/aux_global_position` over uXRCE-DDS. Decide which suits GDN (probably auxiliary global position for absolute fixes), publish with covariance, and use sensor timestamps, not receive time.
- [ ] **R5.2 Output to ArduPilot.** MAVLink `ODOMETRY` into EKF3 as ExternalNav, tested in ArduPilot SITL.
- [ ] **R5.3 Closed-loop missions.** All three airframes fly full missions in SITL with GNSS cut, with PX4 using GDN's output. Include failsafes: what PX4 does when GDN reports degraded or lost.
- [ ] **R5.4 Processor-in-the-loop.** Run GDN, LIO and VIO on the actual Orin Nano or Pi 5, connected to SITL on the dev PC over Ethernet. Measure latency, CPU headroom and thermal behaviour over long runs.
- [ ] **R5.5 Qualification campaign.** Run the full scenario matrix (R2.8) and write it up as a test report against R1.8. Freeze the configuration that passed; that's what goes on the vehicle.

Done when: the qualification report passes for the multirotor with the reference
hardware profile. Quadplane and fixed-wing can follow later, in Phase 8.

## Phase 6: hardware bring-up and real data

Goal: real, time-synced sensor logs with GNSS as ground truth, and proof that the
simulated sensors behave like the real ones. Bench items (R6.1 to R6.4) can start
early, in parallel with Phases 2 to 5.

- [ ] **R6.1 Companion computer image.** Orin Nano on JetPack 7.2+ (or a Pi 5 on Ubuntu 24.04). `setup_ubuntu.sh --headless` works on it, and build times are written down.
- [ ] **R6.2 Drivers.**
  - `livox_ros_driver2` for the Mid-360S.
  - The H30WP ROS 2 driver, built for Jazzy.
  - TFmini-S on the flight controller (PX4 driver), read on the companion as `distance_sensor`.
  - `depthai-ros` for the OAK-D Lite: the stereo pair at 30 Hz or more, the IMU at 200 Hz or more, and the colour camera at a low rate for logging. Check for a Jazzy build, and build from source if there isn't one.
- [ ] **R6.3 Time sync.** PTP from the companion to the Livox. Measure H30WP latency and timestamp it on arrival. Use the OAK-D's device timestamps, synced to the host. Sync the companion and PX4 clocks through uXRCE-DDS. Write down the measured offsets, and feed them back into the R2.4 timing models.
- [ ] **R6.4 Extrinsics.**
  - Lidar to IMU (the Livox has an internal IMU; decide whether LIO uses it or the H30WP).
  - Camera to IMU with Kalibr. Use the OAK-D's own BMI270, or the H30WP with an estimated time offset.
  - Rangefinder pointing and lever arms.

  Store them in the vehicle profile (R2.5).
- [ ] **R6.5 Mount and wiring on a multirotor test frame.** The Mid-360S tilted or inverted so it sees the ground. The OAK-D Lite pointing down or tilted forward-down, so it sees ground texture in hover and in forward flight. Vibration isolation for both IMUs.
- [ ] **R6.6 Logging.** One launch file records everything to MCAP, in the same layout as SITL runs so the same metrics scripts work on both. Add a short checklist for each flight. Datasets live in a release or shared storage, not in git.
- [ ] **R6.7 First datasets.** Multirotor flights with GNSS on and logged: hover, straight legs, lawnmower pattern, and flights over flat, rolling and structured ground, at heights from 5 to 40 m.
- [ ] **R6.8 Sim-to-real check.** Replay the real bags through the stack and compare against the matching SITL scenarios: noise levels, LIO and VIO drift, TRN innovations. Fix the sensor models wherever they're optimistic, and re-run the Phase 5 qualification if anything changed.

Done when: at least three logged flights where every sensor is present, timestamps line
up within tolerance, and the replayed results agree with SITL within an agreed margin.

## Phase 7: flight test

Goal: the multirotor flies without GNSS.

- [ ] **R7.1 Staged flight tests.**
  1. GNSS fused, GDN in shadow mode and logged.
  2. GNSS logged, GDN fused, with a safety pilot.
  3. GNSS switched off in flight.

  Write the abort criteria down before each stage, and only move up a stage after reviewing the previous stage's report.
- [ ] **R7.2 Flight reports.** Each flight produces the same report as a SITL run (R2.7), so sim and flight results sit side by side.

Done when: the multirotor flies a mission leg with GNSS disabled and lands inside the
R1.8 numbers.

## Phase 8: quadplane and fixed-wing

Goal: the same stack on quadplanes and fixed-wing. Each item is built and qualified in
SITL first (Phases 2 to 5 already include these airframes), then flown.

- [ ] **R8.1 Quadplane transitions.** Regime switching through transition, and aiding hand-over between hover sensors (LIO, VIO, rangefinder) and cruise sensors.
- [ ] **R8.2 Tailsitter-safe attitude handling.** Remove the assumptions that only hold in level flight (mag yaw from body x/y, nadir-pointing rangefinder).
- [ ] **R8.3 Airspeed and wind.** Wind states, airspeed fusion and a sideslip constraint for fixed-wing.
- [ ] **R8.4 Long-range altimeter.** A radar or long-range laser altimeter for cruise heights, using the R1.1 model. Choose the part, model it in SITL, then fit it.
- [ ] **R8.5 In-motion alignment.** For catapult or hand launches, where static alignment isn't possible.
- [ ] **R8.6 Camera geo-registration.** Match downward images from the OAK-D Lite's colour camera to orthophotos, for absolute fixes over flat terrain at cruise height. This is where the Orin GPU or the Hailo NPU earns its place. Watch the rolling shutter at speed.
- [ ] **R8.7 Qualification and flight test** for the quadplane, then fixed-wing, repeating R5.5 and Phase 7.

## Phase 9: hardening and release

Goal: something other people can rely on. Much of the groundwork is laid in Phase 1
(R1.9 to R1.11); this phase finishes it.

- [ ] **R9.1 Real-time core.** No allocation in the hot path, a bounded runtime, profiled on target.
- [ ] **R9.2 Fuzzing** of the map loader and message parsers.
- [ ] **R9.3 Nightly full scenario matrix** with trend plots, so regressions show up the day they land.
- [ ] **R9.4 Hazard analysis** (FMEA) and a safety case for each vehicle class.
- [ ] **R9.5 Versioned releases.** Tagged releases with changelog, frozen configuration and the qualification report that went with them.
- [ ] **R9.6 Export-control check** before any sale or external release. Navigation systems for UAVs can fall under dual-use rules.

---

## Not planned for now

- Magnetic-anomaly navigation and celestial aids.
- Running the estimator on the flight controller itself.
- Certification work beyond R9.4.

These can come back once Phase 7 has flown.
