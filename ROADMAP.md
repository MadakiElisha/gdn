# GDN roadmap

This is the plan from where the project stands now (November 2026) to a stack that can
fly a vehicle without GNSS. It's written for anyone picking up work, so each item says
what to build, where it goes, and how we'll know it's done.

Items have IDs (R1.2 and so on). Use the ID in your branch name and PR title, for
example `r1.2-agl-measurement`. Tick items off in the PR that finishes them. If an item
turns out to be wrong or too big, change this file in a PR rather than working around
it.

## Where we are

- The ESKF core, TRN update, baro, mag and ZUPT all work in simulation and on replay.
- The setup was reworked in PR #1. A fresh clone builds and passes its tests on any
  machine, the maps are in git, and three bench bugs were fixed: the simulator/filter
  map-origin offset, the east-scale error, and the node overriding `r_trn`.
- Nothing has flown on real hardware yet.
- The main weaknesses are listed under "Known limitations" in the README. Phases 0 and 1
  deal with them before any hardware work depends on the results.

## Reference hardware for the first build

This is the kit we're bringing up first. Other parts should work as long as they
publish the same ROS 2 messages.

| Role | Part | Notes |
|---|---|---|
| IMU | Yesense / WHEELTEC H30WP | 9-axis MEMS (gyro, accel, mag), up to 400 Hz, UART/RS485. Vendor ROS 2 SDK targets Humble, so it needs a Jazzy build. |
| Lidar | Livox Mid-360S | 360 by 59 degree FOV, about 40 m on dark surfaces, built-in IMU, Ethernet with PTP time sync. |
| Rangefinder | Benewake TFmini-S | 0.1 to 12 m (about 7 m on dark surfaces), UART/I2C. PX4 has a driver. |
| Companion | Jetson Orin Nano (JetPack 7.2+, Ubuntu 24.04) or Raspberry Pi 5 + Hailo HAT (Ubuntu 24.04) | Orin is preferred: more CPU/GPU headroom for LIO, plus a future camera pipeline. The Hailo NPU only helps neural-network parts. |
| Autopilot | PX4 v1.15 (ArduPilot later) | Over uXRCE-DDS, matching `px4_msgs` release/1.15. |

What this kit can and can't do: the lidar sees the ground only up to about 30 m above
ground when mounted pointing down, and the TFmini-S only to about 12 m. So this kit
suits a **multirotor flying low**. Quadplane cruise and fixed-wing work need a
long-range altimeter and a camera (Phase 6). There's no camera in the first build, so
LIO comes before VIO.

---

## Phase 0: close out the setup work

Goal: everyone works from the same code and data, and the baselines are current.

- [ ] **R0.1** Update the dev PC to `main` (steps in `docs/SETUP.md`, "Existing machine").
- [ ] **R0.2** Publish the `flight001` recording as release `data-v1` (`tools/pack_data.sh --publish`) and check `tools/fetch_data.sh` on a second machine.
- [ ] **R0.3** Re-baseline `flight001` after the PR #1 fixes. Run `tools/bench.sh`, `BENCH_NO_BARO=1 tools/bench.sh` and `tools/mc_run.sh 100`, and record the numbers in `docs/test_reports/` and the README.
- [ ] **R0.4** Run `tools/setup_ubuntu.sh` once on a real Ubuntu 24.04 machine using the apt path, and fix anything it trips on.
- [ ] **R0.5** Add GitHub Actions CI: build in a Jazzy container, `run_selftest.sh`, `colcon test`, `make_data.sh`, and a short Zermatt MC (24 seeds) with pass thresholds.
- [ ] **R0.6** Bring `docs/open_items.md` up to date. OI-002 and OI-003 are done, and OI-009, OI-010 and OI-011 are only described in commit messages.

Done when: CI is green on `main`, and the README validation table shows fresh
`flight001` numbers.

## Phase 1: make the simulation honest

Goal: test against the problems a real flight will have, before we tune for them. Each
item should first produce a test that fails, then the fix.

- [ ] **R1.1 Real altimeter measurement model.** TRN measurement = baro altitude minus measured AGL, corrected for tilt. Add an altitude-bias state, and put z into the TRN Jacobian. Update `trn_sensor_sim.py` to publish AGL with realistic errors. Core: `eskf.hpp`, new `ApplyAgl()`. Test: a 15 m baro bias no longer diverges.
- [ ] **R1.2 Remove truth leaks from the node.** ZUPT and the airborne latch use IMU stillness or PX4's land detector, not GNSS-derived velocity. Initial altitude comes from baro, not `vehicle_local_position`.
- [ ] **R1.3 Separate world and database maps.** The MC and bench support a truth map that differs from the onboard map: DEM noise, a bias, a resolution change. Add `make_data.sh` options to build the degraded maps.
- [ ] **R1.4 Covariance consistency fixes.**
  - Zero the gain rows for states an update doesn't correct.
  - Remove the per-update clamps, or inflate the covariance when one fires.
  - Fuse the magnetometer as heading only.
  - Make the map gradient consistent with bilinear interpolation.
- [ ] **R1.5 Consistency metrics in the MC.** Report NEES/NIS and "error over 3 sigma" percentages, and fail the run when they're out of bounds.
- [ ] **R1.6 IMU error model.** Record a static H30WP log of 2 hours or more and fit an Allan variance. Use the result for the process noise and for the MC's simulated IMU (bias instability, random walk, scale factor).
- [ ] **R1.7 Start-uncertainty scenarios.** MC options for a start error of 0, 60, 150 and 500 m, and for GNSS loss mid-flight with a covariance handover.

Done when: MC results with R1.1 to R1.7 all switched on are written up as a new test
report. At this point we expect the large-start-error cases to fail; Phase 4 fixes
those.

## Phase 2: hardware bring-up and data collection

Goal: real, time-synced sensor logs with GNSS as ground truth. Everything after this
gets developed on replay of these logs.

- [ ] **R2.1 Companion computer image.** Orin Nano on JetPack 7.2+ (or a Pi 5 on Ubuntu 24.04). `setup_ubuntu.sh --headless` works on it, and build times are written down.
- [ ] **R2.2 Drivers.**
  - `livox_ros_driver2` for the Mid-360S.
  - The H30WP ROS 2 driver, built for Jazzy.
  - TFmini-S on the flight controller (PX4 driver), read on the companion as `distance_sensor`.
- [ ] **R2.3 Time sync.** PTP from the companion to the Livox. Measure H30WP latency and timestamp it on arrival. Sync the companion and PX4 clocks through uXRCE-DDS. Write down the measured offsets.
- [ ] **R2.4 Extrinsics.** Lidar to IMU (the Livox has an internal IMU; decide whether LIO uses it or the H30WP), rangefinder pointing, and lever arms. Store them in a vehicle profile file (see R6.1).
- [ ] **R2.5 Mount and wiring on a multirotor test frame.** The Mid-360S tilted or inverted so it sees the ground. Vibration isolation for the IMU.
- [ ] **R2.6 Logging.** One launch file records everything to MCAP. Add a short checklist for each flight. Datasets live in a release or shared storage, not in git.
- [ ] **R2.7 First datasets.** Multirotor flights with GNSS on and logged: hover, straight legs, lawnmower pattern, and flights over flat, rolling and structured ground, at heights from 5 to 40 m.

Done when: there are at least three logged flights where every sensor is present and
timestamps line up within the agreed tolerance.

## Phase 3: odometry layer (LIO first, VIO when a camera is fitted)

Goal: bounded short-term drift between absolute fixes.

- [ ] **R3.1 Run FAST-LIO2 or Point-LIO on the Phase 2 bags.** Measure drift against GNSS by height and terrain type, and note where it degenerates (flat ground, out of range).
- [ ] **R3.2 GDN odometry input.** A new core update for body-frame velocity or delta-pose with covariance (`ApplyOdom()`), and a node subscription to `nav_msgs/Odometry`. Test: a unit test plus an MC with simulated odometry noise.
- [ ] **R3.3 Odometry health.** Reject or inflate odometry when LIO reports degeneracy, when the lidar sees too few points, or when its innovations stay too large.
- [ ] **R3.4 Compute budget.** Measure CPU, GPU and memory for LIO plus GDN on the Orin and on the Pi 5, and decide which one we use.
- [ ] **R3.5 VIO slot.** Once a camera is chosen (global shutter, ideally hardware-triggered), run OpenVINS through the same `ApplyOdom()` interface.

Done when: on replay with GNSS ignored, GDN fused with LIO stays within an agreed error
over the Phase 2 flights.

## Phase 4: absolute fixes, acquisition and integrity

Goal: GDN finds its own position from a large error, notices when it's lost, and
recovers.

- [ ] **R4.1 TRN acquisition.** A point-mass or particle filter over position, fed by a sequence of TRN measurements. Hand over to the ESKF once the solution has a single clear peak. Test: the R1.7 start-error cases now pass.
- [ ] **R4.2 Lidar terrain patch matching.** At low height, match a ground patch from the lidar against a high-resolution DSM. This needs 1 to 5 m data, since 30 m SRTM is too coarse for a 60 m wide patch.
- [ ] **R4.3 Map relocalisation.** Localise against a lidar map from an earlier GNSS-on flight over the same area. This is the most practical absolute fix for a low multirotor.
- [ ] **R4.4 Integrity monitor.**
  - Windowed innovation tests on each sensor.
  - Cross-checks between TRN, odometry and the IMU.
  - States: valid, degraded and lost, plus a published protection level.
- [ ] **R4.5 Lost-lock recovery.** On "lost", widen the covariance and go back to R4.1 acquisition.

Done when: an MC with random start errors up to 500 m and injected faults passes,
lost-lock detection stays above an agreed rate, and the false-alarm rate stays below
one.

## Phase 5: autopilot integration and flight test

Goal: PX4 flies on GDN with GNSS off.

- [ ] **R5.1 Output to PX4.** Publish to `vehicle_visual_odometry` with covariance, or use PX4's auxiliary global position input. Check which one v1.15 supports properly. Use sensor timestamps, not receive time.
- [ ] **R5.2 Output to ArduPilot.** MAVLink `ODOMETRY` into EKF3 as ExternalNav.
- [ ] **R5.3 SITL with the full chain.** A Gazebo world from a real DEM (`make_world_final.py`), with simulated lidar and rangefinder, and PX4 flying on GDN.
- [ ] **R5.4 Staged flight tests.** These steps:
  1. GNSS fused, GDN logged.
  2. GNSS logged, GDN fused, with a safety pilot.
  3. GNSS switched off in flight.

  Write down the abort criteria before each stage.

Done when: the multirotor flies a mission leg with GNSS disabled and lands inside the
agreed error.

## Phase 6: vehicle-agnostic expansion

Goal: the same stack on quadplanes and fixed-wing.

- [ ] **R6.1 Vehicle profiles.** A YAML file per airframe: sensors, extrinsics, noise, enabled aiding, regime rules. The node loads one with a parameter.
- [ ] **R6.2 Flight-regime input.** Ground, hover, transition and cruise, taken from the autopilot. Process noise and aiding selection switch by regime.
- [ ] **R6.3 Tailsitter-safe attitude handling.** Remove the assumptions that only hold in level flight (mag yaw from body x/y, nadir-pointing rangefinder).
- [ ] **R6.4 Long-range altimeter.** A radar or long-range laser altimeter for cruise heights, using the R1.1 model.
- [ ] **R6.5 Airspeed and wind.** Wind states, airspeed fusion and a sideslip constraint for fixed-wing.
- [ ] **R6.6 In-motion alignment.** For catapult or hand launches, where static alignment isn't possible.
- [ ] **R6.7 Camera geo-registration.** Match downward images to orthophotos for absolute fixes over flat terrain, where the Hailo or Orin accelerator earns its place.

## Phase 7: hardening

Goal: something other people can rely on.

- [ ] **R7.1 Real-time core.** No allocation in the hot path, a bounded runtime, profiled on target.
- [ ] **R7.2 Static analysis and sanitizers** in CI, plus fuzzing of the map loader and message parsers.
- [ ] **R7.3 Nightly MC** on all scenarios, with trend plots.
- [ ] **R7.4 Hazard analysis** (FMEA) and a safety case for each vehicle class.
- [ ] **R7.5 Export-control check** before any sale or external release. Navigation systems for UAVs can fall under dual-use rules.

---

## Not planned for now

- Magnetic-anomaly navigation and celestial aids.
- Running the estimator on the flight controller itself.
- Certification work beyond R7.4.

These can come back once Phase 5 has flown.
