# GDN — GNSS-Denied Terrain-Referenced Navigation (TRN)

Error-state Kalman filter (ESKF) navigation stack for any aerial vehicle
operating without GNSS. Fuses IMU, a terrain-elevation measurement (terrain-referenced
navigation, TRN) and a barometer against an onboard terrain database to
produce a bounded 3-D position estimate in a local map frame.

Vehicle-agnostic by design: the core contains no airframe model (no rotor,
control-surface or thrust dynamics) — it is a pure state estimator, so the
same binary runs on fixed-wing, multirotor, VTOL and hybrid platforms.
Platform specifics enter only through sensor-noise tuning and the terrain
database; all validation to date uses a fixed-wing PX4 SITL replay.

Repository: https://github.com/MadakiElisha/gdn
Platform: Ubuntu 24.04, ROS 2 Jazzy, rmw_zenoh, Eigen3, px4_msgs

---

## 1. Project status (snapshot)

| Item | Value | Evidence |
|---|---|---|
| Horizontal accuracy (bench 60–300 s) | 15.7 m RMS (σ4 map, r_trn=400) | tools/bench.sh + replay_scorer |
| Horizontal accuracy (replay A, baro off) | 15.2 m RMS, MAX 20.5 m | BENCH_NO_BARO=1 tools/bench.sh |
| Robustness (Monte Carlo) | 100/100 seeds; CEP50 14.9 m; CEP95 40.3 m; worst 31.1 m | tools/mc_run.sh (TEST-012) |
| Unit tests | 14/14, dual-build (-O2 and -O3) | tools/run_selftest.sh |
| Vertical channel (baro, WIP) | 4 m error vs truth when enabled | bench DBG `zt=` field |
| Open coupling | baro-on horizontal 24–28 m; root cause = attitude uncertainty | OI-002/OI-003 ledger (§10) |
| Last pushed commit | 949927d (visualization layer) | git log |
| Uncommitted WIP | OI-002: 16-D core, baro_sim, bench A/B, TEST-013 | working tree |

## 2. Repository layout

    gdn_workspace/
    ├── src/gdn_fusion/
    │   ├── include/gdn/
    │   │   ├── map_db.hpp          # terrain DB: binary grid, bilinear query + gradient
    │   │   └── eskf.hpp            # ROS-free ESKF core (15/16-state)
    │   ├── src/
    │   │   ├── eskf_node.cpp       # thin ROS 2 wrapper (composition + standalone)
    │   │   ├── map_viz_node.cpp    # RViz terrain mesh + path publisher
    │   │   ├── selftest.cpp        # 14 unit tests (no ROS)
    │   │   └── mc_campaign.cpp     # Monte Carlo robustness campaign (no ROS)
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── tools/
    │   ├── bench.sh                # deterministic replay bench (A/B baro switch)
    │   ├── bench_viz.sh            # bench + RViz overlay
    │   ├── mc_run.sh               # N-seed Monte Carlo runner
    │   ├── run_selftest.sh         # dual-optimization selftest gate
    │   ├── trn_sensor_sim.py       # simulated terrain-elevation sensor from truth
    │   ├── baro_sim.py             # simulated barometer (z_true + bias + noise)
    │   ├── replay_scorer.py        # horizontal error scorer vs truth
    │   ├── export_flight_csv.py    # bag truth -> CSV
    │   ├── make_sigma4_map.py      # σ4-smoothed terrain DB generator
    │   └── gdn.rviz                # RViz config (mesh + cloud + path)
    ├── data/
    │   ├── flight001/              # PX4 SITL replay bag (truth + IMU)
    │   └── maps/terrain_db_sigma4.bin
    ├── docs/                       # design notes, test reports
    └── env.sh                      # RMW/zenoh environment

## 3. Architecture

Principle: all navigation math is ROS-free and unit-testable; ROS 2 exists only
as a thin I/O wrapper; every claim is backed by a deterministic replay or a
statistical campaign. The core is vehicle-agnostic: no airframe model exists
anywhere in include/gdn/, so fixed-wing, multirotor and VTOL platforms share
one estimator (§1).

    [IMU + TRN meas + baro meas] -> eskf_node (thin wrapper)
            |                            |
            v                            v
       gdn::Eskf (core)  <--------  gdn::MapDb (terrain)
            |
            v
     /gdn/odom, /gdn/sigma  ->  scorer / RViz / downstream

Layers:
1. Core (include/gdn/): pure C++17 + Eigen, no ROS headers; compiled unchanged
   into selftest, mc_campaign and the node.
2. Node (src/eskf_node.cpp): topic bridging, parameters, DBG telemetry.
3. Simulation (tools/*.py): sensors synthesized from bag truth with realistic
   bias/noise so the filter is validated against honest measurements.
4. Validation (bench / MC / selftest): pre-registered metrics and decision rules.

Node topic contract:
| Dir | Topic | Type | Purpose |
|---|---|---|---|
| sub | /fmu/out/sensor_combined | px4_msgs/SensorCombined | IMU (accel + gyro) |
| sub | /fmu/out/vehicle_local_position | px4_msgs/VehicleLocalPosition | velocity for yaw transfer; truth-z init (OI-002) |
| sub | /gdn/trn_meas | std_msgs/Float64 | terrain-elevation measurement |
| sub | /gdn/baro_meas | std_msgs/Float64 | barometer altitude (NED, simulated) |
| pub | /gdn/odom | nav_msgs/Odometry | fused state (map frame) |
| pub | /gdn/sigma | std_msgs/Float64 | horizontal position sigma |

## 4. Filter formulation

Reference: Sola, "Quaternion kinematics for the error-state Kalman filter",
arXiv:1711.02508; gating and Joseph-form update per Bar-Shalom et al. (2001).

Nominal state: position p, velocity v, attitude q (body->map), accel bias ba,
gyro bias bg, plus baro bias bb in the 16-D WIP.

Error state (15-D base; index map):
    0-2 δp   3-5 δv   6-8 δθ   9-11 δba   12-14 δbg   15 δbb (OI-002 WIP)

Propagation: strapdown, sub-stepped at <=10 ms, stall-tolerant (dt clamped to
1 s with warnings); covariance growth P <- ΦPΦᵀ + Qc·h with PSD repair.

Updates (in order of availability):
1. Static coarse alignment (FeedStatic): motion-gated averaging of still IMU
   samples; roll/pitch from gravity, gyro bias from rate mean, accel bias from
   gravity residual; then ResetCovariance.
2. Yaw transfer (TransferAlignYaw): one-shot re-initialization from GNSS
   velocity heading once speed > 3 m/s (OI-003 removes this dependency).
3. TRN scalar elevation (ApplyTrn): H = [∂h/n, ∂h/e, 0 ...] from the map
   gradient; slope observability gate (slope_min), innovation gate
   (gate_sigma, gate_abs_m), per-state injection clamps, Joseph-form update,
   quaternion error-state reset (J matrix).
4. Barometer (ApplyBaro, WIP): H has 1 at δp_z (index 2) and δb_baro (15);
   observes the SUM z + bias (observability caveat in §10, OI-002).

Config knobs (EskfConfig / node params): r_trn=400 (σ=20 m), slope_min=0.02,
gate_sigma=3, gate_abs_m=40, clamp_dtheta_deg=0.2, clamp_dp=5, clamp_dv=0.5,
clamp_dva=5e-3, clamp_dvg_deg=0.05; baro: r_baro=4, q_baro_bias=1e-3,
p0_baro_bias=25, gate_baro_sigma=5.

Numerical hygiene: every covariance write passes Repair() (LLT check; on
failure, eigenvalue clamp to >=1e-9). Congruence transforms (ΦPΦᵀ, JPJᵀ)
preserve PSD by construction.

## 5. Terrain database

Binary format (little-endian):
    int32 nx, int32 ny, float64 lat0, lon0, dlat, dlon,
    then float32 alt[ny*nx] (row-major, row = latitude index).
Query: bilinear interpolation with analytic gradient of the bilinear patch
(selftest asserts planar maps give exact gradient and altitude).
Out-of-bounds queries return ok=false and are rejected by the filter.

REQ-DB-001: σ4 Gaussian-smoothed terrain (tools/make_sigma4_map.py).
Rationale: contour aliasing on fine texture defeats scalar TRN gating; σ4
keeps real relief while bounding gradient error. The MC campaign validates
100/100 seeds at r_trn=400 on σ4.

## 6. Sensors and simulation

Truth source: data/flight001 (PX4 SITL bag). Topics: sensor_combined,
vehicle_attitude, vehicle_global_position, vehicle_gps_position,
vehicle_local_position, vehicle_status.
NOTE: the bag contains NO barometer topic and px4_msgs/SensorCombined has NO
baro field — hence tools/baro_sim.py.

trn_sensor_sim.py: publishes /gdn/trn_meas = h_map(truth x,y) + noise.
baro_sim.py: publishes /gdn/baro_meas = z_true + 5 m bias + N(0, 1.5 m),
decimated to 10 Hz. Both deterministic (fixed seeds) for replay parity.

## 7. Build and run

Prereqs: Ubuntu 24.04, ROS 2 Jazzy, rmw_zenoh (env.sh), Eigen3, px4_msgs,
colcon, Python 3 with rclpy.

    cd ~/gdn_workspace
    source env.sh
    colcon build --packages-select gdn_fusion --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/setup.bash

Unit tests (dual-optimization gate):
    tools/run_selftest.sh            # builds -O2 and -O3, runs both binaries

Deterministic replay bench:
    tools/bench.sh                   # baro ON
    BENCH_NO_BARO=1 tools/bench.sh   # baro OFF (A/B isolation)
    # prints: RMS 60-300s / MAX / n from the scorer log

Visual bench:
    tools/bench_viz.sh               # + RViz (terrain mesh + path)

Monte Carlo:
    tools/mc_run.sh 100              # 100 seeds, CEP50/CEP95/worst

## 8. Validation ledger

Selftest (14): map load / planar gradient exact / planar altitude exact /
out-of-bounds rejected / static alignment converges / accel bias recovered /
gyro bias recovered / attitude recovered / static propagation stays put /
TRN accepted on observable slope / position converges along gradient /
unobservable axis not pumped / baro updates accepted / baro observable sum
converges.
TEST-011 precedent: the dual-build gate caught uninitialized static
accumulators (sa_/sw_/sa2_) that -O2 masked and -O3 exposed.
TEST-012: MC campaign (100 seeds) as the standing robustness gate.
TEST-013: baro update math (observable sum), added with OI-002.

Bench history (horizontal RMS 60–300 s):
    σ1 map, r_trn=144 : ~30 m (aliasing-limited)
    σ4 map, r_trn=400 : 15.7 m    <- committed baseline
    σ4, baro off (A)  : 15.2 m
    σ4, baro on  (B)  : 24.6–27.6 m   <- open coupling (OI-002/OI-003)
    σ4, partitioned   : 19.6 m horizontal BUT vertical diverges (+59 km) -> rejected

MC (σ4, r_trn=400): 100/100 seeds pass; CEP50 14.9 m; CEP95 40.3 m; worst 31.1 m.

## 9. Visualization

tools/bench_viz.sh launches map_viz_node + RViz (tools/gdn.rviz):
- TerrainMesh: Marker TRIANGLE_LIST (solid relief), republished at 0.5 Hz.
- TerrainCloud: optional height-colored PointCloud2 (off by default).
- GDN Path: nav_msgs Path projected at nominal AGL (terrain rel height + 80 m)
  because the vertical channel was unobservable pre-baro (OI-002).
Gotcha (Jazzy): the Marker display's topic property key is `Topic:`; a config
using `Marker Topic:` is silently ignored and the display subscribes to
nothing (root cause of the original "blank viewport" debugging round).

## 10. Open items ledger

OI-002 Vertical observability (barometer) — IN PROGRESS
  Done: 16-D state, ApplyBaro, TEST-013 (14/14), node wiring, baro_sim;
        vertical tracks truth within 4 m when enabled.
  Open: enabling baro degrades horizontal (15.2 -> 24–28 m); gyro-bias
        estimate inflates ~10x during climb transients.
  Diagnostics run: P health (asymmetry ~1e-11, min-eig > 0); H sparsity clean
        (nonzeros only at indices 2 and 15); partitioned update (zero
        cross-terms first, PSD asserted) -> horizontal protected but vertical
        diverges (+59 km, 5382 rejections) because vertical needs the
        attitude feedback path.
  Conclusion: the coupling is real physics through P cross-terms inflated by
        attitude uncertainty (st ≈ 6.5°). Root-cause fix = OI-003.
OI-003 Magnetometer yaw — NEXT (design-first, no code yet)
  Removes the GNSS-velocity yaw transfer; reduces attitude uncertainty;
  expected to shrink cross-terms and make OI-002 clean.
OI-004 Landing/terminal-phase observability — ACCEPTED LIMITATION (documented).
OI-005 Fine-texture robustness — MITIGATED by σ4 (REQ-DB-001).
OI-006 Velocity injection pumping — FIXED via clamp_dv.

## 11. Field manual (hard-won gotchas)

- bash: never `set -u` around `source install/setup.bash`; ROS scripts
  reference unset vars and, with stderr redirected, the shell dies silently.
- RViz Jazzy: Marker topic key is `Topic:` (see §9).
- px4_msgs: SensorCombined has no barometer field; check `ros2 bag info`
  before assuming a sensor exists in a bag.
- EKF math facts verified the hard way:
  * ΦPΦᵀ and JPJᵀ are congruence transforms: PSD in => PSD out.
  * Zeroing off-diagonal blocks of a PSD matrix preserves PSD.
  * Shrinking a diagonal block while leaving its cross-terms untouched
    violates the Schur complement and produces runaway indefiniteness (this
    was a bug in an early "partitioned update" draft, not physics).
  * A scalar barometer observes only (z + bias); test the observable sum,
    not the individual states, unless an independent altitude anchor exists.
- Terminal/paste hygiene: after any multi-line heredoc, verify on disk
  (wc -l, bash -n, python3 -m py_compile, tail) before trusting a run;
  garbled echo output has twice masked truncated files.
- Bench scorer cadence is n=12 samples in the 60–300 s window; treat deltas
  below ~3 m with caution; use A/B pairs and pre-registered decision rules.

## 12. Milestones

    56dcbde  v3 architecture: ROS-free core, thin node, selftest, MC infra
    18383e4  REQ-DB-001 σ4 terrain + TEST-012 report
    eec623e  r_trn=400 lock + make_sigma4_map.py reproducibility
    949927d  visualization layer (terrain mesh + path, bench_viz)
    (WIP)    OI-002 baro state: 16-D core, TEST-013, baro_sim, bench A/B

## 13. Roadmap

1. OI-003 magnetometer yaw: design doc -> core impl -> selftest -> bench/MC.
2. Re-run OI-002 experiments with tight attitude; choose full vs partitioned
   baro update on measured merit; then commit OI-002.
3. Live SITL confirmation flight (QGC) with the validated stack.
4. Optional: Gazebo heightmap world for an end-to-end demo.
5. OI-004 revisit only if a mission profile demands terminal-phase TRN.

## 14. References

- Sola, J. "Quaternion kinematics for the error-state Kalman filter", 2017.
- Bar-Shalom, Li, Kirubarajan. "Estimation with Applications to Tracking and
  Navigation", 2001 (gating, Joseph form).
- PX4 ROS 2 interface docs; RViz2 user guide (Jazzy).

---
Discipline: hypothesis -> one decisive experiment -> pre-registered decision
rule -> commit. No blind patches; read the file before editing it.
