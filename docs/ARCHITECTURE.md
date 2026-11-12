# GDN architecture and filter reference

This is the technical reference for the estimator. For the project overview see the [README](../README.md); for setup see
[SETUP.md](SETUP.md).

## Design principles

- **All navigation math is ROS-free** (`src/gdn_fusion/include/gdn/`, pure C++17 + Eigen)
  and is compiled unchanged into the unit tests (`gdn_selftest`), the Monte Carlo
  driver (`mc_campaign`) and the ROS 2 node.
- **ROS 2 is a thin I/O wrapper** (`src/gdn_fusion/src/eskf_node.cpp`): topics,
  parameters, telemetry.
- **Every claim is backed by a deterministic replay or a statistical campaign**, with
  pre-registered metrics and decision rules.
- **Vehicle-agnostic core**: the core contains no airframe model, so fixed-wing,
  multirotor and VTOL share one estimator. Platform specifics enter through sensor
  noise tuning and the terrain database. All validation to date is fixed-wing-like.

```
[IMU + TRN meas + baro + mag] -> eskf_node (thin wrapper)
          |                          |
          v                          v
     gdn::Eskf (core)  <-------  gdn::MapDb (terrain)
          |
          v
   /gdn/odom, /gdn/sigma  ->  replay_scorer / RViz / downstream
```

## Node topic contract

| Dir | Topic | Type | Purpose |
|---|---|---|---|
| sub | `/fmu/out/sensor_combined` | `px4_msgs/SensorCombined` | IMU (accel + gyro), drives propagation |
| sub | `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | **Truth/GNSS-derived**: the DBG line's truth column only; never fed to the filter (R1.2) |
| sub | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | truth yaw, logging only |
| sub | `/gdn/mag_meas` | `geometry_msgs/Vector3` | magnetometer, uT, body FRD |
| sub | `/gdn/agl_meas` | `std_msgs/Float64` | height above ground, tilt-corrected, m (`trn_model:=agl`, default) |
| sub | `/gdn/trn_meas` | `std_msgs/Float64` | terrain elevation under the vehicle, m (`trn_model:=ideal`, legacy) |
| sub | `/gdn/baro_meas` | `std_msgs/Float64` | barometric altitude, NED z, m |
| pub | `/gdn/odom` | `nav_msgs/Odometry` | fused state, frame `map` |
| pub | `/gdn/sigma` | `std_msgs/Float64` | horizontal position 1-sigma, m (1 Hz) |

Node parameters: `map_path` (relative paths resolve against `$GDN_ROOT`), `trn_model`
(`agl` or `ideal`: which measurement topic it uses), `filter_config` (default
`config/filter/default.yaml`, see Configuration below), and one parameter for every key
of that file, defaulting to the file's value. So `-p r_agl:=50` or
`-p mag_ref_down:=45` overrides one setting (SITL passes the Gazebo world's field as
`mag_ref_*`); the start-up log names the file, the fingerprint and the overrides.

IMU holes: each `sensor_combined` holds the mean rate over `gyro_integral_dt`. When
the time since the previous message is longer than that, samples were lost upstream.
The node then calls `Eskf::InflateForGap`, which widens the attitude covariance by
`gap_att_rate_scale` (2) times the current rate times the hole plus `gap_att_floor_deg`
(1 degree), and the velocity covariance by `gap_vel_accel` (2 m/s^2) times the hole. If
messages routinely cover much less than their spacing, it logs an error (on PX4, set
`IMU_INTEG_RATE` to the DDS rate).

`map_viz_node` draws the map as a terrain mesh in the RViz `map` frame (ENU, x east,
y north, z = 0 at the map centre), using the map's own metres per degree. Its
`path_mode` parameter chooses how GDN's path is drawn: `altitude` (GDN's own height,
moved by `datum_offset_m`) or `terrain` (the map height plus `terrain_clearance_m`).

In closed-loop SITL (`docs/SIMULATION.md`), `tools/sim_adapter.py` publishes the same
`/gdn/*` measurement topics from Gazebo's sensors, so the node runs unchanged.

## Filter formulation

Reference: Sola, *Quaternion kinematics for the error-state Kalman filter*,
arXiv:1711.02508. Gating and Joseph-form update follow Bar-Shalom, Li and Kirubarajan
(2001).

**Nominal state:** position p, velocity v, attitude q (body to local NED), accel bias,
gyro bias, baro bias, 3-axis mag bias.

**Error state (19-D):**

| Index | 0-2 | 3-5 | 6-8 | 9-11 | 12-14 | 15 | 16-18 |
|---|---|---|---|---|---|---|---|
| State | δp | δv | δθ | δb_a | δb_g | δb_baro | δb_mag |

**Propagation:** strapdown, sub-stepped at 10 ms or less. dt is clamped to 1 s in the
core; the node drops IMU gaps above 0.25 s. Covariance: P ← ΦPΦᵀ + Qc·h, then PSD repair.

**Updates, in the order they become available:**

1. **Static coarse alignment** (`FeedStatic`): motion-gated averaging of still IMU
   samples. Roll/pitch come from gravity, gyro bias from the rate mean, accel bias from
   the gravity residual. Then `ResetCovariance`.
2. **ZUPT** (`ApplyZupt`): zero-velocity pinning while `GroundMonitor`
   (`gdn/ground_monitor.hpp`) sees a still IMU, until 0.5 s of motion sets the airborne
   latch. IMU only, no GNSS (R1.2, TEST-018). ZUPT doesn't correct heading or the baro
   and mag biases. Used by the node and by `mc_campaign --startup=node`.
3. **Yaw alignment**: `AlignYawFromMag` (node) does a one-shot heading from the
   magnetometer. `TransferAlignYaw` (heading from GNSS velocity) is still used by
   `mc_campaign`'s default `--startup=truth`; `--startup=node` uses the mag as the node does.
4. **Magnetometer**, by `mag_fusion` (R1.4, TEST-021). `auto` (default,
   `ApplyMagAuto`): the 3-axis update while the field's strength and dip, averaged over
   about 1 s, match the reference; heading only otherwise. `full` (`ApplyMag`): 3-axis
   field against the reference vector, updating attitude and mag bias. `heading`
   (`ApplyMagHeading`): heading from the levelled field only, attitude gain projected on
   the body vertical so it can't tilt; assumes a calibrated compass and leaves the mag
   bias alone. `align`: initial yaw only.
5. **TRN scalar elevation** (`ApplyTrn`): H = [∂h/∂n, ∂h/∂e, 0 …] from the map gradient,
   which is the slope of the same bilinear surface the height comes from (R1.4).
   The update is gated by slope observability (`slope_min`) and innovation size
   (`gate_sigma`, `gate_abs_m`), with per-block gain limits (`LimitGain`, so the
   covariance matches the applied correction), Joseph form and a quaternion error reset.
6. **Height above ground** (`ApplyAgl`, R1.1): agl = (h0 - p_z) - h_map(n, e), so
   H = [-∂h/∂n, -∂h/∂e, -1, 0 ...]. h0 is the altitude datum (AMSL height of p_z = 0),
   set once from the map height under the vehicle plus the first AGL reading
   (`SetAltitudeDatum`). The baro bias is calibrated at the same moment
   (`InitBaroBias`), with a covariance consistent with b = baro - z. There's no slope
   gate: on flat ground it still observes altitude. The mag-bias rows of the gain
   are zeroed before the update, so the covariance matches what was applied.
7. **Barometer** (`ApplyBaro`): H has 1 at δp_z and δb_baro, so it observes the sum
   z + bias.

**Configuration** (R1.10, TEST-028). Every tuning value is in one file,
`config/filter/default.yaml`, with what each one is for and where its value came from.
The code holds none: `EskfConfig` and `GroundConfig` start out unset (NaN), so a filter
built without a loaded file goes visibly wrong. `gdn/filter_config.hpp` reads the file:

- **Strict.** Every key is required, an unknown key or one given twice is an error, and
  values are range-checked. A typo can't fall back to a value nobody chose.
- **One file for everyone.** The node (`filter_config`), the MC (`--config`) and the
  selftest load the same file, so they can't drift apart. Before R1.10 they had: the
  altitude datum was 5.0 m in the node and 5.4 m in the MC (now 5.4 m in both).
- **Traceable.** The node's log and every MC report carry a `filter:` line: the name,
  the file, a fingerprint (a 64-bit FNV-1a hash of every value, as 12 hex digits) and
  any overrides. Same fingerprint, same numbers. `mc_campaign --dump-config=FILE`
  writes the values run as a file `--config` reads back bit for bit;
  `tools/ci_check_mc.py --expect-filter default` fails a run that isn't the default
  file without overrides; `tools/score_requirements.py` refuses to put reports from
  different configurations on one scorecard.
- **Overrides** for experiments: `mc_campaign --set=KEY=VALUE` (and the older `--r-agl`,
  `--q-baro`, `--q-*`, `--mag` shorthands), or node parameters.
  `tools/config_sensitivity.py` halves and doubles each setting through `--set` and
  scores the result (TEST-028 ranks them).

Groups in the file: TRN (`r_trn`, gates), altimeter (`r_agl`, gates, `datum_sigma_m`),
barometer, IMU process noise, static alignment, starting covariance (`p0_*`,
`yaw_sigma_*`), on-ground detection and ZUPT (`ground_*`, `zupt_sigma_v`), magnetometer
(`mag_fusion`, noise, `mag_auto_*`, the reference field `mag_ref_*`), gain limits
(`clamp_*`) and IMU holes (`gap_*`, node only). Other vehicles or sites get their own
file in `config/filter/`; the selftest loads every file there.

**Numerical hygiene:** every covariance write passes `Repair()`: an LLT check, and on
failure an eigenvalue clamp to at least 1e-9.

## Terrain database

Binary format, little-endian:

```
int32 nx, int32 ny, float64 lat0, lon0, dlat, dlon, float32 alt[ny*nx]  (row-major, row = latitude index)
```

- **Local NED origin** = grid centre: `lat0 + ny·dlat/2`, `lon0 + nx·dlon/2`. The east
  scale uses cos(centre latitude). `MapDb` (C++) and `tools/gdn_paths.load_map` (Python)
  implement the same geometry, and the TRN simulator uses the latter.
- **Query:** bilinear altitude plus a central-difference gradient. Out-of-bounds returns
  `ok=false`, and the filter rejects the update.
- **Load:** rejects files whose size doesn't match the header.
- **REQ-DB-001** (σ4 Gaussian-smoothed synthetic map): contour aliasing on fine texture
  defeats scalar TRN gating. σ4 keeps the relief while bounding gradient error.

| Map | Source | Recipe |
|---|---|---|
| `terrain_db_sigma4.bin` | synthetic fractal, Zurich-centred | `generate_terrain.py` (seed 42) then `export_map_db.py` then `make_sigma4_map.py` |
| `zermatt.bin` | SRTM 1″ crop, Alps | `zermatt.tif` then `make_zermatt_map.py` |

`tools/make_data.sh --verify-maps` rebuilds both and checks them byte-for-byte.

## Sensors and simulation

- **Truth source:** `data/flight001`, a PX4 SITL rosbag2 MCAP with `sensor_combined`,
  `vehicle_attitude`, `vehicle_global_position`, `vehicle_gps_position`,
  `vehicle_local_position` and `vehicle_status`. It has no barometer topic, and
  `SensorCombined` has no baro field.
- **`trn_sensor_sim.py`**: publishes `h_map(truth x, y)` + noise on a 0.5 s grid, plus
  `/gdn/truth` for the scorer.
- **`baro_sim.py`**: `z_true + 5 m bias + N(0, 1.5 m)` at 10 Hz.
- **`mag_sim.py`**: `Rᵀ·m_ref + hard-iron + N(0, 2 uT)` at 20 Hz. The hard iron is the residual after compass calibration, 0.15 × (2, -1, 1) uT by default; `GDN_MAG_HARDIRON=1` gives the uncalibrated (2, -1, 1) uT used before R1.4.
- **`make_zermatt_bag.py`**: the synthetic Zermatt flight as a PX4-style bag, for bench
  smoke tests.

All simulators use fixed seeds for replay parity.

**IMU error model (R1.6, TEST-022, TEST-024 and TEST-025).** `gdn/imu_model.hpp` defines the model: white noise
per axis, Gauss-Markov bias drift and turn-on bias. It reads model files from
`config/imu/` and applies them (`ImuErrorSim`). The Monte Carlo uses it on flights that
have an error-free IMU (`imu_truth.csv`, written by `gen_zermatt_flight.py`); real logs
keep their recorded IMU (`mc_campaign --imu`). The default model is the H30WP's Allan
fit (`config/imu/h30wp.yaml`, from a 2 h bench log; `tools/allan_fit.py` fits the drift
as an upper envelope over 30 s to 10 min, so it doesn't promise a better IMU than was
measured where the GNSS-free windows are scored). Every MC report names the model on
its `imu:` line. The filter's IMU process noise is in the filter config (`q_accel_nd`,
`q_gyro_nd`, `q_accel_bias_rw`, `q_gyro_bias_rw`); the selftest (TEST-024) keeps it at
or above the H30WP's, with the bias random walks within 10x of it, and (TEST-025) not
below the Mid-360S's or OAK-D Lite's either, since LIO and VIO may feed it from those.
`config/imu/` also holds those two fits; `allan_fit.py` resamples uneven sample times as
the filter integrates them and stops at a log's first break. The Gazebo x500 already
carries PX4's white noise; model files for Gazebo arrive with R2.4.

**Scenarios: GNSS loss and start error (R1.7, TEST-023).** The requirements score time
since GNSS was lost, from a converged filter that keeps its covariance.
`Eskf::ApplyGnss` (horizontal position and velocity; no gate; baro and mag biases
untouched) lets the MC fuse a simulated GNSS until `mc_campaign --gnss-loss=T`. After
that the filter carries on with the covariance it had. `--gnss-free=300,600` pools the
5 and 10 minutes after the loss, as REQ-NAV-001 to 003 define them. `--start-error=M`
uses `Eskf::SetHorizontalPosition` to start M metres off with a stated sigma, and reports
REQ-ACQ-001 (under 100 m at 120 s after take-off). Every MC report has a `scenario:`
line. `data/flight_zermatt_long_csv` (15 minutes, `gen_zermatt_flight.py --minutes 15`)
gives the synthetic case room for 10 minutes without GNSS; flight001 (16 minutes) is
scored up to the local-position reset in its recorded truth at 967.7 s. The node has no
GNSS input yet; the PX4 handover belongs with the SITL scenarios (R2.8).

## Validation ledger

- **Selftest** (31 checks; `tools/run_selftest.sh` runs them at -O2 and -O3,
  `colcon test` runs them once):
  - Map DB: load, planar gradient/altitude exact, out-of-bounds rejected, east scale
    uses the map's latitude, truncated file rejected.
  - Static alignment: converges; accel, gyro and attitude recovered; static propagation
    stays put.
  - TRN: accepted on slope, converges along the gradient, unobservable axis not pumped.
  - Baro: updates accepted, observable sum converges.
  - AGL (R1.1): refused without a datum, accepted, holds altitude against a 15 m baro
    bias, estimates the bias, calibration sets a consistent covariance, moves position
    along the gradient.
  - Mag: updates accepted, yaw converges, bias estimated.
  - TEST-015 static alignment.
  - ZUPT: pins velocity and position; the control run drifts.
- **TEST-011:** the dual-build gate caught uninitialised static accumulators that -O2
  masked and -O3 exposed.
- **TEST-012:** Monte Carlo campaign, the standing robustness gate.

**Results after PR #1** (fresh clone, Ubuntu 24.04 + ROS 2 Jazzy):

| Check | Result |
|---|---|
| Zermatt MC, 24 seeds | CEP50 14.2 m, median per-seed RMS 16.8 m, 19/24 seeds under 100 m (chaotic tail = OI-009 false-contour lock) |
| flight001 MC, real altimeter model (TEST-017) | CEP50 26.4 m, CEP95 60.1 m, 100/100; with 15 m baro drift 50.8 m, 97/100 |
| flight001 bench, real altimeter model, baro on | 21.3 m RMS, 35.6 m max |
| Zermatt ROS bench (synthetic bag, baro off) | 13.1 / 20.1 m RMS in two runs (max 17.8 / 30.5 m); ROS timing makes runs non-identical, n=6 |
| flight001, old oracle model, after PR #1 (TEST-016) | MC CEP50 13.4 m, CEP95 37.0 m, 100/100; bench 31.5 m RMS with baro, 22.5 m without |

Historical flight001 numbers, measured before those fixes: bench 15.7 m RMS (σ4,
r_trn=400, early v3), later about 25 to 30 m; MC 100/100 seeds, CEP50 14.9 m, CEP95 40.3 m.

## Known gaps

These are the main things standing between this and a fielded GNSS-denied stack. See
[ROADMAP.md](../ROADMAP.md) (Phase 1 covers most of these) and `docs/open_items.md`.

- **Altitude limits terrain matching.** R1.1 replaced the idealised TRN measurement
  with the real AGL model. Altitude error now couples into horizontal error on gentle
  slopes (TEST-017), and baro drift faster than the bias random walk still costs
  accuracy.
- **The filter starts at the true position** (initial position sigma of about 10 m) and has no acquisition mode.
  Start errors above about 60 m fail (TEST-024: 29 to 82 flights in 100 fail from 150 m, 93 or more from 500 m).
- **Map error is synthetic.** Since R1.3 the MC (`--db-map`) and bench (`GDN_DB_MAP`)
  fly over a truth map and navigate with a different onboard map, built by
  `tools/make_db_map.py` (noise, bias, coarser cells). Both maps come from the same
  SRTM tile, though; a truly independent source (a 1 to 5 m DEM world against an SRTM
  onboard map) arrives with the Gazebo worlds (R2.2). Correlated map noise is not
  modelled in the filter, so it makes the filter overconfident (TEST-020).
- **No integrity monitoring:** a lost filter can report a small sigma.
- **Consistency:** fixed in R1.4 (TEST-021) for the known causes: slope and height
  from the same map surface, zeroed gain rows, gain limiting instead of correction
  clamps. Still missing: a model of correlated map error (5.5% over 3 sigma on
  SRTM-like onboard maps, TEST-020/021).
