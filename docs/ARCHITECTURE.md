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
| sub | `/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | **Truth/GNSS-derived**: ground ZUPT and airborne latch; initial altitude only in `trn_model:=ideal` (see Known gaps) |
| sub | `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | truth yaw, logging only |
| sub | `/gdn/mag_meas` | `geometry_msgs/Vector3` | magnetometer, uT, body FRD |
| sub | `/gdn/agl_meas` | `std_msgs/Float64` | height above ground, tilt-corrected, m (`trn_model:=agl`, default) |
| sub | `/gdn/trn_meas` | `std_msgs/Float64` | terrain elevation under the vehicle, m (`trn_model:=ideal`, legacy) |
| sub | `/gdn/baro_meas` | `std_msgs/Float64` | barometric altitude, NED z, m |
| pub | `/gdn/odom` | `nav_msgs/Odometry` | fused state, frame `map` |
| pub | `/gdn/sigma` | `std_msgs/Float64` | horizontal position 1-sigma, m (1 Hz) |

Node parameters: `map_path` (relative paths resolve against `$GDN_ROOT`), `trn_model`
(`agl` or `ideal`), `r_trn`, `slope_min`, `gate_sigma`, `gate_abs_m`, `clamp_dv`, and
`mag_ref_north`, `mag_ref_east`, `mag_ref_down` (the local field in uT; the defaults
are the bench's, SITL passes the Gazebo world's), and `mag_fusion` (`full`, or `align`
to use the magnetometer for the initial yaw only; a diagnostic for OI-012).

IMU holes: each `sensor_combined` holds the mean rate over `gyro_integral_dt`. When
the time since the previous message is longer than that, samples were lost upstream.
The node then calls `Eskf::InflateForGap`, which widens the attitude covariance by
twice the current rate times the hole plus 1 degree, and the velocity covariance by
2 m/s^2 times the hole. If messages routinely cover much less than their spacing, it
logs an error (on PX4, set `IMU_INTEG_RATE` to the DDS rate). Defaults come from `EskfConfig`, so
the node, selftest and Monte Carlo always run the same configuration.

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
2. **ZUPT** (`ApplyZupt`, node only): zero-velocity pinning on the ground until the
   airborne latch trips.
3. **Yaw alignment**: `AlignYawFromMag` (node) does a one-shot heading from the
   magnetometer. `TransferAlignYaw` (heading from GNSS velocity) is still used by
   `mc_campaign`.
4. **Magnetometer** (`ApplyMag`): 3-axis field against a fixed reference vector.
   Updates attitude and mag bias.
5. **TRN scalar elevation** (`ApplyTrn`): H = [∂h/∂n, ∂h/∂e, 0 …] from the map gradient.
   The update is gated by slope observability (`slope_min`) and innovation size
   (`gate_sigma`, `gate_abs_m`), with per-state injection clamps, Joseph form and a
   quaternion error reset.
6. **Height above ground** (`ApplyAgl`, R1.1): agl = (h0 - p_z) - h_map(n, e), so
   H = [-∂h/∂n, -∂h/∂e, -1, 0 ...]. h0 is the altitude datum (AMSL height of p_z = 0),
   set once from the map height under the vehicle plus the first AGL reading
   (`SetAltitudeDatum`). The baro bias is calibrated at the same moment
   (`InitBaroBias`), with a covariance consistent with b = baro - z. There's no slope
   gate: on flat ground it still observes altitude. The mag-bias rows of the gain
   are zeroed before the update, so the covariance matches what was applied.
7. **Barometer** (`ApplyBaro`): H has 1 at δp_z and δb_baro, so it observes the sum
   z + bias.

**Configuration** (`EskfConfig` defaults):

| Group | Values |
|---|---|
| TRN | `r_trn`=400 (σ=20 m), `slope_min`=0.02, `gate_sigma`=3, `gate_abs_m`=40 |
| AGL | `r_agl`=100 (σ=10 m, sensor plus map error), `gate_agl_sigma`=3, `gate_agl_abs_m`=40; node `datum_sigma_m`=5 |
| Clamps | `clamp_dtheta_deg`=180 (relaxed for mag alignment), `clamp_dv`=0.5, `clamp_dva`=5e-3, `clamp_dvg_deg`=0.05, `clamp_dvm`=5 |
| Baro | `r_baro`=4, `q_baro_bias`=0.05 (was 1e-3; see TEST-017), `p0_baro_bias`=25, `gate_baro_sigma`=5 |
| Mag | `r_mag`=4, `q_mag_bias`=1e-5, `p0_mag_bias`=100, `gate_mag_sigma`=5; reference field N/E/D = 21.5/1.3/42.3 uT (WMM, Zurich) |

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
- **`mag_sim.py`**: `Rᵀ·m_ref + hard-iron (2, -1, 1) uT + N(0, 2 uT)` at 20 Hz.
- **`make_zermatt_bag.py`**: the synthetic Zermatt flight as a PX4-style bag, for bench
  smoke tests.

All simulators use fixed seeds for replay parity.

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
  Start errors above about 60 m fail.
- **The Monte Carlo uses the same map for world and database.** Realistic DEM error
  is not modelled.
- **Truth leaks in the node:** GNSS-derived `vehicle_local_position` drives ZUPT, the
  airborne latch and the initial altitude.
- **No integrity monitoring:** a lost filter can report a small sigma.
- **Consistency:** `ApplyTrn` zeroes some state corrections but still contracts their
  covariance; the clamps do the same.
