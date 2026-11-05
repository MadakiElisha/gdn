# TEST-018: remove truth leaks from the node start-up (R1.2)

**Question.** Before R1.2, `eskf_node` ran ZUPT until GNSS speed passed 2 m/s, and in
`ideal` mode took its start altitude from `vehicle_local_position`, which is PX4's
GNSS-aided estimate. A GNSS-denied system can't have either. Can the node detect "on the
ground" from the IMU alone, and what does the GNSS rule cost today?

**Pre-registered pass rule.** Without GNSS input: ZUPT runs at about 10 Hz while parked,
stops within 0.05 s of a 2 m/s^2 take-off roll and within 0.15 s of a 0.5 m/s^2 one,
latches within 0.6 s and 1.0 s respectively, and doesn't latch on a 0.2 s bump. The
take-off velocity error after ZUPT is under 0.3 m/s. Through the node's start-up path,
the Monte Carlo is no worse than with the GNSS rule.

## What changed

- `gdn/ground_monitor.hpp` (new, ROS-free). "Still" means the 0.2 s mean of the
  accelerometer and gyro is within 0.3 m/s^2 and 0.02 rad/s of what they read at
  alignment (`Eskf::RestSpecificForce()`, `gyro_bias()`). Averaging removes sensor noise
  (0.15 m/s^2 per sample is about 0.03 over 20 samples), while any real acceleration
  moves the mean, a constant one included. Motion for 0.5 s sets the airborne latch;
  shorter motion only suspends ZUPT. Every doubt goes towards "moving", since a missed
  ZUPT costs a little drift and a false one corrupts velocity.
- `Eskf::ApplyZupt` no longer corrects heading (the body-frame component of attitude
  and gyro bias along the vertical) or the baro and mag biases. Zero velocity says
  nothing about them, but the noisy accelerometer sample in the propagation Jacobian
  couples them weakly to velocity, and a full-gain ZUPT walked yaw by 8 degrees in
  60 s parked. The Joseph update keeps the covariance consistent with the reduced gain.
  This is one of the R1.4 items, found here because it made correct ZUPTs harmful.
- `eskf_node`: the monitor drives ZUPT and the latch. `vehicle_local_position` is only
  the DBG line's truth column now. In `ideal` mode the start altitude is z = 0 (the
  local origin is the start point) and the first baro reading sets the baro bias.
  The dead transfer-yaw log block that read GNSS velocity is gone. DBG prints Euler
  angles with `atan2` (yaw in -180..180), not Eigen's `eulerAngles()`, which showed
  a yaw of 242 degrees as "62, pitch 180, roll 180".
- `mc_campaign --startup=node` runs the node's start-up path: magnetometer heading
  while parked, monitor-driven ZUPT, mag updates throughout (simulated as
  `mag_sim.py`: R^T m_ref + hard iron (2, -1, 1) uT + N(0, 2 uT), 10 Hz), TRN from
  yaw alignment on. `--ground=gnss` replays the pre-R1.2 rule for comparison, and
  `--mag=align` uses the magnetometer for the initial heading only.
  `--startup=truth` (heading from truth GNSS velocity at take-off) stays the default,
  so the CI regression baselines are unchanged.
- Tools: `bench.sh` and `bench_viz.sh` stop processes by exact name and script path
  (`pkill -f eskf_node` also killed any shell whose command line mentioned the node,
  and killing only the `ros2 run` wrapper left the node running after each bench).
  `replay_scorer.py` prints its full summary on Jazzy, where SIGINT in `spin()` raises
  `ExternalShutdownException` rather than `KeyboardInterrupt`, and logs at 1 Hz.

## Results: unit tests (selftest, -O2 and -O3)

The stub that reproduced the old node without GNSS (never ZUPT, never latch) failed 6
of the new checks. With the monitor and the ZUPT gain fix all pass:

| Check | Result |
|---|---|
| Parked 60 s, noisy IMU with bias | 10 Hz ZUPT, no latch |
| Heading change over 60 s parked with ZUPT | 8.06 deg before the gain fix, 0.15 deg after |
| 2 m/s^2 roll | ZUPT stops within 0.05 s, latch within 0.6 s |
| 0.5 m/s^2 roll | ZUPT stops within 0.15 s, latch within 1.0 s |
| 0.2 s bump of 1.5 m/s^2 | no latch, ZUPT resumes |
| Turning on the spot at 0.1 rad/s | ZUPT stops, latch |
| Take-off velocity error after 5 s at 2 m/s^2 | 0.05 m/s (0.58 before the gain fix); old GNSS rule 0.73 m/s |

## Results: Monte Carlo through the node's start-up path

Zermatt SRTM, 100 seeds, scored 60 to 300 s. "Gain fix" is the ZUPT change; "IMU
ground" is the monitor. The first row matches the audit's node-path harness
(AUDIT-2026-11, A-01) to the decimal, so `--startup=node` reproduces the node.

| Ground rule | ZUPT gain fix | Model | CEP50 | CEP95 | Seeds under 100 m |
|---|---|---|---|---|---|
| GNSS speed (before) | no | ideal | 68.6 m | 1227 m | 53 |
| GNSS speed | yes | ideal | 65.3 m | 1440 m | 57 |
| IMU | no | ideal | 21.2 m | 287 m | 87 |
| **IMU (after)** | **yes** | ideal | **21.1 m** | **368 m** | **88** |
| GNSS speed (before) | no | agl | 46.0 m | 706 m | 59 |
| GNSS speed | yes | agl | 38.7 m | 731 m | 60 |
| IMU | no | agl | 10.6 m | 287 m | 87 |
| **IMU (after)** | **yes** | agl | **10.8 m** | **288 m** | **87** |

For reference, the truth start-up (`--startup=truth`, no ZUPT, heading from GNSS
velocity) gives 14.5 m / 812 m / 82 (ideal) and 17.8 m / 744 m / 75 (agl).

Median airborne latch: 31.10 s with the GNSS rule, 30.53 s with the IMU. The take-off
roll starts at 30.1 s and reaches 2 m/s at 31.0 s.

flight001 (the recorded fixed-wing flight), 100 seeds, AGL model with 5 m baro drift.
The simulated magnetometer assumes level flight along the velocity vector. That holds for
the synthetic Zermatt flight, but flight001 banks and crabs, and its CSVs carry no attitude
truth, so 3-axis mag updates there fight the true attitude and every seed diverges
(0/100 with either ground rule: a simulation artefact, not a filter result). These runs
therefore use `--mag=align` (initial heading only, the node's `mag_fusion:=align`):

| Start-up | CEP50 | CEP95 | Seeds under 100 m |
|---|---|---|---|
| Node path, GNSS-speed ground (before) | 38.4 m | 102.2 m | 97 |
| **Node path, IMU ground (after)** | **24.9 m** | **60.8 m** | **100** |
| Truth start-up (CI baseline, TEST-017) | 25.6 m | 65.2 m | 100 |

On Zermatt the same switch costs accuracy (`--mag=align`: 16.3 m / 527 m / 70, against
10.8 m / 288 m / 87 with mag updates), so mag updates help when the mag model is right.

## Findings

1. **The GNSS latch was the main cost of the node's start-up path (A-01).** It held
   ZUPT for the first 0.9 s of the take-off roll, telling the filter "velocity is zero"
   while it accelerated at 2 m/s^2. The leftover velocity error then dragged the
   solution into false TRN locks. With the IMU monitor the node path beats the truth
   start-up the MC used until now: the AGL model goes from 46.0 m and 59/100 to 10.8 m
   and 87/100.
2. **The ZUPT gain fix barely shows in the MC**, because the magnetometer updates keep
   pulling heading back while parked. It matters when the magnetometer is weak or
   disabled (`mag_fusion:=align`), as the unit test shows. The two ideal-model rows
   with the monitor (CEP95 287 m against 368 m) differ by which few seeds lock falsely;
   the passing counts (87 and 88) are the same within noise.
3. **13 of 100 seeds still fail.** The tail (CEP95 288 m) is the overconfident
   covariance and false lock of audit A-02. R1.4 and R1.5 are the next items.

## Results: ROS bench

Zermatt bag (regenerated for px4_msgs 1.16), `trn_model:=agl` with baro and mag, one
two runs on 5 November 2026. The node latched at t = 30.5 s, with ZUPT stopping at the start
of the roll; altitude datum 3047.3 m AMSL, baro bias 4.81 m.

| | RMS 60 to 300 s | MAX 60 to 300 s | P95 whole run |
|---|---|---|---|
| This branch, run 1 | 11.3 m | 23.8 m | 21.1 m |
| This branch, run 2 (second PC session) | 20.8 m | 54.6 m | n/a |

Before R1.2 the bench on this bag gave between 18.9 m and 68.0 m on two runs (old
build, px4_msgs 1.15, scorer summary from 6 to 11 samples), and the audit measured
hundreds of metres. Each bench run is a single sample of ROS timing (the two runs differ
by 2x), so the MC above is the measurement; the bench shows the node runs the same path end to end.

## Limits and follow-ups

- The latch is one way. Landing and re-take-off need the flight regime from the
  autopilot (R2.5).
- The reference is the alignment pose. A vehicle re-parked at a different tilt reads as
  moving and latches early (the safe direction). Creeping at under about 0.3 m/s^2 is
  not detected.
- The synthetic Zermatt IMU is smooth. Engine vibration on a real airframe raises the
  per-sample noise; the window mean should still sit well inside 0.3 m/s^2, but this
  needs checking on real logs (R1.6, Phase 6).
- flight001 needs attitude truth in its CSVs (export `vehicle_attitude`) before the MC
  can simulate a realistic magnetometer for it. Until then use `--mag=align` there.
- The MC's default start-up is still `truth`, so CI keeps its baselines. This PR adds a
  CI guard for `--startup=node`; switching the default is a separate decision.

## Reproduce

```bash
tools/run_selftest.sh
tools/mc_run.sh 100 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node
tools/mc_run.sh 100 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node --ground=gnss
tools/mc_run.sh 100 data/flight001_csv data/maps/terrain_db_sigma4.bin --model=agl --baro-drift=5 --startup=node --mag=align
GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin tools/bench.sh
```
