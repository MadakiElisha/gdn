# Open items ledger

One entry per issue, numbered OI-xxx in the order they were raised. Keep closed items
here with their resolution, so the history stays readable. New work items belong in
[ROADMAP.md](../ROADMAP.md); this ledger is for problems and questions found along the
way.

Status values: **open**, **deferred**, **accepted** (known limitation, not being
fixed now), **closed**.

## Open and deferred

### OI-004: late-flight and landing-phase error growth (accepted)
On the flight001 replay, error grows late in the flight and passes 1 km after about
t = 950 s. The terrain near the landing area has a poor TRN signature, and the
vehicle is low and slow. It was first suspected to come from overlapping bag
playbacks (TEST-009). TEST-012 later attributed it to the landing terrain.
Accepted as a limitation for now. It should be revisited once the odometry layer
(ROADMAP Phase 3) gives velocity aiding during approach.

### OI-009: false-contour lock on Zermatt (open)
In the Zermatt Monte Carlo, about 5 of 24 seeds lock onto a parallel contour and stay
there, with errors in the kilometres. A point-gradient filter can't tell similar
contours apart once the error is larger than the terrain's correlation length.
Remedy: a multi-hypothesis or batch acquisition stage before the ESKF (ROADMAP R4.1).

### OI-012: GDN drifts on the x500 in closed-loop SITL (open)
First closed-loop runs (PX4 v1.16, x500, Zermatt, Stage A): PX4 flies the route
fine, but GDN's horizontal error reaches about 2 km within 4 minutes. The same
filter holds 21 m RMS on the fixed-wing `flight001` replay. Found and fixed so far:
- magnetometer units and axes in `sim_adapter.py` (Gazebo publishes gauss, rotated
  from a field vector with odd axes);
- PX4 integrating the IMU at 250 Hz while uXRCE-DDS forwards 100 Hz, so 60% of the
  motion never reached GDN (`IMU_INTEG_RATE 100` in `sim_run.sh`, plus a node error);
- single lost IMU messages (about 0.2 s) during fast manoeuvres left a 15 to 20 degree
  attitude error that the covariance didn't know about, after which the magnetometer
  gate rejected every correction (`Eskf::InflateForGap`, with a unit test).
What's left: the attitude still runs 5 to 12 degrees off in forward flight and the
velocity follows. The gyro alone integrates to within 1 degree over 10 s, so the
error comes in through the updates. Fusing the magnetometer for initial yaw only
(`mag_fusion:=align`) cuts the drift about 5x, so the 3-axis mag update trading tilt
against the mag-bias states is one cause (R1.4, heading-only mag). The larger one is
that nothing observes tilt well on a slow multirotor that pitches hard: TRN alone is
too weak, where the fixed wing got away with it. Velocity aiding (Phase 3) is the
real fix. The new IMU-coverage check also shows the `flight001` recording covers only
50% of the IMU time (4 ms integrals every 8 ms), so the fixed-wing baseline was
earned on half the IMU data; future recordings should set `IMU_INTEG_RATE`.
R1.4 (TEST-021) adds heading-only fusion and makes `mag_fusion:=auto` the default:
3-axis while the field looks like the Earth field, heading only otherwise. A 6 uT
body-z disturbance now tilts a still vehicle by 1.55 degrees (3-axis: 2.52, heading
only: 0.00); auto's tolerances should be tuned on these SITL runs. Not yet re-run on
the x500.
Reproduce with `tools/sim_run.sh`, then `tools/sim_replay.sh` on the bag
it records. Note that bag replays of the same flight vary by roughly 40% between
runs because playback timing isn't deterministic.

### OI-013: filter IMU process noise not yet set from a real IMU (open)
`EskfConfig::q_accel_bias_rw` (5e-4 m/s^3/sqrt(Hz)) and `q_accel_nd` (0.02 m/s^2/sqrt(Hz))
are tuned values.
- **Bias random walk (TEST-022).** With PX4's pre-2025 Gazebo bias drift (0.006, tau
  300 s) the filter falls to 94/100 Zermatt flights and 8.7% over 3 sigma. 4e-3 restores
  100/100 and 0.43%, and improves the real flight001 IMU (CEP95 61.5 to 55.7 m), but costs
  a little on SRTM-like onboard maps (96 to 94 flights).
- **White noise (TEST-023).** After a GNSS handover on flight001's real IMU, the filter
  is overconfident for the first minute: 8.6% over 3 sigma in the first 5 minutes,
  83% in the first 30 s. `q_accel_nd` 0.1 brings that to 0.51% (0.2: 0.09%) and improves
  CEP95 (73 to 62 m); the bias setting alone doesn't (3.1% at 2e-2). It barely changes
  Zermatt. Scored from take-off, flight001 goes from 0.00% to 1.00% over 3 sigma and
  100 to 99 flights.

Set both from the H30WP Allan fit (R1.6, `config/imu/README.md`) plus in-flight data,
then re-run the TEST-022 and TEST-023 sweeps. The CI's flight001 GNSS-loss step is a
regression guard until then.

## Closed

### OI-007: Gazebo heightmap visuals broken (closed)
First seen under WSL2 software rendering. On the dev PC (native Ubuntu, RTX 5060) it
came back in two forms: Ogre2's heightmap shader failed to compile and crashed the
server (the same failure reproduces on Mesa), and Ogre1 rendered the heightmap with
geometry that didn't match the collision, so the simulated radar altimeter was up
to 177 m off. Fixed by drawing the terrain as a mesh on the same grid
(`make_sim_world.py`); collision stays a heightmap. `tools/check_sim_radar.py`
now checks it: within 1.4 m at 26 points with both renderers.

### OI-002: vertical channel unobservable (closed)
With scalar TRN only, z drifted. Added a baro bias state (16th state), then folded it
into the 19-state core. Note that a scalar baro observes only z plus bias. Vertical now
tracks truth within about 4 m on the bench. Test: TEST-013 (selftest).

### OI-003: heading from GNSS velocity (closed)
Heading used to come from a one-time GNSS velocity transfer. It's replaced by
magnetometer alignment at start-up plus 3-axis mag-bias states. Test: TEST-014
(selftest). The Monte Carlo driver still uses velocity transfer, and the node no longer
does.

### OI-005: fine-texture and real-terrain robustness (closed)
The bench world, the database and the smoothing were all identical, so the filter had
never been tested on real relief. Closed by the Zermatt SRTM campaign (commits 85e2d52
and 742b5ca): CEP50 about 14 m on a synthetic flight.

### OI-006: attitude and velocity oscillation under TRN updates (closed)
Pitch oscillated from about t = 352 s on the bench, while position was unaffected. The
cause was unclamped velocity injection. Fixed with `clamp_dv = 0.5 m/s` per update
(v3).

### OI-010: stale nodes between bench runs (closed)
`ros2 node list` showed nodes from earlier runs. This was partly stale ros2 daemon
state and partly processes that survived. `bench.sh` now force-kills with `pkill -9`
before starting.

### OI-011: Zermatt map mirrored north to south (closed)
A hand-patched `zermatt.bin` had its rows flipped while `lat0` still pointed at the
north edge, so earlier Zermatt numbers were on mirrored relief. The map is now
regenerated from the GeoTIFF by `tools/make_zermatt_map.py`. It's tracked in git with a
checksum, and `tools/make_data.sh --verify-maps` rebuilds it byte for byte.

## Not recorded

There is no record of an OI-008. If you remember what it was, add it here.
