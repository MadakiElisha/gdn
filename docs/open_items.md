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
A cheaper lead from TEST-028 (R1.10): on SRTM-like maps, doubling `r_agl` (10 to 14 m
sigma) raised the flights never over 100 m from 37 to 47 of 100 and cut the samples
over 3 sigma from 6.3% to 2.6%. `r_agl` was tuned on a perfect map (TEST-017). Not
applied yet: it needs its own sweep, including the perfect-map regressions.

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

**TEST-032 (2026-11-16, dev image, rendered radar).** Three Stage A/B flights, each
replayed exactly (`gdn_replay`) with one change at a time:
- **Measurements:** the altimeter (0.32 m RMS against the map at the true position), the
  IMU stream (100% covered) and the biases are fine. *Correction (TEST-034): the
  altimeter figure was the landing over the pad. In cruise GDN's altimeter input is
  6 to 9.5 m RMS off the height straight below, from the tilt (OI-016).*
- **What goes wrong:** velocity isn't observed. GDN's speed drains on straight legs (10
  to 2 to 3 m/s within a minute): IMU propagation through a slightly wrong pitch, and the
  altimeter updates through the cross-covariances.
- **Overconfidence:** with altimeter updates on, the error is over GDN's own 3σ 15 to 81%
  of the time; with them off, never.
- **The runaway at the end of a long flight:** when the estimate is 1 to 2 km off, it
  leaves the map, every altimeter reading is skipped, and it runs away at 20 to 40 m/s on
  the IMU and baro. That's the "hallucination" seen in RViz; landing adds only the last
  seconds (the one-way latch; R2.5b re-arms ZUPT at landing, TEST-039).
- **Settings:** no setting fixes it. `r_agl` 400, heading-only or align-only mag, and
  altimeter off each win on one flight and lose on another, from 25 m to 1.7 km. So one
  SITL flight can't rank a setting. `r_agl` 400 is the only one never clearly worse (806 →
  638, 105 → 109, 521 → 25 m), the same lead as OI-009: test it in that MC sweep.
- **Code tests:** keeping the altimeter off the attitude (or the attitude and IMU biases)
  helps a little on one flight. Not kept.
- **The fix stays velocity aiding (Phase 3)** plus integrity monitoring (R4.4) to catch
  "confident and wrong".
- **Stage B fixed:** until TEST-032, Stage B (`GPS_OFF_AT`) never actually cut PX4's GPS.
  PX4's failure injection isn't acted on under Gazebo Harmonic. The cut now uses
  `EKF2_GPS_CTRL`, and `tools/check_gps_cut.py` checks it.

### OI-013: filter IMU white noise not yet set from in-flight data (open, bias part settled)
The H30WP's bench fit (TEST-024, `config/imu/h30wp.yaml`) settled the bias random walks.
The accel white noise still needs the IMU in flight.
- **Bias random walks (settled, TEST-024).** Against the H30WP, `q_accel_bias_rw` 5e-4
  is 5.3x and stays. `q_gyro_bias_rw` was 0.01 deg/s/sqrt(s), 44x, and is now 0.001
  (4.4x): Zermatt 10 minutes after GNSS loss goes from CEP95 32 to 22 m, max 1468 to 77 m
  and 0.93% to 0.00% over 3 sigma; SRTM-like maps from 202 to 81 m. The selftest keeps
  both at or above the H30WP's and within 10x. (TEST-022's stress case, PX4's pre-2025
  Gazebo drift, would need `q_accel_bias_rw` 4e-3; the H30WP drifts far less.)
- **Accel white noise (open, TEST-023 and TEST-024).** After a GNSS handover on
  flight001's real IMU the filter is overconfident for the first minutes. With the new
  gyro setting it is more accurate but less honest: CEP95 73 to 63 m, 8.6% to 13.1% over
  3 sigma in the first 5 minutes (10 minutes: 4.5% to 6.6%). `q_accel_nd` 0.2 brings
  that to 0.09% but fails Zermatt (2.35% over 3 sigma, 98/100 flights); 0.1 costs
  SRTM-like maps (CEP95 53 to 104 m). The bench H30WP is 33x below the current 0.02, so
  what's missing is vibration and flight conditions, not the sensor.

Set `q_accel_nd` from H30WP flight data (Phase 6), then re-run the TEST-024 sweep. The
CI's flight001 GNSS-loss step is a regression guard until then (15% over 3 sigma in the
first 5 minutes; 13.09% on its 24 flights).

### OI-014: altitude follows the onboard map's error (open)
Found by TEST-026 (R1.8). With an SRTM-like onboard map, altitude error p95 is 10.2 m on
Zermatt and 15.2 m on flight001, against REQ-NAV-010's 5 m. On a perfect map it is
1.5 m. With the `agl` model, height comes from the radar altimeter plus the map's
terrain height, so the map's bias and correlated error (3 m bias and about 6 m noise in
the recipe) go straight into altitude. Altitude had only been measured on perfect maps
before, so this was hidden. Likely direction: weigh the barometer more against TRN height
where the map is uncertain, or estimate a map height bias. Re-check with the real second
DEM (R2.2); fix with the TRN and integrity work (Phase 4).

### OI-015: the simulated radar beam lands about 1 m from its aim (open)
Found by TEST-033 (R2.2). Gazebo Harmonic's single-beam `gpu_lidar` (the x500's radar
altimeter) reads the terrain about 1.0 to 1.3 m south-west of the point straight below
it at 100 m range (a 0.6 degree pointing offset). Fitting range errors against the
surface slope gives the same offset on all six worlds, so it's the sensor, not the
worlds' placement. Removing it leaves 0.24 to 0.42 m, the sensor's 0.3 m noise.
- **Effect on hills:** centimetres to a metre of range.
- **Effect on Zurich's building edges** (20 m steps): up to 3.7 m.
- **Effect on GDN:** far below its 10 m altimeter sigma.

`tools/check_sim_radar.py` now allows each point the error a 1.5 m offset explains at its
slope, and fails if the fitted offset passes 2 m (a misplaced world).
- **Next step:** check whether the offset turns with the sensor's yaw (a GpuRays
  half-pixel?), before R2.4 builds the TFmini-S and Mid-360S on the same sensor type.

### OI-016: GDN's altimeter input assumes flat ground under a tilted vehicle (open)
Found by TEST-034 (R2.3). `sim_adapter.py` turns the radar's slant range into a height
above ground as range times the cosine of the tilt, and GDN takes that as the height
straight below. On a slope that's wrong whenever the vehicle is tilted: the beam lands
beside the point below, on higher or lower ground.
- **The radar itself is right.** Cast along the beam at the true attitude, it matches
  the terrain to 0.36 to 0.67 m RMS on all six TEST-034 flights (the sensor's 0.3 m noise
  plus OI-015).
- **The conversion isn't.** Against the height straight below, by tilt (Zermatt): 0.75
  to 1 m level, 2.2 to 3.8 m at 5 to 15 degrees, 4.2 to 5.7 m at 15 to 25, 7.4 to 12 m
  at 25 to 35; on Emmental up to 2.8 times as much.
- **The multirotor is hit hardest,** not the planes: cruising at 10 m/s, the x500 is
  tilted 15 degrees or more about half the time. The planes fly level on the legs and bank only
  in turns.
- **What GDN got over whole flights** (adapter, every reading): x500 6.0 m RMS on
  Zermatt and 9.5 m on Emmental, worst 10 s windows 23 m; VTOL 2.2 and 4.6 m; cessna 3.0
  and 5.1 m. GDN's altimeter sigma is 10 m, and these errors are correlated, not noise.
- **Earlier reports missed it.** TEST-031, TEST-032 and TEST-033 gave the altimeter as
  0.29 to 0.34 m RMS "in flight". Those figures were the adapter's last 10 s report of
  each run, which is the landing over the level pad. `sim_run.sh` now prints the whole
  flight's figure.
- **Possibly part of OI-012** (the x500's runaway, overconfident with altimeter updates
  on). Not tested yet. Next step: fly the x500 a few times with
  `GDN_SIM_ADAPTER_ARGS="-p agl_source:=truth"` (the map at the true position plus 0.3 m
  noise, no tilt error) against as flown, and compare GDN's drift. Runs vary a lot (the
  same x500 route scored 33 m and 4 km over the whole run on two flights in TEST-034),
  so it needs several.
- **Fix:** predict the measurement along the beam (attitude and map) in GDN's TRN
  update, instead of converting it to a height straight below. A real radar altimeter's
  wide beam returns the nearest ground rather than a point; that model is R8.4's. R8.2
  ("nadir-pointing rangefinder") had this down for the planes; it applies to the
  multirotor now.

### OI-017: GDN takes faulty inputs as good, and a stuck baro locks it out for good (open)
Found by TEST-041 (R2.6b): faults on GDN's inputs in SITL, each one in GDN's own record
(`docs/test_reports/test041_data/gdn_response.txt`).
- **No fault detection of its own.** GDN applied every stuck altimeter and stuck baro
  reading. Its innovation gate let through:
  - a third of a magnetometer noise burst (sigma 20 uT);
  - almost all of a baro one (sigma 5 m);
  - a fifth of a stuck magnetometer, the rest gated.

  A stuck or noisy IMU raised nothing.
- **A stuck baro locks the baro out, for good.**
  - **During:** on the x500, GDN followed the 30 s stuck value while the vehicle changed
    height.
  - **After:** the real readings came back 71 m off, and the gate rejected every one of
    them to the end of the flight. Of the altimeter's after it, 153 were applied, 152
    gated and 297 off the map.
  - **The result:** its vertical ran away: the baro innovation reached 0.9 km by the
    landing (2.2 km on a first flight). Its horizontal error reached 18 km (26 km on the
    first), against 1.6 km on the same route without faults.
  - **What's missing:** there's no recovery: no re-acceptance after a run of rejections,
    no reset from another source.
- **An IMU gap over 0.25 s is skipped, not propagated.** A 1 s IMU dropout (cessna)
  ended in "IMU sample dropped: dt 1.008 s". The filter dropped that second (about 25 m
  at cruise) without widening its uncertainty for it, and said so only in the run log,
  not in `gdn.log`. Up to 0.25 s it propagates and widens (TEST-029).
- **Fix (R4.x integrity, with R2.7's scoring):**
  - per-sensor health: stuck, rate and innovation-run checks;
  - re-acceptance after a run of gate rejections, with the covariance widened;
  - propagation across a longer IMU gap with honest uncertainty, or a declared outage.

  The scenarios to judge it are in `config/scenarios/*_faults.yaml`.

### OI-018: GDN's baro and magnetometer innovations are far smaller than it predicts (open)
Found by TEST-042 (R2.7a), from GDN's own update records on ten SITL scenarios.
- **The NIS:** for the applied baro and 3-axis magnetometer updates, the mean NIS
  ranges from 0.001 to 0.36 of its expected value (1 and 3) over the ten flights:
  - on the x500, mostly 0.02 to 0.05;
  - on the planes, about 0.001 for the magnetometer.

  So the predicted innovation spread is about 5 to 7 times the innovations GDN actually
  sees on the x500, and about 30 times for the planes' magnetometer.
- **Sensor-specific on the long flights:** on the eight that flew 4 minutes or more,
  the altimeter's NIS is near its expected value (0.5 to 1.5), while the baro's and the
  magnetometer's stay at 0.001 to 0.36. The same filter state is behind all three, so
  that points at the baro and magnetometer noise settings (R).
- **On the short GNSS-cut flight** (148 s, error inside its 3 sigma: 0% over, NEES
  median 2.9), every sensor's NIS is low, the altimeter's too (0.02). That points at a
  large predicted state uncertainty early in the flight, not at the sensors.
- **What it means:** GDN gives these two sensors far less weight than their real noise
  deserves. A filter this cautious with its baro is also easy to pull off by a stuck
  one (OI-017).
- **Not separated per update yet:** the measurement noise (R, the filter config's baro
  and mag sigmas) against the predicted state uncertainty (HPH'). Next step: log both
  parts per update and refit R from in-flight innovations. That's the in-flight
  noise work of OI-013, and R2.5's agreed "no noise tuning" until then.
- **Also seen:** the ZUPT's NIS after landing is 23 to 24 times its expected value on
  flights where GDN had drifted. The stillness update hits a filter that's far off; it's
  a consequence of OI-012, not a cause.

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
