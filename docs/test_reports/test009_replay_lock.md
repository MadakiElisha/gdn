# TEST-009/010: C++ ESKF node-in-the-loop replay bench - LOCK CONFIRMED
- Harness (TEST-010): flight001 bag -> eskf_node + trn_sensor_sim (0.5 s
  timestamp grid) + replay_scorer; no sim, no stalls, deterministic.
- Result: RMS horiz err 60-300 s = 30.4 m, MAX = 56.7 m; cadence band
  0.7-43.5 m through mid-flight; 0 rejects over first 1400 updates.
- Parity: Python TEST-006 34.6 m / TEST-008 20.8 m. C++ == validated math.
- Key fix: transfer heading alignment treated as RE-INITIALIZATION
  (x=0, fresh P) - error states live in the map frame, not in the nominal.
- Open items: OI-002 vertical/baro state; OI-003 magnetometer yaw;
  OI-004 late-flight (>700 s) divergence - suspect overlapping playbacks,
  to be re-tested with single full-bag playback.
