- OI-002: vertical channel unobservable (no baro state); z drifts. Add 16th state.
- OI-003: yaw via one-time GNSS velocity transfer; replace with magnetometer.
- OI-004: late-flight divergence on replay; verify single-player full-bag run.
- OI-005: bench world == DB == sigma1; fine-texture world robustness untested (TEST-007 family).
- OI-006: attitude/velocity oscillation under scalar TRN updates (DBG t~352+);
  suspect unclamped dv injection; position channel unaffected through mission.
  First experiment: clamp dv to 0.5 m/s per update; re-run bench.sh.
