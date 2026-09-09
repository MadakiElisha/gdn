# TEST-002: Unaided strapdown INS baseline drift
- Dataset: data/flight001 (1049 s, fixed-wing SITL, GPS-on then GPS-denied)
- Method: pure strapdown integration of /fmu/out/sensor_combined (100 Hz),
  identity initial attitude, no alignment, no bias calibration, no aiding.
- Result: North error -245,000 m at t=1049 s (see phase3_naive_ins_drift.png).
- Error growth: ~quadratic => dominant constant accel error ~0.5 m/s^2
  (tilt/bias class), consistent with MEMS-grade IMU theory.
- Requirement (SRD REQ-P-001): <= 100 m over mission.
- Conclusion: unaided INS is unbounded and unusable; absolute aiding
  (map matching) is mandatory. Calibration (TEST-003) reduces early drift
  but cannot bound it. Fusion (Phase 5) must bound it.
