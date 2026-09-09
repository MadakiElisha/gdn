# TEST-008: Database Resolution Sweep (ESKF Robustness)
- Objective: Determine REQ-DB-001 (Onboard DEM resolution limit).
- Method: World = fine terrain. DB = smoothed terrain (sigma 1.0, 1.5, 2.0, 4.0).
- Results:
  - sigma=1.0 (27m): RMS 20.8 m. Perfect lock.
  - sigma=1.5 (40m): Diverged off-map (1258 flat-skipped).
  - sigma=2.0 (54m): Diverged off-map (483 flat-skipped).
  - sigma=4.0 (108m): Locked to displaced contour (RMS ~2 km).
- Conclusion: Point-mass ESKF is highly sensitive to mid-frequency DB mismatch.
- Derived Requirement: REQ-DB-001 - Onboard DEM must be <= 30m resolution.
