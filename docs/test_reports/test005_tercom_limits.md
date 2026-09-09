# TEST-005: Gated batch TERCOM on loiter trajectory - LIMITS
- Dataset flight001 (loiter circle ~300 m over single terrain feature).
- Batch TERCOM (10 s windows, 10 s cadence, gated resets):
  most windows REJECTED (ratio~1.0, ambiguous); accepted fixes after
  t>100 s spurious (314-1750 m err) once drift exceeded search radius.
- Corrected error ~= uncorrected after ~200 s (see flight001_trn.png).
- Findings:
  F1 batch window must be short relative to drift rate;
  F2 low-texture/loiter trajectories starve batch matching;
  F3 hard resets couple badly: blind between fixes, wrong on bad fixes.
- Conclusion: replace batch matching with recursive ESKF (SITAN)
  point-wise terrain-slope updates. -> Phase 5.
