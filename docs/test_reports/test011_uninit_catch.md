# TEST-011: first defect caught by unit-test layer pre-flight

**Finding:** Dual-build selftest (g++ `-O2` script vs colcon `-O3` Release) exposed
uninitialized static-collection accumulators `sa_`, `sw_`, `sa2_` in `Eskf::Eskf()`.

**Symptom:** Under `-O3`, stack garbage in these members survived the plausibility
gates (gyro-mean, |g|-dev, variance), producing a wrong accel bias → propagation
drift → innovations exceeding gates → updates rejected → TRN tests failed.

**Fix:** Initialize `sa_ = sw_ = sa2_ = V3::Zero()` in the constructor.

**Process impact:** `tools/run_selftest.sh` (`-O2`) and `colcon Release`
(`-O3`) selftest are now BOTH required to pass as a merge gate. The bug would
have manifested stochastically in flight (only when the stack happened to be
dirty) — the dual-build strategy converts it into a deterministic catch.

**Category:** Undefined behavior / uninitialized state. Resolved v3.
