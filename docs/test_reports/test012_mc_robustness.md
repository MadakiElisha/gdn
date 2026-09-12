# TEST-012: Monte Carlo Robustness Validation

**Objective:** Verify ESKF lock robustness across noise realizations.

**Method:**
- 100 seeded Monte Carlo runs with realistic sensor noise
- σ1 terrain (original): 27/100 seeds locked, CEP95 = 7130 m
- σ4 terrain (smoothed): 100/100 seeds locked, CEP95 = 40.3 m

**Root Cause Analysis:**
- σ1 terrain has high-frequency structure (correlation length ~20-40 m)
- Point-gradient EKF linearization valid only within correlation length
- Noise draws that push error beyond ~40 m cause false-lock on parallel contours
- σ4 smoothing lengthens correlation length, enlarges convergence basin

**Result:** REQ-DB-001 revised to σ4. Primary window (60-300 s) achieves 15.7 m RMS.

**Known Limitation:** OI-004 (landing-phase observability) remains — terrain near landing zone has poor TRN signature, horizontal error grows to >1 km after t=950 s.
