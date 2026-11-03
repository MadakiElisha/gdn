# TEST-016: flight001 re-baseline after the setup fixes (R0.3)

**Why.** PR #1 fixed three things that affect the replay bench:
- The TRN simulator used a different map origin from the filter (about 17 m off).
- `MapDb` scaled east offsets with the wrong latitude (2.6% on the sigma-4 map).
- The node overrode `r_trn` with 144 instead of the locked 400.

The old flight001 numbers were measured with those bugs in place, so they had to be
re-run.

**Setup.** Commit `2adc28d` (main after PRs #1 to #6). flight001 was fetched from
release `data-v1` with `tools/fetch_data.sh` (checksum verified), on Ubuntu 24.04 with
ROS 2 Jazzy. The TRN model is the original idealised one (`trn_model:=ideal`), so the
numbers compare like for like with the history.

## Monte Carlo (`tools/mc_run.sh 100`)

| | CEP50 | CEP95 | Max | Seeds under 100 m RMS |
|---|---|---|---|---|
| Before (TEST-012, Sep 2026) | 14.9 m | 40.3 m | | 100/100 |
| **Now** | **13.4 m** | **37.0 m** | 86.1 m | **100/100** |

The MC uses the same map for world and filter, so the origin and scale bugs only shifted
the geometry slightly. The small improvement comes from the corrected east scale.

## Replay bench (`tools/bench.sh`, 60 to 300 s, n = 12)

| | RMS | Max |
|---|---|---|
| Baro on, before (README history, after OI-003) | about 25 to 30 m | |
| **Baro on, now** | **31.5 m** | 68.4 m |
| Baro off, before (GNSS-velocity yaw, pre OI-003) | 15.2 m | 20.5 m |
| **Baro off, now** | **22.5 m** | 47.8 m |

Heading now comes from the magnetometer (OI-003) rather than GNSS velocity, so the
baro-off "before" figure isn't a like-for-like comparison. The baro-on case still
comes out worse than baro-off. That's the known OI-002/OI-003 coupling under the
idealised model, and it's one of the reasons for R1.1 (TEST-017).

The bench scorer only takes 12 samples in the window, and ROS timing makes runs
non-identical, so differences of a few metres aren't meaningful.

## Conclusion

These are the reference numbers for the idealised model from now on. The Monte Carlo
figures (13.4 m, 37.0 m, 100/100) are exactly repeatable, and CI guards the Zermatt
equivalent.
