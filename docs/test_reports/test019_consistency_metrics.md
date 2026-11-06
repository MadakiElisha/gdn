# TEST-019: consistency metrics in the Monte Carlo (R1.5)

**Question.** A navigation filter has to be right about how wrong it is. GDN reports
a position sigma on `/gdn/sigma`. Does the error actually stay inside what it reports,
and when it doesn't, which runs are to blame?

**Pass rule for the metric itself.** On errors drawn from a known covariance, the
statistics match the chi-square values, and errors twice the reported sigma are caught.

## What changed

- `gdn/consistency.hpp` (new, ROS-free): `ConsistencyStats` collects, per sample:
  - **horizontal NEES** = e^T P_h^-1 e. Consistent: chi-square with 2 dof, mean 2,
    median 1.39, 1.1% of samples outside the 3-sigma ellipse (NEES > 9);
  - **error > 3 x published sigma**, where the published sigma is
    sqrt(P_nn + P_ee) on `/gdn/sigma`. This is the check for REQ-INT-001 (at most 1%);
  - per TRN or AGL update, **NIS** = innov^2 / S, before the gate. Consistent: mean 1,
    median 0.45, 0.27% above 9.

  Samples along one flight are correlated in time, so these are descriptive
  statistics, not exact chi-square tests. A few false-lock seeds put NEES in the
  thousands, so the median is reported next to the mean.
- `Eskf::P_horiz()` returns the north/east covariance block.
- `mc_campaign` prints four `consistency:` lines over the scoring window, including a
  split between passing and failing seeds (per-seed RMS under or over 100 m).
- `ci_check_mc.py`: `--max-over-3sigma`, `--max-nees-median`, `--max-nis-over9`.
- CI: flight001 gates on REQ-INT-001 (at most 1%), NEES median at most 2.8 (twice the
  consistent value) and NIS > 9 in at most 1%. It passes all three today. On Zermatt
  the same REQ-INT-001 check runs as a non-blocking step: it fails, and it's the
  failing test for R1.4.

## Results: the metric (selftest, 200 000 samples)

| Errors drawn from | NEES mean / median | Outside 3-sigma ellipse | Error > 3 x sigma | NIS mean |
|---|---|---|---|---|
| The reported covariance | 2.003 / 1.390 | 1.12% | 0.09% | 1.003 |
| Twice the reported sigma | 8.01 | 32.6% | 11.4% | 4.01 |

## Results: the filter today (100 seeds, scored 60 to 300 s)

| Run | Error > 3 x sigma (REQ-INT-001) | NEES median | NIS > 9 |
|---|---|---|---|
| flight001, agl, 5 m baro drift | **0.04% (pass)** | 0.91 | 0.0% |
| flight001, agl, 5 m drift, node start-up, `--mag=align` | **0.00% (pass)** | 0.91 | 0.0% |
| Zermatt, ideal, truth start-up | 5.22% (fail) | 0.24 | 0.6% |
| Zermatt, agl, truth start-up | 22.8% (fail) | 1.29 | 6.7% |
| Zermatt, agl, node start-up | **12.8% (fail)** | 1.01 | 4.3% |

Split by seed, Zermatt, agl, node start-up:

| Seeds | Count | Error > 3 x sigma | NEES median |
|---|---|---|---|
| Passing (RMS under 100 m) | 87 | 1.36% | 0.80 |
| Failing | 13 | **89.6%** | **80** |

## Findings

1. **The filter is honest when it's right.** On flight001 and on the passing Zermatt
   seeds, NEES sits at or below the consistent value (median 0.8 to 0.9 against 1.39):
   slightly cautious, which is the safe side.
2. **When it's wrong, it's confidently wrong.** In the failing Zermatt seeds, 90% of
   samples are outside 3 sigma, with NEES around 80: errors about 9 times the reported
   sigma. This is the false lock of audit A-02 and OI-009, now measured. These 13 seeds
   alone fail REQ-INT-001 for the whole set.
3. **NIS shows the same split.** NIS above 9 in 4.3% of Zermatt updates (0.27% if
   consistent): the filter keeps seeing measurements it thinks are impossible, and the
   gate rejects them instead of the covariance growing.

R1.4 has to bring the Zermatt REQ-INT-001 figure under 1%. The ways to get there are
an honest covariance (no clamps that shrink P without applying the correction, no
contraction of states an update doesn't observe) and, beyond Phase 1, integrity
monitoring that inflates or resets on persistent NIS (R4.4).

## Reproduce

```bash
tools/run_selftest.sh
tools/mc_run.sh 100 data/flight_zermatt_csv data/maps/zermatt.bin --model=agl --startup=node
tools/mc_run.sh 100 data/flight001_csv data/maps/terrain_db_sigma4.bin --model=agl --baro-drift=5
python3 tools/ci_check_mc.py /tmp/mc_report.txt --max-over-3sigma 1 --max-nees-median 2.8
```
