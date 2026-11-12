# GDN requirements

**Status: agreed, November 2026 (R1.8).** The GNSS-free durations (5 and 10 minutes)
were set by the project lead. The fixed-wing limits, the definitions and the scoring
rules below are agreed. The multirotor and quadplane limits are **provisional**: they
can't be measured until the closed-loop SITL (R2.7) and the odometry layer (Phase 3),
and get revisited then. Change any of this here, in a PR, as evidence comes in. Every
"done when" in [ROADMAP.md](../ROADMAP.md) is measured against this file, and
`tools/score_requirements.py` reads its limits from it (TEST-026).

REQ-P-001 (horizontal error at most 100 m over the mission, from TEST-002) is the
parent requirement. The tables below split it by vehicle class and by how long GNSS
has been gone.

## Definitions

- **GNSS-free time (t):** time since GNSS was lost or rejected, starting from a
  converged filter. In tests, GNSS is cut mid-flight, and the filter takes over with
  the covariance it had at that moment. Starting the filter with no position at all
  is covered separately, under acquisition.
- **Horizontal error:** the 2D distance between GDN's position and truth.
- **CEP95:** the error that 95% of samples stay under, pooled over all runs of a
  scenario set (Monte Carlo seeds or SITL runs) and over the whole GNSS-free window
  up to time t.
- **Max (99% of flights):** at least 99% of the runs in the scenario set never go over
  the max limit at any moment of the GNSS-free window up to time t. A single worst
  sample over all runs would get harder to pass the more runs there are; this doesn't.
- **Map condition:** accuracy is scored with an onboard map from a different, coarser
  source than the world (REQ-DB-002): SRTM-like maps today (`tools/mc_map_ensemble.py`),
  an independent DEM from R2.2. Runs with the onboard map equal to the world are
  regression checks, not scores.
- **Terrain classes:**
  - **rugged:** mountains, strong relief
  - **rolling:** hills, moderate relief
  - **flat:** farmland, little relief but visible texture
  - **featureless:** water, snow, uniform sand

  TRN only helps on the first two. VIO and LIO carry the flat class.

## Horizontal accuracy

Limits apply on rugged, rolling and flat terrain. On featureless terrain, the
requirement is honesty, not accuracy (see Integrity).

| ID | Vehicle and profile | GNSS-free for 5 min | GNSS-free for 10 min |
|---|---|---|---|
| REQ-NAV-001 | Multirotor, low (5 to 40 m above ground), 0 to 15 m/s. **Provisional** | CEP95 at most 15 m, max 30 m (99% of flights) | CEP95 at most 25 m, max 50 m (99% of flights) |
| REQ-NAV-002 | Quadplane, cruise at 80 to 150 m above ground, plus hover phases. **Provisional** | CEP95 at most 40 m, max 80 m (99% of flights) | CEP95 at most 60 m, max 120 m (99% of flights) |
| REQ-NAV-003 | Fixed-wing, cruise at 100 to 300 m above ground | CEP95 at most 50 m, max 100 m (99% of flights) | CEP95 at most 75 m, max 100 m (99% of flights) |

The fixed-wing 10-minute maximum keeps the original REQ-P-001 figure of 100 m.

For scale: the Monte Carlo's fixed-wing flight covers 9 km in 5 minutes and 18 km in 10
(30 m/s), so 75 m is 0.4% of the distance flown. The multirotor's 25 m at 15 m/s is
0.3%, tighter than visual odometry alone usually manages (0.5 to 2%): it needs TRN or
LIO against a map as well. That is one reason it stays provisional.

## Vertical accuracy

| ID | Requirement |
|---|---|
| REQ-NAV-010 | Altitude error at most 5 m (95%) for the full GNSS-free window, all vehicles. A barometer bias of up to 20 m must not cause divergence. |
| REQ-NAV-011 | Height above ground, when a rangefinder or lidar is in range, within 1 m (95%). |

## Integrity

A wrong answer that looks confident is worse than a flagged one.

| ID | Requirement |
|---|---|
| REQ-INT-001 | Consistency: horizontal error exceeds the reported 3-sigma in at most 1% of samples, in every scenario set. |
| REQ-INT-002 | Lost-lock detection: when the true error passes 3 times the requirement, GDN reports "degraded" or "lost" within 30 s, in at least 99% of cases. |
| REQ-INT-003 | False alarms: at most one "lost" report per 10 hours of nominal simulated flight. |
| REQ-INT-004 | Featureless terrain: accuracy isn't required, but REQ-INT-001 and REQ-INT-002 are. The reported uncertainty must grow honestly. |
| REQ-INT-005 | GNSS spoofing: a slow drift of up to 1 m/s is flagged within 60 s, once GDN has an independent fix (TRN, odometry or map). |

## Acquisition

| ID | Requirement |
|---|---|
| REQ-ACQ-001 | From an initial horizontal error of up to 500 m, on rugged or rolling terrain, reach the REQ-NAV limits within 120 s of TRN-capable flight. |
| REQ-ACQ-002 | After a reported "lost", recover to "valid" within 120 s on rugged or rolling terrain. |

REQ-ACQ-001 is scored with the same 99% rule: at least 99% of flights, from each start
error up to 500 m, are under the class's max limit 120 s after take-off.

## Map

| ID | Requirement |
|---|---|
| REQ-DB-001 | Onboard terrain map at about 30 m resolution, smoothed to sigma 4 cells for scalar TRN (TEST-008, TEST-012). |
| REQ-DB-002 | The onboard map comes from a different, coarser source than the simulation world (for example SRTM or GLO-30 against a 1 to 5 m DEM), so tests always include real map error. |

## Timing and compute

| ID | Requirement |
|---|---|
| REQ-SYS-001 | Navigation output at 50 Hz or more, with latency from IMU sample to output of 20 ms or less on the companion computer. |
| REQ-SYS-002 | GDN plus LIO plus VIO use at most 70% of the companion's CPU in sustained flight, with no thermal throttling over 30 minutes. |

## How each requirement is checked

| Requirement | Checked by | Roadmap |
|---|---|---|
| REQ-NAV-001 to 003, REQ-NAV-010 | MC (R1.7 GNSS-loss scenarios), SITL scenario matrix (R2.8), qualification (R5.5) | R1.7, R2.7 |
| REQ-NAV-011 | SITL with the simulated TFmini-S and lidar, then real flights | R2.4, R6.8 |
| REQ-INT-001 | MC consistency metrics, SITL reports | R1.5, R2.7 |
| REQ-INT-002 to 005 | SITL fault and spoofing scenarios | R2.6, R4.4 |
| REQ-ACQ-001, 002 | MC start-error scenarios, SITL | R1.7, R4.1, R4.5 |
| REQ-DB-002 | World and map generation | R1.3, R2.2 |

`tools/score_requirements.py --class CLASS MC_OUTPUTS...` scores Monte Carlo outputs
against this file: PASS, FAIL, or NOT SCORED with the reason (a perfect-map run, or a
max limit the MC doesn't count yet: it counts flights against 100 m, the fixed-wing
limit).
| REQ-SYS-001, 002 | Processor-in-the-loop | R3.4, R5.4 |

## Where we stand today

Scored by `tools/score_requirements.py` in TEST-026 (November 2026, after R1.6 and
R1.7): fixed-wing, SRTM-like onboard maps, 100 flights per scenario. **1 pass, 25
fails.**

- **GNSS-free windows (REQ-NAV-003), GNSS lost 150 s after take-off.**
  - Zermatt (30 m/s): CEP95 62.4 m at 5 min and 80.8 m at 10 min (limits 50 and 75 m).
    77 and 37 flights in 100 never over 100 m (99 needed). Fail.
  - flight001, real IMU (15 m/s): CEP95 85.5 and 95.8 m; 55 and 26 flights never over
    100 m. Fail.
  - The typical error is close to the limits. The flights that run away to the wrong
    terrain contour are what fails (OI-009).
  - On a perfect map (a regression check, not a score), Zermatt would pass everything:
    CEP95 20.2 and 22.5 m, 100 of 100 flights, 0.00% over 3 sigma.
- **Integrity (REQ-INT-001).** 4.2% to 13.1% over 3 sigma in the GNSS-free windows, and
  3.9% to 95% in the start-error runs (1% allowed). Fails wherever there is map error,
  until integrity monitoring and map-aware TRN (R4.1, R4.4).
- **Vertical (REQ-NAV-010).** Altitude p95 is 10.2 m (Zermatt) and 15.2 m (flight001) on
  SRTM-like maps, so it fails. On a perfect map it is 1.5 m. Altitude follows the map's
  error (OI-014). Barometer drift: with 15 m of drift over 5 minutes, p95 is 9.9 m
  (TEST-017).
- **Acquisition (REQ-ACQ-001).** Under 100 m 120 s after take-off:
  - Zermatt: 99, 84 and 7 flights in 100 from 60, 150 and 500 m. Only 60 m passes.
  - flight001: 69, 22 and 2. Fails.

  As expected until acquisition (R4.1).
- **Map error (REQ-DB-002).** Every score above uses synthetic SRTM-like map error
  (TEST-020's recipe). An independent DEM comes with R2.2.
- **Not yet scored:** REQ-INT-002 to 005 need integrity monitoring (R4.4); REQ-ACQ-002,
  REQ-NAV-011 and the multirotor and quadplane limits need SITL (Phase 2) and the
  odometry layer (Phase 3); REQ-SYS-001 and 002 need the companion computer (R3.4).
