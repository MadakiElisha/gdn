# GDN requirements

**Status: draft, November 2026.** The GNSS-free durations (5 and 10 minutes) were
set by the project lead. The error limits are first proposals, picked to be
demanding but reachable with the reference hardware. Change them here, in a PR, as
evidence comes in. Every "done when" in [ROADMAP.md](../ROADMAP.md) is measured
against this file.

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
- **Max:** the worst sample in that same pool.
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
| REQ-NAV-001 | Multirotor, low (5 to 40 m above ground), 0 to 15 m/s | CEP95 at most 15 m, max 30 m | CEP95 at most 25 m, max 50 m |
| REQ-NAV-002 | Quadplane, cruise at 80 to 150 m above ground, plus hover phases | CEP95 at most 40 m, max 80 m | CEP95 at most 60 m, max 120 m |
| REQ-NAV-003 | Fixed-wing, cruise at 100 to 300 m above ground | CEP95 at most 50 m, max 100 m | CEP95 at most 75 m, max 100 m |

The fixed-wing 10-minute maximum keeps the original REQ-P-001 figure of 100 m.

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
| REQ-SYS-001, 002 | Processor-in-the-loop | R3.4, R5.4 |

## Where we stand today

None of these can be properly measured yet. Here's what we have:

- **GNSS-free windows (R1.7, TEST-023; IMU and filter from the H30WP fit, TEST-024).**
  The MC cuts GNSS mid-flight (`--gnss-loss`) and scores 5 and 10 minutes from the
  loss, on a 15 minute Zermatt flight and on flight001. Fixed-wing (REQ-NAV-003), GNSS
  lost 2 minutes after take-off, 100 flights:
  - Zermatt, perfect map: CEP95 20.2 m at 5 min and 22.5 m at 10 min, max 63 m and
    77 m, 0.00% over 3 sigma (pass).
  - Zermatt, SRTM-like maps: CEP95 81 m at 10 min, max 243 m, 6.3% over 3 sigma (fail).
  - flight001, real IMU: CEP95 63 m and 66 m (pass), max 107 m and 131 m (fail),
    13.1% and 6.6% over 3 sigma (fail; OI-013).
- **Acquisition (REQ-ACQ-001).** From 500 m, 3 to 7 flights in 100 are under 100 m at
  120 s after take-off; from 60 m, 73 to 100; from 150 m, 18 to 71 (TEST-024). Fails
  until acquisition (R4.1).
- **Fixed-wing, rugged terrain, TRN only, real altimeter model.** On Zermatt, CEP50
  is about 18 m, but CEP95 is over 900 m because of false-contour locks (OI-009, 7 of
  24 seeds). That fails REQ-NAV-003 until acquisition (R4.1) is in.
- **Vertical.** With the real altimeter model (R1.1, TEST-017), altitude p95 on the
  recorded flight is 2.9 m with no baro drift and 4.1 m with 5 m of drift, both inside
  REQ-NAV-010. With 15 m of drift over 5 minutes it's 9.9 m, which fails, though the
  filter no longer diverges (97 of 100 seeds stay under 100 m RMS).
- **Map error (REQ-DB-002).** Testable since R1.3 (TEST-020) with synthetic onboard-map error. With an SRTM-like onboard map (about 7 to 10 m RMS against truth) Zermatt passes 181 of 200 flights and flight001 171 of 200, and both fail REQ-INT-001 (13.1% and 10.8%). An independent DEM source comes with R2.2.
- **Integrity.** REQ-INT-001 is measured by the MC since R1.5 (TEST-019), with error > 3 x published sigma as the check. Since R1.4 (TEST-021) both flights pass with a perfect onboard map (flight001 0.00%, Zermatt 0.46%, from 12.8%); with SRTM-like onboard maps Zermatt still fails (5.5%). REQ-INT-002 to 005 need integrity monitoring (R4.4) and are failed by definition until then.
- **Multirotor and quadplane.** Not testable until the closed-loop SITL (Phase 2) and
  the odometry layer (Phase 3).
