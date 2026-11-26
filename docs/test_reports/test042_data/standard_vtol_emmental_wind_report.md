# standard_vtol_emmental_wind: score (R2.7a)

standard_vtol (quadplane, REQ-NAV-002) over ch_emmental (independent map (scored)); route 272 s from its start; GNSS to PX4: none; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-002 CEP95 | 5 min (272 s flown) | 532.7 m (CEP50 232.3 m) | 40 m | FAIL (window not complete) |
| REQ-NAV-002 max | 5 min | 539.9 m | 80 m | FAIL (window not complete) |
| REQ-NAV-002 CEP95 | 10 min (272 s flown) | 532.7 m (CEP50 232.3 m) | 60 m | FAIL (window not complete) |
| REQ-NAV-002 max | 10 min | 539.9 m | 120 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 11.5 m (max 11.8 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 54.60% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 232.3 m, CEP95 532.7 m, max 539.9 m, RMS 318.5 m, at the end 506.1 m |
| NEES (2-D) | mean 180.134 (expected 2), median 120.26, over 9 57.9% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 324.4 m, P95 537.8 m, max 543.3 m |
| NIS AGL | mean 0.81 (expected 1), over the 99.7% point 0.2% (expected 0.3%), n=479 |
| NIS baro | mean 0.02 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=13605 |
| NIS mag (3-axis) | mean 0.00 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=27210 |
| NIS ZUPT | mean 1.56 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=17 |
| time to acquire | 30.2 s (under 80 m for 30 s) |
| eskf_node CPU | mean 3.6%, p95 5.0%, max 5.0% of one core; memory max 46.9 MB |
| latency per IMU sample | processing p50 0.023 ms, p95 0.037 ms, max 1.48 ms; PX4 sample to output p50 52.08 ms, p95 119.42 ms |
| simulation real-time factor | 0.999 (slowest 30 s: 0.996). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
