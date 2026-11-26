# rc_cessna_emmental_start_error: score (R2.7a)

rc_cessna (fixed-wing, REQ-NAV-003) over ch_emmental (independent map (scored)); route 276 s from its start; GNSS to PX4: none; start error 150 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-003 CEP95 | 5 min (276 s flown) | 706.5 m (CEP50 483.9 m) | 50 m | FAIL (window not complete) |
| REQ-NAV-003 max | 5 min | 791.1 m | 100 m | FAIL (window not complete) |
| REQ-NAV-003 CEP95 | 10 min (276 s flown) | 706.5 m (CEP50 483.9 m) | 75 m | FAIL (window not complete) |
| REQ-NAV-003 max | 10 min | 791.1 m | 100 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 10.9 m (max 11.5 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 100.00% | 1% | FAIL |
| REQ-ACQ-001 error at 120 s | from a 150 m start | 449.3 m | 100 m | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 483.9 m, CEP95 706.5 m, max 791.1 m, RMS 542.0 m, at the end 791.1 m |
| NEES (2-D) | mean 996.638 (expected 2), median 624.99, over 9 100.0% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 565.7 m, P95 875.1 m, max 1062.4 m |
| NIS AGL | mean 0.62 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=472 |
| NIS baro | mean 0.02 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=13759 |
| NIS mag (3-axis) | mean 0.00 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=27519 |
| time to acquire | never (under 100 m for 30 s) |
| eskf_node CPU | mean 3.9%, p95 5.0%, max 6.9% of one core; memory max 44.6 MB |
| latency per IMU sample | processing p50 0.024 ms, p95 0.040 ms, max 0.95 ms; PX4 sample to output p50 61.48 ms, p95 123.11 ms |
| simulation real-time factor | 0.998 (slowest 30 s: 0.993). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
