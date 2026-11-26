# x500_emmental_dusk: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 167 s from its start; GNSS to PX4: none; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (167 s flown) | 8390.4 m (CEP50 646.3 m) | 15 m | FAIL (window not complete) |
| REQ-NAV-001 max | 5 min | 9629.9 m | 30 m | FAIL (window not complete) |
| REQ-NAV-001 CEP95 | 10 min (167 s flown) | 8390.4 m (CEP50 646.3 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 9629.9 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 4.2 m (max 4.8 m) | 5 m | PASS |
| REQ-INT-001 over 3 sigma | route | 61.57% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 646.3 m, CEP95 8390.4 m, max 9629.9 m, RMS 3663.7 m, at the end 9629.9 m |
| NEES (2-D) | mean 1739.009 (expected 2), median 844.32, over 9 61.9% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 4727.0 m, P95 11036.3 m, max 12725.5 m |
| NIS AGL | mean 0.52 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=142 |
| NIS baro | mean 0.36 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=8255 |
| NIS mag (3-axis) | mean 0.13 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=16456 |
| NIS mag heading | mean 0.05 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=54 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 4.3%, p95 5.0%, max 6.0% of one core; memory max 47.2 MB |
| latency per IMU sample | processing p50 0.029 ms, p95 0.048 ms, max 0.44 ms; PX4 sample to output p50 104.72 ms, p95 264.49 ms |
| simulation real-time factor | 0.987 (slowest 30 s: 0.983). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
