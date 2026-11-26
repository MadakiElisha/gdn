# x500_emmental_smoke: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 382 s from its start; GNSS to PX4: none; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (300 s flown) | 956.9 m (CEP50 373.7 m) | 15 m | FAIL |
| REQ-NAV-001 max | 5 min | 1227.2 m | 30 m | FAIL |
| REQ-NAV-001 CEP95 | 10 min (382 s flown) | 2662.7 m (CEP50 396.0 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 3264.0 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 7.7 m (max 8.9 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 87.86% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 396.0 m, CEP95 2662.7 m, max 3264.0 m, RMS 1121.4 m, at the end 3182.6 m |
| NEES (2-D) | mean 2261.228 (expected 2), median 763.73, over 9 93.3% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 1271.2 m, P95 3183.2 m, max 3254.2 m |
| NIS AGL | mean 1.07 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=508 |
| NIS baro | mean 0.03 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=18966 |
| NIS mag (3-axis) | mean 0.06 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=37932 |
| NIS ZUPT | mean 68.82 (expected 3), over the 99.7% point 100.0% (expected 0.3%), n=17 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 3.0%, p95 4.0%, max 5.0% of one core; memory max 44.4 MB |
| latency per IMU sample | processing p50 0.021 ms, p95 0.035 ms, max 1.35 ms; PX4 sample to output p50 41.33 ms, p95 299.92 ms; **unreliable**: the simulation ran slower than real time, so this is PX4's clock drift |
| simulation real-time factor | 0.991 (slowest 30 s: 0.953). PX4-clocked timing is unreliable: it needs every 30 s at 0.98 or more |
