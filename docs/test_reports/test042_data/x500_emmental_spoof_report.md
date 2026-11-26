# x500_emmental_spoof: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 367 s from its start; GNSS to PX4: spoof; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (300 s flown) | 2617.3 m (CEP50 561.4 m) | 15 m | FAIL |
| REQ-NAV-001 max | 5 min | 3189.9 m | 30 m | FAIL |
| REQ-NAV-001 CEP95 | 10 min (367 s flown) | 5700.2 m (CEP50 611.0 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 6996.1 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 16.5 m (max 17.3 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 83.18% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 611.0 m, CEP95 5700.2 m, max 6996.1 m, RMS 2407.0 m, at the end 6996.1 m |
| NEES (2-D) | mean 6372.354 (expected 2), median 454.04, over 9 92.7% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 2877.1 m, P95 6981.1 m, max 8618.9 m |
| NIS AGL | mean 1.50 (expected 1), over the 99.7% point 0.3% (expected 0.3%), n=310 |
| NIS baro | mean 0.03 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=18250 |
| NIS mag (3-axis) | mean 0.06 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=36500 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 4.0%, p95 5.0%, max 5.0% of one core; memory max 47.5 MB |
| latency per IMU sample | processing p50 0.025 ms, p95 0.041 ms, max 1.69 ms; PX4 sample to output p50 65.92 ms, p95 163.26 ms |
| simulation real-time factor | 0.996 (slowest 30 s: 0.993). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
