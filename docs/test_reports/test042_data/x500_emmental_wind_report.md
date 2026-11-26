# x500_emmental_wind: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 388 s from its start; GNSS to PX4: none; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (300 s flown) | 9708.5 m (CEP50 870.4 m) | 15 m | FAIL |
| REQ-NAV-001 max | 5 min | 11721.6 m | 30 m | FAIL |
| REQ-NAV-001 CEP95 | 10 min (388 s flown) | 20608.3 m (CEP50 1896.3 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 22778.3 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 9.9 m (max 11.1 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 86.25% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 1896.3 m, CEP95 20608.3 m, max 22778.3 m, RMS 9150.4 m, at the end 22778.3 m |
| NEES (2-D) | mean 6539.432 (expected 2), median 4322.04, over 9 93.3% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 10112.6 m, P95 22592.2 m, max 24887.4 m |
| NIS AGL | mean 0.90 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=325 |
| NIS baro | mean 0.05 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=18475 |
| NIS mag (3-axis) | mean 0.41 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=21325 |
| NIS mag heading | mean 0.25 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=15625 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 3.6%, p95 5.0%, max 6.0% of one core; memory max 45.3 MB |
| latency per IMU sample | processing p50 0.022 ms, p95 0.041 ms, max 0.47 ms; PX4 sample to output p50 244.02 ms, p95 675.19 ms (369 samples left out: PX4's clock sync reset); **unreliable**: the simulation ran slower than real time, so this is PX4's clock drift |
| simulation real-time factor | 0.955 (slowest 30 s: 0.929). PX4-clocked timing is unreliable: it needs every 30 s at 0.98 or more |
