# x500_emmental_gnss_cut: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 148 s from its start; GNSS to PX4: cut; start error 0 m; 0 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (148 s flown) | 114.4 m (CEP50 104.0 m) | 15 m | FAIL (window not complete) |
| REQ-NAV-001 max | 5 min | 124.6 m | 30 m | FAIL (window not complete) |
| REQ-NAV-001 CEP95 | 10 min (148 s flown) | 114.4 m (CEP50 104.0 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 124.6 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 3.7 m (max 3.9 m) | 5 m | PASS |
| REQ-INT-001 over 3 sigma | route | 0.00% | 1% | PASS |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 104.0 m, CEP95 114.4 m, max 124.6 m, RMS 91.5 m, at the end 77.5 m |
| NEES (2-D) | mean 3.467 (expected 2), median 2.90, over 9 0.0% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 88.4 m, P95 114.0 m, max 123.8 m |
| NIS AGL | mean 0.02 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=274 |
| NIS baro | mean 0.02 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=7334 |
| NIS mag (3-axis) | mean 0.05 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=14667 |
| NIS ZUPT | mean 0.34 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=17 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 3.3%, p95 5.0%, max 7.0% of one core; memory max 45.7 MB |
| latency per IMU sample | processing p50 0.021 ms, p95 0.037 ms, max 1.42 ms; PX4 sample to output p50 75.16 ms, p95 350.33 ms; **unreliable**: the simulation ran slower than real time, so this is PX4's clock drift |
| simulation real-time factor | 0.989 (slowest 30 s: 0.975). PX4-clocked timing is unreliable: it needs every 30 s at 0.98 or more |
