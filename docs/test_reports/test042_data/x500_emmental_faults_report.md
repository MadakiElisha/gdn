# x500_emmental_faults: score (R2.7a)

x500 (multirotor, REQ-NAV-001) over ch_emmental (independent map (scored)); route 381 s from its start; GNSS to PX4: none; start error 0 m; 4 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-001 CEP95 | 5 min (300 s flown) | 15290.6 m (CEP50 2053.1 m) | 15 m | FAIL |
| REQ-NAV-001 max | 5 min | 16489.5 m | 30 m | FAIL |
| REQ-NAV-001 CEP95 | 10 min (381 s flown) | 31867.9 m (CEP50 5425.1 m) | 25 m | FAIL (window not complete) |
| REQ-NAV-001 max | 10 min | 38489.5 m | 50 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 10782.4 m (max 12896.2 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 80.81% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 5425.1 m, CEP95 31867.9 m, max 38489.5 m, RMS 14194.7 m, at the end 38489.5 m |
| NEES (2-D) | mean 193.865 (expected 2), median 111.74, over 9 81.0% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 16401.2 m, P95 38063.0 m, max 46610.9 m |
| NIS AGL | mean 0.64 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=213 |
| NIS baro | mean 0.13 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=4973 |
| NIS mag (3-axis) | mean 0.12 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=34524 |
| NIS mag heading | mean 2.22 (expected 1), over the 99.7% point 10.3% (expected 0.3%), n=1754 |
| time to acquire | never (under 30 m for 30 s) |
| eskf_node CPU | mean 4.5%, p95 5.0%, max 6.0% of one core; memory max 46.9 MB |
| latency per IMU sample | processing p50 0.026 ms, p95 0.042 ms, max 1.47 ms; PX4 sample to output p50 83.01 ms, p95 197.14 ms (1 samples left out: PX4's clock sync reset) |
| simulation real-time factor | 0.992 (slowest 30 s: 0.987). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
