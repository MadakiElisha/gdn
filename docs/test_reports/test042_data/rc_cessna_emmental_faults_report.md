# rc_cessna_emmental_faults: score (R2.7a)

rc_cessna (fixed-wing, REQ-NAV-003) over ch_emmental (independent map (scored)); route 278 s from its start; GNSS to PX4: none; start error 0 m; 4 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-003 CEP95 | 5 min (278 s flown) | 1943.5 m (CEP50 635.8 m) | 50 m | FAIL (window not complete) |
| REQ-NAV-003 max | 5 min | 2273.5 m | 100 m | FAIL (window not complete) |
| REQ-NAV-003 CEP95 | 10 min (278 s flown) | 1943.5 m (CEP50 635.8 m) | 75 m | FAIL (window not complete) |
| REQ-NAV-003 max | 10 min | 2273.5 m | 100 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 13.3 m (max 13.8 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 76.72% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 635.8 m, CEP95 1943.5 m, max 2273.5 m, RMS 1144.1 m, at the end 2262.5 m |
| NEES (2-D) | mean 958.612 (expected 2), median 143.10, over 9 79.7% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 1204.4 m, P95 2099.8 m, max 2273.5 m |
| NIS AGL | mean 0.83 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=260 |
| NIS baro | mean 0.32 (expected 1), over the 99.7% point 1.3% (expected 0.3%), n=13704 |
| NIS mag (3-axis) | mean 0.01 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=25505 |
| NIS ZUPT | mean 73.18 (expected 3), over the 99.7% point 100.0% (expected 0.3%), n=15 |
| time to acquire | 0.0 s (under 100 m for 30 s) |
| eskf_node CPU | mean 4.6%, p95 6.0%, max 6.0% of one core; memory max 46.8 MB |
| latency per IMU sample | processing p50 0.028 ms, p95 0.045 ms, max 2.20 ms; PX4 sample to output p50 92.30 ms, p95 225.40 ms (2 samples left out: PX4's clock sync reset) |
| simulation real-time factor | 0.990 (slowest 30 s: 0.986). PX4-clocked timing is reliable: it needs every 30 s at 0.98 or more |
