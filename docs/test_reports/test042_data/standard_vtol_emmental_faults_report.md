# standard_vtol_emmental_faults: score (R2.7a)

standard_vtol (quadplane, REQ-NAV-002) over ch_emmental (independent map (scored)); route 277 s from its start; GNSS to PX4: none; start error 0 m; 4 sensor faults. GNSS-free time counts from the route's start (GDN uses no GNSS in SITL).

![score](report.png)

| Requirement | Window | Measured | Limit | Verdict |
|---|---|---|---|---|
| REQ-NAV-002 CEP95 | 5 min (277 s flown) | 1557.2 m (CEP50 560.3 m) | 40 m | FAIL (window not complete) |
| REQ-NAV-002 max | 5 min | 2470.1 m | 80 m | FAIL (window not complete) |
| REQ-NAV-002 CEP95 | 10 min (277 s flown) | 1557.2 m (CEP50 560.3 m) | 60 m | FAIL (window not complete) |
| REQ-NAV-002 max | 10 min | 2470.1 m | 120 m | FAIL (window not complete) |
| REQ-NAV-010 altitude p95 | route | 7.9 m (max 15.2 m) | 5 m | FAIL |
| REQ-INT-001 over 3 sigma | route | 59.51% | 1% | FAIL |

One run's CEP95 and max; the requirement pools a set of runs (R2.7b's baseline).

| Measure | Value |
|---|---|
| whole route | CEP50 560.3 m, CEP95 1557.2 m, max 2470.1 m, RMS 912.2 m, at the end 2470.1 m |
| NEES (2-D) | mean 1180.123 (expected 2), median 120.64, over 9 63.1% (expected 1.1%) |
| cross-check: replay_scorer (/gdn/odom, whole run incl. take-off and landing) | RMS 1186.9 m, P95 2767.5 m, max 4171.6 m |
| NIS AGL | mean 0.79 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=290 |
| NIS baro | mean 0.03 (expected 1), over the 99.7% point 0.0% (expected 0.3%), n=12759 |
| NIS mag (3-axis) | mean 0.10 (expected 3), over the 99.7% point 0.0% (expected 0.3%), n=25614 |
| NIS mag heading | mean 3.77 (expected 1), over the 99.7% point 3.0% (expected 0.3%), n=335 |
| time to acquire | 0.1 s (under 80 m for 30 s) |
| eskf_node CPU | mean 4.7%, p95 6.0%, max 12.6% of one core; memory max 46.0 MB |
| latency per IMU sample | processing p50 0.027 ms, p95 0.045 ms, max 0.46 ms; PX4 sample to output p50 67.42 ms, p95 157.84 ms (663 samples left out: PX4's clock sync reset); **unreliable**: the simulation ran slower than real time, so this is PX4's clock drift |
| simulation real-time factor | 0.990 (slowest 30 s: 0.953). PX4-clocked timing is unreliable: it needs every 30 s at 0.98 or more |
