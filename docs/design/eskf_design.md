# gdn_fusion v3 — design record

## Architecture
- **ROS-free core** (`include/gdn/map_db.hpp`, `include/gdn/eskf.hpp`): pure Eigen, no rclcpp.
- **Thin component-ready wrapper** (`src/eskf_node.cpp`): topics, params, telemetry only.
- **Dual-build selftest** (`src/selftest.cpp`): runs under both `-O2` (script) and `-O3` (colcon Release).
- **Regression harness** (`tools/bench.sh`): full-bag replay with scoring + DBG telemetry.
- State: nominal `(p, v, q)` + 15-dim error `(dp, dv, dtheta, ba, bg)`.

## Formulation references
- Propagation, F-matrix, error injection, J-reset: J. Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508 (2017).
- Innovation gating (3σ + absolute), Joseph-form update, PSD repair: Bar-Shalom, Li, Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001.
- Recursive terrain-aided point updates: L. D. Hostetler, SITAN lineage (IEEE 1978-85).
- Baseline/limits of batch TERCOM: J. P. Golden, SPIE 1980.
- Observability reasoning (yaw unobservable without aiding; vertical with scalar TRN): T. Barfoot, *State Estimation for Robotics*, CUP 2017.
- Middleware-free estimator precedent: PX4-EKF standalone library (shared by PX4 + ArduPilot).
- Component registration: ROS 2 docs — *About Composition*, *rclcpp_components*.

## Database
- REQ-DB-001: `sigma=1` cell smoothing (TEST-008 sweep) to prevent contour-lock divergence.
- Format: 40-byte header + row-major float32 grid; bilinear altitude + central-difference gradient.
- Local NED frame, map center == vehicle spawn.

## Initialization
- Static collection: motion-excluding (|w| < 0.05 rad/s), vibration-averaging (N=500), plausibility checks.
- Heading transfer = re-initialization: rotate nominal, reset `x = 0` and `P` (with yaw bump). OI-003 open (replace with magnetometer).

## Validation
- TEST-009/010: replay bench lock at ~10 m RMS (60-300 s).
- TEST-011: dual-build selftest caught uninitialized `sa_/sw_/sa2_` pre-flight.

## Open items (see docs/open_items.md)
- OI-002 (baro state for vertical observability)
- OI-003 (magnetometer yaw)
- OI-004 (landing-phase observability)
- OI-005 (fine-texture robustness)
