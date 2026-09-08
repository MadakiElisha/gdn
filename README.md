# GDN — GPS-Denied Navigation System

Production-grade, long-range, GPS-denied navigation for fixed-wing, VTOL and
multirotor UAVs. Map-based absolute localization (TRN / MagNav / visual scene
matching) fused with inertial navigation via an error-state Kalman filter.
Platform-agnostic: publishes 6-DoF state to any MAVLink/DDS flight controller.

## Status
Phase 2 complete: DDS bridge (PX4 v1.15 <-> ROS 2 Jazzy) + live sensor ingestion
via `gdn_sensors`.

## Layout
- `src/`        ROS 2 packages (gdn_sensors, gdn_core, gdn_fusion, gdn_maps, gdn_fdi)
- `src/px4_msgs` submodule pinned to PX4 release/1.15
- `docs/`       SRD, architecture, test reports
- `tests/`      unit, integration, Monte Carlo harness
- `tools/`      offline analysis, bag QoS profiles, venv
- `env.sh`      project-scoped ROS environment (source this first)

## Quick start (Ubuntu 24.04 / WSL2)
    source env.sh
    colcon build --packages-select px4_msgs gdn_sensors
    ros2 run gdn_sensors sensor_monitor

Requirements: docs/requirements/SRD.md
