# IMU error models (R1.6)

Each file describes one IMU's errors, in the terms an Allan-variance fit gives:

| Key | Meaning | Unit |
|---|---|---|
| `accel_noise_density`, `gyro_noise_density` | white noise per axis (velocity / angle random walk) | m/s²/√Hz, rad/s/√Hz |
| `accel_bias_rw`, `gyro_bias_rw` | driving density of the bias drift (first-order Gauss-Markov) | m/s³/√Hz, rad/s²/√Hz |
| `accel_bias_tau`, `gyro_bias_tau` | correlation time of that drift | s |
| `accel_turn_on_sigma`, `gyro_turn_on_sigma` | constant bias drawn once per power-up (1 sigma) | m/s², rad/s |

`gdn::ImuErrorSim` ([imu_model.hpp](../../src/gdn_fusion/include/gdn/imu_model.hpp))
simulates exactly this model, with the same recursion as PX4's Gazebo IMU plugin. Every
key is required, and unknown keys are refused, so a typo can't silently become a zero.
Keys starting with `fit_` are informational.

| File | What it is |
|---|---|
| `h30wp.yaml` | **The H30WP**, fitted from a 2.25 h bench log (TEST-024). The Monte Carlo's default (`--imu=auto`) on flights with an error-free IMU. Its drift over 1 to 10 minutes includes a room-temperature change, so it is on the safe side. |
| `placeholder_px4_default.yaml` | The generic MEMS placeholder. PX4 v1.16's default simulated IMU: white noise only. The Monte Carlo's default before the H30WP fit (TEST-022). |
| `sensitivity_bias_drift.yaml` | The placeholder plus the bias drift PX4's Gazebo IMU had before 2025. A stress case, not a sensor. |

Use one in the Monte Carlo with `mc_campaign --imu=config/imu/FILE.yaml` (`--imu=placeholder`
is the built-in copy of the placeholder). The report's `imu:` line names the model, and
`ci_check_mc.py --expect-imu NAME` checks it.

## Adding a real IMU (H30WP, Mid-360S, OAK-D Lite)

For the H30WP, wiring and first-time setup up to this recording are in
[docs/hardware/H30WP_SETUP.md](../../docs/hardware/H30WP_SETUP.md). Record it straight
from the serial port with `tools/h30wp_log.py` (below), not through the ROS driver: its
factory output has no sample timestamp, and the logger rebuilds even timing from the
IMU's frame numbers.

1. **Record a static log of at least 2 hours.** Mount the sensor on something heavy and
   still, away from fans, motors and foot traffic, at a steady temperature. Power it up
   15 minutes before you start recording, or drop that warm-up later with `--skip 900`.
   Log at the rate GDN will use, through the driver GDN will use:

   ```bash
   systemd-inhibit --what=sleep:idle python3 tools/h30wp_log.py --hours 2.25 --out log/h30wp_static
   ros2 bag record -s mcap -o log/h30wp_static  /imu/data      # or tools/h30wp_node.py, if it sends 0x51
   ros2 bag record -s mcap -o log/mid360_static /livox/imu     # Livox driver
   ros2 bag record -s mcap -o log/oakd_static   /oak/imu/data  # depthai-ros
   ```
   Topic names depend on the driver configuration (`ros2 topic list`). If the IMU
   reaches GDN through PX4, record `/fmu/out/sensor_combined` instead. Check that your
   OAK-D Lite reports an IMU at all: early Lite units shipped without one.

2. **Fit it:**

   ```bash
   python3 tools/allan_fit.py --csv log/h30wp_static/allan.csv --skip 900 \
       --name h30wp --out config/imu/h30wp.yaml --plot h30wp_allan.png
   python3 tools/allan_fit.py --bag log/mid360_static --topic /livox/imu --accel-unit g \
       --skip 900 --name mid360 --out config/imu/mid360.yaml
   ```
   It prints the white noise per axis, the drift (or "not resolved"), and the bias
   instability, for comparison with the datasheet. Look at the plot: the points should
   follow the fitted line. Over 30 s to 10 min (shaded) the model must not sit below
   the points: those are the GNSS-free windows. If least squares does (one drift term
   can't follow a flat stretch followed by a rise, which is usually temperature), the
   fit switches to the smallest drift that stays above the points there, and says so
   (`fit_accel_drift: upper envelope ...`). Short knocks are listed; check that the fit
   doesn't change without them.

3. **Turn-on bias.** One static log is a single power-up, so it can't show this. Give
   `--turn-on-accel` and `--turn-on-gyro` from several short power-ups (the spread of
   the static means), or from the datasheet. Until you do, the placeholder's values are
   kept, and the file says so.

4. **Use it:** `mc_campaign --imu=config/imu/h30wp.yaml`. Then set the filter's IMU
   process noise from it (`EskfConfig` `q_accel_nd`, `q_gyro_nd`, `q_accel_bias_rw`,
   `q_gyro_bias_rw`; try `mc_campaign --q-imu=match` first). The bias random walks are
   the settings that matter (TEST-022, TEST-024): for the H30WP, the gyro one. The
   selftest (TEST-024) checks the defaults against `h30wp.yaml`: never below it, bias
   random walks within 10x. The Gazebo IMU models follow in R2.4.

Check the tool itself at any time with `python3 tools/allan_fit.py --selfcheck` (CI
runs it). To check it against the C++ simulator, run
`install/gdn_fusion/lib/gdn_fusion/imu_sim placeholder 2 100 static.csv`, then
`allan_fit.py --csv static.csv`.
