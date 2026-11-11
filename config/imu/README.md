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
| `mid360s.yaml` | **The Livox Mid-360S's built-in IMU**, from a 2 h bench log with the lidar running (TEST-025). For LIO (R6.4) and the Gazebo lidar model (R2.4). Its own motor shows below 1 s. |
| `oakd_lite.yaml` | **The OAK-D Lite's BMI270**, from a 2 h bench log with all three cameras running (TEST-025). For VIO (R3.5) and the Gazebo camera model (R2.4). Describes the stream as it arrives, about 240 Hz with uneven spacing, which is noisier than the chip alone. |
| `placeholder_px4_default.yaml` | The generic MEMS placeholder. PX4 v1.16's default simulated IMU: white noise only. The Monte Carlo's default before the H30WP fit (TEST-022). |
| `sensitivity_bias_drift.yaml` | The placeholder plus the bias drift PX4's Gazebo IMU had before 2025. A stress case, not a sensor. |

Use one in the Monte Carlo with `mc_campaign --imu=config/imu/FILE.yaml` (`--imu=placeholder`
is the built-in copy of the placeholder). The report's `imu:` line names the model, and
`ci_check_mc.py --expect-imu NAME` checks it.

## Adding a real IMU (H30WP, Mid-360S, OAK-D Lite)

Setup and recording for each sensor are in its guide in [docs/hardware/](../../docs/hardware/):
- **H30WP:** record straight from the serial port with `tools/h30wp_log.py`, not through
  the ROS driver. Its factory output has no sample timestamp, and the logger rebuilds even
  timing from the IMU's frame numbers.
- **Mid-360S:** a bag of `/livox/imu` from the Livox driver (acceleration in g).
- **OAK-D Lite:** `tools/oak_imu_log.py`, on DepthAI 3.12, with the cameras running. The
  apt ROS driver crashed our unit (OAK_D_LITE_SETUP.md). Check that your OAK-D Lite reports
  an IMU at all: early Lite units shipped without one.

1. **Record a static log of at least 2 hours.** Mount the sensor on something heavy and
   still, away from fans, motors and foot traffic, at a steady temperature, and keep away
   from the desk it's on. Drop the warm-up later with `--skip`: look at the minute means
   first (TEST-025: 15 min for the H30WP, 20 for the OAK-D Lite, 40 for the Mid-360S).

   ```bash
   systemd-inhibit --what=sleep:idle python3 tools/h30wp_log.py --hours 2.25 --out log/h30wp_static
   systemd-inhibit --what=sleep:idle timeout --foreground -s INT 2.5h ros2 bag record -s mcap \
       --disable-keyboard-controls -o log/mid360s_static --topics /livox/imu
   systemd-inhibit --what=sleep:idle python3 tools/oak_imu_log.py --hours 2.25 --out log/oakd_lite_static --ros
   ```
   If the IMU reaches GDN through PX4, record `/fmu/out/sensor_combined` instead.
   If the sensor loses power or restarts part-way, the fit stops at the break (a gap
   over 1 s, or the clock going back): what follows is a fresh power-up.

2. **Fit it:**

   ```bash
   python3 tools/allan_fit.py --csv log/h30wp_static/allan.csv --skip 900 \
       --name h30wp --out config/imu/h30wp.yaml --plot h30wp_allan.png
   python3 tools/allan_fit.py --bag log/mid360s_static --topic /livox/imu --accel-unit g \
       --skip 2400 --name mid360s --out config/imu/mid360s.yaml
   python3 tools/allan_fit.py --csv log/oakd_lite_static/imu.csv --skip 1200 \
       --name oakd_lite --out config/imu/oakd_lite.yaml
   ```
   Uneven sample times (the Mid-360S's jitter, the OAK-D Lite's skipped gyro ticks) are
   handled: each sample is held until the next, as GDN integrates it, and that is
   resampled evenly at the mean rate (TEST-025).
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
