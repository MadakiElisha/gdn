# Setting up the H30WP IMU

This is how we get the Yesense / WHEELTEC H30WP talking to the PC, publishing on ROS 2,
and recording the static log we need for the IMU noise fit (R1.6).

Expect about half an hour for the setup, then a long static recording at the end:
at least two hours, longer is better.

## What's in the box

The IMU itself is a black aluminium block with WHEELTEC, YESENSE and H30WP printed on
the lid, along with its axis arrows. It has a round, screw-on waterproof socket on one
side.

The cable has a matching round plug on one end. The other end is a bundle of eight
thin coloured wires (the jacket reads AWM 2464, 8 x 0.25 mm²), each ending in a small
black push-on connector.

The adapter is a small green USB-to-serial board with a USB-C socket. It has two
3-pin headers: one marked `G TX RX`, the other marked `5V G 3V3`. There's also a
white 6-pin socket on it that we don't use.

You also need a USB-C cable that carries data. Plenty of USB-C cables only charge,
and the PC won't see the adapter through one of those.

A few numbers from WHEELTEC's documentation:
- the IMU runs on 5 to 26 V;
- it talks serial at 460800 baud by default;
- it outputs up to 400 samples a second.

## Wiring

Keep the USB cable unplugged the whole time you're wiring.

Only four of the eight wires get connected:

| Wire | What it is | Adapter pin |
|---|---|---|
| Red | power in | `5V` |
| Brown | ground | `G` on the `5V G 3V3` header |
| Yellow | serial data | `TX` or `RX` |
| Green | serial data | the other one of `TX` / `RX` |

Blue, white, pink and grey stay unconnected. They're the RS-485 and spare lines.
Tape their ends or tuck them away, so they can't touch a pin or each other.

WHEELTEC doesn't publish the colour code for this cable, so the table above is the
wiring we found working on our unit, not an official chart. If your IMU came with a
colour chart, follow the chart instead.

Two things to know before you trust the table:
- **Never put anything on `3V3`.** The IMU needs at least 5 V. At 3.3 V it just won't start; nothing gets damaged.
- **Yellow and green can be swapped without risk.** If they're the wrong way round, you get no data, and swapping them fixes it.

Screw the round plug onto the IMU: line up the notch, push it in, then turn the
ring finger-tight. Set the IMU flat on the desk and leave it alone.

It's worth taking a photo of the adapter with the wires on it. Then anyone can
check the wiring later without unplugging anything.

## Preparing PC

This only needs doing once per machine.

```bash
sudo usermod -aG dialout $USER
sudo apt install -y python3-serial
sudo apt remove -y brltty
```

- The first line lets your user open serial ports without sudo.
- The second installs the serial library our tools use (if it isn't already installed).
- The third removes a braille-display service that Ubuntu ships with. It likes to grab USB serial adapters the moment they appear.

Log out and back in afterwards, or the group change won't apply. Running `groups`
should then list `dialout`.

## First connection

Plug the USB-C cable into the adapter and then into the PC. The blue light on the
adapter comes on. Then:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

You should see one new port, normally `/dev/ttyUSB0`.

If nothing shows up, try a different USB-C cable first, since that's the usual
culprit. If that doesn't help, run `sudo dmesg | tail -n 20` straight after
plugging in. A port that appears and then disappears means `brltty` is still
installed.

## Checking the IMU

From the GDN workspace (cd gdn_workspace):

```bash
python3 tools/h30wp_check.py
```

The script finds the port and listens for five seconds. Then it prints what it
decoded and a one-line verdict at the end. On a healthy IMU sitting still, the
output looks roughly like this:

```
baud 460800: ... bytes, ... good frames, 0 bad checksums
frames: 200 per second, 0 lost ...
acceleration (m/s^2, FLU): +0.010 -0.020 +9.790  |a| = 9.790  OK (about 9.8 at rest)
angular rate (deg/s, FLU): +0.0492 -0.0299 +0.0100  OK (still)
magnetic field (mGauss, raw): ...
temperature: 25.1 C

RESULT: PASS, the IMU works
```

Look at three things:
- `|a|`, the size of the measured acceleration, should be close to 9.8. That's gravity.
- The angular rate should be close to zero.
- The frame count should be steady, with nothing lost.

As a last sanity check, tilt the IMU by hand and run the script again. The
acceleration numbers should change with the tilt.

If it doesn't pass, the script says why:
- **`NO PORT`**: the PC can't see the adapter. Go back to the cable and `brltty`.
- **`NO PERMISSION`**: the `dialout` group isn't active yet. Log out and back in.
- **`NO DATA`**: the adapter is fine, but nothing is coming from the IMU. Unplug USB, check that red is on `5V` and brown on `G`, then swap yellow and green. Plug back in and run the check again.
- **`BYTES BUT NO FRAMES`**: data is arriving at a different speed. Run `python3 tools/h30wp_check.py --scan`, which tries the common baud rates.
- **`expected about 9.8`**: the IMU was probably moving, or isn't flat. Run it again with the IMU still. If it keeps happening, post the output in the team channel.

If you've swapped the data wires and it still says `NO DATA`, Hold.

## Giving it a fixed name

Once the TFmini-S, and maybe PX4, are on USB too, `/dev/ttyUSB0` won't always be
the IMU. A udev rule gives the IMU's adapter a permanent name, `/dev/gdn_imu`:

```bash
python3 tools/h30wp_check.py --udev
```

That prints a single line. Save it as `/etc/udev/rules.d/99-gdn-imu.rules` (with
`sudo nano`, or any editor run with sudo), then reload:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/gdn_imu
```

If the adapter has no serial number, the rule matches any adapter of the same
model. That's fine as long as we only have one.

## Running it in ROS 2

```bash
source env.sh
python3 tools/h30wp_node.py
```

The driver uses `/dev/gdn_imu` if it exists, otherwise whichever single
`/dev/ttyUSB*` it finds. To choose the port, add
`--ros-args -p port:=/dev/ttyUSB0`.

It publishes three topics:

| Topic | Type | Contents |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | acceleration (m/s²) and angular rate (rad/s) |
| `/imu/mag` | `sensor_msgs/MagneticField` | magnetic field (tesla) |
| `/imu/temperature` | `sensor_msgs/Temperature` | temperature (°C) |

Every ten seconds it logs the rate, lost frames and bad checksums. The last two
should stay at zero. If the USB cable gets knocked out, the driver says so and
picks the IMU back up when it's plugged in again.

To check from another terminal (after `source env.sh`):

```bash
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

The data stays in the IMU's own axes, as printed on the lid:
- X points away from the cable socket;
- Y points toward the edge with "H30WP" printed on it;
- Z points out of the lid.

When the IMU sits lid-up, that's front-left-up, the usual ROS convention. GDN and
PX4 work in front-right-down, but that conversion belongs where the IMU is mounted
on the vehicle and fed into the filter, not in the driver.

## The static recording

This log is what we fit the IMU's noise and bias drift from. Those numbers go into
the filter tuning and into the simulator, so it's worth doing carefully, once.

The setting matters more than anything else:
- Put the IMU on something heavy and solid. A concrete floor, a stone slab or a thick book on the floor all work.
- Keep it away from fans, the laptop, speakers, doors, and anywhere people walk past.
- Keep the temperature steady. An air conditioner cycling on and off, or sun moving across the room, both show up in the data.
- Once it's started, don't touch the IMU or its cable.

Record straight from the serial port, without the driver running. The factory output
has no sample timestamp, so the driver can only stamp messages on arrival, and USB
delivers them in bursts. `tools/h30wp_log.py` times each frame from the IMU's own
frame numbers instead:

```bash
systemd-inhibit --what=sleep:idle python3 tools/h30wp_log.py --hours 2.25 --out log/h30wp_static
```

It prints a status line every minute (frames, lost, |a|, gyro). `lost` should stay at
0 and |a| within about 0.001 of where it started. `systemd-inhibit` keeps the PC from
sleeping. The 2.25 hours include 15 minutes of warm-up, dropped by the fit. Ctrl-C
stops early and keeps everything so far. Then run the fit:

```bash
python3 tools/allan_fit.py --csv log/h30wp_static/allan.csv --skip 900 \
    --name h30wp --out config/imu/h30wp.yaml --plot h30wp_allan.png
```

The `--skip 900` drops the first 15 minutes, in case the IMU was still settling. The
first log (9 November 2026, TEST-024) is in `config/imu/h30wp.yaml`. The room warmed
during it. A second log at a steady temperature would refine the drift model, but it's
optional.

A good result reports `0 gaps`, and on the plot the measured points follow the
fitted line. If the plot has a bump the line can't follow, the recording was
disturbed and needs doing again. [config/imu/README.md](../../config/imu/README.md)
covers what happens with the fit after that.

Keep `raw.csv`. It's our raw evidence for the IMU model (`h30wp_log.py --finalize DIR`
rebuilds `allan.csv` from it).

## Not necessary for now

WHEELTEC's Windows tool (YIS Manager) can recalibrate the gyro bias and the
magnetometer, and change the output mode and baud rate. Don't use it yet:
- We want the static recording to measure the IMU as it was delivered, and our tools are set up for the default output.
- The magnetometer calibration only means something once the IMU is mounted on the drone, with the motors and wiring around it.

## References

- [WHEELTEC H30 repository](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU): user manual (power, the 460800 default, axes) and the YESENSE Communication Protocol v2.3, which `tools/h30wp_check.py` decodes. The decoder is tested against the protocol document's own example frame.
- [Seeed Studio wiki, H30 MINI and H30WP](https://wiki.seeedstudio.com/wheeltec_imu/): the H30WP's 5 to 26 V input, UART and RS-485 interfaces, and 400 Hz output.
