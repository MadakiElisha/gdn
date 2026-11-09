# H30WP IMU: first-time setup on the PC

This takes the Yesense / WHEELTEC H30WP from the box to a ROS 2 topic and a static
recording for the noise fit (roadmap R1.6). Do the steps in order. Each one ends with
a check, so a problem shows up at the step that caused it.

Time: about 30 minutes, plus the 2 hour (or overnight) static recording at the end.

## 1. What you have

| Part | What it is |
|---|---|
| The IMU | Black metal box with WHEELTEC / YESENSE / H30WP on the lid and axis arrows. It has a round screw-on waterproof socket. |
| The IMU cable | Black cable, round plug on one end. The other end splits into 8 thin coloured wires with small black push-on ends (Dupont). |
| The adapter | Small green board with a USB-C socket and a black chip in the middle. This is a USB-to-serial converter. It has a 3-pin header marked **G TX RX** on one side and pins marked **5V** and **3V3** on the other. |
| A USB-C cable | To connect the adapter to the PC. It must be a data cable. Some cables only charge, and the PC won't see anything through those. |

Specs worth knowing:
- The IMU accepts 5 to 26 V.
- It talks serial (UART) at 460800 baud by default.
- It sends up to 400 samples per second.

## 2. The wiring

**Rules, before touching any wire:**
- **Do all wiring with the USB cable unplugged.** Plug it in only when a step says so.
- The IMU needs **four** wires connected: power, ground, and two data wires.
- The other four wires stay **unconnected**. Keep their ends apart, and away from anything metal. A loose end touching a pin or another wire is the easiest way to damage something.
- Nothing ever goes on the adapter's **3V3** pin. The IMU needs at least 5 V. On 3.3 V it won't start, though it won't be damaged either.

**Which wire is which.** WHEELTEC doesn't publish the colour code for this cable, and it isn't printed on it. If a slip of paper with a colour chart came in the box, or the seller's listing has one, **that chart wins** over anything below. Without one, this is the wiring your unit is already set up with, and it's consistent with the usual convention:

| Wire colour | IMU function (most likely) | Goes to adapter pin |
|---|---|---|
| Red | Power in (5 to 26 V) | **5V** |
| Brown | Ground | **G** (the one next to 5V, or the G on the TX/RX header) |
| Green | Serial data | **TX** or **RX** |
| Yellow | Serial data | the other of **TX** / **RX** |
| Blue, white, pink, grey | RS-485 and spare lines | nothing; ends kept apart |

Green and yellow are the only uncertain pair, and that guess is harmless. If they're the
wrong way round, nothing breaks; you just get no data, and you swap them (step 6).

**Do it:**
1. With USB unplugged, push each of the four Dupont ends fully onto its pin. Push straight on; don't bend the pins.
2. Wrap the four unused wire ends with a bit of tape, or tuck them where they can't touch the board.
3. Screw the round plug onto the IMU: line up the notch, push in, then turn the ring finger-tight.
4. Put the IMU flat on a solid table. Leave it still.

**Check:** take one clear, close photo of the adapter showing all pins and wires. Keep
it; it's the record of how this unit is wired.

## 3. Prepare the PC (once)

```bash
sudo usermod -aG dialout $USER        # lets you open serial ports without sudo
sudo apt install -y python3-serial    # the serial library the GDN tools use
sudo apt remove -y brltty             # a braille-display service that grabs USB serial adapters
```
**Log out and back in, or reboot,** so the new group takes effect.

**Check:** `groups` lists `dialout`.

## 4. Plug in and find the port

Plug the USB-C cable into the adapter, then into the PC. The adapter's blue light
should come on. Then:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
lsusb
```

**Check:** exactly one new port appears, usually `/dev/ttyUSB0`. `lsusb` shows a new
line for the adapter's chip, typically Silicon Labs CP210x or QinHeng (CH340/CH9102).

**If no port appears:**
- Try another USB-C cable. A charge-only cable is the most common cause.
- Run `sudo dmesg | tail -n 20` right after plugging in. If it shows the port being created and then dropped, `brltty` is still installed; go back to step 3.

## 5. Check the IMU

From the GDN workspace:
```bash
cd ~/gdn_workspace
python3 tools/h30wp_check.py
```
It finds the port, reads for 5 seconds, decodes the data, and ends with a plain
RESULT line. A working IMU looks like this:

```
baud 460800: ... bytes, ... good frames, 0 bad checksums
frames: 200 per second, 0 lost ...
acceleration (m/s^2, FLU): ... |a| = 9.79  OK (about 9.8 at rest)
angular rate (deg/s, FLU): ...  OK (still)
magnetic field (mGauss, raw): ... |m| = ... (Earth is roughly 250 to 650)
RESULT: PASS, the IMU works
```

**Check:** RESULT says PASS, the magnitude `|a|` is close to 9.8, and the rate is steady
(the exact number depends on the IMU's settings). Tilt the IMU while it reads and run
it again. The acceleration numbers should move with the tilt, which proves they're
real readings.

## 6. If step 5 doesn't pass

The script says which case you're in:

| Message | Meaning | Do this |
|---|---|---|
| `NO PORT` | The PC doesn't see the adapter | Step 4 again: cable, `brltty` |
| `NO PERMISSION` | Your user can't open the port | Step 3: `dialout`, then log out and in |
| `NO DATA` | The adapter works, but nothing comes from the IMU | Unplug USB. Check red is on **5V** (not 3V3) and brown on **G**. Then swap green and yellow on TX/RX. Plug in and run step 5 again. |
| `BYTES BUT NO FRAMES` | Data arrives at another speed | `python3 tools/h30wp_check.py --scan` |
| `CHECK: expected about 9.8` | Readings are wrong or the IMU was moving | Make sure it's still and flat, then run again. If it stays wrong, send the output. |

If it still fails after the TX/RX swap, stop. Send the full output and the photo from
step 2.

## 7. Give it a fixed name (once)

Later the PC will also have the TFmini-S, and possibly PX4, on USB serial ports, and
`ttyUSB0` can then end up meaning a different device each boot. A udev rule gives the
IMU's adapter its own permanent name, `/dev/gdn_imu`:

```bash
python3 tools/h30wp_check.py --udev
```
It prints one line. Save that line as `/etc/udev/rules.d/99-gdn-imu.rules` (for
example with `sudo nano`), then run:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/gdn_imu
```

**Check:** `/dev/gdn_imu` exists and points to the `ttyUSB` port.

If the printed line has no `serial` part, the rule matches any adapter of that model.
That's fine while there's only one.

## 8. Run it in ROS 2

```bash
source env.sh
python3 tools/h30wp_node.py
```
It uses `/dev/gdn_imu` if it exists, otherwise the only `ttyUSB` port. To set the port
yourself: `--ros-args -p port:=/dev/ttyUSB0`.

| Topic | Type | Contents |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | acceleration in m/s², angular rate in rad/s |
| `/imu/mag` | `sensor_msgs/MagneticField` | magnetic field in tesla |
| `/imu/temperature` | `sensor_msgs/Temperature` | °C |

Every 10 s it logs the rate, lost frames and bad checksums. All three should stay
steady at 0 losses.

**Check**, in a second terminal (also `source env.sh`):
```bash
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

Axes are the IMU's own, as printed on the lid: front, left, up. That's also the ROS
convention. GDN and PX4 use front, right, down, and that conversion happens where the IMU
is wired into the filter, not here.

## 9. The static recording (R1.6)

This recording measures the IMU's noise and bias drift. The numbers then go into the
filter and the simulator. Do it once, carefully. For the rest of the procedure, see
[config/imu/README.md](../../config/imu/README.md).

**Where:**
- Put the IMU flat on something heavy and solid, such as a thick book on the floor, or a stone or concrete surface.
- Keep it away from fans, the laptop, speakers, doors, and anywhere people walk.
- Keep the room at a steady temperature: no air conditioner cycling on and off nearby, no direct sun.
- Don't touch it or the cable once it's running.

**How:**
1. Start `python3 tools/h30wp_node.py`, and let the IMU warm up for 15 minutes.
2. In a second terminal, start the recording:
   ```bash
   source env.sh
   mkdir -p log
   ros2 bag record -s mcap -o log/h30wp_static /imu/data /imu/mag /imu/temperature
   ```
3. Leave it for **at least 2 hours**. Overnight is better.
4. Stop the recording with Ctrl-C, then stop the driver.
5. Fit it:
   ```bash
   python3 tools/allan_fit.py --bag log/h30wp_static --topic /imu/data --skip 900 \
       --name h30wp --out config/imu/h30wp.yaml --plot h30wp_allan.png
   ```

**Check:** the fit prints `0 gaps`, and on the plot the points follow the fitted line.
Send me the printed output and the plot. Keep the bag; it's the raw evidence for the
model.

## What not to change yet

- **The calibrations in WHEELTEC's Windows tool (YIS Manager).** Gyro bias, magnetometer and similar change what the IMU outputs. Leave them until the static recording is done, so it measures the IMU as delivered. The magnetometer calibration only makes sense once the IMU is mounted on the drone, with its motors and wiring around it.
- **The output mode and baud rate.** The defaults work with these tools.

## Sources

- [WHEELTEC H30 repository](https://github.com/Seeed-Projects/WHEELTEC-H30-IMU): user manual (wiring, default 460800 baud, front-left-up axes) and the YESENSE Communication Protocol v2.3, which `tools/h30wp_check.py` follows. Its decoder is tested against the protocol document's example frame.
- [Seeed Studio wiki, H30 MINI / H30WP](https://wiki.seeedstudio.com/wheeltec_imu/): the H30WP's 5 to 26 V input, UART/RS-485 interfaces and 400 Hz output.
