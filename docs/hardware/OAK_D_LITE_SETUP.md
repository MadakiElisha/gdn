# Setting up the OAK-D Lite

This gets the Luxonis OAK-D Lite working on a PC, checks it has everything stereo VIO
needs, and runs it in ROS 2. We use its two global-shutter mono cameras and its IMU
for visual-inertial odometry (R3.5). Later, its colour camera is for matching images
against maps (R8.6).

The OAK-D Lite is the easiest of our sensors to set up: one USB-C cable, no separate
power, no wiring. Allow about 20 minutes.

## What it is

- Two OV7251 mono cameras, 640 x 480, global shutter, 7.5 cm apart. These are the
  ones VIO uses. Global shutter means no smearing when the drone moves fast.
- One IMX214 colour camera, 13 MP, rolling shutter.
- A BMI270 IMU on later units. **Early OAK-D Lite units (the Kickstarter batch) have
  no IMU at all.** The check script below tells you which one you have.
- An onboard processor that can do stereo depth and neural networks on the camera.

It's powered over USB, so there's no power supply to connect.

## Preparing the PC

This only needs doing once per machine.

The camera needs a udev rule, or Linux won't let normal users open it:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The check script uses Luxonis's Python library, DepthAI. Ubuntu 24.04 doesn't let pip
install into the system Python, so it goes into its own virtual environment:

```bash
sudo apt install -y python3-venv
python3 -m venv ~/.venvs/oak
~/.venvs/oak/bin/pip install depthai
```

## Plugging it in

The OAK-D Lite needs a **USB 3** connection. On USB 2 the stereo pair can't run at
full rate.

- Use a USB 3 port on the PC: usually blue, or marked SS. Avoid unpowered hubs.
- Use a cable that does USB 3. Many phone-charging cables are USB 2 only.
- If it shows up as USB 2, flip the USB-C plug over at the camera end and try again. Some cables only make the fast connection one way round.

```bash
lsusb | grep 03e7
```

You should see a Movidius / Intel line. It's normal for it to show up as `03e7:2485`
at first and change to `03e7:f63b` once software opens the camera: the camera boots
its firmware over USB.

## Checking the camera

From the GDN workspace:

```bash
~/.venvs/oak/bin/python tools/oak_check.py
```

It opens the camera and reports:
- the USB link speed;
- the cameras it finds;
- whether there's an IMU;
- the factory calibration: the measured stereo baseline and, if there's an IMU, the
  camera-to-IMU calibration.

It ends with a plain `RESULT` line.

A good result shows:
- a `SUPER` (USB 3) link;
- `CAM_B` and `CAM_C` as `OV7251`;
- an IMU (`BMI270`);
- a baseline close to 7.5 cm.

Then run the live test, which streams both mono cameras and the IMU for five seconds:

```bash
~/.venvs/oak/bin/python tools/oak_check.py --stream
```

You want about 30 fps from each camera, and the IMU at about 200 Hz reading close to
9.8 m/s² with the camera sitting still.

If it says:
- **`NO DEVICE`**: the udev rule isn't active, or the cable is the problem. Re-run the udev commands, then unplug the camera and plug it back in.
- **`USB link isn't USB 3`**: change the port, the cable, or flip the plug.
- **`no IMU on this unit`**: this is a note, not a failure. It's an early unit. VIO then has to use the H30WP as its IMU instead. That works, but needs a camera-to-H30WP calibration that includes the time offset between them (R6.4). Tell the team, because it changes the plan for R3.5.

## Running it in ROS 2

Luxonis's ROS driver for Jazzy is a normal apt package. Use the v3 one, which is the
version Luxonis develops now:

```bash
sudo apt install -y ros-jazzy-depthai-ros-v3
```

Then:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch depthai_ros_driver_v3 driver.launch.py camera_model:=OAK-D-LITE
```

Add `use_rviz:=true` to see the images. In another terminal, `ros2 topic list | grep oak`
lists what it publishes. `ros2 topic hz` on the IMU and left camera topics confirms
the rates.

The default launch runs an RGB-D pipeline with a demo neural network. That's fine for
checking the camera works. The configuration for VIO (synced, rectified mono pair
at a fixed rate, plus the IMU at its full rate, without the colour camera and
network) is part of R6.2, where we'll add it to the repo.

**Known problem on our unit (OAK-D-LITE-AF, November 2026):** the apt driver
(depthai-ros 3.2.1, which bundles DepthAI 3.10) crashes the camera a few seconds after
`Driver ready!`, then reconnects and crashes again every 12 s or so. Every crash dump
(`~/.cache/depthai/crashdumps/`) says `Start Source: Invalid config steps`, meaning a
camera sensor failed to start. It happened with the default launch, with low-bandwidth
images, with the stereo cameras only, and with `pointcloud.enable:=true`, on USB 2 and
USB 3. One low-bandwidth run lasted 3 hours, so the crash isn't every time.
`tools/oak_check.py --stream`, on DepthAI 3.12, ran the same cameras without a crash.
Until a driver built on a newer DepthAI is out, use DepthAI 3.12 directly where you can
(below).

## Recording a static IMU log (R1.6)

`tools/oak_imu_log.py` records the IMU with all three cameras running, so it warms up
as it would in use. It uses DepthAI 3.12, not the ROS driver above:

```bash
cd ~/gdn_workspace && source /opt/ros/jazzy/setup.bash
systemd-inhibit --what=sleep:idle python3 tools/oak_imu_log.py --out log/oakd_lite_static --hours 2.25 --ros
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw      # optional, in another terminal
```

- `--ros` publishes a 10 Hz preview of the three cameras while it records. Only one
  program can open the camera, so this is the only way to watch during a recording.
  Leave it out to run from `~/.venvs/oak/bin/python` without ROS.
- It prints the IMU rate, lost reports and frame rates every minute.
- It stops if the camera restarts, because what follows would be a fresh warm-up.
- The IMU arrives at about 240 Hz: the BMI270's gyro ticks every 2.53 ms, and about
  40% of the ticks don't come through. `tools/allan_fit.py` handles the uneven
  spacing (TEST-025).

Set it on something that can't rock, then keep away from the desk: typing on the same
desk shows up in the gyro. The fit: `config/imu/README.md`.

## Leave these alone

- **The calibration on the camera.** DepthAI can flash and factory-reset calibration
  (`flashCalibration`, `factoryResetCalibration`). Don't. The factory stereo
  calibration is what VIO starts from, and losing it means recalibrating by hand.
- **IMU firmware updates.** The driver can offer to update the BMI270's firmware.
  Leave it unless Luxonis support tells us to.
- **The mounting.** Where it sits on the drone, and which way it points (down or
  forward-down), is decided in R6.5. Until then, keep it on the desk or a tripod.

## References

- [DepthAI ROS driver documentation](https://docs.luxonis.com/software-v3/depthai/ros/driver/): Jazzy packages use the `_v3` suffix (`ros-jazzy-depthai-ros-v3`), and launch with `driver.launch.py`.
- [depthai-ros source](https://github.com/luxonis/depthai-ros): launch arguments (`camera_model`, `use_rviz`) and IMU parameters (default accelerometer 480 Hz, gyro 400 Hz).
- `tools/oak_check.py` uses the DepthAI 3.12 Python API, checked against the installed library, since there was no camera here to test against.
