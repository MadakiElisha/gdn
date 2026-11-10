# Setting up the Livox Mid-360S

This gets the Mid-360S lidar running on a PC: power and cabling, the network, the
Livox SDK and ROS 2 driver, and a first look at the point cloud. We use it for
lidar-inertial odometry (R3.1) and lidar height above ground, so it's the sensor
that keeps working in the dark.

Unlike the IMU, the lidar talks over Ethernet, so most of the setup is getting the PC
and the lidar onto the same small network. Allow about an hour the first time,
mostly for building the driver.

## What you need

The box has the lidar, a hex key, M3 screws and a cleaning cloth. Two things that
are **not** in the box:

- **The Livox 3-wire aviation cable** (sold as "Livox three-wire aviation connector",
  1.5 m). It screws onto the lidar's M12 socket and splits into a power lead, an
  Ethernet plug and a sync lead.
- **A 9 to 27 V DC supply.** The lidar draws about 6.5 W normally, and up to 14 W
  when it heats itself in the cold. A 12 V supply rated 2 A or more is a good bench
  choice. A DC barrel adapter with screw terminals makes connecting the power lead
  easy.

You also need an Ethernet port on the PC. A USB-to-Ethernet adapter works fine.

Note the serial number on the lidar's label before mounting it anywhere awkward.
Livox also calls it the broadcast code. We need its last two digits.

## Specs worth knowing

- 360° horizontal, -7° to 52° vertical field of view. The near blind zone is 0.1 m.
- About 40 m range on dark (10% reflectivity) surfaces, 200,000 points per second, 10 Hz frames.
- A built-in ICM40609 IMU.
- 100 Mbit Ethernet, with PTP time sync.
- IP67, 265 g.

## Cabling and power

Connect in this order:

1. Screw the aviation cable onto the lidar.
2. Connect the cable's power lead to the supply, with the supply switched off or
   unplugged. On Livox's cable red is usually + and black -, but check the label or
   the leaflet that came with your cable before switching on. Reversed polarity is the
   one mistake here that can do damage.
3. Plug the Ethernet end into the PC.
4. Switch the supply on.

When the lidar starts it makes a faint whirring sound and you can feel a slight
vibration. That's normal. The Ethernet light on the PC side should come on.

Leave the sync lead unconnected. It's for GPS time sync, which we don't use on the bench.

## The PC's network

The lidar leaves the factory at **192.168.1.1XX**, where XX is the last two digits of
its serial number. For example, serial `47MDL1C0010081` means `192.168.1.181`. The
PC's Ethernet port needs a fixed address on the same subnet. We use **192.168.1.50**.

First find the name of the Ethernet port the lidar is plugged into:

```bash
ip link
```

It's usually something like `enp3s0` or `enx...` for a USB adapter. Then give it the
fixed address. Replace `enp3s0` with your port's name:

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name livox \
    ipv4.method manual ipv4.addresses 192.168.1.50/24 ipv6.method disabled
sudo nmcli con up livox
```

You can do the same in Settings, Network: on the wired connection, set IPv4 to
Manual, address `192.168.1.50`, netmask `255.255.255.0`, and leave the gateway empty.

One catch: if your home or office router also hands out 192.168.1.x addresses on
Wi-Fi, the PC sees the same subnet twice and may send lidar traffic the wrong way.
If you hit odd problems with Wi-Fi on, turn Wi-Fi off while working with the lidar.

## Checking the lidar is reachable

From the GDN workspace, with the lidar's serial number:

```bash
python3 tools/livox_setup.py --sn 47MDL1C0010081
```

The script works out the lidar's IP from the serial number, finds the PC address on
that subnet, and pings the lidar. If that works, it writes the driver config we use,
`config/sensors/mid360s.json`, with both addresses filled in. It ends with a plain
`RESULT` line.

If it says:
- **`NO HOST ADDRESS`**: the Ethernet port doesn't have its 192.168.1.50 address. Go back to the network step. The address only exists while the cable has a live link, so check that the lidar is powered.
- **`NO REPLY`**: the PC is set up, but the lidar doesn't answer. Check the power and the polarity, the Ethernet plug, and the serial digits. If the lidar's address was ever changed (in Livox Viewer, say), it won't follow the serial number any more. Then find it with Livox Viewer 2 and pass `--ip` instead of `--sn`.

Livox Viewer 2 (version 2.4.2 or later; older versions don't know the Mid-360S) is
an optional extra check. If it lists the Mid-360S with its serial number and shows a
point cloud, the hardware side is fine.

## Building the SDK and driver

Livox's ROS 2 driver needs their SDK installed first.

**Keep the driver in its own workspace, not in `gdn_workspace`.** Livox's build
script deletes `../../build` and `../../install` relative to itself. Inside our
workspace, that would wipe the GDN build.

```bash
sudo apt install -y libpcl-dev ros-jazzy-pcl-conversions

cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j"$(nproc)"
sudo make install

mkdir -p ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/livox_ws/src/livox_ros_driver2
cd ~/livox_ws/src/livox_ros_driver2
source /opt/ros/jazzy/setup.bash
./build.sh jazzy
```

The `CMAKE_POLICY_VERSION_MINIMUM` flag is only there because the SDK declares a very
old CMake version. Ubuntu 24.04's CMake would just warn; newer ones refuse without it.

We built this sequence on Jazzy with the SDK2 and driver as of November 2026 (driver
1.2.8), and started the driver with our config. That
driver version has Mid-360S support (added in 1.2.5) and Jazzy support (1.2.6).

## Running it

```bash
cd ~/gdn_workspace
source env.sh
tools/livox_run.sh
```

This starts Livox's driver with our config, from `~/livox_ws` (set `LIVOX_WS` if it's
elsewhere). It publishes two topics:

| Topic | Type | Notes |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | 10 Hz. Per-point timestamps, which is what FAST-LIO2 and Point-LIO want. |
| `/livox/imu` | `sensor_msgs/Imu` | About 200 Hz. **Acceleration is in g, not m/s².** The driver passes Livox's units straight through. |

Check them in a second terminal:

```bash
source ~/livox_ws/install/setup.bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo --once /livox/imu
```

With the lidar standing still, the IMU's acceleration should add up to about 1.0
(that's 1 g).

To see the point cloud, run it with standard `PointCloud2` output instead:

```bash
XFER=0 tools/livox_run.sh
```

Then open `rviz2`, set the fixed frame to `livox_frame`, and add a PointCloud2 display
on `/livox/lidar`. Walk around it and you should see yourself in the cloud.

## When the driver complains

- **`bind failed` and `Init lds lidar fail!`**: the PC doesn't have the host address
  from the config, so the driver can't listen on it. Usually the Ethernet link is down
  (lidar unpowered or unplugged), and the static address goes with it. Power the lidar,
  check `ip addr` shows 192.168.1.50, and start the driver again.
- **The driver starts but no topics show data**: the ping worked but UDP isn't getting
  through. If `ufw` is on, allow the lidar's subnet with
  `sudo ufw allow from 192.168.1.0/24`. Also rerun `tools/livox_setup.py` to make sure
  the config has the right addresses.
- **Timestamps look like 1970**: the driver stamps data with the lidar's own clock. It isn't synced to the
  PC until we set up PTP (R6.3). It doesn't matter for a first look, but it does for
  recording data we fuse with other sensors.

## Leave these alone for now

- **The lidar's IP address.** Livox Viewer can change it, but then it no longer
  follows the serial number, and everyone's setup breaks. If it ever has to change,
  write the new address on a label on the lidar.
- **The extrinsics block** in `config/sensors/mid360s.json`. It stays at zero until the
  lidar is mounted and calibrated against the IMU (R6.4).
- **Which IMU LIO uses.** The Mid-360S's own IMU or the H30WP is an open question
  (R6.4). For now, record both.

## References

- [Livox Mid-360S specifications](https://www.livoxtech.com/mid-360s/specs): range, FOV, 9 to 27 V supply, 6.5 W typical and 14 W peak, Ethernet, PTP.
- [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) and [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2): build, `MID360s_config.json` format, topics. The changelog has the Mid-360S (1.2.5) and Jazzy (1.2.6) support.
- [NEXTY Electronics, "How to Use Mid-360S"](https://www.nexty-ele.com/en/technical-column/livox_12/): parts not in the box, connection order, the 192.168.1.1XX rule, Livox Viewer 2.4.2 or later.
