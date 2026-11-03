# Setting up GDN

This page covers getting the workspace built and running, on a fresh machine or on
an existing one. For what the project is and where it's going, see the
[README](../README.md) and [ROADMAP](../ROADMAP.md).

## Requirements

- Ubuntu 24.04. ROS 2 Jazzy binaries are only published for 24.04 (amd64 and arm64).
  This covers a desktop, a Raspberry Pi 5 running Ubuntu 24.04, and a Jetson Orin on
  JetPack 7.2 or later.
- About 10 GB of free disk space and sudo access.
- The repo can live in any folder. Nothing assumes `~/gdn_workspace` any more.

## Fresh machine

```bash
git clone --recurse-submodules https://github.com/MadakiElisha/gdn.git ~/gdn_workspace
cd ~/gdn_workspace
tools/setup_ubuntu.sh            # add --headless on a machine without a display
source env.sh                    # in every new terminal
```

`tools/setup_ubuntu.sh` is safe to run again. It does the following:

1. Adds the ROS 2 apt repository and installs ROS 2 Jazzy.
2. Installs the build tools, Eigen and the Python packages.
3. Runs rosdep.
4. Fetches the `px4_msgs` submodule.
5. Builds the workspace. The first `px4_msgs` build takes about 5 minutes on a
   desktop, and much longer on a Pi.
6. Generates the derived data.
7. Runs the tests and writes a small synthetic bag for the bench smoke test.

## Existing machine (updating an older copy)

If your workspace predates the setup fixes (commit `742b5ca` or older), update it
like this. It keeps your existing build, so `px4_msgs` doesn't rebuild from scratch.

```bash
cd ~/gdn_workspace
git status                        # should show no modified tracked files
git pull --ff-only origin main
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release    # same mode as before, no --symlink-install
source env.sh
tools/make_data.sh                # checks the maps already in data/ against SHA256SUMS
tools/run_selftest.sh
```

The map files already in your `data/maps/` are identical to the ones now tracked in
git, so the pull replaces them without conflict. Data that git doesn't track, such as
`data/flight001`, is left alone. Untracked scripts you've added yourself stay where
they are.

Don't add `--symlink-install` to a workspace that was built without it. colcon can't
turn the existing install folders into links and fails on `px4_msgs`. If you want
symlink mode, wipe the build first and rebuild everything:

```bash
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`tools/setup_ubuntu.sh` detects an existing non-symlink build and keeps that mode.

## Manual setup

If you'd rather run each step yourself, this is what the script does.

```bash
# 1. ROS 2 Jazzy apt repository (the method in the official Jazzy install guide)
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 2. ROS, build tools and Python packages
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rclcpp-components build-essential cmake git libeigen3-dev pkg-config \
  python3-numpy python3-scipy python3-matplotlib python3-h5py python3-yaml \
  python3-pil python3-rasterio

# 3. rosdep ("already initialized" is fine)
sudo rosdep init
rosdep update
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# 4. Submodules, if you cloned without --recurse-submodules
git submodule update --init --recursive

# 5. Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source env.sh

# 6. Data and tests
tools/make_data.sh
tools/run_selftest.sh
python3 tools/make_zermatt_bag.py
```

The Python tools run on the system Python, because `rclpy` comes from apt. Don't run
them in a virtualenv unless it was created with `--system-site-packages`.

## Checking the install

| Command | What you should see |
|---|---|
| `tools/run_selftest.sh` | `ALL TESTS PASSED` twice, for `-O2` and `-O3` (24 checks each) |
| `colcon test --packages-select gdn_fusion && colcon test-result` | `1 test, 0 errors, 0 failures` |
| `tools/make_data.sh --verify-maps` | every checksum `OK`, both maps `identical` |
| `tools/mc_run.sh 24 data/flight_zermatt_csv data/maps/zermatt.bin` | CEP50 around 14 m, roughly 19 of 24 seeds passing |
| `BENCH_NO_BARO=1 GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin tools/bench.sh` | `RMS 60-300s` somewhere around 13 to 20 m, about 3 minutes |

The bench number moves between runs because ROS message timing isn't deterministic,
and the scorer only takes 6 samples in that window. Monte Carlo results are exactly
repeatable.

## Data

| Path | Where it comes from | In git? |
|---|---|---|
| `data/maps/terrain_db_sigma4.bin` | synthetic terrain, seed 42 | yes (640 KB) |
| `data/maps/zermatt.bin`, `zermatt.tif` | SRTM crop and its source GeoTIFF | yes (165 KB) |
| `data/flight_zermatt_csv/` | `tools/make_data.sh` | no, generated |
| `data/flight_zermatt/` | `python3 tools/make_zermatt_bag.py` | no, generated |
| `data/flight001/`, `data/flight001_csv/` | `tools/fetch_data.sh` (GitHub release) | no, about 100 MB |

`data/SHA256SUMS` pins every tracked or deterministic file, and `make_data.sh` checks
it. `make_data.sh --verify-maps` also rebuilds both maps from source and confirms they
match byte for byte.

`flight001` is a PX4 SITL recording, and it's the only dataset that can't be
regenerated. To publish it once from the machine that has it:

```bash
gh auth login
tools/pack_data.sh --publish      # creates release data-v1
```

After that, `tools/fetch_data.sh` downloads and verifies it on any machine. Until it's
published, copy it across directly:

```bash
rsync -av OLD_PC:gdn_workspace/data/{flight001,flight001_csv} data/
```

## Live PX4 SITL

This path hasn't been re-checked since the setup rework.

1. Use PX4 v1.15.x, to match the `px4_msgs` submodule (`release/1.15`). If the message
   definitions don't match, topics appear but nothing decodes.
2. Run the Micro XRCE-DDS Agent: `MicroXRCEAgent udp4 -p 8888`.
3. Start PX4 SITL (`make px4_sitl gz_<model>`).
4. Start `eskf_node` along with `trn_sensor_sim.py`, `mag_sim.py` and `baro_sim.py`.
   PX4 doesn't publish the GDN measurement topics; the simulators build them from PX4
   truth.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `cd: ~/gdn_workspace: No such file or directory` | Old scripts. Pull `main`; nothing depends on the clone location now. |
| `map load failed: .../data/maps/...` | `env.sh` wasn't sourced, so `GDN_ROOT` is unset. Or pass `--ros-args -p map_path:=/abs/path`. |
| `px4_msgs` missing or `src/px4_msgs` empty | Cloned without submodules: `git submodule update --init --recursive`. |
| `git pull` says untracked files "would be overwritten" | You have your own copy of a file that's now in git. Move yours aside (`mv tools/x.py ~/x.py.mine`), pull, then compare. |
| colcon fails with "existing path cannot be removed" | You switched to `--symlink-install` over an old build: `rm -rf build install log`, then rebuild. |
| `Eigen/Dense: No such file` | `sudo apt install libeigen3-dev`, or set `EIGEN_INC`. |
| `ros2 bag play` says the mcap plugin is missing | `sudo apt install ros-jazzy-rosbag2-storage-mcap` |
| Bench prints `no scorer samples` | A node didn't start. Look in `/tmp/gdn_bench/node.log`. |
| `pip install` refused ("externally managed") | Use the apt `python3-*` packages listed above. |
| Shell exits silently after `source install/setup.bash` | Don't use `set -u` around ROS setup scripts; they reference unset variables. |
| RViz shows nothing | In Jazzy, the Marker display key is `Topic:`. `Marker Topic:` is silently ignored. |
| You see nodes from another project | `env.sh` sets `ROS_DOMAIN_ID=42`. Change it if that clashes. |
| You want Zenoh instead of Fast DDS | `export GDN_RMW=rmw_zenoh_cpp` before `source env.sh` (needs `ros-jazzy-rmw-zenoh-cpp`). |
