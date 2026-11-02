# GDN: GNSS-Denied Navigation

GDN is a terrain-referenced navigation (TRN) stack for aerial vehicles flying without
GNSS. An error-state Kalman filter (ESKF) fuses an IMU, terrain-elevation
measurements, a barometer and a magnetometer against an onboard terrain database.
The output is a bounded position estimate in a local map frame.

- **Core:** ROS-free C++17 + Eigen, 19-state ESKF, unit-tested at -O2 and -O3.
- **Integration:** a thin ROS 2 Jazzy node consuming PX4 uORB topics through `px4_msgs`.
- **Validation:** deterministic replay bench, Monte Carlo campaign, and real SRTM terrain (Zermatt).
- **Vehicle-agnostic:** the estimator contains no airframe model. Validation to date is fixed-wing-like.

> **Status: research prototype.** It navigates well in simulation under idealised
> conditions: known start position, perfect map, idealised TRN measurement. It is not
> yet a fielded GNSS-denied system. See [Known gaps](docs/ARCHITECTURE.md#known-gaps-honest-status).

---

## Quick start: new PC

Requires **Ubuntu 24.04** (ROS 2 Jazzy ships binaries only for 24.04), about 10 GB of
free disk, and sudo. The repo can live in any folder.

```bash
git clone --recurse-submodules https://github.com/MadakiElisha/gdn.git ~/gdn_workspace
cd ~/gdn_workspace
tools/setup_ubuntu.sh            # add --headless on a machine without a display
source env.sh                    # do this in every new terminal
```

`setup_ubuntu.sh` is safe to re-run. It:

1. Adds the ROS 2 apt repository.
2. Installs ROS 2 Jazzy, the build tools, Eigen and the Python dependencies.
3. Runs `rosdep`.
4. Fetches the `px4_msgs` submodule.
5. Builds the workspace. The first `px4_msgs` build takes about 5 minutes.
6. Generates the derived data.
7. Runs the tests and creates a small synthetic bag for a smoke test.

### Check the install

| Command | Expected |
|---|---|
| `tools/run_selftest.sh` | `ALL TESTS PASSED` for both `-O2` and `-O3` (24 checks each) |
| `colcon test --packages-select gdn_fusion && colcon test-result` | `1 test, 0 errors, 0 failures` |
| `tools/make_data.sh --verify-maps` | all checksums `OK`; both maps `identical` |
| `tools/mc_run.sh 24 data/flight_zermatt_csv data/maps/zermatt.bin` | CEP50 ≈ 14 m, about 19/24 seeds passing |
| `BENCH_NO_BARO=1 GDN_BAG=data/flight_zermatt GDN_MAP=data/maps/zermatt.bin tools/bench.sh` | `RMS 60-300s` about 13–20 m (takes about 3 min; varies run to run because of ROS timing and n=6 samples) |

These were verified from a fresh clone on the `fix/new-pc-setup` branch.

### Get the recorded PX4 flight (main bench)

`flight001` is a ~100 MB PX4 SITL recording. It is the one dataset that can't be
regenerated from the repo, so it ships as a GitHub release asset:

```bash
tools/fetch_data.sh          # downloads + verifies data/flight001 and data/flight001_csv
tools/bench.sh               # replay bench on the sigma4 map
tools/mc_run.sh 100          # Monte Carlo on flight001
```

If the release isn't published yet, copy it from the old machine:

```bash
rsync -av OLD_PC:gdn_workspace/data/{flight001,flight001_csv} data/
```

**Owner, one time:** on the machine that has the recording, run
`tools/pack_data.sh --publish`. It needs the GitHub CLI (`gh auth login`).

---

## Manual setup (what the script does)

<details>
<summary>Step-by-step, if you'd rather run each command yourself</summary>

```bash
# 1. ROS 2 Jazzy apt repository (official method)
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 2. ROS + build + Python deps
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rclcpp-components build-essential cmake git libeigen3-dev pkg-config \
  python3-numpy python3-scipy python3-matplotlib python3-h5py python3-yaml \
  python3-pil python3-rasterio

# 3. rosdep
sudo rosdep init          # only once per machine; "already initialized" is fine
rosdep update
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --rosdistro jazzy

# 4. submodules (if you cloned without --recurse-submodules)
git submodule update --init --recursive

# 5. build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source env.sh

# 6. data + tests
tools/make_data.sh
tools/run_selftest.sh
python3 tools/make_zermatt_bag.py
```
</details>

Python tools use the system Python, because `rclpy` comes from apt. Don't run them
inside a virtualenv unless it was created with `--system-site-packages`.

---

## Running

Run `source env.sh` first in every terminal.

| Task | Command |
|---|---|
| Unit tests (no ROS needed) | `tools/run_selftest.sh` |
| Monte Carlo, flight001 | `tools/mc_run.sh 100` |
| Monte Carlo, Zermatt | `tools/mc_run.sh 24 data/flight_zermatt_csv data/maps/zermatt.bin` |
| Replay bench, flight001 | `tools/bench.sh` (`BENCH_NO_BARO=1` for the baro-off A/B run) |
| Replay bench, any bag/map | `GDN_BAG=... GDN_MAP=... tools/bench.sh` |
| Bench + RViz | `tools/bench_viz.sh` (needs `ros-jazzy-desktop`) |
| Node only | `ros2 run gdn_fusion eskf_node --ros-args -p map_path:=data/maps/zermatt.bin` |

Bench logs go to `/tmp/gdn_bench/` (change with `GDN_LOG_DIR`).

### Live PX4 SITL (optional, not yet re-verified on the new setup)

1. Use **PX4 v1.15.x**. It must match the `px4_msgs` submodule (`release/1.15`).
   Mismatched message definitions fail silently: topics appear but nothing decodes.
2. Install the Micro XRCE-DDS Agent and run `MicroXRCEAgent udp4 -p 8888`.
3. Start PX4 SITL (`make px4_sitl gz_...`).
4. Run `eskf_node` together with `trn_sensor_sim.py`, `mag_sim.py` and `baro_sim.py`.
   PX4 does not publish the GDN measurement topics; the simulators synthesise them from
   PX4 truth.

---

## Data

| Path | How you get it | Tracked in git? |
|---|---|---|
| `data/maps/terrain_db_sigma4.bin` | synthetic, seed 42 (`make_data.sh --verify-maps` rebuilds it) | yes (640 KB) |
| `data/maps/zermatt.bin`, `zermatt.tif` | SRTM crop + its source GeoTIFF | yes (165 KB) |
| `data/flight_zermatt_csv/` | `tools/make_data.sh` | no (generated) |
| `data/flight_zermatt/` (bag) | `python3 tools/make_zermatt_bag.py` | no (generated) |
| `data/flight001/`, `data/flight001_csv/` | `tools/fetch_data.sh` (GitHub release) | no (~100 MB) |

`data/SHA256SUMS` pins every tracked or deterministic file. `make_data.sh` checks it.

---

## Repository layout

```
.
├── env.sh                      # source in every terminal (sets GDN_ROOT, ROS, RMW)
├── src/
│   ├── gdn_fusion/             # main package
│   │   ├── include/gdn/        # ROS-free core: eskf.hpp, map_db.hpp, paths.hpp
│   │   └── src/                # eskf_node, map_viz_node, selftest, mc_campaign
│   ├── gdn_sensors/            # sensor_monitor (IMU/LPOS rate check)
│   └── px4_msgs/               # git submodule, PX4 release/1.15
├── tools/
│   ├── setup_ubuntu.sh         # one-shot new-machine setup
│   ├── make_data.sh            # derived data + checksum/map verification
│   ├── fetch_data.sh / pack_data.sh   # flight001 release download / publish
│   ├── run_selftest.sh  mc_run.sh  bench.sh  bench_viz.sh
│   ├── trn_sensor_sim.py  baro_sim.py  mag_sim.py  replay_scorer.py
│   ├── make_zermatt_bag.py  gen_zermatt_flight.py
│   ├── generate_terrain.py  export_map_db.py  make_sigma4_map.py  make_zermatt_map.py
│   ├── fetch_terrain.py  make_world_final.py      # SRTM -> map/Gazebo world pipeline
│   ├── gdn_paths.py            # shared repo-root + map helpers
│   └── (analysis scripts: analyze_ins, eskf_trn, eskf_mismatch, trn_match, py_trace, ...)
├── data/                       # see Data
└── docs/
    ├── ARCHITECTURE.md         # filter math, topics, parameters, validation, known gaps
    ├── design/eskf_design.md
    ├── open_items.md
    └── test_reports/           # TEST-002 ... TEST-012
```

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `cd: ~/gdn_workspace: No such file or directory` | Old scripts. Pull this branch; nothing depends on the clone location any more. |
| `map load failed: .../data/maps/...` | `source env.sh` wasn't run (`GDN_ROOT` unset), or pass `-p map_path:=/abs/path`. |
| `px4_msgs` not found / empty `src/px4_msgs` | Cloned without submodules: `git submodule update --init --recursive`. |
| `Eigen/Dense: No such file` | `sudo apt install libeigen3-dev`, or set `EIGEN_INC`. |
| `ros2 bag play`: storage plugin `mcap` missing | `sudo apt install ros-jazzy-rosbag2-storage-mcap` |
| Bench prints `no scorer samples` | A node failed to start; check `/tmp/gdn_bench/node.log`. |
| `pip install` refused ("externally managed") | Use the apt `python3-*` packages from the setup script. |
| Shell dies silently after `source install/setup.bash` | Don't use `set -u` around ROS setup scripts; they reference unset variables. |
| RViz shows nothing | In Jazzy, the Marker display key is `Topic:`. `Marker Topic:` is silently ignored. |
| Nodes from another project show up | `env.sh` sets `ROS_DOMAIN_ID=42`; change it if needed. |
| Want Zenoh instead of Fast DDS | `export GDN_RMW=rmw_zenoh_cpp` before `source env.sh` (needs `ros-jazzy-rmw-zenoh-cpp`). |

### Field notes (hard-won)

- **Check before assuming a sensor exists in a bag:** run `ros2 bag info`.
  `SensorCombined` has no barometer field.
- **Covariance PSD facts:**
  - ΦPΦᵀ and JPJᵀ are congruence transforms, so PSD in means PSD out.
  - Shrinking a diagonal block while keeping its cross-terms breaks PSD.
- **A scalar baro observes only z + bias.** Test the observable sum, not the individual
  states.
- **Bench scorer resolution:** n is about 12 samples in the 60–300 s window, so treat
  deltas under about 3 m with caution. Use A/B pairs with pre-registered decision rules.

---

## Contributing discipline

Hypothesis → one decisive experiment → pre-registered decision rule → commit. Every
change keeps `tools/run_selftest.sh` and `colcon test` green. Changes to the core also
re-run the Monte Carlo campaign and the bench.

## References

- J. Solà, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508, 2017.
- Bar-Shalom, Li, Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
- L. D. Hostetler, SITAN recursive terrain-aided navigation (IEEE, 1978–85).
- PX4 ROS 2 user guide; ROS 2 Jazzy installation docs.

## License

MIT (see `src/gdn_fusion/LICENSE`).
