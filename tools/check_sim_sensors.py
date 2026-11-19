#!/usr/bin/env python3
"""Check the simulated sensors against the real ones (roadmap R2.4a TEST-035, R2.4b TEST-036).

    python3 tools/check_sim_sensors.py                   # the payload and the sensor config
    python3 tools/check_sim_sensors.py --run LOG_DIR     # a tools/sim_run.sh flight's bag
    python3 tools/check_sim_sensors.py --selfcheck       # the statistics, on made-up data

REAL below is what each real driver publishes (docs/hardware/) and what the bench logs
measured (TEST-024, TEST-025). A run passes if every simulated sensor, as recorded:
  - publishes on the real topic, with the real message type and frame;
  - at the real rate, and (where measured) with the real spacing spread: the Mid-360S's
    jittered stamps, the OAK-D Lite's skipped gyro ticks;
  - in the real units: at rest on the pad the specific force is gravity (the Mid-360S in
    g, the others in m/s^2);
  - with its R1.6 white noise: at rest, each axis's sample sigma within 25% of the model's
    density times sqrt(mean 1/dt) (gdn::ImuErrorSim adds sigma N / sqrt(dt) per sample);
  - with the configured timing (config/sim/sensors.yaml; for an IMU, a latency under
    half of Gazebo's 4 ms sample interval comes out as half of it, since a sample goes out
    only once the Gazebo sample after it is in), on Gazebo's clock (R2.4b: the
    stamps are Gazebo time plus the offset sim_sensors logs, and a receive time in the
    bag is turned into Gazebo time through the truth's, so a run slower than real time
    is judged in simulated seconds): arrival minus stamp, its median within 2 ms (the
    recorder's own transport delay) of the latency (sensor stamps) or near zero
    (arrival stamps, which carry the latency in the stamp itself), its 1st percentile
    not below -2.5 ms (the clock reference's own delivery jitter is about 2 ms:
    TEST-036); and the share of messages lost within 1 percentage point of
    drop_prob;
  - the TFmini-S: the real Range fields (infrared, 0.1 to 12 m, 2 degree field of view);
    at rest its mean within 5 cm of the true height of the sensor above the ground and
    its sigma within 30% of the model (or "too close", -inf, if it's parked nearer than
    0.1 m to the ground, or within 5 cm above that, where the rendered surface it sees
    and the map differ by a few cm); in flight, +inf above 12 m, and within 12 m its
    error RMS (against the true range along the beam) under twice the model's sigma;
  - the OAK-D Lite's cameras (R2.4b): left and right camera_info on the real topics and
    frames, 640 x 480, the pinhole of a 73 degree field of view with no distortion, the
    right camera's projection carrying the 7.5 cm baseline (Tx = -fx * B); every left
    image has a right one with the same stamp; 30 Hz; the configured timing. If the
    images were recorded (GDN_BAG_IMAGES=1, in LOG_DIR/image_bag): mono8 at 640 x 480 with the camera_info's
    stamps; exposed (median image mean 70 to 170 grey); the read noise as set (two
    images in a row at rest differ by sqrt(2) times it, within 25%); and the stereo
    depth: from 1.5 to 15 m up, the truth's disparity at every pixel (the left
    camera's rays cast onto the world's terrain from its true pose and mount) is
    scaled from -1 to 1.5 and the left image shifted by it; the scale that best
    reproduces the right image, pooled over the frames, must be 0.95 to 1.05 and fit
    at least 5% better than no disparity, judged where the right image has texture
    (the camera often sees only the flat-coloured runway), over the pairs that hold
    disparity at all (their best fit at least 10% better than none: snow or a flat pad
    edge has contrast but no depth to show). A flight with fewer than 5 such pairs (a
    plane climbing out along the runway) gets a note, not a verdict. A swapped pair (-1), a wrong baseline,
    focal length, mount or image time fails.

The static check also reads the payload's two Gazebo cameras: 16-bit grey (linear light
for the exposure), the config's size, field of view and rate, noise- and distortion-
free, pointing the same way 7.5 cm apart with the left one at +y, beside the OAK-D
Lite's IMU and pointing as it does (one board).

Exit 0 if everything passes, 1 if anything fails, 2 if something can't be read.
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
ROOT = Path(__file__).resolve().parents[1]
G_WORLD = 9.8          # our worlds' gravity (make_sim_world.py: <gravity>0 0 -9.8)
G_STD = 9.80665        # what a real IMU's "g" is

# What the real drivers publish, and what the bench measured.
REAL = {
    "h30wp": dict(topic="/imu/data", type="sensor_msgs/msg/Imu", frame="imu_link", rate=(194.0, 206.0),
                  in_g=False, spread=None, model="config/imu/h30wp.yaml",
                  why="tools/h30wp_node.py; TEST-024: 200.004 Hz, 0 lost"),
    "mid360s_imu": dict(topic="/livox/imu", type="sensor_msgs/msg/Imu", frame="livox_frame", rate=(194.0, 206.0),
                        in_g=True, spread=(0.10, 0.20), model="config/imu/mid360s.yaml",
                        why="livox_ros_driver2 1.2.8 (acceleration in g); TEST-025: 200 Hz, spacing spread 15%"),
    "oakd_imu": dict(topic="/oak/imu/data", type="sensor_msgs/msg/Imu", frame="oak_imu_frame", rate=(230.0, 255.0),
                     in_g=False, spread=(0.20, 0.38), model="config/imu/oakd_lite.yaml",
                     why="TEST-025: 239 Hz (248 via the ROS driver), skipped gyro ticks, spread 29%"),
    "tfmini": dict(topic="/tfmini/range", type="sensor_msgs/msg/Range", frame="tfmini_link", rate=(97.0, 103.0),
                   why="Benewake TFmini-S datasheet: 100 Hz, 0.1 to 12 m, 2 degrees"),
}
RAW_REF = "/gdn/sim/raw/imu_h30wp"      # Gazebo-stamped, in the bag since R2.4b (RunClock)
# R2.4c: the Mid-360S lidar as livox_ros_driver2 1.2.8 publishes it (xfer_format 1). The
# numbers it's judged against come from a recording of our unit, as livox_stats() makes
# them (--livox-reference): docs/test_reports/test037_data/mid360s_real.json.
LIVOX = dict(name="mid360s_lidar", topic="/livox/lidar", type="livox_ros_driver2/msg/CustomMsg",
             frame="livox_frame", gz="gdn/mid360s",
             reference=ROOT / "docs/test_reports/test037_data/mid360s_real.json",
             pattern_sha256="aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a")
# R2.4b: the OAK-D Lite's mono pair as depthai_ros_driver_v3 publishes it.
STEREO = dict(name="oakd_stereo", width=640, height=480, hfov_deg=73.0, baseline_m=0.075, rate=(29.0, 31.0),
              encoding="mono8",
              left=dict(topic="/oak/left/image_raw", info="/oak/left/camera_info",
                        frame="oak_left_camera_optical_frame", gz="gdn/oak/left"),
              right=dict(topic="/oak/right/image_raw", info="/oak/right/camera_info",
                         frame="oak_right_camera_optical_frame", gz="gdn/oak/right"),
              why="Luxonis OAK-D Lite: OV7251 640 x 480 global shutter, 73 x 58 deg, 7.5 cm; depthai-ros v3 names")


def ck(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
    return bool(ok)


def load_config(path=ROOT / "config/sim/sensors.yaml"):
    import yaml
    return yaml.safe_load(Path(path).read_text())["gdn_sim_sensors"]["ros__parameters"]


def load_model(path):
    m = {}
    for line in (ROOT / path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = (s.strip() for s in line.split(":", 1))
            if v.startswith("["):
                m[k] = np.array([float(x) for x in v.strip("[]").split(",")])
            else:
                try:
                    m[k] = float(v)
                except ValueError:
                    m[k] = v
    return m


# --- statistics (unit-checked by --selfcheck) ---

def spacing_spread(t):
    dt = np.diff(np.sort(t))
    return float(np.std(dt) / np.mean(dt))


def expected_sigma(density, t):
    """Per-sample white-noise sigma of a stream with sample times t, when each sample
    carries density / sqrt(its own interval): density * sqrt(mean(1 / dt))."""
    dt = np.diff(np.sort(t))
    dt = dt[dt > 0]
    return np.asarray(density) * math.sqrt(float(np.mean(1.0 / dt)))


def lost_share(t, nominal_dt, skip_ticks=False):
    """Share of messages missing from a stream with nominal spacing nominal_dt. With
    skip_ticks (the OAK-D), the spacing is one or two ticks by design, so only gaps
    of three or more ticks count, each as (gap / tick - 2) lost."""
    dt = np.diff(np.sort(t))
    if skip_ticks:
        n_lost = np.clip(np.round(dt / nominal_dt) - 2, 0, None).sum()
    else:
        n_lost = np.clip(np.round(dt / nominal_dt) - 1, 0, None).sum()
    return float(n_lost / (len(dt) + n_lost))


def tfmini_sigma(r, cfg):
    r = np.asarray(r, dtype=float)
    return np.where(r < cfg["near_limit_m"], cfg["noise_near_m"], cfg["noise_far_frac"] * r)


# --- the payload and config ---

def check_static():
    ok = True
    sdf = re.sub(r"<!--.*?-->", "", (ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    cfg = load_config()
    for name, real in REAL.items():
        c = cfg.get(name)
        if c is None:
            ok &= ck(False, f"{name}: in config/sim/sensors.yaml")
            continue
        ok &= ck(c["topic"] == real["topic"] and c["frame_id"] == real["frame"],
                 f"{name}: publishes the real topic and frame", f"{c['topic']} ({c['frame_id']})")
        raw = c["raw_topic"].rsplit("/", 1)[-1]
        gz_topic = {"imu_h30wp": "gdn/imu/h30wp", "imu_mid360s": "gdn/imu/mid360s", "imu_oakd": "gdn/imu/oakd",
                    "tfmini": "gdn/tfmini"}.get(raw)
        sensor = re.search(rf'<sensor name="[^"]+" type="([a-z_]+)">(?:(?!</sensor>).)*<topic>{gz_topic}</topic>'
                           r"(?:(?!</sensor>).)*</sensor>", sdf, re.S)
        noise = sensor is not None and "<noise>" in sensor.group(0)
        ok &= ck(sensor is not None and not noise, f"{name}: an error-free Gazebo sensor in gdn_payload",
                 f"{sensor.group(1)} on {gz_topic}" if sensor else f"no sensor on {gz_topic}")
        if c["kind"] == "imu":
            m = load_model(c["model"])
            ok &= ck(c["model"] == real["model"] and m.get("name"), f"{name}: its R1.6 model", c["model"])
            ok &= ck(bool(c["accel_in_g"]) == real["in_g"], f"{name}: acceleration units",
                     "g" if c["accel_in_g"] else "m/s^2")
    ok &= check_static_stereo(sdf, cfg)
    ok &= check_static_livox(sdf, cfg)
    return ok


def rpy_matrix(r, p, y):
    """Rotation of an SDF pose's roll, pitch, yaw (fixed axes x, y, z: R = Rz Ry Rx)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def payload_sensor(sdf, topic):
    """(type, pose [x y z r p y], body) of the payload sensor publishing on topic."""
    m = re.search(rf'<sensor name="[^"]+" type="([a-z_]+)">\s*<pose>([^<]+)</pose>((?:(?!</sensor>).)*'
                  rf"<topic>{topic}</topic>(?:(?!</sensor>).)*)</sensor>", sdf, re.S)
    if not m:
        return None
    return m.group(1), np.array([float(x) for x in m.group(2).split()]), m.group(3)


def check_static_stereo(sdf, cfg):
    ok = True
    st = STEREO
    c = cfg.get(st["name"])
    if c is None or c.get("kind") != "stereo" or st["name"] not in cfg["sensors"]:
        return ck(False, f"{st['name']}: a stereo sensor in config/sim/sensors.yaml")
    for side in ("left", "right"):
        r = st[side]
        ok &= ck(c[f"{side}_topic"] == r["topic"] and c[f"{side}_info_topic"] == r["info"]
                 and c[f"{side}_frame_id"] == r["frame"], f"{st['name']}: {side} on the real topics and frame",
                 f"{c[side + '_topic']}, {c[side + '_info_topic']} ({c[side + '_frame_id']})")
    ok &= ck((c["width"], c["height"]) == (st["width"], st["height"]) and c["hfov_deg"] == st["hfov_deg"]
             and c["baseline_m"] == st["baseline_m"] and st["rate"][0] <= c["rate_hz"] <= st["rate"][1],
             f"{st['name']}: the OAK-D Lite's {st['width']} x {st['height']}, {st['hfov_deg']:.0f} deg, "
             f"{100 * st['baseline_m']:.1f} cm", f"{c['width']} x {c['height']}, {c['hfov_deg']} deg, "
             f"{c['baseline_m']} m, {c['rate_hz']} Hz")
    cams = {side: payload_sensor(sdf, st[side]["gz"]) for side in ("left", "right")}
    if not all(cams.values()):
        return ck(False, f"{st['name']}: two Gazebo cameras in gdn_payload", "missing " + ", ".join(
            st[k]["gz"] for k, v in cams.items() if not v))
    for side, (typ, pose, body) in cams.items():
        num = {k: float(v) for k, v in re.findall(r"<(width|height|horizontal_fov|update_rate)>([^<]+)<", body)}
        num["update_rate"] = float(re.search(r"<update_rate>([^<]+)<", body).group(1))
        fmt = re.search(r"<format>([^<]+)<", body)
        ok &= ck(typ == "camera" and fmt is not None and fmt.group(1) == "L16" and "<noise>" not in body
                 and "<distortion>" not in body and "<lens>" not in body,
                 f"{st['name']}: {side} is an error-free 16-bit grey Gazebo camera",
                 f"{typ}, {fmt.group(1) if fmt else 'no format'} on {st[side]['gz']}")
        ok &= ck(num.get("width") == c["width"] and num.get("height") == c["height"]
                 and abs(math.degrees(num.get("horizontal_fov", 0.0)) - c["hfov_deg"]) < 0.01
                 and num["update_rate"] == c["rate_hz"],
                 f"{st['name']}: {side} camera as the config (size, field of view, rate)",
                 f"{num.get('width'):.0f} x {num.get('height'):.0f}, "
                 f"{math.degrees(num.get('horizontal_fov', 0.0)):.2f} deg, {num['update_rate']:.0f} Hz")
    (_, pl, _), (_, pr, _) = cams["left"], cams["right"]
    R = rpy_matrix(*pl[3:])
    off = R.T @ (pl[:3] - pr[:3])          # left minus right, in the camera's own frame
    ok &= ck(np.allclose(pl[3:], pr[3:]) and np.allclose(off, [0.0, c["baseline_m"], 0.0], atol=0.001),
             f"{st['name']}: parallel, {100 * c['baseline_m']:.1f} cm apart, left at +y (left of the view)",
             f"left minus right in the camera frame {np.round(off, 4).tolist()} m")
    imu = payload_sensor(sdf, "gdn/imu/oakd")
    if imu:
        mid = 0.5 * (pl[:3] + pr[:3])
        ok &= ck(np.allclose(imu[1][3:], pl[3:]) and np.linalg.norm(mid - imu[1][:3]) < 0.05,
                 f"{st['name']}: on one board with the OAK-D Lite's IMU (same direction, within 5 cm)",
                 f"{100 * np.linalg.norm(mid - imu[1][:3]):.1f} cm apart, pitch "
                 f"{math.degrees(pl[4]):.0f} deg")
    else:
        ok &= ck(False, f"{st['name']}: the OAK-D Lite's IMU in gdn_payload")
    return ok


# --- a run ---

def read_bag(bag, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    cls = {t: get_message(ty) for t, ty in types.items()}
    out = {t: [] for t in topics}
    truth = []
    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic in out:
            out[topic].append((t_ns * 1e-9, deserialize_message(data, cls[topic])))
        elif topic == "/gdn/sim/truth_odom":
            m = deserialize_message(data, cls[topic])
            p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
            truth.append((t_ns * 1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z, math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
                          stamp(m)))
    return types, out, np.array(truth)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def stamp_ns(m):
    """The stamp as an exact integer, to match messages by (R2.4b: rounding seconds to 6
    decimals matched only some, numpy and Python rounding differing in the last digit)."""
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def rest_window(truth):
    """Times (bag clock) while the vehicle stands still on the pad before it first moves
    (the noise and gravity checks at rest use it):
    from the first truth sample to 1 s before it first moves faster than 0.05 m/s or
    turns by 0.05 degrees (R2.4b: on the Zermatt pad the x500 tilts 0.24 degrees as its
    motors spool up, a second before it moves)."""
    q0 = truth[0, 4:8]
    turn = np.degrees(2 * np.arccos(np.clip(np.abs(truth[:, 4:8] @ q0), 0.0, 1.0)))
    moving = np.nonzero((truth[:, 8] > 0.05) | (turn > 0.05))[0]
    t_end = truth[moving[0], 0] - 1.0 if len(moving) else truth[-1, 0]
    return truth[0, 0], t_end


def check_imu(name, real, c, msgs, t_rest, clock):
    ok = True
    arr = np.array([t for t, _ in msgs])
    st = np.array([stamp(m) for _, m in msgs])
    a = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in msgs])
    w = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in msgs])
    frames = {m.header.frame_id for _, m in msgs}
    ok &= ck(frames == {real["frame"]}, f"{name}: frame", ", ".join(sorted(frames)))
    rate = (len(st) - 1) / (st.max() - st.min())
    ok &= ck(real["rate"][0] <= rate <= real["rate"][1], f"{name}: rate {real['rate'][0]:.0f} to {real['rate'][1]:.0f} Hz",
             f"{rate:.1f} Hz")
    sp = spacing_spread(st)
    if real["spread"]:
        ok &= ck(real["spread"][0] <= sp <= real["spread"][1],
                 f"{name}: spacing spread {100 * real['spread'][0]:.0f} to {100 * real['spread'][1]:.0f}% (measured)",
                 f"{100 * sp:.1f}%")
    rest = (arr >= t_rest[0]) & (arr <= t_rest[1])
    if rest.sum() < 200:
        return ck(False, f"{name}: at rest on the pad", f"only {rest.sum()} samples")
    scale = G_STD if c["accel_in_g"] else 1.0
    g = float(np.linalg.norm(a[rest].mean(axis=0)) * scale)
    ok &= ck(abs(g - G_WORLD) < 0.01 * G_WORLD, f"{name}: at rest it reads gravity, in {'g' if c['accel_in_g'] else 'm/s^2'}",
             f"|a| {g:.3f} m/s^2 ({g / G_STD:.4f} g) over {rest.sum()} samples")
    m = load_model(c["model"])
    for what, x, dens, unit in (("accel", a * scale, m["accel_noise_density"], "m/s^2"),
                                ("gyro", w, m["gyro_noise_density"], "rad/s")):
        sig = np.std(x[rest] - x[rest].mean(axis=0), axis=0)
        exp = expected_sigma(dens, st[rest])
        ratio = sig / exp
        ok &= ck(np.all(np.abs(ratio - 1.0) < 0.25), f"{name}: {what} white noise as its R1.6 model (within 25%)",
                 f"sigma {' '.join(f'{s:.2e}' for s in sig)} {unit}, model {' '.join(f'{e:.2e}' for e in exp)}")
    gz = {"h30wp": "gdn/imu/h30wp", "mid360s_imu": "gdn/imu/mid360s", "oakd_imu": "gdn/imu/oakd"}[name]
    sdf = re.sub(r"<!--.*?-->", "", (ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    step_ms = 1000.0 / float(re.search(r"<update_rate>([^<]+)<", payload_sensor(sdf, gz)[2]).group(1))
    ok &= check_timing(name, c, arr, st, 1.0 / (c["tick_hz"] if c["tick_hz"] > 0 else c["rate_hz"]), c["tick_hz"] > 0,
                       clock, step_ms)
    return ok


class RunClock:
    """Gazebo's clock in a run. sim_sensors stamps are Gazebo time plus an offset it logs
    at start-up ("clock: stamps are Gazebo time + ... s"); the bag's receive times are
    wall time, which runs faster than Gazebo's when the simulation is slower than real
    time. A Gazebo-stamped stream received in the same bag converts one to the other:
    the raw H30WP (ref, 250 Hz, bridged as sim_sensors gets it) or else the truth
    (20 Hz). Its receive-minus-Gazebo time, a running median over 5 samples (no lag,
    whatever the real-time factor), interpolated."""

    def __init__(self, truth, offset, ref=None):
        self.offset = offset
        ref = np.asarray(ref) if ref is not None and len(ref) > 100 else truth[:, [0, 9]]
        order = np.argsort(ref[:, 0])
        self.recv = ref[order, 0]
        g = self.recv - ref[order, 1]
        pad = np.pad(g, 2, mode="edge")
        self.g = np.median(np.lib.stride_tricks.sliding_window_view(pad, 5), axis=1)
        self.truth = truth.copy()
        self.truth[:, 0] = truth[:, 9]                      # the truth by Gazebo time

    def sim_of_wall(self, w):
        return np.asarray(w) - np.interp(w, self.recv, self.g)

    def sample(self, st):
        """Gazebo time of a sim_sensors stamp."""
        return np.asarray(st) - self.offset

    @staticmethod
    def offset_from_log(path):
        m = re.search(r"clock: stamps are Gazebo time \+ ([-0-9.]+) s", Path(path).read_text(errors="replace"))
        return float(m.group(1)) if m else None


def check_timing(name, c, arr, st, nominal_dt, skip_ticks, clock, step_ms=0.0):
    """arrival minus stamp on Gazebo's clock: the Gazebo time when the bag received a
    message, less its stamp's Gazebo time. step_ms: for an IMU, Gazebo's sample interval.
    Its samples are interpolated between two Gazebo samples and go out once the second is
    in (R2.4b), so a latency under one step comes out longer: for a sample time anywhere
    in the step, the median delay is the larger of the latency and half a step."""
    ok = True
    lat = (clock.sim_of_wall(arr) - clock.sample(st)) * 1000.0
    med = float(np.median(lat))
    want = max(c["latency_ms"], 0.5 * step_ms) if c["stamp"] == "sensor" else 0.0
    p1 = float(np.percentile(lat, 1))
    ok &= ck(abs(med - want) <= 2.0 and p1 > -2.5 - 3 * c["stamp_jitter_ms"],
             f"{name}: arrival minus stamp {want:.1f} ms on Gazebo's clock ({c['stamp']} stamps, latency "
             f"{c['latency_ms']} ms" + (f", Gazebo samples every {step_ms:.0f} ms" if step_ms else "") + ")",
             f"median {med:.2f} ms, 1st percentile {p1:.2f}, max {lat.max():.2f}")
    lost = lost_share(st, nominal_dt, skip_ticks)
    ok &= ck(abs(lost - c["drop_prob"]) <= 0.01, f"{name}: messages lost {100 * c['drop_prob']:.1f}% (within 1 point)",
             f"{100 * lost:.2f}%")
    return ok


def truth_at(truth, t):
    """The truth (rows t, x, y, z, qw, qx, qy, qz, speed) at time t, interpolated: it
    comes at 20 Hz, and 50 ms is 15 cm at a 3 m/s climb against a 3 cm sensor."""
    k = int(np.clip(np.searchsorted(truth[:, 0], t), 1, len(truth) - 1))
    a, b = truth[k - 1], truth[k]
    f = float(np.clip((t - a[0]) / max(b[0] - a[0], 1e-9), 0.0, 1.0))
    out = a + f * (b - a)
    qb = b[4:8] if np.dot(a[4:8], b[4:8]) >= 0 else -b[4:8]
    q = a[4:8] + f * (qb - a[4:8])
    out[4:8] = q / np.linalg.norm(q)
    return out


def check_tfmini(real, c, msgs, truth, t_rest, mount, world, clock):
    ok = True
    m0 = msgs[0][1]
    ok &= ck({m.header.frame_id for _, m in msgs} == {real["frame"]}, "tfmini: frame", m0.header.frame_id)
    ok &= ck(m0.radiation_type == 1 and abs(m0.min_range - 0.1) < 1e-6 and abs(m0.max_range - 12.0) < 1e-6
             and abs(math.degrees(m0.field_of_view) - 2.0) < 0.01, "tfmini: Range fields (infrared, 0.1 to 12 m, 2 deg)",
             f"type {m0.radiation_type}, {m0.min_range} to {m0.max_range} m, {math.degrees(m0.field_of_view):.2f} deg")
    arr = np.array([t for t, _ in msgs])
    st = np.array([stamp(m) for _, m in msgs])
    r = np.array([m.range for _, m in msgs])
    rate = (len(st) - 1) / (st.max() - st.min())
    ok &= ck(real["rate"][0] <= rate <= real["rate"][1], "tfmini: rate 97 to 103 Hz", f"{rate:.1f} Hz")
    # The truth's prediction of each reading: cast the beam (body down from the mount).
    from check_vehicles import cast
    alt, hdr, z0 = world
    pred = np.full(len(r), np.nan)
    # The reading's sample time, on Gazebo's clock: its stamp (the arrival) less the
    # configured latency.
    t_sample = clock.sample(st) - c["latency_ms"] * 1e-3
    for i in range(len(r)):
        tr = truth_at(clock.truth, t_sample[i])
        q = tr[4:8]
        qw, qx, qy, qz = q
        R = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                      [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
                      [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])
        p = tr[1:4] + R @ np.asarray(mount)
        hit = cast(alt, hdr, z0, p[0], p[1], p[2], q, min_height=0.0)
        if hit is not None:
            pred[i] = hit[0]
    at_rest = (arr >= t_rest[0]) & (arr <= t_rest[1]) & np.isfinite(pred)
    near_limit = at_rest.sum() > 100 and np.median(pred[at_rest]) < c["min_range"] + 0.05
    share = float(np.mean(np.isneginf(r[at_rest]))) if at_rest.any() else 0.0
    if near_limit and (np.median(pred[at_rest]) < c["min_range"] or share > 0.9):
        # Parked closer than its minimum range (a VTOL on its belly): "too close", -inf.
        # Within 5 cm above it, too: the ground it sees is the rendered surface, a few cm
        # off the map there (the runway strip is drawn 2 cm high; TEST-036: 12.1 cm by
        # the map on the Zermatt pad, -inf as read).
        ok &= ck(share > 0.9, "tfmini: at rest closer than 0.1 m (or within 5 cm of it), it reads too close (-inf), "
                 "as the real one", f"{100 * share:.0f}% -inf, true range {np.median(pred[at_rest]):.3f} m by the map")
        at_rest[:] = False
    rest = at_rest & np.isfinite(r)
    if not at_rest.any():
        pass
    elif rest.sum() > 100:
        bias = float(np.mean(r[rest] - pred[rest]))
        sig = float(np.std(r[rest]))
        exp = float(np.mean(tfmini_sigma(pred[rest], c)))
        ok &= ck(abs(bias) < 0.05 and abs(sig / exp - 1.0) < 0.3, "tfmini: at rest, the true height and the model's noise",
                 f"mean {np.mean(r[rest]):.3f} m against {np.mean(pred[rest]):.3f} m, sigma {sig:.3f} m (model {exp:.3f})")
    else:
        ok &= ck(False, "tfmini: at rest on the pad", f"{rest.sum()} finite readings")
    fly = ~((arr >= t_rest[0]) & (arr <= t_rest[1])) & np.isfinite(pred)
    far = fly & (pred > c["max_range"] + 0.5)
    ok &= ck(far.sum() > 0 and np.all(np.isposinf(r[far])), "tfmini: +inf beyond 12 m",
             f"{np.isposinf(r[far]).sum()} of {far.sum()} readings")
    near = fly & (pred > 0.3) & (pred < c["max_range"] - 0.5) & np.isfinite(r)
    if near.sum() > 20:
        e = r[near] - pred[near]
        rms, lim = float(np.sqrt(np.mean(e ** 2))), float(2.0 * np.sqrt(np.mean(tfmini_sigma(pred[near], c) ** 2)))
        ok &= ck(rms < lim, "tfmini: in flight within 12 m, error RMS under twice the model",
                 f"RMS {rms:.3f} m (limit {lim:.3f}) over {near.sum()} readings")
    else:
        ok &= ck(False, "tfmini: in flight within 12 m", f"only {near.sum()} readings")
    ok &= check_timing("tfmini", c, arr, st, 1.0 / c["rate_hz"], False, clock)
    return ok


def quat_matrix(q):
    qw, qx, qy, qz = q
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
                     [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
                     [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])


def pinhole(width, height, hfov_deg):
    """fx (= fy), cx, cy of a distortion-free camera (gdn::simsens::PinholeFromHfov)."""
    fx = 0.5 * width / math.tan(math.radians(hfov_deg) / 2)
    return fx, 0.5 * (width - 1), 0.5 * (height - 1)


def ray_depth(world, origin, dirs, z_max=40.0, step=0.05):
    """Depth (along the optical axis) at which each ray hits the terrain. dirs are world
    directions scaled so that their component along the optical axis is 1, so the
    distance marched along one is the depth. NaN beyond z_max."""
    from make_sim_world import sample_map
    alt, hdr, z0 = world
    s = np.arange(0.0, z_max + step, step)
    x = origin[0] + dirs[:, 0:1] * s
    y = origin[1] + dirs[:, 1:2] * s
    z = origin[2] + dirs[:, 2:3] * s
    below = z <= sample_map(alt, hdr, x, y) - z0
    hit = below.any(axis=1)
    k = np.argmax(below, axis=1)
    out = np.full(len(dirs), np.nan)
    for i in np.nonzero(hit & (k > 0))[0]:
        lo, hi = s[k[i] - 1], s[k[i]]
        for _ in range(12):                 # to about 10 micrometres
            mid = 0.5 * (lo + hi)
            g = float(sample_map(alt, hdr, np.array(origin[0] + dirs[i, 0] * mid),
                                 np.array(origin[1] + dirs[i, 1] * mid))) - z0
            lo, hi = (mid, hi) if origin[2] + dirs[i, 2] * mid > g else (lo, mid)
        out[i] = 0.5 * (lo + hi)
    return out


def true_disparity(world, tr, mount, c, grid):
    """Disparity the truth gives at the left camera's pixels grid (u, v): fx * B / depth
    along the ray cast from the left camera's true pose onto the world's terrain."""
    fx, cx, cy = pinhole(c["width"], c["height"], c["hfov_deg"])
    pos, rpy = mount
    Rb = quat_matrix(tr[4:8])
    Rc = Rb @ rpy_matrix(*rpy)                              # gz camera frame: x view, y left, z up
    origin = tr[1:4] + Rb @ pos
    d_cam = np.stack([np.ones(len(grid)), -(grid[:, 0] - cx) / fx, -(grid[:, 1] - cy) / fx], axis=1)
    depth = ray_depth(world, origin, d_cam @ Rc.T)
    return fx * c["baseline_m"] / depth


SCALES = np.round(np.arange(-1.0, 1.501, 0.05), 2)


def dense_disparity(world, tr, mount, c, step=8):
    """The truth's disparity at every pixel of the left image (cast every step pixels,
    interpolated); 0 where the ray finds no ground within 40 m."""
    import cv2
    gu, gv = np.meshgrid(np.arange(0, c["width"], step), np.arange(0, c["height"], step))
    grid = np.stack([gu.ravel(), gv.ravel()], axis=1).astype(float)
    d = true_disparity(world, tr, mount, c, grid).reshape(gu.shape)
    return cv2.resize(np.nan_to_num(d, nan=0.0).astype(np.float32), (c["width"], c["height"]),
                      interpolation=cv2.INTER_LINEAR)


def warp_errors(left, right, disp, scales=SCALES, min_disp=2.0, min_contrast=4.0):
    """How well the left image, shifted by scale times the disparity disp, reproduces the
    right image, for each scale: the mean absolute difference of the two high-passed
    images where disp >= 2 px and the right image has texture (local contrast, the
    sigma of its high-passed grey over 9 x 9 pixels, at least min_contrast levels: the
    read noise alone gives under 1). The geometry is right where scale 1 fits best: a
    point at x in the left image is at x - disparity in the right. Both images are
    blurred alike first, so a fractional shift (which interpolates, and so averages
    the noise) has no edge over a whole-pixel one. None if fewer than 3000 pixels
    qualify (the camera sees only the flat-coloured runway or pad).
    Unlike a block matcher this needs no texture fine enough to match by itself:
    OpenCV's semi-global matcher read 10 to 15% short on the soft 2 m photo (TEST-036)."""
    import cv2
    disp = np.asarray(disp, np.float32)
    h, w = right.shape
    xs, ys = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))

    def hp(im):
        return im - cv2.GaussianBlur(im, (0, 0), 6)
    lf = cv2.GaussianBlur(left.astype(np.float32), (0, 0), 1.0)
    rh = hp(cv2.GaussianBlur(right.astype(np.float32), (0, 0), 1.0))
    contrast = np.sqrt(np.maximum(cv2.blur(rh * rh, (9, 9)) - cv2.blur(rh, (9, 9)) ** 2, 0.0))
    reach = float(np.max(np.abs(scales))) * disp
    mask = (disp >= min_disp) & (contrast >= min_contrast) & (xs > 16 + reach) & (xs < w - 16 - reach)
    if mask.sum() < 3000:
        return None
    return np.array([float(np.mean(np.abs(hp(cv2.remap(lf, (xs + s * disp).astype(np.float32), ys,
                                                        cv2.INTER_LINEAR)) - rh)[mask])) for s in scales])


def best_scale(err, scales=SCALES):
    """The scale at the minimum of err, refined by a parabola through its neighbours."""
    k = int(np.argmin(err))
    if 0 < k < len(err) - 1:
        a, b, c = err[k - 1], err[k], err[k + 1]
        den = a - 2 * b + c
        if den > 0:
            return float(scales[k] + 0.5 * (a - c) / den * (scales[1] - scales[0]))
    return float(scales[k])


def read_images(bag, topics, want):
    """The images on topics whose stamps (stamp_ns) are in want, as {(topic, stamp_ns): (msg, array)}."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    out = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        m = deserialize_message(data, Image)
        st = stamp_ns(m)
        if st in want:
            out[(topic, st)] = (m, np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width])
    return out


def check_stereo(c, types, msgs, t_rest, world, bag, clock):
    ok = True
    st = STEREO
    name = st["name"]
    fx, cx, cy = pinhole(c["width"], c["height"], c["hfov_deg"])
    info = {}
    for side in ("left", "right"):
        r = st[side]
        m = msgs[r["info"]]
        if not ck(len(m) > 100 and types.get(r["info"]) == "sensor_msgs/msg/CameraInfo",
                  f"{name}: {side} camera_info on {r['info']}", f"{len(m)} messages, {types.get(r['info'])}"):
            return False
        info[side] = m
        m0 = m[0][1]
        tx = -fx * c["baseline_m"] if side == "right" else 0.0
        want_p = [fx, 0, cx, tx, 0, fx, cy, 0, 0, 0, 1, 0]
        ok &= ck({x.header.frame_id for _, x in m} == {r["frame"]} and (m0.width, m0.height) == (st["width"], st["height"]),
                 f"{name}: {side} frame and size", f"{m0.header.frame_id}, {m0.width} x {m0.height}")
        ok &= ck(np.allclose(m0.k, [fx, 0, cx, 0, fx, cy, 0, 0, 1], atol=1e-6) and np.allclose(m0.p, want_p, atol=1e-6)
                 and np.allclose(m0.r, np.eye(3).ravel()) and m0.distortion_model == "plumb_bob"
                 and not np.any(m0.d),
                 f"{name}: {side} camera_info is the {st['hfov_deg']:.0f} deg pinhole, no distortion"
                 + (f", Tx = -fx * {100 * c['baseline_m']:.1f} cm" if side == "right" else ""),
                 f"fx {m0.k[0]:.2f} (want {fx:.2f}), cx {m0.k[2]:.1f}, cy {m0.k[5]:.1f}, Tx {m0.p[3]:.3f}")
    stl = np.array([stamp(x) for _, x in info["left"]])
    str_ = np.array([stamp(x) for _, x in info["right"]])
    both = np.intersect1d([stamp_ns(x) for _, x in info["left"]], [stamp_ns(x) for _, x in info["right"]])
    share = len(both) / max(len(stl), len(str_))
    ok &= ck(share > 0.999, f"{name}: every left image has a right one with the same stamp (a synced pair)",
             f"{len(both)} pairs of {len(stl)} left, {len(str_)} right")
    rate = (len(stl) - 1) / (stl.max() - stl.min())
    ok &= ck(st["rate"][0] <= rate <= st["rate"][1], f"{name}: rate {st['rate'][0]:.0f} to {st['rate'][1]:.0f} Hz",
             f"{rate:.2f} Hz")
    arr = np.array([t for t, _ in info["left"]])
    ok &= check_timing(name, c, arr, stl, 1.0 / c["rate_hz"], False, clock)
    if not (bag / "metadata.yaml").exists():
        print(f"note  {name}: images not recorded (GDN_BAG_IMAGES=1 records them): exposure, noise and depth "
              "not checked")
        return ok
    # Which images to read: pairs in a row at rest, one every 2 s for the exposure, and
    # every 0.5 s from 1.5 to 15 m up for the depth (parked, the camera sees only the
    # flat-coloured pad and runway).
    from make_sim_world import sample_map
    alt, hdr, z0 = world
    t_sample = clock.sample(stl)                            # device stamps: the exposure's Gazebo time
    mid = np.nonzero((arr > t_rest[0] + 2.0) & (arr < t_rest[1] - 1.0))[0]
    rest_idx = mid[len(mid) // 2: len(mid) // 2 + 20] if len(mid) > 40 else np.array([], int)
    expo_idx, low_idx, last_e, last_l = [], [], -1e9, -1e9
    left_mount = payload_sensor(re.sub(r"<!--.*?-->", "", (ROOT / "sim/models/gdn_payload/model.sdf").read_text(),
                                       flags=re.S), st["left"]["gz"])[1]
    mount = (left_mount[:3], left_mount[3:])
    for i, t in enumerate(t_sample):
        if arr[i] <= t_rest[1]:
            continue
        if t - last_e >= 2.0:
            expo_idx.append(i)
            last_e = t
        if t - last_l >= 0.5:
            tr = truth_at(clock.truth, t)
            h = tr[3] - (float(sample_map(alt, hdr, np.array(tr[1]), np.array(tr[2]))) - z0)
            if 1.5 < h < 15.0:
                low_idx.append(i)
                last_l = t
    stl_ns = [stamp_ns(x) for _, x in info["left"]]
    want = {stl_ns[i] for i in np.concatenate([rest_idx, expo_idx, low_idx]).astype(int)}
    imgs = read_images(bag, (st["left"]["topic"], st["right"]["topic"]), want)
    got = [(k, v[0]) for k, v in imgs.items()]
    ok &= ck(len(got) >= 0.99 * 2 * len(want) and all(
        m.encoding == st["encoding"] and (m.width, m.height) == (st["width"], st["height"]) and m.step == m.width
        and m.header.frame_id == st[("left" if k[0] == st["left"]["topic"] else "right")]["frame"] for k, m in got),
        f"{name}: images mono8 640 x 480, with the camera_info's stamps and frames",
        f"{len(got)} of {2 * len(want)} looked at")

    def pair(i):
        k = stl_ns[i]
        a, b = imgs.get((st["left"]["topic"], k)), imgs.get((st["right"]["topic"], k))
        return (a[1], b[1]) if a and b else None
    means = [float(p[0].mean()) for p in map(pair, expo_idx) if p is not None]
    if means:
        ok &= ck(70 <= np.median(means) <= 170, f"{name}: exposed (median image mean 70 to 170 grey)",
                 f"median {np.median(means):.0f}, {min(means):.0f} to {max(means):.0f} over {len(means)} images "
                 "in flight")
    rest_pairs = [pair(i) for i in rest_idx]
    diffs = [(a[0].astype(float) - b[0].astype(float)) for a, b in zip(rest_pairs[:-1], rest_pairs[1:])
             if a is not None and b is not None]
    if diffs:
        sig = float(np.median([np.std(d) for d in diffs]) / math.sqrt(2))
        want_sig = math.sqrt(c["noise_dn"] ** 2 + 1 / 12)
        ok &= ck(abs(sig / want_sig - 1) < 0.25, f"{name}: read noise at rest as set (within 25%)",
                 f"{sig:.2f} grey levels from {len(diffs)} images in a row (model {want_sig:.2f})")
    else:
        ok &= ck(False, f"{name}: images at rest on the pad")
    pooled, best, n_frames, n_tried = np.zeros(len(SCALES)), [], 0, 0
    k0 = int(np.argmin(np.abs(SCALES)))
    for i in low_idx:
        p = pair(i)
        if p is None:
            continue
        err = warp_errors(p[0], p[1], dense_disparity(world, truth_at(clock.truth, t_sample[i]), mount, c))
        if err is None:
            continue
        n_tried += 1
        if err[k0] / err.min() < 1.10:
            continue                # no disparity fits about as well: nothing in this pair to judge by
        pooled += err / err.min()
        best.append(best_scale(err))
        n_frames += 1
    if n_frames >= 5:
        s_best = best_scale(pooled)
        gain = float(pooled[np.argmin(np.abs(SCALES))] / pooled.min())
        ok &= ck(0.95 <= s_best <= 1.05 and gain >= 1.05,
                 f"{name}: stereo geometry as the truth's (the right image is the left shifted by 0.95 to 1.05 "
                 "times the true disparity)",
                 f"best scale {s_best:.3f} over {n_frames} pairs 1.5 to 15 m up ({n_tried} with texture; per pair median "
                 f"{np.median(best):.2f}, {min(best):.2f} to {max(best):.2f}); no disparity fits "
                 f"{gain:.1f} times worse")
    else:
        # A plane takes off along the flat-coloured runway and is past 15 m (under 2 px of
        # disparity) before the ground has texture: nothing to judge the geometry by.
        print(f"note  {name}: stereo geometry not judged on this flight: {n_frames} of {n_tried} pairs 1.5 to "
              "15 m up with texture hold disparity (their best fit 10% better than none; 5 needed)")
    return ok


# --- the Mid-360S lidar (R2.4c) ---

# One CustomPoint as serialized: offset_time, x, y, z, reflectivity, tag, line, padded to 20.
LIVOX_POINT = np.dtype({"names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
                        "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
                        "offsets": [0, 4, 8, 12, 16, 17, 18], "itemsize": 20})


def parse_livox(data, points=True):
    """A serialized livox_ros_driver2/CustomMsg (little-endian CDR) without the message
    package: (stamp s, frame_id, timebase ns, point_num, lidar_id, points or None). As
    rclpy's deserializer, point for point (checked on the real recording), and quick
    enough for a flight's tens of millions of points. The stamp in seconds is a float:
    for exact comparisons use stamp_ns = sec * 1e9 + nanosec, from parse_livox_stamp_ns."""
    b = memoryview(data)[4:]
    sec = int(np.frombuffer(b, "<i4", 1, 0)[0])
    nsec = int(np.frombuffer(b, "<u4", 1, 4)[0])
    n = int(np.frombuffer(b, "<u4", 1, 8)[0])
    frame = bytes(b[12:12 + n - 1]).decode()
    pos = (12 + n + 7) // 8 * 8
    timebase = int(np.frombuffer(b, "<u8", 1, pos)[0])
    point_num = int(np.frombuffer(b, "<u4", 1, pos + 8)[0])
    lidar_id = int(b[pos + 12])
    pts = None
    if points:
        npts = int(np.frombuffer(b, "<u4", 1, pos + 16)[0])
        # The last point isn't padded to 20 bytes.
        pts = np.frombuffer(bytes(b[pos + 20:pos + 20 + 20 * npts - 1]) + b"\0", LIVOX_POINT, npts) if npts else \
            np.zeros(0, LIVOX_POINT)
    return sec + nsec * 1e-9, frame, timebase, point_num, lidar_id, pts


def parse_livox_stamp_ns(data):
    b = memoryview(data)[4:]
    return int(np.frombuffer(b, "<i4", 1, 0)[0]) * 1_000_000_000 + int(np.frombuffer(b, "<u4", 1, 4)[0])


def read_livox(bag, topic="/livox/lidar", points_every=1):
    """Every frame's (receive time, stamp, frame_id, timebase, point_num, lidar_id, stamp in
    integer ns), and
    the points of every points_every-th frame, as {index: points}."""
    import rosbag2_py
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    if topic not in {t.name for t in r.get_all_topics_and_types()}:
        return [], {}
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    frames, pts = [], {}
    while r.has_next():
        _, data, t_ns = r.read_next()
        i = len(frames)
        st, fr, tb, pn, lid, p = parse_livox(data, points=i % points_every == 0)
        frames.append((t_ns * 1e-9, st, fr, tb, pn, lid, parse_livox_stamp_ns(data)))
        if p is not None:
            pts[i] = p
    return frames, pts


def livox_directions(p):
    """Azimuth and elevation (degrees) of the points with a return, and the mask."""
    x, y, z = (p[k].astype(float) for k in "xyz")
    r = np.sqrt(x * x + y * y + z * z)
    hit = r > 0
    az = np.degrees(np.arctan2(y, x))
    el = np.degrees(np.arcsin(np.clip(z / np.where(hit, r, 1.0), -1, 1)))
    return az, el, hit, r


def livox_stats(frames, pts, arrival_minus_stamp=None):
    """What a /livox/lidar stream looks like, for the real recording and for a simulated
    run alike (TEST-037). arrival_minus_stamp: per frame, in ms (the real recording's
    receive times are the host's; a run's go through RunClock first)."""
    st = np.array([f[1] for f in frames])
    tb = np.array([f[3] for f in frames], dtype=np.int64)
    pn = np.array([f[4] for f in frames])
    stamp_ns = np.array([f[6] for f in frames], dtype=np.int64)
    out = dict(frames=len(frames), frame_ids=sorted({f[2] for f in frames}), lidar_ids=sorted({f[5] for f in frames}),
               rate_hz=(len(st) - 1) / (st.max() - st.min()), spacing_ms=[float(x) for x in np.percentile(np.diff(st) * 1e3, [0, 50, 100])],
               points_per_frame=[int(pn.min()), float(np.median(pn)), int(pn.max())],
               stamp_is_timebase=float(np.mean(stamp_ns == tb)))
    if arrival_minus_stamp is not None:
        a = np.asarray(arrival_minus_stamp)
        out["arrival_minus_stamp_ms"] = [float(x) for x in np.percentile(a, [0, 50, 100])]
    P = [pts[k] for k in sorted(pts)]
    allp = np.concatenate(P)
    az, el, hit, r = livox_directions(allp)
    out["line_shares"] = [float(np.mean(allp["line"] == k)) for k in range(4)]
    out["no_return_share"] = float(np.mean(~hit))
    out["no_return_reflectivity_0"] = bool(np.all(allp["reflectivity"][~hit] == 0))
    out["nearest_return_m"] = float(r[hit].min()) if hit.any() else None
    out["offset_time_max_ms"] = float(max(p["offset_time"].max() for p in P) / 1e6)
    d = np.concatenate([np.diff(p["offset_time"].astype(np.int64)) for p in P])
    out["point_spacing_us"] = float(np.median(d[(d > 0) & (d < 10000)]) / 1e3)
    # The pattern's fingerprint: the four lines' elevation spread within a group of four,
    # and the azimuth step from one group to the next (line 0). Only groups with all four
    # returns count, so only frames returning at least half their points are used (as the
    # real unit in a room): where only part of the field hits ground (cruise, a steep
    # slope), the groups left are a biased few (TEST-037: 2.16 against 2.49 deg).
    spread, step, used = [], [], 0
    dense = [p for p in P if np.mean(livox_directions(p)[2]) >= 0.5]
    for p in (dense or P):
        o = np.argsort(p["offset_time"], kind="stable")
        q = p[o]
        n = len(q) // 4 * 4
        if n < 8:
            continue
        a_, e_, h_, _ = livox_directions(q[:n])
        a4, e4, h4, l4 = a_.reshape(-1, 4), e_.reshape(-1, 4), h_.reshape(-1, 4), q["line"][:n].reshape(-1, 4)
        good = h4.all(axis=1) & np.all(l4 == [0, 1, 2, 3], axis=1)
        spread.append(np.diff(e4[good], axis=1).ravel())
        s0 = np.diff(a4[:, 0])
        s0 = (s0 + 180) % 360 - 180
        step.append(s0[h4[1:, 0] & h4[:-1, 0]])
        used += 1
    spread, step = np.concatenate(spread), np.concatenate(step)
    out["pattern_frames"] = used if dense else 0
    out["line_spread_iqr_deg"] = float(np.percentile(spread, 75) - np.percentile(spread, 25))
    out["azimuth_step_deg"] = float(np.median(step))
    return out


def livox_reference(bag, out):
    """Make the reference from a recording of the real unit (static, any room)."""
    frames, pts = read_livox(bag, points_every=10)
    if not frames:
        print(f"no /livox/lidar in {bag}")
        return 2
    s = livox_stats(frames, pts, [(f[0] - f[1]) * 1e3 for f in frames])
    s["source"] = f"{Path(bag).name}: our Mid-360S, livox_ros_driver2 1.2.8, static in a room, every 10th frame's points"
    import json
    Path(out).write_text(json.dumps(s, indent=1) + "\n")
    print(json.dumps(s, indent=1))
    return 0


def check_static_livox(sdf, cfg):
    ok = True
    L = LIVOX
    c = cfg.get(L["name"])
    if c is None or c.get("kind") != "livox" or L["name"] not in cfg["sensors"]:
        return ck(False, f"{L['name']}: a livox sensor in config/sim/sensors.yaml")
    ok &= ck(c["topic"] == L["topic"] and c["frame_id"] == L["frame"], f"{L['name']}: publishes the real topic and frame",
             f"{c['topic']} ({c['frame_id']})")
    sen = payload_sensor(sdf, L["gz"])
    if sen is None:
        return ck(False, f"{L['name']}: a gpu_lidar on {L['gz']} in gdn_payload")
    typ, pose, body = sen
    h = re.search(r"<horizontal><samples>(\d+)</samples>.*?<min_angle>([-0-9.]+)</min_angle><max_angle>([-0-9.]+)<", body, re.S)
    v = re.search(r"<vertical><samples>(\d+)</samples>.*?<min_angle>([-0-9.]+)</min_angle><max_angle>([-0-9.]+)<", body, re.S)
    grid_ok = (h and v and int(h.group(1)) == c["grid_h_samples"] and int(v.group(1)) == c["grid_v_samples"]
               and abs(math.degrees(float(h.group(2))) - c["grid_h_min_deg"]) < 1e-4
               and abs(math.degrees(float(h.group(3))) - c["grid_h_max_deg"]) < 1e-4
               and abs(math.degrees(float(v.group(2))) - c["grid_v_min_deg"]) < 1e-4
               and abs(math.degrees(float(v.group(3))) - c["grid_v_max_deg"]) < 1e-4)
    ok &= ck(typ == "gpu_lidar" and "<noise>" not in body and grid_ok,
             f"{L['name']}: an error-free gpu_lidar with the config's grid of rays",
             f"{typ}, {h.group(1) if h else '?'} x {v.group(1) if v else '?'} rays")
    imu = payload_sensor(sdf, "gdn/imu/mid360s")
    ok &= ck(imu is not None and np.allclose(imu[1], pose), f"{L['name']}: at the Mid-360S's IMU (one unit, one frame)",
             f"pose {pose.tolist()}")
    fetch = (ROOT / "tools/fetch_livox_pattern.sh").read_text()
    pinned = L["pattern_sha256"] in fetch and c["pattern"] == "data/livox/mid360.csv"
    f = ROOT / c["pattern"]
    if f.exists():
        import hashlib
        have = hashlib.sha256(f.read_bytes()).hexdigest()
        ok &= ck(pinned and have == L["pattern_sha256"], f"{L['name']}: Livox's measured Mid-360 pattern, pinned",
                 f"{c['pattern']} sha256 {have[:12]}")
    else:
        ok &= ck(pinned, f"{L['name']}: Livox's measured Mid-360 pattern, pinned (not fetched here)",
                 "tools/fetch_livox_pattern.sh")
    ok &= ck(L["reference"].exists(), f"{L['name']}: the real unit's reference numbers", str(L["reference"].relative_to(ROOT)))
    return ok


def truth_at_many(truth, t):
    """truth_at for an array of times: positions linear, attitudes normalised linear."""
    k = np.clip(np.searchsorted(truth[:, 0], t), 1, len(truth) - 1)
    a, b = truth[k - 1], truth[k]
    f = np.clip((t - a[:, 0]) / np.maximum(b[:, 0] - a[:, 0], 1e-9), 0.0, 1.0)[:, None]
    pos = a[:, 1:4] + f * (b[:, 1:4] - a[:, 1:4])
    qb = np.where((np.sum(a[:, 4:8] * b[:, 4:8], axis=1) >= 0)[:, None], b[:, 4:8], -b[:, 4:8])
    q = a[:, 4:8] + f * (qb - a[:, 4:8])
    return pos, q / np.linalg.norm(q, axis=1, keepdims=True)


def quat_rotate(q, v):
    """Rotate vectors v (N x 3) by unit quaternions q (N x 4, w x y z)."""
    w, u = q[:, :1], q[:, 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def livox_ground_residual(p, t_gz, clock, mount, world, shift=0.0):
    """Height of each returned point above the world's terrain, the point placed in the
    world from the true pose at its own time (Gazebo t_gz, plus shift) and the lidar's
    mount: (residual m, range m)."""
    from make_sim_world import sample_map
    alt, hdr, z0 = world
    _, _, hit, r = livox_directions(p)
    v = np.stack([p["x"], p["y"], p["z"]], axis=1).astype(float)[hit]
    pos, q = truth_at_many(clock.truth, t_gz[hit] + shift)
    Rm = rpy_matrix(*mount[1])
    w = pos + quat_rotate(q, mount[0] + v @ Rm.T)
    return w[:, 2] - (sample_map(alt, hdr, w[:, 0], w[:, 1]) - z0), r[hit]


def check_livox(c, bag, clock, t_rest, world):
    import json
    ok = True
    L = LIVOX
    name = L["name"]
    frames, pts = read_livox(bag, L["topic"], points_every=5)
    if not ck(len(frames) > 50, f"{name}: on {L['topic']}", f"{len(frames)} frames"):
        return False
    ref = json.loads(L["reference"].read_text())
    arr = np.array([f[0] for f in frames])
    st = np.array([f[1] for f in frames])
    lat = (clock.sim_of_wall(arr) - clock.sample(st)) * 1e3            # Gazebo time
    s = livox_stats(frames, pts, lat)
    ok &= ck(s["frame_ids"] == [L["frame"]] and s["lidar_ids"] == ref["lidar_ids"],
             f"{name}: frame and lidar_id as the real unit's", f"{s['frame_ids']}, {s['lidar_ids']}")
    ok &= ck(abs(s["rate_hz"] - 10.0) < 0.1 and s["spacing_ms"][0] > 95 and s["spacing_ms"][2] < 105,
             f"{name}: 10 Hz frames (real {ref['rate_hz']:.2f} Hz, {ref['spacing_ms'][0]:.1f} to {ref['spacing_ms'][2]:.1f} ms)",
             f"{s['rate_hz']:.3f} Hz, {s['spacing_ms'][0]:.1f} to {s['spacing_ms'][2]:.1f} ms")
    lo, hi = ref["points_per_frame"][0], ref["points_per_frame"][2]
    ok &= ck(lo <= s["points_per_frame"][0] and s["points_per_frame"][2] <= hi,
             f"{name}: points a frame within the real unit's {lo} to {hi}",
             f"{s['points_per_frame'][0]} to {s['points_per_frame'][2]}, median {s['points_per_frame'][1]:.0f}")
    ok &= ck(s["stamp_is_timebase"] == 1.0 and abs(s["point_spacing_us"] - ref["point_spacing_us"]) < 0.01
             and abs(s["offset_time_max_ms"] - ref["offset_time_max_ms"]) < 2.0,
             f"{name}: stamp = timebase (the first point), points {ref['point_spacing_us']:.3f} us apart, "
             f"offset_time to about {ref['offset_time_max_ms']:.0f} ms, as the real unit",
             f"{100 * s['stamp_is_timebase']:.0f}%, {s['point_spacing_us']:.3f} us, {s['offset_time_max_ms']:.1f} ms")
    ok &= ck(max(abs(a - b) for a, b in zip(s["line_shares"], ref["line_shares"])) < 0.005,
             f"{name}: four lines, a quarter each", " ".join(f"{100 * x:.2f}%" for x in s["line_shares"]))
    ok &= ck(s["no_return_reflectivity_0"] and (s["nearest_return_m"] or 0) >= c["min_range"] - 0.05,
             f"{name}: no return as the real driver publishes it (0, 0, 0, reflectivity 0); none nearer than "
             f"{c['min_range']} m (real {ref['nearest_return_m']:.3f})",
             f"{100 * s['no_return_share']:.1f}% no return (real, in a room: {100 * ref['no_return_share']:.1f}%), "
             f"nearest {s['nearest_return_m']:.3f} m")
    ok &= ck(s["pattern_frames"] >= 10 and abs(s["azimuth_step_deg"] - ref["azimuth_step_deg"]) < 0.1
             and abs(s["line_spread_iqr_deg"] - ref["line_spread_iqr_deg"]) < 0.3,
             f"{name}: the real unit's scan pattern (azimuth step a group {ref['azimuth_step_deg']:.2f} deg, lines "
             f"spread {ref['line_spread_iqr_deg']:.2f} deg; frames returning half their points or more)",
             f"{s['azimuth_step_deg']:.2f} deg, {s['line_spread_iqr_deg']:.2f} deg over {s['pattern_frames']} frames")
    want = 1e3 * c["frame_hz"] ** -1 + c["latency_ms"]
    # Within 4 ms, not the 2 ms of small messages: a 400 kB frame's way into the bag took
    # 1.8 to 7.7 ms over the TEST-037 flights, with the host's power state (performance,
    # balanced, battery); sim_sensors itself sends every frame 5 ms after its end.
    ok &= ck(abs(s["arrival_minus_stamp_ms"][1] - ref["arrival_minus_stamp_ms"][1]) < 4.0,
             f"{name}: arrival minus stamp in the bag as the real unit's (median "
             f"{ref['arrival_minus_stamp_ms'][1]:.1f} ms: the frame, the driver, the way to the recorder), within "
             "4 ms, on Gazebo's clock",
             f"median {s['arrival_minus_stamp_ms'][1]:.1f} ms (the frame and the driver: {want:.0f}), "
             f"{s['arrival_minus_stamp_ms'][0]:.1f} to {s['arrival_minus_stamp_ms'][2]:.1f}")
    lost = lost_share(st, 0.1)
    ok &= ck(abs(lost - c["drop_prob"]) <= 0.01, f"{name}: frames lost {100 * c['drop_prob']:.1f}% (real: none)",
             f"{100 * lost:.2f}%")
    # Every return on the world's terrain, from the true pose at the point's own time.
    sdf = re.sub(r"<!--.*?-->", "", (ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    mp = payload_sensor(sdf, L["gz"])[1]
    mount = (mp[:3], mp[3:])
    # Returns within 1 m are the airframe (the x500's arms reach 0.6 m from the sensor).
    res, mov = {-0.05: [], 0.0: [], 0.05: []}, []
    for i, p in pts.items():
        if arr[i] <= t_rest[1]:
            continue
        t_gz = clock.sample(st[i]) + p["offset_time"].astype(float) * 1e-9
        moving = truth_at(clock.truth, t_gz[len(t_gz) // 2])[8] > 1.0
        dz, r = livox_ground_residual(p, t_gz, clock, mount, world)
        res[0.0].append(dz[r > 1.0])
        if moving and (r > 1.0).sum() >= 100:
            mov.append(dz[r > 1.0])
            for sh in (-0.05, 0.05):
                d2, r2 = livox_ground_residual(p, t_gz, clock, mount, world, sh)
                res[sh].append(d2[r2 > 1.0])
    dz0 = np.concatenate(res[0.0]) if res[0.0] else np.zeros(0)
    if len(dz0) < 1000:
        return ck(False, f"{name}: returns from the ground in flight", f"{len(dz0)}")
    med, p95 = float(np.median(np.abs(dz0))), float(np.percentile(np.abs(dz0), 95))
    ok &= ck(med < 0.10 and p95 < 0.5, f"{name}: every return on the terrain (|height above it| median under 0.1 m, "
             "95% under 0.5 m)", f"median {med:.3f} m, 95% {p95:.3f} m over {len(dz0)} returns in flight")
    if len(mov) >= 5:
        f0 = float(np.median(np.abs(np.concatenate(mov))))
        fm, fp = (float(np.median(np.abs(np.concatenate(res[k])))) for k in (-0.05, 0.05))
        ok &= ck(f0 < 0.8 * min(fm, fp), f"{name}: each point fits the pose at its own offset_time best (moving, "
                 "against 50 ms before or after, by 20% or more)",
                 f"median |height| {f0:.3f} m at its time, {fm:.3f} m 50 ms before, {fp:.3f} m after, "
                 f"over {len(mov)} moving frames")
    else:
        ok &= ck(False, f"{name}: frames with ground returns while moving", f"{len(mov)}")
    return ok


def payload_mount(sensor_topic):
    """Position of a payload sensor in the payload frame, which is what the truth
    (gdn/truth_odom, published by the payload) gives the pose of."""
    sdf = re.sub(r"<!--.*?-->", "", (ROOT / "sim/models/gdn_payload/model.sdf").read_text(), flags=re.S)
    s = re.search(rf"<sensor[^>]*>\s*<pose>([^<]+)</pose>(?:(?!</sensor>).)*<topic>{sensor_topic}</topic>", sdf, re.S)
    return np.array([float(x) for x in s.group(1).split()[:3]])


def check_run(d):
    d = Path(d)
    info = dict(l.split("=", 1) for l in (d / "run_info.txt").read_text().splitlines() if "=" in l)
    winfo = dict(l.split("=", 1) for l in (Path(info["world_dir"]) / "world_info.txt").read_text().splitlines()
                 if "=" in l)
    print(f"run: {info['vehicle']} over {info['world']}")
    cfg = load_config()
    types, msgs, truth = read_bag(d / "sitl_bag", [r["topic"] for r in REAL.values()]
                                  + [STEREO[s]["info"] for s in ("left", "right")] + [RAW_REF])
    if len(truth) < 100:
        print("check_sim_sensors: no truth in the bag")
        return 2
    t_rest = rest_window(truth)
    print(f"at rest on the pad for {t_rest[1] - t_rest[0]:.0f} s")
    offset = RunClock.offset_from_log(d / "sensors.log") if (d / "sensors.log").exists() else None
    if not ck(offset is not None, "sim_sensors logged its clock (stamps are Gazebo time plus an offset)",
              f"offset {offset:.6f} s" if offset is not None else "no 'clock:' line in sensors.log"):
        return 1
    ref = [(t, stamp(m)) for t, m in msgs[RAW_REF]]
    clock = RunClock(truth, offset, ref)
    print(f"clock reference: {'the raw H30WP, %d samples' % len(ref) if len(ref) > 100 else 'the truth (20 Hz)'}")
    rtf = np.ptp(truth[:, 9]) / np.ptp(truth[:, 0])
    print(f"real-time factor {rtf:.3f} (Gazebo seconds per wall second, from the truth)")
    from gdn_paths import load_map
    alt, hdr = load_map(winfo.get("world_map", winfo["map"]))
    world = (alt.astype(np.float64), hdr, float(winfo["z0_amsl"]))
    ok = True
    for name, real in REAL.items():
        c = cfg[name]
        m = msgs[real["topic"]]
        if not ck(len(m) > 100 and types.get(real["topic"]) == real["type"],
                  f"{name}: on {real['topic']} as {real['type']}", f"{len(m)} messages, {types.get(real['topic'])}"):
            ok = False
            continue
        if c["kind"] == "imu":
            ok &= check_imu(name, real, c, m, t_rest, clock)
        else:
            ok &= check_tfmini(real, c, m, truth, t_rest, payload_mount("gdn/tfmini"), world, clock)
    ok &= check_stereo(cfg[STEREO["name"]], types, msgs, t_rest, world, d / "image_bag", clock)
    ok &= check_livox(cfg[LIVOX["name"]], d / "sitl_bag", clock, t_rest, world)
    return 0 if ok else 1


# --- the statistics, on made-up data ---

def selfcheck():
    ok = True
    rng = np.random.default_rng(2)
    t = np.arange(0, 20, 0.005) + rng.normal(0, 0.00053, 4000)
    ok &= ck(abs(spacing_spread(t) - 0.15) < 0.02, "selfcheck: 0.53 ms stamp jitter is a 15% spacing spread",
             f"{100 * spacing_spread(t):.1f}%")
    ticks = np.cumsum(rng.choice([1, 2], size=5000, p=[0.35, 0.65])) / 394.5
    ok &= ck(abs(1 / np.mean(np.diff(ticks)) - 239) < 3 and abs(spacing_spread(ticks) - 0.29) < 0.03,
             "selfcheck: the OAK-D skip pattern is 239 Hz and a 29% spread",
             f"{1 / np.mean(np.diff(ticks)):.0f} Hz, {100 * spacing_spread(ticks):.0f}%")
    dt = np.diff(ticks)
    noise = 1e-3 / np.sqrt(dt) * rng.standard_normal(len(dt))
    ok &= ck(abs(np.std(noise) / expected_sigma(1e-3, ticks) - 1) < 0.03, "selfcheck: sigma of uneven white noise")
    tt = np.arange(0, 100, 0.01)
    keep = rng.random(len(tt)) > 0.02
    ok &= ck(abs(lost_share(tt[keep], 0.01) - 0.02) < 0.005, "selfcheck: 2% lost is found",
             f"{100 * lost_share(tt[keep], 0.01):.2f}%")
    ok &= ck(lost_share(ticks, 1 / 394.5, skip_ticks=True) == 0.0, "selfcheck: skipped ticks aren't losses")
    cfg = {"near_limit_m": 6.0, "noise_near_m": 0.03, "noise_far_frac": 0.005}
    ok &= ck(np.allclose(tfmini_sigma([1.0, 10.0], cfg), [0.03, 0.05]), "selfcheck: TFmini-S sigma by range")
    # R2.4b: the truth's disparity on flat ground, and the stereo warp test on a made-up pair.
    hdr = {"dlat": -1e-4, "dlon": 1e-4, "m_per_deg_lat": 1e4, "m_per_deg_lon": 1e4}  # 1 m cells
    world = (np.full((200, 200), 500.0), hdr, 490.0)                                # ground at z = 10
    c = {"width": 640, "height": 480, "hfov_deg": 73.0, "baseline_m": 0.075}
    tr = np.array([0, 0, 0, 14.0, 1, 0, 0, 0, 0])           # 4 m up, level, facing +x (east)
    mount = (np.zeros(3), np.array([0.0, math.radians(30), 0.0]))
    fx, cx, cy = pinhole(640, 480, 73.0)
    grid = np.array([[cx, cy], [cx, cy + 100.0]])
    d = true_disparity(world, tr, mount, c, grid)
    # Depth along the optical axis to flat ground 4 m below, 30 deg (and 30 + atan(100 / fx)) down.
    want = [fx * 0.075 * math.sin(math.radians(30)) / 4.0,
            fx * 0.075 / (4.0 / math.sin(math.radians(30) + math.atan(100 / fx)) * math.cos(math.atan(100 / fx)))]
    ok &= ck(np.allclose(d, want, rtol=1e-4), "selfcheck: the truth's disparity on flat ground",
             f"{d[0]:.3f}, {d[1]:.3f} px (exact {want[0]:.3f}, {want[1]:.3f})")
    try:
        import cv2
        tex = cv2.GaussianBlur((rng.random((480, 700)) * 255).astype(np.uint8), (0, 0), 3.0)
        tex = cv2.normalize(tex, None, 0, 255, cv2.NORM_MINMAX)
        left, right = tex[:, 40:680].copy(), tex[:, 52:692].copy()   # a point at x in the left is at x - 12 in the right
        disp = np.full((480, 640), 12.0, np.float32)
        s_ok, s_swap = best_scale(warp_errors(left, right, disp)), best_scale(warp_errors(right, left, disp))
        s_wide = best_scale(warp_errors(left, right, disp / 1.1))            # a baseline 10% too short
        ok &= ck(abs(s_ok - 1.0) < 0.02 and abs(s_swap + 1.0) < 0.05 and abs(s_wide - 1.1) < 0.03,
                 "selfcheck: the stereo warp test finds scale 1, -1 for a swapped pair, 1.1 for a 10% short baseline",
                 f"{s_ok:.3f}, {s_swap:.3f}, {s_wide:.3f}")
    except ImportError:
        ok &= ck(False, "selfcheck: OpenCV (python3-opencv) for the stereo check")
    # R2.4c: the CustomMsg reader on a hand-built message (CDR: 4-byte encapsulation, then
    # each field aligned to its size, the last point unpadded).
    import struct
    pts = [(0, 1.0, 2.0, 3.0, 40, 0, 0), (4947, 0.0, 0.0, 0.0, 0, 0, 1), (9894, -1.5, 0.25, 7.0, 40, 0, 2)]
    frame = b"livox_frame\0"
    body = struct.pack("<iII", 1795000000, 123456789, len(frame)) + frame
    body += b"\0" * ((-len(body)) % 8)
    body += struct.pack("<QIB3xI", 1795000000123456789, len(pts), 192, len(pts))
    body += b"".join(struct.pack("<I3fBBB", *q) + (b"\0" if k < len(pts) - 1 else b"") for k, q in enumerate(pts))
    st, fr, tb, pn, lid, pp = parse_livox(b"\x00\x01\x00\x00" + body)
    ok &= ck(fr == "livox_frame" and tb == 1795000000123456789 and pn == 3 and lid == 192
             and parse_livox_stamp_ns(b"\x00\x01\x00\x00" + body) == 1795000000123456789
             and pp["offset_time"].tolist() == [0, 4947, 9894] and pp["line"].tolist() == [0, 1, 2]
             and np.allclose(pp["x"], [1.0, 0.0, -1.5]) and pp["reflectivity"].tolist() == [40, 0, 40],
             "selfcheck: the CustomMsg reader (fields, alignment, the unpadded last point)")
    # A point on flat ground, from a known pose and the lidar's upside-down mount: 10 m below
    # a level sensor, the point straight down in its own (flipped) frame is straight up.
    hdr1 = {"dlat": -1e-4, "dlon": 1e-4, "m_per_deg_lat": 1e4, "m_per_deg_lon": 1e4}
    world1 = (np.full((400, 400), 500.0), hdr1, 490.0)                    # ground at z = 10
    class Clk:
        truth = np.array([[0.0, 5, 3, 20.05, 1, 0, 0, 0, 0, 0], [1.0, 5, 3, 20.05, 1, 0, 0, 0, 0, 0]])
    p1 = np.zeros(2, LIVOX_POINT)
    p1["z"] = [10.0, 9.0]                                                 # up in the sensor frame
    dz, rr = livox_ground_residual(p1, np.array([0.5, 0.5]), Clk, (np.array([0, 0, -0.05]), np.array([math.pi, 0, 0])),
                                   world1)
    ok &= ck(np.allclose(dz, [0.0, 1.0], atol=1e-6) and np.allclose(rr, [10.0, 9.0]),
             "selfcheck: a lidar return placed in the world through the upside-down mount",
             f"heights above ground {np.round(dz, 4).tolist()} m (want 0 and 1)")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", help="a tools/sim_run.sh log directory")
    ap.add_argument("--selfcheck", action="store_true")
    ap.add_argument("--livox-reference", nargs=2, metavar=("BAG", "OUT_JSON"),
                    help="R2.4c: the reference numbers from a recording of the real Mid-360S")
    a = ap.parse_args()
    if a.livox_reference:
        return livox_reference(*a.livox_reference)
    if a.selfcheck:
        return 0 if selfcheck() else 1
    if a.run:
        return check_run(a.run)
    return 0 if check_static() else 1


if __name__ == "__main__":
    sys.exit(main())
