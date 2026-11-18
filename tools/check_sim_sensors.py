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
  - with the configured timing (config/sim/sensors.yaml), on Gazebo's clock (R2.4b: the
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
    ok &= check_timing(name, c, arr, st, 1.0 / (c["tick_hz"] if c["tick_hz"] > 0 else c["rate_hz"]), c["tick_hz"] > 0,
                       clock)
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


def check_timing(name, c, arr, st, nominal_dt, skip_ticks, clock):
    """arrival minus stamp on Gazebo's clock: the Gazebo time when the bag received a
    message, less its stamp's Gazebo time."""
    ok = True
    lat = (clock.sim_of_wall(arr) - clock.sample(st)) * 1000.0
    med = float(np.median(lat))
    want = c["latency_ms"] if c["stamp"] == "sensor" else 0.0
    p1 = float(np.percentile(lat, 1))
    ok &= ck(abs(med - want) <= 2.0 and p1 > -2.5 - 3 * c["stamp_jitter_ms"],
             f"{name}: arrival minus stamp {want:.1f} ms on Gazebo's clock ({c['stamp']} stamps, latency "
             f"{c['latency_ms']} ms)", f"median {med:.2f} ms, 1st percentile {p1:.2f}, max {lat.max():.2f}")
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
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", help="a tools/sim_run.sh log directory")
    ap.add_argument("--selfcheck", action="store_true")
    a = ap.parse_args()
    if a.selfcheck:
        return 0 if selfcheck() else 1
    if a.run:
        return check_run(a.run)
    return 0 if check_static() else 1


if __name__ == "__main__":
    sys.exit(main())
