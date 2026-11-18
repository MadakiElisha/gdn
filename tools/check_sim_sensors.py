#!/usr/bin/env python3
"""Check the simulated sensors against the real ones (roadmap R2.4a, TEST-035).

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
  - with the configured timing (config/sim/sensors.yaml): arrival minus stamp, its median
    within 2 ms (the recorder's own transport delay) of the latency (sensor stamps) or near zero (arrival stamps, which carry
    the latency in the stamp itself), never negative beyond 0.5 ms; and the share of
    messages lost within 1 percentage point of drop_prob;
  - the TFmini-S: the real Range fields (infrared, 0.1 to 12 m, 2 degree field of view);
    at rest its mean within 5 cm of the true height of the sensor above the ground and
    its sigma within 30% of the model (or "too close", -inf, if it's parked nearer than
    0.1 m to the ground); in flight, +inf above 12 m, and within 12 m its
    error RMS (against the true range along the beam) under twice the model's sigma.

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
            truth.append((t_ns * 1e-9, p.x, p.y, p.z, q.w, q.x, q.y, q.z, math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)))
    return types, out, np.array(truth)


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def rest_window(truth):
    """Times (bag clock) while the vehicle stands still on the pad before it first moves:
    from the first truth sample to 1 s before it first moves faster than 0.05 m/s."""
    moving = np.nonzero(truth[:, 8] > 0.05)[0]
    t_end = truth[moving[0], 0] - 1.0 if len(moving) else truth[-1, 0]
    return truth[0, 0], t_end


def check_imu(name, real, c, msgs, t_rest):
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
    ok &= check_timing(name, c, arr, st, 1.0 / (c["tick_hz"] if c["tick_hz"] > 0 else c["rate_hz"]), c["tick_hz"] > 0)
    return ok


def check_timing(name, c, arr, st, nominal_dt, skip_ticks):
    ok = True
    lat = (arr - st) * 1000.0
    med = float(np.median(lat))
    want = c["latency_ms"] if c["stamp"] == "sensor" else 0.0
    ok &= ck(abs(med - want) <= 2.0 and lat.min() > -0.5 - 4 * c["stamp_jitter_ms"],
             f"{name}: arrival minus stamp {want:.1f} ms ({c['stamp']} stamps, latency {c['latency_ms']} ms)",
             f"median {med:.2f} ms, min {lat.min():.2f}, max {lat.max():.2f}")
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


def check_tfmini(real, c, msgs, truth, t_rest, mount, world):
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
    # The reading's sample time: its arrival less the configured latency (arrival
    # stamps carry that latency); the truth arrives about 1 ms after its own sample.
    t_sample = arr - c["latency_ms"] * 1e-3 + 0.001
    for i in range(len(r)):
        tr = truth_at(truth, t_sample[i])
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
    if at_rest.sum() > 100 and np.median(pred[at_rest]) < c["min_range"]:
        # Parked closer than its minimum range (a VTOL on its belly): "too close", -inf.
        share = float(np.mean(np.isneginf(r[at_rest])))
        ok &= ck(share > 0.9, "tfmini: at rest closer than 0.1 m, it reads too close (-inf), as the real one",
                 f"{100 * share:.0f}% -inf, true range {np.median(pred[at_rest]):.3f} m")
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
    ok &= check_timing("tfmini", c, arr, st, 1.0 / c["rate_hz"], False)
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
    types, msgs, truth = read_bag(d / "sitl_bag", [r["topic"] for r in REAL.values()])
    if len(truth) < 100:
        print("check_sim_sensors: no truth in the bag")
        return 2
    t_rest = rest_window(truth)
    print(f"at rest on the pad for {t_rest[1] - t_rest[0]:.0f} s")
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
            ok &= check_imu(name, real, c, m, t_rest)
        else:
            ok &= check_tfmini(real, c, m, truth, t_rest, payload_mount("gdn/tfmini"), world)
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
