#!/usr/bin/env python3
"""Vehicle profiles check (roadmap R2.5a, TEST-038). Static: no simulation needed.

    python3 tools/check_profiles.py              # every simulated vehicle against its profile
    python3 tools/check_profiles.py --selfcheck  # the reader and the pose arithmetic, on made-up profiles

For every vehicle tools/sim_vehicles.py can fly:
  - config/vehicles/<vehicle>.yaml exists and reads cleanly (tools/vehicle_profile.py:
    every key known, required keys present, numbers parse, each GDN input from one
    sensor, filter overrides are keys of the filter file);
  - its kind, PX4 airframe and Gazebo model are sim_vehicles.py's;
  - extrinsics: every payload sensor (sim/models/gdn_payload) is in the profile, and its
    mount there is where Gazebo has it, through that airframe's payload mount
    (sim/models/<model>/model.sdf), within 1 mm and 0.05 deg (a camera's optical frame
    included). The autopilot's own sensors define base_link: at its origin, unrotated
    (and where PX4's model has them, when PX4 is installed);
  - every simulated driver topic in config/sim/sensors.yaml is in the profile, with the
    same frame_id and, for an IMU, the same noise model file.
"""
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import vehicle_profile as vp  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TOL_M, TOL_DEG = 1e-3, 0.05


def ck(ok, what, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {what}{': ' + detail if detail else ''}")
    return bool(ok)


def strip_comments(sdf):
    return re.sub(r"<!--.*?-->", "", sdf, flags=re.S)


def payload_sensors(sdf):
    """{name: (type, pose 4x4)} of the payload's sensors (pose in the payload link)."""
    out = {}
    for m in re.finditer(r'<sensor name="([^"]+)" type="([a-z_]+)">\s*(?:<!--.*?-->\s*)*<pose>([^<]+)</pose>', sdf,
                         re.S):
        v = [float(x) for x in m.group(3).split()]
        out[m.group(1)] = (m.group(2), vp.pose(v[:3], v[3:]))
    return out


def payload_mount(model_sdf):
    """The pose of the payload in the airframe's base_link, from its <include>."""
    m = re.search(r"<uri>model://gdn_payload</uri>\s*<pose relative_to=\"base_link\">([^<]+)</pose>", model_sdf)
    if not m:
        return None
    v = [float(x) for x in m.group(1).split()]
    return vp.pose(v[:3], v[3:])


def px4_dir():
    for p in (os.environ.get("PX4_DIR"), str(Path.home() / "PX4-Autopilot-1.16"), "/opt/PX4-Autopilot"):
        if p and (Path(p) / "Tools/simulation/gz/models").is_dir():
            return Path(p)
    return None


def px4_sensor_pose(px4, model_sdf, name):
    """The pose of PX4's own model's sensor `name` in base_link (None if not found),
    following PX4's merged includes (x500 holds no sensors; its x500_base does)."""
    base = re.search(r"<uri>model://([\w]+)</uri>", re.sub(r"<uri>model://gdn_payload</uri>", "", model_sdf))
    return _px4_find(px4, base.group(1), name, np.eye(4), 0) if base else None


def _px4_find(px4, model, name, T, depth):
    f = px4 / "Tools/simulation/gz/models" / model / "model.sdf"
    if depth > 4 or not f.exists():
        return None
    s = strip_comments(f.read_text())
    m = re.search(rf'<sensor name="{name}" type="[a-z_]+">((?:(?!</sensor>).)*)</sensor>', s, re.S)
    if m:
        pm = re.match(r"\s*<pose>([^<]+)</pose>", m.group(1))
        v = [float(x) for x in pm.group(1).split()] if pm else [0.0] * 6
        return T @ vp.pose(v[:3], v[3:])
    for inc in re.finditer(r"<include[^>]*>(.*?)</include>", s, re.S):
        u = re.search(r"<uri>model://([\w]+)</uri>", inc.group(1))
        if not u:
            continue
        pm = re.search(r"<pose[^>]*>([^<]+)</pose>", inc.group(1))
        v = [float(x) for x in pm.group(1).split()] if pm else [0.0] * 6
        r = _px4_find(px4, u.group(1), name, T @ vp.pose(v[:3], v[3:]), depth + 1)
        if r is not None:
            return r
    return None


def sim_driver_topics(cfg):
    """{topic: (frame_id, noise model or None, block)} for every simulated driver topic."""
    out = {}
    for name in cfg["sensors"]:
        c = cfg[name]
        if c.get("kind") == "stereo":
            for side in ("left", "right"):
                out[c[f"{side}_topic"]] = (c[f"{side}_frame_id"], None, name)
        else:
            out[c["topic"]] = (c["frame_id"], c.get("model"), name)
    return out


def mount_error(T_prof, T_sim):
    return float(np.linalg.norm(T_prof[:3, 3] - T_sim[:3, 3])), vp.angle_between(T_prof[:3, :3], T_sim[:3, :3])


def profile_pose(s):
    return vp.pose(s["xyz"], np.radians(s["rpy_deg"]))


def check_vehicle(name, v, payload, cfg_topics, px4):
    ok = True
    f = ROOT / "config/vehicles" / f"{name}.yaml"
    if not ck(f.exists(), f"{name}: has a profile", str(f.relative_to(ROOT))):
        return False
    try:
        p = vp.load(f)
    except vp.ProfileError as e:
        return ck(False, f"{name}: profile reads cleanly", str(e))
    t = p["top"]
    ok &= ck(True, f"{name}: profile reads cleanly", f"fingerprint {p['fingerprint']}")
    ok &= ck(t["name"] == name, f"{name}: profile name is the file's", t["name"])
    ok &= ck((t["kind"], t["px4_airframe"], t["sim_model"]) == (v.kind, v.airframe, v.model),
             f"{name}: kind, PX4 airframe and Gazebo model as sim_vehicles.py",
             f"{t['kind']}, {t['px4_airframe']}, {t['sim_model']}")
    ok &= ck(set(p["aiding"]) == set(vp.AIDING), f"{name}: aiding set for {', '.join(vp.AIDING)}",
             ", ".join(f"{a} {'on' if p['aiding'][a] else 'off'}" for a in vp.AIDING))
    model_sdf = strip_comments((ROOT / "sim/models" / v.model / "model.sdf").read_text())
    T_pl = payload_mount(model_sdf)
    if not ck(T_pl is not None, f"{name}: {v.model} mounts the payload on base_link"):
        return False
    sims = {}
    for sid, s in p["sensors"].items():
        if "sim" in s:
            sims.setdefault(s["sim"], []).append(sid)
    # Every payload sensor is in the profile, once, where Gazebo has it.
    for sname, (stype, T_s) in payload.items():
        owners = sims.get(sname, [])
        if not ck(len(owners) == 1, f"{name}: payload sensor {sname} is in the profile once",
                  ", ".join(owners) or "missing"):
            ok = False
            continue
        s = p["sensors"][owners[0]]
        T_sim = T_pl @ T_s @ (vp.OPTICAL if s["kind"] == "camera" else np.eye(4))
        dm, da = mount_error(profile_pose(s), T_sim)
        rpy = np.degrees(vp.rpy_of(T_sim[:3, :3]))
        ok &= ck(dm <= TOL_M and da <= TOL_DEG, f"{name}: {owners[0]} mount matches Gazebo ({sname})",
                 f"{dm * 1000:.2f} mm, {da:.3f} deg off; Gazebo xyz {' '.join(f'{x:+.4f}' for x in T_sim[:3, 3])} m, "
                 f"rpy {' '.join(f'{x:+.2f}' for x in rpy)} deg")
    # Profile sensors that claim a payload sensor that doesn't exist.
    for simname, sids in sims.items():
        if simname.startswith("autopilot:"):
            for sid in sids:
                s = p["sensors"][sid]
                dm, da = mount_error(profile_pose(s), np.eye(4))
                ok &= ck(dm <= TOL_M and da <= TOL_DEG, f"{name}: {sid} (the autopilot's) defines base_link",
                         f"{dm * 1000:.2f} mm, {da:.3f} deg from its origin")
                if px4 is not None:
                    Tp = px4_sensor_pose(px4, model_sdf, simname.split(":", 1)[1])
                    if ck(Tp is not None, f"{name}: PX4's model has {simname.split(':', 1)[1]}"):
                        dm, da = mount_error(profile_pose(s), Tp)
                        ok &= ck(dm <= TOL_M and da <= TOL_DEG, f"{name}: {sid} where PX4's model has it",
                                 f"{dm * 1000:.2f} mm, {da:.3f} deg off")
                    else:
                        ok = False
        elif simname not in payload:
            ok &= ck(False, f"{name}: {', '.join(sids)} names a payload sensor", f"no {simname} in gdn_payload")
    # Every simulated driver topic is in the profile, with its frame and noise model.
    by_topic = {s["topic"]: (sid, s) for sid, s in p["sensors"].items()}
    for topic, (frame, model, block) in cfg_topics.items():
        if topic not in by_topic:
            ok &= ck(False, f"{name}: {topic} (config/sim/sensors.yaml {block}) is in the profile", "missing")
            continue
        sid, s = by_topic[topic]
        good = s["frame_id"] == frame and (model is None or s.get("noise") == model)
        ok &= ck(good, f"{name}: {sid} on {topic} as simulated: frame {frame}" + (f", noise {model}" if model else ""),
                 f"profile: {s['frame_id']}" + (f", {s.get('noise')}" if model else ""))
    return ok


def run():
    import check_sim_sensors as cs
    import sim_vehicles
    payload = payload_sensors(strip_comments((ROOT / "sim/models/gdn_payload/model.sdf").read_text()))
    cfg_topics = sim_driver_topics(cs.load_config())
    px4 = px4_dir()
    ok = ck(len(payload) > 0, "the payload's sensors read from sim/models/gdn_payload", ", ".join(payload))
    for name, v in sim_vehicles.VEHICLES.items():
        ok &= check_vehicle(name, v, payload, cfg_topics, px4)
    extra = sorted(set(p.stem for p in (ROOT / "config/vehicles").glob("*.yaml")) - set(sim_vehicles.VEHICLES)) \
        if (ROOT / "config/vehicles").is_dir() else []
    for e in extra:            # a real airframe's profile is fine; it just has to read
        try:
            vp.load(ROOT / "config/vehicles" / f"{e}.yaml")
            ck(True, f"{e}: profile (not simulated) reads cleanly")
        except vp.ProfileError as err:
            ok &= ck(False, f"{e}: profile (not simulated) reads cleanly", str(err))
    if px4 is None:
        print("note  PX4 not installed here: the autopilot sensors aren't compared with PX4's models")
    print("PROFILES CHECK PASSED" if ok else "PROFILES CHECK FAILED")
    return ok


# --- self-check: the reader and the pose arithmetic on made-up profiles ---
GOOD = """schema: 1
name: test
kind: multirotor
px4_airframe: 4001_gz_x500
sim_model: x500_gdn
filter_config: config/filter/default.yaml
filter.r_agl: 40
aiding.agl: true
aiding.baro: true
aiding.mag: true
aiding.zupt: false
sensor.fc_imu.kind: imu
sensor.fc_imu.topic: /fmu/out/sensor_combined
sensor.fc_imu.frame_id: base_link
sensor.fc_imu.mount_xyz_m: 0 0 0
sensor.fc_imu.mount_rpy_deg: 0 0 0
sensor.fc_imu.mount_source: placeholder
sensor.fc_imu.gdn_input: imu
sensor.radar.kind: radar_altimeter
sensor.radar.topic: /gdn/agl_meas
sensor.radar.frame_id: agl_link
sensor.radar.mount_xyz_m: 0 0 -0.06
sensor.radar.mount_rpy_deg: 0 90 0
sensor.radar.mount_source: placeholder
sensor.radar.gdn_input: agl
sensor.baro.kind: baro
sensor.baro.topic: /gdn/baro_meas
sensor.baro.frame_id: base_link
sensor.baro.mount_xyz_m: 0 0 0
sensor.baro.mount_rpy_deg: 0 0 0
sensor.baro.mount_source: placeholder
sensor.baro.gdn_input: baro
sensor.mag.kind: mag
sensor.mag.topic: /gdn/mag_meas
sensor.mag.frame_id: base_link
sensor.mag.mount_xyz_m: 0 0 0
sensor.mag.mount_rpy_deg: 0 0 0
sensor.mag.mount_source: placeholder
sensor.mag.gdn_input: mag
"""


def selfcheck():
    ok = True

    def bad(text, expect, what):
        try:
            vp.parse(text, "t")
            return ck(False, f"selfcheck: {what} is refused", "accepted")
        except vp.ProfileError as e:
            return ck(expect in str(e), f"selfcheck: {what} is refused", str(e))

    p = vp.parse(GOOD, "t")
    ok &= ck(p["filter"] == {"r_agl": "40"} and not p["aiding"]["zupt"] and len(p["sensors"]) == 4,
             "selfcheck: a good profile reads (override, aiding off, 4 sensors)")
    ok &= bad(GOOD + "aiding.zupt: true\n", "given twice", "a key given twice")
    ok &= bad(GOOD + "colour: red\n", "unknown key", "an unknown key")
    ok &= bad(GOOD.replace("filter.r_agl", "filter.r_algae"), "not a key of", "a filter override of no key")
    ok &= bad(GOOD.replace("aiding.mag: true\n", ""), "missing aiding.mag", "a missing aiding")
    ok &= bad(GOOD.replace("kind: multirotor", "kind: blimp"), "kind must be", "an unknown vehicle kind")
    ok &= bad(GOOD.replace("mount_xyz_m: 0 0 -0.06", "mount_xyz_m: 0 -0.06"), "three numbers", "a mount of two numbers")
    ok &= bad(GOOD.replace("sensor.mag.gdn_input: mag", "sensor.mag.gdn_input: baro"), "more than one sensor",
              "two sensors feeding one input")
    ok &= bad(GOOD.replace("sensor.radar.gdn_input: agl", "sensor.radar.gdn_input: none"), "aiding.agl is true",
              "agl aiding with no altimeter")
    ok &= bad(GOOD.replace("sensor.baro.mount_source: placeholder", "sensor.baro.mount_source: guessed"),
              "mount_source", "an unknown mount source")
    ok &= bad(GOOD.replace("sensor.baro.mount_source: placeholder\n", ""), "missing mount_source",
              "a sensor field missing")
    # Fingerprint: comments, order and spacing don't count; a value does.
    lines = GOOD.splitlines()
    shuffled = "# a comment\n" + "\n".join(reversed(lines)).replace(": ", ":   ") + "\n"
    ok &= ck(vp.parse(shuffled, "t")["fingerprint"] == p["fingerprint"],
             "selfcheck: fingerprint ignores comments, order and spacing", p["fingerprint"])
    ok &= ck(vp.parse(GOOD.replace("filter.r_agl: 40", "filter.r_agl: 41"), "t")["fingerprint"] != p["fingerprint"],
             "selfcheck: fingerprint changes with a value")
    # Poses: SDF composition, rpy round trip, the optical frame, the tolerance.
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(200):
        a = rng.uniform(-math.pi, math.pi, 3)
        a[1] = rng.uniform(-1.5, 1.5)
        worst = max(worst, vp.angle_between(vp.rot(a), vp.rot(vp.rpy_of(vp.rot(a)))))
    ok &= ck(worst < 1e-9, "selfcheck: rpy -> rotation -> rpy round trip", f"worst {worst:.1e} deg")
    T = vp.pose([0, 0, -0.06], [0, 0, 0]) @ vp.pose([0.08, 0.0375, -0.02], [0, math.radians(30), 0])
    ok &= ck(np.allclose(T[:3, 3], [0.08, 0.0375, -0.08]), "selfcheck: a payload sensor through the payload mount")
    C = T @ vp.OPTICAL
    fwd = C[:3, :3] @ np.array([0, 0, 1.0])          # optical z: where the camera looks
    ok &= ck(np.allclose(fwd, [math.cos(math.radians(30)), 0, -math.sin(math.radians(30))]),
             "selfcheck: a camera pitched 30 deg down looks forward and 30 deg down (optical z)",
             " ".join(f"{x:+.3f}" for x in fwd))
    right = C[:3, :3] @ np.array([1.0, 0, 0])        # optical x: image right = the body's right (-y)
    ok &= ck(right[1] < -0.99, "selfcheck: the optical frame's x is the camera's right")
    off = vp.pose([0.0015, 0, 0], [0, 0, math.radians(0.1)])
    dm, da = mount_error(off, np.eye(4))
    ok &= ck(dm > TOL_M and da > TOL_DEG, "selfcheck: 1.5 mm and 0.1 deg off are caught",
             f"{dm * 1000:.1f} mm, {da:.2f} deg")
    print("SELFCHECK PASSED" if ok else "SELFCHECK FAILED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if (selfcheck() if "--selfcheck" in sys.argv else run()) else 1)
