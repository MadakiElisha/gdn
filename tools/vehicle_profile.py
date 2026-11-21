#!/usr/bin/env python3
"""Vehicle profiles (roadmap R2.5): reader and checks, the same rules as the node's
loader (src/gdn_fusion/include/gdn/vehicle_profile.hpp).

A profile is a flat "key: value" file in config/vehicles/<name>.yaml, as the filter
configuration is ('#' comments, a key given twice is an error, unknown keys are errors):

  schema, name, kind (multirotor | vtol | fixedwing), px4_airframe, sim_model
  filter_config                       the filter settings file (config/filter/...)
  filter.<key>: value                 optional overrides of that file
  aiding.agl / .baro / .mag / .zupt   true | false: which aiding GDN uses
  sensor.<id>.<field>                 one block per sensor:
      kind        imu | lidar | camera | range | radar_altimeter | baro | mag
      topic       what its driver publishes on
      frame_id    the frame of those messages
      mount_xyz_m, mount_rpy_deg
                  the pose of frame_id in the airframe's base_link (ROS convention,
                  x forward, y left, z up; roll, pitch, yaw about fixed x, y, z, as in
                  SDF and URDF): the extrinsics
      mount_source  placeholder | measured (R6.4 measures them)
      gdn_input   imu | agl | baro | mag | none: which GDN input it feeds
      noise       optional: the sensor's noise model file (config/imu/...)
      sim         optional: the Gazebo sensor it is in simulation (a payload sensor's
                  name, or autopilot:<PX4 model sensor>)
      note        optional, free text
  description, note                   optional, free text

    python3 tools/vehicle_profile.py config/vehicles/x500.yaml     # read, check, fingerprint
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = "1"
KINDS = ("multirotor", "vtol", "fixedwing")
SENSOR_KINDS = ("imu", "lidar", "camera", "range", "radar_altimeter", "baro", "mag")
GDN_INPUTS = ("imu", "agl", "baro", "mag", "none")
AIDING = ("agl", "baro", "mag", "zupt")
TOP_REQUIRED = ("schema", "name", "kind", "px4_airframe", "sim_model", "filter_config")
TOP_OPTIONAL = ("description", "note")
SENSOR_REQUIRED = ("kind", "topic", "frame_id", "mount_xyz_m", "mount_rpy_deg", "mount_source", "gdn_input")
SENSOR_OPTIONAL = ("noise", "sim", "note")


class ProfileError(ValueError):
    pass


def read_kv(text, label):
    """[(key, value, line)] from flat "key: value" text, as gdn/kv_file.hpp reads it."""
    out, seen = [], set()
    for ln, line in enumerate(text.splitlines(), 1):
        line = line.split("#", 1)[0]
        if not line.strip(" \t\r\"'"):
            continue
        if ":" not in line:
            raise ProfileError(f"{label}:{ln}: expected key: value")
        k, v = line.split(":", 1)
        k, v = k.strip(" \t\r\"'"), v.strip(" \t\r\"'")
        if k in seen:
            raise ProfileError(f"{label}:{ln}: key '{k}' given twice")
        seen.add(k)
        out.append((k, v, ln))
    return out


def filter_keys(path):
    """The keys a filter config file holds (they're all required there)."""
    return {k for k, _, _ in read_kv(Path(path).read_text(), str(path))} - {"schema", "name", "source"}


def parse_bool(v, where):
    if v not in ("true", "false"):
        raise ProfileError(f"{where}: must be true or false (is '{v}')")
    return v == "true"


def parse_vec3(v, where):
    parts = v.split()
    try:
        x = [float(p) for p in parts]
    except ValueError:
        x = []
    if len(x) != 3 or not all(math.isfinite(a) for a in x):
        raise ProfileError(f"{where}: three numbers expected (is '{v}')")
    return np.array(x)


def parse(text, label="profile", root=ROOT):
    """A checked profile as a dict. Raises ProfileError with file:line on any problem."""
    kv = read_kv(text, label)
    line = {k: ln for k, _, ln in kv}
    top, filt, aid, sensors = {}, {}, {}, {}
    for k, v, ln in kv:
        where = f"{label}:{ln}: {k}"
        if k in TOP_REQUIRED or k in TOP_OPTIONAL:
            top[k] = v
        elif k.startswith("filter."):
            filt[k[len("filter."):]] = v
        elif k.startswith("aiding."):
            a = k[len("aiding."):]
            if a not in AIDING:
                raise ProfileError(f"{where}: unknown aiding (known: {', '.join(AIDING)})")
            aid[a] = parse_bool(v, where)
        elif k.startswith("sensor."):
            parts = k.split(".")
            if len(parts) != 3 or not parts[1]:
                raise ProfileError(f"{where}: expected sensor.<id>.<field>")
            _, sid, fld = parts
            if fld not in SENSOR_REQUIRED and fld not in SENSOR_OPTIONAL:
                raise ProfileError(f"{where}: unknown sensor field '{fld}'")
            sensors.setdefault(sid, {})[fld] = (v, ln)
        else:
            raise ProfileError(f"{where}: unknown key")
    for k in TOP_REQUIRED:
        if k not in top:
            raise ProfileError(f"{label}: missing {k}")
    if top["schema"] != SCHEMA:
        raise ProfileError(f"{label}:{line['schema']}: schema {top['schema']}, this reads schema {SCHEMA}")
    if top["kind"] not in KINDS:
        raise ProfileError(f"{label}:{line['kind']}: kind must be one of {', '.join(KINDS)}")
    for a in AIDING:
        if a not in aid:
            raise ProfileError(f"{label}: missing aiding.{a}")
    fc = root / top["filter_config"]
    if not fc.exists():
        raise ProfileError(f"{label}:{line['filter_config']}: no such filter config {top['filter_config']}")
    known = filter_keys(fc)
    for k in filt:
        if k not in known:
            raise ProfileError(f"{label}:{line['filter.' + k]}: filter.{k}: not a key of {top['filter_config']}")
    if not sensors:
        raise ProfileError(f"{label}: no sensors")
    out_s = {}
    for sid, f in sensors.items():
        for fld in SENSOR_REQUIRED:
            if fld not in f:
                raise ProfileError(f"{label}: sensor.{sid}: missing {fld}")
        s = {fld: v for fld, (v, _) in f.items()}
        w = {fld: f"{label}:{ln}: sensor.{sid}.{fld}" for fld, (_, ln) in f.items()}
        if s["kind"] not in SENSOR_KINDS:
            raise ProfileError(f"{w['kind']}: kind must be one of {', '.join(SENSOR_KINDS)}")
        if s["gdn_input"] not in GDN_INPUTS:
            raise ProfileError(f"{w['gdn_input']}: gdn_input must be one of {', '.join(GDN_INPUTS)}")
        if s["mount_source"] not in ("placeholder", "measured"):
            raise ProfileError(f"{w['mount_source']}: mount_source must be placeholder or measured")
        s["xyz"] = parse_vec3(s["mount_xyz_m"], w["mount_xyz_m"])
        s["rpy_deg"] = parse_vec3(s["mount_rpy_deg"], w["mount_rpy_deg"])
        if "noise" in s and not (root / s["noise"]).exists():
            raise ProfileError(f"{w['noise']}: no such file {s['noise']}")
        out_s[sid] = s
    # Each GDN input from exactly one sensor; an aiding switched on needs its sensor.
    for inp in ("imu", "agl", "baro", "mag"):
        n = [sid for sid, s in out_s.items() if s["gdn_input"] == inp]
        if len(n) > 1:
            raise ProfileError(f"{label}: gdn_input {inp} from more than one sensor ({', '.join(n)})")
        if not n and (inp == "imu" or aid.get(inp)):
            raise ProfileError(f"{label}: no sensor has gdn_input {inp}" +
                               ("" if inp == "imu" else f", but aiding.{inp} is true"))
    return {"top": top, "filter": filt, "aiding": aid, "sensors": out_s, "label": label,
            "fingerprint": fingerprint(kv)}


def load(path, root=ROOT):
    p = Path(path)
    if not p.is_absolute():
        p = root / p
    return parse(p.read_text(), str(Path(path)), root)


def fingerprint(kv):
    """64-bit FNV-1a of 'key=value' lines sorted by key, as 12 hex digits (comments,
    order and spacing don't count; any value does). Same as gdn::ProfileFingerprint."""
    h = 1469598103934665603
    for k, v in sorted((k, v) for k, v, _ in kv):
        for ch in f"{k}={v}\n".encode():
            h ^= ch
            h = (h * 1099511628211) & 0xFFFFFFFFFFFFFFFF
    return f"{h >> 16:012x}"


# --- poses: SDF/URDF convention, R = Rz(yaw) Ry(pitch) Rx(roll) ---
def rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def rpy_of(R):
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        return np.array([math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])])
    return np.array([math.atan2(-R[1, 2], R[1, 1]), p, 0.0])     # gimbal lock: yaw folded into roll


def pose(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rot(rpy)
    T[:3, 3] = xyz
    return T


def angle_between(Ra, Rb):
    """Angle of the rotation taking Ra to Rb, degrees. atan2 of its sine and cosine:
    acos of the cosine alone loses precision near zero (1e-6 deg for an exact match)."""
    D = Ra.T @ Rb
    c = (np.trace(D) - 1.0) / 2.0
    s = 0.5 * np.linalg.norm([D[2, 1] - D[1, 2], D[0, 2] - D[2, 0], D[1, 0] - D[0, 1]])
    return math.degrees(math.atan2(s, c))


# A camera's optical frame (x right, y down, z forward) in its Gazebo/ROS camera frame
# (x forward): roll -90, yaw -90 degrees.
OPTICAL = pose([0, 0, 0], [-math.pi / 2, 0.0, -math.pi / 2])


def main():
    if len(sys.argv) != 2 or sys.argv[1] in ("-h", "--help"):
        sys.exit(__doc__)
    try:
        p = load(sys.argv[1])
    except (ProfileError, OSError) as e:
        sys.exit(f"ERROR: {e}")
    t = p["top"]
    print(f"{t['name']} ({t['kind']}, PX4 {t['px4_airframe']}, model {t['sim_model']}), fingerprint {p['fingerprint']}")
    print(f"  filter: {t['filter_config']}" + (f", overrides {p['filter']}" if p["filter"] else ""))
    print("  aiding: " + ", ".join(f"{a} {'on' if p['aiding'][a] else 'off'}" for a in AIDING))
    for sid, s in p["sensors"].items():
        print(f"  {sid:15s} {s['kind']:15s} {s['topic']:28s} {s['frame_id']:32s} "
              f"xyz {' '.join(f'{x:+.3f}' for x in s['xyz'])} m, rpy {' '.join(f'{x:+.1f}' for x in s['rpy_deg'])} deg"
              f"  [{s['mount_source']}, GDN {s['gdn_input']}]")


if __name__ == "__main__":
    main()
